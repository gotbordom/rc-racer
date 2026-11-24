// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include "state_estimation/state_estimation_logic.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace state_estimation
{

namespace
{

constexpr double kGravity = 9.80665;  // m/s^2

void require_positive(double value, const std::string & name)
{
  if (!(value > 0.0) || !std::isfinite(value)) {
    throw std::invalid_argument(name + " must be positive and finite");
  }
}

bool valid_sample(double value, double variance)
{
  return std::isfinite(value) && std::isfinite(variance) && variance > 0.0;
}

}  // namespace

StateEstimationLogic::StateEstimationLogic(const StateEstimationConfig & config)
: config_(config),
  frame_(make_robot_frame(config.motion_model, config.robot)),
  fusion_(make_fusion_method(config.fusion_method))
{
  require_positive(config.imu_timeout, "inputs.imu.timeout");
  require_positive(config.wheel_odom_timeout, "inputs.wheel_odom.timeout");
  require_positive(config.accel_noise_density, "process_noise.accel");
  require_positive(config.accel_noise_density_no_imu, "process_noise.accel_no_imu");
  require_positive(config.yaw_accel_noise_density, "process_noise.yaw_accel");
  require_positive(config.distance_noise_density, "process_noise.distance");
  if (!(config.wheel_scale_stddev >= 0.0) || !std::isfinite(config.wheel_scale_stddev)) {
    throw std::invalid_argument("inputs.wheel_odom.scale_stddev must be >= 0");
  }
  require_positive(config.slip_threshold, "slip.threshold");
  require_positive(config.slip_yaw_rate_margin, "slip.yaw_rate_margin");
  require_positive(config.slip_window, "slip.window");
  require_positive(config.slip_max_duration, "slip.max_duration");
  require_positive(config.stationary_speed, "stationary.speed");
  require_positive(config.stationary_yaw_rate, "stationary.yaw_rate");
  require_positive(config.stationary_accel, "stationary.accel");
  require_positive(config.initial_pose_variance, "initial_covariance.pose");
  require_positive(config.initial_speed_variance, "initial_covariance.speed");
  require_positive(config.initial_yaw_rate_variance, "initial_covariance.yaw_rate");
  if (!(config.slip_variance_scale >= 1.0) || !std::isfinite(config.slip_variance_scale)) {
    throw std::invalid_argument("slip.variance_scale must be >= 1");
  }
  require_positive(config.slip_braking_accel, "slip.braking_accel");
  require_positive(config.slip_correction_time, "slip.correction_time");
  if (!(config.slip_correction_uncertainty >= 0.0) ||
    !std::isfinite(config.slip_correction_uncertainty))
  {
    throw std::invalid_argument("slip.correction_uncertainty must be >= 0");
  }
  if (!(config.recovery_covariance_scale >= 1.0) ||
    !std::isfinite(config.recovery_covariance_scale))
  {
    throw std::invalid_argument("recovery_covariance_scale must be >= 1");
  }
  reset();
}

void StateEstimationLogic::reset()
{
  state_ = RobotState{};
  state_.covariance.setZero();
  state_.covariance(kX, kX) = config_.initial_pose_variance;
  state_.covariance(kY, kY) = config_.initial_pose_variance;
  state_.covariance(kYaw, kYaw) = config_.initial_pose_variance;
  state_.covariance(kSpeed, kSpeed) = config_.initial_speed_variance;
  state_.covariance(kYawRate, kYawRate) = config_.initial_yaw_rate_variance;
  last_good_ = state_;

  // Input ages describe the sensors, not the estimate, so they survive a
  // reset; the slip flag and counters start again.
  const double imu_age = status_.imu_age;
  const double wheel_age = status_.wheel_age;
  status_ = StateEstimationStatus{};
  status_.imu_age = imu_age;
  status_.wheel_age = wheel_age;
  update_staleness();
  speed_history_.clear();
  agreeing_since_.reset();
  pending_correction_.setZero();
  correction_time_left_ = 0.0;
}

void StateEstimationLogic::predict(double dt)
{
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument("dt must be finite and non-negative");
  }
  status_.imu_age += dt;
  status_.wheel_age += dt;
  update_staleness();

  PredictInputs inputs;
  inputs.yaw_accel_noise_density = config_.yaw_accel_noise_density;
  inputs.distance_noise_density = config_.distance_noise_density;
  if (status_.imu_stale) {
    // Do not keep applying an old acceleration.
    inputs.forward_accel = 0.0;
    inputs.accel_noise_density = config_.accel_noise_density_no_imu;
  } else {
    // The average of the IMU samples since the last predict (the last one's
    // if none arrived), so none is dropped.
    if (accel_samples_ > 0) {
      predict_accel_ = accel_sum_ / accel_samples_;
      predict_accel_variance_ = accel_variance_sum_ / accel_samples_;
    }
    inputs.forward_accel = predict_accel_;
    // Per-sample variance sigma^2 held for dt gives sigma^2 * dt^2 on v,
    // i.e. a density of sigma^2 * dt.
    inputs.accel_noise_density = config_.accel_noise_density + predict_accel_variance_ * dt;
  }
  accel_sum_ = 0.0;
  accel_variance_sum_ = 0.0;
  accel_samples_ = 0;

  fusion_->predict(state_, *frame_, inputs, dt);
  apply_pending_correction(dt);
  guard_numerics();

  elapsed_ += dt;
  accel_integral_ += inputs.forward_accel * dt;
  speed_history_.push_back(
    {elapsed_, state_.mean(kSpeed), state_.covariance(kSpeed, kSpeed), accel_integral_});
  // Keep the newest sample that is at least slip_window old at the front.
  while (speed_history_.size() > 1 && speed_history_[1].time <= elapsed_ - config_.slip_window) {
    speed_history_.pop_front();
  }
}

std::optional<StateEstimationLogic::ReferenceSpeed>
StateEstimationLogic::imu_reference_speed(double age) const
{
  // Needs fresh IMU over the whole span: without it the acceleration input is
  // 0 and the reference would call every real speed change slip.
  if (speed_history_.empty() || status_.imu_stale ||
    elapsed_ - speed_history_.front().time < age)
  {
    return std::nullopt;
  }
  // The newest sample at least `age` old (the front is at least a window old).
  std::size_t i = 0;
  while (i + 1 < speed_history_.size() && speed_history_[i + 1].time <= elapsed_ - age) {
    ++i;
  }
  const SpeedSample & then = speed_history_[i];
  return ReferenceSpeed{
    then.speed + (accel_integral_ - then.accel_integral), then.speed_variance, i};
}

void StateEstimationLogic::apply_pending_correction(double dt)
{
  if (correction_time_left_ <= 0.0) {
    return;
  }
  const double share = std::min(1.0, dt / correction_time_left_);
  const Eigen::Vector2d step = share * pending_correction_;
  state_.mean(kX) -= step.x();
  state_.mean(kY) -= step.y();
  pending_correction_ -= step;
  correction_time_left_ -= dt;
}

void StateEstimationLogic::rebase_history_before(std::size_t from)
{
  const SpeedSample anchor = speed_history_[from];
  for (std::size_t i = 0; i < from; ++i) {
    SpeedSample & s = speed_history_[i];
    s.speed = anchor.speed - (anchor.accel_integral - s.accel_integral);
    s.speed_variance = anchor.speed_variance;
  }
}

double StateEstimationLogic::correct_window_retroactively(const ReferenceSpeed & from)
{
  const SpeedSample start = speed_history_[from.index];
  double correction = 0.0;  // distance the wheel added over the IMU path
  for (std::size_t i = from.index + 1; i < speed_history_.size(); ++i) {
    SpeedSample & s = speed_history_[i];
    const double imu_path = start.speed + (s.accel_integral - start.accel_integral);
    correction += (s.speed - imu_path) * (s.time - speed_history_[i - 1].time);
    s.speed = imu_path;
    s.speed_variance = start.speed_variance;
  }
  const double yaw = state_.mean(kYaw);
  const Eigen::Vector2d along(std::cos(yaw), std::sin(yaw));
  // Applied over slip_correction_time, not at once: odom must stay
  // continuous (REP-105; control runs on it). The covariance grows now, so
  // the estimate is honest while the position catches up.
  pending_correction_ += correction * along;
  correction_time_left_ = config_.slip_correction_time;
  // The IMU path is only as good as the speed it starts from, over the
  // window; and the window start may itself have been partly off.
  const double span = elapsed_ - start.time;
  const double variance = start.speed_variance * span * span +
    std::pow(config_.slip_correction_uncertainty * correction, 2);
  state_.covariance.block<2, 2>(kX, kX) += variance * along * along.transpose();
  return correction;
}

void StateEstimationLogic::update_imu(const ImuMeasurement & imu)
{
  const bool offset_ok = std::isfinite(imu.forward_offset);
  const bool pitch_ok =
    !imu.pitch || (std::isfinite(*imu.pitch) && std::isfinite(imu.pitch_variance) &&
    imu.pitch_variance >= 0.0);
  if (!valid_sample(imu.yaw_rate, imu.yaw_rate_variance) ||
    !valid_sample(imu.forward_specific_force, imu.forward_specific_force_variance) || !pitch_ok ||
    !offset_ok)
  {
    ++status_.rejected_measurements;
    return;
  }
  status_.imu_age = 0.0;
  update_staleness();

  // Specific force on a body pitched nose-down by theta at rest reads
  // -g * sin(theta) on x; add it back to get the real acceleration.
  const double pitch = imu.pitch.value_or(0.0);
  // Lever arm from the point with no sideways motion (rear axle) to the IMU.
  const double lever = frame_->rear_axle_offset() + imu.forward_offset;
  forward_accel_ = imu.forward_specific_force + kGravity * std::sin(pitch) +
    imu.yaw_rate * imu.yaw_rate * lever;
  const double gravity_gain = kGravity * std::cos(pitch);  // d(accel) / d(pitch)
  forward_accel_variance_ = imu.forward_specific_force_variance +
    (imu.pitch ? gravity_gain * gravity_gain * imu.pitch_variance : 0.0);
  accel_sum_ += forward_accel_;
  accel_variance_sum_ += forward_accel_variance_;
  ++accel_samples_;

  last_yaw_rate_ = imu.yaw_rate;
  fusion_->update(state_, kYawRate, imu.yaw_rate, imu.yaw_rate_variance);
  guard_numerics();
}

void StateEstimationLogic::update_wheel_speed(const WheelSpeedMeasurement & wheel)
{
  if (!valid_sample(wheel.speed, wheel.variance)) {
    ++status_.rejected_measurements;
    return;
  }
  status_.wheel_age = 0.0;
  update_staleness();

  // REQ-GNC-005. Wheel slip: the wheels disagree with the speed predicted
  // from the IMU (spinning on launch, locking on braking). Slide: the IMU
  // yaw rate is more than the motion model allows at this speed. Either
  // way the wheel speed is not the car's speed until they agree again.
  const double residual = wheel.speed - state_.mean(kSpeed);
  // The 3-sigma term stops a filter that is still unsure of its speed (e.g.
  // started while moving) from calling a correct wheel reading slip.
  const double speed_sigma = std::sqrt(state_.covariance(kSpeed, kSpeed));
  const bool step_slip = std::abs(residual) > config_.slip_threshold + 3.0 * speed_sigma;
  // Slip that builds up over several steps: compare with the estimate from
  // slip_window ago carried forward by the IMU alone.
  // Its 3-sigma term matters at start-up: switched on while moving, the
  // reference speed is the uncertain initial guess.
  //
  // That reference trusts the estimate at the window start. After an
  // earlier slip too small to catch, it may not be: the estimate followed
  // the wheel, and the reference now blames a correct wheel (2026-11-24, L2
  // sim loop: a launch led the wheel 0.29 m/s ahead, undetected, and half a
  // second later the check "corrected" a right speed 0.33 m/s too high).
  // The sign tells them apart: a driven wheel can only fall behind the car
  // while braking (or locked, which is braking too). So a wheel behind the
  // reference counts as slip only with braking on the IMU over the recent
  // half window; without, the history is stale and is rebuilt from now.
  // A wheel ahead is not restricted: spin on launch, a pinned car and
  // understeer all put it ahead.
  const std::optional<ReferenceSpeed> reference = imu_reference_speed(config_.slip_window);
  const bool window_gap = reference &&
    std::abs(wheel.speed - reference->speed) >
    config_.slip_threshold + 3.0 * std::sqrt(reference->variance);
  bool consistent = true;
  if (window_gap) {
    const double direction = reference->speed >= 0.0 ? 1.0 : -1.0;
    const bool wheel_behind = (reference->speed - wheel.speed) * direction > 0.0;
    const std::optional<ReferenceSpeed> recent = imu_reference_speed(0.5 * config_.slip_window);
    const SpeedSample & mid = speed_history_[recent ? recent->index : reference->index];
    const double span = elapsed_ - mid.time;
    const double recent_accel = span > 0.0 ? (accel_integral_ - mid.accel_integral) / span : 0.0;
    const bool braking = recent_accel * direction < -config_.slip_braking_accel;
    consistent = !wheel_behind || braking;
  }
  const bool window_slip = window_gap && consistent;
  if (window_gap && !consistent && !status_.slip_detected) {
    rebase_history_before(speed_history_.size() - 1);
    ++status_.stale_history_rebases;
  }
  const bool wheel_slip = step_slip || window_slip;

  // Check the yaw rate against the best speed available: the wheels unless
  // they are slipping.
  const VelocityInputs velocity{
    wheel_slip ? state_.mean(kSpeed) : wheel.speed, state_.mean(kYawRate)};
  const double feasible_yaw_rate = frame_->constrain(velocity).angular;
  const bool sliding =
    std::abs(velocity.angular - feasible_yaw_rate) > config_.slip_yaw_rate_margin;
  bool slipping = wheel_slip || sliding;
  if (status_.slip_detected) {
    // Hysteresis: while slipping the wheel is not fused, so the speed is
    // IMU-only. The wheel is trusted again only after agreeing with it
    // (half the threshold plus 3 sigma) for half a slip window without a
    // break, so a wheel still spinning down cannot end the slip on its way.
    const bool agrees =
      std::abs(residual) < 0.5 * config_.slip_threshold + 3.0 * speed_sigma && !sliding;
    if (!agrees) {
      agreeing_since_.reset();
    } else if (!agreeing_since_) {
      agreeing_since_ = elapsed_;
    }
    const bool agreed = agreeing_since_ && elapsed_ - *agreeing_since_ >= 0.5 * config_.slip_window;
    // Safety nets, so the filter never stays on the IMU alone: the car is
    // clearly stopped (wheel ~0, IMU quiet), or the IMU-only stretch has
    // lasted too long. Then trust the wheel again from now on: re-anchor the
    // speed to it and forget the IMU-only history, or the windowed check
    // would fire again on the stale reference.
    const bool stationary = std::abs(wheel.speed) < config_.stationary_speed &&
      !status_.imu_stale && std::abs(last_yaw_rate_) < config_.stationary_yaw_rate &&
      std::abs(forward_accel_) < config_.stationary_accel;
    const bool too_long = elapsed_ - slip_since_ >= config_.slip_max_duration;
    if (stationary || too_long) {
      state_.mean(kSpeed) = wheel.speed;
      speed_history_.clear();
    }
    slipping = !(agreed || stationary || too_long);
  } else if (slipping) {
    ++status_.slip_events;
    slip_since_ = elapsed_;
    agreeing_since_.reset();
    if (window_slip) {
      // The estimate followed the slipping wheel over the window; the IMU
      // reference is the better speed. Start the IMU-only stretch from it,
      // and take back what the wheel added to the position meanwhile.
      state_.mean(kSpeed) = reference->speed;
      correct_window_retroactively(*reference);
    }
  }
  status_.slip_detected = slipping;

  status_.wheel_variance_used =
    status_.slip_detected ? wheel.variance * config_.slip_variance_scale : wheel.variance;
  fusion_->update(state_, kSpeed, wheel.speed, status_.wheel_variance_used);
  guard_numerics();
  if (!speed_history_.empty()) {  // after the update
    speed_history_.back().speed = state_.mean(kSpeed);
    speed_history_.back().speed_variance = state_.covariance(kSpeed, kSpeed);
  }
}

RobotState StateEstimationLogic::estimate() const
{
  RobotState out = state_;
  const double s2 = config_.wheel_scale_stddev * config_.wheel_scale_stddev;
  const Eigen::Vector2d p(state_.mean(kX), state_.mean(kY));
  out.covariance.block<2, 2>(kX, kX) += s2 * p * p.transpose();
  out.covariance(kSpeed, kSpeed) += s2 * state_.mean(kSpeed) * state_.mean(kSpeed);
  return out;
}

void StateEstimationLogic::guard_numerics()
{
  if (state_.is_finite()) {
    last_good_ = state_;
    return;
  }
  ++status_.numerical_resets;
  state_ = last_good_;
  state_.covariance *= config_.recovery_covariance_scale;
}

void StateEstimationLogic::update_staleness()
{
  status_.imu_stale = !(status_.imu_age <= config_.imu_timeout);
  status_.wheel_stale = !(status_.wheel_age <= config_.wheel_odom_timeout);
}

}  // namespace state_estimation
