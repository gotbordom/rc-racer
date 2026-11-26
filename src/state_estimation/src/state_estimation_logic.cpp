// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include "state_estimation/state_estimation_logic.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "state_estimation/pinned_hold.hpp"
#include "state_estimation/slip_checks.hpp"

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
  fusion_(make_fusion_method(config.fusion_method)),
  history_(config.slip_window),
  spread_(config.slip_correction_time)
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
  require_positive(config.slip_braking_threshold, "slip.braking_threshold");
  require_positive(config.slip_correction_time, "slip.correction_time");
  require_positive(config.slip_lateral_threshold, "slip.lateral_threshold");
  require_positive(config.slip_lateral_min_yaw_rate, "slip.lateral_min_yaw_rate");
  require_positive(config.no_slip_lateral_speed_stddev, "no_slip.lateral_speed_stddev");
  require_positive(
    config.no_slip_slide_lateral_speed_stddev, "no_slip.slide_lateral_speed_stddev");
  require_positive(config.slip_accel_span, "slip.accel_span");
  require_positive(config.slip_accel_margin, "slip.accel_margin");
  require_positive(config.slip_accel_gain, "slip.accel_gain");
  require_positive(config.slip_impact_accel, "slip.impact_accel");
  require_positive(config.slip_impact_lookback, "slip.impact_lookback");
  require_positive(config.slip_pinned_speed, "slip.pinned_speed");
  if (!(config.slip_pinned_fraction > 0.0 && config.slip_pinned_fraction < 1.0)) {
    throw std::invalid_argument("slip.pinned_fraction must be in (0, 1)");
  }
  require_positive(config.slip_pinned_max_duration, "slip.pinned_max_duration");
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
  state_.covariance(kSideSpeed, kSideSpeed) = config_.initial_speed_variance;
  last_good_ = state_;

  // Input ages describe the sensors, not the estimate, so they survive a
  // reset; the slip flag and counters start again.
  const double imu_age = status_.imu_age;
  const double wheel_age = status_.wheel_age;
  status_ = StateEstimationStatus{};
  status_.imu_age = imu_age;
  status_.wheel_age = wheel_age;
  update_staleness();
  history_.clear();
  agreeing_since_.reset();
  spread_.clear();
  hold_.reset();
  slide_in_slip_ = false;
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
  if (status_.imu_stale) {
    predict_lateral_ = 0.0;
  } else if (lateral_samples_ > 0) {
    predict_lateral_ = lateral_sum_ / lateral_samples_;
  }
  lateral_sum_ = 0.0;
  lateral_samples_ = 0;
  // Sideways: the IMU's lateral acceleration (at base_footprint; an IMU
  // offset forward adds lever x yaw acceleration, ignored: 0 on the car).
  // Without it, the no-slip value omega x v_x, so v_y doesn't change.
  if (lateral_available_ && !status_.imu_stale) {
    inputs.lateral_accel = predict_lateral_;
    inputs.lateral_accel_noise_density = config_.accel_noise_density +
      predict_accel_variance_ * dt;
  } else {
    inputs.lateral_accel = state_.mean(kYawRate) * state_.mean(kSpeed);
    inputs.lateral_accel_noise_density = config_.accel_noise_density_no_imu;
  }

  fusion_->predict(state_, *frame_, inputs, dt);
  // No-slip constraint (spec §6.1): the rear axle doesn't move sideways,
  // v_y = omega x rear_axle_offset, held at no_slip_lateral_speed_stddev. It
  // is a pseudo-measurement of v_y alone, at the current yaw rate: the gyro
  // measures omega directly, and a constraint on both would let an error in
  // the lateral acceleration leak into the yaw rate (seen in the node test:
  // 0.168 rad/s for a 0.2 rad/s gyro, with a lateral reading of 0).
  // In a slide (step 2, VAL-018) the rear axle does move sideways: the
  // constraint drops to a loose prior, so v_y follows the IMU's lateral
  // acceleration without drifting without bound. Not without lateral data,
  // and not for spin or braking slips, where the car goes straight.
  const bool sliding_now = status_.slip_detected && slide_in_slip_ && lateral_available_ &&
    !status_.imu_stale;
  const double no_slip_sigma = sliding_now ?
    config_.no_slip_slide_lateral_speed_stddev : config_.no_slip_lateral_speed_stddev;
  const bool regripped = status_.side_slip_free && !sliding_now;
  status_.side_slip_free = sliding_now;
  const Eigen::Vector2d before(state_.mean(kX), state_.mean(kY));
  fusion_->update(
    state_, kSideSpeed, state_.mean(kYawRate) * frame_->rear_axle_offset(),
    no_slip_sigma * no_slip_sigma);
  if (regripped) {
    // The constraint snapping tight again corrects v_y, and through its
    // correlation with the position (built while v_y drifted) moves the
    // position at once (0.15 m in VAL-004/018). Spread that, as at a slip's
    // end, so odom stays continuous.
    spread_since(before);
  }
  const Eigen::Vector2d share = spread_.step(dt);
  state_.mean(kX) += share.x();
  state_.mean(kY) += share.y();
  guard_numerics();

  // The forward speed's own rate of change, v_x' = a_x + omega x v_y, is
  // what the slip windows compare the wheel against.
  history_.advance(
    dt, inputs.forward_accel + state_.mean(kYawRate) * state_.mean(kSideSpeed), predict_lateral_,
    last_wheel_speed_ * last_yaw_rate_, state_.mean(kSpeed), state_.covariance(kSpeed, kSpeed),
    last_yaw_rate_, last_wheel_speed_);
}

std::optional<ReferenceSpeed> StateEstimationLogic::imu_reference_speed(double age) const
{
  // Needs fresh IMU over the whole span: without it the acceleration input is
  // 0 and the reference would call every real speed change slip.
  if (status_.imu_stale) {
    return std::nullopt;
  }
  return history_.reference(age);
}

void StateEstimationLogic::spread_since(const Eigen::Vector2d & before)
{
  const Eigen::Vector2d moved = Eigen::Vector2d(state_.mean(kX), state_.mean(kY)) - before;
  state_.mean(kX) = before.x();
  state_.mean(kY) = before.y();
  spread_.add(moved);
}

double StateEstimationLogic::restart_from_impact()
{
  double speed_before = state_.mean(kSpeed);
  // An impact spread over two cycles: the first one's drop was under the
  // slip threshold, so that cycle's wheel update pulled the speed back up
  // and part of the impact was lost (VAL-007, 2026-11-24: stranded at
  // 0.38 m/s, too fast to count as pinned). Take the speed from before the
  // impact plus the IMU since instead: the wheel's pull is undone.
  if (const auto before = imu_reference_speed(config_.slip_impact_lookback)) {
    speed_before = history_.at(before->index).speed;
    state_.mean(kSpeed) = before->speed;
  }
  return speed_before;
}

double StateEstimationLogic::correct_window_retroactively(const ReferenceSpeed & from)
{
  const SpeedSample start = history_.at(from.index);
  // Distance the wheel added over the IMU path
  const double correction = history_.rewrite_as_imu_path(from.index);
  const double yaw = state_.mean(kYaw);
  const Eigen::Vector2d along(std::cos(yaw), std::sin(yaw));
  // Applied over slip_correction_time, not at once: odom must stay
  // continuous (REP-105; control runs on it). The covariance grows now, so
  // the estimate is honest while the position catches up.
  spread_.add(-correction * along);
  // The IMU path is only as good as the speed it starts from, over the
  // window; and the window start may itself have been partly off.
  const double span = history_.elapsed() - start.time;
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
    !std::isfinite(imu.lateral_specific_force) || !(imu.lateral_specific_force_variance >= 0.0) ||
    (imu.roll && !std::isfinite(*imu.roll)) ||
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
  // At base_footprint, the point the state describes: an IMU ahead of it
  // feels -omega^2 x its offset forward, which is not acceleration there.
  // (The rear axle's no-sideslip effect is the state's omega x v_y now.)
  forward_accel_ = imu.forward_specific_force + kGravity * std::sin(pitch) +
    imu.yaw_rate * imu.yaw_rate * imu.forward_offset;
  // The understeer check compares the IMU's own lateral reading with the
  // no-slip kinematics at its mount: its lever is from the rear axle.
  const double lever = frame_->rear_axle_offset() + imu.forward_offset;
  const double gravity_gain = kGravity * std::cos(pitch);  // d(accel) / d(pitch)
  forward_accel_variance_ = imu.forward_specific_force_variance +
    (imu.pitch ? gravity_gain * gravity_gain * imu.pitch_variance : 0.0);
  accel_sum_ += forward_accel_;
  accel_variance_sum_ += forward_accel_variance_;
  ++accel_samples_;
  // Lateral: gravity removed using roll (and pitch) like forward.
  const double roll = imu.roll.value_or(0.0);
  lateral_sum_ += imu.lateral_specific_force - kGravity * std::sin(roll) * std::cos(pitch);
  ++lateral_samples_;
  imu_lever_ = lever;
  lateral_available_ = imu.lateral_specific_force_variance > 0.0;

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
  // Slip that builds up over several steps, and stale history after an
  // earlier slip too small to catch (slip_checks.hpp).
  const std::optional<ReferenceSpeed> reference = imu_reference_speed(config_.slip_window);
  const WindowCheck window = window_check(
    history_, reference, history_.reference(0.5 * config_.slip_window), wheel.speed,
    {config_.slip_threshold, config_.slip_braking_accel, config_.slip_braking_threshold});
  const bool window_slip = window.slip();
  if (window.stale() && !status_.slip_detected) {
    history_.rebase_before(history_.size() - 1);
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

  // Understeer: a slide the yaw-rate check can't see (slip_checks.hpp).
  const bool understeer = reference && lateral_available_ &&
    state_estimation::understeer(
    history_, *reference, last_yaw_rate_, imu_lever_,
    {config_.slip_lateral_threshold, config_.slip_lateral_min_yaw_rate});
  status_.understeer_detected = understeer;
  if (understeer || sliding) {
    slide_in_slip_ = true;  // reset when the slip ends
  }
  bool slipping = wheel_slip || sliding || understeer;

  // Impact (slip_checks.hpp).
  bool impact = false;
  if (const auto back = imu_reference_speed(config_.slip_impact_lookback)) {
    if (const auto when = state_estimation::impact(history_, *back, config_.slip_impact_accel)) {
      impact = true;
      impact_time_ = *when;
    }
  }

  if (status_.slip_detected) {
    // Hysteresis: while slipping the wheel is not fused, so the speed is
    // IMU-only. The wheel is trusted again only after agreeing with it
    // (half the threshold plus 3 sigma) for half a slip window without a
    // break, so a wheel still spinning down cannot end the slip on its way.
    const bool agrees =
      std::abs(residual) < 0.5 * config_.slip_threshold + 3.0 * speed_sigma && !sliding &&
      !understeer;
    if (!agrees) {
      agreeing_since_.reset();
    } else if (!agreeing_since_) {
      agreeing_since_ = history_.elapsed();
    }
    const bool agreed = agreeing_since_ &&
      history_.elapsed() - *agreeing_since_ >= 0.5 * config_.slip_window;
    // Safety nets, so the filter never stays on the IMU alone: the car is
    // clearly stopped (wheel ~0, IMU quiet), or the IMU-only stretch has
    // lasted too long. Then trust the wheel again from now on: re-anchor the
    // speed to it and forget the IMU-only history, or the windowed check
    // would fire again on the stale reference.
    const bool stationary = std::abs(wheel.speed) < config_.stationary_speed &&
      !status_.imu_stale && std::abs(last_yaw_rate_) < config_.stationary_yaw_rate &&
      std::abs(forward_accel_) < config_.stationary_accel;
    // Pinned against an obstacle after an impact (pinned_hold.hpp).
    if (impact && !hold_.after_impact()) {
      hold_.impact(impact_time_, restart_from_impact());
    }
    const bool imu_quiet = !status_.imu_stale &&
      std::abs(last_yaw_rate_) < config_.stationary_yaw_rate &&
      std::abs(forward_accel_) < config_.stationary_accel;
    const PinnedAction hold = hold_.update(
      history_, state_.mean(kSpeed), imu_quiet,
      {config_.slip_pinned_fraction, config_.slip_pinned_speed, config_.slip_pinned_max_duration});
    if (hold.speed) {
      state_.mean(kSpeed) = *hold.speed;
    }
    if (hold.take_back) {
      const double yaw = state_.mean(kYaw);
      spread_.add(-*hold.take_back * Eigen::Vector2d(std::cos(yaw), std::sin(yaw)));
    }
    if (hold.pinned) {
      slip_since_ = history_.elapsed();  // max_duration counts from leaving pinned
    }
    const bool pinned = hold.pinned;
    status_.pinned = pinned;
    const bool too_long = !pinned && history_.elapsed() - slip_since_ >= config_.slip_max_duration;
    if (stationary || too_long) {
      state_.mean(kSpeed) = wheel.speed;
      history_.clear();
    }
    slipping = !(agreed || stationary || too_long);
  } else if (slipping) {
    ++status_.slip_events;
    slip_since_ = history_.elapsed();
    agreeing_since_.reset();
    hold_.reset();
    if (impact) {
      hold_.impact(impact_time_, restart_from_impact());
    }
    if (window_slip) {
      // The estimate followed the slipping wheel over the window; the IMU
      // reference is the better speed. Start the IMU-only stretch from it,
      // and take back what the wheel added to the position meanwhile.
      state_.mean(kSpeed) = reference->speed;
      correct_window_retroactively(*reference);
    }
  }
  const bool slip_ended = status_.slip_detected && !slipping;
  status_.slip_detected = slipping;
  if (!slipping) {
    slide_in_slip_ = false;
    hold_.reset();
    status_.pinned = false;
  }

  // Not yet slip, but the wheel out-accelerating the car: trust it less
  // in proportion (slip_checks.hpp).
  double early_variance = 0.0;
  if (const auto span = imu_reference_speed(config_.slip_accel_span)) {
    early_variance = early_distrust_variance(
      history_, *span, wheel.speed, {config_.slip_accel_margin, config_.slip_accel_gain});
  }
  status_.wheel_variance_used = status_.slip_detected ?
    wheel.variance * config_.slip_variance_scale : wheel.variance + early_variance;
  const Eigen::Vector2d before(state_.mean(kX), state_.mean(kY));
  fusion_->update(state_, kSpeed, wheel.speed, status_.wheel_variance_used);
  if (slip_ended) {
    // The first trusted wheel sample after an IMU-only stretch also moves
    // the position, through its correlation with the speed: right, but at
    // once (0.16 m in one step in VAL-004). Spread that part like the other
    // corrections, so odom stays continuous; the speed is corrected now.
    spread_since(before);
  }
  guard_numerics();
  last_wheel_speed_ = wheel.speed;
  // After the update
  history_.update_latest(state_.mean(kSpeed), state_.covariance(kSpeed, kSpeed));
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
