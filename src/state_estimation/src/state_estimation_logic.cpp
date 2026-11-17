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
  require_positive(config.slip_threshold, "slip.threshold");
  require_positive(config.slip_yaw_rate_margin, "slip.yaw_rate_margin");
  require_positive(config.initial_pose_variance, "initial_covariance.pose");
  require_positive(config.initial_speed_variance, "initial_covariance.speed");
  require_positive(config.initial_yaw_rate_variance, "initial_covariance.yaw_rate");
  if (!(config.slip_variance_scale >= 1.0) || !std::isfinite(config.slip_variance_scale)) {
    throw std::invalid_argument("slip.variance_scale must be >= 1");
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
  if (status_.imu_stale) {
    // Do not keep applying an old acceleration.
    inputs.forward_accel = 0.0;
    inputs.accel_noise_density = config_.accel_noise_density_no_imu;
  } else {
    inputs.forward_accel = forward_accel_;
    // Per-sample variance sigma^2 held for dt gives sigma^2 * dt^2 on v,
    // i.e. a density of sigma^2 * dt.
    inputs.accel_noise_density = config_.accel_noise_density + forward_accel_variance_ * dt;
  }

  fusion_->predict(state_, *frame_, inputs, dt);
  guard_numerics();
}

void StateEstimationLogic::update_imu(const ImuMeasurement & imu)
{
  const bool pitch_ok = !imu.pitch || std::isfinite(*imu.pitch);
  if (!valid_sample(imu.yaw_rate, imu.yaw_rate_variance) ||
    !valid_sample(imu.forward_specific_force, imu.forward_specific_force_variance) || !pitch_ok)
  {
    ++status_.rejected_measurements;
    return;
  }
  status_.imu_age = 0.0;
  update_staleness();

  // Specific force on a body pitched nose-down by theta at rest reads
  // -g * sin(theta) on x; add it back to get the real acceleration.
  forward_accel_ = imu.forward_specific_force + kGravity * std::sin(imu.pitch.value_or(0.0));
  forward_accel_variance_ = imu.forward_specific_force_variance;

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
  const bool wheel_slip = std::abs(residual) > config_.slip_threshold + 3.0 * speed_sigma;

  // Check the yaw rate against the best speed available: the wheels unless
  // they are slipping.
  const VelocityInputs velocity{
    wheel_slip ? state_.mean(kSpeed) : wheel.speed, state_.mean(kYawRate)};
  const double feasible_yaw_rate = frame_->constrain(velocity).angular;
  const bool sliding =
    std::abs(velocity.angular - feasible_yaw_rate) > config_.slip_yaw_rate_margin;
  status_.slip_detected = wheel_slip || sliding;

  status_.wheel_variance_used =
    status_.slip_detected ? wheel.variance * config_.slip_variance_scale : wheel.variance;
  fusion_->update(state_, kSpeed, wheel.speed, status_.wheel_variance_used);
  guard_numerics();
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
