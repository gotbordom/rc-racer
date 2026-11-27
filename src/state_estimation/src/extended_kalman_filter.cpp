// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include "state_estimation/extended_kalman_filter.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace state_estimation
{

void ExtendedKalmanFilter::predict(
  RobotState & state, const RobotFrame & /*frame*/, const PredictInputs & inputs, double dt) const
{
  // Rigid-body kinematics of base_footprint in the plane, with the IMU's
  // acceleration as input (spec §6.1): no tyre model. Whether the car grips
  // is the no-slip constraint's job (StateEstimationLogic), not this one's.
  // Midpoint heading over the step.
  const double yaw = state.mean(kYaw);
  const double vx = state.mean(kSpeed);
  const double vy = state.mean(kSideSpeed);
  const double w = state.mean(kYawRate);
  const double mid = yaw + 0.5 * w * dt;
  const double c = std::cos(mid);
  const double s = std::sin(mid);

  StateMatrix f = StateMatrix::Identity();
  f(kX, kYaw) = (-vx * s - vy * c) * dt;
  f(kX, kSpeed) = c * dt;
  f(kX, kSideSpeed) = -s * dt;
  f(kX, kYawRate) = (-vx * s - vy * c) * 0.5 * dt * dt;
  f(kY, kYaw) = (vx * c - vy * s) * dt;
  f(kY, kSpeed) = s * dt;
  f(kY, kSideSpeed) = c * dt;
  f(kY, kYawRate) = (vx * c - vy * s) * 0.5 * dt * dt;
  f(kYaw, kYawRate) = dt;
  f(kSpeed, kSideSpeed) = w * dt;
  f(kSpeed, kYawRate) = vy * dt;
  f(kSideSpeed, kSpeed) = -w * dt;
  f(kSideSpeed, kYawRate) = -vx * dt;

  StateMatrix q = StateMatrix::Zero();
  q(kSpeed, kSpeed) = inputs.accel_noise_density * dt;
  q(kSideSpeed, kSideSpeed) = inputs.lateral_accel_noise_density * dt;
  q(kYawRate, kYawRate) = inputs.yaw_accel_noise_density * dt;
  // Along the heading only: q * distance driven this step.
  const Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
  q.block<2, 2>(kX, kX) = inputs.distance_noise_density * std::abs(vx) * dt *
    heading * heading.transpose();

  state.mean(kX) += (vx * c - vy * s) * dt;
  state.mean(kY) += (vx * s + vy * c) * dt;
  state.mean(kYaw) = normalize_angle(yaw + w * dt);
  state.mean(kSpeed) += (inputs.forward_accel + w * vy) * dt;
  state.mean(kSideSpeed) += (inputs.lateral_accel - w * vx) * dt;

  state.covariance = f * state.covariance * f.transpose() + q;
  state.covariance = 0.5 * (state.covariance + state.covariance.transpose());
}

void ExtendedKalmanFilter::update(
  RobotState & state, StateIndex index, double measurement, double variance) const
{
  // Direct measurement of one element: H is a unit row vector.
  const double residual = measurement - state.mean(index);
  const double innovation_variance = state.covariance(index, index) + variance;
  const StateVector gain = state.covariance.col(index) / innovation_variance;

  state.mean += gain * residual;
  state.mean(kYaw) = normalize_angle(state.mean(kYaw));

  StateMatrix i_kh = StateMatrix::Identity();
  i_kh.col(index) -= gain;
  state.covariance = i_kh * state.covariance * i_kh.transpose() +
    gain * variance * gain.transpose();
  state.covariance = 0.5 * (state.covariance + state.covariance.transpose());
}

std::unique_ptr<SensorFusionMethod> make_fusion_method(const std::string & method)
{
  if (method == "ekf") {
    return std::make_unique<ExtendedKalmanFilter>();
  }
  if (method == "ukf" || method == "complementary") {
    throw std::invalid_argument("fusion.method '" + method + "' is planned but not implemented");
  }
  throw std::invalid_argument(
          "unknown fusion.method '" + method + "' (expected 'ekf', 'ukf' or 'complementary')");
}

}  // namespace state_estimation
