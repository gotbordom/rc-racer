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
  RobotState & state, const RobotFrame & frame, const PredictInputs & inputs, double dt) const
{
  const double accel = inputs.forward_accel;
  const VelocityInputs mid_step{state.mean(kSpeed) + 0.5 * accel * dt, state.mean(kYawRate)};

  const Pose2D next = frame.get_next_pose_estimate(state.pose(), mid_step, dt);
  const PoseJacobians j = frame.jacobians(state.pose(), mid_step, dt);

  StateMatrix f = StateMatrix::Identity();
  f.block<3, 3>(kX, kX) = j.wrt_pose;
  f.block<3, 2>(kX, kSpeed) = j.wrt_inputs;

  StateMatrix q = StateMatrix::Zero();
  q(kSpeed, kSpeed) = inputs.accel_noise_density * dt;
  q(kYawRate, kYawRate) = inputs.yaw_accel_noise_density * dt;
  // Along the heading only: q * distance driven this step.
  const Eigen::Vector2d heading(std::cos(state.mean(kYaw)), std::sin(state.mean(kYaw)));
  q.block<2, 2>(kX, kX) = inputs.distance_noise_density * std::abs(state.mean(kSpeed)) * dt *
    heading * heading.transpose();

  state.mean(kX) = next.x;
  state.mean(kY) = next.y;
  state.mean(kYaw) = next.yaw;
  state.mean(kSpeed) += accel * dt;

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
