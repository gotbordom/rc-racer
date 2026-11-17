// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#ifndef STATE_ESTIMATION__EXTENDED_KALMAN_FILTER_HPP_
#define STATE_ESTIMATION__EXTENDED_KALMAN_FILTER_HPP_

#include <string>

#include "state_estimation/sensor_fusion_method.hpp"

namespace state_estimation
{

/// EKF over [x, y, yaw, v, omega].
///
/// Predict: pose follows the motion model at the mid-step speed,
/// v += a * dt (IMU acceleration as a control input), omega is constant.
/// Process noise enters through v and omega; pose uncertainty grows from
/// them through the motion model Jacobians. Update: Joseph form, which
/// keeps the covariance symmetric and positive semi-definite.
class ExtendedKalmanFilter : public SensorFusionMethod
{
public:
  std::string name() const override {return "ekf";}

  void predict(
    RobotState & state, const RobotFrame & frame, const PredictInputs & inputs,
    double dt) const override;

  void update(
    RobotState & state, StateIndex index, double measurement, double variance) const override;
};

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__EXTENDED_KALMAN_FILTER_HPP_
