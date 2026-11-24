// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Pluggable fusion method for StateEstimationLogic (spec §6,
// `fusion.method`). Pure C++, no ROS.

#ifndef STATE_ESTIMATION__SENSOR_FUSION_METHOD_HPP_
#define STATE_ESTIMATION__SENSOR_FUSION_METHOD_HPP_

#include <memory>
#include <string>

#include "state_estimation/robot_frame.hpp"
#include "state_estimation/robot_state.hpp"

namespace state_estimation
{

/// Inputs to one predict step.
struct PredictInputs
{
  /// Body-frame acceleration of base_footprint, gravity removed, m/s^2; 0
  /// when unknown. The state's speeds change as v_x' = a_x + omega v_y and
  /// v_y' = a_y - omega v_x.
  double forward_accel{0.0};
  double lateral_accel{0.0};
  /// Variance of lateral_accel plus unmodelled lateral acceleration,
  /// (m/s^2)^2 per second of prediction.
  double lateral_accel_noise_density{0.0};
  /// Variance of forward_accel plus unmodelled acceleration, (m/s^2)^2 per
  /// second of prediction (continuous-time spectral density).
  double accel_noise_density{0.0};
  /// Unmodelled change in yaw rate, (rad/s^2)^2 per second of prediction.
  double yaw_accel_noise_density{0.0};
  /// Along-track position noise per metre driven, m^2/m (wheel scale error
  /// that varies, e.g. tyre scrub in turns).
  double distance_noise_density{0.0};
};

class SensorFusionMethod
{
public:
  virtual ~SensorFusionMethod() = default;

  /// Name used for the `fusion.method` parameter.
  virtual std::string name() const = 0;

  /// Propagate the state dt seconds through the motion model.
  virtual void predict(
    RobotState & state, const RobotFrame & frame, const PredictInputs & inputs,
    double dt) const = 0;

  /// Fuse a direct, scalar measurement of one state element.
  virtual void update(
    RobotState & state, StateIndex index, double measurement, double variance) const = 0;
};

/// Build the fusion method named by `fusion.method`. Only "ekf" exists so
/// far; "ukf" and "complementary" are planned (spec §3.4).
/// @throws std::invalid_argument for an unknown or not-yet-implemented name.
std::unique_ptr<SensorFusionMethod> make_fusion_method(const std::string & method);

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__SENSOR_FUSION_METHOD_HPP_
