// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// ROS message <-> logic struct conversions for StateEstimationNode, and the
// health rules behind /diagnostics (spec §5). Uses message types only (no
// node, no spinning), so it is unit tested directly.

#ifndef STATE_ESTIMATION__MESSAGE_CONVERSIONS_HPP_
#define STATE_ESTIMATION__MESSAGE_CONVERSIONS_HPP_

#include <Eigen/Geometry>

#include <cstdint>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "state_estimation/robot_state.hpp"
#include "state_estimation/state_estimation_logic.hpp"

namespace state_estimation
{

/// Frame names (Architecture Conventions §2; part of the interface).
constexpr char kOdomFrame[] = "odom";
constexpr char kBaseFrame[] = "base_footprint";

/// Variance reported for /odom fields a 2D estimate does not estimate
/// (z, roll, pitch and their rates, lateral speed), as in ICD-SNS-GNC.
constexpr double kUnestimatedVariance = 1.0e6;

/// IMU message to a measurement in base_footprint.
///
/// @param base_from_imu rotation of the IMU mount (TF base_footprint ->
///        imu frame), applied to rates, specific force and covariances.
/// @param forward_offset IMU position ahead of base_footprint (TF
///        translation x), for the centripetal lever-arm correction.
/// A covariance marked not-provided (-1) gives a NaN variance, so the logic
/// rejects the sample. Pitch is set only when the orientation is provided.
ImuMeasurement to_imu_measurement(
  const sensor_msgs::msg::Imu & msg, const Eigen::Quaterniond & base_from_imu,
  double forward_offset = 0.0);

/// Wheel odometry to forward speed: only twist.linear.x (ICD-SNS-GNC §4).
WheelSpeedMeasurement to_wheel_speed(const nav_msgs::msg::Odometry & msg);

/// Estimate to /odom (odom -> base_footprint, full covariance).
nav_msgs::msg::Odometry to_odometry(
  const RobotState & state, const builtin_interfaces::msg::Time & stamp);

/// Estimate to the TF odom -> base_footprint.
geometry_msgs::msg::TransformStamped to_transform(
  const RobotState & state, const builtin_interfaces::msg::Time & stamp);

/// What the health rules need beyond the logic's status.
struct HealthInputs
{
  StateEstimationStatus status;
  bool active{false};              ///< lifecycle state is active
  bool imu_mount_known{true};      ///< TF to the IMU frame was found
  bool new_numerical_reset{false};  ///< since the previous report
  bool new_rejected_sample{false};  ///< since the previous report
};

struct HealthReport
{
  std::uint8_t level{0};  ///< diagnostic_msgs/DiagnosticStatus level
  std::string message;
};

/// Health rules (spec §5, ICD-SNS-GNC §5). Worst condition wins, ranked
/// ERROR > STALE > WARN > OK, and every condition is listed in the message.
/// STALE until each input has arrived (or while inactive); ERROR on a stale
/// IMU, unknown IMU mount or numerical reset; WARN on stale wheel odometry,
/// slip or rejected samples; else OK.
HealthReport summarize_health(const HealthInputs & inputs);

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__MESSAGE_CONVERSIONS_HPP_
