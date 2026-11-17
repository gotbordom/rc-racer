// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include "state_estimation/message_conversions.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace state_estimation
{

namespace
{

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Indices into the 6x6 row-major pose / twist covariance of Odometry.
constexpr int kLinX = 0;
constexpr int kLinY = 1;
constexpr int kLinZ = 2;
constexpr int kRotX = 3;
constexpr int kRotY = 4;
constexpr int kRotZ = 5;

Eigen::Matrix3d to_matrix(const std::array<double, 9> & cov)
{
  return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cov.data());
}

// Variance of one base-frame axis of a vector measured in the IMU frame,
// or NaN when the sensor did not provide a covariance.
double rotated_variance(
  const std::array<double, 9> & cov, const Eigen::Matrix3d & r, int axis)
{
  if (cov[0] == -1.0) {
    return kNaN;
  }
  return (r * to_matrix(cov) * r.transpose())(axis, axis);
}

Eigen::Vector3d to_vector(const geometry_msgs::msg::Vector3 & v) {return {v.x, v.y, v.z};}

void set_quaternion_from_yaw(double yaw, geometry_msgs::msg::Quaternion & q)
{
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
}

// Severity for "worst wins": ERROR > STALE > WARN > OK. (The message
// constants put STALE numerically above ERROR, so they cannot be compared.)
int severity(std::uint8_t level)
{
  switch (level) {
    case DiagnosticStatus::ERROR: return 3;
    case DiagnosticStatus::STALE: return 2;
    case DiagnosticStatus::WARN: return 1;
    default: return 0;
  }
}

void append(HealthReport & report, std::uint8_t level, const std::string & message)
{
  if (severity(level) > severity(report.level)) {
    report.level = level;
  }
  report.message += report.message.empty() ? message : "; " + message;
}

}  // namespace

ImuMeasurement to_imu_measurement(
  const sensor_msgs::msg::Imu & msg, const Eigen::Quaterniond & base_from_imu,
  double forward_offset)
{
  const Eigen::Matrix3d r = base_from_imu.normalized().toRotationMatrix();

  ImuMeasurement imu;
  imu.forward_offset = forward_offset;
  imu.yaw_rate = (r * to_vector(msg.angular_velocity)).z();
  imu.yaw_rate_variance = rotated_variance(msg.angular_velocity_covariance, r, 2);
  imu.forward_specific_force = (r * to_vector(msg.linear_acceleration)).x();
  imu.forward_specific_force_variance = rotated_variance(msg.linear_acceleration_covariance, r, 0);

  const Eigen::Quaterniond q_ref_imu(
    msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if (msg.orientation_covariance[0] != -1.0 && q_ref_imu.norm() > 0.5) {
    // Orientation of base_footprint in the IMU's reference frame. Only
    // pitch is used (ZYX: pitch = asin(-R20); REP-103 positive = nose down).
    const Eigen::Matrix3d r_ref_base =
      (q_ref_imu.normalized() * base_from_imu.normalized().conjugate()).toRotationMatrix();
    imu.pitch = std::asin(std::clamp(-r_ref_base(2, 0), -1.0, 1.0));
    // Orientation covariance is about the IMU's axes; rotate it to the car's
    // and take the pitch (y) axis.
    imu.pitch_variance = rotated_variance(msg.orientation_covariance, r, 1);
  }
  return imu;
}

WheelSpeedMeasurement to_wheel_speed(const nav_msgs::msg::Odometry & msg)
{
  return {msg.twist.twist.linear.x, msg.twist.covariance[kLinX * 6 + kLinX]};
}

nav_msgs::msg::Odometry to_odometry(
  const RobotState & state, const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = kOdomFrame;
  odom.child_frame_id = kBaseFrame;

  odom.pose.pose.position.x = state.mean(kX);
  odom.pose.pose.position.y = state.mean(kY);
  set_quaternion_from_yaw(state.mean(kYaw), odom.pose.pose.orientation);
  odom.twist.twist.linear.x = state.mean(kSpeed);
  odom.twist.twist.angular.z = state.mean(kYawRate);

  const StateMatrix & p = state.covariance;
  auto & pose_cov = odom.pose.covariance;
  const std::array<std::pair<int, int>, 3> pose_map{{{kLinX, kX}, {kLinY, kY}, {kRotZ, kYaw}}};
  for (const auto & [row_msg, row_state] : pose_map) {
    for (const auto & [col_msg, col_state] : pose_map) {
      pose_cov[row_msg * 6 + col_msg] = p(row_state, col_state);
    }
  }
  for (int i : {kLinZ, kRotX, kRotY}) {
    pose_cov[i * 6 + i] = kPlanarPoseVariance;
  }

  auto & twist_cov = odom.twist.covariance;
  const std::array<std::pair<int, int>, 2> twist_map{{{kLinX, kSpeed}, {kRotZ, kYawRate}}};
  for (const auto & [row_msg, row_state] : twist_map) {
    for (const auto & [col_msg, col_state] : twist_map) {
      twist_cov[row_msg * 6 + col_msg] = p(row_state, col_state);
    }
  }
  for (int i : {kLinY, kLinZ, kRotX, kRotY}) {
    twist_cov[i * 6 + i] = kUnestimatedVariance;
  }
  return odom;
}

geometry_msgs::msg::TransformStamped to_transform(
  const RobotState & state, const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = kOdomFrame;
  tf.child_frame_id = kBaseFrame;
  tf.transform.translation.x = state.mean(kX);
  tf.transform.translation.y = state.mean(kY);
  set_quaternion_from_yaw(state.mean(kYaw), tf.transform.rotation);
  return tf;
}

HealthReport summarize_health(const HealthInputs & inputs)
{
  HealthReport report;
  const StateEstimationStatus & s = inputs.status;

  if (!inputs.active) {
    append(report, DiagnosticStatus::STALE, "inactive");
    return report;
  }
  if (std::isinf(s.imu_age)) {
    append(report, DiagnosticStatus::STALE, "waiting for /imu");
  } else if (s.imu_stale) {
    append(report, DiagnosticStatus::ERROR, "/imu stale");
  }
  if (std::isinf(s.wheel_age)) {
    append(report, DiagnosticStatus::STALE, "waiting for /wheel/odom");
  } else if (s.wheel_stale) {
    append(report, DiagnosticStatus::WARN, "/wheel/odom stale");
  }
  if (!inputs.imu_mount_known) {
    append(report, DiagnosticStatus::ERROR, "no TF from base_footprint to the IMU frame");
  }
  if (inputs.new_numerical_reset) {
    append(report, DiagnosticStatus::ERROR, "estimate was not finite; restored last good");
  }
  // Latched: slip that started and ended between two 1 Hz reports is still
  // reported once.
  if (s.slip_detected || inputs.new_slip_event) {
    append(report, DiagnosticStatus::WARN, "wheel slip detected");
  }
  if (inputs.new_rejected_sample) {
    append(report, DiagnosticStatus::WARN, "rejected samples with NaN/Inf or bad covariance");
  }
  if (report.message.empty()) {
    report.message = "OK";
  }
  return report;
}

}  // namespace state_estimation
