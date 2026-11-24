// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// L4 unit tests for the node's message conversions and health rules
// (VEH.GNC.SE spec §3, §5). Verifies REQ-GNC-006 (full covariance on
// /odom) and the ICD-SNS-GNC input rules, without spinning ROS.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "state_estimation/message_conversions.hpp"

namespace state_estimation
{
namespace
{

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
constexpr double kTol = 1e-12;

sensor_msgs::msg::Imu make_imu()
{
  sensor_msgs::msg::Imu msg;
  msg.angular_velocity.x = 0.1;
  msg.angular_velocity.y = 0.2;
  msg.angular_velocity.z = 0.3;
  msg.linear_acceleration.x = 1.0;
  msg.linear_acceleration.y = 2.0;
  msg.linear_acceleration.z = 9.8;
  msg.orientation.w = 1.0;
  for (int i : {0, 4, 8}) {
    msg.orientation_covariance[i] = 0.01;
  }
  msg.angular_velocity_covariance[0] = 1e-4;
  msg.angular_velocity_covariance[4] = 2e-4;
  msg.angular_velocity_covariance[8] = 3e-4;
  msg.linear_acceleration_covariance[0] = 4e-3;
  msg.linear_acceleration_covariance[4] = 5e-3;
  msg.linear_acceleration_covariance[8] = 6e-3;
  return msg;
}

// ---------------------------------------------------------------------------
// /imu
// ---------------------------------------------------------------------------

TEST(ImuConversionTest, AlignedMountPassesThrough)
{
  const ImuMeasurement imu = to_imu_measurement(make_imu(), Eigen::Quaterniond::Identity());
  EXPECT_NEAR(imu.yaw_rate, 0.3, kTol);
  EXPECT_NEAR(imu.yaw_rate_variance, 3e-4, kTol);
  EXPECT_NEAR(imu.forward_specific_force, 1.0, kTol);
  EXPECT_NEAR(imu.forward_specific_force_variance, 4e-3, kTol);
  ASSERT_TRUE(imu.pitch.has_value());
  EXPECT_NEAR(*imu.pitch, 0.0, kTol);
  EXPECT_NEAR(imu.pitch_variance, 0.01, kTol);
}

TEST(ImuConversionTest, RotatedMountIsUndone)
{
  // IMU mounted yawed +90 deg: its x axis points left on the car, so the
  // car's forward axis is the IMU's -y.
  const Eigen::Quaterniond base_from_imu(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  const ImuMeasurement imu = to_imu_measurement(make_imu(), base_from_imu);
  EXPECT_NEAR(imu.forward_specific_force, -2.0, kTol);
  EXPECT_NEAR(imu.forward_specific_force_variance, 5e-3, kTol);
  EXPECT_NEAR(imu.yaw_rate, 0.3, kTol);

  // Mounted upside down (roll 180 deg): yaw rate flips sign.
  const Eigen::Quaterniond flipped(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  EXPECT_NEAR(to_imu_measurement(make_imu(), flipped).yaw_rate, -0.3, kTol);
}

TEST(ImuConversionTest, ForwardOffsetPassedThrough)
{
  const ImuMeasurement imu =
    to_imu_measurement(make_imu(), Eigen::Quaterniond::Identity(), 0.23);
  EXPECT_DOUBLE_EQ(imu.forward_offset, 0.23);
}

TEST(ImuConversionTest, PitchFromOrientationIsNoseDownPositive)
{
  // REP-103: positive pitch is a rotation about +y, which tips the nose down.
  // Yaw 0.7 then pitch 0.2 (ZYX); the yaw must not leak into the pitch.
  sensor_msgs::msg::Imu msg = make_imu();
  const Eigen::Quaterniond zyx = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY());
  msg.orientation.w = zyx.w();
  msg.orientation.x = zyx.x();
  msg.orientation.y = zyx.y();
  msg.orientation.z = zyx.z();
  const ImuMeasurement imu = to_imu_measurement(msg, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(imu.pitch.has_value());
  EXPECT_NEAR(*imu.pitch, 0.2, 1e-9);
}

TEST(ImuConversionTest, PitchOfTheCarNotTheChip)
{
  // A chip mounted pitched 0.1 rad on a level car reads pitch 0.1; the car's
  // pitch is 0.
  const Eigen::Quaterniond mount(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()));
  sensor_msgs::msg::Imu msg = make_imu();
  msg.orientation.w = mount.w();
  msg.orientation.x = mount.x();
  msg.orientation.y = mount.y();
  msg.orientation.z = mount.z();
  const ImuMeasurement imu = to_imu_measurement(msg, mount);
  ASSERT_TRUE(imu.pitch.has_value());
  EXPECT_NEAR(*imu.pitch, 0.0, 1e-9);
}

TEST(ImuConversionTest, MissingOrientationGivesNoPitch)
{
  sensor_msgs::msg::Imu msg = make_imu();
  msg.orientation_covariance[0] = -1.0;
  EXPECT_FALSE(to_imu_measurement(msg, Eigen::Quaterniond::Identity()).pitch.has_value());

  msg = make_imu();
  msg.orientation.w = 0.0;  // all-zero quaternion: not set
  EXPECT_FALSE(to_imu_measurement(msg, Eigen::Quaterniond::Identity()).pitch.has_value());
}

TEST(ImuConversionTest, MissingCovarianceGivesNaNVariance)
{
  // The logic rejects NaN variances, so a sensor without covariances is
  // refused rather than trusted blindly (REQ-GNC-006).
  sensor_msgs::msg::Imu msg = make_imu();
  msg.angular_velocity_covariance[0] = -1.0;
  msg.linear_acceleration_covariance[0] = -1.0;
  const ImuMeasurement imu = to_imu_measurement(msg, Eigen::Quaterniond::Identity());
  EXPECT_TRUE(std::isnan(imu.yaw_rate_variance));
  EXPECT_TRUE(std::isnan(imu.forward_specific_force_variance));
}

// ---------------------------------------------------------------------------
// /wheel/odom
// ---------------------------------------------------------------------------

TEST(WheelConversionTest, UsesOnlyForwardSpeed)
{
  // ICD-SNS-GNC §4: consumers may rely on twist.linear.x only.
  nav_msgs::msg::Odometry msg;
  msg.twist.twist.linear.x = 1.5;
  msg.twist.twist.angular.z = 9.0;
  msg.pose.pose.position.x = 100.0;
  msg.twist.covariance[0] = 4e-4;
  const WheelSpeedMeasurement wheel = to_wheel_speed(msg);
  EXPECT_DOUBLE_EQ(wheel.speed, 1.5);
  EXPECT_DOUBLE_EQ(wheel.variance, 4e-4);
}

// ---------------------------------------------------------------------------
// /odom and TF
// ---------------------------------------------------------------------------

RobotState make_state()
{
  RobotState s;
  s.mean << 1.0, 2.0, 0.5, 1.2, -0.3, 0.07;
  StateMatrix a = StateMatrix::Random();
  s.covariance = a * a.transpose() + StateMatrix::Identity();
  return s;
}

TEST(OdometryConversionTest, FramesAndValues)
{
  const RobotState s = make_state();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34;
  const nav_msgs::msg::Odometry odom = to_odometry(s, stamp);

  EXPECT_EQ(odom.header.frame_id, "odom");
  EXPECT_EQ(odom.child_frame_id, "base_footprint");
  EXPECT_EQ(odom.header.stamp.sec, 12);
  EXPECT_EQ(odom.header.stamp.nanosec, 34u);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.z, 0.0);
  EXPECT_NEAR(
    2.0 * std::atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w), 0.5,
    kTol);
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.x, 1.2);
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.y, 0.07);  // sideways speed (2.5)
  EXPECT_DOUBLE_EQ(odom.twist.twist.angular.z, -0.3);
}

TEST(OdometryConversionTest, FullCovarianceMapped)
{
  // REQ-GNC-006: pose (x, y, yaw) and twist (v_x, v_y, omega) blocks,
  // including cross terms. Pose z, roll, pitch are 0 by definition of base_footprint
  // (variance 0, which RViz needs to draw a 2D ellipse); velocities a 2D
  // estimate does not estimate get 1e6.
  const RobotState s = make_state();
  const nav_msgs::msg::Odometry odom = to_odometry(s, builtin_interfaces::msg::Time{});
  const auto & pc = odom.pose.covariance;
  const auto & tc = odom.twist.covariance;
  const StateMatrix & p = s.covariance;

  EXPECT_DOUBLE_EQ(pc[0 * 6 + 0], p(kX, kX));
  EXPECT_DOUBLE_EQ(pc[0 * 6 + 1], p(kX, kY));
  EXPECT_DOUBLE_EQ(pc[1 * 6 + 5], p(kY, kYaw));
  EXPECT_DOUBLE_EQ(pc[5 * 6 + 0], p(kYaw, kX));
  EXPECT_DOUBLE_EQ(pc[5 * 6 + 5], p(kYaw, kYaw));
  for (int i : {2, 3, 4}) {
    EXPECT_DOUBLE_EQ(pc[i * 6 + i], kPlanarPoseVariance);
  }

  EXPECT_DOUBLE_EQ(tc[0 * 6 + 0], p(kSpeed, kSpeed));
  EXPECT_DOUBLE_EQ(tc[0 * 6 + 1], p(kSpeed, kSideSpeed));
  EXPECT_DOUBLE_EQ(tc[1 * 6 + 1], p(kSideSpeed, kSideSpeed));
  EXPECT_DOUBLE_EQ(tc[1 * 6 + 5], p(kSideSpeed, kYawRate));
  EXPECT_DOUBLE_EQ(tc[0 * 6 + 5], p(kSpeed, kYawRate));
  EXPECT_DOUBLE_EQ(tc[5 * 6 + 5], p(kYawRate, kYawRate));
  for (int i : {2, 3, 4}) {
    EXPECT_DOUBLE_EQ(tc[i * 6 + i], kUnestimatedVariance);
  }
}

TEST(TransformConversionTest, MatchesOdometry)
{
  const RobotState s = make_state();
  const auto tf = to_transform(s, builtin_interfaces::msg::Time{});
  const auto odom = to_odometry(s, builtin_interfaces::msg::Time{});
  EXPECT_EQ(tf.header.frame_id, "odom");
  EXPECT_EQ(tf.child_frame_id, "base_footprint");
  EXPECT_DOUBLE_EQ(tf.transform.translation.x, odom.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(tf.transform.translation.y, odom.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(tf.transform.rotation.z, odom.pose.pose.orientation.z);
  EXPECT_DOUBLE_EQ(tf.transform.rotation.w, odom.pose.pose.orientation.w);
}

// ---------------------------------------------------------------------------
// Health (spec §5)
// ---------------------------------------------------------------------------

HealthInputs healthy()
{
  HealthInputs in;
  in.active = true;
  in.status.imu_age = 0.01;
  in.status.wheel_age = 0.01;
  in.status.imu_stale = false;
  in.status.wheel_stale = false;
  return in;
}

TEST(HealthTest, OkWhenInputsFresh)
{
  const HealthReport r = summarize_health(healthy());
  EXPECT_EQ(r.level, DiagnosticStatus::OK);
  EXPECT_EQ(r.message, "OK");
}

TEST(HealthTest, StaleUntilInputsArriveOrWhenInactive)
{
  HealthInputs in = healthy();
  in.status.imu_age = std::numeric_limits<double>::infinity();
  in.status.imu_stale = true;
  EXPECT_EQ(summarize_health(in).level, DiagnosticStatus::STALE);

  in = healthy();
  in.active = false;
  EXPECT_EQ(summarize_health(in).level, DiagnosticStatus::STALE);
  EXPECT_EQ(summarize_health(in).message, "inactive");
}

TEST(HealthTest, StaleWheelIsWarnStaleImuIsError)
{
  // ICD-SNS-GNC §5.
  HealthInputs in = healthy();
  in.status.wheel_stale = true;
  EXPECT_EQ(summarize_health(in).level, DiagnosticStatus::WARN);

  in = healthy();
  in.status.imu_stale = true;
  EXPECT_EQ(summarize_health(in).level, DiagnosticStatus::ERROR);
}

TEST(HealthTest, SlipRejectsAndNumericalResets)
{
  HealthInputs in = healthy();
  in.status.slip_detected = true;
  EXPECT_EQ(summarize_health(in).level, DiagnosticStatus::WARN);

  in = healthy();
  in.new_slip_event = true;  // slip came and went since the last report
  EXPECT_EQ(summarize_health(in).level, DiagnosticStatus::WARN);
  EXPECT_NE(summarize_health(in).message.find("slip"), std::string::npos);

  in = healthy();
  in.new_rejected_sample = true;
  EXPECT_EQ(summarize_health(in).level, DiagnosticStatus::WARN);

  in = healthy();
  in.new_numerical_reset = true;
  EXPECT_EQ(summarize_health(in).level, DiagnosticStatus::ERROR);

  in = healthy();
  in.imu_mount_known = false;
  EXPECT_EQ(summarize_health(in).level, DiagnosticStatus::ERROR);
}

TEST(HealthTest, ErrorOutranksStaleAndAllReasonsListed)
{
  // STALE is numerically above ERROR in the message; ERROR must still win.
  HealthInputs in = healthy();
  in.status.imu_stale = true;                                      // ERROR
  in.status.wheel_age = std::numeric_limits<double>::infinity();   // STALE
  in.status.slip_detected = true;                                  // WARN
  const HealthReport r = summarize_health(in);
  EXPECT_EQ(r.level, DiagnosticStatus::ERROR);
  EXPECT_NE(r.message.find("/imu stale"), std::string::npos);
  EXPECT_NE(r.message.find("waiting for /wheel/odom"), std::string::npos);
  EXPECT_NE(r.message.find("slip"), std::string::npos);

  in = healthy();
  in.status.wheel_age = std::numeric_limits<double>::infinity();  // STALE
  in.status.slip_detected = true;                                 // WARN
  EXPECT_EQ(summarize_health(in).level, DiagnosticStatus::STALE);
}

}  // namespace
}  // namespace state_estimation
