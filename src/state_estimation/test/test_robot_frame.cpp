// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// L4 unit tests for the motion models (VEH.GNC.SE spec §7, test_robot_frame).
// Verifies REQ-GNC-001 (prediction for the 50 Hz estimate) and REQ-GNC-007
// (motion model chosen by name).

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>

#include "state_estimation/robot_frame.hpp"

namespace state_estimation
{
namespace
{

constexpr double kWheelbase = 0.384;      // m, rc_racer.urdf.xacro
constexpr double kMaxSteering = 0.52;     // rad, wheels.urdf.xacro
constexpr double kRearAxleOffset = kWheelbase / 2.0;  // base_footprint midway
constexpr double kTol = 1e-9;

AckermannSteering make_car()
{
  return AckermannSteering(kWheelbase, kMaxSteering, kRearAxleOffset);
}

// Turn centre and radius of base_footprint for a car starting at the origin
// facing +x and turning left with rear-axle radius r: the rear axle (at
// x = -a) circles (-a, r), and base_footprint, a ahead of it, circles the
// same centre at sqrt(r^2 + a^2).
double footprint_radius(double r) {return std::hypot(r, kRearAxleOffset);}

// Drive n steps of dt and return the final pose.
Pose2D drive(const RobotFrame & frame, Pose2D pose, VelocityInputs in, double dt, int n)
{
  for (int i = 0; i < n; ++i) {
    pose = frame.get_next_pose_estimate(pose, in, dt);
  }
  return pose;
}

// ---------------------------------------------------------------------------
// Straight line
// ---------------------------------------------------------------------------

TEST(RobotFrameTest, StraightLineFollowsHeading)
{
  const TwoWheelDifferentialSteering unicycle;
  const AckermannSteering car = make_car();
  const Pose2D start{1.0, -2.0, M_PI / 4.0};

  for (const RobotFrame * frame : {static_cast<const RobotFrame *>(&unicycle),
      static_cast<const RobotFrame *>(&car)})
  {
    const Pose2D end = frame->get_next_pose_estimate(start, {2.0, 0.0}, 1.5);
    EXPECT_NEAR(end.x, 1.0 + 3.0 * std::cos(M_PI / 4.0), kTol) << frame->name();
    EXPECT_NEAR(end.y, -2.0 + 3.0 * std::sin(M_PI / 4.0), kTol) << frame->name();
    EXPECT_NEAR(end.yaw, M_PI / 4.0, kTol) << frame->name();
  }
}

TEST(RobotFrameTest, ReverseDrivesBackwards)
{
  const AckermannSteering car = make_car();
  const Pose2D end = car.get_next_pose_estimate({0.0, 0.0, 0.0}, {-1.0, 0.0}, 2.0);
  EXPECT_NEAR(end.x, -2.0, kTol);
  EXPECT_NEAR(end.y, 0.0, kTol);
}

TEST(RobotFrameTest, ZeroDtLeavesPoseUnchanged)
{
  const TwoWheelDifferentialSteering unicycle;
  const Pose2D start{3.0, 4.0, 1.0};
  const Pose2D end = unicycle.get_next_pose_estimate(start, {5.0, 2.0}, 0.0);
  EXPECT_NEAR(end.x, start.x, kTol);
  EXPECT_NEAR(end.y, start.y, kTol);
  EXPECT_NEAR(end.yaw, start.yaw, kTol);
}

// ---------------------------------------------------------------------------
// Constant-radius turn
// ---------------------------------------------------------------------------

TEST(RobotFrameTest, ConstantTurnStaysOnCircle)
{
  // v = 1 m/s, omega = 0.5 rad/s: radius 2 m, centre at (0, 2) for a
  // CCW turn starting at the origin facing +x.
  const TwoWheelDifferentialSteering unicycle;
  Pose2D pose;
  for (int i = 0; i < 300; ++i) {
    pose = unicycle.get_next_pose_estimate(pose, {1.0, 0.5}, 0.02);
    EXPECT_NEAR(std::hypot(pose.x, pose.y - 2.0), 2.0, 1e-9);
  }
}

TEST(RobotFrameTest, FullCircleReturnsToStart)
{
  const TwoWheelDifferentialSteering unicycle;
  const double omega = 0.5;
  const int steps = 1000;
  const double dt = 2.0 * M_PI / omega / steps;
  const Pose2D end = drive(unicycle, {}, {1.0, omega}, dt, steps);
  EXPECT_NEAR(end.x, 0.0, 1e-9);
  EXPECT_NEAR(end.y, 0.0, 1e-9);
  EXPECT_NEAR(std::abs(end.yaw), 0.0, 1e-9);
}

TEST(RobotFrameTest, IntegrationIsExactForConstantInputs)
{
  // One big step equals many small ones: the arc is integrated exactly, so
  // the result does not depend on the publish rate.
  const TwoWheelDifferentialSteering unicycle;
  const Pose2D start{0.5, 0.5, -0.3};
  const VelocityInputs in{1.3, -0.7};
  const Pose2D one = unicycle.get_next_pose_estimate(start, in, 2.0);
  const Pose2D many = drive(unicycle, start, in, 0.02, 100);
  EXPECT_NEAR(one.x, many.x, 1e-9);
  EXPECT_NEAR(one.y, many.y, 1e-9);
  EXPECT_NEAR(one.yaw, many.yaw, 1e-9);
}

TEST(RobotFrameTest, BicycleTurnRadiusMatchesSteeringAngle)
{
  // Kinematic bicycle: R = L / tan(delta).
  const AckermannSteering car = make_car();
  const double delta = 0.3;
  const double radius = kWheelbase / std::tan(delta);
  const double omega = car.yaw_rate_from_steering(1.0, delta);
  EXPECT_NEAR(omega, 1.0 / radius, kTol);

  Pose2D pose;
  for (int i = 0; i < 200; ++i) {
    pose = car.get_next_pose_estimate(pose, {1.0, omega}, 0.02);
    EXPECT_NEAR(
      std::hypot(pose.x + kRearAxleOffset, pose.y - radius), footprint_radius(radius), 1e-9);
  }
}

TEST(RobotFrameTest, YawIsNormalised)
{
  const TwoWheelDifferentialSteering unicycle;
  const Pose2D end = unicycle.get_next_pose_estimate({0.0, 0.0, 3.0}, {0.0, 1.0}, 1.0);
  EXPECT_NEAR(end.yaw, 4.0 - 2.0 * M_PI, kTol);
  EXPECT_GE(end.yaw, -M_PI);
  EXPECT_LT(end.yaw, M_PI);
}

TEST(NormalizeAngleTest, WrapsIntoHalfOpenRange)
{
  EXPECT_NEAR(normalize_angle(0.0), 0.0, kTol);
  EXPECT_NEAR(normalize_angle(M_PI), -M_PI, kTol);
  EXPECT_NEAR(normalize_angle(-M_PI), -M_PI, kTol);
  EXPECT_NEAR(normalize_angle(7.0 * M_PI / 2.0), -M_PI / 2.0, kTol);
  EXPECT_NEAR(normalize_angle(-5.0 * M_PI / 2.0), -M_PI / 2.0, kTol);
}

// ---------------------------------------------------------------------------
// Steering limit
// ---------------------------------------------------------------------------

TEST(AckermannSteeringTest, YawRateClampedToSteeringLimit)
{
  const AckermannSteering car = make_car();
  const double max_yaw_rate = 2.0 * std::tan(kMaxSteering) / kWheelbase;

  EXPECT_NEAR(car.constrain({2.0, 100.0}).angular, max_yaw_rate, kTol);
  EXPECT_NEAR(car.constrain({2.0, -100.0}).angular, -max_yaw_rate, kTol);
  EXPECT_NEAR(car.constrain({-2.0, 100.0}).angular, max_yaw_rate, kTol);
  EXPECT_NEAR(car.constrain({2.0, 0.5}).angular, 0.5, kTol);  // inside the limit
  EXPECT_NEAR(car.constrain({2.0, 100.0}).linear, 2.0, kTol);  // speed untouched
}

TEST(AckermannSteeringTest, ClampedTurnUsesMinimumRadius)
{
  const AckermannSteering car = make_car();
  const double min_radius = kWheelbase / std::tan(kMaxSteering);
  const VelocityInputs in = car.constrain({1.0, 50.0});
  Pose2D pose;
  for (int i = 0; i < 100; ++i) {
    pose = car.get_next_pose_estimate(pose, in, 0.02);
    EXPECT_NEAR(
      std::hypot(pose.x + kRearAxleOffset, pose.y - min_radius), footprint_radius(min_radius),
      1e-9);
  }
}

TEST(AckermannSteeringTest, FootprintSlidesSidewaysOnATurn)
{
  // Only the rear axle has no sideways motion. After a half turn the
  // footprint is displaced by a * (heading_end - heading_start) = (-2a, 0)
  // relative to where the rear axle's arc alone would put it.
  const AckermannSteering car = make_car();
  const TwoWheelDifferentialSteering axle_only;
  const double omega = 0.5;
  const double dt = M_PI / omega / 100.0;
  const Pose2D car_end = drive(car, {}, {1.0, omega}, dt, 100);
  const Pose2D axle_end = drive(axle_only, {}, {1.0, omega}, dt, 100);
  EXPECT_NEAR(car_end.x - axle_end.x, -2.0 * kRearAxleOffset, 1e-9);
  EXPECT_NEAR(car_end.y - axle_end.y, 0.0, 1e-9);
  EXPECT_NEAR(car_end.yaw, axle_end.yaw, 1e-9);
}

TEST(AckermannSteeringTest, OffsetDoesNotMatterOnAStraightLine)
{
  const AckermannSteering car = make_car();
  const Pose2D end = car.get_next_pose_estimate({1.0, 2.0, 0.3}, {2.0, 0.0}, 1.0);
  EXPECT_NEAR(end.x, 1.0 + 2.0 * std::cos(0.3), kTol);
  EXPECT_NEAR(end.y, 2.0 + 2.0 * std::sin(0.3), kTol);
}

TEST(AckermannSteeringTest, CannotTurnInPlace)
{
  const AckermannSteering car = make_car();
  const Pose2D start{1.0, 1.0, 0.5};
  const Pose2D end = car.get_next_pose_estimate(start, car.constrain({0.0, 1.0}), 1.0);
  EXPECT_NEAR(end.x, start.x, kTol);
  EXPECT_NEAR(end.y, start.y, kTol);
  EXPECT_NEAR(end.yaw, start.yaw, kTol);
}

TEST(AckermannSteeringTest, IntegrationFollowsASlideBeyondTheLimit)
{
  // A yaw rate the steering cannot produce is a slide; the pose must still
  // follow it, or a skid would corrupt the heading for good.
  const AckermannSteering car = make_car();
  const Pose2D end = car.get_next_pose_estimate({}, {0.0, 1.0}, 1.0);
  EXPECT_NEAR(end.yaw, 1.0, kTol);
}

TEST(AckermannSteeringTest, SteeringConversionsRoundTrip)
{
  const AckermannSteering car = make_car();
  for (double speed : {-2.0, -0.5, 0.5, 3.0}) {
    for (double delta : {-0.5, -0.1, 0.0, 0.2, 0.5}) {
      const double omega = car.yaw_rate_from_steering(speed, delta);
      EXPECT_NEAR(car.steering_from_yaw_rate(speed, omega), delta, 1e-12)
        << "speed " << speed << " delta " << delta;
    }
  }
}

TEST(AckermannSteeringTest, SteeringConversionsClampToLimit)
{
  const AckermannSteering car = make_car();
  EXPECT_NEAR(car.steering_from_yaw_rate(1.0, 100.0), kMaxSteering, kTol);
  EXPECT_NEAR(car.steering_from_yaw_rate(1.0, -100.0), -kMaxSteering, kTol);
  EXPECT_NEAR(car.steering_from_yaw_rate(0.0, 1.0), 0.0, kTol);
  EXPECT_NEAR(
    car.yaw_rate_from_steering(1.0, 1.0),
    std::tan(kMaxSteering) / kWheelbase, kTol);
}

TEST(TwoWheelDifferentialSteeringTest, TurnsInPlace)
{
  const TwoWheelDifferentialSteering unicycle;
  const Pose2D end = unicycle.get_next_pose_estimate({1.0, 1.0, 0.0}, {0.0, 1.0}, 1.0);
  EXPECT_NEAR(end.x, 1.0, kTol);
  EXPECT_NEAR(end.y, 1.0, kTol);
  EXPECT_NEAR(end.yaw, 1.0, kTol);
}

// ---------------------------------------------------------------------------
// Jacobians (used by the EKF predict step)
// ---------------------------------------------------------------------------

// Central finite differences of the next pose with respect to (x, y, yaw, v, omega).
Eigen::Matrix<double, 3, 5> numeric_jacobian(
  const RobotFrame & frame, const Pose2D & pose, const VelocityInputs & in, double dt)
{
  const double h = 1e-6;
  Eigen::Matrix<double, 5, 1> base;
  base << pose.x, pose.y, pose.yaw, in.linear, in.angular;
  Eigen::Matrix<double, 3, 5> jac;
  for (int k = 0; k < 5; ++k) {
    Eigen::Matrix<double, 5, 1> plus = base;
    Eigen::Matrix<double, 5, 1> minus = base;
    plus(k) += h;
    minus(k) -= h;
    const Pose2D p = frame.get_next_pose_estimate(
      {plus(0), plus(1), plus(2)}, {plus(3), plus(4)}, dt);
    const Pose2D m = frame.get_next_pose_estimate(
      {minus(0), minus(1), minus(2)}, {minus(3), minus(4)}, dt);
    jac(0, k) = (p.x - m.x) / (2.0 * h);
    jac(1, k) = (p.y - m.y) / (2.0 * h);
    jac(2, k) = normalize_angle(p.yaw - m.yaw) / (2.0 * h);
  }
  return jac;
}

void expect_jacobians_match(
  const RobotFrame & frame, const Pose2D & pose, const VelocityInputs & in, double dt)
{
  const PoseJacobians j = frame.jacobians(pose, in, dt);
  Eigen::Matrix<double, 3, 5> analytic;
  analytic << j.wrt_pose, j.wrt_inputs;
  const Eigen::Matrix<double, 3, 5> numeric = numeric_jacobian(frame, pose, in, dt);
  EXPECT_TRUE(analytic.isApprox(numeric, 1e-6) || (analytic - numeric).norm() < 1e-6)
    << "analytic:\n" << analytic << "\nnumeric:\n" << numeric;
}

TEST(RobotFrameJacobianTest, MatchesFiniteDifferencesWhileTurning)
{
  const TwoWheelDifferentialSteering unicycle;
  expect_jacobians_match(unicycle, {0.3, -1.2, 0.8}, {1.5, 0.6}, 0.02);
  expect_jacobians_match(unicycle, {0.3, -1.2, 2.9}, {-0.8, -1.4}, 0.5);
}

TEST(RobotFrameJacobianTest, MatchesFiniteDifferencesOnStraightLine)
{
  // omega = 0 exercises the straight-line branch.
  const TwoWheelDifferentialSteering unicycle;
  expect_jacobians_match(unicycle, {0.3, -1.2, 0.8}, {1.5, 0.0}, 0.02);
}

TEST(RobotFrameJacobianTest, BicycleInsideSteeringLimit)
{
  const AckermannSteering car = make_car();
  expect_jacobians_match(car, {0.0, 0.0, -0.4}, {2.0, 1.0}, 0.02);
  expect_jacobians_match(car, {1.0, -2.0, 2.5}, {1.0, 0.0}, 0.02);  // straight branch
  expect_jacobians_match(car, {1.0, -2.0, 2.5}, {-0.7, 3.0}, 0.3);  // beyond the limit
}

TEST(RobotFrameJacobianTest, YawRowIsSimple)
{
  const TwoWheelDifferentialSteering unicycle;
  const PoseJacobians j = unicycle.jacobians({}, {1.0, 0.5}, 0.02);
  EXPECT_NEAR(j.wrt_pose(2, 2), 1.0, kTol);
  EXPECT_NEAR(j.wrt_inputs(2, 0), 0.0, kTol);
  EXPECT_NEAR(j.wrt_inputs(2, 1), 0.02, kTol);
}

// ---------------------------------------------------------------------------
// Input validation and factory (REQ-GNC-007)
// ---------------------------------------------------------------------------

TEST(RobotFrameTest, RejectsInvalidDt)
{
  const TwoWheelDifferentialSteering unicycle;
  EXPECT_THROW(unicycle.get_next_pose_estimate({}, {1.0, 0.0}, -0.01), std::invalid_argument);
  EXPECT_THROW(unicycle.get_next_pose_estimate({}, {1.0, 0.0}, NAN), std::invalid_argument);
  EXPECT_THROW(unicycle.jacobians({}, {1.0, 0.0}, INFINITY), std::invalid_argument);
}

TEST(AckermannSteeringTest, RejectsInvalidParameters)
{
  EXPECT_THROW(AckermannSteering(0.0, 0.5, 0.0), std::invalid_argument);
  EXPECT_THROW(AckermannSteering(-0.3, 0.5, 0.0), std::invalid_argument);
  EXPECT_THROW(AckermannSteering(NAN, 0.5, 0.0), std::invalid_argument);
  EXPECT_THROW(AckermannSteering(0.384, 0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(AckermannSteering(0.384, M_PI_2, 0.0), std::invalid_argument);
  EXPECT_THROW(AckermannSteering(0.384, 0.5, -0.1), std::invalid_argument);
  EXPECT_THROW(AckermannSteering(0.384, 0.5, 0.5), std::invalid_argument);
}

TEST(MakeRobotFrameTest, BuildsModelByName)
{
  const RobotFrameParams params{0.5, 0.4};
  const auto bicycle = make_robot_frame("bicycle", params);
  const auto unicycle = make_robot_frame("unicycle", params);
  EXPECT_EQ(bicycle->name(), "bicycle");
  EXPECT_EQ(unicycle->name(), "unicycle");

  const auto * car = dynamic_cast<const AckermannSteering *>(bicycle.get());
  ASSERT_NE(car, nullptr);
  EXPECT_DOUBLE_EQ(car->wheelbase(), 0.5);
  EXPECT_DOUBLE_EQ(car->max_steering_angle(), 0.4);
}

TEST(MakeRobotFrameTest, DefaultsMatchTheCar)
{
  const RobotFrameParams params;
  EXPECT_DOUBLE_EQ(params.wheelbase, kWheelbase);
  EXPECT_DOUBLE_EQ(params.max_steering_angle, kMaxSteering);
  EXPECT_DOUBLE_EQ(params.rear_axle_offset, kRearAxleOffset);
  EXPECT_DOUBLE_EQ(make_robot_frame("bicycle", params)->rear_axle_offset(), kRearAxleOffset);
  EXPECT_DOUBLE_EQ(make_robot_frame("unicycle", params)->rear_axle_offset(), 0.0);
}

TEST(MakeRobotFrameTest, RejectsUnknownModel)
{
  EXPECT_THROW(make_robot_frame("tank", RobotFrameParams{}), std::invalid_argument);
  EXPECT_THROW(make_robot_frame("", RobotFrameParams{}), std::invalid_argument);
  EXPECT_THROW(make_robot_frame("bicycle", RobotFrameParams{-1.0, 0.5}), std::invalid_argument);
}

}  // namespace
}  // namespace state_estimation
