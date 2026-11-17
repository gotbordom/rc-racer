// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include "state_estimation/robot_frame.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace state_estimation
{

namespace
{

// Below this turn angle per step (rad) the arc formulas divide by ~0, so
// integrate along the chord at the mid-step heading instead. The error is
// O(v * dt * (omega * dt)^2), far below anything measurable.
constexpr double kStraightLineTurn = 1e-6;

// Below this speed (m/s) a bicycle has no defined steering geometry.
constexpr double kStoppedSpeed = 1e-9;

void check_dt(double dt)
{
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument("dt must be finite and non-negative");
  }
}

}  // namespace

double normalize_angle(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

Pose2D RobotFrame::get_next_pose_estimate(
  const Pose2D & pose, const VelocityInputs & inputs, double dt) const
{
  check_dt(dt);
  const double v = inputs.linear;
  const double w = inputs.angular;
  const double turn = w * dt;

  Pose2D next;
  if (std::abs(turn) < kStraightLineTurn) {
    const double mid_yaw = pose.yaw + 0.5 * turn;
    next.x = pose.x + v * dt * std::cos(mid_yaw);
    next.y = pose.y + v * dt * std::sin(mid_yaw);
  } else {
    const double r = v / w;
    next.x = pose.x + r * (std::sin(pose.yaw + turn) - std::sin(pose.yaw));
    next.y = pose.y + r * (std::cos(pose.yaw) - std::cos(pose.yaw + turn));
  }
  next.yaw = normalize_angle(pose.yaw + turn);
  return next;
}

PoseJacobians RobotFrame::jacobians(
  const Pose2D & pose, const VelocityInputs & inputs, double dt) const
{
  check_dt(dt);
  const double v = inputs.linear;
  const double w = inputs.angular;
  const double turn = w * dt;

  PoseJacobians j;
  j.wrt_pose.setIdentity();
  j.wrt_inputs.setZero();
  j.wrt_inputs(2, 1) = dt;  // d yaw / d omega

  if (std::abs(turn) < kStraightLineTurn) {
    const double mid_yaw = pose.yaw + 0.5 * turn;
    const double c = std::cos(mid_yaw);
    const double s = std::sin(mid_yaw);
    j.wrt_pose(0, 2) = -v * dt * s;
    j.wrt_pose(1, 2) = v * dt * c;
    j.wrt_inputs(0, 0) = dt * c;
    j.wrt_inputs(1, 0) = dt * s;
    j.wrt_inputs(0, 1) = -0.5 * v * dt * dt * s;
    j.wrt_inputs(1, 1) = 0.5 * v * dt * dt * c;
  } else {
    const double s0 = std::sin(pose.yaw);
    const double c0 = std::cos(pose.yaw);
    const double s1 = std::sin(pose.yaw + turn);
    const double c1 = std::cos(pose.yaw + turn);
    j.wrt_pose(0, 2) = v / w * (c1 - c0);
    j.wrt_pose(1, 2) = v / w * (s1 - s0);
    j.wrt_inputs(0, 0) = (s1 - s0) / w;
    j.wrt_inputs(1, 0) = (c0 - c1) / w;
    j.wrt_inputs(0, 1) = v * (dt * c1 / w - (s1 - s0) / (w * w));
    j.wrt_inputs(1, 1) = v * (dt * s1 / w - (c0 - c1) / (w * w));
  }
  return j;
}

AckermannSteering::AckermannSteering(double wheelbase, double max_steering_angle)
: wheelbase_(wheelbase), max_steering_angle_(max_steering_angle)
{
  if (!(wheelbase > 0.0) || !std::isfinite(wheelbase)) {
    throw std::invalid_argument("wheelbase must be positive");
  }
  if (!(max_steering_angle > 0.0 && max_steering_angle < M_PI_2)) {
    throw std::invalid_argument("max_steering_angle must be in (0, pi/2)");
  }
}

VelocityInputs AckermannSteering::constrain(const VelocityInputs & inputs) const
{
  const double max_yaw_rate = std::abs(inputs.linear) * std::tan(max_steering_angle_) / wheelbase_;
  return {inputs.linear, std::clamp(inputs.angular, -max_yaw_rate, max_yaw_rate)};
}

double AckermannSteering::yaw_rate_from_steering(double speed, double steering_angle) const
{
  const double delta = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);
  return speed * std::tan(delta) / wheelbase_;
}

double AckermannSteering::steering_from_yaw_rate(double speed, double yaw_rate) const
{
  if (std::abs(speed) < kStoppedSpeed) {
    return 0.0;
  }
  const double delta = std::atan(yaw_rate * wheelbase_ / speed);
  return std::clamp(delta, -max_steering_angle_, max_steering_angle_);
}

VelocityInputs TwoWheelDifferentialSteering::constrain(const VelocityInputs & inputs) const
{
  return inputs;
}

std::unique_ptr<RobotFrame> make_robot_frame(
  const std::string & motion_model, const RobotFrameParams & params)
{
  if (motion_model == "bicycle") {
    return std::make_unique<AckermannSteering>(params.wheelbase, params.max_steering_angle);
  }
  if (motion_model == "unicycle") {
    return std::make_unique<TwoWheelDifferentialSteering>();
  }
  throw std::invalid_argument(
          "unknown motion_model '" + motion_model + "' (expected 'bicycle' or 'unicycle')");
}

}  // namespace state_estimation
