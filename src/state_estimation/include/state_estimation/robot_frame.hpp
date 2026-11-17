// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Motion models for the vehicle's robot frame (VEH.GNC.SE, spec §6).
//
// Pure C++, no ROS: used by state_estimation for the predict step and
// reusable by planning_control. A RobotFrame turns body-frame velocities
// into the next 2D pose, after limiting them to what the chassis can do.

#ifndef STATE_ESTIMATION__ROBOT_FRAME_HPP_
#define STATE_ESTIMATION__ROBOT_FRAME_HPP_

#include <Eigen/Dense>

#include <memory>
#include <string>

namespace state_estimation
{

/// 2D pose in the odom frame (REP-103: x forward, y left, yaw CCW).
struct Pose2D
{
  double x{0.0};    ///< m
  double y{0.0};    ///< m
  double yaw{0.0};  ///< rad, normalised to [-pi, pi)
};

/// Body-frame velocities, held constant over one step.
struct VelocityInputs
{
  double linear{0.0};   ///< forward speed v, m/s (negative = reverse)
  double angular{0.0};  ///< yaw rate omega, rad/s (CCW positive)
};

/// Derivatives of the next pose (x, y, yaw) for the EKF predict step.
struct PoseJacobians
{
  Eigen::Matrix3d wrt_pose;                ///< d(next pose) / d(x, y, yaw)
  Eigen::Matrix<double, 3, 2> wrt_inputs;  ///< d(next pose) / d(v, omega)
};

/// Parameters for building a RobotFrame (spec §3.4).
struct RobotFrameParams
{
  double wheelbase{0.384};           ///< m, bicycle model only
  double max_steering_angle{0.52};   ///< rad, bicycle model only
};

/// Wrap an angle to [-pi, pi).
double normalize_angle(double angle);

/// Motion model interface (the bridge in the Software Design diagram).
///
/// Integration is exact for constant (v, omega) over dt: the pose moves
/// along a circular arc, or a straight line when omega is ~0.
class RobotFrame
{
public:
  virtual ~RobotFrame() = default;

  /// Name used for the `motion_model` parameter.
  virtual std::string name() const = 0;

  /// Limit the inputs to what this chassis can physically do.
  virtual VelocityInputs constrain(const VelocityInputs & inputs) const = 0;

  /// Next pose after dt seconds with the constrained inputs.
  /// @throws std::invalid_argument if dt is negative or not finite.
  Pose2D get_next_pose_estimate(
    const Pose2D & pose, const VelocityInputs & inputs, double dt) const;

  /// Jacobians of get_next_pose_estimate(), evaluated at the constrained
  /// inputs. Derivatives are with respect to the constrained inputs; a
  /// clamped yaw rate is treated as locally free.
  /// @throws std::invalid_argument if dt is negative or not finite.
  PoseJacobians jacobians(
    const Pose2D & pose, const VelocityInputs & inputs, double dt) const;
};

/// Ackermann car as a kinematic bicycle (the Traxxas Slash).
///
/// Yaw rate is tied to speed by omega = v * tan(delta) / L, so the car
/// cannot turn in place and |omega| <= |v| * tan(delta_max) / L.
class AckermannSteering : public RobotFrame
{
public:
  /// @throws std::invalid_argument unless wheelbase > 0 and
  ///         0 < max_steering_angle < pi/2.
  AckermannSteering(double wheelbase, double max_steering_angle);

  std::string name() const override {return "bicycle";}
  VelocityInputs constrain(const VelocityInputs & inputs) const override;

  /// Yaw rate for a front-wheel steering angle, after clamping the angle.
  double yaw_rate_from_steering(double speed, double steering_angle) const;

  /// Steering angle that gives this yaw rate at this speed, clamped to the
  /// steering limit. Returns 0 when the speed is ~0.
  double steering_from_yaw_rate(double speed, double yaw_rate) const;

  double wheelbase() const {return wheelbase_;}
  double max_steering_angle() const {return max_steering_angle_;}

private:
  double wheelbase_;
  double max_steering_angle_;
};

/// Two-wheel differential drive as a unicycle: v and omega are independent,
/// and it can turn in place.
class TwoWheelDifferentialSteering : public RobotFrame
{
public:
  std::string name() const override {return "unicycle";}
  VelocityInputs constrain(const VelocityInputs & inputs) const override;
};

/// Build the motion model named by the `motion_model` parameter
/// ("bicycle" or "unicycle"), REQ-GNC-007.
/// @throws std::invalid_argument for an unknown name or invalid parameters.
std::unique_ptr<RobotFrame> make_robot_frame(
  const std::string & motion_model, const RobotFrameParams & params);

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__ROBOT_FRAME_HPP_
