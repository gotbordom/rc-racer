// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// State estimation logic for VEH.GNC.SE (spec §4, §6): fuses IMU and wheel
// speed into the vehicle's own motion in the odom frame. Pure C++, no ROS;
// StateEstimationNode converts messages to these structs and calls it.
//
// It never uses world-frame data (ISSUE-002): the IMU's absolute yaw is
// ignored, and only its roll and pitch are used, to remove gravity.

#ifndef STATE_ESTIMATION__STATE_ESTIMATION_LOGIC_HPP_
#define STATE_ESTIMATION__STATE_ESTIMATION_LOGIC_HPP_

#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "state_estimation/robot_frame.hpp"
#include "state_estimation/robot_state.hpp"
#include "state_estimation/sensor_fusion_method.hpp"

namespace state_estimation
{

/// One IMU sample, already rotated into base_footprint by the node.
struct ImuMeasurement
{
  double yaw_rate{0.0};           ///< rad/s
  double yaw_rate_variance{0.0};  ///< (rad/s)^2
  /// Forward specific force, m/s^2, INCLUDING gravity (ICD-SNS-GNC).
  double forward_specific_force{0.0};
  double forward_specific_force_variance{0.0};  ///< (m/s^2)^2
  /// Pitch from the IMU's orientation (REP-103: positive = nose down), used
  /// to remove gravity. Unset when the IMU gives no orientation; the ground
  /// is then assumed level.
  std::optional<double> pitch;
  /// rad^2. Pitch noise becomes acceleration noise (g * cos(pitch))^2 *
  /// pitch_variance through gravity removal (0.02 rad is ~0.2 m/s^2).
  double pitch_variance{0.0};
};

/// Forward speed from wheel odometry (ICD-SNS-GNC: only twist.linear.x).
struct WheelSpeedMeasurement
{
  double speed{0.0};     ///< m/s, signed
  double variance{0.0};  ///< (m/s)^2
};

/// Parameters (spec §3.4). Defaults marked TBD are tuned in the sim.
struct StateEstimationConfig
{
  std::string motion_model{"bicycle"};
  std::string fusion_method{"ekf"};
  RobotFrameParams robot;

  double imu_timeout{0.1};         ///< s, inputs.imu.timeout
  double wheel_odom_timeout{0.2};  ///< s, inputs.wheel_odom.timeout

  /// Unmodelled forward acceleration, (m/s^2)^2/s, while IMU acceleration
  /// is used as input (covers tilt and bias errors). TBD.
  double accel_noise_density{0.05};
  /// Forward acceleration, (m/s^2)^2/s, when the IMU is stale and nothing
  /// is known about acceleration. TBD.
  double accel_noise_density_no_imu{4.0};
  /// Unmodelled change in yaw rate, (rad/s^2)^2/s. TBD.
  double yaw_accel_noise_density{1.0};

  /// Wheel speed minus predicted speed (m/s) beyond which the wheels are
  /// slipping, on top of 3 sigma of the predicted speed. TBD.
  double slip_threshold{0.3};
  /// Yaw rate (rad/s) beyond what the motion model allows at this speed
  /// that counts as a slide. Never triggers for the unicycle. TBD.
  double slip_yaw_rate_margin{0.2};
  /// Factor on wheel-speed variance while slipping. TBD.
  double slip_variance_scale{1.0e4};

  /// Initial covariance diagonal after reset: the pose is exact by
  /// definition (odom starts at the car), the velocities are not.
  double initial_pose_variance{1.0e-6};
  double initial_speed_variance{1.0};
  double initial_yaw_rate_variance{0.1};

  /// Factor on the covariance after recovering from a non-finite estimate.
  double recovery_covariance_scale{10.0};
};

/// Health values for /diagnostics (spec §5).
struct StateEstimationStatus
{
  /// Seconds of prediction since the last accepted sample; +inf before the
  /// first one.
  double imu_age{std::numeric_limits<double>::infinity()};
  double wheel_age{std::numeric_limits<double>::infinity()};
  bool imu_stale{true};
  bool wheel_stale{true};

  bool slip_detected{false};
  /// Variance used for the most recent wheel-speed update (inflated while
  /// slipping).
  double wheel_variance_used{0.0};

  int numerical_resets{0};       ///< recoveries from a non-finite estimate
  int rejected_measurements{0};  ///< samples with NaN/Inf or bad variance
};

class StateEstimationLogic
{
public:
  /// @throws std::invalid_argument for an invalid configuration.
  explicit StateEstimationLogic(const StateEstimationConfig & config = StateEstimationConfig{});

  /// Advance the estimate dt seconds (the 50 Hz timer).
  /// @throws std::invalid_argument if dt is negative or not finite.
  void predict(double dt);

  /// Fuse an IMU sample: updates yaw rate, and keeps its acceleration as the
  /// input for the following predict steps.
  void update_imu(const ImuMeasurement & imu);

  /// Fuse a wheel-speed sample, after checking for slip.
  void update_wheel_speed(const WheelSpeedMeasurement & wheel);

  /// Back to the odom origin with the initial covariance (`~/reset`).
  void reset();

  const RobotState & state() const {return state_;}
  const StateEstimationStatus & status() const {return status_;}
  const StateEstimationConfig & config() const {return config_;}

private:
  /// Restore the last good estimate if the current one is not finite.
  void guard_numerics();
  void update_staleness();

  StateEstimationConfig config_;
  std::unique_ptr<RobotFrame> frame_;
  std::unique_ptr<SensorFusionMethod> fusion_;

  RobotState state_;
  RobotState last_good_;
  StateEstimationStatus status_;

  double forward_accel_{0.0};           ///< gravity removed, from the last IMU sample
  double forward_accel_variance_{0.0};
};

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__STATE_ESTIMATION_LOGIC_HPP_
