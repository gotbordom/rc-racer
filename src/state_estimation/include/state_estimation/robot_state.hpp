// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Filter state for VEH.GNC.SE (spec §4): 2D pose in odom plus the body's
// forward and sideways speed and yaw rate, with full covariance. Pure C++,
// no ROS.

#ifndef STATE_ESTIMATION__ROBOT_STATE_HPP_
#define STATE_ESTIMATION__ROBOT_STATE_HPP_

#include <Eigen/Dense>

#include "state_estimation/robot_frame.hpp"

namespace state_estimation
{

/// Index of each state element in RobotState::mean and ::covariance.
enum StateIndex : int
{
  kX = 0,      ///< m, odom
  kY = 1,      ///< m, odom
  kYaw = 2,    ///< rad, odom, normalised to [-pi, pi)
  kSpeed = 3,  ///< forward speed v_x of base_footprint, body frame, m/s
  kYawRate = 4,  ///< yaw rate omega, rad/s
  /// Sideways (left) speed v_y of base_footprint, body frame, m/s. Without
  /// slip it is omega x the rear axle offset (spec §6.1, feature 2.5).
  kSideSpeed = 5,
  kStateSize = 6,
};

using StateVector = Eigen::Matrix<double, kStateSize, 1>;
using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;

struct RobotState
{
  StateVector mean{StateVector::Zero()};
  StateMatrix covariance{StateMatrix::Identity()};

  Pose2D pose() const {return {mean(kX), mean(kY), mean(kYaw)};}
  VelocityInputs velocity() const {return {mean(kSpeed), mean(kYawRate)};}

  /// True when every element of the mean and covariance is finite.
  bool is_finite() const {return mean.allFinite() && covariance.allFinite();}
};

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__ROBOT_STATE_HPP_
