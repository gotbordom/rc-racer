// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Shared helpers for the StateEstimationLogic tests (test_state_estimation_logic.cpp
// and test_logic_*.cpp): sensor samples with the sim's noise levels and one
// 50 Hz cycle as the node runs it.

#ifndef LOGIC_TEST_HELPERS_HPP_
#define LOGIC_TEST_HELPERS_HPP_

#include "state_estimation/state_estimation_logic.hpp"

namespace state_estimation
{

constexpr double kDt = 0.02;          // 50 Hz predict
constexpr double kGyroStd = 0.005;    // rad/s, config/sensor_noise.yaml
constexpr double kAccelStd = 0.05;    // m/s^2
constexpr double kWheelStd = 0.02;    // m/s

inline ImuMeasurement imu_sample(double yaw_rate, double forward_specific_force = 0.0)
{
  ImuMeasurement imu;
  imu.yaw_rate = yaw_rate;
  imu.yaw_rate_variance = kGyroStd * kGyroStd;
  imu.forward_specific_force = forward_specific_force;
  imu.forward_specific_force_variance = kAccelStd * kAccelStd;
  return imu;
}

inline WheelSpeedMeasurement wheel_sample(double speed) {return {speed, kWheelStd * kWheelStd};}

// An IMU sample in a turn: lateral (left) specific force too.
inline ImuMeasurement turn_sample(double yaw_rate, double lateral, double forward = 0.0)
{
  ImuMeasurement imu = imu_sample(yaw_rate, forward);
  imu.lateral_specific_force = lateral;
  imu.lateral_specific_force_variance = kAccelStd * kAccelStd;
  return imu;
}

// One 50 Hz cycle as the node runs it: predict, two IMU samples (100 Hz),
// one wheel sample.
inline void cycle(StateEstimationLogic & logic, const ImuMeasurement & imu, double wheel_speed)
{
  logic.predict(kDt);
  logic.update_imu(imu);
  logic.update_imu(imu);
  logic.update_wheel_speed(wheel_sample(wheel_speed));
}

inline bool covariance_is_valid(const StateMatrix & p)
{
  if (!p.isApprox(p.transpose(), 1e-12)) {
    return false;
  }
  const Eigen::SelfAdjointEigenSolver<StateMatrix> eig(p);
  return eig.eigenvalues().minCoeff() > -1e-12;
}

}  // namespace state_estimation

#endif  // LOGIC_TEST_HELPERS_HPP_
