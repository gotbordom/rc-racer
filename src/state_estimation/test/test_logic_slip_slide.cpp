// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// L4 unit tests for the filter logic (VEH.GNC.SE spec §7): Slides: understeer
// and the yaw-rate check (REQ-GNC-005, VAL-004).
// Split from test_state_estimation_logic.cpp by topic (2.5).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

#include "logic_test_helpers.hpp"
#include "state_estimation/state_estimation_logic.hpp"

namespace state_estimation
{
namespace
{

// Drive a turn as the bicycle model has it, without slip: the IMU (at the
// default mount, 0.192 m ahead of the rear axle) reads yaw rate x speed plus
// lever x yaw acceleration sideways and -omega^2 x lever forward, and the
// wheel reads `wheel_factor` x the body speed. The yaw rate ramps from 0 to
// `omega` over `turn_in` s, then holds. Returns whether understeer was ever
// flagged.
bool turn(
  StateEstimationLogic & logic, double speed, double omega, double turn_in,
  double seconds, double wheel_factor = 1.0)
{
  const double lever = 0.192;
  bool flagged = false;
  double w = 0.0;
  for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {
    const double alpha = (i * kDt < turn_in) ? omega / turn_in : 0.0;
    for (int k = 0; k < 2; ++k) {
      w += alpha * 0.5 * kDt;
      // Ahead of the rear axle the IMU also reads -omega^2 x lever forward.
      logic.update_imu(turn_sample(w, w * speed + lever * alpha, -w * w * lever));
    }
    logic.predict(kDt);
    logic.update_wheel_speed(wheel_sample(wheel_factor * speed));
    flagged = flagged || logic.status().understeer_detected;
  }
  return flagged;
}

TEST(StateEstimationLogicTest, NoUndersteerInCleanTurns)
{
  // Turns the kinematics allow, both ways, forward and in reverse, with a
  // sharp turn-in (large lever x yaw acceleration): never understeer.
  for (double speed : {1.0, 2.0, 3.0, -1.0}) {
    // 90 % of the steering limit at this speed (0.52 rad, 0.384 m wheelbase)
    const double limit = 0.9 * std::abs(speed) * std::tan(0.52) / 0.384;
    for (double omega : {limit, -limit}) {
      StateEstimationLogic logic;
      for (int i = 0; i < 50; ++i) {
        cycle(logic, imu_sample(0.0), speed);
      }
      EXPECT_FALSE(turn(logic, speed, omega, 0.2, 3.0))
        << "speed " << speed << " omega " << omega;
      EXPECT_EQ(logic.status().slip_events, 0) << "speed " << speed << " omega " << omega;
    }
  }
}

TEST(StateEstimationLogicTest, UndersteerIsDetected)
{
  // VAL-004: sliding wide at 2 rad/s, the body moves forward at 2.6 m/s while
  // the driven wheel reads 3.0. The lateral acceleration (5.2 m/s^2) falls
  // short of what the wheel implies (6.0).
  StateEstimationLogic logic;
  for (int i = 0; i < 50; ++i) {
    cycle(logic, imu_sample(0.0), 2.6);
  }
  ASSERT_FALSE(turn(logic, 2.6, 2.0, 0.3, 1.0));  // grip: no understeer
  EXPECT_TRUE(turn(logic, 2.6, 2.0, 0.02, 1.0, 3.0 / 2.6));
  EXPECT_GE(logic.status().slip_events, 1);
}

TEST(StateEstimationLogicTest, BicycleFlagsYawRateTheSteeringCannotMake)
{
  // Spinning in place at 1 rad/s: impossible for an Ackermann car, so it is
  // sliding. A differential-drive robot does this normally.
  StateEstimationConfig bicycle_cfg;
  bicycle_cfg.motion_model = "bicycle";
  StateEstimationConfig unicycle_cfg;
  unicycle_cfg.motion_model = "unicycle";
  StateEstimationLogic bicycle(bicycle_cfg);
  StateEstimationLogic unicycle(unicycle_cfg);

  for (int i = 0; i < 10; ++i) {
    cycle(bicycle, imu_sample(1.0), 0.0);
    cycle(unicycle, imu_sample(1.0), 0.0);
  }
  EXPECT_TRUE(bicycle.status().slip_detected);
  EXPECT_FALSE(unicycle.status().slip_detected);
  // The heading still follows the gyro in both.
  EXPECT_NEAR(bicycle.state().mean(kYaw), unicycle.state().mean(kYaw), 1e-6);
  EXPECT_GT(bicycle.state().mean(kYaw), 0.15);
}

TEST(StateEstimationLogicTest, NormalTurnIsNotASlide)
{
  // 1 m/s at 1 rad/s is inside the steering limit (max ~1.49 rad/s).
  StateEstimationLogic logic;
  for (int i = 0; i < 50; ++i) {
    cycle(logic, imu_sample(1.0), 1.0);
    EXPECT_FALSE(logic.status().slip_detected) << "cycle " << i;
  }
}

}  // namespace
}  // namespace state_estimation
