// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// L4 unit tests for the filter logic (VEH.GNC.SE spec §7): Wheel spin: the
// windowed check, stale history, early distrust, the retroactive correction
// (REQ-GNC-005, VAL-005, VAL-017).
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

// A launch as in the L2 sim loop (2026-11-24), with the sim's sensor noise:
// the body accelerates at `accel` to 1 m/s while the wheel leads it by
// `lead`, then both cruise at 1 m/s for 2 s.
struct LaunchRun
{
  int slip_events_in_launch;
  int slip_events_in_cruise;
  double max_cruise_speed;   // from 0.2 s into the cruise
  double final_speed;
  double max_position_error;
  bool always_inside_3_sigma;
};

LaunchRun noisy_launch(double accel, double lead, int seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> gyro(0.0, kGyroStd);
  std::normal_distribution<double> accel_noise(0.0, kAccelStd);
  std::normal_distribution<double> wheel_noise(0.0, kWheelStd);
  StateEstimationLogic logic;
  double true_x = 0.0;
  LaunchRun run{0, 0, 0.0, 0.0, 0.0, true};
  auto step = [&](double a, double v_body, double v_wheel) {
      true_x += v_body * kDt;
      logic.predict(kDt);
      logic.update_imu(imu_sample(gyro(rng), a + accel_noise(rng)));
      logic.update_imu(imu_sample(gyro(rng), a + accel_noise(rng)));
      logic.update_wheel_speed(wheel_sample(v_wheel + wheel_noise(rng)));
      const RobotState e = logic.estimate();
      const double error = std::abs(e.mean(kX) - true_x);
      run.max_position_error = std::max(run.max_position_error, error);
      // + (2 cm)^2 for comparing at a 20 ms step, as the L2 test allows
      run.always_inside_3_sigma = run.always_inside_3_sigma &&
        error <= 3.0 * std::sqrt(e.covariance(kX, kX) + 4e-4);
    };
  for (int i = 0; i < 50; ++i) {
    step(0.0, 0.0, 0.0);
  }
  double body = 0.0;
  while (body < 1.0) {
    const double a = std::min(accel, (1.0 - body) / kDt);
    body += a * kDt;
    step(a, body, std::min(1.0, body + lead));
  }
  run.slip_events_in_launch = logic.status().slip_events;
  for (int i = 0; i < 100; ++i) {
    step(0.0, 1.0, 1.0);
    if (i >= 10) {
      run.max_cruise_speed = std::max(run.max_cruise_speed, logic.state().mean(kSpeed));
    }
  }
  run.slip_events_in_cruise = logic.status().slip_events - run.slip_events_in_launch;
  run.final_speed = logic.state().mean(kSpeed);
  return run;
}

TEST(StateEstimationLogicTest, WheelSpinIsDetectedAndDistrusted)
{
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
  }
  ASSERT_FALSE(logic.status().slip_detected);
  EXPECT_DOUBLE_EQ(logic.status().wheel_variance_used, kWheelStd * kWheelStd);

  // Wheels spin up to 3 m/s while the IMU says the car is not accelerating.
  for (int i = 0; i < 25; ++i) {
    cycle(logic, imu_sample(0.0), 3.0);
    EXPECT_TRUE(logic.status().slip_detected) << "cycle " << i;
  }
  EXPECT_DOUBLE_EQ(
    logic.status().wheel_variance_used,
    kWheelStd * kWheelStd * logic.config().slip_variance_scale);
  EXPECT_NEAR(logic.state().mean(kSpeed), 1.0, 0.2);

  // Grip again: once the wheels have agreed with the estimate for a full
  // slip window (hysteresis), slip clears.
  for (int i = 0; i < 30; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
  }
  EXPECT_FALSE(logic.status().slip_detected);
  EXPECT_DOUBLE_EQ(logic.status().wheel_variance_used, kWheelStd * kWheelStd);
}

TEST(StateEstimationLogicTest, ImuSamplesBetweenPredictsAreAveraged)
{
  // Regression (2026-11-24, L2 sim loop): predict used only the latest IMU
  // acceleration, and the IMU runs at twice the predict rate, so every other
  // sample was dropped. A short real jolt (+/-30 m/s^2 for 10 ms each way, as
  // the wheels regain grip after a launch slip) could lose its second half:
  // +0.3 m/s on a speed that, while slipping, follows the IMU alone.
  // Here the two samples per cycle are +2 then 0 m/s^2: 1 m/s^2 on average.
  StateEstimationLogic logic;
  logic.update_wheel_speed(wheel_sample(0.0));
  for (int i = 0; i < 50; ++i) {  // 1 s, no wheel odometry
    logic.predict(kDt);
    logic.update_imu(imu_sample(0.0, 2.0));
    logic.update_imu(imu_sample(0.0, 0.0));
  }
  logic.predict(kDt);  // applies the last cycle's samples
  EXPECT_NEAR(logic.state().mean(kSpeed), 1.0, 0.03);
}

TEST(StateEstimationLogicTest, StaleHistoryAfterUndetectedSlipIsNotSlip)
{
  // Regression (2026-11-24, L2 sim loop, ~1 run in 3): the wheel led the
  // body by just under the slip threshold during the launch, so the
  // estimate followed it undetected. Half a window later the windowed check
  // built its reference on that contaminated estimate, blamed the now-correct
  // wheel, and "corrected" the speed 0.33 m/s too high for 2 s. The old
  // check did this in 34 of 200 seeds at a 0.26 m/s lead.
  for (double lead : {0.26, 0.27}) {
    for (int seed = 1; seed <= 50; ++seed) {
      const LaunchRun run = noisy_launch(1.9, lead, seed);
      EXPECT_EQ(run.slip_events_in_cruise, 0) << "lead " << lead << " seed " << seed;
      EXPECT_LT(run.max_cruise_speed, 1.1) << "lead " << lead << " seed " << seed;
    }
  }
}

TEST(StateEstimationLogicTest, CaughtLaunchSlipCorrectsThePositionBack)
{
  // VAL-017 (#9): until the windowed check fires, the estimate follows the
  // spinning wheel and runs ahead. On detection the position is moved back
  // by what the wheel added over the window, and the covariance covers the
  // rest, so the true error stays inside 3 sigma.
  for (int seed = 1; seed <= 20; ++seed) {
    const LaunchRun run = noisy_launch(1.7, 0.6, seed);
    ASSERT_GE(run.slip_events_in_launch, 1) << "seed " << seed;
    EXPECT_TRUE(run.always_inside_3_sigma) << "seed " << seed;
    EXPECT_LT(run.max_position_error, 0.08) << "seed " << seed;
    EXPECT_NEAR(run.final_speed, 1.0, 0.05) << "seed " << seed;
  }
}

TEST(StateEstimationLogicTest, SlowlyBuildingSlipIsStillDetected)
{
  // Guard for the stale-history handling: a slip that builds over seconds
  // (understeer, VAL-004: the wheel drifting ahead of the body) must not be
  // mistaken for stale history. A first redesign (2026-11-24) cross-checked
  // against half the window and stopped detecting exactly this.
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
  }
  // 0.8 m/s per s: 0.4 m/s within one 0.5 s window, over the threshold.
  // (Slower drift stays under it within any one window, so the windowed
  // check cannot see it at all: a known limit, VAL-018.)
  int detected_at = -1;
  for (int i = 0; i < 150 && detected_at < 0; ++i) {
    cycle(logic, imu_sample(0.0), 1.0 + 0.8 * i * kDt);
    if (logic.status().slip_detected) {
      detected_at = i;
    }
  }
  ASSERT_GE(detected_at, 0) << "never detected";
  EXPECT_LT(detected_at * kDt, 1.0);
}

TEST(StateEstimationLogicTest, RetroactiveCorrectionKeepsOdomContinuous)
{
  // The correction on detecting a slip is spread over slip_correction_time:
  // odom must not jump (REP-105). A one-step correction moved it 7-8 cm in
  // the VAL-005 and VAL-017 sim scenarios.
  StateEstimationLogic logic;
  for (int i = 0; i < 50; ++i) {
    cycle(logic, imu_sample(0.0), 0.0);
  }
  double body = 0.0;
  double max_jump = 0.0;
  double last_x = logic.state().mean(kX);
  for (int i = 0; i < 150; ++i) {
    const double a = body < 1.0 ? std::min(1.7, (1.0 - body) / kDt) : 0.0;
    body += a * kDt;
    cycle(logic, imu_sample(0.0, a), std::min(1.0, body + 0.6));
    const double x = logic.state().mean(kX);
    max_jump = std::max(max_jump, std::abs(x - last_x) - logic.state().mean(kSpeed) * kDt);
    last_x = x;
  }
  ASSERT_GE(logic.status().slip_events, 1);
  EXPECT_LT(max_jump, 0.01);
}

TEST(StateEstimationLogicTest, DevelopingSpinIsDistrustedBeforeDetection)
{
  // VAL-017 (#9): before the windowed check decides, the estimate used to
  // follow a spinning wheel almost fully. Now the wheel's variance grows with
  // how far it out-accelerates the IMU, so the estimate stays near the body.
  StateEstimationLogic logic;
  for (int i = 0; i < 50; ++i) {
    cycle(logic, imu_sample(0.0), 0.0);
  }
  double body = 0.0;
  double wheel = 0.0;
  double max_gap = 0.0;
  while (body < 1.0) {  // the body at 1.9 m/s^2, the wheel spinning up at 3
    const double a = std::min(1.9, (1.0 - body) / kDt);
    body += a * kDt;
    wheel = std::min(1.0, wheel + 3.0 * kDt);
    cycle(logic, imu_sample(0.0, a), wheel);
    max_gap = std::max(max_gap, logic.state().mean(kSpeed) - body);
  }
  EXPECT_LT(max_gap, 0.12);  // following the wheel, it was ~0.3
}

TEST(StateEstimationLogicTest, MatchedHardAccelerationKeepsTheWheelTrusted)
{
  // The early distrust is about a mismatch, not acceleration: wheel and IMU
  // agreeing at 2.5 m/s^2 leave the wheel variance as it is.
  StateEstimationLogic logic;
  for (int i = 0; i < 50; ++i) {
    cycle(logic, imu_sample(0.0), 0.0);
  }
  for (int i = 1; i <= 60; ++i) {
    cycle(logic, imu_sample(0.0, 2.5), 2.5 * i * kDt);
    ASSERT_DOUBLE_EQ(logic.status().wheel_variance_used, kWheelStd * kWheelStd) << "cycle " << i;
  }
}

TEST(StateEstimationLogicTest, SlipEndingKeepsOdomContinuous)
{
  // The first trusted wheel sample after an IMU-only stretch moves the
  // position too, through its correlation with the speed. Spread out, not
  // in one step (2.5: 0.16 m at once in VAL-004).
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
  }
  for (int i = 0; i < 75; ++i) {  // 1.5 s of wheel spin: IMU-only, which
    cycle(logic, imu_sample(0.0, 0.2), 2.5);  // drifts (a 0.2 m/s^2 bias)
  }
  ASSERT_TRUE(logic.status().slip_detected);
  double max_jump = 0.0;
  double last_x = logic.state().mean(kX);
  for (int i = 0; i < 100; ++i) {  // grip again: the slip ends
    cycle(logic, imu_sample(0.0), 1.0);
    const double x = logic.state().mean(kX);
    max_jump = std::max(max_jump, std::abs(x - last_x - logic.state().mean(kSpeed) * kDt));
    last_x = x;
  }
  ASSERT_FALSE(logic.status().slip_detected);
  // The scenarios' continuity limit; at once it was 0.25 m. Spread over
  // 0.5 s, the ~0.25 m correction is ~1 cm per step on its own.
  EXPECT_LT(max_jump, 0.05);
}

TEST(StateEstimationLogicTest, LaunchSlipIsCaughtOnceAndRecovers)
{
  // A launch slip that is caught late and ends soon after: one slip event,
  // and the speed settles on the true 1 m/s without a second, false slip.
  StateEstimationLogic logic;
  for (int i = 0; i < 50; ++i) {  // at rest, settled
    cycle(logic, imu_sample(0.0), 0.0);
  }
  // Launch: the body accelerates at 2 m/s^2 with the wheel spinning 0.45 m/s
  // ahead of it, until both reach 1 m/s. The estimate follows the wheel part
  // of the way before the windowed check catches it; the slip then ends
  // soon after, while those wheel-following samples are still in the window.
  double body = 0.0;
  while (body < 1.0) {
    const double accel = std::min(2.0, (1.0 - body) / kDt);
    body += accel * kDt;
    cycle(logic, imu_sample(0.0, accel), std::min(1.0, body + 0.45));
  }
  ASSERT_EQ(logic.status().slip_events, 1u);
  // Cruise at 1 m/s, wheel and IMU agreeing, for 2 s.
  double max_speed = 0.0;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
    if (i > 15) {
      max_speed = std::max(max_speed, logic.state().mean(kSpeed));
    }
  }
  EXPECT_EQ(logic.status().slip_events, 1u) << "a second, false slip";
  EXPECT_FALSE(logic.status().slip_detected);
  EXPECT_NEAR(logic.state().mean(kSpeed), 1.0, 0.05);
  EXPECT_LT(max_speed, 1.1);
}

TEST(StateEstimationLogicTest, HardAccelerationIsNotSlip)
{
  // 0 -> 3 m/s in 1 s and a hard stop, with wheels and IMU agreeing (sim
  // noise): the windowed check must not fire.
  std::mt19937 rng(7);
  std::normal_distribution<double> accel_noise(0.0, kAccelStd);
  std::normal_distribution<double> wheel_noise(0.0, kWheelStd);
  StateEstimationLogic logic;
  double v = 0.0;
  for (int i = 0; i < 200; ++i) {
    const double t = i * kDt;
    const double a = t < 1.0 ? 3.0 : (t < 2.0 ? 0.0 : (t < 2.5 ? -6.0 : 0.0));
    v = std::max(0.0, v + a * kDt);
    cycle(logic, imu_sample(0.0, a + accel_noise(rng)), v + wheel_noise(rng));
    ASSERT_FALSE(logic.status().slip_detected) << "t " << t;
  }
  EXPECT_EQ(logic.status().slip_events, 0);
}

TEST(StateEstimationLogicTest, NoWindowedSlipCheckWithoutImu)
{
  // With the IMU stale the acceleration input is 0, so the IMU reference
  // would call any real speed change slip. The windowed check is skipped.
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {  // 0 -> 2 m/s in 2 s, wheels only
    logic.predict(kDt);
    logic.update_wheel_speed(wheel_sample(0.02 * i));
    ASSERT_FALSE(logic.status().slip_detected) << "cycle " << i;
  }
}

TEST(StateEstimationLogicTest, SwitchedOnWhileMovingIsNotSlip)
{
  // Started at 1 m/s with no acceleration: neither the one-step nor the
  // windowed check may call the first second of readings slip (the initial
  // speed guess is uncertain), and the estimate must settle on the wheels.
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
    ASSERT_FALSE(logic.status().slip_detected) << "cycle " << i;
  }
  EXPECT_NEAR(logic.state().mean(kSpeed), 1.0, 0.02);
}

TEST(StateEstimationLogicTest, FirstWheelReadingWhileMovingIsNotSlip)
{
  // Started while already driving: the filter's speed is uncertain, so a
  // large first residual is not evidence of slip.
  StateEstimationLogic logic;
  logic.update_wheel_speed(wheel_sample(2.0));
  EXPECT_FALSE(logic.status().slip_detected);
  EXPECT_NEAR(logic.state().mean(kSpeed), 2.0, 0.01);
}

}  // namespace
}  // namespace state_estimation
