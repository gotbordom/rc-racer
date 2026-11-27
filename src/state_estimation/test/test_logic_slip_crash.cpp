// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// L4 unit tests for the filter logic (VEH.GNC.SE spec §7): Crashes: impact,
// pinned, and the safety nets that end a slip (REQ-GNC-005, VAL-007, VAL-019).
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

TEST(StateEstimationLogicTest, PinnedCarEstimateStopsWithIt)
{
  // VAL-019 (#14): into the bench at 1 m/s, then pushing against it for 4 s
  // with the wheels spinning at 1 m/s, then the drive stops. The IMU shows
  // the impact (-50 m/s^2 for one step, as the sim does since DEV-07) and is
  // quiet after. The estimate must stop with the car, not drive on when the
  // 2 s max_duration release comes.
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
  }
  const double stop_x = logic.state().mean(kX);
  cycle(logic, imu_sample(0.0, -50.0), 1.0);  // impact: -1 m/s in 20 ms
  bool pinned = false;
  for (int i = 0; i < 200; ++i) {  // pushing, 4 s
    cycle(logic, imu_sample(0.0), 1.0);
    pinned = pinned || logic.status().pinned;
  }
  EXPECT_TRUE(pinned);
  EXPECT_TRUE(logic.status().slip_detected);
  EXPECT_NEAR(logic.state().mean(kX), stop_x, 0.1);
  for (int i = 0; i < 25; ++i) {  // the drive stops: wheels spin down
    cycle(logic, imu_sample(0.0), 1.0 - 0.04 * (i + 1));
  }
  for (int i = 0; i < 50; ++i) {
    cycle(logic, imu_sample(0.0), 0.0);
  }
  EXPECT_FALSE(logic.status().slip_detected);
  EXPECT_FALSE(logic.status().pinned);
  EXPECT_NEAR(logic.state().mean(kSpeed), 0.0, 0.05);
  EXPECT_NEAR(logic.state().mean(kX), stop_x, 0.15);
}

TEST(StateEstimationLogicTest, ImpactThatLeavesSomeSpeedStillPins)
{
  // VAL-007 (2026-11-24): the IMU-only speed came out of the impact at
  // 0.37 m/s (the car crept on into the bench), over a fixed 0.3 m/s pinned
  // limit, so the estimate drove on when max_duration released the slip.
  // The impact took 63 % of the speed: that is what pins now.
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
  }
  const double stop_x = logic.state().mean(kX);
  cycle(logic, imu_sample(0.0, -31.5), 1.0);  // -0.63 m/s
  bool pinned = false;
  for (int i = 0; i < 200; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
    pinned = pinned || logic.status().pinned;
  }
  EXPECT_TRUE(pinned);
  EXPECT_NEAR(logic.state().mean(kX), stop_x, 0.15);
}

TEST(StateEstimationLogicTest, ImpactTheImuOnlyPartlySawStillPins)
{
  // VAL-007, another run (2026-11-24): the IMU-only speed came out of the
  // impact at 0.54 m/s, 46 % of the speed taken (runs so far: 46-87 %, the
  // true loss ~80-100 %). The signature is what counts: a big impact, then
  // the wheel still at its old speed and the IMU quiet.
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
  }
  const double stop_x = logic.state().mean(kX);
  cycle(logic, imu_sample(0.0, -23.0), 1.0);  // -0.46 m/s
  for (int i = 0; i < 25; ++i) {  // 0.5 s settling: the IMU not yet quiet
    cycle(logic, imu_sample(0.0, i % 2 ? 0.5 : -0.5), 1.0);
  }
  bool pinned = false;
  for (int i = 0; i < 200; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
    pinned = pinned || logic.status().pinned;
  }
  EXPECT_TRUE(pinned);
  // The ~0.27 m driven on the leftover speed until the IMU went quiet is
  // taken back.
  EXPECT_NEAR(logic.state().mean(kX), stop_x, 0.05);
}

TEST(StateEstimationLogicTest, PinnedCarThatDrivesAwayIsFollowed)
{
  // After pinning, the car backs off the obstacle and drives on: the IMU
  // shows it picking up speed, which ends the pinned hold.
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
  }
  cycle(logic, imu_sample(0.0, -50.0), 1.0);
  for (int i = 0; i < 50; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
  }
  ASSERT_TRUE(logic.status().pinned);
  double v = 0.0;
  for (int i = 0; i < 50; ++i) {  // 1 m/s^2 for 1 s, the wheel matching
    v += 1.0 * kDt;
    cycle(logic, imu_sample(0.0, 1.0), std::max(1.0, v));
  }
  EXPECT_FALSE(logic.status().pinned);
  EXPECT_NEAR(logic.state().mean(kSpeed), 1.0, 0.15);
}

TEST(StateEstimationLogicTest, BumpIsNotAPinnedStop)
{
  // A sharp jolt back and forward (a curb, a rut) with no net speed loss:
  // the IMU spikes past the impact level but the car carries on. Never
  // pinned, and the estimate keeps the speed.
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
  }
  const double x0 = logic.state().mean(kX);
  cycle(logic, imu_sample(0.0, -20.0), 1.0);
  cycle(logic, imu_sample(0.0, 20.0), 1.0);
  for (int i = 0; i < 150; ++i) {
    cycle(logic, imu_sample(0.0), 1.0);
    ASSERT_FALSE(logic.status().pinned) << "cycle " << i;
  }
  EXPECT_NEAR(logic.state().mean(kSpeed), 1.0, 0.05);
  EXPECT_NEAR(logic.state().mean(kX) - x0, 152 * kDt, 0.1);
}

TEST(StateEstimationLogicTest, CrashSpinDownIsDetected)
{
  // Regression from a manual sim test (turbo into a bench): the body stops
  // in 0.4 s (IMU -7.5 m/s^2) while the wheels spin down over 1 s. The
  // one-step check missed it (-7.5 m/s^2 is 0.15 m/s per step, and each
  // update re-anchored the estimate to the wheels), so the estimate read
  // 2 m/s with the car stopped. The windowed check must catch it.
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {  // 2 s steady at 3 m/s
    cycle(logic, imu_sample(0.0), 3.0);
  }
  ASSERT_FALSE(logic.status().slip_detected);

  bool detected = false;
  for (int i = 1; i <= 50; ++i) {  // the crash, 1 s
    const double t = i * kDt;
    const double body_accel = t <= 0.4 ? -7.5 : 0.0;
    const double wheel = std::max(0.0, 3.0 * (1.0 - t / 1.0));
    cycle(logic, imu_sample(0.0, body_accel), wheel);
    detected = detected || logic.status().slip_detected;
    if (t >= 0.5) {
      // Body stopped at 0.4 s; the wheels still read 1.5 m/s at 0.5 s.
      EXPECT_LT(logic.state().mean(kSpeed), 0.5) << "t " << t;
    }
  }
  EXPECT_TRUE(detected);
  EXPECT_EQ(logic.status().slip_events, 1);

  for (int i = 0; i < 50; ++i) {  // stopped, wheels agree again
    cycle(logic, imu_sample(0.0), 0.0);
  }
  EXPECT_FALSE(logic.status().slip_detected);
  EXPECT_NEAR(logic.state().mean(kSpeed), 0.0, 0.05);
  EXPECT_EQ(logic.status().slip_events, 1);
}

TEST(StateEstimationLogicTest, CrashTheImuMissedEndsWhenStopped)
{
  // Regression from a manual sim test: the IMU did not register the crash
  // (a short impulse a 100 Hz IMU can miss, or a saturated one), so the
  // IMU-only speed stayed at 2.5 m/s while the car sat stopped against the
  // bench, and slip never ended. Wheel ~0 with a quiet IMU means stopped.
  StateEstimationLogic logic;
  for (int i = 0; i < 100; ++i) {
    cycle(logic, imu_sample(0.0), 3.0);
  }
  for (int i = 1; i <= 25; ++i) {  // wheel 3 -> 0 in 0.5 s, IMU sees nothing
    cycle(logic, imu_sample(0.0), std::max(0.0, 3.0 * (1.0 - i * kDt / 0.5)));
  }
  EXPECT_EQ(logic.status().slip_events, 1);
  for (int i = 0; i < 10; ++i) {  // 0.2 s stopped
    cycle(logic, imu_sample(0.0), 0.0);
  }
  EXPECT_FALSE(logic.status().slip_detected);
  EXPECT_NEAR(logic.state().mean(kSpeed), 0.0, 0.05);
  for (int i = 0; i < 100; ++i) {  // and it stays settled: no new events
    cycle(logic, imu_sample(0.0), 0.0);
  }
  EXPECT_EQ(logic.status().slip_events, 1);
}

TEST(StateEstimationLogicTest, ImuOnlyStretchIsBounded)
{
  // Wheels spinning at 3 m/s with a quiet IMU (stuck, or an IMU that missed
  // a launch): after slip.max_duration the wheel is trusted again, rather
  // than running on IMU-only speed for ever.
  StateEstimationLogic logic;
  for (int i = 0; i < 50; ++i) {
    cycle(logic, imu_sample(0.0), 0.0);
  }
  cycle(logic, imu_sample(0.0), 3.0);
  ASSERT_TRUE(logic.status().slip_detected);
  int cycles = 1;
  while (logic.status().slip_detected && cycles < 500) {
    cycle(logic, imu_sample(0.0), 3.0);
    ++cycles;
  }
  EXPECT_NEAR(cycles * kDt, logic.config().slip_max_duration, 0.1);
  for (int i = 0; i < 50; ++i) {
    cycle(logic, imu_sample(0.0), 3.0);
  }
  EXPECT_FALSE(logic.status().slip_detected);
  EXPECT_NEAR(logic.state().mean(kSpeed), 3.0, 0.05);
  EXPECT_EQ(logic.status().slip_events, 1);
}

}  // namespace
}  // namespace state_estimation
