// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// L4 unit tests for the filter logic (VEH.GNC.SE spec §7): Braking slides
// (REQ-GNC-005, VAL-006).
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

// Cruise at 3 m/s, then brake to a stop: the body slows at `body_decel`,
// the wheel at `wheel_decel`, with the sim's sensor noise. Returns the time
// from the start of braking to the first slip (or -1), and the slip count.
std::pair<double, int> braking(double body_decel, double wheel_decel, int seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> gyro(0.0, kGyroStd);
  std::normal_distribution<double> accel_noise(0.0, kAccelStd);
  std::normal_distribution<double> wheel_noise(0.0, kWheelStd);
  StateEstimationLogic logic;
  auto step = [&](double a, double v_wheel) {
      logic.predict(kDt);
      logic.update_imu(imu_sample(gyro(rng), a + accel_noise(rng)));
      logic.update_imu(imu_sample(gyro(rng), a + accel_noise(rng)));
      logic.update_wheel_speed(wheel_sample(v_wheel + wheel_noise(rng)));
    };
  for (int i = 0; i < 100; ++i) {
    step(0.0, 3.0);
  }
  double body = 3.0;
  double wheel = 3.0;
  double detected = -1.0;
  for (int i = 0; body > 0.0 || i < 150; ++i) {
    const double a = body > 0.0 ? -std::min(body_decel, body / kDt) : 0.0;
    body += a * kDt;
    wheel = std::max(0.0, wheel - wheel_decel * kDt);
    step(a, wheel);
    if (detected < 0.0 && logic.status().slip_detected) {
      detected = (i + 1) * kDt;
    }
  }
  return {detected, logic.status().slip_events};
}

TEST(StateEstimationLogicTest, BrakingSlideIsDetectedWithinHalfASecond)
{
  // VAL-006: braking from 3 m/s on mu 0.5, the drive slows the wheel at
  // 3 m/s^2 while the car manages ~2.5: the car slides on faster than its
  // wheels. It must be caught within 0.5 s, as a launch slip is.
  for (int seed = 1; seed <= 20; ++seed) {
    const auto [detected, events] = braking(2.5, 3.0, seed);
    EXPECT_GE(detected, 0.0) << "seed " << seed << ": not detected";
    EXPECT_LE(detected, 0.5) << "seed " << seed;
  }
}

TEST(StateEstimationLogicTest, HardStopIsNotABrakingSlide)
{
  // VAL-001 at mu 1.0: wheel and car slow together at 3 m/s^2. No slip.
  for (int seed = 1; seed <= 50; ++seed) {
    EXPECT_EQ(braking(3.0, 3.0, seed).second, 0) << "seed " << seed;
  }
}

}  // namespace
}  // namespace state_estimation
