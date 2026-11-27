// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Unit tests for turn_speed() (feature 2.5 step 2b): forward speed from a
// steady turn, v_x ~ a_y / omega.

#include <gtest/gtest.h>

#include <cmath>

#include "state_estimation/turn_speed.hpp"

namespace state_estimation
{
namespace
{

constexpr double kDt = 0.02;
const TurnSpeedParams kParams{1.0, 0.5, 0.1, 0.05};

// 1 s of history, the yaw rate ramping from w0 to w1, lateral acceleration
// yaw rate x v (a steady slide at forward speed v).
SpeedHistory turning(double v, double w0, double w1)
{
  SpeedHistory h(0.5);
  for (int i = 0; i < 50; ++i) {
    const double w = w0 + (w1 - w0) * i / 49.0;
    h.advance(kDt, 0.0, w * v, 3.0 * w, 3.0, 1e-4, w, 3.0);  // the wheel says 3.0
  }
  return h;
}

TEST(TurnSpeedTest, SteadyTurnGivesTheBodySpeed)
{
  const SpeedHistory h = turning(2.6, 2.0, 2.0);
  const auto turn = turn_speed(h, *h.reference(0.5), 2.0, kParams);
  ASSERT_TRUE(turn.has_value());
  EXPECT_NEAR(turn->speed, 2.6, 1e-6);  // not the wheel's 3.0
  EXPECT_NEAR(std::sqrt(turn->variance), 0.1 + 0.05 * 2.6, 1e-9);
}

TEST(TurnSpeedTest, BothWaysAndInReverse)
{
  const SpeedHistory right = turning(2.6, -2.0, -2.0);
  EXPECT_NEAR(turn_speed(right, *right.reference(0.5), -2.0, kParams)->speed, 2.6, 1e-6);
  const SpeedHistory reverse = turning(-1.5, 1.5, 1.5);
  EXPECT_NEAR(turn_speed(reverse, *reverse.reference(0.5), 1.5, kParams)->speed, -1.5, 1e-6);
}

TEST(TurnSpeedTest, NotOfferedForAGentleTurn)
{
  const SpeedHistory h = turning(2.6, 0.6, 0.6);
  EXPECT_FALSE(turn_speed(h, *h.reference(0.5), 0.6, kParams).has_value());
}

TEST(TurnSpeedTest, NotOfferedWhileTheTurnTightensOrOpens)
{
  // Turn-in: the steady-slide assumption doesn't hold.
  const SpeedHistory h = turning(2.6, 1.2, 2.4);
  EXPECT_FALSE(turn_speed(h, *h.reference(0.5), 2.4, kParams).has_value());
}

}  // namespace
}  // namespace state_estimation
