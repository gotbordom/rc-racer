// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Unit tests for PinnedHold (VAL-019): the actions it returns.

#include <gtest/gtest.h>

#include "state_estimation/pinned_hold.hpp"

namespace state_estimation
{
namespace
{

constexpr double kDt = 0.02;
const PinnedParams kParams{0.3, 0.3, 10.0};

// Steps at `speed` with forward rate `rate`.
void run(SpeedHistory & h, int n, double speed, double rate = 0.0)
{
  for (int i = 0; i < n; ++i) {
    h.advance(kDt, rate, 0.0, 0.0, speed, 1e-4, 0.0, 1.0);
  }
}

TEST(PinnedHoldTest, PinsAfterAnImpactThatTookMostOfTheSpeed)
{
  SpeedHistory h(0.5);
  run(h, 20, 1.0);
  PinnedHold hold;
  hold.impact(h.elapsed(), 1.0);
  run(h, 10, 0.4);  // the leftover IMU-only speed until the IMU went quiet
  const PinnedAction a = hold.update(h, 0.4, true, kParams);
  EXPECT_TRUE(a.pinned);
  ASSERT_TRUE(a.speed.has_value());
  EXPECT_DOUBLE_EQ(*a.speed, 0.0);
  ASSERT_TRUE(a.take_back.has_value());
  EXPECT_NEAR(*a.take_back, 0.4 * 10 * kDt, 1e-9);  // driven since the impact
  // Held from then on, with nothing more to take back.
  run(h, 5, 0.0);
  const PinnedAction b = hold.update(h, 0.0, true, kParams);
  EXPECT_TRUE(b.pinned);
  EXPECT_FALSE(b.take_back.has_value());
}

TEST(PinnedHoldTest, ASmallDropOrANoisyImuDoesNotPin)
{
  SpeedHistory h(0.5);
  run(h, 20, 1.0);
  PinnedHold hold;
  hold.impact(h.elapsed(), 1.0);
  EXPECT_FALSE(hold.update(h, 0.8, true, kParams).pinned);   // only 20 % taken
  EXPECT_FALSE(hold.update(h, 0.2, false, kParams).pinned);  // IMU not quiet yet
}

TEST(PinnedHoldTest, NoImpactNoPin)
{
  SpeedHistory h(0.5);
  run(h, 20, 0.0);
  PinnedHold hold;
  EXPECT_FALSE(hold.update(h, 0.0, true, kParams).pinned);
}

TEST(PinnedHoldTest, PickingUpSpeedEndsTheHold)
{
  SpeedHistory h(0.5);
  run(h, 20, 1.0);
  PinnedHold hold;
  hold.impact(h.elapsed(), 1.0);
  ASSERT_TRUE(hold.update(h, 0.1, true, kParams).pinned);
  run(h, 20, 0.0, 1.0);  // 0.4 s at 1 m/s^2: 0.4 m/s picked up
  const PinnedAction a = hold.update(h, 0.0, false, kParams);
  EXPECT_FALSE(a.pinned);
  ASSERT_TRUE(a.speed.has_value());
  EXPECT_NEAR(*a.speed, 0.4, 1e-9);  // carry on from the IMU's speed
  EXPECT_FALSE(hold.after_impact());
}

TEST(PinnedHoldTest, TheBackstopEndsAHoldThatLastsTooLong)
{
  SpeedHistory h(0.5);
  run(h, 20, 1.0);
  PinnedHold hold;
  hold.impact(h.elapsed(), 1.0);
  ASSERT_TRUE(hold.update(h, 0.0, true, {0.3, 0.3, 1.0}).pinned);
  run(h, 60, 0.0);  // 1.2 s
  EXPECT_FALSE(hold.update(h, 0.0, true, {0.3, 0.3, 1.0}).pinned);
}

}  // namespace
}  // namespace state_estimation
