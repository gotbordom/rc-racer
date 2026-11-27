// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Unit tests for the slip checks (slip_checks.hpp), each on a hand-built
// SpeedHistory: what each decides, without the estimator around it.

#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include "state_estimation/slip_checks.hpp"

namespace state_estimation
{
namespace
{

constexpr double kDt = 0.02;
constexpr double kWindow = 0.5;
const WindowCheckParams kWindowParams{0.3, 0.5, 0.15};

// 1 s of history: the estimate at `speed`, the forward rate `rate`.
SpeedHistory steady(double speed, double rate, double yaw_rate = 0.0, double lateral = 0.0)
{
  SpeedHistory h(kWindow);
  double v = speed;
  for (int i = 0; i < 50; ++i) {
    v += rate * kDt;
    h.advance(kDt, rate, lateral, v * yaw_rate, v, 1e-4, yaw_rate, v);
  }
  return h;
}

WindowCheck check(const SpeedHistory & h, double wheel)
{
  return window_check(h, h.reference(kWindow), h.reference(0.5 * kWindow), wheel, kWindowParams);
}

TEST(WindowCheckTest, AgreeingWheelIsNoSlip)
{
  const SpeedHistory h = steady(1.0, 0.0);
  EXPECT_FALSE(check(h, 1.0).gap);
  EXPECT_FALSE(check(h, 1.0).slip());
}

TEST(WindowCheckTest, WheelAheadIsSlip)
{
  const SpeedHistory h = steady(1.0, 0.0);
  EXPECT_TRUE(check(h, 1.5).slip());
}

TEST(WindowCheckTest, WheelBehindWithoutBrakingIsStaleHistory)
{
  // The reference says 1.0 and the IMU no braking, but the wheel says 0.5:
  // a driven wheel can't fall behind without braking, so the reference is
  // what's wrong.
  const SpeedHistory h = steady(1.0, 0.0);
  const WindowCheck c = check(h, 0.5);
  EXPECT_TRUE(c.gap);
  EXPECT_FALSE(c.slip());
  EXPECT_TRUE(c.stale());
}

TEST(WindowCheckTest, WheelBehindWhileBrakingIsSlipAtTheTighterThreshold)
{
  // Braking at 2.5 m/s^2 from 3 m/s: the reference is now ~2.5 m/s.
  const SpeedHistory h = steady(3.0, -2.5);
  const double reference = h.reference(kWindow)->speed;
  EXPECT_TRUE(check(h, reference - 0.2).slip());   // over 0.15 + 3 sigma
  EXPECT_FALSE(check(h, reference - 0.1).gap);     // under it
}

TEST(WindowCheckTest, NoReferenceNoDecision)
{
  const SpeedHistory h(kWindow);
  const WindowCheck c =
    window_check(h, std::nullopt, std::nullopt, 5.0, kWindowParams);
  EXPECT_FALSE(c.gap);
  EXPECT_FALSE(c.stale());
}

TEST(EarlyDistrustTest, MatchedAccelerationAddsNothing)
{
  const SpeedHistory h = steady(0.0, 2.0);
  const auto from = h.reference(0.2);
  ASSERT_TRUE(from.has_value());
  // The wheel accelerated as the IMU says.
  EXPECT_DOUBLE_EQ(early_distrust_variance(h, *from, h.at(h.size() - 1).speed, {0.6, 0.5}), 0.0);
}

TEST(EarlyDistrustTest, MismatchGrowsTheVariance)
{
  const SpeedHistory h = steady(0.0, 2.0);
  const auto from = h.reference(0.2);
  ASSERT_TRUE(from.has_value());
  // The wheel gained 0.4 m/s more than the IMU over 0.2 s: 2 m/s^2 extra.
  const double extra = early_distrust_variance(
    h, *from, h.at(h.size() - 1).speed + 0.4, {0.6, 0.5});
  EXPECT_NEAR(extra, std::pow(0.5 * (2.0 - 0.6), 2), 1e-6);
}

TEST(UndersteerTest, LateralAccelerationThatMatchesIsNot)
{
  const double v = 2.6;
  const double w = 2.0;
  const SpeedHistory h = steady(v, 0.0, w, v * w);
  EXPECT_FALSE(understeer(h, *h.reference(kWindow), w, 0.0, {0.5, 0.3}));
}

TEST(UndersteerTest, ShortfallIsUndersteer)
{
  // The history's wheel reads 3.0 x 2 rad/s = 6 m/s^2 expected; the IMU
  // measures 5.2 (the body at 2.6): 0.8 short.
  const SpeedHistory h = steady(3.0, 0.0, 2.0, 5.2);
  EXPECT_TRUE(understeer(h, *h.reference(kWindow), 2.0, 0.0, {0.5, 0.3}));
}

TEST(UndersteerTest, NotCheckedWhenNotTurning)
{
  const SpeedHistory h = steady(3.0, 0.0, 0.1, 0.0);
  EXPECT_FALSE(understeer(h, *h.reference(kWindow), 0.1, 0.0, {0.5, 0.3}));
}

TEST(ImpactTest, FoundOnlyBeyondTheDeceleration)
{
  SpeedHistory h = steady(1.0, 0.0);
  EXPECT_FALSE(impact(h, *h.reference(0.25), 8.0).has_value());
  h.advance(kDt, -40.0, 0.0, 0.0, 0.2, 1e-4, 0.0, 1.0);
  const auto when = impact(h, *h.reference(0.25), 8.0);
  ASSERT_TRUE(when.has_value());
  EXPECT_NEAR(*when, h.elapsed(), 1e-9);
}

}  // namespace
}  // namespace state_estimation
