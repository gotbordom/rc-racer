// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Unit tests for SpeedHistory, the look-back the slip checks share.

#include <gtest/gtest.h>

#include "state_estimation/speed_history.hpp"

namespace state_estimation
{
namespace
{

constexpr double kDt = 0.02;

// n steps at a constant forward rate, the speed estimate following it.
void accelerate(SpeedHistory & h, int n, double rate, double & speed)
{
  for (int i = 0; i < n; ++i) {
    speed += rate * kDt;
    h.advance(kDt, rate, 0.0, 0.0, speed, 0.01, 0.0, speed);
  }
}

TEST(SpeedHistoryTest, KeepsTheSpanAndNoMore)
{
  SpeedHistory h(0.5);
  double v = 0.0;
  accelerate(h, 100, 0.0, v);
  EXPECT_NEAR(h.elapsed(), 2.0, 1e-9);
  // The front is the newest sample at least 0.5 s old.
  EXPECT_LE(h.at(0).time, h.elapsed() - 0.5 + 1e-9);
  EXPECT_GT(h.at(1).time, h.elapsed() - 0.5);
}

TEST(SpeedHistoryTest, ReferenceCarriesTheSpeedForwardByTheIntegral)
{
  SpeedHistory h(0.5);
  double v = 1.0;
  EXPECT_FALSE(h.reference(0.5).has_value());  // nothing recorded yet
  accelerate(h, 50, 2.0, v);                     // 1 s at 2 m/s^2: v 1 -> 3
  const auto ref = h.reference(0.5);
  ASSERT_TRUE(ref.has_value());
  // The sample's own speed plus what the integral adds since: the speed now.
  EXPECT_NEAR(ref->speed, v, 1e-9);
  EXPECT_NEAR(h.mean_accel_since(ref->index), 2.0, 1e-9);
}

TEST(SpeedHistoryTest, RebaseRewritesOlderSamplesFromTheAnchor)
{
  SpeedHistory h(1.0);
  double v = 1.0;
  accelerate(h, 50, 0.0, v);
  const std::size_t anchor = h.size() - 1;
  h.update_latest(0.7, 0.01);  // the anchor's speed is now known to be 0.7
  h.rebase_before(anchor);
  for (std::size_t i = 0; i < anchor; ++i) {
    EXPECT_NEAR(h.at(i).speed, 0.7, 1e-9);  // no acceleration: same speed
  }
}

TEST(SpeedHistoryTest, RewriteReturnsTheDistanceTheOldSpeedsAdded)
{
  // The window starts at the true 1.0 m/s; after it the estimate followed a
  // wheel reading 1.2 while the IMU said no acceleration.
  SpeedHistory h(1.0);
  h.advance(kDt, 0.0, 0.0, 0.0, 1.0, 0.01, 0.0, 1.0);
  for (int i = 0; i < 49; ++i) {
    h.advance(kDt, 0.0, 0.0, 0.0, 1.2, 0.01, 0.0, 1.2);
  }
  const double added = h.rewrite_as_imu_path(0);
  EXPECT_NEAR(added, 0.2 * 49 * kDt, 1e-9);
  for (std::size_t i = 1; i < h.size(); ++i) {
    EXPECT_NEAR(h.at(i).speed, 1.0, 1e-9);  // now the IMU path from the start
  }
}

TEST(SpeedHistoryTest, FindsAStepBeyondADeceleration)
{
  SpeedHistory h(0.5);
  double v = 1.0;
  accelerate(h, 10, 0.0, v);
  const std::size_t from = 0;
  EXPECT_FALSE(h.first_step_beyond(from, 8.0, 1.0).has_value());
  accelerate(h, 1, -40.0, v);  // an impact
  const auto when = h.first_step_beyond(from, 8.0, 1.0);
  ASSERT_TRUE(when.has_value());
  EXPECT_NEAR(*when, h.elapsed(), 1e-9);
  EXPECT_FALSE(h.first_step_beyond(from, 8.0, -1.0).has_value());  // wrong direction
}

TEST(SpeedHistoryTest, DistanceAfterSumsTheRecordedSpeeds)
{
  SpeedHistory h(1.0);
  double v = 0.5;
  accelerate(h, 20, 0.0, v);
  const double t = h.elapsed();
  accelerate(h, 10, 0.0, v);
  EXPECT_NEAR(h.distance_after(t), 0.5 * 10 * kDt, 1e-9);
}

}  // namespace
}  // namespace state_estimation
