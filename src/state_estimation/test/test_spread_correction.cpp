// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Unit tests for SpreadCorrection: position changes applied over time.

#include <gtest/gtest.h>

#include "state_estimation/spread_correction.hpp"

namespace state_estimation
{
namespace
{

constexpr double kDt = 0.02;

TEST(SpreadCorrectionTest, AppliesTheWholeChangeOverItsDuration)
{
  SpreadCorrection spread(0.5);
  spread.add({0.25, -0.1});
  Eigen::Vector2d total = Eigen::Vector2d::Zero();
  double largest = 0.0;
  for (int i = 0; i < 25; ++i) {  // 0.5 s
    const Eigen::Vector2d share = spread.step(kDt);
    largest = std::max(largest, share.norm());
    total += share;
  }
  EXPECT_NEAR(total.x(), 0.25, 1e-9);
  EXPECT_NEAR(total.y(), -0.1, 1e-9);
  EXPECT_LT(largest, 0.012);  // ~1/25 of 0.27 m per step, not all at once
  EXPECT_TRUE(spread.step(kDt).isZero());  // nothing left
}

TEST(SpreadCorrectionTest, ALaterChangeCarriesWhatIsStillPending)
{
  SpreadCorrection spread(0.5);
  spread.add({0.2, 0.0});
  Eigen::Vector2d total = Eigen::Vector2d::Zero();
  for (int i = 0; i < 10; ++i) {
    total += spread.step(kDt);
  }
  spread.add({-0.1, 0.0});
  for (int i = 0; i < 30; ++i) {
    total += spread.step(kDt);
  }
  EXPECT_NEAR(total.x(), 0.1, 1e-9);
}

TEST(SpreadCorrectionTest, ClearDropsWhatIsPending)
{
  SpreadCorrection spread(0.5);
  spread.add({1.0, 1.0});
  spread.clear();
  EXPECT_TRUE(spread.step(kDt).isZero());
}

}  // namespace
}  // namespace state_estimation
