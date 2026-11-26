// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// L4 unit tests for the filter logic (VEH.GNC.SE spec §7): Wheel encoder scale
// error and out-and-back (REQ-GNC-006, VAL-016).
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

// Drive straight at 1 m/s for n cycles; the wheel reads (1 + bias) * 1 m/s.
// Returns the estimate's x error.
double drive_straight(StateEstimationLogic & logic, double bias, int n)
{
  std::mt19937 rng(1);
  std::normal_distribution<double> gyro(0.0, kGyroStd);
  std::normal_distribution<double> accel(0.0, kAccelStd);
  std::normal_distribution<double> wheel(0.0, kWheelStd);
  for (int i = 0; i < n; ++i) {
    logic.predict(kDt);
    logic.update_imu(imu_sample(gyro(rng), accel(rng)));
    logic.update_imu(imu_sample(gyro(rng), accel(rng)));
    logic.update_wheel_speed(wheel_sample(1.0 + bias + wheel(rng)));
  }
  return logic.state().mean(kX) - n * kDt;
}

// Out and back (SDP §3.6 VAL-016, issue #8): the published covariance
// shrinks for a while when the car drives straight back.

struct OutAndBackSample
{
  double true_x;
  double error;        // estimate - truth, along x
  double variance;     // published (estimate()) x variance
  double scale_part;   // its wheel scale term, estimate() - state()
};

// Drive 1 m/s straight out for `cruise_cycles`, stop and drive straight
// back to the start, accelerating at 1 m/s^2 with a matching IMU reading.
// The wheel reads (1 + bias_forward) * v forward and (1 + bias_reverse) * v
// in reverse.
std::vector<OutAndBackSample> out_and_back(
  StateEstimationLogic & logic, double bias_forward, double bias_reverse, int cruise_cycles)
{
  std::mt19937 rng(3);
  std::normal_distribution<double> gyro(0.0, kGyroStd);
  std::normal_distribution<double> accel(0.0, kAccelStd);
  std::normal_distribution<double> wheel(0.0, kWheelStd);
  const int ramp = 50;  // 1 s from 0 to 1 m/s
  std::vector<double> speeds;
  for (int i = 1; i <= ramp; ++i) {
    speeds.push_back(i * kDt);
  }
  speeds.insert(speeds.end(), cruise_cycles, 1.0);
  for (int i = 1; i <= 2 * ramp; ++i) {  // through 0 to -1 m/s
    speeds.push_back(1.0 - i * kDt);
  }
  speeds.insert(speeds.end(), cruise_cycles, -1.0);
  for (int i = 1; i <= ramp; ++i) {
    speeds.push_back(-1.0 + i * kDt);
  }

  std::vector<OutAndBackSample> samples;
  double true_x = 0.0;
  double previous = 0.0;
  for (double v : speeds) {
    true_x += 0.5 * (previous + v) * kDt;
    const double a = (v - previous) / kDt;
    previous = v;
    const double bias = v >= 0.0 ? bias_forward : bias_reverse;
    logic.predict(kDt);
    logic.update_imu(imu_sample(gyro(rng), a + accel(rng)));
    logic.update_imu(imu_sample(gyro(rng), a + accel(rng)));
    logic.update_wheel_speed(wheel_sample((1.0 + bias) * v + wheel(rng)));
    const RobotState e = logic.estimate();
    samples.push_back(
      {true_x, e.mean(kX) - true_x, e.covariance(kX, kX),
        e.covariance(kX, kX) - logic.state().covariance(kX, kX)});
  }
  return samples;
}

const OutAndBackSample & turn_point(const std::vector<OutAndBackSample> & samples)
{
  return *std::max_element(
    samples.begin(), samples.end(),
    [](const auto & a, const auto & b) {return a.true_x < b.true_x;});
}

// The return leg's sample nearest true_x.
const OutAndBackSample & on_the_way_back(
  const std::vector<OutAndBackSample> & samples, double true_x)
{
  const auto turn = std::max_element(
    samples.begin(), samples.end(),
    [](const auto & a, const auto & b) {return a.true_x < b.true_x;});
  return *std::min_element(
    turn, samples.end(),
    [true_x](const auto & a, const auto & b) {
      return std::abs(a.true_x - true_x) < std::abs(b.true_x - true_x);
    });
}

TEST(StateEstimationLogicTest, MatchesDeadReckoningOnUnbiasedInputs)
{
  // Regression: estimating the scale as a filter state let wheel updates
  // move the position through its correlation with the scale: 3.2 m off
  // after 100 m of unbiased inputs. The position must follow the wheels.
  StateEstimationLogic logic;
  EXPECT_NEAR(drive_straight(logic, 0.0, 5000), 0.0, 0.1);
}

TEST(StateEstimationLogicTest, ScaleErrorIsInsidePublishedCovariance)
{
  // REQ-GNC-006: a wheel reading 0.5 % high puts the estimate 0.5 m ahead
  // after 100 m. The filter cannot see that (state() stays tight), but the
  // published estimate() must cover it. Distance noise off, to check the
  // constant-scale term on its own.
  StateEstimationConfig cfg;
  cfg.distance_noise_density = 1e-12;
  StateEstimationLogic logic(cfg);
  const double error = drive_straight(logic, 0.005, 5000);
  EXPECT_NEAR(error, 0.5, 0.1);
  EXPECT_LT(3.0 * std::sqrt(logic.state().covariance(kX, kX)), error);  // filter alone: too tight
  EXPECT_GT(3.0 * std::sqrt(logic.estimate().covariance(kX, kX)), error);
}

TEST(StateEstimationLogicTest, ScaleCovarianceGrowsWithDistanceFromOrigin)
{
  // A constant scale error scales the path about the odom origin.
  StateEstimationConfig cfg;
  cfg.distance_noise_density = 1e-12;  // isolate the constant part
  StateEstimationLogic logic(cfg);
  drive_straight(logic, 0.0, 2500);  // 50 m
  const RobotState e = logic.estimate();
  const RobotState s = logic.state();
  const double x = e.mean(kX);
  EXPECT_NEAR(
    e.covariance(kX, kX) - s.covariance(kX, kX),
    cfg.wheel_scale_stddev * cfg.wheel_scale_stddev * x * x, 1e-9);
  const double y = e.mean(kY);  // ~mm from gyro noise
  EXPECT_NEAR(
    e.covariance(kY, kY) - s.covariance(kY, kY),
    cfg.wheel_scale_stddev * cfg.wheel_scale_stddev * y * y, 1e-12);
  EXPECT_TRUE(e.mean.isApprox(s.mean));  // only the covariance changes
}

TEST(StateEstimationLogicTest, DistanceNoiseGrowsAlongTrackOnly)
{
  StateEstimationConfig quiet;
  quiet.distance_noise_density = 1e-12;
  StateEstimationLogic logic;
  StateEstimationLogic without(quiet);
  drive_straight(logic, 0.0, 2500);
  drive_straight(without, 0.0, 2500);
  const double added = logic.state().covariance(kX, kX) - without.state().covariance(kX, kX);
  EXPECT_NEAR(added, StateEstimationConfig{}.distance_noise_density * 50.0, 0.1 * 0.05);
  EXPECT_NEAR(logic.state().covariance(kY, kY), without.state().covariance(kY, kY), 1e-6);
}

TEST(StateEstimationLogicTest, OutAndBackCovarianceShrinksThenGrows)
{
  // #8: variance ~ s^2 x^2 (scale) + q d (distance noise, d travelled). On
  // the way back it falls while 2 s^2 x > q, so to x = q / (2 s^2) = 5 m,
  // then grows again: what the owner saw.
  StateEstimationLogic logic;
  const StateEstimationConfig cfg;
  const auto samples = out_and_back(logic, 0.0, 0.0, 450);  // ~10 m out
  const double x_min = cfg.distance_noise_density /
    (2.0 * cfg.wheel_scale_stddev * cfg.wheel_scale_stddev);
  ASSERT_NEAR(x_min, 5.0, 1e-9);
  const OutAndBackSample & turn = turn_point(samples);
  const OutAndBackSample & lowest = on_the_way_back(samples, x_min);
  ASSERT_GT(turn.true_x, 9.5);
  EXPECT_LT(lowest.variance, 0.95 * turn.variance);
  EXPECT_LT(lowest.variance, 0.95 * samples.back().variance);
  EXPECT_LT(on_the_way_back(samples, 7.5).variance, turn.variance);
  EXPECT_GT(on_the_way_back(samples, 2.5).variance, lowest.variance);
}

TEST(StateEstimationLogicTest, OutAndBackScaleErrorCancelsAsTheCovarianceSays)
{
  // #8, question 2: with the same 1 % over-read both ways the true error
  // shrinks back to ~0 at the start, as the scale term does, and stays
  // inside 3 sigma all the way. Distance noise off, to check the scale term.
  StateEstimationConfig cfg;
  cfg.distance_noise_density = 1e-12;
  StateEstimationLogic logic(cfg);
  const auto samples = out_and_back(logic, 0.01, 0.01, 450);
  const OutAndBackSample & turn = turn_point(samples);
  EXPECT_NEAR(turn.error, 0.01 * turn.true_x, 0.03);
  EXPECT_LT(std::abs(samples.back().error), 0.2 * turn.error);
  EXPECT_LT(samples.back().scale_part, 0.05 * turn.scale_part);
  for (const auto & s : samples) {
    EXPECT_LT(std::abs(s.error), 3.0 * std::sqrt(s.variance)) << "at x = " << s.true_x;
  }
}

TEST(StateEstimationLogicTest, OutAndBackDifferentScaleInReverseIsCovered)
{
  // #8, question 3: if the scale differs in reverse, the error does not
  // cancel while the scale term still shrinks. Worst case here: 1 % over
  // forward, 1 % under in reverse, so the car is 2 % of the leg (0.2 m)
  // out at the start. The distance noise must still cover it.
  StateEstimationLogic logic;
  const auto samples = out_and_back(logic, 0.01, -0.01, 450);
  EXPECT_NEAR(samples.back().error, 0.02 * turn_point(samples).true_x, 0.05);
  for (const auto & s : samples) {
    EXPECT_LT(std::abs(s.error), 3.0 * std::sqrt(s.variance)) << "at x = " << s.true_x;
  }
}

}  // namespace
}  // namespace state_estimation
