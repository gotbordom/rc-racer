// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// L4 unit tests for the filter logic (VEH.GNC.SE spec §7,
// test_state_estimation_logic). Verifies REQ-GNC-001 (pose and velocity in
// odom), REQ-GNC-005 (slip detection), REQ-GNC-006 (covariance) and
// REQ-GNC-007 (configurable motion model), plus the spec §4 degraded modes.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

#include "state_estimation/extended_kalman_filter.hpp"
#include "state_estimation/state_estimation_logic.hpp"

namespace state_estimation
{
namespace
{

constexpr double kDt = 0.02;          // 50 Hz predict
constexpr double kGyroStd = 0.005;    // rad/s, config/sensor_noise.yaml
constexpr double kAccelStd = 0.05;    // m/s^2
constexpr double kWheelStd = 0.02;    // m/s

ImuMeasurement imu_sample(double yaw_rate, double forward_specific_force = 0.0)
{
  ImuMeasurement imu;
  imu.yaw_rate = yaw_rate;
  imu.yaw_rate_variance = kGyroStd * kGyroStd;
  imu.forward_specific_force = forward_specific_force;
  imu.forward_specific_force_variance = kAccelStd * kAccelStd;
  return imu;
}

WheelSpeedMeasurement wheel_sample(double speed) {return {speed, kWheelStd * kWheelStd};}

// An IMU sample in a turn: lateral (left) specific force too.
ImuMeasurement turn_sample(double yaw_rate, double lateral, double forward = 0.0)
{
  ImuMeasurement imu = imu_sample(yaw_rate, forward);
  imu.lateral_specific_force = lateral;
  imu.lateral_specific_force_variance = kAccelStd * kAccelStd;
  return imu;
}

// One 50 Hz cycle as the node runs it: predict, two IMU samples (100 Hz),
// one wheel sample.
void cycle(StateEstimationLogic & logic, const ImuMeasurement & imu, double wheel_speed)
{
  logic.predict(kDt);
  logic.update_imu(imu);
  logic.update_imu(imu);
  logic.update_wheel_speed(wheel_sample(wheel_speed));
}

bool covariance_is_valid(const StateMatrix & p)
{
  if (!p.isApprox(p.transpose(), 1e-12)) {
    return false;
  }
  const Eigen::SelfAdjointEigenSolver<StateMatrix> eig(p);
  return eig.eigenvalues().minCoeff() > -1e-12;
}

// ---------------------------------------------------------------------------
// Consistent inputs
// ---------------------------------------------------------------------------

TEST(StateEstimationLogicTest, TracksConstantTurnFromNoisySensors)
{
  // REQ-GNC-001: drive a 2 m/s, 0.3 rad/s circle for 10 s (20 m) with the
  // sim's sensor noise, and compare with the exact path.
  std::mt19937 rng(42);
  std::normal_distribution<double> gyro(0.0, kGyroStd);
  std::normal_distribution<double> accel(0.0, kAccelStd);
  std::normal_distribution<double> wheel(0.0, kWheelStd);

  const double v = 2.0;
  const double omega = 0.3;
  StateEstimationLogic logic;  // bicycle, the car's defaults
  const AckermannSteering truth_model(0.384, 0.52, 0.192);
  Pose2D truth;

  for (int i = 0; i < 500; ++i) {
    truth = truth_model.get_next_pose_estimate(truth, {v, omega}, kDt);
    logic.predict(kDt);
    for (int k = 0; k < 2; ++k) {
      logic.update_imu(imu_sample(omega + gyro(rng), accel(rng)));
    }
    logic.update_wheel_speed(wheel_sample(v + wheel(rng)));
  }

  const RobotState & s = logic.state();
  EXPECT_NEAR(s.mean(kSpeed), v, 3.0 * std::sqrt(s.covariance(kSpeed, kSpeed)) + 0.01);
  EXPECT_NEAR(s.mean(kYawRate), omega, 3.0 * std::sqrt(s.covariance(kYawRate, kYawRate)) + 0.005);
  EXPECT_NEAR(s.mean(kX), truth.x, 0.2);
  EXPECT_NEAR(s.mean(kY), truth.y, 0.2);
  EXPECT_NEAR(normalize_angle(s.mean(kYaw) - truth.yaw), 0.0, 0.02);
  EXPECT_FALSE(logic.status().slip_detected);
  EXPECT_FALSE(logic.status().imu_stale);
  EXPECT_FALSE(logic.status().wheel_stale);
  EXPECT_TRUE(covariance_is_valid(s.covariance));
}

TEST(StateEstimationLogicTest, UpdatesShrinkVelocityVariance)
{
  StateEstimationLogic logic;
  const double speed_var = logic.state().covariance(kSpeed, kSpeed);
  const double yaw_rate_var = logic.state().covariance(kYawRate, kYawRate);
  logic.update_wheel_speed(wheel_sample(1.0));
  logic.update_imu(imu_sample(0.1));
  EXPECT_LT(logic.state().covariance(kSpeed, kSpeed), speed_var);
  EXPECT_LT(logic.state().covariance(kYawRate, kYawRate), yaw_rate_var);
  EXPECT_NEAR(logic.state().mean(kSpeed), 1.0, 0.01);
  EXPECT_NEAR(logic.state().mean(kYawRate), 0.1, 0.01);
}

TEST(StateEstimationLogicTest, ImuAccelerationDrivesSpeedBetweenWheelUpdates)
{
  StateEstimationLogic logic;
  logic.update_wheel_speed(wheel_sample(0.0));
  for (int i = 0; i < 50; ++i) {  // 1 s at 1 m/s^2, no wheel odometry
    logic.update_imu(imu_sample(0.0, 1.0));
    logic.predict(kDt);
  }
  EXPECT_NEAR(logic.state().mean(kSpeed), 1.0, 0.01);
}

TEST(StateEstimationLogicTest, GravityRemovedUsingPitch)
{
  // Parked nose-down on a 0.1 rad slope: the accelerometer reads
  // -g sin(0.1) forward, but the car is not accelerating.
  StateEstimationLogic logic;
  ImuMeasurement imu = imu_sample(0.0, -9.80665 * std::sin(0.1));
  imu.pitch = 0.1;
  logic.update_wheel_speed(wheel_sample(0.0));
  for (int i = 0; i < 50; ++i) {
    logic.update_imu(imu);
    logic.predict(kDt);
  }
  EXPECT_NEAR(logic.state().mean(kSpeed), 0.0, 1e-9);
}

TEST(StateEstimationLogicTest, CentripetalLeverArmIsNotDeceleration)
{
  // Steady turn at omega = 1 rad/s: an IMU 0.05 m ahead of base_footprint is
  // 0.192 + 0.05 m ahead of the rear axle and reads -omega^2 * 0.242 on its
  // forward axis. The speed must not drop.
  StateEstimationLogic logic;
  ImuMeasurement imu = imu_sample(1.0, -1.0 * 1.0 * (0.192 + 0.05));
  imu.forward_offset = 0.05;
  logic.update_wheel_speed(wheel_sample(1.0));
  for (int i = 0; i < 50; ++i) {  // 1 s, IMU only
    logic.update_imu(imu);
    logic.predict(kDt);
  }
  EXPECT_NEAR(logic.state().mean(kSpeed), 1.0, 0.01);
}

TEST(StateEstimationLogicTest, PitchNoiseWidensSpeedUncertainty)
{
  // Removing gravity with a noisy pitch adds (g * sigma_pitch)^2 to the
  // acceleration variance: 0.02 rad is ~0.2 m/s^2, four times the
  // accelerometer's own noise.
  StateEstimationLogic with_pitch;
  StateEstimationLogic without_pitch;
  ImuMeasurement imu = imu_sample(0.0);
  imu.pitch = 0.0;
  imu.pitch_variance = 0.02 * 0.02;
  for (int i = 0; i < 50; ++i) {
    with_pitch.update_imu(imu);
    with_pitch.predict(kDt);
    without_pitch.update_imu(imu_sample(0.0));
    without_pitch.predict(kDt);
  }
  EXPECT_GT(
    with_pitch.state().covariance(kSpeed, kSpeed),
    without_pitch.state().covariance(kSpeed, kSpeed));
}

TEST(StateEstimationLogicTest, RejectsBadPitchVariance)
{
  StateEstimationLogic logic;
  ImuMeasurement imu = imu_sample(0.0);
  imu.pitch = 0.0;
  imu.pitch_variance = NAN;
  logic.update_imu(imu);
  EXPECT_EQ(logic.status().rejected_measurements, 1);
}

// ---------------------------------------------------------------------------
// Wheel encoder scale error (one encoder, rear axle)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Missing inputs (spec §4 degraded modes)
// ---------------------------------------------------------------------------

TEST(StateEstimationLogicTest, CovarianceGrowsWithoutUpdates)
{
  // REQ-GNC-006: with no inputs the estimate keeps going and says it is
  // getting less sure.
  StateEstimationLogic logic;
  logic.update_wheel_speed(wheel_sample(1.0));
  logic.update_imu(imu_sample(0.2));

  double trace = logic.state().covariance.trace();
  double pos_var = logic.state().covariance(kX, kX) + logic.state().covariance(kY, kY);
  for (int i = 0; i < 100; ++i) {
    logic.predict(kDt);
    const StateMatrix & p = logic.state().covariance;
    EXPECT_GT(p.trace(), trace);
    EXPECT_GE(p(kX, kX) + p(kY, kY), pos_var);
    trace = p.trace();
    pos_var = p(kX, kX) + p(kY, kY);
  }
  EXPECT_TRUE(logic.status().imu_stale);
  EXPECT_TRUE(logic.status().wheel_stale);
  EXPECT_TRUE(covariance_is_valid(logic.state().covariance));
}

TEST(StateEstimationLogicTest, StaleImuAccelerationIsNotReused)
{
  StateEstimationLogic logic;
  logic.update_wheel_speed(wheel_sample(0.0));
  logic.update_imu(imu_sample(0.0, 2.0));
  for (int i = 0; i < 50; ++i) {  // 1 s; the IMU goes stale after 0.1 s
    logic.predict(kDt);
  }
  // At most 0.1 s of 2 m/s^2 (one extra step for the boundary).
  EXPECT_LE(logic.state().mean(kSpeed), 2.0 * (0.1 + kDt) + 1e-9);
  EXPECT_TRUE(logic.status().imu_stale);
}

TEST(StateEstimationLogicTest, InputAgesAndStaleness)
{
  StateEstimationLogic logic;
  EXPECT_TRUE(std::isinf(logic.status().imu_age));
  EXPECT_TRUE(logic.status().imu_stale);
  EXPECT_TRUE(logic.status().wheel_stale);

  logic.update_imu(imu_sample(0.0));
  logic.update_wheel_speed(wheel_sample(0.0));
  logic.predict(0.15);
  EXPECT_NEAR(logic.status().imu_age, 0.15, 1e-12);
  EXPECT_TRUE(logic.status().imu_stale);     // timeout 0.1 s
  EXPECT_FALSE(logic.status().wheel_stale);  // timeout 0.2 s
  logic.predict(0.1);
  EXPECT_TRUE(logic.status().wheel_stale);
}

// ---------------------------------------------------------------------------
// Slip (REQ-GNC-005)
// ---------------------------------------------------------------------------

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

// Drive a turn as the bicycle model has it, without slip: the IMU (at the
// default mount, 0.192 m ahead of the rear axle) reads yaw rate x speed plus
// lever x yaw acceleration sideways, and the wheel reads `wheel_factor` x the
// body speed. The yaw rate ramps from 0 to `omega` over `turn_in` s, then
// holds. Returns whether understeer was ever flagged.
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
      logic.update_imu(turn_sample(w, w * speed + lever * alpha));
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

// ---------------------------------------------------------------------------
// Bad numbers
// ---------------------------------------------------------------------------

TEST(StateEstimationLogicTest, RejectsNonFiniteOrInvalidSamples)
{
  StateEstimationLogic logic;
  logic.update_wheel_speed(wheel_sample(1.0));
  const RobotState before = logic.state();

  ImuMeasurement nan_rate = imu_sample(NAN);
  ImuMeasurement inf_force = imu_sample(0.0, INFINITY);
  ImuMeasurement nan_pitch = imu_sample(0.0);
  nan_pitch.pitch = NAN;
  ImuMeasurement zero_var = imu_sample(0.0);
  zero_var.yaw_rate_variance = 0.0;
  logic.update_imu(nan_rate);
  logic.update_imu(inf_force);
  logic.update_imu(nan_pitch);
  logic.update_imu(zero_var);
  logic.update_wheel_speed({NAN, 1e-4});
  logic.update_wheel_speed({1.0, -1.0});

  EXPECT_EQ(logic.status().rejected_measurements, 6);
  EXPECT_TRUE(logic.state().mean.isApprox(before.mean));
  EXPECT_TRUE(logic.state().covariance.isApprox(before.covariance));
  EXPECT_TRUE(std::isinf(logic.status().imu_age));  // nothing accepted
}

TEST(StateEstimationLogicTest, RecoversFromNonFiniteEstimate)
{
  // A finite but absurd speed, then a long step, overflows the pose to Inf.
  StateEstimationLogic logic;
  logic.update_wheel_speed({1e300, 1e-4});
  const RobotState good = logic.state();
  ASSERT_TRUE(good.is_finite());

  logic.predict(1e10);

  EXPECT_EQ(logic.status().numerical_resets, 1);
  EXPECT_TRUE(logic.state().is_finite());
  EXPECT_TRUE(logic.state().mean.isApprox(good.mean));
  EXPECT_TRUE(
    logic.state().covariance.isApprox(
      good.covariance * logic.config().recovery_covariance_scale));
}

TEST(StateEstimationLogicTest, RejectsInvalidDt)
{
  StateEstimationLogic logic;
  EXPECT_THROW(logic.predict(-0.01), std::invalid_argument);
  EXPECT_THROW(logic.predict(NAN), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Reset and configuration
// ---------------------------------------------------------------------------

TEST(StateEstimationLogicTest, ResetReturnsToOrigin)
{
  StateEstimationLogic logic;
  for (int i = 0; i < 50; ++i) {
    cycle(logic, imu_sample(0.5), 2.0);
  }
  for (int i = 0; i < 5; ++i) {
    cycle(logic, imu_sample(0.5), 5.0);  // slip
  }
  ASSERT_TRUE(logic.status().slip_detected);
  ASSERT_GT(logic.state().mean.head<2>().norm(), 1.0);

  logic.reset();
  EXPECT_TRUE(logic.state().mean.isZero());
  EXPECT_DOUBLE_EQ(logic.state().covariance(kX, kX), logic.config().initial_pose_variance);
  EXPECT_DOUBLE_EQ(logic.state().covariance(kSpeed, kSpeed), logic.config().initial_speed_variance);
  EXPECT_FALSE(logic.status().slip_detected);
  EXPECT_FALSE(logic.status().imu_stale);  // the sensors are still alive
}

TEST(StateEstimationLogicTest, YawStaysNormalised)
{
  StateEstimationLogic logic;
  for (int i = 0; i < 1000; ++i) {  // 20 s at 1 rad/s: over three turns
    cycle(logic, imu_sample(1.0), 1.0);
    ASSERT_GE(logic.state().mean(kYaw), -M_PI);
    ASSERT_LT(logic.state().mean(kYaw), M_PI);
  }
  EXPECT_TRUE(covariance_is_valid(logic.state().covariance));
}

TEST(StateEstimationLogicTest, RejectsInvalidConfig)
{
  StateEstimationConfig cfg;
  cfg.motion_model = "tank";
  EXPECT_THROW(StateEstimationLogic{cfg}, std::invalid_argument);

  cfg = StateEstimationConfig{};
  cfg.fusion_method = "ukf";  // planned, not implemented
  EXPECT_THROW(StateEstimationLogic{cfg}, std::invalid_argument);

  cfg = StateEstimationConfig{};
  cfg.imu_timeout = 0.0;
  EXPECT_THROW(StateEstimationLogic{cfg}, std::invalid_argument);

  cfg = StateEstimationConfig{};
  cfg.slip_threshold = -1.0;
  EXPECT_THROW(StateEstimationLogic{cfg}, std::invalid_argument);

  cfg = StateEstimationConfig{};
  cfg.slip_variance_scale = 0.5;
  EXPECT_THROW(StateEstimationLogic{cfg}, std::invalid_argument);

  cfg = StateEstimationConfig{};
  cfg.wheel_scale_stddev = -0.01;
  EXPECT_THROW(StateEstimationLogic{cfg}, std::invalid_argument);

  cfg = StateEstimationConfig{};
  cfg.robot.wheelbase = 0.0;
  EXPECT_THROW(StateEstimationLogic{cfg}, std::invalid_argument);
}

// ---------------------------------------------------------------------------
// EKF details
// ---------------------------------------------------------------------------

TEST(ExtendedKalmanFilterTest, UpdateMatchesScalarKalmanGain)
{
  const ExtendedKalmanFilter ekf;
  RobotState s;
  s.covariance = StateMatrix::Identity() * 0.5;
  s.mean(kSpeed) = 1.0;
  ekf.update(s, kSpeed, 2.0, 0.5);  // equal variances: halfway, half the variance
  EXPECT_NEAR(s.mean(kSpeed), 1.5, 1e-12);
  EXPECT_NEAR(s.covariance(kSpeed, kSpeed), 0.25, 1e-12);
  EXPECT_NEAR(s.covariance(kX, kX), 0.5, 1e-12);  // uncorrelated elements untouched
}

TEST(ExtendedKalmanFilterTest, SpeedUpdateCorrectsCorrelatedPosition)
{
  // After driving, position and speed are correlated, so learning the
  // speed also moves the position estimate.
  const ExtendedKalmanFilter ekf;
  const TwoWheelDifferentialSteering frame;
  RobotState s;
  s.covariance = StateMatrix::Identity() * 1e-6;
  s.covariance(kSpeed, kSpeed) = 1.0;
  s.mean(kSpeed) = 1.0;
  ekf.predict(s, frame, PredictInputs{}, 1.0);
  ASSERT_NEAR(s.mean(kX), 1.0, 1e-9);
  ASSERT_GT(s.covariance(kX, kSpeed), 0.5);

  ekf.update(s, kSpeed, 2.0, 1e-6);
  EXPECT_GT(s.mean(kX), 1.9);
}

TEST(MakeFusionMethodTest, BuildsEkfAndRejectsOthers)
{
  EXPECT_EQ(make_fusion_method("ekf")->name(), "ekf");
  EXPECT_THROW(make_fusion_method("ukf"), std::invalid_argument);
  EXPECT_THROW(make_fusion_method("complementary"), std::invalid_argument);
  EXPECT_THROW(make_fusion_method("magic"), std::invalid_argument);
}

}  // namespace
}  // namespace state_estimation
