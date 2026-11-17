// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// L4 unit tests for the filter logic (VEH.GNC.SE spec §7,
// test_state_estimation_logic). Verifies REQ-GNC-001 (pose and velocity in
// odom), REQ-GNC-005 (slip detection), REQ-GNC-006 (covariance) and
// REQ-GNC-007 (configurable motion model), plus the spec §4 degraded modes.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <stdexcept>

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
  StateEstimationLogic logic;
  const TwoWheelDifferentialSteering truth_model;
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

  // Grip again: wheels agree with the estimate, slip clears.
  cycle(logic, imu_sample(0.0), 1.0);
  EXPECT_FALSE(logic.status().slip_detected);
  EXPECT_DOUBLE_EQ(logic.status().wheel_variance_used, kWheelStd * kWheelStd);
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
