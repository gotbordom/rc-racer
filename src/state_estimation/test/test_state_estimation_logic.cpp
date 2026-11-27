// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// L4 unit tests for the filter logic (VEH.GNC.SE spec §7,
// test_state_estimation_logic): REQ-GNC-001 (pose and velocity in odom),
// REQ-GNC-006 (covariance) and REQ-GNC-007 (configurable motion model), the
// spec §4 degraded modes, bad numbers, reset and configuration, the EKF.
// The slip tests (REQ-GNC-005) and the scale-error tests are in
// test_logic_*.cpp, by topic.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

#include "state_estimation/extended_kalman_filter.hpp"
#include "logic_test_helpers.hpp"
#include "state_estimation/state_estimation_logic.hpp"

namespace state_estimation
{
namespace
{

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

TEST(StateEstimationLogicTest, SideSpeedFollowsTheNoSlipKinematics)
{
  // Feature 2.5, step 1: in a turn that grips, base_footprint slides
  // sideways at omega x rear_axle_offset (0.192 m), as the bicycle model has
  // it. The no-slip constraint holds v_y there, and /odom reports it.
  StateEstimationLogic logic;
  const double v = 2.0;
  const double omega = 1.0;
  for (int i = 0; i < 200; ++i) {
    logic.predict(kDt);
    for (int k = 0; k < 2; ++k) {
      // Ahead of the rear axle the IMU reads -omega^2 x lever forward and
      // omega x v sideways in a steady turn.
      logic.update_imu(turn_sample(omega, omega * v, -omega * omega * 0.192));
    }
    logic.update_wheel_speed(wheel_sample(v));
  }
  EXPECT_NEAR(logic.state().mean(kSideSpeed), omega * 0.192, 0.01);
  EXPECT_NEAR(logic.state().mean(kSpeed), v, 0.02);
  EXPECT_FALSE(logic.status().slip_detected);
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
// Missing inputs (spec §4 degraded modes)
// ---------------------------------------------------------------------------

TEST(StateEstimationLogicTest, CovarianceGrowsWithoutUpdates)
{
  // REQ-GNC-006: with no inputs the estimate keeps going and says it is
  // getting less sure. The first predict also applies the no-slip
  // constraint, which takes the initial sideways-speed uncertainty (2.5), so
  // the total is checked from there.
  StateEstimationLogic logic;
  logic.update_wheel_speed(wheel_sample(1.0));
  logic.update_imu(imu_sample(0.2));
  logic.predict(kDt);

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
