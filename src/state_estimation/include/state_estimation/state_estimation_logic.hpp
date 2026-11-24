// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// State estimation logic for VEH.GNC.SE (spec §4, §6): fuses IMU and wheel
// speed into the vehicle's own motion in the odom frame. Pure C++, no ROS;
// StateEstimationNode converts messages to these structs and calls it.
//
// It never uses world-frame data (ISSUE-002): the IMU's absolute yaw is
// ignored, and only its roll and pitch are used, to remove gravity.

#ifndef STATE_ESTIMATION__STATE_ESTIMATION_LOGIC_HPP_
#define STATE_ESTIMATION__STATE_ESTIMATION_LOGIC_HPP_

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "state_estimation/robot_frame.hpp"
#include "state_estimation/robot_state.hpp"
#include "state_estimation/sensor_fusion_method.hpp"

namespace state_estimation
{

/// One IMU sample, already rotated into base_footprint by the node.
struct ImuMeasurement
{
  double yaw_rate{0.0};           ///< rad/s
  double yaw_rate_variance{0.0};  ///< (rad/s)^2
  /// Forward specific force, m/s^2, INCLUDING gravity (ICD-SNS-GNC).
  double forward_specific_force{0.0};
  double forward_specific_force_variance{0.0};  ///< (m/s^2)^2
  /// Pitch from the IMU's orientation (REP-103: positive = nose down), used
  /// to remove gravity. Unset when the IMU gives no orientation; the ground
  /// is then assumed level.
  std::optional<double> pitch;
  /// rad^2. Pitch noise becomes acceleration noise (g * cos(pitch))^2 *
  /// pitch_variance through gravity removal (0.02 rad is ~0.2 m/s^2).
  double pitch_variance{0.0};
  /// m, IMU position ahead of base_footprint (from TF). In a turn an
  /// accelerometer ahead of the no-sideslip point (rear axle) feels
  /// -omega^2 * lever on its forward axis, which is not acceleration.
  double forward_offset{0.0};
};

/// Forward speed from wheel odometry (ICD-SNS-GNC: only twist.linear.x).
struct WheelSpeedMeasurement
{
  double speed{0.0};     ///< m/s, signed
  double variance{0.0};  ///< (m/s)^2
};

/// Parameters (spec §3.4). Defaults marked TBD are tuned in the sim.
struct StateEstimationConfig
{
  std::string motion_model{"bicycle"};
  std::string fusion_method{"ekf"};
  RobotFrameParams robot;

  double imu_timeout{0.1};         ///< s, inputs.imu.timeout
  double wheel_odom_timeout{0.2};  ///< s, inputs.wheel_odom.timeout

  /// Unmodelled forward acceleration, (m/s^2)^2/s, while IMU acceleration
  /// is used as input (covers tilt and bias errors): ~0.05 m/s of speed
  /// sigma after 1 s of IMU only. TBD.
  double accel_noise_density{0.003};
  /// Forward acceleration, (m/s^2)^2/s, when the IMU is stale and nothing
  /// is known about acceleration. TBD.
  double accel_noise_density_no_imu{4.0};
  /// Unmodelled change in yaw rate, (rad/s^2)^2/s. TBD.
  double yaw_accel_noise_density{1.0};

  /// Wheel encoder scale error (one encoder, rear axle). It reads (1 + k) v
  /// with k unknown (calibration, tyre wear). k cannot be observed without
  /// an absolute reference, so it is not estimated; its effect is added to
  /// the published covariance instead (estimate()). Constant part: stddev of
  /// k. TBD (calibration).
  double wheel_scale_stddev{0.01};
  /// Varying part (scrub in turns): along-track noise, m^2 per metre
  /// driven. TBD, set from the L2 sim test.
  double distance_noise_density{1.0e-3};

  /// Wheel speed minus predicted speed (m/s) beyond which the wheels are
  /// slipping, on top of 3 sigma of the predicted speed. Also the limit for
  /// the windowed check against slip_window seconds of IMU. TBD.
  double slip_threshold{0.3};
  /// s. The wheel speed is also compared with the estimate from this long
  /// ago plus the IMU acceleration since. The one-step check alone misses
  /// slip that builds up over several steps (e.g. wheels spinning down for
  /// 0.5 s after a crash), because each update re-anchors the estimate to
  /// the slipping wheel. TBD.
  double slip_window{0.5};
  /// s. Longest IMU-only stretch while slipping. IMU-only speed drifts, and
  /// a crash impulse can be too short for a 100 Hz IMU (or saturate it), so
  /// after this the wheel is trusted again. TBD.
  double slip_max_duration{2.0};
  /// Stationary: wheel below this (m/s) with a quiet IMU (yaw rate below
  /// stationary_yaw_rate, forward acceleration below stationary_accel).
  /// Ends slip at once: e.g. pinned against an obstacle after a crash. TBD.
  double stationary_speed{0.05};
  double stationary_yaw_rate{0.05};   ///< rad/s
  double stationary_accel{0.3};       ///< m/s^2
  /// Yaw rate (rad/s) beyond what the motion model allows at this speed
  /// that counts as a slide. Never triggers for the unicycle. TBD.
  double slip_yaw_rate_margin{0.2};
  /// Factor on wheel-speed variance while slipping: large enough that the
  /// slipping wheel has no real pull (1e4 still dragged the estimate after
  /// a crash). TBD.
  double slip_variance_scale{1.0e6};
  /// When the windowed check catches a slip, the position is moved back by
  /// what the wheel added over the window (VAL-017). This share of that
  /// correction is added as 1-sigma along-track uncertainty, as the window
  /// start may itself have been partly off. TBD.
  double slip_correction_uncertainty{0.3};
  /// m/s^2. The windowed check counts a wheel slower than its IMU reference
  /// as slip only while the IMU shows at least this much deceleration over
  /// the recent half window: a driven wheel falls behind the car only while
  /// braking. TBD.
  double slip_braking_accel{0.5};
  /// s. The retroactive correction is applied over this long, so odom stays
  /// continuous. TBD.
  double slip_correction_time{0.5};

  /// Initial covariance diagonal after reset: the pose is exact by
  /// definition (odom starts at the car), the velocities are not.
  double initial_pose_variance{1.0e-6};
  double initial_speed_variance{1.0};
  double initial_yaw_rate_variance{0.1};

  /// Factor on the covariance after recovering from a non-finite estimate.
  double recovery_covariance_scale{10.0};
};

/// Health values for /diagnostics (spec §5).
struct StateEstimationStatus
{
  /// Seconds of prediction since the last accepted sample; +inf before the
  /// first one.
  double imu_age{std::numeric_limits<double>::infinity()};
  double wheel_age{std::numeric_limits<double>::infinity()};
  bool imu_stale{true};
  bool wheel_stale{true};

  bool slip_detected{false};
  /// Times slip started. Counted so a health report can warn about slip
  /// that came and went between reports.
  int slip_events{0};
  /// Times the windowed check found its history stale (a wheel behind its
  /// reference with no braking: left by a slip too small to catch) and
  /// rebuilt it, instead of calling slip.
  int stale_history_rebases{0};
  /// Variance used for the most recent wheel-speed update (inflated while
  /// slipping).
  double wheel_variance_used{0.0};

  int numerical_resets{0};       ///< recoveries from a non-finite estimate
  int rejected_measurements{0};  ///< samples with NaN/Inf or bad variance
};

class StateEstimationLogic
{
public:
  /// @throws std::invalid_argument for an invalid configuration.
  explicit StateEstimationLogic(const StateEstimationConfig & config = StateEstimationConfig{});

  /// Advance the estimate dt seconds (the 50 Hz timer).
  /// @throws std::invalid_argument if dt is negative or not finite.
  void predict(double dt);

  /// Fuse an IMU sample: updates yaw rate, and keeps its acceleration as the
  /// input for the following predict steps.
  void update_imu(const ImuMeasurement & imu);

  /// Fuse a wheel-speed sample, after checking for slip.
  void update_wheel_speed(const WheelSpeedMeasurement & wheel);

  /// Back to the odom origin with the initial covariance (`~/reset`).
  void reset();

  /// Internal filter state (no wheel-scale term).
  const RobotState & state() const {return state_;}

  /// Published estimate: state() plus the covariance an unknown, constant
  /// wheel scale error k adds. Dead reckoning scales the path about the
  /// odom origin by (1 + k), so position gains wheel_scale_stddev^2 * p p^T
  /// (p = position in odom) and speed gains wheel_scale_stddev^2 * v^2.
  RobotState estimate() const;
  const StateEstimationStatus & status() const {return status_;}
  const StateEstimationConfig & config() const {return config_;}

private:
  /// The estimate from `age` ago (the newest history sample at least that
  /// old) plus the IMU acceleration since, and the variance it had then;
  /// unset until that much fresh IMU history exists.
  struct ReferenceSpeed
  {
    double speed;
    double variance;
    std::size_t index;  ///< of the history sample it starts from
  };
  std::optional<ReferenceSpeed> imu_reference_speed(double age) const;

  /// History older than sample `from` holds speeds the IMU path from it
  /// contradicts (an earlier, undetected slip left them): rewrite them as
  /// that sample's speed minus the IMU integral back to them.
  void rebase_history_before(std::size_t from);

  /// Move the position by this step's share of a pending correction.
  void apply_pending_correction(double dt);
  Eigen::Vector2d pending_correction_{Eigen::Vector2d::Zero()};  ///< m, still to apply
  double correction_time_left_{0.0};

  /// A slip caught by the windowed check: the estimate followed the wheel
  /// since the window start, the last speed trusted. Rewrite the history as
  /// the IMU path from there, move the position back by the distance the
  /// wheel added on top of it, and grow the covariance by that correction's
  /// uncertainty (VAL-017). Returns the correction (m, along track).
  double correct_window_retroactively(const ReferenceSpeed & from);


  /// Restore the last good estimate if the current one is not finite.
  void guard_numerics();
  void update_staleness();

  StateEstimationConfig config_;
  std::unique_ptr<RobotFrame> frame_;
  std::unique_ptr<SensorFusionMethod> fusion_;

  RobotState state_;
  RobotState last_good_;
  StateEstimationStatus status_;

  double forward_accel_{0.0};           ///< gravity removed, from the last IMU sample
  double forward_accel_variance_{0.0};
  /// IMU samples since the last predict, averaged there: the IMU runs faster
  /// than predict, and using only the latest sample drops the others (a
  /// short jolt could lose half of itself).
  double accel_sum_{0.0};
  double accel_variance_sum_{0.0};
  int accel_samples_{0};
  double predict_accel_{0.0};           ///< the average used by the last predict
  double predict_accel_variance_{0.0};

  /// For the windowed slip check: per predict step, the time, the speed
  /// estimate and the running integral of the acceleration input.
  struct SpeedSample
  {
    double time;
    double speed;
    double speed_variance;
    double accel_integral;
  };
  std::deque<SpeedSample> speed_history_;
  /// While slipping: since when the wheel has agreed again (hysteresis).
  std::optional<double> agreeing_since_;
  double slip_since_{0.0};       ///< elapsed_ when the current slip started
  double last_yaw_rate_{0.0};    ///< rad/s, from the last IMU sample
  double elapsed_{0.0};
  double accel_integral_{0.0};
};

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__STATE_ESTIMATION_LOGIC_HPP_
