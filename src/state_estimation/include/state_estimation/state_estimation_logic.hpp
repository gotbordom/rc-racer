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

#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "state_estimation/robot_frame.hpp"
#include "state_estimation/robot_state.hpp"
#include "state_estimation/sensor_fusion_method.hpp"
#include "state_estimation/speed_history.hpp"
#include "state_estimation/spread_correction.hpp"

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
  /// Lateral (left) specific force, m/s^2, including gravity. In a turn
  /// without slip it is yaw rate x speed (plus the lever arm term): the
  /// understeer check compares it with what the wheel speed implies.
  double lateral_specific_force{0.0};
  /// (m/s^2)^2; 0 = not provided, and the understeer check is skipped.
  double lateral_specific_force_variance{0.0};
  /// Pitch from the IMU's orientation (REP-103: positive = nose down), used
  /// to remove gravity. Unset when the IMU gives no orientation; the ground
  /// is then assumed level.
  std::optional<double> pitch;
  /// rad^2. Pitch noise becomes acceleration noise (g * cos(pitch))^2 *
  /// pitch_variance through gravity removal (0.02 rad is ~0.2 m/s^2).
  double pitch_variance{0.0};
  /// Roll (REP-103: positive = right side down), to remove gravity from the
  /// lateral specific force; unset like pitch.
  std::optional<double> roll;
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
  /// m/s. The windowed check's threshold for that case, a wheel
  /// behind its reference while braking (on top of 3 sigma), tighter than
  /// slip_threshold: a braking slide builds slowly (VAL-006). TBD.
  double slip_braking_threshold{0.15};
  /// s. The retroactive correction is applied over this long, so odom stays
  /// continuous. TBD.
  double slip_correction_time{0.5};
  /// m/s^2. Understeer (a slide the yaw-rate check cannot see, VAL-004):
  /// over slip_window, the measured lateral acceleration falls short of the
  /// wheel speed x yaw rate by more than this. Checked only while turning
  /// faster than slip_lateral_min_yaw_rate (rad/s). TBD.
  double slip_lateral_threshold{0.5};
  double slip_lateral_min_yaw_rate{0.3};
  /// Before the slip decision (VAL-017): over the last slip_accel_span s,
  /// the wheel's acceleration is compared with the IMU's. Beyond
  /// slip_accel_margin (m/s^2, above the wheel's differenced noise) the
  /// wheel's variance grows by (slip_accel_gain x excess)^2, the speed error
  /// that mismatch could have built (s). A developing spin is distrusted
  /// gradually instead of followed until the hard decision. TBD.
  double slip_accel_span{0.2};
  /// m/s. The no-slip constraint's 1 sigma: how far the rear axle may move
  /// sideways while gripping (spec §6.1). Tight, so the filter matches the
  /// kinematic bicycle model. TBD.
  double no_slip_lateral_speed_stddev{0.02};
  /// m/s. Its 1 sigma during a slide (understeer or yaw-rate slip): loose,
  /// so v_y follows the IMU's lateral acceleration (VAL-018). TBD.
  double no_slip_slide_lateral_speed_stddev{0.5};
  double slip_accel_margin{0.6};
  double slip_accel_gain{0.5};
  /// Impact (VAL-019): a deceleration beyond slip_impact_accel (m/s^2, far
  /// beyond braking) in any step of the last slip_impact_lookback s. After
  /// one that took at least slip_pinned_fraction of the speed, with the IMU
  /// quiet and the wheel disagreeing, the car is pinned: speed held at 0,
  /// no max_duration release, until the IMU shows it has picked up
  /// slip_pinned_speed (m/s), the wheels stop, or slip_pinned_max_duration
  /// (s) passes. TBD.
  double slip_impact_accel{8.0};
  double slip_impact_lookback{0.25};
  double slip_pinned_fraction{0.3};
  double slip_pinned_speed{0.3};
  double slip_pinned_max_duration{10.0};

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
  /// The last wheel sample showed understeer (see slip_lateral_threshold).
  bool understeer_detected{false};
  /// Pinned against an obstacle after an impact (see slip_impact_accel).
  bool pinned{false};
  /// The no-slip constraint is relaxed: a slide, v_y follows the IMU.
  bool side_slip_free{false};
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
  /// history_.reference(age), but only with fresh IMU: without it the
  /// acceleration input is 0 and every real speed change would look like
  /// slip.
  std::optional<ReferenceSpeed> imu_reference_speed(double age) const;

  /// Restart the IMU-only speed from before an impact plus the IMU since.
  void restart_from_impact();
  /// On pinning: take back the distance driven since the impact.
  void take_back_distance_since_impact();

  /// Undo the position change since `before` and spread it instead.
  void spread_since(const Eigen::Vector2d & before);
  bool after_impact_{false};             ///< this slip started with or saw an impact
  bool slide_in_slip_{false};            ///< this slip included a slide
  double speed_before_impact_{0.0};      ///< m/s, the IMU path's start
  double impact_time_{0.0};              ///< s (history_.elapsed()), of the impact step
  std::optional<double> pinned_since_;   ///< when it last became pinned
  double pinned_accel_integral_{0.0};    ///< history_.accel_integral() when pinned

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
  /// Lateral acceleration, gravity removed: averaged between predicts too.
  double lateral_sum_{0.0};
  int lateral_samples_{0};
  double predict_lateral_{0.0};
  double last_wheel_speed_{0.0};
  double imu_lever_{0.0};               ///< m, rear axle to IMU, from the last sample
  bool lateral_available_{false};       ///< the last IMU sample gave lateral force

  /// The recent past the slip checks look back over (speed_history.hpp).
  SpeedHistory history_;
  /// Position corrections applied over time (spread_correction.hpp).
  SpreadCorrection spread_;
  /// While slipping: since when the wheel has agreed again (hysteresis).
  std::optional<double> agreeing_since_;
  double slip_since_{0.0};       ///< history_.elapsed() when the current slip started
  double last_yaw_rate_{0.0};    ///< rad/s, from the last IMU sample
};

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__STATE_ESTIMATION_LOGIC_HPP_
