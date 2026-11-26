// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// The recent past the slip checks look back over (VEH.GNC.SE spec §4):
// one sample per predict step, plus running integrals of the inputs, so a
// check can ask "what did the IMU say since then?" in O(1). Pure C++, no
// ROS. Owned by StateEstimationLogic; read by the checks in slip_checks.hpp.

#ifndef STATE_ESTIMATION__SPEED_HISTORY_HPP_
#define STATE_ESTIMATION__SPEED_HISTORY_HPP_

#include <cstddef>
#include <deque>
#include <optional>

namespace state_estimation
{

/// One predict step.
struct SpeedSample
{
  double time;                  ///< s since construction (elapsed)
  double speed;                 ///< estimated forward speed after this step's updates
  double speed_variance;
  double accel_integral;        ///< of the forward speed's rate, v_x' = a_x + omega v_y
  double lateral_integral;      ///< of the measured lateral acceleration
  double centripetal_integral;  ///< of wheel speed x yaw rate
  double yaw_rate;              ///< IMU, at this sample
  double wheel_speed;           ///< the last wheel reading, at this sample
};

/// A sample's speed carried forward to now by the IMU alone.
struct ReferenceSpeed
{
  double speed;
  double variance;     ///< the sample's speed variance
  std::size_t index;   ///< of the sample it starts from
};

class SpeedHistory
{
public:
  /// Keeps the newest sample at least `span` s old at the front.
  explicit SpeedHistory(double span);

  /// Advance one predict step of dt and record a sample. The rates are this
  /// step's: forward speed rate, measured lateral acceleration, and wheel
  /// speed x yaw rate.
  void advance(
    double dt, double forward_rate, double lateral_accel, double centripetal,
    double speed, double speed_variance, double yaw_rate, double wheel_speed);

  /// Overwrite the newest sample's speed (after this step's wheel update).
  void update_latest(double speed, double speed_variance);

  /// The newest sample at least `age` old, carried forward by the IMU;
  /// unset until the history spans `age`.
  std::optional<ReferenceSpeed> reference(double age) const;

  /// Mean forward speed rate from sample `from` to now (0 if no time).
  double mean_accel_since(std::size_t from) const;

  /// Samples before `from` hold speeds the IMU path from it contradicts:
  /// rewrite them as its speed minus the IMU integral back to them.
  void rebase_before(std::size_t from);

  /// Rewrite the samples after `from` as the IMU path from it. Returns the
  /// distance (m) the old speeds added on top of that path.
  double rewrite_as_imu_path(std::size_t from);

  /// Distance (m) the recorded speeds cover after `time`.
  double distance_after(double time) const;

  /// Time of the first step after sample `from` whose forward rate,
  /// along `direction` (+1 or -1), is below -decel. Unset if none.
  std::optional<double> first_step_beyond(std::size_t from, double decel, double direction) const;

  /// Forget the samples (the integrals and time run on).
  void clear() {samples_.clear();}

  bool empty() const {return samples_.empty();}
  std::size_t size() const {return samples_.size();}
  const SpeedSample & at(std::size_t i) const {return samples_[i];}
  double elapsed() const {return elapsed_;}
  double accel_integral() const {return accel_integral_;}
  double lateral_integral() const {return lateral_integral_;}
  double centripetal_integral() const {return centripetal_integral_;}

private:
  double span_;
  std::deque<SpeedSample> samples_;
  double elapsed_{0.0};
  double accel_integral_{0.0};
  double lateral_integral_{0.0};
  double centripetal_integral_{0.0};
};

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__SPEED_HISTORY_HPP_
