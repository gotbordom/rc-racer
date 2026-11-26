// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Pinned against an obstacle (VAL-019, #14; VEH.GNC.SE spec Open Item 8):
// the one slip detector with memory. After an impact that took most of the
// car's speed, with the IMU quiet and the wheel still disagreeing (a slip),
// the car is stopped against something, whatever the spinning wheels say.
// It returns actions; StateEstimationLogic applies them. Pure C++, no ROS.

#ifndef STATE_ESTIMATION__PINNED_HOLD_HPP_
#define STATE_ESTIMATION__PINNED_HOLD_HPP_

#include <optional>

#include "state_estimation/speed_history.hpp"

namespace state_estimation
{

struct PinnedParams
{
  /// An impact that took at least this share of the speed can pin
  /// (slip.pinned_fraction). The IMU sees 46-87 % of a crash's speed loss,
  /// so the share, not a fixed speed, decides.
  double fraction;
  double pick_up_speed;  ///< m/s the IMU must show to leave pinned (slip.pinned_speed)
  double max_duration;   ///< s, a backstop on holding (slip.pinned_max_duration)
};

/// What the logic should do this wheel update.
struct PinnedAction
{
  bool pinned{false};
  /// Set the speed estimate to this: 0 while pinned, or the IMU's speed
  /// since pinning on leaving it.
  std::optional<double> speed;
  /// On pinning: the car stopped at the impact, so the distance driven since
  /// on the leftover IMU-only speed is error. Take it back (m, along track).
  std::optional<double> take_back;
};

class PinnedHold
{
public:
  /// An impact at `time` (history time), the speed before it `speed_before`.
  void impact(double time, double speed_before);

  /// While slipping, each wheel update. `speed` is the current estimate,
  /// `imu_quiet` whether the IMU shows the car still (yaw rate and
  /// acceleration under the stationary limits).
  ///
  /// The hold suspends the slip's max_duration release, which exists because
  /// IMU-only speed drifts: after a stop, a quiet IMU is the best evidence
  /// there is. It is decided by the impact's outcome, not its spike: a bump
  /// that lets the car carry on keeps most of its speed and never pins, and
  /// a glancing blow slows the wheels too, so it isn't a slip. It ends when
  /// the IMU shows the car has picked up pick_up_speed since pinning, or
  /// (the logic's) when the wheels stop, or after max_duration.
  PinnedAction update(
    const SpeedHistory & history, double speed, bool imu_quiet, const PinnedParams & params);

  /// The slip ended: forget it all.
  void reset();

  bool after_impact() const {return after_impact_;}
  bool pinned() const {return pinned_since_.has_value();}

private:
  bool after_impact_{false};
  double impact_time_{0.0};
  double speed_before_{0.0};
  std::optional<double> pinned_since_;
  double accel_integral_at_pin_{0.0};
};

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__PINNED_HOLD_HPP_
