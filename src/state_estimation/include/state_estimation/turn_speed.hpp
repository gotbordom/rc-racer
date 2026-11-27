// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// Forward speed from the turn (feature 2.5 step 2b, VAL-018; spec §6.1):
// in a slide the wheel is distrusted and the no-slip constraint relaxed, so
// nothing measures the forward speed, and its error from before detection
// just swaps with the sideways speed. The IMU can measure it while turning:
// at the IMU the lateral acceleration is v_y' + omega v_x, and in a
// sustained slide v_y changes little, so v_x ~ a_y / omega. It is a
// steady-state assumption, so it is only offered while turning hard at a
// steady yaw rate, and it is a soft measurement. This is our own
// engineering choice, not a method from the literature
// (documentation/research/vehicle-state-estimation-reading.md); the
// scenarios VAL-004/018 are what prove it. Pure C++, no ROS.

#ifndef STATE_ESTIMATION__TURN_SPEED_HPP_
#define STATE_ESTIMATION__TURN_SPEED_HPP_

#include <optional>

#include "state_estimation/speed_history.hpp"

namespace state_estimation
{

struct TurnSpeedParams
{
  /// rad/s: only while turning at least this hard (over the whole window),
  /// so a_y / omega is well conditioned (slip.turn_min_yaw_rate).
  double min_yaw_rate;
  /// rad/s: and only while the yaw rate changed less than this over the
  /// window: turn-in and turn-out break the steady-slide assumption
  /// (slip.turn_max_yaw_change).
  double max_yaw_change;
  /// 1-sigma of the measurement as an independent sample: base (m/s) plus a
  /// share of the speed (slip.turn_stddev, slip.turn_relative_stddev).
  double stddev;
  double relative_stddev;
};

struct TurnSpeed
{
  double speed;     ///< m/s, signed like the yaw rate and lateral acceleration imply
  double variance;  ///< (m/s)^2, as one independent measurement
};

/// The forward speed the turn implies, from window means since `from`, or
/// unset when the turn doesn't qualify.
std::optional<TurnSpeed> turn_speed(
  const SpeedHistory & history, const ReferenceSpeed & from, double yaw_rate,
  const TurnSpeedParams & params);

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__TURN_SPEED_HPP_
