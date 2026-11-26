// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include "state_estimation/pinned_hold.hpp"

#include <cmath>

namespace state_estimation
{

void PinnedHold::impact(double time, double speed_before)
{
  after_impact_ = true;
  impact_time_ = time;
  speed_before_ = speed_before;
  pinned_since_.reset();
}

PinnedAction PinnedHold::update(
  const SpeedHistory & history, double speed, bool imu_quiet, const PinnedParams & params)
{
  PinnedAction action;
  const bool stopped_by_impact = after_impact_ && imu_quiet &&
    std::abs(speed) < (1.0 - params.fraction) * std::abs(speed_before_);
  if (pinned_since_) {
    const double picked_up = history.accel_integral() - accel_integral_at_pin_;
    if (std::abs(picked_up) >= params.pick_up_speed) {
      // Moving again: an ordinary slip from here, on the IMU's speed.
      action.speed = picked_up;
      after_impact_ = false;
      pinned_since_.reset();
      return action;
    }
    action.pinned = history.elapsed() - *pinned_since_ < params.max_duration;
  } else if (stopped_by_impact) {
    pinned_since_ = history.elapsed();
    accel_integral_at_pin_ = history.accel_integral();
    action.pinned = true;
    action.take_back = history.distance_after(impact_time_);
  }
  if (action.pinned) {
    action.speed = 0.0;
  }
  return action;
}

void PinnedHold::reset()
{
  after_impact_ = false;
  pinned_since_.reset();
}

}  // namespace state_estimation
