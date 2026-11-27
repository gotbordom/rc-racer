// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include "state_estimation/slip_checks.hpp"

#include <cmath>

namespace state_estimation
{

WindowCheck window_check(
  const SpeedHistory & history, const std::optional<ReferenceSpeed> & reference,
  const std::optional<ReferenceSpeed> & recent, double wheel_speed,
  const WindowCheckParams & params)
{
  WindowCheck result;
  if (!reference) {
    return result;
  }
  const double direction = reference->speed >= 0.0 ? 1.0 : -1.0;
  const bool wheel_behind = (reference->speed - wheel_speed) * direction > 0.0;
  const double recent_accel =
    history.mean_accel_since(recent ? recent->index : reference->index);
  const bool braking = recent_accel * direction < -params.braking_accel;
  const double threshold = wheel_behind && braking ? params.braking_threshold : params.threshold;
  result.gap = std::abs(wheel_speed - reference->speed) >
    threshold + 3.0 * std::sqrt(reference->variance);
  result.consistent = !wheel_behind || braking;
  return result;
}

double early_distrust_variance(
  const SpeedHistory & history, const ReferenceSpeed & from, double wheel_speed,
  const EarlyDistrustParams & params)
{
  const SpeedSample & then = history.at(from.index);
  const double dt = history.elapsed() - then.time;
  if (!(dt > 0.0)) {
    return 0.0;
  }
  const double wheel_accel = (wheel_speed - then.wheel_speed) / dt;
  const double imu_accel = history.mean_accel_since(from.index);
  const double excess = std::abs(wheel_accel - imu_accel) - params.margin;
  return excess > 0.0 ? std::pow(params.gain * excess, 2) : 0.0;
}

bool understeer(
  const SpeedHistory & history, const ReferenceSpeed & from, double yaw_rate, double imu_lever,
  const UndersteerParams & params)
{
  const SpeedSample & then = history.at(from.index);
  const double span = history.elapsed() - then.time;
  const bool turning = std::abs(yaw_rate) > params.min_yaw_rate &&
    std::abs(then.yaw_rate) > params.min_yaw_rate;
  if (!turning || !(span > 0.0)) {
    return false;
  }
  const double measured = (history.lateral_integral() - then.lateral_integral) / span;
  const double expected = (history.centripetal_integral() - then.centripetal_integral) / span +
    imu_lever * (yaw_rate - then.yaw_rate) / span;
  const double direction = expected >= 0.0 ? 1.0 : -1.0;
  return (expected - measured) * direction > params.threshold;
}

std::optional<double> impact(
  const SpeedHistory & history, const ReferenceSpeed & from, double decel)
{
  const double direction = history.at(from.index).speed >= 0.0 ? 1.0 : -1.0;
  return history.first_step_beyond(from.index, decel, direction);
}

}  // namespace state_estimation
