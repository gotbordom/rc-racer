// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include "state_estimation/speed_history.hpp"

namespace state_estimation
{

SpeedHistory::SpeedHistory(double span)
: span_(span) {}

void SpeedHistory::advance(
  double dt, double forward_rate, double lateral_accel, double centripetal,
  double speed, double speed_variance, double yaw_rate, double wheel_speed)
{
  elapsed_ += dt;
  accel_integral_ += forward_rate * dt;
  lateral_integral_ += lateral_accel * dt;
  centripetal_integral_ += centripetal * dt;
  samples_.push_back(
    {elapsed_, speed, speed_variance, accel_integral_, lateral_integral_, centripetal_integral_,
      yaw_rate, wheel_speed});
  // Keep the newest sample that is at least span old at the front.
  while (samples_.size() > 1 && samples_[1].time <= elapsed_ - span_) {
    samples_.pop_front();
  }
}

void SpeedHistory::update_latest(double speed, double speed_variance)
{
  if (!samples_.empty()) {
    samples_.back().speed = speed;
    samples_.back().speed_variance = speed_variance;
  }
}

std::optional<ReferenceSpeed> SpeedHistory::reference(double age) const
{
  if (samples_.empty() || elapsed_ - samples_.front().time < age) {
    return std::nullopt;
  }
  std::size_t i = 0;
  while (i + 1 < samples_.size() && samples_[i + 1].time <= elapsed_ - age) {
    ++i;
  }
  const SpeedSample & then = samples_[i];
  return ReferenceSpeed{
    then.speed + (accel_integral_ - then.accel_integral), then.speed_variance, i};
}

double SpeedHistory::mean_accel_since(std::size_t from) const
{
  const SpeedSample & then = samples_[from];
  const double span = elapsed_ - then.time;
  return span > 0.0 ? (accel_integral_ - then.accel_integral) / span : 0.0;
}

void SpeedHistory::rebase_before(std::size_t from)
{
  const SpeedSample anchor = samples_[from];
  for (std::size_t i = 0; i < from; ++i) {
    SpeedSample & s = samples_[i];
    s.speed = anchor.speed - (anchor.accel_integral - s.accel_integral);
    s.speed_variance = anchor.speed_variance;
  }
}

double SpeedHistory::rewrite_as_imu_path(std::size_t from)
{
  const SpeedSample start = samples_[from];
  double distance = 0.0;
  for (std::size_t i = from + 1; i < samples_.size(); ++i) {
    SpeedSample & s = samples_[i];
    const double imu_path = start.speed + (s.accel_integral - start.accel_integral);
    distance += (s.speed - imu_path) * (s.time - samples_[i - 1].time);
    s.speed = imu_path;
    s.speed_variance = start.speed_variance;
  }
  return distance;
}

double SpeedHistory::distance_after(double time) const
{
  double distance = 0.0;
  for (std::size_t i = 1; i < samples_.size(); ++i) {
    const SpeedSample & s = samples_[i];
    if (s.time > time) {
      distance += s.speed * (s.time - samples_[i - 1].time);
    }
  }
  return distance;
}

std::optional<double> SpeedHistory::first_step_beyond(
  std::size_t from, double decel, double direction) const
{
  for (std::size_t i = from + 1; i < samples_.size(); ++i) {
    const SpeedSample & a = samples_[i - 1];
    const SpeedSample & b = samples_[i];
    const double dt = b.time - a.time;
    if (dt > 0.0 && (b.accel_integral - a.accel_integral) / dt * direction < -decel) {
      return b.time;
    }
  }
  return std::nullopt;
}

}  // namespace state_estimation
