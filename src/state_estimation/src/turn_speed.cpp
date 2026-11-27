// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include "state_estimation/turn_speed.hpp"

#include <algorithm>
#include <cmath>

namespace state_estimation
{

std::optional<TurnSpeed> turn_speed(
  const SpeedHistory & history, const ReferenceSpeed & from, double yaw_rate,
  const TurnSpeedParams & params)
{
  const SpeedSample & then = history.at(from.index);
  const double span = history.elapsed() - then.time;
  if (!(span > 0.0)) {
    return std::nullopt;
  }
  // Mean yaw rate over the window, and its smallest magnitude: the turn
  // must be hard the whole way, and steady.
  double yaw_sum = 0.0;
  double weight = 0.0;
  double weakest = std::abs(then.yaw_rate);
  for (std::size_t i = from.index + 1; i < history.size(); ++i) {
    const SpeedSample & s = history.at(i);
    const double dt = s.time - history.at(i - 1).time;
    yaw_sum += s.yaw_rate * dt;
    weight += dt;
    weakest = std::min(weakest, std::abs(s.yaw_rate));
  }
  if (!(weight > 0.0) || weakest < params.min_yaw_rate ||
    std::abs(yaw_rate - then.yaw_rate) > params.max_yaw_change)
  {
    return std::nullopt;
  }
  const double mean_yaw_rate = yaw_sum / weight;
  const double mean_lateral = (history.lateral_integral() - then.lateral_integral) / span;
  const double speed = mean_lateral / mean_yaw_rate;
  const double sigma = params.stddev + params.relative_stddev * std::abs(speed);
  return TurnSpeed{speed, sigma * sigma};
}

}  // namespace state_estimation
