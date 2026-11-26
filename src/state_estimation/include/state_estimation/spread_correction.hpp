// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// A position change applied over time instead of at once, so odom stays
// continuous (REP-105: control runs on it). The slip handling makes several
// position corrections (VEH.GNC.SE spec §4, 2.4/2.5): a caught launch slip,
// a pinned car's distance since the impact, a slip's end, a slide's end.
// Each is right but, applied in one step, would jump odom. Pure C++, no ROS.

#ifndef STATE_ESTIMATION__SPREAD_CORRECTION_HPP_
#define STATE_ESTIMATION__SPREAD_CORRECTION_HPP_

#include <Eigen/Dense>

#include <algorithm>

namespace state_estimation
{

class SpreadCorrection
{
public:
  /// Each add() is spread over `duration` s (slip.correction_time).
  explicit SpreadCorrection(double duration)
  : duration_(duration) {}

  /// Add a position change (m, odom) to apply. Whatever is still pending is
  /// spread over a fresh `duration` with it.
  void add(const Eigen::Vector2d & delta)
  {
    pending_ += delta;
    time_left_ = duration_;
  }

  /// This step's share of the pending change (m): add it to the position.
  Eigen::Vector2d step(double dt)
  {
    if (time_left_ <= 0.0) {
      return Eigen::Vector2d::Zero();
    }
    const Eigen::Vector2d share = std::min(1.0, dt / time_left_) * pending_;
    pending_ -= share;
    time_left_ -= dt;
    return share;
  }

  void clear()
  {
    pending_.setZero();
    time_left_ = 0.0;
  }

  const Eigen::Vector2d & pending() const {return pending_;}

private:
  double duration_;
  Eigen::Vector2d pending_{Eigen::Vector2d::Zero()};
  double time_left_{0.0};
};

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__SPREAD_CORRECTION_HPP_
