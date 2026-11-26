// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// The slip checks (VEH.GNC.SE spec §4, REQ-GNC-005), as pure functions: each
// reads the SpeedHistory and the latest readings and returns a decision.
// None changes the filter: StateEstimationLogic combines the decisions and
// acts on them. Each takes only the parameters it uses, so what a check
// depends on is in its signature. Pure C++, no ROS.

#ifndef STATE_ESTIMATION__SLIP_CHECKS_HPP_
#define STATE_ESTIMATION__SLIP_CHECKS_HPP_

#include <optional>

#include "state_estimation/speed_history.hpp"

namespace state_estimation
{

// ---------------------------------------------------------------------------
// Windowed check: the wheel against the speed slip_window ago carried
// forward by the IMU alone. Catches slip that builds over several steps,
// which the one-step check misses because each update re-anchors the
// estimate to the slipping wheel.
// ---------------------------------------------------------------------------

struct WindowCheckParams
{
  double threshold;          ///< m/s on top of 3 sigma (slip.threshold)
  double braking_accel;      ///< m/s^2 of IMU deceleration that is braking
  double braking_threshold;  ///< m/s, for a wheel behind while braking
};

struct WindowCheck
{
  bool gap{false};         ///< the wheel and the reference disagree
  bool consistent{true};   ///< the disagreement is physically possible as slip
  bool slip() const {return gap && consistent;}
  /// The reference is what's wrong (an earlier slip too small to catch left
  /// the history following the wheel): rebuild the history, don't call slip.
  bool stale() const {return gap && !consistent;}
};

/// `reference` is the history's reference over slip_window (unset without
/// fresh IMU), `recent` its reference over half that.
///
/// The reference trusts the estimate at the window start. After an earlier
/// slip too small to catch, it may not be right: the estimate followed the
/// wheel, and the reference then blames a correct wheel (2026-11-24, L2 sim
/// loop: a launch led the wheel 0.29 m/s ahead, undetected, and half a
/// second later the check "corrected" a right speed 0.33 m/s too high). The
/// sign tells them apart: a driven wheel can only fall behind the car while
/// braking (or locked, which is braking too). So a wheel behind the
/// reference is slip only with braking on the IMU over the recent half
/// window. A wheel ahead is not restricted: spin on launch, a pinned car and
/// understeer all put it ahead.
///
/// Behind while braking is also the one case with a tighter threshold: a
/// braking slide builds slowly (VAL-006: the wheel slows at the drive's
/// 3 m/s^2, the car at ~2.5 on mu 0.5), and a normal hard stop, wheel and car
/// slowing together, gives no gap at all.
WindowCheck window_check(
  const SpeedHistory & history, const std::optional<ReferenceSpeed> & reference,
  const std::optional<ReferenceSpeed> & recent, double wheel_speed,
  const WindowCheckParams & params);

// ---------------------------------------------------------------------------
// Early distrust (VAL-017, #9): before the slip decision, a wheel out-
// accelerating (or out-braking) the car is trusted less in proportion, so
// the estimate doesn't follow a spin that's still under the slip threshold.
// A matched hard acceleration leaves it alone.
// ---------------------------------------------------------------------------

struct EarlyDistrustParams
{
  double margin;  ///< m/s^2 of mismatch tolerated (slip.accel_margin)
  double gain;    ///< s (slip.accel_gain)
};

/// Extra wheel variance, (gain x excess mismatch)^2, comparing the wheel's
/// and the IMU's acceleration since `from` (over slip.accel_span).
double early_distrust_variance(
  const SpeedHistory & history, const ReferenceSpeed & from, double wheel_speed,
  const EarlyDistrustParams & params);

// ---------------------------------------------------------------------------
// Understeer (VAL-004): the car slides wide with its yaw rate under what the
// steering allows, so the yaw-rate check can't fire. In a turn without slip
// the lateral acceleration at the IMU is yaw rate x speed, plus lever x yaw
// acceleration; sliding wide, the wheel claims more speed than the turn
// shows. Compared as means over the window, so noise averages out and the
// lever term is exact (lever x the yaw rate's change). One-sided: only a
// shortfall counts.
// ---------------------------------------------------------------------------

struct UndersteerParams
{
  double threshold;     ///< m/s^2 of shortfall (slip.lateral_threshold)
  double min_yaw_rate;  ///< rad/s; checked only when turning faster
};

/// From `from` (the window start) to now. `imu_lever` is the IMU's distance
/// ahead of the rear axle (m).
bool understeer(
  const SpeedHistory & history, const ReferenceSpeed & from, double yaw_rate, double imu_lever,
  const UndersteerParams & params);

// ---------------------------------------------------------------------------
// Impact (VAL-019, #14): a deceleration far beyond what braking can do, in
// any step since `from` (slip.impact_lookback). The sim IMU shows one since
// DEV-07 (~-50 m/s^2 into the bench); the drive brakes at 3.
// ---------------------------------------------------------------------------

/// The time of the impact step, or unset.
std::optional<double> impact(
  const SpeedHistory & history, const ReferenceSpeed & from, double decel);

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__SLIP_CHECKS_HPP_
