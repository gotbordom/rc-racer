# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Unit tests for scripts/steering_relay.py (SDP §3.6 VAL-014).

The drive turns a yaw rate into a steering angle with
delta = atan(L * w / v) (ICD-GNC-ACT GNC-ACT-01), so these check the angle
the drive will steer for each stick fraction and speed.
"""

import math

import pytest

from steering_relay import yaw_rate

WHEELBASE = 0.384           # m, ICD-GNC-ACT
MAX_STEERING_ANGLE = 0.52   # rad, ICD-GNC-ACT
SPEEDS = [0.3, 1.0, 3.0, 5.0, -0.5, -2.0]


def steering_angle(speed, fraction):
    """Return the angle the drive steers for the relay's yaw rate."""
    w = yaw_rate(speed, fraction, WHEELBASE, MAX_STEERING_ANGLE)
    # For v < 0, atan(L * w / v) gives the wheel angle, just as for v > 0.
    return math.atan(WHEELBASE * w / speed)


@pytest.mark.parametrize('speed', SPEEDS)
def test_full_stick_is_full_lock_at_any_speed(speed):
    """Full stick steers to the limit, both ways, forward and in reverse."""
    assert steering_angle(speed, 1.0) == pytest.approx(MAX_STEERING_ANGLE)
    assert steering_angle(speed, -1.0) == pytest.approx(-MAX_STEERING_ANGLE)


@pytest.mark.parametrize('speed', SPEEDS)
@pytest.mark.parametrize('fraction', [-0.75, -0.25, 0.0, 0.5, 0.9])
def test_angle_is_the_stick_fraction_of_full_lock(speed, fraction):
    """The steering angle does not depend on speed."""
    assert steering_angle(speed, fraction) == pytest.approx(fraction * MAX_STEERING_ANGLE)


def test_reverse_turns_the_other_way():
    """The wheels point where the stick does; in reverse the car yaws the other way."""
    forward = yaw_rate(1.0, 0.5, WHEELBASE, MAX_STEERING_ANGLE)
    reverse = yaw_rate(-1.0, 0.5, WHEELBASE, MAX_STEERING_ANGLE)
    assert forward > 0.0
    assert reverse == pytest.approx(-forward)


@pytest.mark.parametrize('fraction', [1.5, 3.0])
def test_fraction_beyond_full_stick_is_clamped(fraction):
    """A teleop scale above 1 cannot command past the steering limit."""
    assert steering_angle(2.0, fraction) == pytest.approx(MAX_STEERING_ANGLE)
    assert steering_angle(2.0, -fraction) == pytest.approx(-MAX_STEERING_ANGLE)


def test_no_yaw_rate_when_stopped():
    """An Ackermann car cannot turn in place, so v = 0 sends no yaw rate."""
    assert yaw_rate(0.0, 1.0, WHEELBASE, MAX_STEERING_ANGLE) == 0.0
