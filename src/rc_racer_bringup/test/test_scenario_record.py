# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Unit tests for scripts/scenario_record.py and scenario_replay.py (SDP §3.6).

A recorded drive must come back as the same commands: the recorder turns
/cmd_vel into scenario steps, and the replay (like the scenario test) turns
the steps back into /cmd_vel through steering_relay's yaw_rate().
"""

import math

import pytest
import yaml

from scenario_record import scenario_text, steering_fraction, to_steps
from scenario_replay import command_at
from steering_relay import yaw_rate

L = 0.384
MAX = 0.52


@pytest.mark.parametrize('speed', [0.5, 3.0, -1.0, -2.5])
@pytest.mark.parametrize('fraction', [-1.0, -0.4, 0.0, 0.3, 1.0])
def test_steering_fraction_inverts_the_relay(speed, fraction):
    """The recorder recovers the stick fraction the relay sent, at any speed."""
    w = yaw_rate(speed, fraction, L, MAX)
    assert steering_fraction(speed, w, L, MAX) == pytest.approx(fraction)


def test_stopped_has_no_steering():
    """At v = 0 the yaw rate carries no steering (and the keyboard may send some)."""
    assert steering_fraction(0.0, 1.0, L, MAX) == 0.0


def test_keyboard_yaw_rate_beyond_lock_is_clamped():
    """A yaw rate the steering cannot make is full lock."""
    assert steering_fraction(1.0, 10.0, L, MAX) == 1.0
    assert steering_fraction(1.0, -10.0, L, MAX) == -1.0


def commands_from(segments, period=0.05, t0=100.0):
    """Commands every period for [(duration, speed, fraction)] segments."""
    out, t = [], t0
    for duration, v, f in segments:
        for _ in range(round(duration / period)):
            out.append((t, v, yaw_rate(v, f, L, MAX)))
            t += period
    return out, t


def test_steady_stretches_become_single_steps():
    """20 Hz commands of a few steady stretches give one step each, plus a stop."""
    commands, end = commands_from([(2.0, 1.0, 0.0), (3.0, 1.0, 1.0), (1.5, -0.5, -0.5)])
    steps = to_steps(commands, end, L, MAX)
    assert [(s['speed'], s['steer']) for s in steps] == [
        (1.0, 0.0), (1.0, 1.0), (-0.5, -0.5), (0.0, 0.0)]
    assert [s['duration'] for s in steps[:3]] == pytest.approx([2.0, 3.0, 1.5])


def test_noise_within_tolerance_is_merged():
    """A pad's stick jitter does not split a step."""
    commands, end = commands_from([(1.0, 1.0, 0.3)])
    jittered = [(t, v + (0.01 if i % 2 else -0.01), w) for i, (t, v, w) in enumerate(commands)]
    steps = to_steps(jittered, end, L, MAX)
    assert len(steps) == 2  # the drive and the final stop
    assert steps[0]['duration'] == pytest.approx(1.0)


def test_last_command_holds_until_the_end():
    """Gazebo holds the last command, so the last step lasts until recording stopped."""
    steps = to_steps([(10.0, 2.0, 0.0)], 14.0, L, MAX)
    assert steps[0] == {'duration': 4.0, 'speed': 2.0, 'steer': 0.0}


def test_total_time_is_kept():
    """Merging and short-step handling keep the drive's total duration."""
    commands, end = commands_from([(0.05, 0.4, 0.0), (1.0, 1.0, 0.0), (0.05, 1.5, 0.2),
                                   (2.5, 0.0, 0.0)])
    steps = to_steps(commands, end, L, MAX)
    assert sum(s['duration'] for s in steps) == pytest.approx(end - commands[0][0])


def test_a_drive_ending_stopped_keeps_one_final_stop():
    """A stop at the end is extended to 2 s, not followed by another."""
    short, end = commands_from([(1.0, 1.0, 0.0), (0.5, 0.0, 0.3)])
    steps = to_steps(short, end, L, MAX)
    assert steps[-1] == {'duration': 2.0, 'speed': 0.0, 'steer': 0.0}
    assert len(steps) == 2


def test_written_file_is_a_valid_scenario_that_replays_the_drive():
    """The file loads as a scenario, and replaying it gives back the commands."""
    segments = [(2.0, 1.0, 0.0), (3.0, 1.0, 1.0), (1.5, -0.5, -0.5)]
    commands, end = commands_from(segments)
    meta = {'recorded': '2026-11-23', 'samples': len(commands), 'duration': end - 100.0,
            'world': 'practice_course', 'tire_mu': 0.5, 'start': (0.0, 0.0, 0.0),
            'id': 'VAL-007', 'title': 'Bench: driven into it'}
    scenario = yaml.safe_load(scenario_text(to_steps(commands, end, L, MAX), meta))
    assert scenario['id'] == 'VAL-007'
    assert scenario['title'] == 'Bench: driven into it'
    assert scenario['world'] == 'practice_course.world'
    assert scenario['tire_mu'] == 0.5
    assert set(scenario['checks']) == {'never_error', 'ends_ok', 'settles'}
    for t, v, w in commands:
        speed, steer = command_at(scenario['steps'], t - 100.0 + 1e-6)
        assert speed == pytest.approx(v)
        assert yaw_rate(speed, steer, L, MAX) == pytest.approx(w, abs=0.02)
    assert command_at(scenario['steps'], end - 100.0 + 0.5) == (0.0, 0.0)  # final stop
    assert command_at(scenario['steps'], end - 100.0 + 10.0) is None


def test_replay_ramps_like_the_scenario_test():
    """speed_end ramps linearly over the step."""
    steps = [{'duration': 2.0, 'speed': 0.0, 'speed_end': 2.0}]
    assert command_at(steps, 0.5)[0] == pytest.approx(0.5)
    assert command_at(steps, 1.5)[0] == pytest.approx(1.5)
    assert math.isclose(command_at(steps, 0.0)[1], 0.0)
