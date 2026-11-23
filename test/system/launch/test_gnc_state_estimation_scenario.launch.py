# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Scenario test: state_estimation on one scripted validation case (SDP §3.6).

Runs the scenario file named by the SCENARIO_FILE environment variable
(test/system/scenarios/<name>.yaml; CMake adds one test per file). Each run
starts its own headless Gazebo on the empty course at the scenario's
tire_mu, so one scenario cannot disturb the next.

The car is driven open-loop by the scenario's steps through the same
steering_relay the pad uses: speed (m/s, optionally ramped) and steer (a
fraction of full lock, + left). The estimate on /odom is compared with
Gazebo's ground truth (/sim/truth/poses), and the scenario's checks are
applied. Checks test properties (error bounds, covariance honesty, slip
seen or not, health), not an exact trajectory.

state_estimation reports health at 10 Hz here instead of 1 Hz
(diagnostic_updater.period, set for this test only), so slip detection can
be timed to about 0.1 s.

A scenario marked expected_fail documents a known failure. It names the
check that fails (expected_fail.failing, matched against the failure
text): the test passes while that check fails and nothing else does, and
fails once it passes, so the mark gets removed when the bug is fixed.

Characteristics:
  - SLOW: ~1 minute each (Gazebo startup + the scenario's sim time)

CI/CD: Nightly
"""

import math
import os
import tempfile
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from se_sim import compare, distance_travelled, SimRecorder

COMMAND_PERIOD_S = 0.05     # sim; 20 Hz commands, as the pad sends
STOP_S = 1.0                # sim; keep recording after the last step
DIAGNOSTICS_PERIOD_S = 0.1  # sim; health report period in these runs
DIAGNOSTICS_LAG_S = 1.5 * DIAGNOSTICS_PERIOD_S
MAX_STEP = 0.1              # m between consecutive /odom samples (no jumps)
STEP_KEYS = {
    'duration', 'speed', 'speed_end', 'steer', 'expect_slip', 'expect_slip_within',
    'min_consistent_fraction',
}
CHECK_KEYS = {
    'max_position_error', 'final_position_error', 'max_yaw_error_deg',
    'min_consistent_fraction', 'slip_events', 'never_error', 'ends_ok', 'settles',
    'covariance_dip',
}


def load_scenario(path):
    """Load and validate a scenario file; raise ValueError if malformed."""
    with open(path) as f:
        scenario = yaml.safe_load(f)
    for key in ('id', 'title', 'tire_mu', 'steps', 'checks'):
        if key not in scenario:
            raise ValueError(f'{path}: missing {key!r}')
    for i, step in enumerate(scenario['steps']):
        unknown = set(step) - STEP_KEYS
        if unknown or 'duration' not in step or 'speed' not in step:
            raise ValueError(f'{path}: step {i} needs duration and speed; unknown {unknown}')
        if abs(step.get('steer', 0.0)) > 1.0:
            raise ValueError(f'{path}: step {i} steer must be in [-1, 1]')
    unknown = set(scenario['checks']) - CHECK_KEYS
    if unknown:
        raise ValueError(f'{path}: unknown checks {unknown}')
    expected_fail = scenario.get('expected_fail')
    if expected_fail is not None and set(expected_fail) != {'reason', 'failing'}:
        raise ValueError(f'{path}: expected_fail needs reason and failing')
    return scenario


SCENARIO_FILE = os.environ.get('SCENARIO_FILE', '')
SCENARIO = load_scenario(SCENARIO_FILE)
NAME = os.path.splitext(os.path.basename(SCENARIO_FILE))[0]
# ctest runs in the package's build directory
RESULTS_DIR = os.path.join('test_results', 'rc_racer_system_tests')
TRACE_FILE = os.path.join(RESULTS_DIR, f'scenario_{NAME}_trace.csv')
HEALTH_FILE = os.path.join(RESULTS_DIR, f'scenario_{NAME}_health.csv')


def params_with_fast_diagnostics():
    """Write state_estimation's default params plus a 10 Hz health period; return the path."""
    default = os.path.join(
        get_package_share_directory('state_estimation'), 'config', 'state_estimation.yaml')
    with open(default) as f:
        params = yaml.safe_load(f)
    params['state_estimation']['ros__parameters']['diagnostic_updater.period'] = \
        DIAGNOSTICS_PERIOD_S
    out = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix='se_params_', delete=False)
    with out:
        yaml.safe_dump(params, out)
    return out.name


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the empty course at the scenario's tire_mu, state_estimation and the relay."""
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rc_racer_simulation'), 'launch', 'simulation.launch.py')),
        launch_arguments={
            'headless': 'true',
            'world': 'empty_course.world',
            'tire_mu': str(SCENARIO['tire_mu']),
        }.items(),
    )
    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('state_estimation'), 'launch',
            'state_estimation.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_with_fast_diagnostics(),
        }.items(),
    )
    steering_relay = launch_ros.actions.Node(
        package='rc_racer_bringup',
        executable='steering_relay.py',
        name='steering_relay',
        parameters=[os.path.join(
            get_package_share_directory('rc_racer_bringup'), 'config', 'xbox_teleop.yaml')],
    )
    return launch.LaunchDescription([
        simulation,
        state_estimation,
        steering_relay,
        launch_testing.actions.ReadyToTest(),
    ])


def drive(sim, steps):
    """Run the steps; return [(start, end)] sim times for each step."""
    windows = []
    for step in steps:
        start = sim.now()
        end = start + step['duration']
        v0 = step['speed']
        v1 = step.get('speed_end', v0)
        while sim.now() < end:
            f = (sim.now() - start) / step['duration']
            sim.command(v0 + f * (v1 - v0), step.get('steer', 0.0))
            sim.sleep_sim(COMMAND_PERIOD_S)
        windows.append((start, sim.now()))
    sim.command(0.0, 0.0)
    sim.sleep_sim(STOP_S)
    return windows


def first_slip_after(health, start):
    """Return the sim time of the first report counting a new slip at or after start."""
    before = [h.slip_events for h in health if h.t < start]
    baseline = before[-1] if before else 0
    return next((h.t for h in health if h.t >= start and h.slip_events > baseline), None)


def slip_events_between(health, start, end):
    """Slip events counted by health reports in [start, end]."""
    before = [h.slip_events for h in health if h.t < start]
    during = [h.slip_events for h in health if start <= h.t <= end]
    if not during:
        return 0
    return max(during) - (before[-1] if before else 0)


def run_checks(checks, steps, windows, samples, health):
    """Apply the scenario's checks; return (metrics lines, failures)."""
    metrics, failures = [], []

    def check(ok, what):
        if not ok:
            failures.append(what)

    max_error = max(s.error for s in samples)
    last = samples[-1]
    max_yaw = math.degrees(max(s.yaw_error for s in samples))
    consistent = sum(s.consistent for s in samples) / len(samples)
    max_step = max(math.hypot(b.est[0] - a.est[0], b.est[1] - a.est[1])
                   for a, b in zip(samples, samples[1:]))
    total_slips = slip_events_between(health, samples[0].t, samples[-1].t + DIAGNOSTICS_LAG_S)
    errors = [h.message for h in health if h.is_error]
    metrics += [
        f'distance travelled      {distance_travelled(samples):8.2f} m',
        f'max position error      {max_error:8.3f} m',
        f'final position error    {last.error:8.3f} m',
        f'max yaw error           {max_yaw:8.2f} deg',
        f'inside 3-sigma ellipse  {100.0 * consistent:8.1f} %',
        f'max step between msgs   {max_step:8.3f} m',
        f'final speed estimate    {last.v:+8.3f} m/s',
        f'slip events             {total_slips:8d}',
        f'diagnostics ERRORs      {len(errors):8d}',
        f'final health            {health[-1].message if health else "none"}',
    ]
    check(max_step <= MAX_STEP, f'estimate jumped {max_step:.3f} m between samples')

    if 'max_position_error' in checks:
        limit = checks['max_position_error']
        check(max_error <= limit, f'max position error {max_error:.3f} m > {limit} m')
    if 'final_position_error' in checks:
        limit = checks['final_position_error']
        check(last.error <= limit, f'final position error {last.error:.3f} m > {limit} m')
    if 'max_yaw_error_deg' in checks:
        limit = checks['max_yaw_error_deg']
        check(max_yaw <= limit, f'max yaw error {max_yaw:.2f} deg > {limit} deg')
    if 'min_consistent_fraction' in checks:
        limit = checks['min_consistent_fraction']
        check(consistent >= limit,
              f'only {100.0 * consistent:.1f} % inside 3 sigma (< {100.0 * limit:.0f} %)')
    if 'slip_events' in checks:
        bounds = checks['slip_events']
        if 'max' in bounds:
            check(total_slips <= bounds['max'],
                  f'{total_slips} slip events (max {bounds["max"]}): false slip?')
        if 'min' in bounds:
            check(total_slips >= bounds['min'], f'{total_slips} slip events (min {bounds["min"]})')
    for i, (step, (start, end)) in enumerate(zip(steps, windows)):
        if step.get('expect_slip') or 'expect_slip_within' in step:
            first = first_slip_after(health, start)
            seen = first is not None and first <= end + DIAGNOSTICS_LAG_S
            metrics.append(f'slip seen in step {i}      '
                           + (f'{first - start:8.2f} s after it began' if seen else '      no'))
            check(seen, f'no slip in step {i} detected ({start:.1f}-{end:.1f} s)')
            if seen and 'expect_slip_within' in step:
                limit = step['expect_slip_within']
                check(first - start <= limit + DIAGNOSTICS_LAG_S,
                      f'slip in step {i} detected {first - start:.2f} s after it began '
                      f'(limit {limit} s)')
        if 'min_consistent_fraction' in step:
            during = [s for s in samples if start <= s.t <= end]
            fraction = sum(s.consistent for s in during) / max(len(during), 1)
            metrics.append(f'inside 3-sigma, step {i}  {100.0 * fraction:8.1f} %')
            limit = step['min_consistent_fraction']
            check(fraction >= limit, f'step {i}: only {100.0 * fraction:.1f} % inside 3 sigma '
                  f'(< {100.0 * limit:.0f} %)')
    if checks.get('never_error'):
        check(not errors, f'state_estimation reported ERROR: {errors[:3]}')
    if checks.get('ends_ok'):
        check(bool(health) and health[-1].is_ok,
              f'health did not end OK: {health[-1].message if health else "none"}')
    if checks.get('settles'):
        check(abs(last.v) < 0.05, f'speed estimate did not settle to 0: {last.v:+.3f} m/s')
    if 'covariance_dip' in checks:
        # Out and back (VAL-016): the position variance falls for a while on
        # the way back, then grows again.
        variance = [s.var_x + s.var_y for s in samples]
        turn = max(range(len(samples)), key=lambda i: math.hypot(*samples[i].truth[:2]))
        lowest = min(range(turn, len(samples)), key=lambda i: variance[i])
        drop = 1.0 - variance[lowest] / variance[turn]
        metrics += [
            f'variance at the turn    {variance[turn]:8.5f} m^2',
            f'lowest on the way back  {variance[lowest]:8.5f} m^2 ({100.0 * drop:.1f} % lower,'
            f' {math.hypot(*samples[lowest].truth[:2]):.1f} m from start)',
            f'variance at the end     {variance[-1]:8.5f} m^2',
        ]
        min_drop = checks['covariance_dip']['min_drop']
        check(drop >= min_drop, f'variance fell {100.0 * drop:.1f} % (< {100.0 * min_drop:.0f} %)')
        check(variance[-1] > variance[lowest], 'variance did not grow again after its low')
    return metrics, failures


class TestScenario(unittest.TestCase):
    """Drive the scenario and apply its checks."""

    @classmethod
    def setUpClass(cls):
        """Create the sim-time recorder; commands go through steering_relay."""
        rclpy.init()
        cls.sim = SimRecorder('test_gnc_state_estimation_scenario',
                              cmd_topic='/steering_relay/input')

    @classmethod
    def tearDownClass(cls):
        """Stop the car and shut rclpy down."""
        cls.sim.shutdown()
        rclpy.shutdown()

    def test_scenario(self):
        """Drive the scenario's steps; its checks hold (or fail, if expected_fail)."""
        sim = self.sim
        sim.wait_for_sim()
        origin = sim.reset_odom()
        sim.start_recording()
        windows = drive(sim, SCENARIO['steps'])
        history, odom, health = sim.stop_recording()

        samples = compare(odom, history, origin)
        self.assertGreater(len(samples), 100, 'too few /odom samples')
        metrics, failures = run_checks(
            SCENARIO['checks'], SCENARIO['steps'], windows, samples, health)
        try:
            os.makedirs(os.path.dirname(TRACE_FILE), exist_ok=True)
            with open(TRACE_FILE, 'w') as f:
                f.write('t,true_x,true_y,true_yaw,est_x,est_y,est_yaw,est_v,est_w,var_x,var_y\n')
                for s in samples:
                    f.write(','.join(f'{v:.5f}' for v in (
                        s.t, *s.truth, *s.est, s.v, s.w, s.var_x, s.var_y)) + '\n')
            with open(HEALTH_FILE, 'w') as f:
                f.write('t,level,slip_detected,slip_events,message\n')
                for h in health:
                    f.write(f'{h.t:.3f},{h.level[0] if isinstance(h.level, bytes) else h.level},'
                            f'{h.values.get("slip_detected", "")},{h.slip_events},'
                            f'"{h.message}"\n')
        except OSError as e:
            print(f'could not write trace: {e}')

        expected_fail = SCENARIO.get('expected_fail')
        print(f'\n  {SCENARIO["id"]} {NAME}: {SCENARIO["title"]} (tire_mu {SCENARIO["tire_mu"]})\n'
              + ''.join(f'  {m}\n' for m in metrics)
              + ''.join(f'  FAILED: {f}\n' for f in failures)
              + (f'  expected to fail: {expected_fail["reason"]}\n' if expected_fail else ''),
              flush=True)
        if expected_fail:
            known = [f for f in failures if expected_fail['failing'] in f]
            self.assertTrue(known, f'{SCENARIO["id"]} now passes '
                            f'({expected_fail["failing"]!r}): remove expected_fail from '
                            f'{NAME}.yaml and update the register (SDP §3.6)')
            failures = [f for f in failures if f not in known]
        self.assertEqual(failures, [], f'{SCENARIO["id"]} {NAME}')


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Verify every launched process shuts down cleanly."""

    def test_exit_codes(self, proc_info):
        """All processes exit with code 0 on SIGINT."""
        launch_testing.asserts.assertExitCodes(proc_info)
