# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Subsystem Test (L2): state_estimation in simulation.

Drives a ~100 m loop around the practice course and compares the estimate
on /odom with Gazebo's ground-truth pose (/sim/truth/poses, ICD-VEH-TST
VEH-TST-07). The car is steered by pure pursuit on the TRUE pose, so the
driving does not depend on the estimate under test.

Validates (VEH.GNC.SE spec §7):
  - REQ-GNC-001: /odom at >= 50 Hz (sim time) while driving
  - REQ-GNC-002: the estimate is continuous (no jumps) and its drift stays
    within DRIFT_LIMIT of distance travelled (TBD-003, set from this test)
  - REQ-GNC-006: the reported position covariance is honest: the true
    error is inside its 3-sigma ellipse for nearly all samples
  - Health stays out of ERROR, and the run ends in OK
  - REQ-GNC-006 (SDP §3.6 VAL-008): the position variance grows with
    distance driven, by at least most of the distance process noise
    (not monotonically: the scale term shrinks as the loop returns
    toward the start, VAL-016)
  - REQ-GNC-002 (VAL-009): /odom speed follows the true wheel speed
    (/sim/truth/wheel_odom) with no bias while cruising and no lag
  - VAL-013: the full TF tree odom -> base_footprint -> base_link ->
    sensors resolves, and only state_estimation and robot_state_publisher
    publish /tf (Architecture Conventions §2.3)

The measured numbers are printed as a table, so a run also documents the
estimator's performance.

Characteristics:
  - SLOW: ~2-4 minutes (Gazebo startup + ~100 s of simulated driving)

CI/CD: Nightly
"""

import bisect
import math
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
import yaml
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from se_sim import compare, SimRecorder, wrap
from tf2_ros import Buffer, TransformListener

DRIVE_TIMEOUT_S = 600.0     # wall, for the whole loop

# Route in the practice course (map frame, metres; see practice_course.yaml).
# Passes 1.5-2 m from M1, M2 and the finish cone, and >= 2 m from rock_1
# (8, 2.5), tree_1 (25, 8) and bench_1 (40, 8), then returns toward the start.
ROUTE = [
    (0.0, 0.0), (10.0, 0.0), (15.0, 3.5), (25.0, 5.0), (34.0, 13.5),
    (36.0, 6.0), (43.5, -1.5), (30.0, -6.0), (10.0, -3.0), (2.0, -1.0),
]
SPEED = 1.0                 # m/s
LOOKAHEAD = 2.5             # m, pure pursuit
WHEELBASE = 0.384           # m
MAX_STEERING = 0.52         # rad
GOAL_TOLERANCE = 1.0        # m, end of route
MAX_CROSS_TRACK = 3.0       # m, beyond this the driver has lost the route

# Pass criteria
MIN_RATE_HZ = 0.9 * 50.0
MAX_STEP = 0.1              # m between consecutive /odom samples (no jumps)
# TBD-003 proposal (TBR): drift <= 2 % of distance travelled. Measured drift
# on the first runs was well under this; see the printed table.
DRIFT_LIMIT = 0.02
MAX_FINAL_YAW_ERROR = 0.1   # rad
MIN_CONSISTENT_FRACTION = 0.95
# VAL-008: at each checkpoint the position variance (x + y) must be at least
# this share of process_noise.distance x distance driven. Speed updates take
# a little off through the speed-position correlation.
VARIANCE_CHECKPOINT_M = 10.0
MIN_DISTANCE_NOISE_SHARE = 0.8
# VAL-009: speed against the true wheel speed. Bias over cruise samples (true
# wheel speed within CRUISE_BAND of SPEED, no slip); lag as the time shift
# (searched within MAX_SHIFT_S) that best lines the two up.
CRUISE_BAND = 0.05          # m/s
MAX_SPEED_BIAS = 0.02       # m/s
MAX_SPEED_LAG_S = 0.05
MAX_SHIFT_S = 0.3
# VAL-013: frames that must resolve from odom, and the only /tf publishers
TF_PARENTS = {
    'base_footprint': 'odom', 'base_link': 'base_footprint', 'imu_link': 'base_link',
    'gps_link': 'base_link', 'camera_link': 'base_link', 'camera_optical_link': 'camera_link',
}
SENSOR_FRAMES = list(TF_PARENTS)
TF_PUBLISHERS = {'state_estimation', 'robot_state_publisher'}
TF_TIMEOUT_S = 10.0         # wall

# Per-sample trace (truth vs estimate), saved with the test results so a
# failure can be analysed. ctest runs in the package's build directory.
TRACE_FILE = os.path.join('test_results', 'rc_racer_system_tests',
                          'gnc_state_estimation_trace.csv')


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the practice course headless, plus state_estimation on sim time."""
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rc_racer_simulation'), 'launch', 'simulation.launch.py')),
        launch_arguments={'headless': 'true', 'world': 'practice_course.world'}.items(),
    )
    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('state_estimation'), 'launch',
            'state_estimation.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    return launch.LaunchDescription([
        simulation,
        state_estimation,
        launch_testing.actions.ReadyToTest(),
    ])


class PurePursuit:
    """Follow a polyline with pure pursuit; returns (v, omega) commands."""

    def __init__(self, route):
        """Precompute segment lengths along the route."""
        self.route = route
        self.cumulative = [0.0]
        for (x0, y0), (x1, y1) in zip(route, route[1:]):
            self.cumulative.append(self.cumulative[-1] + math.hypot(x1 - x0, y1 - y0))
        self.progress = 0.0  # arc length of the closest point so far

    def _point_at(self, s):
        s = min(max(s, 0.0), self.cumulative[-1])
        for i in range(len(self.route) - 1):
            if s <= self.cumulative[i + 1]:
                (x0, y0), (x1, y1) = self.route[i], self.route[i + 1]
                f = (s - self.cumulative[i]) / (self.cumulative[i + 1] - self.cumulative[i])
                return x0 + f * (x1 - x0), y0 + f * (y1 - y0)
        return self.route[-1]

    def _closest(self, x, y):
        """Arc length and distance of the closest route point (never backwards)."""
        best = (float('inf'), self.progress)
        for i in range(len(self.route) - 1):
            (x0, y0), (x1, y1) = self.route[i], self.route[i + 1]
            length = self.cumulative[i + 1] - self.cumulative[i]
            f = ((x - x0) * (x1 - x0) + (y - y0) * (y1 - y0)) / (length * length)
            f = min(max(f, 0.0), 1.0)
            px, py = x0 + f * (x1 - x0), y0 + f * (y1 - y0)
            s = self.cumulative[i] + f * length
            d = math.hypot(x - px, y - py)
            if s >= self.progress - 1.0 and d < best[0]:
                best = (d, s)
        return best

    def command(self, pose):
        """Return (v, omega, cross_track, done) for the true pose (x, y, yaw)."""
        x, y, yaw = pose
        cross_track, s = self._closest(x, y)
        self.progress = max(self.progress, s)
        gx, gy = self.route[-1]
        if self.cumulative[-1] - self.progress < GOAL_TOLERANCE and \
                math.hypot(gx - x, gy - y) < GOAL_TOLERANCE:
            return 0.0, 0.0, cross_track, True
        tx, ty = self._point_at(self.progress + LOOKAHEAD)
        alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
        curvature = 2.0 * math.sin(alpha) / LOOKAHEAD
        max_curvature = math.tan(MAX_STEERING) / WHEELBASE
        curvature = min(max(curvature, -max_curvature), max_curvature)
        return SPEED, SPEED * curvature, cross_track, False


def distance_noise_density():
    """Return process_noise.distance (m^2/m) from state_estimation's config."""
    path = os.path.join(get_package_share_directory('state_estimation'), 'config',
                        'state_estimation.yaml')
    with open(path) as f:
        params = yaml.safe_load(f)['state_estimation']['ros__parameters']
    return params['process_noise']['distance']


def variance_checkpoints(samples, every):
    """Return [(distance driven, x + y variance)] each `every` metres."""
    out, distance, next_mark = [], 0.0, every
    for a, b in zip(samples, samples[1:]):
        distance += math.hypot(b.truth[0] - a.truth[0], b.truth[1] - a.truth[1])
        if distance >= next_mark:
            out.append((distance, b.var_x + b.var_y))
            next_mark += every
    return out


def speed_tracking(samples, wheel, slip_times):
    """Return (bias, lag) of the /odom speed against the true wheel speed."""
    times = [t for t, _ in wheel]

    def wheel_at(t):
        i = min(max(bisect.bisect_left(times, t), 1), len(wheel) - 1)
        (t0, v0), (t1, v1) = wheel[i - 1], wheel[i]
        return v0 if t1 == t0 else v0 + (t - t0) / (t1 - t0) * (v1 - v0)

    inside = [s for s in samples
              if times[0] + MAX_SHIFT_S <= s.t <= times[-1] - MAX_SHIFT_S
              and not any(abs(s.t - t) < 1.0 for t in slip_times)]
    cruise = [s.v - wheel_at(s.t) for s in inside if abs(wheel_at(s.t) - SPEED) < CRUISE_BAND]
    bias = sum(cruise) / len(cruise) if cruise else float('nan')

    def rms(shift):
        return math.sqrt(sum((s.v - wheel_at(s.t - shift)) ** 2 for s in inside) / len(inside))
    shifts = [i * 0.01 for i in range(-int(MAX_SHIFT_S * 100), int(MAX_SHIFT_S * 100) + 1)]
    lag = min(shifts, key=rms)
    return bias, lag, len(cruise)


class TestStateEstimationInSim(unittest.TestCase):
    """Drive the loop once and check the estimate against ground truth."""

    @classmethod
    def setUpClass(cls):
        """Create the sim-time recorder node."""
        rclpy.init()
        cls.sim = SimRecorder('test_gnc_state_estimation_sim')

    @classmethod
    def tearDownClass(cls):
        """Stop the car and shut rclpy down."""
        cls.sim.shutdown()
        rclpy.shutdown()

    def test_estimate_tracks_ground_truth(self):
        """REQ-GNC-001, -002, -006: drive the loop and compare /odom with truth."""
        sim = self.sim
        # --- Start up, reset odom at the car's true pose, record -------------
        sim.wait_for_sim()
        origin = sim.reset_odom()
        sim.start_recording()
        t_start = sim.now()

        # --- Drive the route on ground truth -----------------------------------
        driver = PurePursuit(ROUTE)
        deadline = time.monotonic() + DRIVE_TIMEOUT_S
        max_cross_track = 0.0
        while True:
            self.assertLess(time.monotonic(), deadline, 'route not finished in time')
            pose = sim.true_pose()
            v, omega, cross_track, done = driver.command(pose)
            max_cross_track = max(max_cross_track, cross_track)
            self.assertLess(cross_track, MAX_CROSS_TRACK,
                            f'driver left the route at {pose} (test problem, not the estimator)')
            sim.command(v, omega)
            if done:
                break
            sim.sleep_sim(0.05)
        sim.command(0.0, 0.0)
        sim.sleep_sim(1.0)  # roll to a stop; keep comparing
        recording = sim.stop_recording()
        history, odom, health = recording.history, recording.odom, recording.health
        t_end = sim.now()

        # --- Metrics --------------------------------------------------------------
        samples = compare(odom, history, origin)
        self.assertGreater(len(samples), 100, 'too few /odom samples')
        distance = 0.0
        est_distance = 0.0
        max_step = 0.0
        warn_times = [h.t for h in health if not h.is_ok]
        trace = ['t,true_x,true_y,true_yaw,est_x,est_y,est_yaw,est_v,est_w,var_x,var_y,'
                 'slip_or_warn']
        for prev, cur in zip([None] + samples, samples):
            if prev is not None:
                distance += math.hypot(cur.truth[0] - prev.truth[0], cur.truth[1] - prev.truth[1])
                step = math.hypot(cur.est[0] - prev.est[0], cur.est[1] - prev.est[1])
                max_step = max(max_step, step)
                est_distance += step
            trace.append(','.join(f'{v:.5f}' for v in (
                cur.t, *cur.truth, *cur.est, cur.v, cur.w, cur.var_x, cur.var_y,
                float(any(abs(t - cur.t) < 0.5 for t in warn_times)))))

        try:
            os.makedirs(os.path.dirname(TRACE_FILE), exist_ok=True)
            with open(TRACE_FILE, 'w') as f:
                f.write('\n'.join(trace) + '\n')
        except OSError as e:
            print(f'could not write trace: {e}')
        rate = (len(samples) - 1) / (samples[-1].t - samples[0].t)
        last = samples[-1]
        final_error, final_yaw_error = last.error, last.yaw_error
        # Final error split along / across the true heading: along-track error
        # comes from speed (scale), cross-track from heading.
        ex, ey = last.est[0] - last.truth[0], last.est[1] - last.truth[1]
        along = ex * math.cos(last.truth[2]) + ey * math.sin(last.truth[2])
        across = -ex * math.sin(last.truth[2]) + ey * math.cos(last.truth[2])
        sx, sy = math.sqrt(last.var_x), math.sqrt(last.var_y)
        max_error = max(s.error for s in samples)
        drift = final_error / distance
        consistent_fraction = sum(s.consistent for s in samples) / len(samples)
        final_sigma = math.sqrt(last.var_x + last.var_y)
        errors_seen = [h.message for h in health if h.is_error]
        last_health = health[-1] if health else None
        q = distance_noise_density()
        checkpoints = variance_checkpoints(samples, VARIANCE_CHECKPOINT_M)
        noise_share = min(var / (q * d) for d, var in checkpoints)
        slip_times = [h.t for h in health if h.values.get('slip_detected') == 'True']
        bias, lag, n_cruise = speed_tracking(samples, recording.wheel, slip_times)

        print('\n'
              '  state_estimation vs ground truth (practice course loop)\n'
              f'  sim time driven         {t_end - t_start:8.1f} s\n'
              f'  distance travelled      {distance:8.1f} m\n'
              f'  driver max cross-track  {max_cross_track:8.2f} m\n'
              f'  /odom rate (sim)        {rate:8.1f} Hz\n'
              f'  max step between msgs   {max_step:8.3f} m\n'
              f'  final position error    {final_error:8.3f} m\n'
              f'    along / across track  {along:+8.3f} / {across:+.3f} m\n'
              f'    final 1-sigma x / y   {sx:8.3f} / {sy:.3f} m\n'
              f'  estimated distance      {est_distance:8.2f} m ('
              f'{100.0 * (est_distance / distance - 1.0):+.2f} %)\n'
              f'  max position error      {max_error:8.3f} m\n'
              f'  drift                   {100.0 * drift:8.2f} % of distance\n'
              f'  final yaw error         {math.degrees(final_yaw_error):8.2f} deg\n'
              f'  final 1-sigma (x+y)     {final_sigma:8.3f} m\n'
              f'  inside 3-sigma ellipse  {100.0 * consistent_fraction:8.1f} %\n'
              f'  diagnostics ERRORs      {len(errors_seen):8d}\n'
              f'  non-OK diagnostics      {len(warn_times):8d} of {len(health)}\n'
              f'  position variance at    '
              + ', '.join(f'{d:.0f} m: {v:.4f}' for d, v in checkpoints) + ' m^2\n'
              f'  lowest share of q x d   {noise_share:8.2f} (q = {q:g} m^2/m)\n'
              f'  speed bias, cruising    {bias:+8.4f} m/s ({n_cruise} samples)\n'
              f'  speed lag vs wheel      {lag:+8.2f} s\n',
              flush=True)

        # --- Pass criteria -------------------------------------------------------
        self.assertGreater(distance, 80.0, 'route shorter than expected')
        self.assertGreaterEqual(rate, MIN_RATE_HZ, 'REQ-GNC-001: /odom rate')
        self.assertLessEqual(max_step, MAX_STEP, 'REQ-GNC-002: estimate jumped')
        self.assertLessEqual(drift, DRIFT_LIMIT, 'REQ-GNC-002 / TBD-003: drift')
        self.assertLessEqual(final_yaw_error, MAX_FINAL_YAW_ERROR, 'final heading error')
        self.assertGreaterEqual(consistent_fraction, MIN_CONSISTENT_FRACTION,
                                'REQ-GNC-006: covariance smaller than the real error')
        self.assertEqual(errors_seen, [], 'state_estimation reported ERROR while driving')
        self.assertIsNotNone(last_health, 'no diagnostics from state_estimation')
        self.assertTrue(last_health.is_ok, last_health.message)
        self.assertGreater(checkpoints[-1][1], checkpoints[0][1],
                           'VAL-008: position variance did not grow over the loop')
        self.assertGreaterEqual(noise_share, MIN_DISTANCE_NOISE_SHARE,
                                'VAL-008: position variance below the distance process noise')
        self.assertGreater(n_cruise, 100, 'VAL-009: too few cruise samples')
        self.assertLessEqual(abs(bias), MAX_SPEED_BIAS, 'VAL-009: speed biased against the wheel')
        self.assertLessEqual(abs(lag), MAX_SPEED_LAG_S, 'VAL-009: speed lags the wheel')

    def test_tf_tree_and_publishers(self):
        """VAL-013: odom -> base_footprint -> base_link -> sensors; two /tf publishers."""
        buffer = Buffer()
        TransformListener(buffer, self.sim.node)
        missing = list(SENSOR_FRAMES)
        deadline = time.monotonic() + TF_TIMEOUT_S
        while missing and time.monotonic() < deadline:
            missing = [f for f in missing if not buffer.can_transform('odom', f, Time())]
            time.sleep(0.1)
        self.assertEqual(missing, [], 'VAL-013: frames not reachable from odom')
        parents = {frame: info['parent']
                   for frame, info in yaml.safe_load(buffer.all_frames_as_yaml()).items()}
        for frame, parent in TF_PARENTS.items():
            self.assertEqual(parents.get(frame), parent, f'VAL-013: parent of {frame}')
        publishers = {p.node_name for p in self.sim.node.get_publishers_info_by_topic('/tf')}
        self.assertEqual(publishers, TF_PUBLISHERS, 'VAL-013: unexpected /tf publishers')


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Verify the simulation and state_estimation shut down cleanly."""

    def test_exit_codes(self, proc_info):
        """All processes exit with code 0 on SIGINT."""
        launch_testing.asserts.assertExitCodes(proc_info)
