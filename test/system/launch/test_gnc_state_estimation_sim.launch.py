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

The measured numbers are printed as a table, so a run also documents the
estimator's performance.

Characteristics:
  - SLOW: ~2-4 minutes (Gazebo startup + ~100 s of simulated driving)

CI/CD: Nightly
"""

import bisect
import math
import os
import threading
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

STARTUP_TIMEOUT_S = 120.0   # wall; Gazebo on a CPU-only runner is slow
DRIVE_TIMEOUT_S = 600.0     # wall, for the whole loop
SETTLE_S = 3.0              # sim; let the car drop onto the ground after spawning

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
CHI2_2DOF_3SIGMA = 11.83    # 99.73 % for 2 degrees of freedom
# Truth poses are unstamped and matched to /odom by receive time on the sim
# clock, which is good to about one truth period (~17 ms, ~2 cm at 1 m/s).
# Added to the reported covariance so the test's own error is not blamed on
# the estimator.
TRUTH_SYNC_VARIANCE = 0.02 ** 2   # m^2
MIN_CONSISTENT_FRACTION = 0.95

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


def wrap(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q):
    """Yaw of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def relative_pose(origin, pose):
    """Express pose (x, y, yaw) in the frame of origin (x, y, yaw)."""
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - origin[2]))


def truth_at(history, times, t):
    """Interpolate the truth history (sorted by time) at time t."""
    i = bisect.bisect_left(times, t)
    if i == 0:
        return history[0][1]
    if i >= len(history):
        return history[-1][1]
    (t0, p0), (t1, p1) = history[i - 1], history[i]
    f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    return (p0[0] + f * (p1[0] - p0[0]), p0[1] + f * (p1[1] - p0[1]),
            wrap(p0[2] + f * wrap(p1[2] - p0[2])))


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


class TestStateEstimationInSim(unittest.TestCase):
    """Drive the loop once and check the estimate against ground truth."""

    @classmethod
    def setUpClass(cls):
        """Create a sim-time node and spin it in the background."""
        rclpy.init()
        cls.node = rclpy.create_node(
            'test_gnc_state_estimation_sim',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.lock = threading.Lock()
        cls.truth = None           # latest (sim time, (x, y, yaw)) in map
        cls.truth_history = []     # every (sim time, (x, y, yaw)) while recording
        cls.odom = []              # (stamp, Odometry) while recording
        cls.levels = []            # (sim time, level, message)
        cls.recording = False
        cls.cmd_pub = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        cls.node.create_subscription(TFMessage, '/sim/truth/poses', cls._on_truth, 10)
        cls.node.create_subscription(Odometry, '/odom', cls._on_odom, 50)
        cls.node.create_subscription(DiagnosticArray, '/diagnostics', cls._on_diagnostics, 10)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        """Stop the car, stop spinning and shut rclpy down."""
        cls.cmd_pub.publish(Twist())
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _now(cls):
        return cls.node.get_clock().now().nanoseconds * 1e-9

    @classmethod
    def _on_truth(cls, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == 'rc_racer':
                t = tf.transform
                sample = (cls._now(), (t.translation.x, t.translation.y, yaw_of(t.rotation)))
                with cls.lock:
                    cls.truth = sample
                    if cls.recording:
                        cls.truth_history.append(sample)

    @classmethod
    def _on_odom(cls, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with cls.lock:
            if cls.recording:
                cls.odom.append((stamp, msg))

    @classmethod
    def _on_diagnostics(cls, msg):
        for status in msg.status:
            if 'state_estimation' in status.name:
                with cls.lock:
                    cls.levels.append((cls._now(), status.level, status.message))

    def _wait(self, condition, timeout, what):
        deadline = time.monotonic() + timeout
        while not condition():
            self.assertLess(time.monotonic(), deadline, f'timed out waiting for {what}')
            time.sleep(0.1)

    def _sleep_sim(self, seconds):
        end = self._now() + seconds
        self._wait(lambda: self._now() >= end, STARTUP_TIMEOUT_S, f'{seconds} s of sim time')

    def test_estimate_tracks_ground_truth(self):
        """REQ-GNC-001, -002, -006: drive the loop and compare /odom with truth."""
        # --- Start up: clock, truth, an active estimator -----------------------
        self._wait(lambda: self._now() > 0.0, STARTUP_TIMEOUT_S, '/clock')
        self._wait(lambda: self.truth is not None, STARTUP_TIMEOUT_S, '/sim/truth/poses')
        reset = self.node.create_client(Trigger, '/state_estimation/reset')
        self.assertTrue(reset.wait_for_service(timeout_sec=STARTUP_TIMEOUT_S),
                        'state_estimation ~/reset not available')
        self._sleep_sim(SETTLE_S)

        # --- Reset odom at the car's true pose and start recording -----------
        future = reset.call_async(Trigger.Request())
        self._wait(future.done, 10.0, '~/reset')
        self.assertTrue(future.result().success, future.result().message)
        with self.lock:
            origin = self.truth[1]
            type(self).recording = True
            self.levels.clear()
        t_start = self._now()

        # --- Drive the route on ground truth -----------------------------------
        driver = PurePursuit(ROUTE)
        deadline = time.monotonic() + DRIVE_TIMEOUT_S
        max_cross_track = 0.0
        while True:
            self.assertLess(time.monotonic(), deadline, 'route not finished in time')
            with self.lock:
                pose = self.truth[1]
            v, omega, cross_track, done = driver.command(pose)
            max_cross_track = max(max_cross_track, cross_track)
            self.assertLess(cross_track, MAX_CROSS_TRACK,
                            f'driver left the route at {pose} (test problem, not the estimator)')
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = omega
            self.cmd_pub.publish(cmd)
            if done:
                break
            self._sleep_sim(0.05)
        self.cmd_pub.publish(Twist())
        self._sleep_sim(1.0)  # roll to a stop; keep comparing
        with self.lock:
            type(self).recording = False
            samples = list(self.odom)
            history = list(self.truth_history)
            levels = list(self.levels)
        t_end = self._now()

        # --- Metrics --------------------------------------------------------------
        self.assertGreater(len(samples), 100, 'too few /odom samples')
        distance = 0.0
        est_distance = 0.0
        prev_truth = None
        max_step = 0.0
        prev_est = None
        errors = []
        consistent = 0
        trace = ['t,true_x,true_y,true_yaw,est_x,est_y,est_yaw,est_v,est_w,var_x,var_y,'
                 'slip_or_warn']
        warn_times = [t for t, lvl, _ in levels if lvl != DiagnosticStatus.OK]
        truth_times = [t for t, _ in history]
        samples = [(s, m) for s, m in samples if truth_times[0] <= s <= truth_times[-1]]
        for stamp, msg in samples:
            true_rel = relative_pose(origin, truth_at(history, truth_times, stamp))
            est = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                   yaw_of(msg.pose.pose.orientation))
            if prev_truth is not None:
                distance += math.hypot(true_rel[0] - prev_truth[0], true_rel[1] - prev_truth[1])
            if prev_est is not None:
                step = math.hypot(est[0] - prev_est[0], est[1] - prev_est[1])
                max_step = max(max_step, step)
                est_distance += step
            prev_truth, prev_est = true_rel, est

            trace.append(','.join(f'{v:.5f}' for v in (
                stamp, *true_rel, *est, msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                msg.pose.covariance[0], msg.pose.covariance[7],
                float(any(abs(t - stamp) < 0.5 for t in warn_times)))))
            ex, ey = est[0] - true_rel[0], est[1] - true_rel[1]
            errors.append((math.hypot(ex, ey), abs(wrap(est[2] - true_rel[2]))))
            c = msg.pose.covariance
            sxx, sxy, syy = c[0] + TRUTH_SYNC_VARIANCE, c[1], c[7] + TRUTH_SYNC_VARIANCE
            det = sxx * syy - sxy * sxy
            if det > 0.0:
                mahalanobis2 = (syy * ex * ex - 2.0 * sxy * ex * ey + sxx * ey * ey) / det
                consistent += mahalanobis2 <= CHI2_2DOF_3SIGMA

        try:
            os.makedirs(os.path.dirname(TRACE_FILE), exist_ok=True)
            with open(TRACE_FILE, 'w') as f:
                f.write('\n'.join(trace) + '\n')
        except OSError as e:
            print(f'could not write trace: {e}')
        stamps = [s for s, _ in samples]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        final_error, final_yaw_error = errors[-1]
        # Final error split along / across the true heading: along-track error
        # comes from speed (scale), cross-track from heading.
        last_truth = relative_pose(origin, truth_at(history, truth_times, samples[-1][0]))
        last_msg = samples[-1][1]
        ex = last_msg.pose.pose.position.x - last_truth[0]
        ey = last_msg.pose.pose.position.y - last_truth[1]
        along = ex * math.cos(last_truth[2]) + ey * math.sin(last_truth[2])
        across = -ex * math.sin(last_truth[2]) + ey * math.cos(last_truth[2])
        cov = last_msg.pose.covariance
        sx, sy = math.sqrt(cov[0]), math.sqrt(cov[7])
        max_error = max(e for e, _ in errors)
        drift = final_error / distance
        consistent_fraction = consistent / len(samples)
        final_sigma = math.sqrt(samples[-1][1].pose.covariance[0]
                                + samples[-1][1].pose.covariance[7])
        errors_seen = [m for _, lvl, m in levels if lvl == DiagnosticStatus.ERROR]
        last_level = levels[-1] if levels else None

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
              f'  non-OK diagnostics      {len(warn_times):8d} of {len(levels)}\n',
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
        self.assertIsNotNone(last_level, 'no diagnostics from state_estimation')
        self.assertEqual(last_level[1], DiagnosticStatus.OK, last_level[2])


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Verify the simulation and state_estimation shut down cleanly."""

    def test_exit_codes(self, proc_info):
        """All processes exit with code 0 on SIGINT."""
        launch_testing.asserts.assertExitCodes(proc_info)
