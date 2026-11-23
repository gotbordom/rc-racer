# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Shared support for the state_estimation system tests in simulation.

Used by the L2 loop (test_gnc_state_estimation_sim) and the validation
scenarios (test_gnc_state_estimation_scenario):
  - SimRecorder: a sim-time node that records Gazebo ground truth
    (/sim/truth/poses, ICD-VEH-TST VEH-TST-07), /odom and the
    state_estimation diagnostics, and resets odom at the car's true pose
  - compare(): matches each /odom sample with truth and measures the error
    and whether it is inside the reported 3-sigma ellipse

Every time here is sim time (s).
"""

import bisect
import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

STARTUP_TIMEOUT_S = 120.0   # wall; Gazebo on a CPU-only runner is slow
SETTLE_S = 3.0              # sim; let the car drop onto the ground after spawning

CHI2_2DOF_3SIGMA = 11.83    # 99.73 % for 2 degrees of freedom
# Truth poses are unstamped and matched to /odom by receive time on the sim
# clock, which is good to about one truth period (~17 ms, ~2 cm at 1 m/s).
# Added to the reported covariance so the test's own error is not blamed on
# the estimator.
TRUTH_SYNC_VARIANCE = 0.02 ** 2   # m^2


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


class Sample:
    """One /odom sample against truth, both in the odom frame."""

    __slots__ = ('t', 'truth', 'est', 'v', 'w', 'cov', 'error', 'yaw_error', 'consistent')

    def __init__(self, t, truth, msg):
        """Compare msg with the true pose (odom frame) at stamp t."""
        self.t = t
        self.truth = truth
        p = msg.pose.pose
        self.est = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z
        self.cov = msg.pose.covariance
        ex, ey = self.est[0] - truth[0], self.est[1] - truth[1]
        self.error = math.hypot(ex, ey)
        self.yaw_error = abs(wrap(self.est[2] - truth[2]))
        c = self.cov
        sxx, sxy, syy = c[0] + TRUTH_SYNC_VARIANCE, c[1], c[7] + TRUTH_SYNC_VARIANCE
        det = sxx * syy - sxy * sxy
        self.consistent = det > 0.0 and (
            (syy * ex * ex - 2.0 * sxy * ex * ey + sxx * ey * ey) / det <= CHI2_2DOF_3SIGMA)

    @property
    def var_x(self):
        """Return the reported x variance (m^2)."""
        return self.cov[0]

    @property
    def var_y(self):
        """Return the reported y variance (m^2)."""
        return self.cov[7]


def compare(odom, history, origin):
    """
    Match each recorded /odom message with truth.

    odom: [(stamp, Odometry)], history: [(t, (x, y, yaw) in map)], origin:
    the true pose at the odom reset. Samples outside the truth history are
    dropped. Returns [Sample].
    """
    times = [t for t, _ in history]
    return [Sample(stamp, relative_pose(origin, truth_at(history, times, stamp)), msg)
            for stamp, msg in odom if times and times[0] <= stamp <= times[-1]]


def distance_travelled(samples):
    """Return the true path length over the samples (m)."""
    return sum(math.hypot(b.truth[0] - a.truth[0], b.truth[1] - a.truth[1])
               for a, b in zip(samples, samples[1:]))


class Health:
    """One state_estimation diagnostics report."""

    __slots__ = ('t', 'level', 'message', 'values')

    def __init__(self, t, status):
        """Keep the level, message and key/values of a DiagnosticStatus."""
        self.t = t
        self.level = status.level
        self.message = status.message
        self.values = {v.key: v.value for v in status.values}

    @property
    def slip_events(self):
        """Return the slip events counted since activation or reset."""
        return int(self.values.get('slip_events', 0))

    @property
    def is_ok(self):
        """Return whether this report is OK."""
        return self.level == DiagnosticStatus.OK

    @property
    def is_error(self):
        """Return whether this report is ERROR."""
        return self.level == DiagnosticStatus.ERROR


class SimRecorder:
    """
    Sim-time node that drives the car and records truth, /odom and health.

    Spins in a background thread. Call start_recording() after
    reset_odom(); stop_recording() returns what was recorded.
    """

    def __init__(self, name, cmd_topic='/cmd_vel'):
        """Create the node, its publishers and subscriptions, and spin it."""
        self.node = rclpy.create_node(
            name, parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        self.truth = None           # latest (t, (x, y, yaw)) in map
        self._history = []          # every (t, (x, y, yaw)) while recording
        self._odom = []             # (stamp, Odometry) while recording
        self._health = []           # Health while recording
        self._recording = False
        self.cmd_pub = self.node.create_publisher(Twist, cmd_topic, 10)
        self.node.create_subscription(TFMessage, '/sim/truth/poses', self._on_truth, 10)
        self.node.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.node.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, 10)
        self._reset = self.node.create_client(Trigger, '/state_estimation/reset')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def shutdown(self):
        """Stop the car and stop spinning."""
        self.cmd_pub.publish(Twist())
        self._executor.shutdown()
        self.node.destroy_node()

    def now(self):
        """Return the sim time (s)."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    # -- callbacks -------------------------------------------------------------

    def _on_truth(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == 'rc_racer':
                t = tf.transform
                sample = (self.now(), (t.translation.x, t.translation.y, yaw_of(t.rotation)))
                with self.lock:
                    self.truth = sample
                    if self._recording:
                        self._history.append(sample)

    def _on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            if self._recording:
                self._odom.append((stamp, msg))

    def _on_diagnostics(self, msg):
        for status in msg.status:
            if 'state_estimation' in status.name:
                with self.lock:
                    if self._recording:
                        self._health.append(Health(self.now(), status))

    # -- waiting and driving ------------------------------------------------------

    @staticmethod
    def wait(condition, timeout, what):
        """Wait (wall time) for condition(); raise AssertionError on timeout."""
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                raise AssertionError(f'timed out waiting for {what}')
            time.sleep(0.02)

    def sleep_sim(self, seconds):
        """Sleep for seconds of sim time."""
        end = self.now() + seconds
        self.wait(lambda: self.now() >= end, STARTUP_TIMEOUT_S, f'{seconds} s of sim time')

    def true_pose(self):
        """Return the latest true pose (x, y, yaw) in map."""
        with self.lock:
            return self.truth[1]

    def command(self, speed, angular):
        """Publish one command on the command topic."""
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def wait_for_sim(self):
        """Wait for /clock, truth and the estimator, then let the car settle."""
        self.wait(lambda: self.now() > 0.0, STARTUP_TIMEOUT_S, '/clock')
        self.wait(lambda: self.truth is not None, STARTUP_TIMEOUT_S, '/sim/truth/poses')
        if not self._reset.wait_for_service(timeout_sec=STARTUP_TIMEOUT_S):
            raise AssertionError('state_estimation ~/reset not available')
        self.sleep_sim(SETTLE_S)

    def reset_odom(self):
        """Reset odom; return the true pose (map) the odom origin now sits at."""
        future = self._reset.call_async(Trigger.Request())
        self.wait(future.done, 10.0, '~/reset')
        if not future.result().success:
            raise AssertionError(future.result().message)
        return self.true_pose()

    def start_recording(self):
        """Clear and start recording truth, /odom and health."""
        with self.lock:
            self._history, self._odom, self._health = [], [], []
            self._recording = True

    def stop_recording(self):
        """Stop recording; return (truth history, odom, health)."""
        with self.lock:
            self._recording = False
            return list(self._history), list(self._odom), list(self._health)
