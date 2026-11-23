# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Component test: state_estimation node (VEH.GNC.SE spec §7, L3).

Launches the node with its launch file (lifecycle autostart) and drives it
with stub /imu and /wheel/odom publishers that follow ICD-SNS-GNC. A static
transform stands in for the URDF's IMU mount.

Validates:
  - The node comes up active and reports STALE until its inputs arrive
  - /odom at 50 Hz in odom -> base_footprint with non-zero covariance, and
    the estimate follows the inputs (REQ-GNC-001, REQ-GNC-006)
  - TF odom -> base_footprint is available and this node is the only /tf
    publisher (Architecture Conventions §2.3)
  - ~/reset puts the estimate back at the origin
  - Stale wheel odometry is WARN, stale IMU is ERROR, and /odom keeps
    publishing with growing covariance (ICD-SNS-GNC §5, spec §4)

Characteristics:
  - ~20 s, no simulator

CI/CD: Every push (colcon test)
"""

import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

SPEED = 1.0      # m/s, stub wheel odometry
YAW_RATE = 0.2   # rad/s, stub gyro
PUBLISH_RATE = 50.0
TIMEOUT_S = 15.0


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch state_estimation and a stand-in IMU mount transform."""
    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('state_estimation') + '/launch/state_estimation.launch.py'
        )
    )
    imu_mount = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--z', '0.194', '--frame-id', 'base_footprint', '--child-frame-id', 'imu_link'],
    )
    return launch.LaunchDescription([
        state_estimation,
        imu_mount,
        launch_testing.actions.ReadyToTest(),
    ])


def make_imu(stamp):
    """Level, stationary-attitude IMU sample turning at YAW_RATE."""
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = 'imu_link'
    msg.orientation.w = 1.0
    msg.orientation_covariance = [4e-4, 0.0, 0.0, 0.0, 4e-4, 0.0, 0.0, 0.0, 2.5e-3]
    msg.angular_velocity.z = YAW_RATE
    msg.angular_velocity_covariance = [2.5e-5, 0.0, 0.0, 0.0, 2.5e-5, 0.0, 0.0, 0.0, 2.5e-5]
    msg.linear_acceleration.z = 9.81
    msg.linear_acceleration_covariance = [
        2.5e-3, 0.0, 0.0, 0.0, 2.5e-3, 0.0, 0.0, 0.0, 2.5e-3]
    return msg


def make_wheel_odom(stamp):
    """Wheel odometry at SPEED; only linear.x is meaningful (ICD-SNS-GNC §4)."""
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_footprint'
    msg.twist.twist.linear.x = SPEED
    cov = [0.0] * 36
    for i in range(6):
        cov[i * 6 + i] = 1e6
    cov[0] = 4e-4
    msg.twist.covariance = cov
    msg.pose.covariance = [1e6 if i % 7 == 0 else 0.0 for i in range(36)]
    return msg


class TestStateEstimationNode(unittest.TestCase):
    """Drive the node with stub sensors. Tests run in name order."""

    @classmethod
    def setUpClass(cls):
        """Start a stub-sensor node and spin it in a background thread."""
        rclpy.init()
        cls.node = rclpy.create_node('state_estimation_test')
        cls.lock = threading.Lock()
        cls.odom = []       # (receive time, Odometry)
        cls.diagnostics = []  # DiagnosticStatus for state_estimation
        cls.send_imu = False
        cls.send_wheel = False

        cls.imu_pub = cls.node.create_publisher(Imu, '/imu', 10)
        cls.wheel_pub = cls.node.create_publisher(Odometry, '/wheel/odom', 10)
        cls.node.create_subscription(Odometry, '/odom', cls._on_odom, 10)
        cls.node.create_subscription(DiagnosticArray, '/diagnostics', cls._on_diagnostics, 10)
        cls.node.create_timer(0.01, cls._publish_imu)
        cls.node.create_timer(0.02, cls._publish_wheel)
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)

        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        """Stop spinning and shut rclpy down."""
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    # -- stub sensors and recorders -----------------------------------------

    @classmethod
    def _publish_imu(cls):
        if cls.send_imu:
            cls.imu_pub.publish(make_imu(cls.node.get_clock().now().to_msg()))

    @classmethod
    def _publish_wheel(cls):
        if cls.send_wheel:
            cls.wheel_pub.publish(make_wheel_odom(cls.node.get_clock().now().to_msg()))

    @classmethod
    def _on_odom(cls, msg):
        with cls.lock:
            cls.odom.append((time.monotonic(), msg))

    @classmethod
    def _on_diagnostics(cls, msg):
        with cls.lock:
            cls.diagnostics.extend(s for s in msg.status if 'state_estimation' in s.name)

    # -- helpers -------------------------------------------------------------

    def wait_for_diagnostic(self, level, text=None, timeout=TIMEOUT_S):
        """Wait for a state_estimation status at this level (and message text)."""
        with self.lock:
            self.diagnostics.clear()
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            with self.lock:
                statuses = list(self.diagnostics)
            for status in statuses:
                last = status
                if status.level == level and (text is None or text in status.message):
                    return status
            time.sleep(0.1)
        self.fail(f'no diagnostic at level {level!r} ({text}); last: '
                  f'{last.level if last else None!r} {last.message if last else ""}')

    def odom_since(self, start):
        """Return /odom messages received since monotonic time start."""
        with self.lock:
            return [m for t, m in self.odom if t >= start]

    def call(self, client, request):
        """Call a service and wait for the response."""
        self.assertTrue(client.wait_for_service(timeout_sec=TIMEOUT_S), client.srv_name)
        future = client.call_async(request)
        deadline = time.monotonic() + TIMEOUT_S
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertTrue(future.done(), f'{client.srv_name} timed out')
        return future.result()

    # -- tests -----------------------------------------------------------------

    def test_1_active_and_stale_until_inputs_arrive(self):
        """Lifecycle autostart reaches active; health is STALE with no inputs."""
        client = self.node.create_client(GetState, '/state_estimation/get_state')
        deadline = time.monotonic() + TIMEOUT_S
        state = None
        while time.monotonic() < deadline:
            state = self.call(client, GetState.Request()).current_state
            if state.id == State.PRIMARY_STATE_ACTIVE:
                break
            time.sleep(0.2)
        self.assertEqual(state.id, State.PRIMARY_STATE_ACTIVE, state.label)
        self.wait_for_diagnostic(DiagnosticStatus.STALE, 'waiting for /imu')

    def test_2_odom_rate_frames_and_covariance(self):
        """REQ-GNC-001, REQ-GNC-006: /odom at 50 Hz, right frames, covariance."""
        type(self).send_imu = True
        type(self).send_wheel = True
        time.sleep(2.0)  # let the estimate settle on the inputs
        start = time.monotonic()
        time.sleep(2.0)
        msgs = self.odom_since(start)

        rate = len(msgs) / 2.0
        self.assertGreater(rate, 0.8 * PUBLISH_RATE)
        self.assertLess(rate, 1.2 * PUBLISH_RATE)
        last = msgs[-1]
        self.assertEqual(last.header.frame_id, 'odom')
        self.assertEqual(last.child_frame_id, 'base_footprint')
        self.assertGreater(last.pose.covariance[0], 0.0)
        self.assertGreater(last.pose.covariance[35], 0.0)
        self.assertGreater(last.twist.covariance[0], 0.0)
        self.assertGreater(last.twist.covariance[35], 0.0)
        self.assertAlmostEqual(last.twist.twist.linear.x, SPEED, delta=0.05)
        self.assertAlmostEqual(last.twist.twist.angular.z, YAW_RATE, delta=0.02)
        self.assertGreater(last.pose.pose.position.x, 1.0)  # it has been driving
        self.wait_for_diagnostic(DiagnosticStatus.OK)

    def test_3_tf_published_by_this_node_only(self):
        """Conventions §2.3: odom -> base_footprint exists, one /tf publisher."""
        deadline = time.monotonic() + TIMEOUT_S
        while not self.tf_buffer.can_transform('odom', 'base_footprint', Time()):
            self.assertLess(time.monotonic(), deadline, 'no TF odom -> base_footprint')
            time.sleep(0.1)
        self.assertEqual(self.node.count_publishers('/tf'), 1)

    def test_4_reset_returns_to_origin(self):
        """~/reset restarts odom at the car's current position."""
        client = self.node.create_client(Trigger, '/state_estimation/reset')
        response = self.call(client, Trigger.Request())
        self.assertTrue(response.success, response.message)
        start = time.monotonic()
        time.sleep(0.1)
        first = self.odom_since(start)[0]
        self.assertLess(abs(first.pose.pose.position.x), 0.2)
        self.assertLess(abs(first.pose.pose.position.y), 0.2)

    def test_5_stale_wheel_odometry_is_warn(self):
        """ICD-SNS-GNC §5: no /wheel/odom for 0.2 s is WARN."""
        type(self).send_wheel = False
        self.wait_for_diagnostic(DiagnosticStatus.WARN, '/wheel/odom stale')

    def test_6_stale_imu_is_error_and_odom_continues(self):
        """ICD-SNS-GNC §5 and spec §4: stale IMU is ERROR; /odom keeps going."""
        type(self).send_imu = False
        self.wait_for_diagnostic(DiagnosticStatus.ERROR, '/imu stale')
        start = time.monotonic()
        time.sleep(1.0)
        msgs = self.odom_since(start)
        self.assertGreater(len(msgs), 0.8 * PUBLISH_RATE)
        self.assertGreater(msgs[-1].pose.covariance[0], msgs[0].pose.covariance[0])
        self.assertGreater(msgs[-1].twist.covariance[0], msgs[0].twist.covariance[0])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Checks after the launched processes stop."""

    def test_exit_codes(self, proc_info):
        """Every launched process exits cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
