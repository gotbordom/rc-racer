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
  - deactivate stops /odom with STALE "inactive", and activate restarts
    odom at the car (SDP §3.6 VAL-011)
  - motion_model 'tank' fails configure naming the parameter, and
    'unicycle' configures and runs (REQ-GNC-007, SDP §3.6 VAL-012)

Characteristics:
  - ~30 s, no simulator

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
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
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

    def get_state(self):
        """Return the node's lifecycle state."""
        client = self.node.create_client(GetState, '/state_estimation/get_state')
        return self.call(client, GetState.Request()).current_state

    def transition(self, transition_id):
        """Request a lifecycle transition; return whether it succeeded."""
        client = self.node.create_client(ChangeState, '/state_estimation/change_state')
        request = ChangeState.Request()
        request.transition.id = transition_id
        return self.call(client, request).success

    def wait_for_state(self, state_id, timeout=TIMEOUT_S):
        """Wait for the node to reach a lifecycle state."""
        deadline = time.monotonic() + timeout
        state = self.get_state()
        while state.id != state_id and time.monotonic() < deadline:
            time.sleep(0.2)
            state = self.get_state()
        self.assertEqual(state.id, state_id, state.label)

    def set_motion_model(self, model):
        """Set the motion_model parameter (read at configure)."""
        client = self.node.create_client(SetParameters, '/state_estimation/set_parameters')
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name='motion_model',
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=model))]
        result = self.call(client, request).results[0]
        self.assertTrue(result.successful, result.reason)

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

    def test_7_deactivate_stops_odom_and_is_stale(self):
        """VAL-011: deactivate stops /odom; health is STALE "inactive"."""
        type(self).send_imu = True
        type(self).send_wheel = True
        self.wait_for_diagnostic(DiagnosticStatus.OK)
        time.sleep(1.0)  # drive away from the odom origin
        self.assertTrue(self.transition(Transition.TRANSITION_DEACTIVATE))
        self.wait_for_state(State.PRIMARY_STATE_INACTIVE)
        self.wait_for_diagnostic(DiagnosticStatus.STALE, 'inactive')
        start = time.monotonic()
        time.sleep(1.0)
        self.assertEqual(self.odom_since(start), [])

    def test_8_activate_restarts_odom_at_the_car(self):
        """VAL-011: activate restarts odom at the car, and health is OK again."""
        with self.lock:
            before = self.odom[-1][1].pose.pose.position
        self.assertGreater(before.x, 0.5)  # odom had left its origin
        start = time.monotonic()
        self.assertTrue(self.transition(Transition.TRANSITION_ACTIVATE))
        self.wait_for_diagnostic(DiagnosticStatus.OK)
        first = self.odom_since(start)[0].pose.pose.position
        self.assertLess(abs(first.x), 0.2)
        self.assertLess(abs(first.y), 0.2)

    def test_9_motion_model_tank_fails_unicycle_runs(self, proc_output):
        """REQ-GNC-007, VAL-012: 'tank' fails configure; 'unicycle' runs."""
        self.assertTrue(self.transition(Transition.TRANSITION_DEACTIVATE))
        self.assertTrue(self.transition(Transition.TRANSITION_CLEANUP))
        self.wait_for_state(State.PRIMARY_STATE_UNCONFIGURED)

        self.set_motion_model('tank')
        self.assertFalse(self.transition(Transition.TRANSITION_CONFIGURE))
        proc_output.assertWaitFor("unknown motion_model 'tank'", timeout=TIMEOUT_S)
        self.assertEqual(self.get_state().id, State.PRIMARY_STATE_UNCONFIGURED)

        # The launch file's autostart activates the node after any configure.
        self.set_motion_model('unicycle')
        self.assertTrue(self.transition(Transition.TRANSITION_CONFIGURE))
        self.wait_for_state(State.PRIMARY_STATE_ACTIVE)
        status = self.wait_for_diagnostic(DiagnosticStatus.OK)
        values = {v.key: v.value for v in status.values}
        self.assertEqual(values['motion_model'], 'unicycle')
        start = time.monotonic()
        time.sleep(1.0)
        last = self.odom_since(start)[-1]
        self.assertAlmostEqual(last.twist.twist.linear.x, SPEED, delta=0.05)
        self.assertGreater(last.pose.pose.position.x, 0.5)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Checks after the launched processes stop."""

    def test_exit_codes(self, proc_info):
        """Every launched process exits cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
