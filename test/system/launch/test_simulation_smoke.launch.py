# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
System Test: Simulation Smoke.

Validates:
  - Gazebo Fortress starts headless and spawns the robot
  - Simulation clock is published (/clock)
  - Simulated sensors publish plausible data through ros_gz_bridge
      /imu     (sensor_msgs/Imu)
      /navsat  (sensor_msgs/NavSatFix)
      /camera  (sensor_msgs/Image)
  - Sensor data is stamped with its TF frame, and that frame is in the TF
    tree under base_link (ICD-SNS-GNC, Architecture Conventions TF registry)
  - Sensor covariances match config/sensor_noise.yaml (never zero), and the
    noise actually added matches the stated covariance (DEV-02, REQ-GNC-006)
  - The car tracks /cmd_vel (Ackermann drive): ground-truth speed and turn
    rate match the command, and /wheel/odom is published
  - Every process (including Gazebo) exits cleanly on shutdown

This is the foundation for every other simulation-based test: if this
fails, nothing downstream of the sensors or drive can be trusted.

Characteristics:
  - SLOW: ~1-2 minutes (Gazebo startup + software rendering on CI)

CI/CD: Nightly
"""

import math
import os
import re
import subprocess
import time
import unittest

import statistics

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix

# Gazebo startup on a CPU-only CI runner is slow; be generous
SENSOR_TIMEOUT_S = 90.0

GRAVITY = 9.81

SENSOR_NOISE = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rc_racer_simulation'), 'config', 'sensor_noise.yaml'
)))['sensor_adapter']['ros__parameters']
NOISE_SAMPLES = 300
NOISE_TOLERANCE = 0.3  # measured stddev within 30% of configured

# Sensor topic -> (message type, TF frame it must be stamped with)
SENSOR_FRAMES = {
    '/imu': (Imu, 'imu_link'),
    '/navsat': (NavSatFix, 'gps_link'),
    '/camera': (Image, 'camera_optical_link'),
    '/camera_info': (CameraInfo, 'camera_optical_link'),
}

# Drive tests: command (DRIVE_SPEED, turn rate) for DRIVE_TIME_S of SIM time and
# compare against Gazebo ground truth. Tolerances allow for the acceleration
# ramp and a little tire slip, but catch a car that skids instead of rolling.
# The turn test lets the steering settle first: Fortress' AckermannSteering
# steers with a fixed ~1 s time constant (no gain parameter until Garden).
DRIVE_SPEED = 0.5
DRIVE_TURN_RATE = 0.5      # rad/s; radius 1 m, within the steering limit
DRIVE_TIME_S = 3.0
STEERING_SETTLE_S = 3.0
SPEED_TOLERANCE = 0.2      # fraction of commanded
TURN_RATE_TOLERANCE = 0.2  # fraction of commanded

POSE_RE = re.compile(r'\[\s*(\S+)\s+(\S+)\s+(\S+)\s*\]')


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the RC Racer simulation headless."""
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rc_racer_simulation'),
                'launch',
                'simulation.launch.py',
            )
        ),
        launch_arguments={'headless': 'true'}.items(),
    )

    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class SimulationTestCase(unittest.TestCase):
    """Shared rclpy setup and helpers for simulation tests."""

    @classmethod
    def setUpClass(cls):
        """Initialize rclpy once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown rclpy."""
        rclpy.shutdown()

    def setUp(self):
        """Create a fresh test node per test."""
        self.node = rclpy.create_node('test_simulation_sensors')

    def tearDown(self):
        """Destroy the test node."""
        self.node.destroy_node()

    def _wait_for_message(self, msg_type, topic, timeout=SENSOR_TIMEOUT_S):
        """Spin until one message arrives on topic, or fail after timeout."""
        received = []
        sub = self.node.create_subscription(
            msg_type, topic, received.append, qos_profile_sensor_data)
        try:
            end = time.monotonic() + timeout
            while not received and time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(sub)

        self.assertTrue(received, f'No message on {topic} within {timeout:.0f}s')
        return received[0]


class TestSimulationSensors(SimulationTestCase):
    """Verify the simulation publishes clock and sensor data."""

    def test_clock_published(self):
        """Simulation clock is published and advancing."""
        first = self._wait_for_message(Clock, '/clock')
        time.sleep(1.0)
        second = self._wait_for_message(Clock, '/clock')

        t0 = first.clock.sec + first.clock.nanosec * 1e-9
        t1 = second.clock.sec + second.clock.nanosec * 1e-9
        self.assertGreater(t1, t0, 'Simulation clock is not advancing (is gz paused?)')

    def test_imu_data_valid(self):
        """IMU publishes finite data with gravity on Z while at rest."""
        msg = self._wait_for_message(Imu, '/imu')

        accel = msg.linear_acceleration
        for value in (accel.x, accel.y, accel.z):
            self.assertTrue(math.isfinite(value))
        self.assertAlmostEqual(accel.z, GRAVITY, delta=1.0)

    def test_navsat_data_valid(self):
        """GPS publishes a valid latitude/longitude fix."""
        msg = self._wait_for_message(NavSatFix, '/navsat')

        self.assertTrue(math.isfinite(msg.latitude))
        self.assertTrue(math.isfinite(msg.longitude))
        self.assertGreaterEqual(msg.latitude, -90.0)
        self.assertLessEqual(msg.latitude, 90.0)
        self.assertGreaterEqual(msg.longitude, -180.0)
        self.assertLessEqual(msg.longitude, 180.0)

    def test_camera_data_valid(self):
        """Camera publishes images at the configured resolution."""
        msg = self._wait_for_message(Image, '/camera')

        self.assertEqual(msg.width, 640)
        self.assertEqual(msg.height, 480)
        self.assertEqual(len(msg.data), msg.step * msg.height)


class TestSensorNoise(SimulationTestCase):
    """Verify sensor covariances and noise match config/sensor_noise.yaml."""

    def _diag(self, cov, n):
        return [cov[i * (n + 1)] for i in range(n)]

    def test_imu_covariance(self):
        """IMU covariances are the squares of the configured stddevs."""
        msg = self._wait_for_message(Imu, '/imu')
        imu = SENSOR_NOISE['imu']
        rp, yaw = imu['orientation_roll_pitch_stddev'], imu['orientation_yaw_stddev']
        self.assertEqual(self._diag(msg.orientation_covariance, 3),
                         [rp ** 2, rp ** 2, yaw ** 2])
        self.assertEqual(self._diag(msg.angular_velocity_covariance, 3),
                         [imu['angular_velocity_stddev'] ** 2] * 3)
        self.assertEqual(self._diag(msg.linear_acceleration_covariance, 3),
                         [imu['linear_acceleration_stddev'] ** 2] * 3)

    def test_navsat_covariance(self):
        """GNSS covariance is diagonal-known and matches the configured stddevs."""
        msg = self._wait_for_message(NavSatFix, '/navsat')
        nav = SENSOR_NOISE['navsat']
        self.assertEqual(msg.position_covariance_type, NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN)
        h, v = nav['horizontal_stddev'] ** 2, nav['vertical_stddev'] ** 2
        self.assertEqual(self._diag(msg.position_covariance, 3), [h, h, v])

    def test_wheel_odom_covariance(self):
        """Wheel odometry: only linear.x is measured; the rest is 'unprovided'."""
        msg = self._wait_for_message(Odometry, '/wheel/odom')
        wheel = SENSOR_NOISE['wheel_odom']
        big = wheel['unprovided_variance']
        self.assertEqual(self._diag(msg.twist.covariance, 6),
                         [wheel['linear_x_stddev'] ** 2] + [big] * 5)
        self.assertEqual(self._diag(msg.pose.covariance, 6), [big] * 6)

    def test_imu_noise_matches_covariance(self):
        """Gyro and accelerometer noise (noisy - truth) has the stated stddev."""
        noisy, truth = {}, {}

        def key(msg):
            return (msg.header.stamp.sec, msg.header.stamp.nanosec)

        subs = [
            self.node.create_subscription(
                Imu, '/imu', lambda m: noisy.__setitem__(key(m), m), qos_profile_sensor_data),
            self.node.create_subscription(
                Imu, '/sim/truth/imu', lambda m: truth.__setitem__(key(m), m),
                qos_profile_sensor_data),
        ]
        try:
            end = time.monotonic() + SENSOR_TIMEOUT_S
            while len(set(noisy) & set(truth)) < NOISE_SAMPLES and time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        finally:
            for sub in subs:
                self.node.destroy_subscription(sub)

        stamps = sorted(set(noisy) & set(truth))
        self.assertGreaterEqual(len(stamps), NOISE_SAMPLES, 'Too few matched IMU samples')
        gyro = [noisy[s].angular_velocity.z - truth[s].angular_velocity.z for s in stamps]
        accel = [noisy[s].linear_acceleration.x - truth[s].linear_acceleration.x
                 for s in stamps]
        imu = SENSOR_NOISE['imu']
        for name, residuals, expected in (
                ('gyro z', gyro, imu['angular_velocity_stddev']),
                ('accel x', accel, imu['linear_acceleration_stddev'])):
            with self.subTest(axis=name):
                self.assertAlmostEqual(
                    statistics.pstdev(residuals), expected, delta=expected * NOISE_TOLERANCE,
                    msg=f'{name} noise does not match its stated covariance')


class TestSensorFrames(SimulationTestCase):
    """Verify sensor data is stamped with frames that exist in the TF tree."""

    def test_sensor_frame_ids(self):
        """Each sensor topic uses its URDF frame, and that frame is under base_link."""
        tf_buffer = Buffer()
        TransformListener(tf_buffer, self.node)
        for topic, (msg_type, expected_frame) in SENSOR_FRAMES.items():
            with self.subTest(topic=topic):
                msg = self._wait_for_message(msg_type, topic)
                self.assertEqual(msg.header.frame_id, expected_frame)

                end = time.monotonic() + 10.0
                while (not tf_buffer.can_transform('base_link', expected_frame, Time())
                       and time.monotonic() < end):
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                self.assertTrue(
                    tf_buffer.can_transform('base_link', expected_frame, Time()),
                    f'No TF path from base_link to {expected_frame}')


def _ground_truth_pose():
    """Return the car's true (x, y, yaw) from Gazebo, or None."""
    try:
        out = subprocess.run(['ign', 'model', '-m', 'rc_racer', '-p'],
                             capture_output=True, text=True, timeout=15).stdout
    except subprocess.TimeoutExpired:
        return None
    matches = POSE_RE.findall(out)
    if len(matches) < 2:
        return None
    (x, y, _), (_, _, yaw) = matches[-2], matches[-1]
    return float(x), float(y), float(yaw)


class TestSimulationDrive(SimulationTestCase):
    """Verify the Ackermann drive tracks velocity commands (ground truth)."""

    def setUp(self):
        """Create a test node on simulation time."""
        self.node = rclpy.create_node(
            'test_simulation_drive',
            parameter_overrides=[Parameter('use_sim_time', value=True)])

    def _sim_time(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _drive(self, linear_x, angular_z, sim_duration, settle=0.0):
        """
        Publish a constant Twist for settle + sim_duration SIM seconds, then stop.

        Measures over sim_duration only, after settle seconds have passed.
        Returns (start_pose, end_pose, elapsed_sim_seconds) from ground truth.
        """
        pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        try:
            wall_end = time.monotonic() + SENSOR_TIMEOUT_S
            while self._sim_time() == 0.0 and time.monotonic() < wall_end:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            self.assertGreater(self._sim_time(), 0.0, 'No simulation time on /clock')

            settle_end = self._sim_time() + settle
            while self._sim_time() < settle_end:
                pub.publish(cmd)
                rclpy.spin_once(self.node, timeout_sec=0.05)

            start = _ground_truth_pose()
            t0 = self._sim_time()
            while self._sim_time() - t0 < sim_duration:
                pub.publish(cmd)
                rclpy.spin_once(self.node, timeout_sec=0.05)
            end = _ground_truth_pose()
            elapsed = self._sim_time() - t0

            stop_end = self._sim_time() + 1.0
            while self._sim_time() < stop_end:
                pub.publish(Twist())
                rclpy.spin_once(self.node, timeout_sec=0.05)
        finally:
            self.node.destroy_publisher(pub)

        self.assertIsNotNone(start, 'Could not read car pose from Gazebo')
        self.assertIsNotNone(end, 'Could not read car pose from Gazebo')
        return start, end, elapsed

    def test_drives_straight_at_commanded_speed(self):
        """Forward command moves the car straight at about the commanded speed."""
        (x0, y0, yaw0), (x1, y1, yaw1), elapsed = self._drive(DRIVE_SPEED, 0.0, DRIVE_TIME_S)

        speed = math.hypot(x1 - x0, y1 - y0) / elapsed
        self.assertAlmostEqual(
            speed, DRIVE_SPEED, delta=DRIVE_SPEED * SPEED_TOLERANCE,
            msg=f'Car moved at {speed:.3f} m/s; commanded {DRIVE_SPEED} m/s '
                '(wheels skidding or dragging?)')
        heading_change = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        self.assertLess(abs(heading_change), 0.1, 'Car veered while driving straight')

    def test_turns_at_commanded_rate(self):
        """Turn command yaws the car at about the commanded rate (no front-end skid)."""
        (_, _, yaw0), (_, _, yaw1), elapsed = self._drive(
            DRIVE_SPEED, DRIVE_TURN_RATE, DRIVE_TIME_S, settle=STEERING_SETTLE_S)

        # Yaw change over 3 s at 0.5 rad/s is 1.5 rad, < pi, so atan2 unwraps it
        turn_rate = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)) / elapsed
        self.assertAlmostEqual(
            turn_rate, DRIVE_TURN_RATE, delta=DRIVE_TURN_RATE * TURN_RATE_TOLERANCE,
            msg=f'Car turned at {turn_rate:.3f} rad/s; commanded {DRIVE_TURN_RATE} rad/s '
                '(front wheels sliding instead of steering?)')

    def test_wheel_odometry_published(self):
        """Wheel odometry (encoder stand-in) is published on /wheel/odom."""
        msg = self._wait_for_message(Odometry, '/wheel/odom')
        self.assertEqual(msg.header.frame_id, 'odom')


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Verify the simulation shuts down cleanly."""

    def test_exit_codes(self, proc_info):
        """All processes, including Gazebo, exit with code 0 on SIGINT."""
        launch_testing.asserts.assertExitCodes(proc_info)
