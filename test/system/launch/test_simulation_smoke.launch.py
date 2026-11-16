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
  - The car responds to /cmd_vel (Ackermann drive) and reports /wheel/odom

This is the foundation for every other simulation-based test: if this
fails, nothing downstream of the sensors or drive can be trusted.

Characteristics:
  - SLOW: ~1-2 minutes (Gazebo startup + software rendering on CI)

CI/CD: Nightly
"""

import math
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, NavSatFix

# Gazebo startup on a CPU-only CI runner is slow; be generous
SENSOR_TIMEOUT_S = 90.0

GRAVITY = 9.81

# Drive test: command forward at DRIVE_SPEED for DRIVE_TIME_S (wall clock)
DRIVE_SPEED = 0.5
DRIVE_TIME_S = 5.0
MIN_DISTANCE_M = 0.2


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


class TestSimulationDrive(SimulationTestCase):
    """Verify the Ackermann drive responds to velocity commands."""

    def _drive(self, linear_x, angular_z, duration):
        """Publish a constant Twist on /cmd_vel for duration seconds, then stop."""
        pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        try:
            end = time.monotonic() + duration
            while time.monotonic() < end:
                pub.publish(cmd)
                rclpy.spin_once(self.node, timeout_sec=0.05)
            pub.publish(Twist())
        finally:
            self.node.destroy_publisher(pub)

    def test_drives_forward(self):
        """Commanding forward velocity moves the car forward in /wheel/odom."""
        start = self._wait_for_message(Odometry, '/wheel/odom').pose.pose.position

        self._drive(DRIVE_SPEED, 0.0, DRIVE_TIME_S)

        end = self._wait_for_message(Odometry, '/wheel/odom').pose.pose.position
        self.assertGreater(
            end.x - start.x, MIN_DISTANCE_M,
            f'Car moved {end.x - start.x:.3f} m forward; expected > {MIN_DISTANCE_M} m')
