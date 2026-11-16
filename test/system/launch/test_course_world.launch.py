# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
System Test: Course World.

Validates that the generated practice course matches its course file
(rc_racer_simulation/courses/practice_course.yaml):
  - Every waypoint cone and obstacle exists in the running world
  - Each cone sits where the course file says, upright and on the ground
  - Noise-free NavSat (/sim/truth/navsat) at the start point reports the
    course's start lat/lon, i.e. the world origin and the course file share
    one coordinate frame. Truth is used because this checks the world, not
    GNSS accuracy (/navsat carries ~2 m of simulated receiver error)

If this fails, any test or planner that uses the course file as its map
would be steering to the wrong place.

Characteristics:
  - SLOW: ~1 minute (Gazebo startup)

CI/CD: Nightly
"""

import math
import os
import re
import subprocess
import sys
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

SIM_SHARE = get_package_share_directory('rc_racer_simulation')
sys.path.insert(0, os.path.join(SIM_SHARE, 'scripts'))
from generate_course_world import load_course, local_position  # noqa: E402

COURSE_NAME = 'practice_course'
COURSE = load_course(os.path.join(SIM_SHARE, 'courses', f'{COURSE_NAME}.yaml'))

WORLD_READY_TIMEOUT_S = 90.0
SENSOR_TIMEOUT_S = 90.0
POSITION_TOLERANCE_M = 0.05
UPRIGHT_TOLERANCE_RAD = 0.05
# GPS antenna is mounted 0.1 m behind base_link; allow for that plus settling
START_TOLERANCE_M = 0.5

POSE_RE = re.compile(r'\[\s*(\S+)\s+(\S+)\s+(\S+)\s*\]')


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the simulation headless in the practice course."""
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SIM_SHARE, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'headless': 'true',
            'world': f'{COURSE_NAME}.world',
        }.items(),
    )

    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


def _ign(*args):
    """Run an `ign` CLI command, returning stdout ('' on failure/timeout)."""
    try:
        return subprocess.run(['ign', *args], capture_output=True, text=True,
                              timeout=15).stdout
    except subprocess.TimeoutExpired:
        return ''


def _model_pose(name):
    """Return ((x, y, z), (roll, pitch, yaw)) for a model in the running world."""
    out = _ign('model', '-m', name, '-p')
    matches = POSE_RE.findall(out)
    if len(matches) < 2:
        return None
    xyz, rpy = matches[-2], matches[-1]
    return tuple(map(float, xyz)), tuple(map(float, rpy))


class TestCourseWorld(unittest.TestCase):
    """Verify the running world matches the course file."""

    @classmethod
    def setUpClass(cls):
        """Wait until Gazebo has loaded the course world and spawned the car."""
        end = time.monotonic() + WORLD_READY_TIMEOUT_S
        cls.models = []
        while time.monotonic() < end:
            out = _ign('model', '--list')
            cls.models = re.findall(r'^\s*-\s*(\S+)\s*$', out, re.MULTILINE)
            if 'rc_racer' in cls.models:
                break
            time.sleep(1.0)

    def test_world_loaded(self):
        """The course world is running with the car spawned."""
        self.assertIn(
            'rc_racer', self.models,
            f'Car not spawned within {WORLD_READY_TIMEOUT_S:.0f}s; models: {self.models}')

    def test_all_course_items_present(self):
        """Every waypoint cone and obstacle from the course file is in the world."""
        expected = [f'cone_{w["name"]}' for w in COURSE['waypoints']] + \
            [o['name'] for o in COURSE.get('obstacles', [])]
        missing = sorted(set(expected) - set(self.models))
        self.assertFalse(missing, f'Missing from world: {missing}')

    def test_cones_at_course_positions(self):
        """Each cone sits at its course position, upright and on the ground."""
        for waypoint in COURSE['waypoints']:
            name = f'cone_{waypoint["name"]}'
            with self.subTest(cone=name):
                pose = _model_pose(name)
                self.assertIsNotNone(pose, f'Could not read pose of {name}')
                (x, y, z), (roll, pitch, _) = pose
                ex, ey = local_position(waypoint, COURSE['start'])
                self.assertLess(math.hypot(x - ex, y - ey), POSITION_TOLERANCE_M,
                                f'{name} at ({x:.3f}, {y:.3f}), course says ({ex:.3f}, {ey:.3f})')
                self.assertAlmostEqual(z, 0.0, delta=0.02, msg=f'{name} not resting on the ground')
                self.assertLess(abs(roll), UPRIGHT_TOLERANCE_RAD, f'{name} tipped over')
                self.assertLess(abs(pitch), UPRIGHT_TOLERANCE_RAD, f'{name} tipped over')

    def test_navsat_at_start_matches_course(self):
        """Noise-free GPS at the start point reports the course file's start lat/lon."""
        rclpy.init()
        node = rclpy.create_node('test_course_world')
        try:
            received = []
            node.create_subscription(
                NavSatFix, '/sim/truth/navsat', received.append, qos_profile_sensor_data)
            end = time.monotonic() + SENSOR_TIMEOUT_S
            while not received and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()
            rclpy.shutdown()

        self.assertTrue(received,
                        f'No message on /sim/truth/navsat within {SENSOR_TIMEOUT_S:.0f}s')
        fix = received[0]
        east, north = local_position(
            {'latitude': fix.latitude, 'longitude': fix.longitude}, COURSE['start'])
        self.assertLess(math.hypot(east, north), START_TOLERANCE_M,
                        f'GPS reads {math.hypot(east, north):.2f} m from the course start')


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Verify the simulation shuts down cleanly."""

    def test_exit_codes(self, proc_info):
        """All processes, including Gazebo, exit with code 0."""
        launch_testing.asserts.assertExitCodes(proc_info)
