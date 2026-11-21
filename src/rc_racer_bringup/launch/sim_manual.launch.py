# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Drive the simulated car by hand and watch the estimate.

Starts the Gazebo simulation (GUI), state_estimation on sim time (lifecycle
autostart), RViz (TF tree, the car, and /odom with its covariance) and,
optionally, Xbox controller teleop (config/xbox_teleop.yaml: hold LB, left
stick = speed, right stick = turn, RB = turbo).

Launch arguments: world (default practice_course.world), headless
(default false), rviz (default true), joystick (default true), tire_mu
(tyre-ground friction, default 0.5).

For the keyboard instead of a controller, in another terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    """Launch the simulation, state_estimation, RViz and joystick teleop."""
    bringup = get_package_share_directory('rc_racer_bringup')
    teleop_config = os.path.join(bringup, 'config', 'xbox_teleop.yaml')
    sim_time = {'use_sim_time': True}
    joystick = IfCondition(LaunchConfiguration('joystick'))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='practice_course.world'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('joystick', default_value='true'),
        DeclareLaunchArgument('tire_mu', default_value='0.5'),

        _include('rc_racer_simulation', 'simulation.launch.py', {
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'tire_mu': LaunchConfiguration('tire_mu'),
        }),
        _include('state_estimation', 'state_estimation.launch.py', {'use_sim_time': 'true'}),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(bringup, 'rviz', 'sim_manual.rviz')],
            parameters=[sim_time],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        # SDL game-controller layout: the same indices for any Xbox pad
        Node(
            package='joy',
            executable='game_controller_node',
            name='joy_node',
            parameters=[teleop_config, sim_time],
            condition=joystick,
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[teleop_config, sim_time],
            condition=joystick,
        ),
    ])
