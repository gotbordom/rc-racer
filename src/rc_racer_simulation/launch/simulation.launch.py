# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
RC Racer Simulation Launch File.

Launches Gazebo with the RC Racer robot model and sensors.

Usage:
    ros2 launch rc_racer_simulation simulation.launch.py
    ros2 launch rc_racer_simulation simulation.launch.py world:=cone_course.world
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for RC Racer simulation."""
    # Package directories
    pkg_rc_racer_sim = get_package_share_directory('rc_racer_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_course.world',
        description='Gazebo world file name'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Paths
    world_path = [pkg_rc_racer_sim, '/worlds/', LaunchConfiguration('world')]
    urdf_path = os.path.join(pkg_rc_racer_sim, 'urdf', 'rc_racer.urdf.xacro')

    # Process URDF with xacro - wrap in ParameterValue to avoid YAML parsing issues
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Gazebo server
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_rc_racer',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rc_racer',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
        output='screen'
    )

    # Joint state publisher (for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
    ])
