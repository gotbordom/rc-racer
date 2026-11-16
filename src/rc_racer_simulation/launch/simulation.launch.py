# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
RC Racer Simulation Launch File - Gazebo Fortress.

Launches Gazebo Fortress with the RC Racer robot model and sensors.
Uses ros_gz_bridge to connect Gazebo topics to ROS 2.

Usage:
    ros2 launch rc_racer_simulation simulation.launch.py
    ros2 launch rc_racer_simulation simulation.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for RC Racer simulation."""
    # Package directories
    pkg_rc_racer_sim = get_package_share_directory('rc_racer_simulation')

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

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)'
    )

    # Paths
    world_path = PathJoinSubstitution([pkg_rc_racer_sim, 'worlds', LaunchConfiguration('world')])
    urdf_path = os.path.join(pkg_rc_racer_sim, 'urdf', 'rc_racer.urdf.xacro')

    # Process URDF with xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Gazebo Fortress. Run `ign gazebo` directly rather than including
    # ros_gz_sim's gz_sim.launch.py: that wraps it in `sh -c`, so the SIGINT
    # sent on shutdown never reaches Gazebo and it gets SIGTERM'd after 5 s.
    # Gazebo exiting (e.g. GUI window closed) shuts down the whole launch.
    gz_sim_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', world_path],  # -r = run, -s = server only
        output='screen',
        on_exit=Shutdown(),
        condition=IfCondition(LaunchConfiguration('headless'))
    )

    gz_sim_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],  # -r = run (with GUI)
        output='screen',
        on_exit=Shutdown(),
        condition=UnlessCondition(LaunchConfiguration('headless'))
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

    # Spawn robot in Gazebo Fortress
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_rc_racer',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rc_racer',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
        output='screen'
    )

    # Bridge Gazebo topics to ROS 2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            # Direction: '[' = gz -> ROS, ']' = ROS -> gz, '@' = both
            # IMU
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # GPS/NavSat
            '/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            # Camera
            '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # Ackermann drive: commands in (ROS -> gz), wheel odometry out (gz -> ROS)
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/wheel/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # Joint states (wheel spin + steering angle) for robot_state_publisher
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        # Gazebo sensors are noise-free: publish them as truth for the sensor adapter
        remappings=[
            ('/imu', '/sim/truth/imu'),
            ('/navsat', '/sim/truth/navsat'),
            ('/wheel/odom', '/sim/truth/wheel_odom'),
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Simulated sensor drivers: add noise + covariance, publish the ICD topics
    sensor_adapter = Node(
        package='rc_racer_simulation',
        executable='sensor_adapter_node.py',
        name='sensor_adapter',
        output='screen',
        parameters=[
            os.path.join(pkg_rc_racer_sim, 'config', 'sensor_noise.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        headless_arg,
        gz_sim_server,
        gz_sim_gui,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        sensor_adapter,
    ])
