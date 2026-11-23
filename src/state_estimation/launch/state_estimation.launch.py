# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Launch state_estimation (VEH.GNC.SE) as a managed lifecycle node.

With autostart (the default) the node is configured and then activated;
otherwise it waits unconfigured for a lifecycle manager.

Launch arguments: use_sim_time (true in simulation, default false),
params_file (default: this package's config/state_estimation.yaml) and
autostart (configure and activate on launch, default true).
"""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Launch the node and, with autostart, configure then activate it."""
    default_params = os.path.join(
        get_package_share_directory('state_estimation'), 'config', 'state_estimation.yaml'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    node = LifecycleNode(
        package='state_estimation',
        executable='state_estimation_node',
        name='state_estimation',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': use_sim_time}],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )
    activate_when_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(autostart),
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('autostart', default_value='true'),
        node,
        activate_when_inactive,
        configure,
    ])
