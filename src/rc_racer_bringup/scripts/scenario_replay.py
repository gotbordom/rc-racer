#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Drive the running sim from a validation scenario file, to watch it.

Plays the scenario's steps (test/system/scenarios/README.md) on /cmd_vel
at 20 Hz in sim time, as the scenario system test does through
steering_relay: each step's steering fraction becomes the yaw rate for
that angle at the step's speed. Nothing is checked; run the scenario's
test for that (make test-system FILTER=test_scenario_<name>).

The scenario starts wherever the car is, so restart the sim (or put the
car back at the spawn point) first for the same drive as the test. The
world and tire_mu of the running sim are not changed; a mismatch with the
file is reported.

Usage: scenario_replay.py FILE
Parameters (must match the car, ICD-GNC-ACT): wheelbase, max_steering_angle.
"""

import sys

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from scenario_record import gazebo_world, tire_mu
from steering_relay import yaw_rate

COMMAND_PERIOD_S = 0.05     # sim; 20 Hz, as the pad and the scenario test send


def command_at(steps, t):
    """Return (speed, steering fraction) at time t into the steps, or None after them."""
    start = 0.0
    for step in steps:
        if t < start + step['duration']:
            f = (t - start) / step['duration']
            v0 = step['speed']
            v1 = step.get('speed_end', v0)
            return v0 + f * (v1 - v0), step.get('steer', 0.0)
        start += step['duration']
    return None


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(args) != 1:
        print(__doc__)
        return 2
    with open(args[0]) as f:
        scenario = yaml.safe_load(f)

    rclpy.init()
    node = rclpy.create_node('scenario_replay',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    wheelbase = node.declare_parameter('wheelbase', 0.384).value
    max_angle = node.declare_parameter('max_steering_angle', 0.52).value
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    world = gazebo_world() + '.world'
    mu = tire_mu(node)
    if world != scenario.get('world', 'empty_course.world') or mu != scenario['tire_mu']:
        print(f'Note: the sim runs {world} at tire_mu {mu}; the scenario is for '
              f'{scenario.get("world", "empty_course.world")} at {scenario["tire_mu"]}. '
              'Restart with `make sim MU=... ` (and SIM_WORLD) to match.', flush=True)
    total = sum(s['duration'] for s in scenario['steps'])
    print(f'Replaying {scenario["id"]}: {scenario["title"]} ({total:.1f} s). Ctrl+C stops.',
          flush=True)

    start = [None]

    def tick():
        now = node.get_clock().now().nanoseconds * 1e-9
        if start[0] is None:
            start[0] = now
        command = command_at(scenario['steps'], now - start[0])
        msg = Twist()
        if command is None:
            pub.publish(msg)
            print('Done.', flush=True)
            raise SystemExit(0)
        speed, steer = command
        msg.linear.x = float(speed)
        msg.angular.z = yaw_rate(speed, steer, wheelbase, max_angle)
        pub.publish(msg)

    node.create_timer(COMMAND_PERIOD_S, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
