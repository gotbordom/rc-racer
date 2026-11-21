#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Make the teleop's turn axis a steering angle, as on an RC transmitter.

teleop_twist_joy scales the stick to a fixed angular.z. The drive turns a
yaw rate into a steering angle with delta = atan(L * w / v) (ICD-GNC-ACT
GNC-ACT-01), so the same stick gives less steering the faster the car goes:
full stick was full lock at 0.5 m/s but only a third of it at 3 m/s.

This relay reads angular.z as a steering fraction (-1 = full right,
+1 = full left) on ~/input (teleop's cmd_vel) and publishes /cmd_vel with
the yaw rate for that angle at the commanded speed:

    w = v * tan(fraction * max_steering_angle) / wheelbase

So full stick is full lock at any speed, and the wheels point the way the
stick does in reverse too. With v = 0 there is no yaw rate to send, so the
wheels do not move (an Ackermann car cannot turn in place).

Parameters (must match the car, ICD-GNC-ACT): wheelbase (m),
max_steering_angle (rad).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException


def yaw_rate(speed, fraction, wheelbase, max_steering_angle):
    """Return the yaw rate that steers at fraction of full lock at speed."""
    fraction = max(-1.0, min(1.0, fraction))
    return speed * math.tan(fraction * max_steering_angle) / wheelbase


def main():
    rclpy.init()
    node = rclpy.create_node('steering_relay')
    wheelbase = node.declare_parameter('wheelbase', 0.384).value
    max_angle = node.declare_parameter('max_steering_angle', 0.52).value
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def on_command(msg):
        out = Twist()
        out.linear.x = msg.linear.x
        out.angular.z = yaw_rate(msg.linear.x, msg.angular.z, wheelbase, max_angle)
        pub.publish(out)

    node.create_subscription(Twist, '~/input', on_command, 10)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
