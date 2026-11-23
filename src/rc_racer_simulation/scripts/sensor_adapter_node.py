#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Simulated sensor adapter: the simulator's stand-in for the sensor drivers.

Gazebo's sensors are noise-free. This node takes their output on /sim/truth/*,
adds realistic noise, sets covariances from the same noise config, and
publishes the ICD-SNS-GNC topics (/imu, /navsat, /wheel/odom) - the way real
drivers set covariances from the sensor datasheet.

Noise is added here rather than in Gazebo because Fortress' NavSat sensor
applies its horizontal noise in degrees, and the Ackermann odometry has no
noise model at all. Logic lives in rc_racer_simulation.sensor_noise.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from rc_racer_simulation.sensor_noise import (
    GaussMarkov,
    ImuNoise,
    NavSatNoise,
    WheelOdomNoise,
    imu_covariances,
    make_rng,
    navsat_covariance,
    offset_geodetic,
    perturb_orientation,
    validate,
    wheel_odom_covariances,
)


def _stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class SensorAdapterNode(Node):
    """Adds noise and covariance to simulated sensor data."""

    def __init__(self):
        super().__init__('sensor_adapter')
        seed = self.declare_parameter('seed', 0).value
        self._rng = make_rng(seed)

        self._imu = validate(ImuNoise(
            self._param('imu.angular_velocity_stddev', 0.005),
            self._param('imu.linear_acceleration_stddev', 0.05),
            self._param('imu.orientation_roll_pitch_stddev', 0.02),
            self._param('imu.orientation_yaw_stddev', 0.05),
        ))
        self._navsat = validate(NavSatNoise(
            self._param('navsat.horizontal_stddev', 2.0),
            self._param('navsat.vertical_stddev', 4.0),
            self._param('navsat.correlation_time', 30.0),
        ))
        self._wheel = validate(WheelOdomNoise(
            self._param('wheel_odom.linear_x_stddev', 0.02),
            self._param('wheel_odom.unprovided_variance', 1.0e6),
        ))

        self._imu_cov = imu_covariances(self._imu)
        self._navsat_cov = navsat_covariance(self._navsat)
        self._wheel_pose_cov, self._wheel_twist_cov = wheel_odom_covariances(self._wheel)

        gm = self._navsat
        self._gps_error = [GaussMarkov(gm.horizontal_stddev, gm.correlation_time, self._rng),
                           GaussMarkov(gm.horizontal_stddev, gm.correlation_time, self._rng),
                           GaussMarkov(gm.vertical_stddev, gm.correlation_time, self._rng)]
        self._last_gps_stamp = None

        qos = qos_profile_sensor_data
        self._imu_pub = self.create_publisher(Imu, '/imu', qos)
        self._navsat_pub = self.create_publisher(NavSatFix, '/navsat', qos)
        self._wheel_pub = self.create_publisher(Odometry, '/wheel/odom', qos)
        self.create_subscription(Imu, '/sim/truth/imu', self._on_imu, qos)
        self.create_subscription(NavSatFix, '/sim/truth/navsat', self._on_navsat, qos)
        self.create_subscription(Odometry, '/sim/truth/wheel_odom', self._on_wheel_odom, qos)
        self.get_logger().info(
            f'Sensor noise: {self._imu}, {self._navsat}, {self._wheel} (seed {seed})')

    def _param(self, name, default):
        return float(self.declare_parameter(name, default).value)

    def _gauss(self, stddev):
        return self._rng.gauss(0.0, stddev) if stddev > 0 else 0.0

    def _on_imu(self, msg):
        n = self._imu
        g, a = n.angular_velocity_stddev, n.linear_acceleration_stddev
        msg.angular_velocity.x += self._gauss(g)
        msg.angular_velocity.y += self._gauss(g)
        msg.angular_velocity.z += self._gauss(g)
        msg.linear_acceleration.x += self._gauss(a)
        msg.linear_acceleration.y += self._gauss(a)
        msg.linear_acceleration.z += self._gauss(a)
        q = msg.orientation
        q.x, q.y, q.z, q.w = perturb_orientation(
            (q.x, q.y, q.z, q.w),
            self._gauss(n.orientation_roll_pitch_stddev),
            self._gauss(n.orientation_roll_pitch_stddev),
            self._gauss(n.orientation_yaw_stddev))
        (msg.orientation_covariance, msg.angular_velocity_covariance,
         msg.linear_acceleration_covariance) = self._imu_cov
        self._imu_pub.publish(msg)

    def _on_navsat(self, msg):
        stamp = _stamp_seconds(msg)
        dt = 0.0 if self._last_gps_stamp is None else stamp - self._last_gps_stamp
        self._last_gps_stamp = stamp
        north, east, up = (e.step(dt) for e in self._gps_error)

        if msg.status.status != NavSatStatus.STATUS_NO_FIX:
            msg.latitude, msg.longitude = offset_geodetic(msg.latitude, msg.longitude, north, east)
            msg.altitude += up
        msg.position_covariance = self._navsat_cov
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._navsat_pub.publish(msg)

    def _on_wheel_odom(self, msg):
        msg.twist.twist.linear.x += self._gauss(self._wheel.linear_x_stddev)
        msg.pose.covariance = self._wheel_pose_cov
        msg.twist.covariance = self._wheel_twist_cov
        self._wheel_pub.publish(msg)


def main():
    """Run the sensor adapter node."""
    rclpy.init()
    node = SensorAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
