# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Simulated sensor noise models (no ROS dependencies).

Gazebo publishes noise-free sensor data. These models add realistic noise and
compute the covariance that goes with it, from one set of standard deviations
(config/sensor_noise.yaml), so the two can never disagree.

It also holds TruthAccelerometer, which derives an ideal accelerometer's
reading from the body's true motion: Gazebo's IMU samples acceleration at an
instant and misses contact impulses (ICD-VEH-TST DEV-07).

Used by scripts/sensor_adapter_node.py; unit-tested in test/test_sensor_noise.py.
"""

import math
import random
from dataclasses import dataclass

# WGS84 ellipsoid, for converting metre offsets to lat/lon offsets
_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3


@dataclass(frozen=True)
class ImuNoise:
    """IMU noise standard deviations (SI units)."""

    angular_velocity_stddev: float
    linear_acceleration_stddev: float
    orientation_roll_pitch_stddev: float
    orientation_yaw_stddev: float


@dataclass(frozen=True)
class NavSatNoise:
    """GNSS noise standard deviations (metres) and error correlation time (s)."""

    horizontal_stddev: float
    vertical_stddev: float
    correlation_time: float


@dataclass(frozen=True)
class WheelOdomNoise:
    """Wheel odometry noise: forward speed stddev (m/s), variance for unmeasured fields."""

    linear_x_stddev: float
    unprovided_variance: float


def validate(noise):
    """Raise ValueError if any field of a noise config is negative."""
    for name, value in vars(noise).items():
        if value < 0:
            raise ValueError(f'{type(noise).__name__}.{name} must be >= 0, got {value}')
    return noise


def diagonal3(variances):
    """Row-major 3x3 covariance with the given diagonal."""
    vx, vy, vz = variances
    return [vx, 0.0, 0.0, 0.0, vy, 0.0, 0.0, 0.0, vz]


def diagonal6(variances):
    """Row-major 6x6 covariance with the given diagonal."""
    cov = [0.0] * 36
    for i, v in enumerate(variances):
        cov[i * 7] = v
    return cov


def imu_covariances(noise):
    """Return (orientation, angular_velocity, linear_acceleration) 3x3 covariances."""
    rp, yaw = noise.orientation_roll_pitch_stddev ** 2, noise.orientation_yaw_stddev ** 2
    gyro = noise.angular_velocity_stddev ** 2
    accel = noise.linear_acceleration_stddev ** 2
    return diagonal3((rp, rp, yaw)), diagonal3((gyro,) * 3), diagonal3((accel,) * 3)


def navsat_covariance(noise):
    """ENU position covariance (m^2) for a NavSatFix."""
    h, v = noise.horizontal_stddev ** 2, noise.vertical_stddev ** 2
    return diagonal3((h, h, v))


def wheel_odom_covariances(noise):
    """
    Return (pose, twist) 6x6 covariances for wheel odometry.

    Only twist.linear.x is measured by the hardware (ICD-SNS-GNC); every other
    field gets the large 'unprovided' variance.
    """
    big = noise.unprovided_variance
    twist = diagonal6((noise.linear_x_stddev ** 2, big, big, big, big, big))
    return diagonal6((big,) * 6), twist


class GaussMarkov:
    """
    First-order Gauss-Markov process: zero-mean noise with a correlation time.

    Stationary standard deviation is `stddev` for any sample interval.
    correlation_time == 0 gives white noise. Real GNSS error wanders slowly
    rather than jumping each fix; white noise would let a filter average it
    away unrealistically fast.
    """

    def __init__(self, stddev, correlation_time, rng):
        self.stddev = stddev
        self.correlation_time = correlation_time
        self._rng = rng
        self.value = rng.gauss(0.0, stddev) if stddev > 0 else 0.0

    def step(self, dt):
        """Advance by dt seconds and return the new value."""
        if self.stddev == 0:
            return 0.0
        if self.correlation_time <= 0:
            self.value = self._rng.gauss(0.0, self.stddev)
            return self.value
        # dt <= 0 (e.g. duplicate timestamp) gives a = 1: value unchanged
        a = math.exp(-max(dt, 0.0) / self.correlation_time)
        self.value = a * self.value + math.sqrt(1.0 - a * a) * self._rng.gauss(0.0, self.stddev)
        return self.value


def offset_geodetic(lat_deg, lon_deg, north_m, east_m):
    """Move a WGS84 lat/lon by a small north/east offset in metres."""
    lat = math.radians(lat_deg)
    s2 = math.sin(lat) ** 2
    meridian_radius = _WGS84_A * (1 - _WGS84_E2) / (1 - _WGS84_E2 * s2) ** 1.5
    normal_radius = _WGS84_A / math.sqrt(1 - _WGS84_E2 * s2)
    return (lat_deg + math.degrees(north_m / meridian_radius),
            lon_deg + math.degrees(east_m / (normal_radius * math.cos(lat))))


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quaternion_from_rpy(roll, pitch, yaw):
    """(x, y, z, w) quaternion for intrinsic roll-pitch-yaw (REP-103)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def perturb_orientation(q, roll, pitch, yaw):
    """Rotate quaternion q (x, y, z, w) by a small body-frame roll/pitch/yaw error."""
    x, y, z, w = _quat_multiply(q, quaternion_from_rpy(roll, pitch, yaw))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


def make_rng(seed):
    """Random generator; seed 0 means a different sequence every run."""
    return random.Random(None if seed == 0 else seed)


def rotate(q, v):
    """Rotate vector v by unit quaternion q (x, y, z, w)."""
    x, y, z, _ = _quat_multiply(_quat_multiply(q, (v[0], v[1], v[2], 0.0)),
                                (-q[0], -q[1], -q[2], q[3]))
    return x, y, z


def rotate_inverse(q, v):
    """Rotate vector v by the inverse of unit quaternion q (world to body)."""
    return rotate((-q[0], -q[1], -q[2], q[3]), v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class TruthAccelerometer:
    """
    An ideal accelerometer's reading, from the body's true motion (DEV-07).

    A real accelerometer senses continuously, so a contact impulse shows in
    its reading. Gazebo's IMU samples the acceleration at single instants and
    misses most of one. This takes the body's true pose and body-frame
    velocity as they come (Gazebo's OdometryPublisher, add_motion()), and
    each read() returns the specific force averaged over the interval since
    the previous read(), at the IMU's mount:

        a_imu = a_body + alpha x r + omega x (omega x r)
        f     = R^T (a_imu - g)

    with r the mount offset from the body origin (rotated to the world), and
    a_body and alpha the changes in the world-frame velocities over that
    interval. Because each read spans exactly the time since the last one,
    every velocity change is counted once, however the IMU and motion
    samples interleave: none is dropped or doubled. The IMU frame is assumed
    aligned with the body (true for the car's imu_joint).
    """

    def __init__(self, mount_offset, gravity):
        """mount_offset: IMU position in the body frame (m); gravity: m/s^2, > 0."""
        self._mount = tuple(mount_offset)
        self._g = (0.0, 0.0, -gravity)
        self._latest = None     # newest (stamp, orientation, v_world, w_world)
        self._read_from = None  # the motion sample the last read ended at
        self._force = None      # last reading, held when no motion is new

    def add_motion(self, stamp, orientation, linear_body, angular_body):
        """
        Add one true-motion sample; a sample that isn't newer is ignored.

        stamp in s; orientation (x, y, z, w) of the body in the world;
        linear_body and angular_body: the body's velocities in its own frame.
        """
        if self._latest is not None and stamp <= self._latest[0]:
            return
        self._latest = (stamp, orientation, rotate(orientation, linear_body),
                        rotate(orientation, angular_body))
        if self._read_from is None:
            self._read_from = self._latest

    def read(self):
        """
        Return the specific force (x, y, z) since the last read, or None.

        With no new motion since the last read, the last reading is held.
        None until two motion samples have arrived.
        """
        if self._latest is None or self._latest is self._read_from:
            return self._force
        (t0, _, v0, w0), (t1, q, v1, w1) = self._read_from, self._latest
        self._read_from = self._latest
        dt = t1 - t0
        accel = tuple((a - b) / dt for a, b in zip(v1, v0))
        alpha = tuple((a - b) / dt for a, b in zip(w1, w0))
        r = rotate(q, self._mount)
        tangential = _cross(alpha, r)
        centripetal = _cross(w1, _cross(w1, r))
        a_imu = tuple(a + t + c for a, t, c in zip(accel, tangential, centripetal))
        self._force = rotate_inverse(q, tuple(a - g for a, g in zip(a_imu, self._g)))
        return self._force
