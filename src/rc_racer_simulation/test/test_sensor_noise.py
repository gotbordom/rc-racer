# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""Unit tests for rc_racer_simulation.sensor_noise (DEV-02)."""

import math
import os
import random
import statistics
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rc_racer_simulation.sensor_noise import (  # noqa: E402
    GaussMarkov,
    ImuNoise,
    NavSatNoise,
    WheelOdomNoise,
    imu_covariances,
    navsat_covariance,
    offset_geodetic,
    perturb_orientation,
    quaternion_from_rpy,
    validate,
    wheel_odom_covariances,
)

IMU = ImuNoise(0.005, 0.05, 0.02, 0.05)
NAVSAT = NavSatNoise(2.0, 4.0, 30.0)
WHEEL = WheelOdomNoise(0.02, 1.0e6)


def _diag(cov, n):
    return [cov[i * (n + 1)] for i in range(n)]


def _off_diag(cov, n):
    return [v for i, v in enumerate(cov) if i % (n + 1) != 0]


class TestCovariances:
    """Covariances are the squares of the configured stddevs (REQ-GNC-006)."""

    def test_imu(self):
        orientation, gyro, accel = imu_covariances(IMU)
        assert _diag(orientation, 3) == pytest.approx([0.02 ** 2, 0.02 ** 2, 0.05 ** 2])
        assert _diag(gyro, 3) == pytest.approx([0.005 ** 2] * 3)
        assert _diag(accel, 3) == pytest.approx([0.05 ** 2] * 3)
        for cov in (orientation, gyro, accel):
            assert len(cov) == 9
            assert not any(_off_diag(cov, 3))

    def test_navsat(self):
        cov = navsat_covariance(NAVSAT)
        assert _diag(cov, 3) == pytest.approx([4.0, 4.0, 16.0])

    def test_wheel_odom_only_linear_x_is_measured(self):
        pose, twist = wheel_odom_covariances(WHEEL)
        assert len(pose) == len(twist) == 36
        assert _diag(twist, 6)[0] == pytest.approx(0.02 ** 2)
        assert _diag(twist, 6)[1:] == [1.0e6] * 5
        assert _diag(pose, 6) == [1.0e6] * 6

    def test_no_zero_variances(self):
        """DEV-02: no measured field may report zero variance."""
        for cov in (*imu_covariances(IMU), navsat_covariance(NAVSAT)):
            assert all(v > 0 for v in _diag(cov, 3))
        for cov in wheel_odom_covariances(WHEEL):
            assert all(v > 0 for v in _diag(cov, 6))


class TestValidate:
    """Configuration validation."""

    def test_negative_rejected(self):
        with pytest.raises(ValueError, match='horizontal_stddev'):
            validate(NavSatNoise(-1.0, 4.0, 30.0))

    def test_valid_returned(self):
        assert validate(IMU) is IMU


class TestGaussMarkov:
    """Gauss-Markov GNSS error model."""

    @pytest.mark.parametrize('correlation_time', [0.0, 5.0])
    def test_stationary_stddev(self, correlation_time):
        """Long-run stddev equals the configured stddev, white or correlated."""
        process = GaussMarkov(2.0, correlation_time, random.Random(1))
        samples = [process.step(0.1) for _ in range(200_000)]
        assert statistics.pstdev(samples) == pytest.approx(2.0, rel=0.05)
        assert statistics.fmean(samples) == pytest.approx(0.0, abs=0.2)

    def test_correlated_changes_slowly(self):
        """With a 30 s correlation time, consecutive 0.1 s samples barely change."""
        process = GaussMarkov(2.0, 30.0, random.Random(2))
        steps = []
        previous = process.value
        for _ in range(1000):
            value = process.step(0.1)
            steps.append(abs(value - previous))
            previous = value
        assert statistics.fmean(steps) < 0.2

    def test_white_noise_is_uncorrelated(self):
        process = GaussMarkov(2.0, 0.0, random.Random(3))
        samples = [process.step(0.1) for _ in range(20_000)]
        lag1 = statistics.correlation(samples[:-1], samples[1:])
        assert abs(lag1) < 0.05

    def test_zero_stddev_is_silent(self):
        process = GaussMarkov(0.0, 30.0, random.Random(4))
        assert all(process.step(0.1) == 0.0 for _ in range(100))


class TestGeodetic:
    """Metre offsets to lat/lon."""

    def test_north_offset(self):
        lat, lon = offset_geodetic(37.6602, -121.8890, 1.0, 0.0)
        assert lon == pytest.approx(-121.8890, abs=1e-12)
        assert (lat - 37.6602) * 111_000 == pytest.approx(1.0, rel=0.01)

    def test_east_offset(self):
        lat, lon = offset_geodetic(37.6602, -121.8890, 0.0, 1.0)
        assert lat == pytest.approx(37.6602, abs=1e-12)
        metres = (lon + 121.8890) * 111_000 * math.cos(math.radians(37.6602))
        assert metres == pytest.approx(1.0, rel=0.01)


class TestOrientation:
    """Orientation perturbation."""

    def test_zero_perturbation_is_identity(self):
        q = quaternion_from_rpy(0.1, -0.2, 1.0)
        assert perturb_orientation(q, 0.0, 0.0, 0.0) == pytest.approx(q)

    def test_yaw_perturbation_adds_yaw(self):
        q = perturb_orientation((0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.05)
        assert q == pytest.approx(quaternion_from_rpy(0.0, 0.0, 0.05))

    def test_unit_norm(self):
        q = perturb_orientation(quaternion_from_rpy(0.3, 0.2, -2.0), 0.02, -0.02, 0.05)
        assert math.sqrt(sum(c * c for c in q)) == pytest.approx(1.0)
