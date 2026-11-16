# ICD-SNS-GNC: Sensing to GNC

**Status:** Draft · **Level:** L2 · **Owner:** VEH (Vehicle segment)
**Parties:** VEH.SNS (provider: `imu_driver`, `gps_driver`, `encoder_driver`) → VEH.GNC (consumer: `state_estimation`, `localization`)

In simulation, the provider side is implemented by TST.SIM ([ICD-VEH-TST](ICD-VEH-TST)).

## 1. Purpose

Raw, calibrated motion measurements from the car's sensors to GNC, which fuses them into pose and velocity estimates. Sensing publishes measurements only: no filtering, no fusion, no frame changes beyond calibration.

## 2. Requirements

REQ-GNC-001 (estimate at ≥ 50 Hz), REQ-GNC-004 (ride through GNSS outages), REQ-GNC-005 (detect wheel slip), REQ-GNC-006 (covariances), REQ-TST-001 (simulator equivalence).

## 3. Interface Items

| ID | Name | Kind | Type | Direction | Frame | Rate | QoS |
| --- | --- | --- | --- | --- | --- | --- | --- |
| SNS-GNC-01 | `/imu` | topic | `sensor_msgs/msg/Imu` | SNS → GNC | `imu_link` | 100 Hz | Sensor data |
| SNS-GNC-02 | `/navsat` | topic | `sensor_msgs/msg/NavSatFix` | SNS → GNC | `gps_link` | ≥ 5 Hz (sim: 10 Hz) | Sensor data |
| SNS-GNC-03 | `/wheel/odom` | topic | `nav_msgs/msg/Odometry` | SNS → GNC | `odom` → child `base_footprint` | 50 Hz | Sensor data |
| SNS-GNC-04 | sensor mount transforms | TF (static) | `tf2_msgs/msg/TFMessage` | URDF → GNC | `base_link` → sensor frames | static | Transient local |

## 4. Item Details

### SNS-GNC-01 `/imu`

| Field | Content | Required |
| --- | --- | --- |
| `angular_velocity` | Body rates, rad/s, in `imu_link` | Yes |
| `linear_acceleration` | Specific force **including gravity** (≈ +9.81 on Z at rest, level), m/s² | Yes |
| `orientation` | Absolute orientation if the sensor provides it (BNO085 does), else unset | Optional |
| `*_covariance` | Per-axis variance on the diagonal. `orientation_covariance[0] = -1` when orientation is not provided | Yes |

### SNS-GNC-02 `/navsat`

| Field | Content | Required |
| --- | --- | --- |
| `latitude`, `longitude`, `altitude` | WGS84, degrees / metres | Yes |
| `status.status` | `STATUS_NO_FIX` (-1) when there is no fix; consumers **must** ignore position then | Yes |
| `position_covariance`, `position_covariance_type` | ENU variance in m²; type at least `COVARIANCE_TYPE_DIAGONAL_KNOWN` | Yes |

### SNS-GNC-03 `/wheel/odom`

Wheel odometry from the drive motor encoder (128 PPR, final drive 9.384:1, see [Rc-Car-Design](Rc-Car-Design)).

| Field | Content | Required |
| --- | --- | --- |
| `twist.twist.linear.x` | Forward speed of `base_footprint`, m/s, signed | **Yes - the only field consumers may rely on** |
| `twist.twist.angular.z` | Yaw rate, rad/s | Optional |
| `pose.pose` | Dead-reckoned pose in `odom` | Optional |
| `twist.covariance` / `pose.covariance` | Variance of the fields provided; unprovided fields set to a large variance (≥ 1e6) | Yes |

**Why only `linear.x`:** the real car has a single encoder on the drive motor, so it measures average rear-wheel speed. It cannot measure yaw rate or position without assumptions about steering. The simulator's Ackermann plugin provides all fields, but GNC must not depend on anything the hardware cannot provide (REQ-TST-001); it fuses yaw from the IMU instead.

No transform is published for wheel odometry ([Conventions §2](Architecture-Conventions)).

## 5. Timing and Failure

| Item | Stale after | GNC behaviour when stale |
| --- | --- | --- |
| `/imu` | 0.1 s | ERROR; estimate quality drops sharply; Safety decides whether to stop |
| `/navsat` | 2.0 s, or `STATUS_NO_FIX` | WARN; continue on IMU + wheel odometry (REQ-GNC-004) |
| `/wheel/odom` | 0.2 s | WARN; continue on IMU (+ GNSS) with growing velocity uncertainty |

No startup order is assumed: GNC waits for each input and reports STALE until it arrives.

## 6. Verification

| Side | Test | Checks |
| --- | --- | --- |
| Provider (sim) | `test_simulation_smoke` | Topics publish plausible data. **Frames and covariances are not yet checked - see ICD-VEH-TST deviations** |
| Provider (hardware) | per driver, Phase 5 | Same checks as sim, on the bench |
| Consumer | `state_estimation` component tests | Runs from recorded/stub inputs that follow this ICD, including stale and `NO_FIX` cases |
| Both | GNC subsystem test in sim | Estimate vs ground truth |

## 7. Change Log

| Date | Change | Agreed by |
| --- | --- | --- |
| 2026-11-16 | Initial draft | |
