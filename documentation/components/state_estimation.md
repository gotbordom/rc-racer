# state_estimation

**Element ID:** VEH.GNC.SE · **Package:** `state_estimation` · **Status:** Draft

## 1. Purpose

Fuses IMU and wheel odometry into a smooth, continuous estimate of the car's pose and velocity in the `odom` frame, and publishes the `odom` → `base_footprint` transform.

It does **not**:
- give a globally correct position or lat/lon. That is `localization` (`map` frame), see Open Item 1;
- decide where to go or how to drive (PLN, EXE);
- filter or interpret camera data (PER).

## 2. Requirements

| Requirement | How this component meets it |
| --- | --- |
| REQ-GNC-001 | Publishes `/odom` and TF at 50 Hz from a fixed-rate predict step |
| REQ-GNC-002 | Continuous `odom` estimate (no GPS jumps); drift measured against ground truth in sim (TBD-003) |
| REQ-GNC-004 | Does not depend on GNSS (by design, Open Item 1) |
| REQ-GNC-005 | Detects slip by comparing wheel speed with IMU acceleration; inflates wheel-odometry covariance while slipping |
| REQ-GNC-006 | Publishes the full pose and twist covariance |
| REQ-GNC-007 | Motion model chosen by parameter (`motion_model`) via the `RobotFrame` bridge |

## 3. Interfaces

### 3.1 Inputs

| Name | Type | ICD | Required? | Stale after |
| --- | --- | --- | --- | --- |
| `/imu` | `sensor_msgs/msg/Imu` | [ICD-SNS-GNC](ICD-SNS-GNC) SNS-GNC-01 | Yes | 0.1 s |
| `/wheel/odom` (uses `twist.linear.x` only) | `nav_msgs/msg/Odometry` | [ICD-SNS-GNC](ICD-SNS-GNC) SNS-GNC-03 | Yes | 0.2 s |
| Sensor mount transforms | TF | [ICD-SNS-GNC](ICD-SNS-GNC) SNS-GNC-04 | Yes | - |

### 3.2 Outputs

| Name | Type | ICD | Rate | Notes |
| --- | --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | GNC-internal (SE → LOC, PLN) | 50 Hz | `frame_id: odom`, `child_frame_id: base_footprint`, full covariance |
| TF `odom` → `base_footprint` | `tf2_msgs/msg/TFMessage` | [Conventions §2](Architecture-Conventions) | 50 Hz | Only publisher of this edge |
| `/state_estimation/debug` | TBD | none (debug only) | 50 Hz | Innovations, slip flag. Not an interface: nothing may depend on it. **Not implemented yet**; the slip flag is in `/diagnostics` |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | [Conventions §8](Architecture-Conventions) | 1 Hz | See §5 |

### 3.3 Actions / Services offered

| Name | Type | ICD | Behaviour |
| --- | --- | --- | --- |
| `~/reset` | `std_srvs/srv/Trigger` | ICD-EXE-GNC (not started) | Resets the estimate to the origin with initial covariance (between attempts, REQ-SYS-024) |

### 3.4 Parameters

| Name | Type | Default | Meaning |
| --- | --- | --- | --- |
| `publish_rate` | double | 50.0 | Hz, predict + publish rate |
| `publish_tf` | bool | true | Publish `odom` → `base_footprint` |
| `motion_model` | string | `bicycle` | `bicycle` (Ackermann) or `unicycle` (differential) |
| `fusion.method` | string | `ekf` | `ekf`, `ukf` or `complementary` (`SensorFusionMethod`) |
| `robot.wheelbase` | double | 0.384 | m, used by the bicycle model |
| `inputs.imu.timeout` | double | 0.1 | s |
| `inputs.wheel_odom.timeout` | double | 0.2 | s |
| `robot.max_steering_angle` | double | 0.52 | rad, used by the bicycle model's slide check |
| `robot.rear_axle_offset` | double | 0.192 | m, rear axle forward to `base_footprint` (midway between the axles, Conventions §2.1). The bicycle model integrates the rear axle, which has no sideways motion, then moves to `base_footprint` |
| `process_noise.accel` | double | 0.05 (TBD) | (m/s²)²/s, unmodelled forward acceleration while IMU acceleration is the input |
| `process_noise.accel_no_imu` | double | 4.0 (TBD) | (m/s²)²/s, forward acceleration when the IMU is stale |
| `process_noise.yaw_accel` | double | 1.0 (TBD) | (rad/s²)²/s, unmodelled change in yaw rate |
| `slip.threshold` | double | 0.3 (TBD) | m/s. Wheel speed minus predicted speed beyond this, plus 3σ of the predicted speed, counts as slip |
| `slip.yaw_rate_margin` | double | 0.2 (TBD) | rad/s. Yaw rate beyond what the motion model allows at this speed that counts as a slide |
| `slip.variance_scale` | double | 1e4 (TBD) | Factor on the wheel-speed variance while slipping |
| `initial_covariance.pose` / `.speed` / `.yaw_rate` | double | 1e-6 / 1.0 / 0.1 | Covariance diagonal after start and `~/reset` |

## 4. Behaviour

- **State:** 2D: `x, y, yaw` in `odom`, plus forward speed `v` and yaw rate `ω` (Open Item 3).
- **Lifecycle:** `configure` validates parameters and builds the logic object. `activate` starts subscriptions, timer and publishers, with the estimate at the origin. `deactivate` stops publishing. `cleanup` drops state.
- **Predict** on a fixed-rate timer using the motion model, with the latest IMU forward acceleration as the speed input (not reused once the IMU is stale).
- **Update** on each `/imu` (yaw rate) and `/wheel/odom` (forward speed).
- **IMU orientation:** only roll and pitch are used, to remove gravity from the forward acceleration. The absolute yaw is world-frame data and is ignored (ISSUE-002).
- **Motion model:** the bicycle model integrates the **rear axle**, the only point of an Ackermann car that does not move sideways, and moves the result to `base_footprint`; on a turn `base_footprint` slides sideways at ω × `robot.rear_axle_offset`. Leaving this out cost 0.37-0.45 m over the L2 loop. Integration follows the measured yaw rate even beyond the steering limit, so a slide cannot corrupt the heading. The model's limit (`RobotFrame::constrain`) is used only to detect slides.
- **Slip:** two checks. (1) *Wheel slip*: wheel speed and the IMU-predicted speed disagree by more than `slip.threshold` plus 3σ of the predicted speed. (2) *Slide*: the yaw rate exceeds what the motion model allows at the current speed by `slip.yaw_rate_margin`; this never triggers for the unicycle. While either is detected, the wheel-odometry variance is multiplied by `slip.variance_scale`, until they agree again.
- **Degraded modes:**

| Missing input | Behaviour | Diagnostics |
| --- | --- | --- |
| `/wheel/odom` | Continue on IMU; speed variance grows | WARN |
| `/imu` | Continue on wheel speed only; yaw variance grows quickly | ERROR |
| both | Keep predicting with growing covariance; do not stop publishing | ERROR |
| NaN/Inf in an input | Sample rejected and counted | WARN |
| NaN/Inf in estimate | Reset to last good estimate with inflated covariance | ERROR |

## 5. Health

`/diagnostics` status `state_estimation` (1 Hz, `diagnostic_updater`). The worst condition sets the level, ranked ERROR > STALE > WARN > OK, and every condition is listed in the message:

| Level | When |
| --- | --- |
| STALE | Inactive, or an input has not arrived yet (ICD-SNS-GNC §5) |
| ERROR | Stale IMU; no TF from `base_footprint` to the IMU frame; numerical reset since the last report |
| WARN | Stale wheel odometry; slip detected; rejected samples since the last report |
| OK | Otherwise |

Key values: `imu_age`, `wheel_odom_age`, `slip_detected`, `covariance_trace`, `numerical_resets`, `rejected_measurements`, `motion_model`, `fusion_method`. A WARN on position covariance above a limit waits for that limit (TBD, from the L2 test).

**IMU mount:** the node looks up `base_footprint` → IMU frame once from TF (static, from the URDF) and rotates rates, specific force and covariances into `base_footprint`. Until the transform exists, IMU samples are ignored.

## 6. Internal Design

From [Software Design](Software-Design) (State Estimation class diagram), adjusted to the logic/adapter split ([SDP §2](Software-Development-Plan)):

| Class | ROS? | Role |
| --- | --- | --- |
| `RobotFrame` (+ `AckermannSteering`, `TwoWheelDifferentialSteering`) | No | Motion model: `get_next_pose_estimate(state, inputs, dt)` (bicycle / unicycle) |
| `StateEstimationLogic` | No | Owns the filter (`SensorFusionMethod`) and a `RobotFrame`; `predict(dt)`, `update_imu()`, `update_wheel_speed()`, slip detection |
| `RobotState` | No | Pose, twist, covariance |
| `StateEstimationNode` (`RobotNode`) | Yes | Lifecycle node: parameters, subscriptions, timer, publishers, TF; converts messages ↔ plain structs and calls the logic |

Keeping `RobotFrame` free of ROS (rather than inside `RobotNode` as in the original diagram) means the motion models can be unit-tested and reused by `planning_control`.

## 7. Verification

| Level | Test | Checks |
| --- | --- | --- |
| L4 unit | `test_robot_frame` | Bicycle/unicycle prediction: straight line, constant-radius turn, steering limit |
| L4 unit | `test_state_estimation_logic` | Converges on consistent inputs; covariance grows without updates; slip inflates wheel variance; NaN recovery |
| L3 component | `test_state_estimation_node` | Stub inputs per ICD-SNS-GNC → `/odom` at 50 Hz with correct frames and non-zero covariance; TF published once; STALE/WARN/ERROR on missing inputs; `~/reset` |
| L2 subsystem | `test_gnc_state_estimation_sim` | Drive the practice course in sim; `/odom` vs ground truth within REQ-GNC-002 drift (TBD-003) |

## 8. Open Items

| # | Question | Options | Recommendation |
| --- | --- | --- | --- |
| 1 | Where is GPS fused? (ISSUE-002) | (a) in `state_estimation`, as the class diagram shows; (b) `state_estimation` fuses IMU + wheel only in `odom`, and `localization` adds GPS in `map` | ✅ **Decided 2026-11-16: (b).** REP-105 split: `state_estimation` tracks the vehicle's own motion; where it is in the world is `localization`'s job. `odom` stays continuous for control. There is no onboard GNSS for now (ISSUE-001) |
| 2 | Write our own filter or use `robot_localization`? | own / library | **Own** (the point of the project, and the design calls for pluggable fusion methods), but run `robot_localization` in the L2 sim test as a baseline to compare against |
| 3 | 2D or 3D state? | 2D now, 3D for hills | **2D** for Phase 2; revisit with terrain worlds (REQ-SYS-009) |
| 4 | Simulator deviations DEV-01 (frame IDs) and DEV-02 (zero covariances) | - | ✅ Both fixed ([ICD-VEH-TST](ICD-VEH-TST)). Use `/sim/truth/*` and ground-truth pose in the L2 test to measure estimate error |
