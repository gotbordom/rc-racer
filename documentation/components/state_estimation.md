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
| `/odom` | `nav_msgs/msg/Odometry` | GNC-internal (SE → LOC, PLN) | 50 Hz | `frame_id: odom`, `child_frame_id: base_footprint`, full covariance. Pose z, roll, pitch variance is **0**: `base_footprint` is on the ground plane by definition (REP-120), and RViz draws a 2D ellipse only then. Velocities not estimated (lateral, vertical, roll/pitch rates) have variance 1e6 |
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
| `inputs.wheel_odom.scale_stddev` | double | 0.01 (TBD) | Stddev of the encoder's constant scale error k (calibration, tyre wear). Added to the published covariance, never estimated (§4) |
| `robot.max_steering_angle` | double | 0.52 | rad, used by the bicycle model's slide check |
| `robot.rear_axle_offset` | double | 0.192 | m, rear axle forward to `base_footprint` (midway between the axles, Conventions §2.1). The bicycle model integrates the rear axle, which has no sideways motion, then moves to `base_footprint` |
| `process_noise.accel` | double | 0.003 (TBD) | (m/s²)²/s, unmodelled forward acceleration while IMU acceleration is the input (~0.05 m/s of speed σ after 1 s on the IMU alone) |
| `process_noise.accel_no_imu` | double | 4.0 (TBD) | (m/s²)²/s, forward acceleration when the IMU is stale |
| `process_noise.yaw_accel` | double | 1.0 (TBD) | (rad/s²)²/s, unmodelled change in yaw rate |
| `process_noise.distance` | double | 1e-3 (TBD) | m²/m, along-track position noise per metre driven: wheel scale error that varies (tyre scrub in turns, 0.3-0.8 % in sim) |
| `slip.threshold` | double | 0.3 (TBD) | m/s. Wheel speed minus predicted speed beyond this, plus 3σ of the predicted speed, counts as slip (in both the one-step and the windowed check) |
| `slip.window` | double | 0.5 (TBD) | s. Window for the second slip check, and twice the time the wheel must agree before slip ends |
| `slip.max_duration` | double | 2.0 (TBD) | s. Longest IMU-only stretch while slipping; then the wheel is trusted again |
| `stationary.speed` / `.yaw_rate` / `.accel` | double | 0.05 / 0.05 / 0.3 (TBD) | Wheel below m/s, with the IMU's yaw rate (rad/s) and forward acceleration (m/s²) below these: stopped, ends slip at once |
| `slip.yaw_rate_margin` | double | 0.2 (TBD) | rad/s. Yaw rate beyond what the motion model allows at this speed that counts as a slide |
| `slip.variance_scale` | double | 1e6 (TBD) | Factor on the wheel-speed variance while slipping: in effect, the wheel is not fused (1e4 still dragged the estimate after a crash) |
| `initial_covariance.pose` / `.speed` / `.yaw_rate` | double | 1e-6 / 1.0 / 0.1 | Covariance diagonal after start and `~/reset` |

## 4. Behaviour

- **State:** 2D: `x, y, yaw` in `odom`, plus forward speed `v` and yaw rate `ω` (Open Item 3).
- **Lifecycle:** `configure` validates parameters and builds the logic object. `activate` starts subscriptions, timer and publishers, with the estimate at the origin. `deactivate` stops publishing. `cleanup` drops state.
- **Predict** on a fixed-rate timer using the motion model, with the latest IMU forward acceleration as the speed input (not reused once the IMU is stale).
- **Update** on each `/imu` (yaw rate) and `/wheel/odom` (forward speed).
- **IMU orientation:** only roll and pitch are used, to remove gravity from the forward acceleration. The absolute yaw is world-frame data and is ignored (ISSUE-002). Pitch noise is carried into the acceleration variance ((g·σ_pitch)², ~0.2 m/s² for 0.02 rad).
- **IMU lever arm:** in a turn an accelerometer ahead of the rear axle feels −ω²·l on its forward axis (l = `robot.rear_axle_offset` + the IMU's forward offset from TF); it is added back, as it is not acceleration.
- **Wheel encoder scale error:** one encoder on the rear axle reads (1 + k)·v with k unknown. With no absolute reference, k is **unobservable**: estimating it, even as a Schmidt consider state, let the EKF invent information and drift the speed and position (3.2 m after 100 m of unbiased inputs). So k is not in the state. Its constant part is added to the **published** covariance only: dead reckoning scales the path about the `odom` origin by (1 + k), so position gains `scale_stddev`²·p·pᵀ and speed `scale_stddev`²·v². Its varying part is `process_noise.distance`. `localization` can estimate k later from landmark fixes.
- **Motion model:** the bicycle model integrates the **rear axle**, the only point of an Ackermann car that does not move sideways, and moves the result to `base_footprint`; on a turn `base_footprint` slides sideways at ω × `robot.rear_axle_offset`. Leaving this out cost 0.37-0.45 m over the L2 loop. Integration follows the measured yaw rate even beyond the steering limit, so a slide cannot corrupt the heading. The model's limit (`RobotFrame::constrain`) is used only to detect slides.
- **Slip** (REQ-GNC-005). Slip starts on any of three checks:
  1. *Step*: the wheel speed and the speed predicted one step ahead disagree by more than `slip.threshold` + 3σ.
  2. *Window*: the wheel speed and the estimate from `slip.window` ago, carried forward by the IMU alone, disagree by more than `slip.threshold` + 3σ of that reference. This catches slip that builds up over several steps, such as wheels spinning down for 0.5 s after a crash. The step check misses it, because every update re-anchors the estimate to the wheel. When this check fires, the speed is reset to the IMU reference.
  3. *Slide*: the yaw rate exceeds what the motion model allows at the current speed by `slip.yaw_rate_margin`. This never triggers for the unicycle.

  While slipping, the wheel variance is multiplied by `slip.variance_scale`, so the speed is IMU-only. **Safety nets**, so the filter never stays on the IMU alone: slip also ends at once when the car is clearly stopped (wheel below `stationary.speed`, IMU quiet), or after `slip.max_duration`. Then the speed is re-anchored to the wheel and the window history is cleared. These matter because an IMU can miss or saturate on a crash impulse (DEV-07 in sim): the IMU-only speed once stayed at 2.5 m/s with the car stopped against the bench. Slip ends (hysteresis) when the wheel has agreed with that speed within `slip.threshold`/2 + 3σ, with no slide, for `slip.window`/2 without a break. A wheel still spinning down can't end it on the way. Found by a manual sim test, turbo into the bench: the estimate had read 2 m/s with the car stopped.
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
| WARN | Stale wheel odometry; slip now or **any slip since the last report** (latched: `slip_events` counts them, so a short slip between two 1 Hz reports is still reported); rejected samples since the last report |
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
| L2 subsystem | `test_gnc_state_estimation_sim` | Drive a ~100 m loop of the practice course in sim (pure pursuit on ground truth, `/sim/truth/poses`); `/odom` rate ≥ 45 Hz, no jumps (≤ 0.1 m between samples), drift ≤ TBD-003, final heading ≤ 0.1 rad, ≥ 95 % of samples inside the 3σ ellipse, no ERROR diagnostics. Prints a results table and saves a per-sample trace |

**L2 results** (2026-11-17, practice course loop, 98.7 m in 100 s): final error 0.13 m (**0.13 % drift**), max 0.20 m, heading 0.2°, `/odom` 50.0 Hz, max step 0.022 m, 100 % inside 3σ (reported 1σ 0.34 m). The test found four model errors, now fixed and covered by unit tests: the bicycle model ignored `base_footprint`'s sideways slide (0.37-0.45 m), pitch noise was missing from the acceleration variance, the IMU lever arm read as braking in turns, and the wheel scale error was treated as white noise (overconfident: 17 % inside 3σ).

**With `tire_mu` 0.5** (2026-11-21, the new sim default; the result above was at 1.0): 98.8 m in 100 s, final error 0.37 m (**0.37 % drift**), max 0.38 m, heading 0.04°, estimated distance +0.51 % (the tyres creep under drive and in turns, so the wheel over-reads), 100 % inside 3σ (reported 1σ 0.34 m), no non-OK diagnostics.

## 8. Open Items

| # | Question | Options | Recommendation |
| --- | --- | --- | --- |
| 1 | Where is GPS fused? (ISSUE-002) | (a) in `state_estimation`, as the class diagram shows; (b) `state_estimation` fuses IMU + wheel only in `odom`, and `localization` adds GPS in `map` | ✅ **Decided 2026-11-16: (b).** REP-105 split: `state_estimation` tracks the vehicle's own motion; where it is in the world is `localization`'s job. `odom` stays continuous for control. There is no onboard GNSS for now (ISSUE-001) |
| 2 | Write our own filter or use `robot_localization`? | own / library | **Own** (the point of the project, and the design calls for pluggable fusion methods), but run `robot_localization` in the L2 sim test as a baseline to compare against |
| 3 | 2D or 3D state? | 2D now, 3D for hills | **2D** for Phase 2; revisit with terrain worlds (REQ-SYS-009) |
| 4 | Simulator deviations DEV-01 (frame IDs) and DEV-02 (zero covariances) | - | ✅ Both fixed ([ICD-VEH-TST](ICD-VEH-TST)). Use `/sim/truth/*` and ground-truth pose in the L2 test to measure estimate error |
