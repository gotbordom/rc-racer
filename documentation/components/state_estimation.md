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
- **Wheel encoder scale error:** one encoder on the rear axle reads (1 + k)·v with k unknown. With no absolute reference, k is **unobservable**: estimating it, even as a Schmidt consider state, let the EKF invent information and drift the speed and position (3.2 m after 100 m of unbiased inputs). So k is not in the state. Its constant part is added to the **published** covariance only: dead reckoning scales the path about the `odom` origin by (1 + k), so position gains `scale_stddev`²·p·pᵀ and speed `scale_stddev`²·v². Its varying part is `process_noise.distance`. `localization` can estimate k later from landmark fixes. **Decided 2026-11-22 (owner):** keep k out of the state. Keep it small by **calibrating before a competition**: a rollout over a measured distance (e.g. 10 m tape) on the competition surface gives the effective rolling radius under load to about ±0.2 %, better than measuring the unloaded diameter (±0.7 % for ±0.5 mm). The calibrated radius goes in the wheel encoder driver's config, which turns ticks into speed (ICD-SNS-GNC); `scale_stddev` here is then set to the calibration accuracy (0.01 until then).
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

### 6.1 Planned: sideways velocity in the state (VAL-018, Open Item 7)

**Why.** Today the state is `[x, y, ψ, v, ω]` and the motion model assumes the rear axle never moves sideways. That holds while the tyres grip. In a slide (VAL-004/018: full lock on turbo at μ 0.5) the car moves sideways at 0.4–0.8 m/s, a 10–17° slip angle, and two things go wrong:
- The model can't represent where the car goes: the position error is metres.
- While slipping, the speed comes from the IMU alone. In the body frame, the forward velocity changes as v̇_x = a_x + ω·v_y. Integrating a_x alone drops the ω·v_y term, which is about 1–1.5 m/s² in these slides, so the IMU-only speed runs away (4.4 m/s against a true 2.6–2.9).

Detecting the slide (understeer check, 2.4) and distrusting the wheel can't fix either. Both are missing physics.

**The change.** State `[x, y, ψ, v_x, v_y, ω]`, where v_y is the sideways velocity of `base_footprint` in the body frame.
- **Propagation (pure kinematics, no tyre model):**
  - ẋ = v_x cos ψ − v_y sin ψ
  - ẏ = v_x sin ψ + v_y cos ψ
  - v̇_x = a_x + ω·v_y
  - v̇_y = a_y − ω·v_x
  - ω̇ = noise

  Here a_x and a_y are the IMU's specific forces with gravity and lever arm removed, as a_x is today. The ω·v_y term is what stops the IMU-only speed running away.
- **Measurements:**
  - wheel speed → v_x (unchanged);
  - gyro → ω (unchanged);
  - a **no-slip pseudo-measurement** at the rear axle: v_y − ω·`rear_axle_offset` = 0.
- **The pseudo-measurement's variance is what the slip logic controls.** While gripping it's small, and the filter reproduces today's bicycle model (VAL-001/002/003/016 and the L2 loop must not change). When the understeer check or the yaw-rate check flags a slide, it's relaxed, so v_y follows the IMU and the covariance grows sideways honestly. When grip returns, it tightens again.
- **The unicycle** uses the same structure with the constraint at its centre (offset 0).

**What changes in the code:**
- the EKF state and Jacobians (5 → 6), `RobotState`, `RobotFrame`'s interface (it gives the constraint point instead of integrating a no-slip arc), and `estimate()`'s scale term;
- `/odom`'s `twist.linear.y` becomes non-zero (ICD-GNC-*: allowed by `nav_msgs/Odometry`; check what consumers assume);
- every logic test that inspects the state indices.

**Is it the right fix? Yes, for the cause of VAL-018.** It's the kinematics the car actually obeys, it needs no tyre or friction model (μ is unknown on a real course), and it turns the IMU's lateral axis, unused today, into information. It also makes the understeer check a natural switch rather than a bolt-on.

Limits to expect:
- **v_y drifts while the constraint is relaxed,** because it's integrated from a_y with nothing measuring it directly. That's fine for slides lasting seconds, but a mounting bias in a_y would grow the drift. Calibrate the mount (as for `rear_axle_offset`) before trusting long slides.
- **Errors rotate between v_x and v_y in a turn** rather than shrinking. The covariance must show that, and VAL-018's 3σ check is the test.
- **The pinned car (VAL-019) is a different problem:** that's impact detection, not sideways motion.

**Rejected alternatives:**
- *Speed from the turn* (v_x = a_y/ω while turning). It's the steady-state special case of the above (v̇_y = 0). It's simpler but wrong during turn-in and turn-out, when v̇_y ≠ 0, and it gives nothing about sideways position.
- *Cap and blend* (keep IMU-only, cap at the wheel speed). It only bounds the error.

**Plan:** feature 2.5 `lateral-velocity` (owner, 2026-11-24), after 2.4 (which keeps detection, early distrust, braking and impact detection). Steps:
1. state and Jacobians with the constraint always tight, proving nothing changes (all scenarios and the L2 loop as before);
2. relax the constraint on understeer or yaw-rate slide;
3. tune and test against VAL-004/018, then remove VAL-018's expected-fail mark.

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
| 5 | **Braking slip is detected late or not at all.** Found 2026-11-21 in the sim at `tire_mu` 0.5: braking at the drive's 3 m/s² with grip for about 1.7 m/s², the wheel slowed faster than the car, which slid on up to 0.6 m/s faster for about 0.8 s. The gap stayed under the windowed check's 3σ, so the estimate followed the wheel (cost ~3 cm from 3 m/s). Launch slip in the same run was caught. **Scenario VAL-006, 2026-11-23** (hard stop from 3 m/s, health timed at 10 Hz): one run detected the slide ~1.1 s after braking began, near the end of the slide; the next did not detect it at all. Error 0.09-0.12 m, inside 3σ both times. **Decided 2026-11-23 (owner):** braking slip must be detected within 0.5 s, as launch slip is. VAL-006 checks that and stays expected-fail until it does | Make the check more sensitive to a wheel decelerating faster than the IMU says the body does (e.g. a one-sided test, or a tighter threshold while braking); fix together with Open Item 6 | ✅ **Fixed 2026-11-24 (2.4).** A wheel behind its window reference while the IMU shows braking (`slip.braking_accel`) uses a tighter threshold, `slip.braking_threshold` (0.15 m/s on top of 3σ, against 0.3): a normal hard stop, wheel and car slowing together, gives no gap, so this case can be sensitive. **Requirement restated (owner, 2026-11-24):** flagged within 0.3 s of the body getting 0.15 m/s ahead of its wheels, timed from ground truth, because the slide's size varies between runs (2.5-2.8 m/s² against the wheel's 3). VAL-006: flagged 0.02 s before the slide reached 0.15 m/s, 100 % inside 3σ |
| 6 | **Covariance overconfident until a launch slip is detected** (VAL-017, #9). Found 2026-11-23 by the 2.3 scenario tests at `tire_mu` 0.5: in a full-throttle launch the windowed slip check fires after ~0.45-0.56 s. Until then the filter trusts the spinning wheel: the speed runs ahead (1.2 m/s against a true 0.75) and ~0.1 m of along-track error builds while the reported sigma is ~0.02 m (about 5 sigma off). ~15 % of the first second's samples are outside 3 sigma. Detection and recovery themselves work (VAL-005) | (a) faster detection: a per-sample check of wheel against IMU acceleration, alongside the windowed one; (b) grow the wheel-speed variance with how far the wheel's acceleration exceeds the IMU's, so the covariance covers the error as it builds; (c) accept a transient allowance in REQ-GNC-006 | ✅ **Fixed 2026-11-24 (2.4).** On detection the windowed check corrects retroactively: the history is rewritten as the IMU path from the window start, the position moves back by what the wheel added (spread over `slip.correction_time`, 0.5 s, so odom stays continuous), and the covariance grows by the correction's uncertainty (`slip.correction_uncertainty`). Before detection, the wheel's variance grows with how far it out-accelerates the IMU (`slip.accel_*`), so a developing spin is distrusted gradually. VAL-017: 100 % inside 3σ in its first second (96.1 % with the correction alone, 94.2 % on another run) |
| 7 | **Estimate and covariance not honest in a sideways slide** (VAL-018, #11). Found 2026-11-23 by the VAL-004 scenario (full lock on turbo, `tire_mu` 0.5): the car understeers, sliding at a 10-17° slip angle, with a yaw rate under the steering limit, so the yaw-rate check cannot fire. The bicycle model has no sideways velocity. The windowed wheel check flags the slide 2-3 s late, and once slip is declared the IMU-only speed runs away (4.4 m/s against a true 2.6-2.9) until `slip.max_duration` ends it. Result: 1.1-1.76 m error at σ ≤ 0.24 m (24-56 % inside 3σ), and a ~7 cm position step when slip ends. Heading stays within 1°, and it never gets stuck (VAL-004) | (a) model the slide: sideways velocity in the state, IMU lateral acceleration as a measurement; (b) detect understeer: IMU yaw rate against what the steering command and wheel speed predict, and grow the covariance sideways while sliding; (c) a safe fallback: don't integrate IMU acceleration without bound while slipping, cap or blend it with the wheel, and re-enter smoothly; (d) accept and document a lateral-acceleration limit for planning | **Decided 2026-11-24 (owner):** fix the cause, option (a): sideways velocity in the state, with a no-slip pseudo-measurement the slip logic relaxes in a slide (§6.1). Options (b) understeer detection and the early wheel distrust are in 2.4 and stay; (c) and (d) rejected (they bound or document the error instead of removing it). Its own feature, 2.5 `lateral-velocity` (owner, 2026-11-24) |
| 8 | **Estimate drives on when the car is pinned against an obstacle** (VAL-019, #14). Found 2026-11-24 by the VAL-007 scenario: pushing against the bench at 1 m/s for ~4 s, the driven wheels spin at a steady speed and the IMU shows no acceleration, which looks like cruising. No slip is flagged, and the estimate ends 4.30-4.36 m past the bench at σ ≈ 0.5 m. The impact is the only sign, and the sim IMU misses it (DEV-07, ICD-VEH-TST) | Two parts, in order: (1) the sim sees impacts: the sensor adapter derives acceleration from truth velocity (DEV-07); (2) the estimator detects the impact (deceleration far beyond braking, ≤ 3 m/s²), then keeps distrusting the wheel until the IMU agrees the car moves, and grows the covariance meanwhile | ✅ **Fixed 2026-11-24 (2.4).** (1) The sim IMU now sees impacts (DEV-07). (2) Impact: a deceleration beyond `slip.impact_accel` (8 m/s²). After one that took at least `slip.pinned_fraction` (0.3) of the speed, with the wheel still disagreeing and the IMU quiet, the car is **pinned**: speed held at 0, the `slip.max_duration` release suspended, and the distance driven since the impact taken back (spread, so odom stays continuous). It ends when the IMU shows `slip.pinned_speed` picked up, the wheels stop, or `slip.pinned_max_duration` passes. The IMU sees 46-87 % of a crunch's speed loss, so the share, not a fixed speed, decides. VAL-019: 0.10-0.26 m final error on three runs (was 4.3 m) |
