# ICD-VEH-TST: Vehicle to Test & Simulation

**Status:** Draft · **Level:** L1 · **Owner:** L0 (System)
**Parties:** VEH (Vehicle segment software) ↔ TST.SIM (`rc_racer_simulation`), TST.HAR (tests)

## 1. Purpose

The simulator stands in for the **environment** and the vehicle's **sensing and actuation hardware**, so vehicle software runs unchanged against it (REQ-TST-001). Instead of redefining those topics, this ICD states which vehicle-internal ICDs the simulator implements, and adds the items that exist only in simulation.

## 2. Requirements

REQ-SYS-040, REQ-TST-001 … REQ-TST-004.

## 3. Substitution

| Vehicle ICD | Simulator implements | In place of |
| --- | --- | --- |
| [ICD-SNS-GNC](ICD-SNS-GNC) | Provider side: `/imu`, `/navsat`, `/wheel/odom`, published by the **sensor adapter** (noise + covariance from `config/sensor_noise.yaml`) | `imu_driver`, `gps_driver`, `encoder_driver` |
| ICD-SNS-PER (not started) | Provider side: `/camera`, `/camera_info` | `camera_driver` |
| [ICD-GNC-ACT](ICD-GNC-ACT) | Consumer side: `/cmd_vel` → Ackermann drive | `motor_controller` + ESC + servo |

Vehicle software must not use anything from §4 below, so that the same code runs on hardware.

## 4. Simulation-only Items

| ID | Name | Kind | Type | Direction | Used by |
| --- | --- | --- | --- | --- | --- |
| VEH-TST-01 | `/clock` | topic | `rosgraph_msgs/msg/Clock` | TST → all | Every node (`use_sim_time:=true`) |
| VEH-TST-02 | Ground-truth pose | CLI (`ign model -m rc_racer -p`) | text | TST → HAR | Tests only |
| VEH-TST-03 | `/joint_states` | topic | `sensor_msgs/msg/JointState` | TST → `robot_state_publisher` | Wheel/steering visualisation |
| VEH-TST-04 | Course file | file | `courses/<name>.yaml` | GND.MPL → TST | World generation (REQ-TST-002) |
| VEH-TST-05 | Launch arguments | launch | `world`, `headless`, `use_sim_time` | HAR → TST | Selecting world / GUI |
| VEH-TST-06 | `/sim/truth/imu`, `/sim/truth/navsat`, `/sim/truth/wheel_odom` | topic | as the ICD topics, noise-free | TST → HAR | Tests only: truth to compare noisy data or estimates against |

## 5. Known Deviations

Places where the simulator does not yet meet the ICDs it implements. Each is a task to fix before the consumer depends on it.

| # | ICD item | Deviation | Impact | Fix |
| --- | --- | --- | --- | --- |
| ~~DEV-01~~ | SNS-GNC-01, -02, camera | ✅ **Fixed 2026-11-16.** `header.frame_id` was Gazebo's sensor name (`rc_racer/base_footprint/imu_sensor`, `…/navsat_sensor`, `…/camera_sensor`), which is not in the TF tree | - | `<ignition_frame_id>` on each sensor in `sensors.urdf.xacro`; now `imu_link`, `gps_link`, `camera_optical_link`. Checked by `test_sensor_frame_ids` |
| ~~DEV-02~~ | SNS-GNC-01, -02, -03 | ✅ **Fixed 2026-11-16.** All covariances were **0**, and the configured noise was unrealistically small | - | Gazebo now publishes truth on `/sim/truth/*`; the sensor adapter adds realistic noise and sets covariances from the same `config/sensor_noise.yaml`. Checked by `TestSensorNoise` (covariances match config; measured noise matches covariance) |
| DEV-03 | GNC-ACT-01 | No deadman: the Ackermann plugin keeps the last command if commands stop (to verify) | A crashed planner leaves the car driving in sim | Add a timeout or command relay; test it |
| DEV-04 | GNC-ACT-01 | Steering responds with a ~1 s time constant ([SDP §4.2](Software-Development-Plan)) | Path following tuned in sim will be wrong on hardware | Phase 4 (accepted until then) |
| DEV-05 | SNS-GNC-03 | `/wheel/odom` also provides yaw rate and pose (more than hardware can) | None if consumers follow ICD-SNS-GNC §4 | Allowed; consumers must use only `linear.x`. Their variance is reported as 1e6 (unmeasured) |
| DEV-06 | (all) | Gazebo occasionally hangs on shutdown: ignores SIGINT and SIGTERM and is SIGKILLed (seen 1 in ~8 local runs on 2026-11-16) | Intermittent `test_exit_codes` failure; no effect on sensor data | Investigate (backtrace of the hung process). The exit-code check stays strict |

## 6. Verification

| Test | Checks |
| --- | --- |
| `test_simulation_smoke` | Sensor topics publish plausible data; drive tracks commands (ground truth); clean shutdown |
| `test_course_world` | Course world matches its course file; GPS origin matches course start |
| `test_generate_course_world` | Course file validation and world generation |

## 7. Change Log

| Date | Change | Agreed by |
| --- | --- | --- |
| 2026-11-16 | Initial draft; deviations DEV-01…05 recorded | |
| 2026-11-16 | DEV-01 fixed (sensor frame IDs) | |
| 2026-11-16 | DEV-02 fixed (sensor adapter: noise + covariance); VEH-TST-06 truth topics added; DEV-06 recorded | |
