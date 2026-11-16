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
| [ICD-SNS-GNC](ICD-SNS-GNC) | Provider side: `/imu`, `/navsat`, `/wheel/odom` | `imu_driver`, `gps_driver`, `encoder_driver` |
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

## 5. Known Deviations

Places where the simulator does not yet meet the ICDs it implements. Each is a task to fix before the consumer depends on it.

| # | ICD item | Deviation | Impact | Fix |
| --- | --- | --- | --- | --- |
| DEV-01 | SNS-GNC-01, -02 | `header.frame_id` is Gazebo's sensor name (`rc_racer/base_footprint/imu_sensor`, `…/navsat_sensor`), not `imu_link` / `gps_link`. No TF frame exists with those names. `/camera` likely the same (to verify) | GNC cannot transform measurements into the body frame | Set the sensor frame IDs in `sensors.urdf.xacro` (or re-stamp in a small relay), and check frame IDs in the smoke test. **Before `state_estimation` uses frames** |
| DEV-02 | SNS-GNC-01, -02, -03 | All covariances are **0** (IMU, GPS, wheel odometry) | Filters treat measurements as perfect | Set realistic noise in the sensor models and matching covariances; check in the smoke test. **Before `state_estimation`** |
| DEV-03 | GNC-ACT-01 | No deadman: the Ackermann plugin keeps the last command if commands stop (to verify) | A crashed planner leaves the car driving in sim | Add a timeout or command relay; test it |
| DEV-04 | GNC-ACT-01 | Steering responds with a ~1 s time constant ([SDP §4.2](Software-Development-Plan)) | Path following tuned in sim will be wrong on hardware | Phase 4 (accepted until then) |
| DEV-05 | SNS-GNC-03 | `/wheel/odom` also provides yaw rate and pose (more than hardware can) | None if consumers follow ICD-SNS-GNC §4 | Allowed; consumers must use only `linear.x` |

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
