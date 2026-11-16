# ICD-GNC-ACT: GNC to Actuation

**Status:** Draft · **Level:** L2 · **Owner:** VEH (Vehicle segment)
**Parties:** VEH.GNC (provider: `planning_control`) → VEH.ACT (consumer: `motor_controller`)

In simulation, the consumer side is implemented by TST.SIM's Ackermann drive ([ICD-VEH-TST](ICD-VEH-TST)). During manual testing, `teleop_twist_keyboard` stands in for the provider.

## 1. Purpose

The drive command: how fast to go and how sharply to turn. Actuation turns it into ESC and steering servo signals (via the PCA9685) and enforces the hardware limits.

## 2. Requirements

REQ-SYS-001 (drive autonomously), REQ-SYS-023 (fail-safe stop), REQ-TST-001 (simulator equivalence).

## 3. Interface Items

| ID | Name | Kind | Type | Direction | Frame | Rate | QoS |
| --- | --- | --- | --- | --- | --- | --- | --- |
| GNC-ACT-01 | `/cmd_vel` | topic | `geometry_msgs/msg/Twist` | GNC → ACT | `base_footprint` (implicit) | ≥ 10 Hz | Reliable, depth 10 |

## 4. Item Details

### GNC-ACT-01 `/cmd_vel`

| Field | Meaning | Range |
| --- | --- | --- |
| `linear.x` | Desired forward speed, m/s (negative = reverse) | ±3.0 m/s (sim). Hardware: **TBD** |
| `angular.z` | Desired yaw rate, rad/s (positive = left) | Any; limited by steering (below) |
| all other fields | Ignored | 0 |

Actuation converts the command to a steering angle with the bicycle model, `δ = atan(L · ω / v)` with wheelbase L = 0.384 m. It **clamps** δ to ±0.52 rad. A command tighter than the minimum turning radius (≈ 0.67 m) gives the tightest turn possible, not an error. A non-zero `angular.z` with `linear.x = 0` cannot be achieved by an Ackermann car and results in no steering change.

## 5. Timing and Failure

| Condition | Actuation behaviour |
| --- | --- |
| No command for 0.5 s (deadman) | Stop: speed 0, steering held |
| E-stop from Safety (ICD-FDR-ACT, not started) | Stop immediately; ignore `/cmd_vel` until e-stop clears |
| Command outside limits | Clamp; report WARN on `/diagnostics` |

## 6. Verification

| Side | Test | Checks |
| --- | --- | --- |
| Consumer (sim) | `test_simulation_smoke` drive tests | Ground-truth speed and turn rate within 20% of command |
| Consumer (hardware) | `motor_controller` tests, Phase 5 | Clamping, deadman, e-stop |
| Provider | `planning_control` component tests | Commands within limits, published at ≥ 10 Hz while active |

## 7. Open Items

| # | Question | Recommendation |
| --- | --- | --- |
| 1 | `Twist` vs `ackermann_msgs/AckermannDriveStamped` (speed + steering angle). The latter maps directly onto the hardware and would fit a future replacement of the simulator's steering ([SDP §4.2](Software-Development-Plan) steering lag) | Keep `Twist` for now: the simulator, teleop and Nav2-style planners all speak it. Revisit in Phase 4 |

## 8. Change Log

| Date | Change | Agreed by |
| --- | --- | --- |
| 2026-11-16 | Initial draft | |
