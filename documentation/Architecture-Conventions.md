# Architecture Conventions

**Status:** Draft · **Owner:** Anthony Tracy

Rules every element in the [System Architecture](System-Architecture) follows. ICDs and component specs rely on these instead of repeating them, so an ICD only has to describe what is specific to its interface.

Where a ROS Enhancement Proposal (REP) exists, we follow it.

## 1. Units and Coordinates

- **SI units** everywhere: metres, seconds, radians, m/s, rad/s, kg ([REP-103](https://www.ros.org/reps/rep-0103.html)). Degrees only in human-facing files (course files use WGS84 decimal degrees).
- **Right-handed axes.** Body frames: X forward, Y left, Z up. World frames: **ENU** (X east, Y north, Z up).
- **Angles** in radians, yaw counter-clockwise from +X, normalised to (-π, π].
- **Geodetic** coordinates are WGS84 latitude/longitude in decimal degrees, altitude in metres above the ellipsoid.

## 2. Coordinate Frames

Following [REP-105](https://www.ros.org/reps/rep-0105.html):

```
earth ─(fixed)─> map ─(LOC)─> odom ─(SE)─> base_footprint ─(URDF)─> base_link ─> sensor frames
```

| Frame | Meaning | Published by | Property |
| --- | --- | --- | --- |
| `map` | World-fixed ENU frame, origin at the course start point | `localization` (`map`→`odom`) | Accurate globally, may jump |
| `odom` | World-fixed ENU frame where the car starts each run | `state_estimation` (`odom`→`base_footprint`) | Continuous, drifts slowly |
| `base_footprint` | Car's reference point on the ground, directly below `base_link` (midway between the axles). Motion models that work about the rear axle convert to this point | URDF root | Robot reference for odometry |
| `base_link` | Car body (chassis centre) | `robot_state_publisher` (URDF) | Fixed to `base_footprint` |
| `imu_link`, `gps_link`, `camera_link`, `camera_optical_link` | Sensor mounts | `robot_state_publisher` (URDF) | Fixed to `base_link` |

Rules:
- **Each transform has exactly one publisher.** No two nodes may publish the same TF edge.
- Wheel odometry is published as a topic only (`/wheel/odom`), never as a TF, so it cannot fight `state_estimation`.
- The course file's start point is the origin of `map` (same as the simulator's world origin).
- `camera_optical_link` uses the optical convention (Z forward, X right, Y down) for image data.

## 3. Time

- All headers use the node clock. Every node supports `use_sim_time`, which is `true` in simulation and `false` on hardware.
- Sensor messages carry the **measurement** time, not the publish time.
- Timeouts and rates are in simulation time when `use_sim_time` is true.

## 4. Naming

| Item | Convention | Example |
| --- | --- | --- |
| Package | `snake_case`, matches component name in the tree | `state_estimation` |
| Node | `<component>` (one node per component unless the spec says otherwise) | `state_estimation` |
| Executable | `<component>_node` | `state_estimation_node` |
| Topics | Absolute, lower case, grouped by source: `/<group>/<name>` | `/wheel/odom`, `/imu`, `/camera` |
| Parameters | `snake_case`, grouped with dots | `fusion.method`, `inputs.gps.enabled` |
| Custom interfaces | In `rc_racer_interfaces`, `PascalCase` | `NavigateToWaypoint.action` |

Topic names are part of an ICD. A component may **remap** but may not rename without changing the ICD.

## 5. Message Types

1. Use a **standard** ROS type if one fits (`sensor_msgs`, `nav_msgs`, `geometry_msgs`, `vision_msgs`, `diagnostic_msgs`).
2. Otherwise define the type in **`rc_racer_interfaces`**. No package defines its own messages, so no component depends on another just to get a type.
3. Every estimate carries a **covariance** (REQ-GNC-006). Unknown covariance is `-1` in the first element (ROS convention), never zero.

## 6. Quality of Service

| Data | Reliability | Durability | Depth | Example |
| --- | --- | --- | --- | --- |
| High-rate sensor streams | Best effort (`SensorDataQoS`) | Volatile | 5 | `/imu`, `/camera` |
| State estimates, commands | Reliable | Volatile | 10 | `/odom`, `/cmd_vel` |
| Data that late joiners need | Reliable | Transient local | 1 | course map, static config |
| Health | Reliable | Volatile | 10 | `/diagnostics` |

Subscribers to sensor streams use `SensorDataQoS` so they work with both reliable (simulator bridge) and best-effort (drivers) publishers.

## 7. Node Structure

- **Logic / adapter split** ([SDP §2](Software-Development-Plan)): algorithms live in a plain class with no ROS includes; the node only converts messages and calls the logic.
- **Managed lifecycle nodes** for all vehicle components (`configure` → `activate` → `deactivate` → `cleanup`). `rc_racer_bringup` brings them up in order; Safety can deactivate them.
- **Parameters** come from YAML files in `rc_racer_bringup/config/`, one section per node. Nodes validate parameters at `configure` and fail there, not at runtime.
- **Capabilities** that the Executive needs to sequence ("do X and tell me when done or failed") are exposed as **actions**, not topics. See [Mission Orchestration](Mission-Orchestration).

## 8. Health and Telemetry

Instead of an ICD per element for health (the FDR column of the L2 N² chart):

- Every vehicle component publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics` (via `diagnostic_updater`) at ≥ 1 Hz, with level OK / WARN / ERROR / STALE and the reason.
- Every component reports **stale inputs** (no message within its ICD's timeout) as WARN or ERROR.
- Safety treats a component with no diagnostics for longer than its timeout as failed.
- Telemetry records `/diagnostics`, all ICD topics and TF with `rosbag2` (REQ-SYS-025).

## 9. Documents

| Document | Contains | Template |
| --- | --- | --- |
| [System Requirements](System-Requirements) | Numbered requirements, allocation, TBD register | - |
| [System Architecture](System-Architecture) | Breakdown tree, N² charts, ICD register | - |
| `ICD-<A>-<B>` | One interface between two siblings | [ICD Template](ICD-Template) |
| `<component>` | One component's spec: inputs, outputs, parameters, behaviour, verification | [Component Template](Component-Template) |
