# Architecture Conventions

**Status:** Draft · **Owner:** Anthony Tracy

Rules every element in the [System Architecture](System-Architecture) follows. ICDs and component specs rely on these instead of repeating them, so an ICD only has to describe what is specific to its interface.

Where a ROS Enhancement Proposal (REP) exists, we follow it.

## 1. Units and Coordinates

- **SI units** everywhere: metres, seconds, radians, m/s, rad/s, kg ([REP-103](https://www.ros.org/reps/rep-0103.html)). Degrees only in human-facing files (course files use WGS84 decimal degrees).
- **Right-handed axes.** Body frames: X forward, Y left, Z up. World frames: **ENU** (X east, Y north, Z up).
- **Angles** in radians, yaw counter-clockwise from +X, normalised to (-π, π].
- **Geodetic** coordinates are WGS84 latitude/longitude in decimal degrees, altitude in metres above the ellipsoid.

## 2. Reference Frames and the TF Tree

ROS 2 keeps all coordinate frames in a **TF tree** (`tf2`): every frame has exactly one parent, and each parent → child edge is a transform published by exactly one node. Any node can then ask where frame A is in frame B at time t, and tf2 chains the transforms along the single path between them. We follow [REP-105](https://www.ros.org/reps/rep-0105.html) (frames for mobile robots) and [REP-103](https://www.ros.org/reps/rep-0103.html) (axes and units).

This section is the **TF registry**: the authoritative list of frames, of which component owns each edge, and of the frame each sensor's data is stamped with. It plays the role of an ICD for the TF tree.

### 2.1 Frame definitions

**World frames** (fixed to the ground):

| Frame | Origin | Axes | Property | Used for |
| --- | --- | --- | --- | --- |
| WGS84 (lat/lon) | Earth ellipsoid | - | Not a TF frame (curved surface) | Course files, `/navsat`. Converted to `map` (§2.5) |
| `map` | **Course start point** (the course file's `start`) | ENU: X east, Y north, Z up | Globally accurate, may **jump** when corrected | Waypoints, course map, global planning |
| `odom` | Where the car was when `state_estimation` was activated / reset | ENU at start (heading follows the initial pose) | **Continuous**, drifts slowly | Control, short-term motion, local planning |

**Robot frames** (fixed to the car):

| Frame | Origin | Axes | Used for |
| --- | --- | --- | --- |
| `base_footprint` | On the ground, directly below `base_link` (midway between the axles) | X forward, Y left, Z up | The car's pose: what `/odom` and the map pose describe. Motion models about the rear axle convert to this point |
| `base_link` | Chassis centre | X forward, Y left, Z up | Rigid body of the car; parent of all mounts |
| `imu_link` | IMU chip | Body axes (X fwd, Y left, Z up) | `/imu` data |
| `gps_link` | GNSS antenna phase centre | Body axes | `/navsat` data. The antenna's offset from `base_footprint` (lever arm) matters when turning |
| `camera_link` | Camera body | Body axes, pitched down 0.1 rad | Camera mount |
| `camera_optical_link` | Camera optical centre | **Optical**: Z forward (out of lens), X right, Y down | `/camera`, `/camera_info` image data |
| `*_steering_link` | Front kingpins | Rotates about Z with steering angle | Visualisation |
| `*_wheel_link` | Wheel centres | Rotates about Y with wheel spin | Visualisation |

### 2.2 The tree

```
map                                        world, ENU, origin = course start
 └── odom                                  world, ENU, origin = start of run
      └── base_footprint                   the car on the ground
           └── base_link                   chassis centre (+0.134 m up)
                ├── imu_link               (+0.06 m up)
                ├── gps_link               (-0.10 m back, +0.10 m up)
                ├── camera_link            (+0.23 m fwd, +0.07 m up, pitched down 0.1 rad)
                │    └── camera_optical_link   (rotated to Z-forward / X-right / Y-down)
                ├── front_left_steering_link   ── front_left_wheel_link
                ├── front_right_steering_link  ── front_right_wheel_link
                ├── rear_left_wheel_link
                └── rear_right_wheel_link
```

Offsets are for reference: the **URDF (`rc_racer_simulation/urdf/`) is the source of truth** for every robot frame and mount offset. Real-hardware mounts will be measured and written back into the URDF.

### 2.3 TF registry: edge ownership

| Edge (parent → child) | Owner (only publisher) | Topic | Rate | Status |
| --- | --- | --- | --- | --- |
| `map` → `odom` | VEH.GNC.LOC `localization` | `/tf` | ≥ 10 Hz | ⏳ planned |
| `odom` → `base_footprint` | VEH.GNC.SE `state_estimation` | `/tf` | 50 Hz | ✅ (single publisher checked by `test_state_estimation_node`) |
| `base_footprint` → `base_link` | `robot_state_publisher` (URDF) | `/tf_static` | static | ✅ |
| `base_link` → `imu_link`, `gps_link`, `camera_link` | `robot_state_publisher` (URDF) | `/tf_static` | static | ✅ |
| `camera_link` → `camera_optical_link` | `robot_state_publisher` (URDF) | `/tf_static` | static | ✅ |
| `base_link` → steering / wheel links | `robot_state_publisher` from `/joint_states` | `/tf` | ~20 Hz | ✅ (sim); hardware TBD |

**Why `map` → `odom` instead of `map` → `base_footprint`:** `base_footprint` already has a parent (`odom`), and a frame may have only one. Localization works out the true pose of the car in `map` (GPS, later cones), then publishes the **correction** `map → odom = (map → base_footprint) ∘ (odom → base_footprint)⁻¹`. The car's global pose is the lookup `map → base_footprint`. A GPS correction makes only `map → odom` jump; `odom` stays smooth for control.

**Not TF edges:** `/wheel/odom` is a topic only, so it cannot fight `state_estimation` over `odom → base_footprint`. The simulator's Ackermann plugin TF is not bridged for the same reason.

### 2.4 Sensor data frames

Every message's `header.frame_id` must name a frame in this tree.

| Topic | `frame_id` | Notes |
| --- | --- | --- |
| `/imu` | `imu_link` | |
| `/navsat` | `gps_link` | Position is WGS84; frame says where the antenna is on the car |
| `/camera`, `/camera_info` | `camera_optical_link` | Optical convention |
| `/wheel/odom` | `odom`, child `base_footprint` | Topic only, no TF |
| `/odom` | `odom`, child `base_footprint` | Published with the matching TF |

Checked in simulation by `test_simulation_smoke` (`test_sensor_frame_ids`): each sensor topic's `frame_id` matches this table and is reachable from `base_link` in the TF tree.

### 2.5 WGS84 ↔ `map`

Lat/lon is converted to `map` metres with an exact WGS84 → ECEF → local ENU conversion about the course start point (`geodetic_to_enu` / `enu_to_geodetic`, currently in `rc_racer_simulation/scripts/generate_course_world.py`). The simulator's world origin and NavSat reference are set to the same point, so simulator, course files and (later) localization agree. All components must use this one implementation; it moves to a shared package before localization is built ([Mission Orchestration](Mission-Orchestration) open item 2).

### 2.6 Rules

1. **One publisher per edge.** A component spec lists the edges it owns; nothing else may publish them.
2. **One parent per frame.** New frames are added to this registry first, then to the URDF or the owning component.
3. **Frame names are part of the interface.** Changing one is an ICD change.
4. Static mounts come from the URDF via `robot_state_publisher`, never from ad-hoc `static_transform_publisher` calls.
5. Data is stamped in the frame it was measured in. Consumers transform it with tf2; producers do not pre-transform.

### 2.7 Inspecting the tree

```bash
ros2 run tf2_tools view_frames                 # writes frames_<date>.pdf of the whole tree
ros2 run tf2_ros tf2_echo map base_footprint   # one transform, live
rviz2                                          # add the "TF" display to see every frame on the car
```

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
