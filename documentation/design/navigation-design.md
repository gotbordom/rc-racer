# Navigation Design (feature 3.0)

**Status:** Draft · **Feature:** 3.0 `navigation-design` · **Covers:** VEH.SNS (cameras), VEH.PER, VEH.GNC (SE, LOC, MAP) · **Started:** 2026-11-28

How the car knows where it is and what is around it: the tiers from dead reckoning up to SLAM, the gates between them, the landmark catalogue, the camera options, and which ROS packages we use or write for each part. It sets the requirements that 3.1 `localization`, 3.2 `mapping` and the paused 2.2 `object-detection` are built to.

The aim is a car that is **more robust than the competition strictly needs**: every tier degrades to the one below, down to the dead reckoning we already have.

Owner decisions are marked **Decided (owner, date)**; everything else is a recommendation. §11 lists the decisions.

---

## 1. Context and constraints

| | Now | Notes |
| --- | --- | --- |
| Dead reckoning | ✅ `state_estimation`: IMU + one rear-axle encoder, EKF, slip handling (2.1–2.5) | 0.2–0.4 % drift in grip; poor in long slides (VAL-018, #11): nothing measures velocity in a slide |
| Absolute position | ❌ No GPS (ISSUE-001) | The course is given in WGS84; without GPS, only **known landmarks** tie the car to it |
| Computer | Jetson Nano 4 GB (Maxwell GPU) | JetPack 4.6 is the last release for the Nano (Ubuntu 18.04); ROS 2 Humble runs in the community `dustynv/ros:humble-*-l4t-r32.7.1` containers. NVIDIA's own Isaac ROS needs an Orin and JetPack 6 |
| Cameras | 2 × IMX219, 200° (in use), 1 × IMX219 105° (in testing) | Rolling shutter, **no hardware trigger** on the IMX219 |
| Competition | Three attempts on the same course (REQ-SYS-024) | **Owner, 2026-11-28:** attempt 1 is commonly a **mapping run**, attempts 2–3 compete. To be checked against the event's rules |
| Sim | Gazebo Fortress, one ideal 60° pinhole camera (DEV-08), flat untextured ground | Visual odometry needs texture: the sim world needs work before it can test this (§9) |

---

## 2. The architecture in one picture

```
 cameras ──► camera driver ──► stereo pairing ──► rectify / depth ─────────────► local obstacle map (MAP)
                                     │
                                     ▼
                           visual front end  (features: track, match, promote)
                              │            │                     ▲
             relative motion  │            │  observations of    │ what should be in view
                              ▼            ▼  known landmarks    │
 IMU + wheel ──► state_estimation      localization ◄──── landmark catalogue ◄── course file
                 (odom → base_footprint) (map → odom,         ▲                   survey file
                  smooth, never jumps)    tiers, gates)        │                   saved run-1 map
                              │            │                   │
                              └──► tier + uncertainty ──► EXE / PLN / Safety
 object_detection (cones) ──► semantic landmark observations ──┘
```

**The principle: corrections, not switches.** The pose source is never switched. `state_estimation` always runs, and every other source enters as a **measurement** that is gated, and simply not fused when it fails. The REP-105 split already gives this structure:
- **`odom` → `base_footprint`** (`state_estimation`): the motion layer. It is always available, always smooth and never jumps. Relative motion from visual odometry is fused here.
- **`map` → `odom`** (`localization`): the correction layer. Absolute fixes from known landmarks and SLAM adjust it. When they fail it **holds** its last value while its uncertainty grows: the car carries on dead reckoning from the last good fix.

---

## 3. Tiers

| Tier | Position comes from | Available when | Expected accuracy (to be measured) |
| --- | --- | --- | --- |
| **T3 Map-localized** | Matches against the run-1 map + T2 | Runs 2–3, map matches passing their gate | Bounded: doesn't grow with distance |
| **T2 Visual odometry** | VO + T1, with fixes from known semantic landmarks (cones, survey points) | Cameras healthy, features tracked | Drift ≪ 2 %; survives wheel slip |
| **T1 Dead reckoning** | IMU + wheel (today's EKF) | IMU healthy | 0.2–2 % drift, worse in slides |
| **T0 Unsafe** | Nothing trustworthy | IMU lost, or the uncertainty above what the current task needs | Stop: Safety / FDIR |

`localization` publishes the **current tier and its uncertainty** (§10, `rc_racer_msgs/LocalizationStatus`). **It only reports**: the executive decides what a tier means for the mission. For example, in T1 it slows down and widens the cone search, and in T0 it stops. Safety watches the same topic. *(Owner agreed 2026-11-28.)*

---

## 4. Gates

Every tier has an **entry gate**, an **exit gate** and **hysteresis**, so the tier doesn't flap.

### 4.1 Relative-motion gate (VO into `state_estimation`), per measurement

1. **Quality:** enough tracked features (≥ N), a high inlier ratio, and a low reprojection error. Reprojection error measures how well the computed motion explains where the features appeared.
2. **Physics:** the VO turn rate agrees with the gyro, the most trustworthy sensor. Its forward speed agrees with the wheel **while no slip is detected**, and while slipping VO is the referee instead: that is the point of having it.
3. **Statistics:** the innovation (χ²) gate: fused only if within the filter's predicted uncertainty, the same machinery as the slip checks.
4. **Exit / re-entry:** lost after N rejected frames in a row; back after M accepted frames in a row.

### 4.2 Landmark association gate (`localization`), per observation

- **Semantic landmarks** (cones, survey points): match by class and position within 3σ, and the match must be **unambiguous**: the nearest candidate clearly closer than the second-nearest. When several are seen at once, joint compatibility (JCBB, §14) instead of one by one.
- **Visual landmarks** (run-1 features): place recognition (bag of words), then a geometric check on the inliers. Features of the run-1 map that are no longer seen are counted, so a changed scene is noticed.

### 4.3 Correction consistency gate (`localization`), per correction

- The correction a fix asks for must fit within the uncertainty that dead reckoning has built up since the last good fix: 0.3 m after 20 m of driving is fine, 5 m is suspect.
- **Re-entry after a lapse is gated harder:** several consistent fixes in a row, because the scene may have changed since run 1.
- A **large but repeatedly consistent** correction is accepted: that's the car realising it was really lost. It is spread over time like the slip corrections (`SpreadCorrection`), so `map` doesn't snap.

### 4.4 Integrity gate (any tier to T0)

The position uncertainty, against what the current **mission phase** needs: metres while driving between waypoints, centimetres in the final cone approach. A tier is "usable" only if its bound meets the phase's limit. This is the *protection level* idea from aviation GNSS integrity monitoring (§14).

---

## 5. Features and the landmark catalogue

### 5.1 Two kinds of features, one front end

| | **Tracked features** | **Known landmarks** |
| --- | --- | --- |
| What | Corners and textures found and followed from frame to frame | Things whose position is known: cones, start and finish, surveyed points, features saved from the mapping run |
| Lives | Seconds | The whole run, and across runs |
| Gives | **Relative motion** → `state_estimation` | **Absolute position** → `localization` |
| Error | Drifts slowly | Bounded |

In each frame, the visual front end:
1. tracks features from the previous frame and gives a **relative-motion** measurement;
2. asks the catalogue what **should** be in view from the predicted pose, matches it, and gives **absolute observations**.

One observation can feed both outputs.

### 5.2 Feature lifecycle

> **tracked** (seconds) → **promoted** (seen consistently from several viewpoints, so its position is well known) → **saved** in the run-1 map → **known** in runs 2–3

On run 1 SLAM builds the catalogue; on runs 2–3 it mostly reads from it.

### 5.3 The catalogue (`landmark_catalog`, new)

One component owns the known landmarks, and everything else asks it.

| Source | Kinds | Format |
| --- | --- | --- |
| Course file (`courses/<name>.yaml`) | cones (mandatory, bonus, finish), start | WGS84, as now |
| Survey file (`surveys/<name>.yaml`, new) | hand-measured points: poles, corners, a fiducial tag, if allowed | WGS84 or a local datum, with its uncertainty |
| Saved run-1 map | visual features with descriptors | the SLAM package's own database (binary), **referenced, not copied** |
| Future: GPS survey | any | WGS84 |

Each entry:

```yaml
- id: M1
  kind: cone            # cone | survey_point | fiducial | visual (in the SLAM db)
  source: course        # course | survey | run1_map
  position: {latitude: 37.6602450, longitude: -121.8888300, altitude: 100.0}
  stddev: {horizontal: 0.05, vertical: 0.10}     # m: a course cone vs a hand survey
  recognise: {class: cone, height: 0.457, colour: orange}
  static_confidence: 0.7  # a cone can be bumped; a building can't
```

It answers two questions:
- **what's in view** from a given pose, with predicted bearing, range and uncertainty. This also helps detection: "a cone about 8 m ahead, 5° left";
- **lookup by id.**

---

## 6. Cameras and depth

### 6.1 Why sync matters

Pairing two streams by timestamp (`message_filters`) only pairs the frames: it doesn't make the exposures simultaneous. Two free-running sensors can be up to half a frame apart, **16 ms at 30 fps**:
- driving straight at 3 m/s, that's 5 cm of travel between the left and right image: a depth error;
- **turning at 2 rad/s, it's 0.032 rad: about 18 px of false disparity** in a 640 px, 60° image. That ruins stereo depth.

So sync is a requirement for stereo depth while turning, not a refinement.

### 6.2 Sync options

| # | Option | How it syncs | Shutter | Effort | Notes |
| --- | --- | --- | --- | --- | --- |
| A | **Synchronized IMX219 stereo kit** (e.g. Arducam's for the Jetson Nano) | The two sensors share one clock and are started together by an I²C broadcast; the pair arrives as one frame | Rolling | Low: vendor driver | Keeps the current sensor class; **rolling shutter remains** (skew at speed and over bumps) |
| B | **Global-shutter pair with external trigger** (e.g. OV9281 mono, AR0234 colour) **+ our trigger-and-timestamp board** | A microcontroller (or a Jetson GPIO/PWM pin) pulses both trigger pins; the board also timestamps each trigger and the IMU sample | Global | Medium–high: board, level shifting, and the sensor driver's trigger mode | Best quality. The trigger timestamps also give **camera-IMU time sync**, which VIO needs most. Check each module's trigger support: some variants don't (e.g. Arducam's AR0234 xISP) |
| C | **Integrated stereo depth camera** (e.g. OAK-D, RealSense) over USB | In hardware, inside the camera | Global on most models | Low: vendor ROS driver | **Depth is computed on the camera**, which takes the heaviest work off the Nano. Bulkier; fixed baseline; USB power and bandwidth |
| 0 | *Baseline:* two free-running IMX219s, paired by timestamp | none | Rolling | none | Fine for monocular VO on one camera; poor for stereo depth while turning (§6.1). Measure the real offset on the bench (film a fast LED with both) |

**Decided (owner, 2026-11-28): stereo, planned as B** (global shutter, hardware-synced), **open to C**. The owner evaluates which to buy. The design assumes time-synced images, and the sim provides them. Because the bought camera may turn out not to be synced or global shutter, **the camera's timing is configuration** (§6.5), not an assumption in the code.

### 6.3 Depth range

Stereo depth error grows with distance squared: `Δz ≈ z² · Δd / (f · B)`, with disparity error Δd ≈ 0.25 px.

| | f (640 px wide) | B | Δz at 5 m | Δz at 10 m |
| --- | --- | --- | --- | --- |
| 60° lens | 554 px | 0.12 m | 0.09 m | 0.38 m |
| 105° lens | 245 px | 0.12 m | 0.21 m | 0.85 m |

So dense depth suits **obstacles out to about 5–8 m**. Cones at 10 m are better ranged from their known size (2.2). The 200° lenses are good for VO, with lots of features in view, but poor for dense depth, which needs rectifying to a narrower view: the 105° lens is the better stereo compromise.

### 6.4 Monocular alternative: motion stereo

One camera, with depth from its own motion between frames, and the scale from the IMU and, for us, the wheel.

| | Stereo | Monocular motion stereo |
| --- | --- | --- |
| Hardware | Two synced cameras | One camera: no sync problem |
| Scale | Direct, from the baseline | From the IMU or the wheel; **unobservable at constant velocity** for IMU-only, which the **wheel fixes** ("VINS on Wheels", §14) |
| Depth ahead | Yes | **No: there is no parallax at the point you drive towards** (the focus of expansion), so an obstacle straight ahead is the hardest thing to see |
| Depth while stopped | Yes | No |
| Compute | Matching (or on the camera, option C) | Lighter for sparse VO; dense mono depth is heavy (REMODE), and learned mono depth (MiDaS, Depth Anything) needs the GPU and gives only relative depth |
| VO quality | Good | Good with IMU + wheel |

**Decided (owner, 2026-11-28): fallback only, and a nice-to-have** built after everything else. Monocular VIO + wheel is a sound VO tier with one camera, but weak for obstacles ahead, which is what the local map needs.

### 6.5 Camera timing is configuration

The bought camera may not be global shutter or hardware-synced, so the pipeline reads its timing from the robot's configuration:

```yaml
stereo:
  sync: hardware            # hardware | software
  max_pair_offset: 0.0005   # s: worst offset between left and right exposures (measured, §9)
  shutter: global           # global | rolling
  readout_time: 0.0         # s: top-to-bottom row time for rolling shutter (~0.02 for an IMX219)
  time_offset_imu: 0.0      # s: camera-IMU offset, from Kalibr (§8)
```

Mostly this **changes how the error is calculated**, as the owner expected:
- **Pair offset:** the depth and VO covariance gains a motion term, roughly the speed and yaw rate times `max_pair_offset`. With hardware sync it is negligible.
- **Rolling shutter:** each row has its own time, so the motion during `readout_time` is another covariance term, unless the front end models it.

It also changes **two gates**, not only the error:
- with software sync, **stereo depth is not used while turning faster than a limit**, because at 2 rad/s the false disparity (§6.1) is larger than any honest covariance;
- with rolling shutter, VO is gated harder over bumps, where the skew is worst.

The sim uses `hardware` / `global`; the other settings are tested by faking the offset and skew in the sim's camera adapter (§9).

---

## 7. Mapping run and runs 2–3

| | Run 1: mapping | Runs 2–3: competition |
| --- | --- | --- |
| Speed | Slow, careful | Race |
| SLAM | Builds the map, closes loops, promotes features | **Localization mode** against the saved map |
| Catalogue | Gains the run-1 visual landmarks | Reads them |
| Obstacles | Mapped and saved | Known in advance: planning routes ahead |
| Cones | Confirmed and refined against the course file | Fixes, as checks |
| Fallback | T2 → T1 | T3 → T2 → T1 |

**Decided (owner, 2026-11-28): the mapping run is not required, it is configurable.** Some competitions won't allow it, so the robot has **operation modes** chosen in its configuration, which the mission's decision tree (Mission Orchestration) builds on later:

| Mode | What runs | Top tier |
| --- | --- | --- |
| `mapping` | SLAM builds and saves the map; slow mission profile | T2 (T3 isn't available while building) |
| `operating` | Localizes against a saved map **if one is configured**, else without | T3 with a map, T2 without |

```yaml
operation:
  mode: operating           # mapping | operating
  map: maps/practice_course_run1.db   # optional: absent = no map, top tier T2
```

**Proposed requirement** (System Requirements):
> REQ-SYS-0xx: The system shall support a mapping mode, which builds and saves a course map, and an operating mode, which uses a saved map when one is configured and operates without one otherwise, selected by configuration.

---

## 8. Packages: use or write

**Use** = an existing package, configured. **Evaluate** = candidates to benchmark before choosing. **Write** = our code.

| Feature | Decision | Package(s) | Notes |
| --- | --- | --- | --- |
| ROS 2 on the Nano | **Use** | `dustynv/ros:humble-*-l4t-r32.7.1` (jetson-containers) | JetPack 4.6, Humble in Docker |
| Camera driver (CSI) | **Use** | `gscam` with `nvarguscamerasrc` (Nano); `isaac_ros_argus_camera` only on an Orin | |
| Camera driver (option C) | **Use** | `depthai-ros` (OAK-D) or `realsense2_camera` | Vendor drivers |
| Trigger and timestamp board (option B) | **Write** | firmware + `sync_board_driver` node | Publishes trigger stamps; pairs them with frames and IMU |
| Stereo pairing | **Use** | `message_filters` (ExactTime when hardware-synced, ApproximateTime otherwise) | |
| Calibration: intrinsics, stereo | **Use** | `camera_calibration` | |
| Calibration: camera-IMU, time offset | **Use** (offline) | Kalibr | ROS 1 tool, run in its own container |
| Rectify, disparity, point cloud | **Use** | `stereo_image_proc`, `image_proc` (image_pipeline) | CPU; on the camera for option C |
| Visual front end: VO, SLAM, map save/load | **Evaluate**, recommend **RTAB-Map** first | `rtabmap_ros` (`rtabmap_odom` stereo odometry, `rtabmap_slam`): released for Humble, maintained, has **mapping and localization modes with a saved database**, which fits §7. Others: **OpenVINS** (MSCKF VIO; ROS 2 build from source), **ORB-SLAM3** (community ROS 2 wrappers; stereo-inertial, multi-map), **VINS-Fusion** (community ROS 2 ports). `isaac_ros_visual_slam` only with an Orin | Choose by a bench benchmark on the Nano (§9): rate, CPU, drift |
| VO into the EKF, gates (§4.1) | **Write** | `state_estimation` (extended) | Keep our tested EKF rather than `robot_localization`; VO is one more gated measurement |
| Landmark catalogue (§5.3) | **Write** | `landmark_catalog` (new) | Loads the course and survey files, references the SLAM database |
| Association, fixes, tiers, gates (§3–4) | **Write** | `localization` (3.1) | Publishes `map` → `odom` and the tier status |
| Local obstacle map | **Use** | `nav2_costmap_2d` with `spatio_temporal_voxel_layer` (Humble) | From the stereo point cloud; a terrain layer is written later if needed (gullies, REQ-SYS-010) |
| Saved obstacle map (run 1) | **Use / write** | RTAB-Map's map output (occupancy grid) + a small saver/loader | |
| Object detection (§13) | **Write** the package and the `colour_shape` plugin; **use** a learned model | `object_detection` (2.2) with `pluginlib` detectors; a YOLO-family model on TensorRT for the `learned` plugin | One package for any labelled object; cones are a configured class |
| Shared gate library (§12) | **Write** | `fusion_gates` (new, pure C++) | Used by `state_estimation` and `localization` |
| Operation modes (§7) | **Write** | robot configuration YAML (`mapping` / `operating`), read by `localization` and the SLAM launch | Later driven by the mission's decision tree |
| Tier status message | **Write** | `rc_racer_msgs` (new): `LocalizationStatus` | tier, uncertainty, active sources, gate counters |
| Sim stereo pair, textured world | **Write** | `rc_racer_simulation` | Second camera in the URDF; textured terrain, scattered features |
| Fault injection for the gates | **Write** | `sensor_adapter_node` (extended), camera fault node | Black or frozen frames, featureless stretches, delays |
| Monocular fallback (§6.4) | **Evaluate**, later | OpenVINS (mono) or ORB-SLAM3 (mono-inertial) | Nice-to-have, after everything else (Decision 7) |

### 8.1 The visual front-end candidates, explained

The **visual front end** is the box in §2 that turns images into motion and map matches:
- **feature detection and tracking:** finding corners in each image and following them;
- **motion estimation:** working out the camera's movement from how the features moved, rejecting bad matches (RANSAC);
- **local optimisation (bundle adjustment):** refining recent poses and feature positions together;
- for SLAM: **place recognition** (have I seen this before?) and **map storage**.

That is years of specialised work, which these packages have already done and tested. We **use** one and write the glue that turns its output into gated measurements (§4) and catalogue entries (§5).

| Package | What it is | Gives us | Save a map, localize in it later (§7) | Uses the IMU | Humble | Licence |
| --- | --- | --- | --- | --- | --- | --- |
| **RTAB-Map** (`rtabmap_ros`) | Graph SLAM library with its own stereo VO | VO **and** SLAM **and** an occupancy grid, in one | **Yes**: a saved database and a localization mode | Optional | **Released binaries** | BSD |
| **OpenVINS** | Visual-inertial odometry only: an MSCKF, an EKF variant close to our own | Very good VO; estimates the camera-IMU time offset online | No (VO only) | Yes, central | Builds from source | GPL-3.0 |
| **ORB-SLAM3** | Feature-based SLAM, stereo-inertial, several maps | Accurate VO and SLAM; strong relocalization | Yes (saved "atlas") | Yes | Community wrappers | GPL-3.0 |
| **VINS-Fusion** | Optimisation-based visual-inertial odometry with loop closure | Good VO; a GPU version for Jetson | Limited | Yes, central | Community ports | GPL-3.0 |

**Why RTAB-Map first:** one package covers the VO tier, the mapping run, localization against the saved map and the obstacle grid, it's released for Humble, and it's BSD-licensed like our MIT code. **Its risk:** its VO is less accurate than OpenVINS or ORB-SLAM3. A valid combination is OpenVINS for the VO tier with RTAB-Map for the map tier. The GPL packages are fine to run as separate nodes for our own use; distributing them inside the car's software would bring GPL obligations.

**Decision:** pending the Nano benchmark (§9).

---

## 9. Validation

**Bench spikes** (on the real hardware, before choosing):
1. **Sync offset** of the current two IMX219s (film a fast LED or a phone stopwatch with both cameras).
2. **Front-end benchmark on the Nano:** RTAB-Map stereo odometry, and OpenVINS, at 640 × 480 and lower, measuring rate, CPU, memory and drift on a walked loop.
3. **Option C trial**, if chosen: depth quality outdoors and its effect on the Nano.

**In the sim** (TST work first):
- a stereo camera pair in the URDF; a **textured** ground and scattered features, since VO fails on the current plain plane;
- **fault injection** for every gate in §4;
- scenarios like the VAL ones: each fault makes the tier drop, then recover, with `odom` continuous and the covariance honest throughout; plus a mapping run followed by a localization run.

**Known sim gaps:** Gazebo has no rolling shutter, exposure changes or motion blur, so those are tested on the bench and on recorded real footage.

---

## 10. Interfaces to add

| Interface | Content | ICD |
| --- | --- | --- |
| Stereo images | `/stereo/left/image_raw`, `/stereo/right/image_raw` + `camera_info` (or the vendor's topics) | ICD-SNS-PER (extend) |
| Depth | `/stereo/points` (`sensor_msgs/PointCloud2`) | ICD-SNS-PER or a PER-internal ICD, depending on where depth lives (Open Decision 3) |
| VO relative motion | `nav_msgs/Odometry` (twist with covariance) → `state_estimation` | new: PER-GNC or GNC-internal |
| Landmark observations | bearing/range or 3D, with covariance, class or descriptor id | ICD-PER-GNC (extend) |
| Catalogue queries | service or latched topic: landmarks in view of a pose | GNC-internal |
| Tier status | `rc_racer_msgs/LocalizationStatus` | ICD-GNC-EXE (new) |

---

## 11. Decisions

| # | Decision | Options | Status |
| --- | --- | --- | --- |
| 1 | Camera route | A, B, C or 0 (§6.2) | **Decided (owner, 2026-11-28): stereo, B planned, open to C**; owner evaluates which to buy. Timing is configuration (§6.5) |
| 2 | Compute | Stay on the Nano / move to an Orin Nano | **Decided: stay on the Nano**; benchmarks (§9) before any move to a new GPU |
| 3 | Where depth lives | Sensing / its own Perception component | **Decided: Sensing**, like a lidar would be |
| 4 | Front-end package | RTAB-Map / OpenVINS / ORB-SLAM3 / VINS-Fusion (§8.1) | Open: by the Nano benchmark; RTAB-Map recommended first |
| 5 | Fusion structure | Split back ends / one tightly coupled estimator | **Decided: split**; nothing tightly coupled unless it has to be. Modular gates in each fusion package (§12) |
| 6 | Mapping run | Required / configurable | **Decided: configurable**, operation modes `mapping` / `operating` (§7) |
| 7 | Monocular route | Fallback / main | **Decided: fallback only, a nice-to-have** after everything else |
| 8 | Roadmap | 3.0 → TST sim work → 3.1 `localization` (VO, catalogue) → 3.2 `mapping` → 2.2 detection | **Decided: as listed** |
| 9 | Object detection scope | One package per object / one package for any object | Proposed: one package, any object (§13) |

---

## 12. Modular fusion: one gate framework, two estimators

"Split" (Decision 5) means **two estimators, each owning one frame**, connected only by topics:
- `state_estimation` owns `odom` → `base_footprint`; its inputs are the relative ones: IMU, wheel, VO;
- `localization` owns `map` → `odom`; its inputs are `/odom` and the absolute ones: landmark fixes, SLAM matches.

Neither knows the other's internals. `localization` takes `/odom` as its motion and **must not fuse VO again**: that would count the same information twice. The correlation between `odom` and the fixes is ignored, the usual REP-105 practice.

**Inside each, sources and gates are modules, not code paths.** The pattern already exists in `state_estimation` (`slip_checks.hpp`: the checks as pure functions); 3.1 generalises it:

| Piece | What | Example |
| --- | --- | --- |
| **Measurement adapter** | Turns one source's message into a measurement: value, covariance, which states it observes | `WheelSpeed`, `ImuYawRate`, `VoBodyVelocity`, `ConeFix` |
| **Gate** | A pure function: (measurement, state) → accept or reject, and why | `finite`, `innovation_chi2`, `slip_window`, `vo_quality`, `gyro_agreement`, `unambiguous_association` |
| **Source monitor** | Hysteresis per source: active → suspect → lost → active, from its gate results | lost after 5 rejects in a row, back after 10 accepts |
| **Filter core** | Predict and update; knows nothing about sources | today's `ExtendedKalmanFilter` |

The **gate library is shared** by both estimators (a small pure C++ package, e.g. `fusion_gates`), so `innovation_chi2` is written and tested once. Each estimator lists its sources in its configuration:

```yaml
sources:
  wheel:
    adapter: wheel_speed
    topic: /wheel/odom
    gates: [finite, slip_checks, innovation_chi2]
  vo:
    adapter: body_velocity
    topic: /vo/odom
    gates: [finite, vo_quality, gyro_agreement, innovation_chi2]
    monitor: {lose_after: 5, regain_after: 10}
```

So adding a source is an adapter plus configuration, with no change to the filter, and a source can be switched off in configuration to test the fallback. `localization` derives the **tier** (§3) from its source monitors. **Honest note:** today the IMU and wheel are wired into `state_estimation` directly; moving them onto this pattern is a refactor at the start of 3.1, before VO is added, like the slip-check refactor in 2.5.

---

## 13. Object detection: one package, any object

A package per object (`cone_detection`, `person_detection`, …) would repeat the camera handling, the geometry and the output in each. Instead, **one `object_detection` package with pluggable detectors** (`pluginlib`, like nav2's costmap layers), all producing the same output:

```
images (+ depth) ──► detector plugins ──► common detection: class, score, position + covariance ──► merge duplicates ──► /perception/detections
                      • colour_shape  (cones: needs no training)
                      • learned       (a multi-class model, e.g. YOLO on TensorRT: person, robot, cone, …)
                      • future: open-vocabulary (any object named in text), compute permitting
```

**Classes are configuration**, not code. Each says which detector finds it, its size (for range from size, as 2.2 specced for cones), whether it moves, and whether it's a landmark kind (§5.3):

```yaml
classes:
  cone:   {detector: colour_shape, height: 0.457, dynamic: false, landmark: cone}
  person: {detector: learned, height: 1.7, dynamic: true}
  robot:  {detector: learned, dynamic: true}
```

**"Any object" means two different things**, handled in two places:
1. **Objects that need a label:** cones are goals, people and robots move. This is `object_detection`: it can only label what a detector knows.
2. **Anything in the way, whatever it is:** this doesn't need a label. **Stereo depth goes straight into the obstacle map** (`mapping`, §8). A rock is avoided because it is there, not because it was recognised.

Effect on 2.2: its spec becomes the `colour_shape` plugin plus the package frame, and `/perception/cones` becomes `/perception/detections` with `class_id` (ICD-PER-GNC changes when 2.2 resumes).

---

## 14. Leading research and papers

Read in roughly this order.

**Overviews**
- Cadena et al., *Past, Present, and Future of Simultaneous Localization and Mapping: Toward the Robust-Perception Age*, IEEE T-RO 2016 (arXiv 1606.05830). The field map, including robustness and failure detection.
- Scaramuzza & Fraundorfer, *Visual Odometry, Part I* (IEEE RAM 2011) and Fraundorfer & Scaramuzza, *Part II* (2012). VO from first principles.

**Visual and visual-inertial odometry**
- Mourikis & Roumeliotis, *A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation*, ICRA 2007. The filter behind OpenVINS: VIO in an EKF, close to how we work.
- Geneva et al., *OpenVINS: A Research Platform for Visual-Inertial Estimation*, ICRA 2020.
- Qin, Li & Shen, *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator*, IEEE T-RO 2018 (arXiv 1708.03852).
- Forster et al., *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry*, IEEE T-RO 2017.
- Usenko et al., *Visual-Inertial Mapping with Non-Linear Factor Recovery* (Basalt), IEEE RA-L 2020 (arXiv 1904.06504).
- **Wu, Ahmed, Georgiou & Roumeliotis, *VINS on Wheels*, ICRA 2017.** Ground vehicles: why IMU-only scale fails at constant velocity and how wheel odometry fixes it. Directly relevant to us.
- Jeon, Jung & Myung, *Run Your Visual-Inertial Odometry on NVIDIA Jetson: Benchmark Tests on a Micro Aerial Vehicle*, 2021 (arXiv 2103.01655). VIO packages benchmarked on Jetson boards.

**SLAM and map reuse**
- Campos et al., *ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM*, IEEE T-RO 2021 (arXiv 2007.11898).
- Labbé & Michaud, *RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation*, Journal of Field Robotics 2019. Mapping then localization against a saved map.
- Schneider et al., *maplab: An Open Framework for Research in Visual-Inertial Mapping and Localization*, IEEE RA-L 2018. Map once, localize later.
- Rosinol et al., *Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping*, ICRA 2020 (arXiv 1910.02490). Semantic landmarks with geometric ones.
- Gálvez-López & Tardós, *Bags of Binary Words for Fast Place Recognition in Image Sequences* (DBoW2), IEEE T-RO 2012. How relocalization finds the place.

**Estimation back ends**
- Kaess et al., *iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree*, IJRR 2012, and Dellaert & Kaess, *Factor Graphs for Robot Perception*, 2017. The tightly coupled alternative (Open Decision 5).

**Gates, association, integrity**
- Bar-Shalom, Li & Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley 2001. Innovation (χ²) gating and filter consistency.
- Neira & Tardós, *Data Association in Stochastic Mapping Using the Joint Compatibility Test* (JCBB), IEEE T-RA 2001. Associating several landmarks at once.
- Zhu, Marais, Bétaille & Berbineau, *GNSS Position Integrity in Urban Environments: A Review of Literature*, IEEE T-ITS 2018. Protection levels and integrity monitoring (§4.4).

**Stereo, sync and cameras**
- Hirschmüller, *Stereo Processing by Semiglobal Matching and Mutual Information*, IEEE TPAMI 2008. The matcher behind `stereo_image_proc`'s SGBM.
- Furgale, Rehder & Siegwart, *Unified Temporal and Spatial Calibration for Multi-Sensor Systems*, IROS 2013 (Kalibr), and Li & Mourikis, *Online Temporal Calibration for Camera-IMU Systems*, IJRR 2014. Camera-IMU time offsets.
- Schubert et al., *Direct Sparse Odometry with Rolling Shutter*, ECCV 2018. What rolling shutter does to VO, and a fix.

**Monocular depth**
- Pizzoli, Forster & Scaramuzza, *REMODE: Probabilistic, Monocular Dense Reconstruction in Real Time*, ICRA 2014.
- Ranftl et al., *Towards Robust Monocular Depth Estimation* (MiDaS), IEEE TPAMI 2020, and Yang et al., *Depth Anything*, CVPR 2024. Learned monocular depth.

See also the state-estimation reading list: [vehicle-state-estimation-reading](vehicle-state-estimation-reading).
