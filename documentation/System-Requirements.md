# System Requirements

**Status:** Draft · **Owner:** Anthony Tracy

This document turns the competition rules in [RoboMagellan Requirements](RoboMagellan-Requirements) into numbered, verifiable requirements, and allocates each one to the architecture elements defined in [System Architecture](System-Architecture). Lower-level (derived) requirements are written just in time, before the element they apply to is designed.

## 1. Conventions

### 1.1 Requirement IDs

`REQ-<ELEMENT>-<NNN>`, where `<ELEMENT>` is the architecture element that owns the requirement:

| Prefix | Level | Element |
| --- | --- | --- |
| `REQ-SYS` | L0 | RC Racer System |
| `REQ-VEH`, `REQ-GND`, `REQ-TST` | L1 | Vehicle, Ground, Test & Simulation segments |
| `REQ-GNC`, `REQ-SIM`, ... | L2 | Subsystems (codes in [System Architecture](System-Architecture)) |

IDs are never reused. A deleted requirement keeps its ID with status **Deleted**.

### 1.2 Wording

| Word | Meaning |
| --- | --- |
| **shall** | Mandatory. Must be verified. |
| **should** | Goal. Desirable, not required for acceptance. |
| **will** | Statement of fact or intent, not a requirement. |

**TBD** = value not yet known. **TBR** = value proposed, to be reviewed. Every TBD/TBR is listed in [§6](#6-tbd--tbr-register).

### 1.3 Verification methods

| Code | Method | Example |
| --- | --- | --- |
| **T** | Test | Automated test in simulation or on hardware with pass/fail criteria |
| **D** | Demonstration | Operate the system and observe (e.g. field run) |
| **A** | Analysis | Calculation or model (e.g. battery energy budget) |
| **I** | Inspection | Examine the item or document (e.g. weigh the car) |

### 1.4 Traceability

Each requirement lists its **Source** (a parent requirement or a source document) and its **Allocation** (the elements responsible for meeting it). A requirement allocated to more than one element gets derived requirements on each.

---

## 2. System Requirements (L0)

Source: **RM** = [RoboMagellan Requirements](RoboMagellan-Requirements) / [RoboGames rules](https://robogames.net/rules/magellan.php). **SD** = [Software Design](Software-Design).

### 2.1 Mission

| ID | Requirement | Source | Ver. | Allocation |
| --- | --- | --- | --- | --- |
| REQ-SYS-001 | The system shall autonomously drive from a start point to a destination point, both given as WGS84 coordinates. | RM Objective | T, D | VEH.GNC, VEH.EXE |
| REQ-SYS-002 | The system shall touch every mandatory waypoint cone, in the specified order when an order is given. | RM Play Rules | T, D | VEH.EXE, VEH.GNC, VEH.PER |
| REQ-SYS-003 | The system shall be able to touch any bonus waypoint cone selected in the mission plan. | RM Car Starting Point | T, D | VEH.EXE, VEH.GNC, VEH.PER |
| REQ-SYS-004 | The system shall touch waypoint cones without displacing them by more than **TBD-001** m. | RM Play Rules | T, D | VEH.GNC, VEH.PER |
| REQ-SYS-005 | The system shall complete a course with a straight-line start-to-destination distance of up to 305 m (1,000 ft) and a total path length of up to **TBD-002** m within 15 minutes. | RM Travel Distance, Play Rules | T, A, D | VEH |
| REQ-SYS-006 | The system shall detect 18" orange traffic cones. | RM Car Starting Point | T, D | VEH.PER |
| REQ-SYS-007 | The system shall avoid static obstacles on the course. | RM Course Design | T, D | VEH.GNC, VEH.PER |
| REQ-SYS-008 | The system shall avoid dynamic obstacles, including other robots running the course at the same time. | RM Play Rules | T, D | VEH.PER, VEH.GNC |
| REQ-SYS-009 | The system shall operate on outdoor terrain including pavement, dirt, small rocks, grass, hills, gullies, curbs and weeds. | RM Obstacles and materials | T, D | VEH |
| REQ-SYS-010 | The system shall not enter course areas marked as non-traversable (e.g. gullies that could trap it). | RM Travel Distance | T, D | VEH.GNC |
| REQ-SYS-011 | The system should select which bonus waypoints to attempt so as to minimise the expected score. | RM Scoring | A, T | GND.MPL, VEH.EXE |

### 2.2 Inputs and operations

| ID | Requirement | Source | Ver. | Allocation |
| --- | --- | --- | --- | --- |
| REQ-SYS-020 | The system shall accept course coordinates in the WGS84 datum, given as degrees and decimal minutes to 4 decimal places. | RM Car Starting Point | T, I | GND.MPL, VEH.EXE |
| REQ-SYS-021 | The system shall accept a pre-surveyed course map before a run, containing the start, mandatory waypoints (with order), bonus waypoints (with multiplier), destination and obstacles (with traversability). | RM Play Rules | T, I | GND.MPL, VEH.EXE |
| REQ-SYS-022 | The system shall operate without human input during a run, except through the fail-safe device. | RM Car requirements | D | VEH |
| REQ-SYS-023 | The system shall provide a fail-safe device, operated by the handler, that stops the vehicle. | RM Car requirements | T, D | GND.OPS, VEH.FDR, VEH.ACT |
| REQ-SYS-024 | The system shall support three attempts per event, with a reset of vehicle state between attempts. | RM Play Rules | D | VEH.EXE, GND.OPS |
| REQ-SYS-025 | The system shall record its sensor data, state estimates and decisions during a run for post-run analysis. | SD Data requirements | I, D | VEH.TMC, GND.DAT |

### 2.3 Physical

| ID | Requirement | Source | Ver. | Allocation |
| --- | --- | --- | --- | --- |
| REQ-SYS-030 | The vehicle shall weigh less than 50 lb (22.7 kg). | RM Car requirements | I | VEH |
| REQ-SYS-031 | The vehicle shall fit within a 4 ft x 4 ft x 10 ft envelope. | RM Car requirements | I | VEH |
| REQ-SYS-032 | The vehicle's centre of mass shall be lower than 4 ft. | RM Car requirements | A, I | VEH |
| REQ-SYS-033 | Any part of the vehicle above 4 ft shall fit within a 6 in x 6 in x 6 ft column and shall not create a tipping or other hazard. | RM Car requirements | I, A | VEH |
| REQ-SYS-034 | The vehicle shall carry enough energy to drive the maximum course path length (REQ-SYS-005) with a margin of **TBR-001** %. | RM Travel Distance | A, T | VEH |

### 2.4 Development constraints

| ID | Requirement | Source | Ver. | Allocation |
| --- | --- | --- | --- | --- |
| REQ-SYS-040 | Every vehicle software capability shall be verified in simulation before it is run on hardware. | SDP §1 (simulation-first) | I | TST |
| REQ-SYS-041 | The vehicle software shall support more than one robot motion model (Ackermann/bicycle and differential/unicycle), selected by configuration. | SD State Estimation | I, T | VEH.GNC |

---

## 3. Segment Requirements (L1)

### 3.1 Test & Simulation Segment (TST)

| ID | Requirement | Source | Ver. | Allocation | Status |
| --- | --- | --- | --- | --- | --- |
| REQ-TST-001 | The simulator shall provide every interface that the vehicle's sensing hardware and actuation hardware provide, as defined in [ICD-VEH-TST](ICD-VEH-TST), so vehicle software runs unchanged against it. | REQ-SYS-040 | T | TST.SIM | ✅ Met for IMU, GPS, camera, wheel odometry, drive command (open deviations in ICD-VEH-TST §5) |
| REQ-TST-002 | The simulator shall generate a course world from a course file, with cones and obstacles at the file's WGS84 coordinates. | REQ-SYS-040, REQ-SYS-021 | T | TST.SIM | ✅ Met (`test_course_world`) |
| REQ-TST-003 | The simulator shall provide ground-truth vehicle pose for verifying vehicle software. | REQ-SYS-040 | T | TST.SIM | ✅ Met (Gazebo model pose) |
| REQ-TST-004 | Simulation tests shall run headless without a GPU, nightly in CI and locally with one command. | REQ-SYS-040 | T, I | TST.HAR | ✅ Met (nightly, `run-system-tests.sh`) |
| REQ-TST-005 | Simulated sensors shall carry realistic noise, and each message's covariance shall match the noise actually applied. | REQ-SYS-040, REQ-GNC-006 | T | TST.SIM | ✅ Met (sensor adapter, `TestSensorNoise`) |

### 3.2 Ground Segment (GND)

The ground segment is a **placeholder**: it is defined so its interfaces and allocations are visible, but no ground software is planned yet. Course files are currently hand-written YAML ([SDP §4.4](Software-Development-Plan)).

### 3.3 Vehicle Segment (VEH)

Vehicle segment requirements are derived per subsystem (§4).

---

## 4. Subsystem Requirements (L2)

Written just in time, before each subsystem's components are designed.

### 4.1 GNC - Guidance, Navigation and Control

| ID | Requirement | Source | Ver. | Allocation |
| --- | --- | --- | --- | --- |
| REQ-GNC-001 | GNC shall estimate the vehicle's pose (position, heading) and velocity in the `odom` frame at ≥ 50 Hz. | REQ-SYS-001 | T | VEH.GNC.SE |
| REQ-GNC-002 | The `odom`-frame estimate shall be continuous (no jumps) and drift by no more than **TBD-003** % of distance travelled. | REQ-SYS-001, REQ-SYS-005 | T | VEH.GNC.SE |
| REQ-GNC-003 | GNC shall estimate the vehicle's WGS84 position to within **TBD-004** m (95%) for navigation between waypoints. | REQ-SYS-001, REQ-SYS-002 | T | VEH.GNC.LOC |
| REQ-GNC-004 | GNC shall keep estimating pose through GNSS outages of up to **TBD-005** s. *(Deferred: no GNSS receiver, ISSUE-001)* | REQ-SYS-001 | T | VEH.GNC.SE, VEH.GNC.LOC |
| REQ-GNC-005 | GNC shall detect wheel slip and reduce its trust in wheel odometry while slip is detected. | RM "Take into account slip" | T | VEH.GNC.SE |
| REQ-GNC-006 | GNC shall report the covariance of every estimate it publishes. | Architecture Conventions | T, I | VEH.GNC |
| REQ-GNC-007 | GNC shall select its motion model (bicycle or unicycle) from configuration, without code changes. | REQ-SYS-041 | T | VEH.GNC.SE |

---

## 5. Open Issues

| ID | Issue | Affects | Proposed resolution |
| --- | --- | --- | --- |
| ~~ISSUE-001~~ | The overview diagram marks the **GPS node as "Work Not Planned"** and `Rc-Car-Design.md` lists no GPS hardware, but every RoboMagellan waypoint is given in GPS coordinates (REQ-SYS-001, -020) and the state estimation design subscribes to GPS. | REQ-SYS-001, REQ-GNC-003, VEH.SNS.GPS | ✅ **Decided 2026-11-16 (owner).** Waypoints **are** handled as WGS84 coordinates: `localization` converts them to the `map` frame, anchored at the known course start point (REQ-SYS-020). There is **no onboard GNSS receiver** for now: global position comes from dead reckoning from the start point, corrected by camera sightings of landmarks with known positions (waypoint cones, arena edges, other surveyed features). A GNSS receiver and `gps_driver` are a **future improvement** (VEH.SNS.GPS). REQ-GNC-004 is deferred until then. |
| ~~ISSUE-002~~ | Whether `state_estimation` fuses GPS (as in its class diagram) or only IMU + wheel odometry, leaving GPS to `localization` (the REP-105 `odom`/`map` split). | VEH.GNC.SE, VEH.GNC.LOC | ✅ **Decided 2026-11-16 (owner): option (b).** `state_estimation` estimates the vehicle's own motion (IMU + wheel odometry, `odom` frame) and never uses world-frame data. `localization` owns the vehicle's pose in the `map` / WGS84 frame and is where GNSS would be fused. See [state_estimation](state_estimation) Open Item 1. |

## 6. TBD / TBR Register

| ID | Requirement | What is needed | How it will be resolved |
| --- | --- | --- | --- |
| TBD-001 | REQ-SYS-004 | Maximum allowed cone displacement on contact | Check event judging practice; test cone push in sim |
| TBD-002 | REQ-SYS-005 | Maximum total path length | Estimate from past course maps (e.g. 2-3x straight-line) |
| TBD-003 | REQ-GNC-002 | Allowed odometry drift. **Proposed: 2 % (TBR)** | Sim L2 test measures 0.13 % over ~100 m; 2 % leaves margin for real tyres, terrain and encoder calibration (1 % scale error assumed). Confirm on hardware (Phase 5) |
| TBD-004 | REQ-GNC-003 | Required global position accuracy | No GNSS (ISSUE-001): set from dead-reckoning drift (TBD-003) over the course length (REQ-SYS-005) and the camera's landmark detection range |
| TBD-005 | REQ-GNC-004 | Longest GNSS outage to ride through | Deferred with REQ-GNC-004 until a GNSS receiver is added (ISSUE-001) |
| TBR-001 | REQ-SYS-034 | Battery energy margin (proposed 30%) | Battery stress test |
