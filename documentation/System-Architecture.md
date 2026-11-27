# System Architecture

**Status:** Draft · **Owner:** Anthony Tracy

This document breaks the RC Racer system down into a tree of elements, the same way a satellite program splits into a ground segment and a space segment and then into subsystems and components. The tree is used for three things:

1. **Interfaces** - it tells us which elements talk to each other, and so which Interface Control Documents (ICDs) must exist.
2. **Requirements** - every requirement in [System Requirements](System-Requirements) is allocated to elements in this tree.
3. **Verification** - each level is verified on its own before it is combined with the level above.

Related: [Architecture Conventions](Architecture-Conventions) (frames, units, naming, QoS) · [Mission Orchestration](Mission-Orchestration) (the Executive subsystem) · [Software Design](Software-Design) (original design notes and diagrams).

## 1. Levels

| Level | Name | What it is | RC Racer example | Verified by |
| --- | --- | --- | --- | --- |
| **L0** | System | The whole entrant | RC Racer | Field test at a practice site |
| **L1** | Segment | A major part, often on its own hardware | Vehicle, Ground, Test & Simulation | System tests (nightly, simulator) |
| **L2** | Subsystem | A group of components with one job | GNC, Perception, Actuation | Subsystem integration tests (simulator + stubs) |
| **L3** | Component | One ROS 2 package / node | `state_estimation` | Component integration tests (`launch_testing`, stub neighbours) |
| **L4** | Unit | A logic class inside a component ([SDP §2](Software-Development-Plan)) | `StateEstimationLogic` | Unit tests (no ROS) |

### 1.1 Element IDs

Elements are identified by their path in the tree, with a short code per level:

```
VEH.GNC.SE      = Vehicle segment / GNC subsystem / state_estimation component
```

Codes are used in requirement allocations, ICD names and test names.

---

## 2. Product Breakdown Tree

Status: ✅ exists · 🟨 stub package (structure only, no code) · ⏳ planned · ▫️ placeholder / future improvement

```
L0  RC Racer System
│
├── L1  VEH  Vehicle Segment                   ("flight software" - Jetson Nano, on the car)
│   ├── L2  SNS  Sensing                       turns hardware into ROS data
│   │   ├── L3  IMU  imu_driver                ⏳ Phase 5   Adafruit BNO085
│   │   ├── L3  GPS  gps_driver                ▫️ future    GNSS receiver, not in HW list (ISSUE-001)
│   │   ├── L3  CAM  camera_driver             ⏳ Phase 5   stereo cameras
│   │   └── L3  ENC  encoder_driver            ⏳ Phase 5   drive motor encoder (128 PPR)
│   ├── L2  PER  Perception
│   │   └── L3  OD   object_detection          🟨   cones, obstacles, other robots
│   ├── L2  GNC  Guidance, Navigation & Control
│   │   ├── L3  SE   state_estimation          🟨   fused pose/velocity, odom frame
│   │   ├── L3  LOC  localization              🟨   global pose, map frame / WGS84
│   │   ├── L3  MAP  mapping                   🟨   2D weighted obstacle/terrain map
│   │   └── L3  PLN  planning_control          🟨   path planning + path following
│   ├── L2  ACT  Actuation
│   │   └── L3  MC   motor_controller          🟨   ESC + steering servo via PCA9685
│   ├── L2  EXE  Executive
│   │   └── L3  MX   mission_executive         ⏳   behavior-tree mission runner
│   ├── L2  FDR  Safety / FDIR                 (fault detection, isolation, recovery)
│   │   └── L3  SAF  safety                    🟨   watchdogs, e-stop, limits
│   ├── L2  TMC  Telemetry & Command
│   │   └── L3  TLM  telemetry                 🟨   logging, telemetry to ground, ground commands
│   └── L2  PLT  Platform
│       └── L3  BRG  rc_racer_bringup          ⏳   launch files, configuration, lifecycle manager
│
├── L1  GND  Ground Segment                    ▫️ placeholder ("ground software" - laptop / home lab / back end)
│   ├── L2  MPL  Mission Planning              ▫️   course files from site survey, mission trees
│   ├── L2  OPS  Mission Operations            ▫️   live monitoring, commands, fail-safe / e-stop
│   └── L2  DAT  Data Processing               ▫️   logs, rosbag replay, post-run analysis
│
└── L1  TST  Test & Simulation Segment         ("simulator and test equipment")
    ├── L2  SIM  Vehicle Simulator
    │   └── L3  SIMPKG  rc_racer_simulation    ✅   robot model, sensors + sensor adapter, worlds, course generator
    └── L2  HAR  Test Harness
        ├── L3  ITEST   rc_racer_integration_tests  🟨
        ├── L3  STEST   rc_racer_system_tests       ✅   smoke + course world tests
        └── L3  CI      CI/nightly workflows, run-system-tests.sh  ✅

Shared (owned at L0):
    rc_racer_interfaces   ⏳   custom msg/srv/action types that implement the ICDs
```

### 2.1 Responsibilities

| Element | Responsible for | Not responsible for |
| --- | --- | --- |
| **VEH.SNS** | Reading sensors and publishing calibrated data with timestamps and frames | Filtering, fusion, interpretation |
| **VEH.PER** | Detecting and classifying objects (cones, obstacles, robots) from sensor data | Deciding what to do about them |
| **VEH.GNC** | Knowing where the car is (navigation), deciding the path (guidance) and following it (control) | Mission sequencing - which waypoint is next |
| **VEH.ACT** | Turning drive commands into ESC/servo signals; enforcing hardware limits | Choosing the command |
| **VEH.EXE** | Running the mission: which goal next, retries, giving up on bonus cones | How to drive to a goal |
| **VEH.FDR** | Monitoring health of all elements; stopping the car on faults or e-stop. Can override EXE and GNC | Recovering the mission (EXE does that) |
| **VEH.TMC** | Recording data on the car; telemetry to ground; passing ground commands in | Acting on commands itself |
| **VEH.PLT** | Starting, configuring and managing the lifecycle of all vehicle nodes | Any runtime logic |
| **GND** | Preparing missions before a run, watching and stopping the car during it, analysing after | Anything the car must do on its own (REQ-SYS-022) |
| **TST** | Standing in for the real world (simulator) and checking the system against requirements (harness) | Being part of the delivered system |

---

## 3. Interfaces

### 3.1 The ICD rule

> **Every interface between two siblings has exactly one ICD, owned by their nearest common parent.**
> A component reads only the ICDs it is party to.

ICDs are named `ICD-<A>-<B>` after the two elements, in data-flow order. An ICD between two segments (L1) is a system-level document; an ICD between two GNC components (L3) belongs to GNC. Each interface is drawn once, at the lowest level where both sides are visible.

### 3.2 L0 - External interfaces

| Interface | Between | Nature |
| --- | --- | --- |
| Course hand-out | Competition → GND.MPL | Start/waypoint coordinates (WGS84, degrees + decimal minutes) |
| Course environment | Environment ↔ VEH | Terrain, cones, obstacles, GNSS signals, light |
| Handler | Operator ↔ GND.OPS | Fail-safe device, start/stop |

### 3.3 L1 - Between segments

N² chart: row element **provides** to column element.

| from \ to | VEH | GND | TST |
| --- | --- | --- | --- |
| **VEH** | ■ | telemetry, logs (ICD-VEH-GND) | drive commands (ICD-VEH-TST) |
| **GND** | course file, mission, commands, e-stop (ICD-VEH-GND) | ■ | course files (ICD-VEH-TST) |
| **TST** | simulated sensor data, ground truth (ICD-VEH-TST) | | ■ |

`TST` substitutes for the environment and the sensing/actuation hardware. [ICD-VEH-TST](ICD-VEH-TST) therefore says *which vehicle-internal ICDs the simulator implements*, so vehicle software cannot tell the difference.

### 3.4 L2 - Inside the Vehicle segment

| from \ to | SNS | PER | GNC | ACT | EXE | FDR | TMC |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **SNS** | ■ | images (SNS-PER) | IMU, GNSS, wheel odom (SNS-GNC) | | | health | sensor data |
| **PER** | | ■ | detections (PER-GNC) | | cone found (PER-EXE) | health | detections |
| **GNC** | | | ■ | drive command (GNC-ACT) | action results (EXE-GNC) | health | state, path |
| **ACT** | | | | ■ | | health | actuator state |
| **EXE** | | detect requests (PER-EXE) | goals as actions (EXE-GNC) | | ■ | health | mission status |
| **FDR** | | | | e-stop (FDR-ACT) | abort (FDR-EXE) | ■ | faults |
| **TMC** | | | | | ground commands (TMC-EXE) | ground e-stop (TMC-FDR) | ■ |

Health and telemetry flows (the FDR and TMC columns) are common to every element and are covered once by [Architecture Conventions](Architecture-Conventions) (`/diagnostics`, recording), not by separate ICDs. PLT starts and configures everything and has no runtime data interfaces.

### 3.5 L3 - Inside subsystems

Drawn in each subsystem's design when its components are designed. GNC (planned):

| from \ to | SE | LOC | MAP | PLN |
| --- | --- | --- | --- | --- |
| **SE** | ■ | `odom` pose/velocity | | `odom` pose/velocity |
| **LOC** | | ■ | `map` pose | `map` pose |
| **MAP** | | | ■ | costmap |
| **PLN** | | | | ■ |

### 3.6 ICD register

| ICD | Level | Between | Status | Needed for |
| --- | --- | --- | --- | --- |
| [ICD-VEH-TST](ICD-VEH-TST) | L1 | Vehicle ↔ Simulator | Draft | everything simulated |
| ICD-VEH-GND | L1 | Vehicle ↔ Ground | Not started | Ground segment (placeholder) |
| [ICD-SNS-GNC](ICD-SNS-GNC) | L2 | Sensing → GNC | Draft | `state_estimation` |
| [ICD-GNC-ACT](ICD-GNC-ACT) | L2 | GNC → Actuation | Draft | `planning_control`, `motor_controller` |
| [ICD-SNS-PER](ICD-SNS-PER) | L2 | Sensing → Perception | Draft | `object_detection` |
| [ICD-PER-GNC](ICD-PER-GNC) | L2 | Perception → GNC | Draft | `localization`, `mapping`, cone approach |
| ICD-EXE-GNC | L2 | Executive ↔ GNC | Not started | `mission_executive` (see [Mission Orchestration](Mission-Orchestration)) |
| ICD-PER-EXE | L2 | Perception ↔ Executive | Not started | `mission_executive` |
| ICD-FDR-ACT | L2 | Safety → Actuation | Not started | `safety` |
| ICD-FDR-EXE | L2 | Safety → Executive | Not started | `safety` |
| ICD-TMC-EXE, ICD-TMC-FDR | L2 | Telemetry → Executive / Safety | Not started | Ground segment |

Templates: [ICD Template](ICD-Template) · [Component Template](Component-Template).

---

## 4. Verification Approach

Each level is verified **against its own ICDs and requirements**, with its neighbours replaced. That is what "isolated" means in practice: an element can be proven correct without the real elements around it.

| Level | Test type | Neighbours replaced by | Where |
| --- | --- | --- | --- |
| L4 unit | Unit tests, no ROS | Plain test data | `src/<pkg>/test/` |
| L3 component | Integration (`launch_testing`) | Stub publishers/subscribers that follow the ICD | `test/integration/` |
| L2 subsystem | Subsystem integration | Simulator for SNS/ACT; stubs for other subsystems | `test/integration/`, `test/system/` |
| L1 segment | System tests | TST segment in place of the world | `test/system/` (nightly) |
| L0 system | Field test | Nothing | Practice site |

When an integration test fails, the tree points at the interface that broke, and the ICD says which side is wrong.

## 5. Process

1. Before a component is coded, its [component spec](Component-Template) and every ICD it is party to are at least **Draft**, and its derived requirements are in [System Requirements](System-Requirements).
2. An ICD change needs agreement from both sides and an update to `rc_racer_interfaces` where custom types are involved.
3. Tests reference requirement IDs (e.g. `REQ-GNC-001`) in their docstrings so coverage can be traced.
