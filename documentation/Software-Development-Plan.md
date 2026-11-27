# Software Development Plan

## RC Racer - RoboMagellan Competition

---

## 1. Project Overview

This document outlines the software development plan for the RC Racer autonomous vehicle project. The system is built on ROS 2 Humble with a modular microservices architecture.

**Development Environment:** All development, building, and testing is done inside Docker containers for reproducibility and isolation.

**Development Approach:** Simulation-first development - build and validate in Gazebo before deploying to hardware.

### 1.1 Development Roadmap

> **Current Phase: Perception (Phase 2)**

| Phase                    | Status         | Description                                  |
| ------------------------ | -------------- | -------------------------------------------- |
| **Phase 1: Simulation**  | ✅ Complete    | Gazebo world, robot model, sensor simulation (completed 2026-11-15, §4.7) |
| **Phase 2: Perception**  | 🚧 In Progress | Object detection, state estimation           |
| **Phase 3: Navigation**  | ⏳ Planned     | Localization, mapping, path planning         |
| **Phase 4: Control**     | ⏳ Planned     | Motor control, safety systems (first: fix sim steering lag, §4.2) |
| **Phase 5: Integration** | ⏳ Planned     | Full system integration, hardware deployment |

**Features.** Each phase is split into numbered features, `<phase>.<feature>`. A feature is one component or one piece of work that is developed on one branch and merged when it is done. Features are numbered in the order they are planned. Add new ones at the end of their phase, and never renumber a feature once a branch uses its number.

**Status:** ⏳ Planned → 🚧 In progress (on its branch) → 🔍 In validation (merged, but some of its [`manual-validation`](https://github.com/gotbordom/rc-racer/issues?q=is%3Aopen+label%3Amanual-validation) issues are still open, §3.6) → ✅ Done (merged and every manual check closed).

| Feature | Name | Element | Status |
| --- | --- | --- | --- |
| 2.1 | `state-estimation` | VEH.GNC.SE | 🔍 In validation: merged 2026-11-23 (PR #1); manual checks open: #2–#7 |
| 2.2 | `object-detection` | VEH.PER.OD | 🚧 In progress: orange cone and obstacle detection from the camera (spec: `documentation/components/object_detection.md`) |
| 2.3 | `validation-scenarios` | TST | ✅ Done: merged 2026-11-24 (PR #10). Validation cases automated (§3.6); the bugs it found are fixed in 2.4 (#9, #14) or planned in 2.5 (#11) |
| 2.4 | `slip-handling` | VEH.GNC.SE, TST (DEV-07) | ✅ Done: merged 2026-11-24 (PR #15). Sim IMU sees impacts; VAL-006, -017, -019 pass, VAL-004 restored; VAL-018 moves to 2.5 |
| 2.5 | `lateral-velocity` | VEH.GNC.SE | ✅ Done: merged 2026-11-27 (PR #16). Sideways velocity in the state, no-slip pseudo-measurement relaxed in slides, forward speed from the turn; VAL-018 error halved but still expected-fail (#11, spec Open Item 7) |
| 3.1 | `localization` | VEH.GNC.LOC | ⏳ Planned (spec to be written first) |
| 3.2 | `mapping` | VEH.GNC.MAP | ⏳ Planned |
| 3.3 | `planning-control` | VEH.GNC.PLN | ⏳ Planned |
| 4.1 | `sim-steering-lag` | TST (DEV-04) | ⏳ Planned |
| 4.2 | `motor-controller` | VEH.ACT.MC | ⏳ Planned |
| 4.3 | `safety` | VEH.FDR.SAF | ⏳ Planned |

Phase 1 was completed before features were numbered. Its branch, `feature/simulation`, keeps its name. Phase 5 features will be numbered when Phase 5 is planned in detail.

**Branch names.** A feature branch is named `feature/<phase>.<feature>-<name>`, using the number and name from the table, e.g. `feature/2.1-state-estimation`. Merged feature branches are kept for later review.

**Commits.** Keep commits small: one change each, so any one can be reviewed, reverted or bisected on its own.

**Decisions go in the commit message.** Whenever a commit carries a choice between approaches (a design, a fix, how to record a finding, a requirement restated), its message records:
1. **the problem**, with the evidence (numbers from the run, the trace);
2. **the options** that were considered, including the rejected ones and why;
3. **which was chosen, by whom and when**, e.g. "owner, 2026-11-24";
4. **the result**, measured, or "next commit" when it comes later.

Use the headings *The problem*, *The decision*, *Changes* or *Result*, as in VAL-005/017 (f3d6090), the slip-check redesign (887d4ce) or the turn measurement (fb16ee4). `git log --grep="The decision"` then lists every choice made in the project, with its reasons. A choice in a spec or the register points to the commit that records it.

### 1.2 Package Structure

```
src/
├── rc_racer_simulation/ # Gazebo world, robot model, sensors (Phase 1)
├── state_estimation/    # Sensor fusion and state tracking
├── localization/        # map/WGS84 pose: dead reckoning + camera landmark fixes (GNSS later)
├── mapping/             # 2D weighted obstacle/terrain mapping
├── object_detection/    # Orange cone and obstacle detection
├── planning_control/    # Path planning and navigation
├── motor_controller/    # Low-level motor control
├── safety/              # Safety monitoring and e-stop
└── telemetry/           # Data logging and monitoring
```

Each package follows the standard ROS 2 structure:

```
src/<package>/
├── src/                 # Source files (.cpp)
├── include/<package>/   # Header files (.hpp)
├── test/                # Unit and integration tests
├── CMakeLists.txt       # Build configuration
└── package.xml          # Package manifest
```

### 1.3 License

**License:** MIT License

**SPDX Identifier:** `MIT`

All source files must include a license header using the SPDX format:

```python
# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT
```

```cpp
// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT
```

**Why SPDX format?**

- Machine-readable standard for license identification
- Concise (2 lines vs. 20+ line license blocks)
- Widely recognized by license scanning tools
- Avoids verbose license text in every file

**Full license text:** See `LICENSE` file in repository root.

> **Note:** The `ament_copyright` linter doesn't recognize SPDX format, so we use specific linters
> (`ament_cmake_flake8`, `ament_cmake_pep257`, `ament_cmake_xmllint`) instead of `ament_lint_common`
> in test packages.

### 1.4 Architecture and Interface Documents

The system is broken down like a satellite program (System → Segments → Subsystems → Components → Units), with an Interface Control Document (ICD) for each interface between siblings:

| Document | Contains |
| --- | --- |
| [System Requirements](System-Requirements) | Numbered requirements (`REQ-...`), verification method, allocation, TBD register, open issues |
| [System Architecture](System-Architecture) | Product breakdown tree, element IDs, N² interface charts, ICD register, verification levels |
| [Architecture Conventions](Architecture-Conventions) | Units, reference frames and the **TF registry** (REP-103/105), naming, message types, QoS, lifecycle, health |
| [Mission Orchestration](Mission-Orchestration) | Executive subsystem: behavior-tree missions and the capability catalogue |
| `documentation/icd/` | ICDs, e.g. [ICD-SNS-GNC](ICD-SNS-GNC), [ICD-GNC-ACT](ICD-GNC-ACT), [ICD-VEH-TST](ICD-VEH-TST); [ICD Template](ICD-Template) |
| `documentation/components/` | Component specs, e.g. [state_estimation](state_estimation); [Component Template](Component-Template) |

**Definition of Ready** - before a component's code is written:

1. Its component spec exists (status Draft or later).
2. Every ICD it is party to exists (Draft or later).
3. Its derived requirements are in System Requirements.
4. Any simulator deviations it depends on ([ICD-VEH-TST §5](ICD-VEH-TST)) are fixed.

Tests reference requirement IDs (e.g. `REQ-GNC-001`) in their docstrings.

---

## 2. Critical Design Rule: Testable Nodes

> **This rule saves years of pain. Follow it religiously.**

### 2.1 The Principle

**Nodes should be testable without ROS spinning.**

This means:

| Do                                      | Don't                                  |
| --------------------------------------- | -------------------------------------- |
| Put logic in plain C++ / Python classes | Put logic directly in callbacks        |
| ROS node is a thin adapter              | ROS node contains business logic       |
| Test the class, not the node            | Test by spinning up ROS infrastructure |

### 2.2 Architecture Pattern

```
┌─────────────────────────────────────────────────────────┐
│                     ROS 2 Node                          │
│  (thin adapter: subscriptions, publishers, parameters)  │
├─────────────────────────────────────────────────────────┤
│                    Logic Class                          │
│  (pure C++/Python: algorithms, state, computations)     │
└─────────────────────────────────────────────────────────┘
```

### 2.3 Package Structure (Enforcing the Pattern)

```
src/<package>/
├── include/<package>/
│   ├── <package>_node.hpp      # ROS node (thin adapter)
│   └── <package>_logic.hpp     # Pure logic class (no ROS deps)
├── src/
│   ├── <package>_node.cpp      # Wires ROS to logic class
│   └── <package>_logic.cpp     # All the real work
└── test/
    ├── test_<package>_logic.cpp  # Tests logic WITHOUT ROS
    └── test_<package>_node.cpp   # (Optional) Integration test with ROS
```

### 2.4 Example: State Estimation

**Bad (untestable):**

```cpp
class StateEstimationNode : public rclcpp::Node {
  void imu_callback(const Imu::SharedPtr msg) {
    // 200 lines of Kalman filter math here
    // Can only test by spinning up ROS
  }
};
```

**Good (testable):**

```cpp
// state_estimation_logic.hpp - NO ROS INCLUDES
class StateEstimationLogic {
public:
  State update(const ImuData& imu);  // Pure function
  State predict(double dt);
private:
  KalmanFilter filter_;
};

// state_estimation_node.cpp - Thin adapter
class StateEstimationNode : public rclcpp::Node {
  void imu_callback(const Imu::SharedPtr msg) {
    ImuData data = convert(msg);      // Convert ROS → plain struct
    State state = logic_.update(data); // Delegate to logic
    publisher_->publish(convert(state)); // Convert back to ROS
  }
  StateEstimationLogic logic_;
};
```

**Test (no ROS required):**

```cpp
TEST(StateEstimationLogicTest, UpdateWithImu) {
  StateEstimationLogic logic;
  ImuData imu{.accel = {0, 0, 9.81}, .gyro = {0, 0, 0}};

  State result = logic.update(imu);

  EXPECT_NEAR(result.orientation.w, 1.0, 0.01);
}
```

### 2.5 Benefits

- **Fast tests:** No ROS executor, no DDS, milliseconds not seconds
- **Deterministic:** No timing issues, no flaky tests
- **Debuggable:** Step through logic in debugger without ROS noise
- **Reusable:** Logic can be used outside ROS (simulation, other frameworks)
- **Reviewable:** Separation makes code easier to understand

---

## 3. Testing Strategy

### 3.1 Test Pyramid

```
┌─────────────────────────────┐
│ System / End-to-End Tests   │  ← few, slow, high confidence
├─────────────────────────────┤
│ Integration Tests           │  ← some, medium cost
├─────────────────────────────┤
│ Unit / Node-local Tests     │  ← many, fast
└─────────────────────────────┘
```

### 3.2 Test Mapping by Package

| Bucket          | Package / Component                     | What We Test                                     |
| --------------- | --------------------------------------- | ------------------------------------------------ |
| **Unit**        | `state_estimation`                      | EKF math, covariance updates, prediction step    |
|                 | `planning_control`                      | Path planner logic, trajectory generation        |
|                 | `motor_controller`                      | Controller stability, PID tuning, limit clamping |
|                 | `localization`                          | Coordinate transforms, map lookups               |
|                 | `object_detection`                      | Detection algorithms, clustering logic           |
|                 | `safety`                                | State machine transitions, timeout logic         |
| **Integration** | `state_estimation` ↔ `localization`     | EKF output flows to localization, topics exist   |
|                 | `localization` → `planning_control`     | Pose updates trigger replanning                  |
|                 | `planning_control` → `motor_controller` | Commands propagate, timing acceptable            |
|                 | `safety` ↔ all nodes                    | E-stop propagates, watchdog triggers             |
| **System**      | Full stack                              | Complete sim lap, cone navigation                |
|                 | Safety subsystem                        | E-stop behavior, graceful degradation            |
|                 | Sensor pipeline                         | Sensor dropout handling, recovery                |

### 3.3 Unit Tests (Node-Local)

**Location:** `src/<package>/test/`

**Run Command:**

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

**Scope - What Unit Tests Cover:**

- Pure logic functions
- Mathematical computations
- State machine transitions
- Parameter validation
- Callback behavior (with mocks)

**Scope - What Unit Tests Do NOT Cover:**

- DDS communication
- Node discovery
- Timing between nodes
- Hardware interfaces

**Frameworks:**

- **C++:** `ament_cmake_gtest` (Google Test)
- **Python:** `ament_cmake_pytest`

**Testing Utilities:**

- Mock publishers/subscribers
- Fake clocks (`rclcpp::Clock` with `RCL_ROS_TIME`)
- Parameter injection

**Example GTest Setup (CMakeLists.txt):**

```cmake
ament_add_gtest(test_my_logic test/test_my_logic.cpp)
target_include_directories(test_my_logic PRIVATE include)
ament_target_dependencies(test_my_logic rclcpp std_msgs)
```

**Example pytest Setup (CMakeLists.txt):**

```cmake
ament_add_pytest_test(test_my_logic_py test/test_my_logic.py)
```

### 3.4 Integration Tests

**Purpose:** Test ROS graphs (not individual nodes) with real DDS communication.

**Location:** `test/integration/` (project root, separate package)

**Package:** `rc_racer_integration_tests`

**Run Command:**

```bash
colcon test --packages-select rc_racer_integration_tests
colcon test-result --verbose
```

#### 3.4.1 What Integration Tests Do

Integration tests launch **multiple real nodes** with **real DDS** to verify:

| Assertion Type          | Description                                       |
| ----------------------- | ------------------------------------------------- |
| **Topics Appear**       | Expected topics exist on the ROS graph            |
| **Messages are Sane**   | Published messages have valid data, correct types |
| **State Converges**     | System reaches expected state within timeout      |
| **Dead Nodes Detected** | Node failures trigger appropriate responses       |

#### 3.4.2 What Integration Tests Do NOT Do

- Test individual node logic (use unit tests)
- Test full end-to-end scenarios (use system tests)
- Test hardware interfaces directly

#### 3.4.3 Framework

- **`launch_testing`** - Launch nodes and run assertions
- **`launch_testing_ros`** - ROS 2 specific test utilities
- **`launch_testing_ament_cmake`** - CMake integration

#### 3.4.4 Integration Test Structure

```
test/integration/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── test_sensor_pipeline.launch.py
│   ├── test_planning_pipeline.launch.py
│   ├── test_safety_system.launch.py
│   └── test_full_graph_startup.launch.py
└── config/
    └── test_params.yaml
```

#### 3.4.5 Planned Integration Tests

| Test Name                 | Nodes Under Test                                   | Assertions                                           |
| ------------------------- | -------------------------------------------------- | ---------------------------------------------------- |
| `test_sensor_pipeline`    | state_estimation ↔ localization                    | Data flows, topics exist, messages valid             |
| `test_planning_pipeline`  | localization → planning_control → motor_controller | Commands propagate, timing acceptable                |
| `test_safety_system`      | safety + all critical nodes                        | E-stop triggers, watchdog works, node death detected |
| `test_full_graph_startup` | All nodes                                          | All nodes launch, no crashes, all topics exist       |

#### 3.4.6 Example CMakeLists.txt Entry

```cmake
find_package(launch_testing_ament_cmake REQUIRED)

add_launch_test(
  launch/test_sensor_pipeline.launch.py
  TARGET test_sensor_pipeline
  TIMEOUT 60
)
```

#### 3.4.7 Launch Test File Structure

Each launch test file (`*.launch.py`) contains:

1. **`generate_test_description()`** - Launches nodes under test
2. **Test Classes** - unittest.TestCase classes with assertions
3. **Post-shutdown Tests** - Verify clean exit codes

```python
# Example structure (see test/integration/launch/test_template.launch.py.example)

@launch_testing.markers.keep_alive
def generate_test_description():
    node = launch_ros.actions.Node(package='...', executable='...')
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])

class TestTopicsExist(unittest.TestCase):
    def test_topic_exists(self):
        # Assert topics appear on graph
        pass

class TestMessageFlow(unittest.TestCase):
    def test_messages_received(self):
        # Subscribe and verify messages arrive
        pass

class TestStateConvergence(unittest.TestCase):
    def test_state_converges(self):
        # Monitor state, assert convergence within timeout
        pass

class TestNodeHealth(unittest.TestCase):
    def test_dead_node_detected(self):
        # Kill node, verify detection
        pass

@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
```

### 3.5 System / End-to-End Tests

**Purpose:** Full system validation in simulation with realistic sensors.

**Location:** `test/system/` (project root, separate package)

**Package:** `rc_racer_system_tests`

#### 3.5.1 Characteristics

| Property      | Value                                            |
| ------------- | ------------------------------------------------ |
| **Speed**     | SLOW (minutes per test)                          |
| **Stability** | BRITTLE (sensitive to timing, environment)       |
| **Value**     | EXTREMELY VALUABLE (catches real-world failures) |

#### 3.5.2 What System Tests Validate

- **Startup order** - Nodes initialize in correct dependency order
- **Failure modes** - System handles crashes and failures gracefully
- **Safety behavior** - E-stop, watchdogs, and safety limits work correctly
- **Real-time-ish constraints** - Control loops and sensor pipelines meet timing requirements

#### 3.5.3 Test Files

```
test/system/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── test_full_stack_sim.launch.py   # Complete Gazebo simulation
│   ├── test_emergency_stop.launch.py   # E-stop and safety behavior
│   └── test_startup_shutdown.py        # Init/cleanup sequences
├── config/                              # Test parameters
├── worlds/                              # Gazebo world files
└── docker/
    ├── Dockerfile                       # Test environment image
    └── docker-compose.yml               # Orchestrated test runs
```

#### 3.5.4 System Test Descriptions

| Test                    | Validates                                                     | Timeout |
| ----------------------- | ------------------------------------------------------------- | ------- |
| `test_full_stack_sim`   | Full system in Gazebo, realistic sensors, navigation pipeline | 5 min   |
| `test_emergency_stop`   | E-stop triggers, system halts safely, watchdog works          | 2 min   |
| `test_startup_shutdown` | Startup order, clean shutdown, no zombies/leaks               | 2 min   |

#### 3.5.5 When CI/CD Runs System Tests

| Trigger                        | Tests Run                           |
| ------------------------------ | ----------------------------------- |
| **Nightly builds**             | All system tests                    |
| **Pre-release**                | All system tests (required to pass) |
| **Before hardware deployment** | All system tests + manual review    |

#### 3.5.6 Running System Tests

**Local (with ROS 2 + Gazebo installed):**

```bash
colcon test --packages-select rc_racer_system_tests
colcon test-result --verbose
```

**From the host (preferred - same steps as nightly CI):**

```bash
./scripts/run-system-tests.sh                        # all system tests
./scripts/run-system-tests.sh test_simulation_smoke  # tests matching a regex
```

**CI/CD Pipeline:**

```bash
docker compose up --abort-on-container-exit --exit-code-from system-tests
```

#### 3.5.7 Docker Test Environment

The system tests run in an isolated Docker environment to ensure reproducibility:

- **Base image:** `osrf/ros:humble-desktop` (includes Gazebo)
- **Isolation:** Dedicated `ROS_DOMAIN_ID=99`
- **Headless:** Gazebo server mode for CI (no GUI)
- **Results:** Mounted volume for test output

### 3.6 Validation Cases

Hands-on checks in the sim (driving by hand, §5.6) find problems that the planned tests miss. Each check, and each problem it finds, becomes a **validation case** in the register below, so every one ends up as an automated test at some level. Feature **2.3 `validation-scenarios`** built the tooling and automated the cases found so far. After that, the register is a living list: add a row for every new manual check or finding, in any feature, and automate it in that feature or note where it will be.

**Test levels**, cheapest first. Use the cheapest level that can show the problem.

| Level | Where | Use for |
| --- | --- | --- |
| Unit | The component's logic tests (no ROS, §2) | Behaviour of the algorithm: slip logic, model geometry. A fix found in a scenario gets a unit test here too |
| Launch | The component's launch test (no Gazebo) | Node behaviour: lifecycle, parameters, services, diagnostics, TF publishers |
| Scenario | `test_gnc_state_estimation_scenario` (`rc_racer_system_tests`): one file per case in `test/system/scenarios/`, each its own test `test_scenario_<name>` in a fresh headless Gazebo, with the world, `tire_mu`, scripted steps (time, speed, steering fraction) and checks. Format and how to watch them: `test/system/scenarios/README.md` | Driving cases with simple, well-defined inputs |
| Recording | The same test, driven by a recorded manual run: `make record NAME=...` turns your `/cmd_vel` in the running `make sim` into a scenario file, then you set its checks (§5.6) | Messy, human-shaped driving that is hard to script |
| L2 | The component's L2 sim test | Properties over a whole drive (drift, covariance) |
| Manual | This register only | Visual checks that need a person (RViz) |

**Scenario and recording tests check properties, not a trajectory** ("estimate within X of truth", "≥ 95 % inside 3σ", "WARN within 0.5 s"), because replays never reproduce a run exactly: Gazebo's real-time factor varies, and slides and spins diverge.

**Deferring a manual check.** A PR can merge before its manual checks are done: each deferred check gets a GitHub issue labelled [`manual-validation`](https://github.com/gotbordom/rc-racer/issues?q=label%3Amanual-validation) that references the PR, so it shows on the PR's timeline and stays open after the merge. Close the issue when the check is done, and update its row here. [Open manual checks](https://github.com/gotbordom/rc-racer/issues?q=is%3Aopen+label%3Amanual-validation) is the to-do list.

**Manual validation runbook.** For a hands-on session: working through open manual checks, or checking driving behaviour after a change.

1. **Pick the checks.** [Open manual checks](https://github.com/gotbordom/rc-racer/issues?q=is%3Aopen+label%3Amanual-validation) lists them. Each issue has its steps, pass criteria and setup.
2. **Start the sim fresh,** in the world and at the μ the check names: `make sim [WORLD=...] [MU=...]` (§5.6). Restart it between checks that need the car at the spawn point.
3. **Watch health** in a second terminal: `make se-watch`. Use the pad (hold LB) or `make keyboard`.
4. **Record anything worth keeping,** before driving it: `make record NAME=<name> ID=VAL-xxx TITLE='...'`, then Enter when done. A recording makes a finding reproducible and becomes its scenario.
5. **To see an automated case,** watch its scenario: `make replay NAME=<name>`. `test/system/scenarios/README.md` lists, for every scenario, the `make sim` world and μ, what a good run looks like, and what its known failure looks like.
6. **Record the result:**
   - **Pass:** comment on the issue (what you drove, what you saw), set the case's *Manual* column to ✅ with the date, and close the issue.
   - **Fail or new behaviour:** don't fix it in the session. Add a VAL row and a scenario that shows it (from the recording, or scripted), and decide with the owner how to record it. A known bug becomes an expected-fail case with a `bug` issue and a spec open item, and the commit message says what was decided and which options were rejected (see VAL-005/017, VAL-004/018, VAL-007/019).
7. **When a feature's manual issues are all closed,** its §1.1 status goes from 🔍 In validation to ✅ Done.

**Register.** *Automated:* ⏳ not yet · ✅ yes · ❌ known failure (the test is marked expected-fail until fixed). *Manual:* ✅ passed · 🕓 pending, with its issue · - not needed. *PR:* where the case was added or last changed.

| ID | Case | Found / checked | Requirement | PR | Level | Automated | Manual |
| --- | --- | --- | --- | --- | --- | --- | --- |
| VAL-001 | Turbo (3 m/s) straight and hard stop, forward and reverse: no false slip, speed tracks the wheel, settles to 0 | Manual pass 2026-11-21 (μ 1.0). Scenario 2026-11-23 (μ 1.0): 0.03 m max error, no slip | REQ-GNC-002, -005 | #1, #10 | Scenario ✅ (`val001_turbo_straight_stop`, μ 1.0). At μ 0.5 full throttle and a hard stop really slip: VAL-005, VAL-006 | ✅ | 🕓 #2 |
| VAL-002 | Reverse with turns: heading within 1° of truth, arrows the way the car faces | Manual pass 2026-11-17. Scenario 2026-11-23: 0.28° max yaw error | REQ-GNC-002 | #1, #10 | Scenario ✅ (`val002_reverse_turns`) | ✅ | ✅ |
| VAL-003 | Full-lock circles at normal speed, both ways: geometry at full lock, IMU lever arm, **no false yaw-rate slip** at the steering limit (margin `slip.yaw_rate_margin`, TBD) | Manual, to do. Scenario 2026-11-23: no slip events, 0.16 m max error over 21 m | REQ-GNC-002, -005 | #1, #10 | Scenario ✅ (`val003_full_lock_circles`) | ✅ | 🕓 #2 |
| VAL-004 | Full-lock turbo at μ 0.5: the car slides or spins. WARN on oversteer, recovers to OK, never stuck; measure the position error | Scenario 2026-11-23, 5 runs: it **understeers** (slides at 10-17°), slip flagged 2.0-3.2 s into each turn, back to OK, never stuck; error 1.1-1.76 m max over 32 m. Honesty in the slide is VAL-018 | REQ-GNC-005 | #1, #10 | Scenario ✅ (`val004_full_lock_turbo`) | ✅ | 🕓 #2 |
| VAL-005 | Full-throttle launch at μ 0.5: the wheel spins up ahead of the body. WARN within 0.5 s, estimate follows the IMU, back to OK | Manual pass 2026-11-21 (0.03 m error over 5.9 m). Scenario 2026-11-23: detected after 0.45-0.56 s, max error 0.11-0.20 m. The covariance before detection is VAL-017 | REQ-GNC-005 | #1, #10 | Scenario ✅ (`val005_launch_slip`) | ✅ | ✅ |
| VAL-006 | Braking from 3 m/s at μ 0.5: the car slides on faster than the wheels. Flagged within 0.3 s of the body getting 0.15 m/s ahead of its wheels (timed from ground truth), estimate inside 3σ | Found 2026-11-21: not detected (spec Open Item 5). Scenario 2026-11-23: ~1.1 s late or not at all. **Fixed 2026-11-24 (2.4):** a tighter window threshold for a wheel behind while braking; the slide reaches 0.15 m/s 0.64 s in, flagged 0.02 s before that (two runs), 100 % inside 3σ | REQ-GNC-005 | #1, #10, 2.4 | Unit ✅ (`BrakingSlideIsDetectedWithinHalfASecond`, `HardStopIsNotABrakingSlide`); scenario ✅ (`val006_braking_slip`) | ✅ | - |
| VAL-007 | Driving into an obstacle (the bench): slip ends via the safety nets, no getting stuck; impact itself not seen (DEV-07) | Manual 2026-11-17. Scenario 2026-11-24 (scripted arc to the bench, 4 runs): reaches it every time, slip ends when the wheels stop, settles, OK. The estimate drove on while pinned (VAL-019); since 2.4 it stops, 0.09-0.30 m final error | REQ-GNC-005 | #1, #10 | Unit ✅ (`CrashSpinDownIsDetected`, `CrashTheImuMissedEndsWhenStopped`); scenario ✅ (`val007_bench_crash`, practice course); recording (optional, `make record`) | ✅ | ✅ |
| VAL-008 | Covariance ellipse grows while driving | Manual, to do. L2 2026-11-23: x+y variance rises to 0.25 m² at 60 m, falls to 0.12 m² as the loop returns toward the start (scale term, VAL-016), and stays ≥ 1.28 × q·d (distance noise) throughout | REQ-GNC-006 | #1, #10 | Unit ✅ (`CovarianceGrowsWithoutUpdates`); L2 ✅ (variance ≥ 0.8 × q·d at every 10 m, ends above its start) | ✅ | 🕓 #3 |
| VAL-009 | `/odom` speed against `/sim/truth/wheel_odom`: no lag or bias | Recorder log 2026-11-21. L2 2026-11-23: bias −0.0002 m/s cruising, lag +0.01 s | REQ-GNC-002 | #1, #10 | L2 ✅ (bias ≤ 0.02 m/s, lag ≤ 0.05 s) | ✅ | 🕓 #4 |
| VAL-010 | `~/reset` while driving: back to the origin, covariance collapses | Manual pass 2026-11-17 | - | #1 | Launch ✅ (`test_4_reset_returns_to_origin`) | ✅ | ✅ |
| VAL-011 | Lifecycle: `deactivate` stops `/odom` with STALE "inactive"; `activate` restarts at the car's current position | Manual, to do | - | #1, #10 | Launch ✅ (`test_7_deactivate_stops_odom_and_is_stale`, `test_8_activate_restarts_odom_at_the_car`) | ✅ | 🕓 #5 |
| VAL-012 | `motion_model: unicycle` configures and runs; `tank` fails `configure` with a clear error | Manual, to do | REQ-GNC-007 | #1, #10 | Launch ✅ (`test_9_motion_model_tank_fails_unicycle_runs`) | ✅ | 🕓 #6 |
| VAL-013 | TF tree `odom → base_footprint → base_link → sensors`; only `state_estimation` and `robot_state_publisher` publish `/tf` | Manual pass 2026-11-17 | - | #1, #10 | Launch ✅ (`test_3_tf_published_by_this_node_only`); L2 ✅ (`test_tf_tree_and_publishers`: every frame's parent, and the two `/tf` publishers, in the full sim stack) | ✅ | ✅ |
| VAL-014 | Pad steering: full stick is full lock at any speed, wheels follow the stick in reverse (`steering_relay`) | Added 2026-11-21, not driven yet | - | #1, #10 | Unit ✅ (`test_steering_relay.py`) | ✅ | 🕓 #7 |
| VAL-015 | RViz: frames and the `/odom` arrow follow the car, ellipse drawn flat | Manual pass 2026-11-17 | - | #1 | Manual | - | ✅ |
| VAL-016 | Drive forward, then straight back: the covariance ellipse shrinks for a while. Expected from the scale-error term (it scales with distance from the odom origin) if the scale error is the same both ways; check the true error shrinks too, or the covariance is overconfident on the way back | Owner, manual 2026-11-22 (#8). Unit tests 2026-11-23: hypothesis confirmed; the true error shrinks with it, and a scale that differs in reverse stays inside 3σ. Scenario 2026-11-23: variance 12.9 % lower 5.0 m from the start, as predicted, 100 % inside 3σ | REQ-GNC-006 | #1, #10 | Unit ✅ (`OutAndBackCovarianceShrinksThenGrows`, `OutAndBackScaleErrorCancelsAsTheCovarianceSays`, `OutAndBackDifferentScaleInReverseIsCovered`); scenario ✅ (`val016_out_and_back`) | ✅ | - |
| VAL-017 | Launch slip at μ 0.5: the covariance stays honest while the slip develops, before it is detected | Found 2026-11-23 by the VAL-005 scenario: overconfident until detection, ~0.1 m error at ~0.02 m sigma, ~15 % of the first second outside 3σ. **Fixed 2026-11-24 (2.4, #9):** on detection the position is moved back by what the wheel added over the window (spread over 0.5 s), the covariance grows by the correction's uncertainty; before detection the wheel is distrusted in proportion to its acceleration mismatch: 100 % inside | REQ-GNC-006 | #10, 2.4 | Unit ✅ (`CaughtLaunchSlipCorrectsThePositionBack`); scenario ✅ (`val017_launch_slip_covariance`) | ✅ | - |
| VAL-018 | Full-lock turbo slide at μ 0.5: the estimate and covariance stay honest, with no position steps | Found 2026-11-23 by the VAL-004 scenario: **1.1-1.76 m error at σ ≤ 0.24 m** (24-56 % inside 3σ); no sideways-velocity state, slide caught 2-3 s late, the IMU fallback runs away, a ~7 cm step when slip ends (spec Open Item 7, #11) **2.5 (2026-11-27):** sideways velocity in the state, the no-slip constraint relaxed in a slide, forward speed from the turn: 0.72-0.94 m, 58-91 % inside 3σ (five runs), no steps. Still expected-fail: the slide isn't steady; needs a tyre model or a velocity sensor (spec Open Item 7, #11) | REQ-GNC-002, -005, -006 | #10, 2.5 | Scenario (`val018_sideways_slide_honest`, expected-fail) | ❌ | - |
| VAL-019 | Pinned against an obstacle with the wheels spinning: the estimate stops with the car | Found 2026-11-24 by the VAL-007 scenario: drove on at wheel speed, 4.3 m past the bench after 4 s at σ 0.5 m; the sim IMU missed the impact (DEV-07). **Fixed 2026-11-24 (2.4, #14):** the sim IMU sees the impact (DEV-07 fix); after an impact that took ≥ 30 % of the speed, with the wheel unchanged and the IMU quiet, the car is held pinned and the distance since the impact is taken back: final error 0.10-0.26 m on three runs | REQ-GNC-005, -006 | #10, 2.4 | Unit ✅ (`PinnedCarEstimateStopsWithIt`, `ImpactThatLeavesSomeSpeedStillPins`, `ImpactTheImuOnlyPartlySawStillPins`, `PinnedCarThatDrivesAwayIsFollowed`, `BumpIsNotAPinnedStop`); scenario ✅ (`val019_pinned_estimate_stops`) | ✅ | - |

---

## 4. Simulation Environment (Phase 1)

> **Phase 1 complete (2026-11-15).** See [4.7 Phase 1 Exit Criteria](#47-phase-1-exit-criteria) and the known steering-lag limitation in [4.2](#42-robot-model-urdf).

Simulation-first development allows testing without hardware, faster iteration, and reproducible test scenarios.

**Simulator:** Gazebo Fortress (the ROS 2 Humble pairing of "new" Gazebo, formerly Ignition), via the `ros_gz` packages. Gazebo Classic is end-of-life and is not used.

### 4.1 Simulation Package Structure

```
src/rc_racer_simulation/
├── CMakeLists.txt               # Also generates course worlds at build time
├── package.xml
├── urdf/
│   ├── rc_racer.urdf.xacro      # Main robot description + dimensions
│   ├── chassis.urdf.xacro       # Chassis geometry
│   ├── wheels.urdf.xacro        # Steering hinges, wheels, Ackermann drive plugin
│   └── sensors.urdf.xacro       # IMU, GPS, camera links and Gazebo sensors
├── worlds/
│   └── empty_course.world       # Flat test world; also the template for course worlds
├── courses/
│   └── practice_course.yaml     # Course file: start, cones, obstacles (WGS84)
├── scripts/
│   └── generate_course_world.py # Course file -> worlds/<name>.world (build time)
├── test/
│   └── test_generate_course_world.py
├── config/                       # (reserved)
├── meshes/                       # (reserved - primitives are used for now)
└── launch/
    └── simulation.launch.py     # Gazebo + robot + bridge; args: world, headless
```

A single launch file covers every case via arguments; separate `gazebo.launch.py` / `spawn_robot.launch.py` files are not needed.

### 4.2 Robot Model (URDF)

Based on Traxxas Slash hardware specifications from `documentation/Rc-Car-Design.md`:

| Component         | Value                       | Source           |
| ----------------- | --------------------------- | ---------------- |
| Wheelbase         | 384 mm                      | Rc-Car-Design.md |
| Wheel Diameter    | 67.4 mm                     | Rc-Car-Design.md |
| Wheel Width       | 28.5 mm                     | Rc-Car-Design.md |
| Final Drive Ratio | 9.384:1                     | Rc-Car-Design.md |
| Encoder PPR       | 128                         | Rc-Car-Design.md |
| Drivetrain        | 2WD, rear wheels driven     | Magnum 272 transmission |
| Steering          | Ackermann, ±0.52 rad (~30°) | Stock Slash servo (approx.) |
| Speed limit (sim) | ±3.0 m/s, ±3.0 m/s²         | Simulation setting |

Front wheels are mounted on steering hinges (revolute about Z); the Gazebo `AckermannSteering` system computes the inner/outer steering angles and rear wheel speed from a `Twist` command.

**Physics:** Gazebo's DART engine handles gravity, rigid-body dynamics (from each link's mass/inertia), contacts and Coulomb friction. Not modelled: tire deformation, suspension, motor torque curves, or terrain-dependent friction.

**Tyre friction** is the `tire_mu` launch argument (`make sim MU=...`), default **0.5**: dirt or dry grass, the low-grip end of the usual RoboMagellan surfaces (REQ-SYS-009). Dry pavement is about 0.8, and wet grass about 0.3. The ground plane keeps the SDF default (1.0), and Gazebo uses the lower of the two. Only the rear wheels drive, so grip caps the acceleration at about **3.3 × `tire_mu` m/s²** (measured 2026-11-21: 1.7 m/s² at 0.5, 0.6 m/s² at 0.2). The drive ramps the wheel speed at 3 m/s², so below about 0.9 a full-throttle launch spins the rear wheels: the wheel reaches the commanded speed in 1 s and the body follows more slowly. Use a low value (for example `MU=0.2`) to exercise slip handling.

Wheel joints carry only bearing-level losses (`damping = 0.001`, `friction = 0`). With 33.7 mm wheels, larger values become tens of newtons of drag at the tire and the free-rolling front wheels skid instead of rolling (the car crawls and slides sideways instead of turning). The smoke test checks ground-truth speed and turn rate against the command to catch this.

**Known limitation - steering lag:** Fortress' `AckermannSteering` drives the steering joints with a hard-coded gain of 1, i.e. a first-order lag with a **~1 s time constant** (~63% of the target angle after 1 s; a real servo takes ~0.15 s). It has no gain parameter (`steer_p_gain` arrives in Gazebo Garden). Steady-state turning is correct, so this is accepted for Phases 1-3. **Revisit before tuning path following (Phase 4)**: either drive the steering joints with `JointPositionController` (settable gain) from a small `/cmd_vel` -> steering-angle node, or move to a newer Gazebo.

### 4.3 Simulated Sensors and Topics

All Gazebo topics reach ROS 2 through `ros_gz_bridge`. Sensors flow one way (Gazebo → ROS); commands flow one way (ROS → Gazebo).

Gazebo's IMU, GNSS and wheel odometry are **noise-free** and published as truth on `/sim/truth/*`. The **sensor adapter** (`sensor_adapter_node.py`, the simulator's stand-in for the sensor drivers) adds realistic noise and sets each message's covariance from the same values in `config/sensor_noise.yaml`, then publishes the ICD topics. Vehicle software uses only the ICD topics; `/sim/truth/*` is for tests ([ICD-VEH-TST](ICD-VEH-TST)).

| Topic          | Type                       | Direction    | Source                                  | Rate   |
| -------------- | -------------------------- | ------------ | --------------------------------------- | ------ |
| `/imu`         | `sensor_msgs/Imu`          | gz → ROS     | `Imu` system → sensor adapter           | 100 Hz |
| `/navsat`      | `sensor_msgs/NavSatFix`    | gz → ROS     | `NavSat` system → sensor adapter        | 10 Hz  |
| `/camera`      | `sensor_msgs/Image`        | gz → ROS     | `Sensors` (ogre2)                       | 30 Hz  |
| `/camera_info` | `sensor_msgs/CameraInfo`   | gz → ROS     | `Sensors` (ogre2)                       | 30 Hz  |
| `/wheel/odom`  | `nav_msgs/Odometry`        | gz → ROS     | `AckermannSteering` → sensor adapter    | 50 Hz  |
| `/sim/truth/imu`, `/sim/truth/navsat`, `/sim/truth/wheel_odom` | as above | gz → ROS | Gazebo, noise-free (tests only); `/sim/truth/imu` from the sensor adapter: ideal accelerometer from true motion (DEV-07) | as above |
| `/sim/truth/body_odom` | `nav_msgs/Odometry` | gz → ROS | `OdometryPublisher`: true body pose and velocity, stamped (tests only) | 100 Hz |
| `/joint_states`| `sensor_msgs/JointState`   | gz → ROS     | `JointStatePublisher`   | -      |
| `/clock`       | `rosgraph_msgs/Clock`      | gz → ROS     | (simulation time)       | -      |
| `/cmd_vel`     | `geometry_msgs/Twist`      | ROS → gz     | `AckermannSteering`     | -      |

`/wheel/odom` is computed from wheel rotation and stands in for the wheel encoders. It is deliberately not named `/odom`: the fused estimate published by `state_estimation` will own `/odom`.

**Simulated sensor noise** (`config/sensor_noise.yaml`, proposed values to be tuned against real sensor logs in Phase 5):

| Sensor | Noise (1σ) | Model |
| --- | --- | --- |
| IMU gyro / accelerometer | 0.005 rad/s / 0.05 m/s² | White, per sample |
| IMU orientation | 0.02 rad roll/pitch, 0.05 rad yaw | White, per sample |
| GNSS | 2.0 m horizontal, 4.0 m vertical | Gauss-Markov, 30 s correlation time (real GNSS error wanders slowly) |
| Wheel odometry | 0.02 m/s on forward speed | White; other fields reported as unmeasured (variance 1e6) |

Noise is added in the adapter rather than in Gazebo because Fortress' NavSat sensor applies its horizontal noise in **degrees** (2.0 gives ~200 km), and the Ackermann odometry has no noise model.

The camera renders with OGRE 2; on machines without a GPU (e.g. CI) it uses software rendering (`LIBGL_ALWAYS_SOFTWARE=1`).

### 4.4 Worlds and Course Files

**Empty Course** (`empty_course.world`):

- Flat ground plane, sun, origin marker and X/Y axes
- Required systems: Physics, UserCommands, SceneBroadcaster, Sensors, Imu, NavSat
- `spherical_coordinates` set so GPS reports real lat/lon
- Template for all generated course worlds

**Course files** (`courses/<name>.yaml`) are the single source of truth for a RoboMagellan course. Like the competition hand-out, they list the start point, waypoint cones (mandatory with `order`, bonus with `multiplier`, finish) and obstacles in WGS84 decimal degrees. At build time, CMake runs `generate_course_world.py` on every course file to produce `worlds/<name>.world`:

- The start point is the world origin (`spherical_coordinates`); the car spawns there facing east (+X)
- Each waypoint becomes an 18" orange cone built from primitives (works offline, no Fuel download). Cones are dynamic, so a car that bumps one moves it
- Each obstacle becomes a static box or cylinder

Because the world is generated from the course file, the same file can later be loaded by the car as its course map, and tests can compare sim positions against it.

**Practice Course** (`practice_course.yaml`): ~50 × 50 m, 2 mandatory cones (ordered), 2 bonus cones, a finish cone, and 3 static obstacles (one between the start and the first cone). Its location is a **placeholder** (Alameda County Fairgrounds, Pleasanton, CA) until a real course is known.

**Planned:**

- **Full-scale course:** up to the 1,000 ft (~305 m) straight-line limit, for endurance/planning tests
- **Outdoor Terrain:** height-map terrain (grass, dirt, hills) with per-surface friction

### 4.5 Simulation Tests

| Test | Package | Runs in | Validates |
| ---- | ------- | ------- | --------- |
| `test_generate_course_world` | `rc_racer_simulation` | Nightly | Geodetic conversion, course validation, generated SDF |
| `test_simulation_smoke` | `rc_racer_system_tests` | Nightly | `/clock` advances; IMU, GPS, camera publish plausible data; car drives forward on `/cmd_vel`; clean shutdown |
| `test_course_world` | `rc_racer_system_tests` | Nightly | Practice course matches its course file: all cones and obstacles present, cones at course positions and upright, GPS at start = course start; clean shutdown |

Simulation packages are skipped by the fast CI pipeline (its image has no Gazebo) and run nightly inside the simulation image. See [3.5 System / End-to-End Tests](#35-system--end-to-end-tests).

### 4.6 Running the Simulation

**Run the simulation tests (from host, same steps as nightly CI):**

```bash
./scripts/run-system-tests.sh
```

**Watch and drive it (GUI):**

```bash
# Terminal 1 (host): start Gazebo with GUI
xhost +local:docker
cd docker && docker compose up simulation-gui

# Terminal 2 (host): drive from the dev container
cd docker && docker compose up -d dev && docker compose exec dev bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`simulation-gui` mounts `src/rc_racer_simulation` and rebuilds it on start, so URDF/world/launch/course edits only need `docker compose restart simulation-gui`.
It opens the practice course by default; choose another world with `SIM_WORLD=empty_course.world docker compose up simulation-gui`.

**Inside the dev container (headless or a specific world):**

```bash
ros2 launch rc_racer_simulation simulation.launch.py headless:=true
ros2 launch rc_racer_simulation simulation.launch.py world:=practice_course.world

# Verify topics
ros2 topic list
ros2 topic echo /navsat
ros2 topic echo /wheel/odom
```

**Developing nodes against the simulation:** leave the simulation running; build and restart only your node in the dev container. Gazebo only needs a restart when the robot model, world or course changes.

### 4.7 Phase 1 Exit Criteria

Phase 1 is complete when all of the following are true:

| Criterion | Status |
| --------- | ------ |
| Robot model with IMU, GPS, camera and Ackermann drive | ✅ Done |
| Simulation tests green in the nightly pipeline | ✅ Done |
| Practice cone course generated from a course file | ✅ Done |
| Simulation runs from the dev container (Gazebo Fortress) | ✅ Done |
| Car driven manually around the practice course in the GUI, with no visual/physics issues found (wheel orientation, steering direction, chassis height, camera view) | ✅ Done (found and fixed wheel skidding; steering lag accepted, §4.2) |

---

## 5. Docker Development Environment

All development is done inside Docker containers for reproducibility. This ensures:

- Consistent ROS 2 Humble + Gazebo environment
- No "works on my machine" issues
- Identical dev/CI/production environments
- Easy onboarding for new developers

### 5.1 Supported Platforms

> **⚠️ Primary Platform: Ubuntu Linux (22.04+)**
>
> The setup scripts and Docker configuration are designed for **Ubuntu/Debian Linux**.
> Other platforms may work but are not officially supported.

| Platform          | Status           | Notes                                                             |
| ----------------- | ---------------- | ----------------------------------------------------------------- |
| **Ubuntu 22.04+** | ✅ Supported     | Primary development platform                                      |
| **Debian 12+**    | ✅ Supported     | Should work identically to Ubuntu                                 |
| **Other Linux**   | ⚠️ Partial       | May need manual Docker/X11 setup                                  |
| **macOS**         | ❌ Not Supported | Requires Docker Desktop, XQuartz for GUI, different volume mounts |
| **Windows**       | ❌ Not Supported | Requires Docker Desktop + WSL2, VcXsrv/WSLg for GUI               |

**Linux-Specific Assumptions:**

- Docker installed via `get.docker.com` or `apt-get`
- X11 display server (for Gazebo, RViz GUI)
- `/tmp/.X11-unix` socket for X11 forwarding
- `/dev/dri` for GPU passthrough (optional)

### 5.2 New Developer Setup (Ubuntu/Debian)

#### Prerequisites

| Requirement        | Version   | Check Command            |
| ------------------ | --------- | ------------------------ |
| **Docker**         | 20.10+    | `docker --version`       |
| **Docker Compose** | V2 (2.0+) | `docker compose version` |
| **X11** (Linux)    | -         | `echo $DISPLAY`          |

#### Installing Prerequisites (Ubuntu/Debian)

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# LOG OUT AND BACK IN for group changes to take effect

# Install Docker Compose (V2 plugin)
sudo apt-get install docker-compose-plugin

# Verify
docker --version
docker compose version
```

#### Automated Setup Script (Recommended)

The setup script checks prerequisites, configures X11, and builds containers:

```bash
# Clone the repository
git clone <repo-url> rc-racer && cd rc-racer

# Run the setup script
./scripts/setup-dev-env.sh
```

**What the script does:**

1. ✅ Checks Docker and Docker Compose are installed
2. ✅ Configures X11 forwarding for GUI apps (Gazebo, RViz)
3. ✅ Creates `docker/.env` from template
4. ✅ Builds the development container
5. ✅ Verifies the setup works

### 5.3 Manual Setup (Alternative)

If you prefer manual setup or the script fails:

```bash
# 1. Clone the repository
git clone <repo-url> rc-racer && cd rc-racer

# 2. Copy environment template
cp docker/.env.example docker/.env

# 3. Allow X11 connections (for GUI apps)
xhost +local:docker

# 4. Build and start development container
cd docker
docker compose up -d dev

# 5. Enter the container
docker compose exec dev bash

# 6. (Inside container) Build the workspace
colcon build

# 7. (Inside container) Source and run
source install/setup.bash
ros2 launch rc_racer_bringup rc_racer.launch.py
```

### 5.4 Directory Structure

**Directory:** `docker/` (project root)

```
docker/
├── Dockerfile.dev           # Development environment (GUI support)
├── Dockerfile.ci            # CI/CD environment (lean, no GUI)
├── Dockerfile               # Production/hardware deployment
├── docker-compose.yml       # Service orchestration
└── .env.example             # Environment variables template

scripts/
└── setup-dev-env.sh         # Automated setup for new developers
```

### 5.5 Development Workflow with Docker

#### Starting the Dev Container

```bash
cd docker
docker compose up -d dev
docker compose exec dev bash
```

#### Rebuilding After Code Changes

Source code is mounted into the container, so edits on host are immediately available:

```bash
# Inside container - rebuild changed packages
colcon build --packages-select <package_name>
source install/setup.bash
```

#### Running Tests Inside Container

```bash
# Inside container
colcon test --packages-select <package_name>
colcon test-result --verbose
```

#### Stopping the Container

```bash
# From host
cd docker
docker compose down
```

### 5.6 GUI Support (Gazebo, RViz)

For visualization tools, X11 forwarding is configured:

```bash
# On host (Linux) - allow X connections
xhost +local:docker

# Start container with GUI support
docker compose up -d dev

# Inside container - launch Gazebo
ros2 launch gazebo_ros gazebo.launch.py
```

**Driving the car by hand** (`rc_racer_bringup/launch/sim_manual.launch.py`): Gazebo GUI, `state_estimation` on sim time, and RViz with the TF tree, the car, and `/odom` with its 3σ covariance ellipse.

```bash
make sim                     # = xhost +local:docker && docker compose ... up sim-manual
make sim WORLD=empty_course.world
make sim MU=0.2               # low-grip tyres (default 0.5), see §4.2
```

| Control | Xbox controller (`config/xbox_teleop.yaml`) |
| --- | --- |
| Enable (deadman) | Hold **LB**. Releasing it sends one zero command, which stops the car |
| Speed | Left stick up / down (1 m/s full stick) |
| Steer | Right stick left / right: a steering angle, like an RC transmitter. Full stick is full lock (±0.52 rad) at any speed, and the wheels follow the stick in reverse too (`steering_relay` turns it into the matching yaw rate on `/cmd_vel`) |
| Turbo | Hold **RB** as well (3 m/s) |

The controller is read by `joy`'s `game_controller_node`, which uses SDL's standard gamepad layout, so any pad SDL knows works the same over USB or Bluetooth. Plug it in before or after starting. With `make sim` running, in a second terminal:

| Command | Does |
| --- | --- |
| `make joy-echo` | Show what the pad sends on `/joy` |
| `make joy-devices` | List the pads SDL sees, with their GUIDs |
| `make joy-sdl-map` | Pad drives wrongly (scrambled buttons, a stick that can't centre)? SDL has no mapping for it. This measures the raw pad and prints a mapping line for `SDL_GAMECONTROLLERCONFIG` in the `sim-manual` service. The Xbox One Elite 2 over Bluetooth is already there |
| `make joy-calibrate` | Quick fix: detect the buttons and axes and apply them to teleop until it restarts |
| `make keyboard` | Drive with the keyboard instead |
| `make se-reset`, `se-deactivate`, `se-activate`, `se-diagnostics` | `state_estimation` service, lifecycle and health |
| `make se-watch` | Stream `state_estimation`'s health, one line per report. A slip WARN shows for only one report (about 1 s), so watch here rather than with `se-diagnostics`; a rise in `slip_events` is a slip even if its WARN scrolled past |
| `make record NAME=<name> ID=VAL-xxx TITLE='...'` | Record your drive (pad or keyboard) as a validation scenario, `test/system/scenarios/<name>.yaml`, until you press Enter. Restart `make sim` first so the car starts at the spawn point. Fill in the file's checks, then run it with `make test-system FILTER=test_scenario_<name>` (§3.6) |
| `make replay NAME=<name>` | Drive the running sim from a scenario file, to watch it in Gazebo and RViz (open-loop, nothing checked) |
| `make shell` | Shell in the sim container with ROS sourced |

`make` on its own lists every target.

### 5.7 Container Architecture

| Container       | Purpose                 | Base Image                 |
| --------------- | ----------------------- | -------------------------- |
| `dev`           | Interactive development | `osrf/ros:humble-desktop`  |
| `ci`            | CI/CD pipeline          | `osrf/ros:humble-ros-base` |
| `system-tests`  | System test runner      | `osrf/ros:humble-desktop`  |
| `gazebo-server` | Headless simulation     | `osrf/ros:humble-desktop`  |

### 5.8 Microservice Architecture Roadmap

> **Status: PLANNED** - Currently all packages run in a single container. The roadmap below
> outlines the transition to a microservice architecture where each ROS package runs in its
> own container.

#### 4.8.1 Current Architecture (Monolith)

```
┌─────────────────────────────────────────────────────────────────┐
│                     rc-racer-dev Container                       │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐│
│  │   state_    │ │ localization│ │   mapping   │ │  object_    ││
│  │ estimation  │ │             │ │             │ │ detection   ││
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘│
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐│
│  │  planning_  │ │   motor_    │ │   safety    │ │  telemetry  ││
│  │   control   │ │ controller  │ │             │ │             ││
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

**Pros:** Simple setup, fast communication, easy debugging
**Cons:** Single point of failure, can't scale/update independently

#### 4.8.2 Target Architecture (Microservices)

```
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  state_estimation│  │   localization   │  │     mapping      │
│    Container     │  │    Container     │  │    Container     │
│  (ros-network)   │  │  (ros-network)   │  │  (ros-network)   │
└────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘
         │                     │                     │
         └─────────────────────┼─────────────────────┘
                               │
                    ┌──────────┴──────────┐
                    │    ROS 2 Network    │
                    │  (DDS Discovery)    │
                    └──────────┬──────────┘
                               │
         ┌─────────────────────┼─────────────────────┐
         │                     │                     │
┌────────┴─────────┐  ┌────────┴─────────┐  ┌────────┴─────────┐
│ planning_control │  │ motor_controller │  │      safety      │
│    Container     │  │    Container     │  │    Container     │
└──────────────────┘  └──────────────────┘  └──────────────────┘
```

**Pros:** Independent scaling, isolated failures, per-package updates
**Cons:** Network overhead, more complex orchestration

#### 4.8.3 Implementation Phases

| Phase       | Milestone           | Description                                                 |
| ----------- | ------------------- | ----------------------------------------------------------- |
| **Phase 0** | ✅ Current          | Monolith - all packages in single container                 |
| **Phase 1** | Shared Base Image   | Multi-stage Dockerfile with per-package targets             |
| **Phase 2** | Critical Path Split | Separate `safety` into its own container (highest priority) |
| **Phase 3** | Perception Split    | Split `object_detection` and `state_estimation`             |
| **Phase 4** | Full Microservices  | All 8 packages in separate containers                       |

#### 4.8.4 Phase 1: Multi-Stage Dockerfile

Create a single Dockerfile with build targets for each package:

```dockerfile
# docker/Dockerfile.packages
# Base stage with common dependencies
FROM ros:humble-ros-base AS base
RUN apt-get update && apt-get install -y \
    ros-humble-tf2 \
    ros-humble-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Build stage
FROM base AS builder
COPY src/ /ros2_ws/src/
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build

# Per-package runtime images
FROM base AS state_estimation
COPY --from=builder /ros2_ws/install/state_estimation /ros2_ws/install/state_estimation
CMD ["ros2", "run", "state_estimation", "state_estimation_node"]

FROM base AS localization
COPY --from=builder /ros2_ws/install/localization /ros2_ws/install/localization
CMD ["ros2", "run", "localization", "localization_node"]

# ... repeat for each package
```

#### 4.8.5 Phase 1: Docker Compose for Microservices

```yaml
# docker/docker-compose.microservices.yml (future)
services:
  state_estimation:
    build:
      context: ..
      dockerfile: docker/Dockerfile.packages
      target: state_estimation
    networks:
      - ros-network
    environment:
      - ROS_DOMAIN_ID=0

  localization:
    build:
      context: ..
      dockerfile: docker/Dockerfile.packages
      target: localization
    networks:
      - ros-network
    depends_on:
      - state_estimation

  safety:
    build:
      context: ..
      dockerfile: docker/Dockerfile.packages
      target: safety
    networks:
      - ros-network
    restart: always # Critical service - auto-restart
    deploy:
      resources:
        limits:
          cpus: "0.5"
          memory: 256M

  # ... additional services

networks:
  ros-network:
    driver: bridge
```

#### 4.8.6 Considerations for Microservice Migration

| Consideration        | Solution                                                        |
| -------------------- | --------------------------------------------------------------- |
| **DDS Discovery**    | Use `ROS_DOMAIN_ID` or Fast-DDS Discovery Server                |
| **Shared TF**        | Run `tf2_ros/static_transform_publisher` in dedicated container |
| **Parameter Server** | Use `ros2 param` or shared config volume                        |
| **Logging**          | Centralized logging with mounted volume or log aggregator       |
| **Health Checks**    | Docker health checks + ROS lifecycle nodes                      |
| **Startup Order**    | Use `depends_on` with `condition: service_healthy`              |

#### 4.8.7 When to Migrate

Migrate to microservices when:

- ✅ All unit and integration tests pass
- ✅ System tests demonstrate stable behavior
- ✅ Performance profiling shows bottlenecks that benefit from isolation
- ✅ Team needs to deploy/update packages independently

**Do NOT migrate prematurely** - the monolith is simpler for early development.

---

## 6. Build & Test Commands

From the host, the `Makefile` wraps the Docker commands: `make test` (all unit and component tests plus linters, in the dev image; `PKG=<package>` for one package), `make test-system` (the Gazebo system tests; `FILTER=<regex>`), and `make images` (rebuild the images after a Dockerfile change). Run `make` to list them all.

> **Note:** All commands below are run **inside the Docker container** unless otherwise specified.

### 6.1 Building

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build with tests
colcon build --cmake-args -DBUILD_TESTING=ON

# Clean build
rm -rf build install log && colcon build
```

### 6.2 Testing

```bash
# Run all tests (excludes slow system tests)
colcon test --packages-skip rc_racer_system_tests

# Run tests for specific package
colcon test --packages-select <package_name>

# View test results
colcon test-result --verbose

# Run with output on failure
colcon test --event-handlers console_direct+

# Run specific test by name
colcon test --packages-select <package> --ctest-args -R <test_name>
```

### 6.3 Code Quality

```bash
# Linting (ament_lint)
colcon test --packages-select <package> --ctest-args -R lint

# Format check (if configured)
ament_clang_format --check src/<package>
```

---

## 7. Adding Tests

> **Note:** All commands below are run **inside the Docker container**.

### 7.1 Adding a New Unit Test (C++)

1. Create test file in `src/<package>/test/test_<name>_logic.cpp` (test the logic class!)
2. Add to `CMakeLists.txt`:
   ```cmake
   ament_add_gtest(test_<name>_logic test/test_<name>_logic.cpp)
   target_include_directories(test_<name>_logic PRIVATE include)
   # Note: minimal dependencies - ideally NO ROS deps for logic tests
   ```
3. Run: `colcon test --packages-select <package>`

### 7.2 Adding a New Unit Test (Python)

1. Create test file in `src/<package>/test/test_<name>_logic.py`
2. Add to `CMakeLists.txt`:
   ```cmake
   ament_add_pytest_test(test_<name>_logic_py test/test_<name>_logic.py)
   ```
3. Run: `colcon test --packages-select <package>`

### 7.3 Adding a New Integration Test

1. Copy template: `cp test/integration/launch/test_template.launch.py.example test/integration/launch/test_<name>.launch.py`
2. Modify the launch file:
   - Update `generate_test_description()` to launch your nodes
   - Add test classes for your assertions
3. Add to `test/integration/CMakeLists.txt`:
   ```cmake
   add_launch_test(
     launch/test_<name>.launch.py
     TARGET test_<name>
     TIMEOUT 60
   )
   ```
4. Run: `colcon test --packages-select rc_racer_integration_tests`

### 7.4 Adding a New System Test

1. Create launch test file in `test/system/launch/test_<name>.launch.py`
2. Include:
   - `generate_test_description()` that launches full system + Gazebo
   - Test classes for validation
   - Post-shutdown cleanup verification
3. Add to `test/system/CMakeLists.txt`:
   ```cmake
   add_launch_test(
     launch/test_<name>.launch.py
     TARGET test_<name>
     TIMEOUT 300  # System tests need longer timeouts
   )
   ```
4. If needed, add Gazebo world file to `test/system/worlds/`
5. Test locally (inside container): `colcon test --packages-select rc_racer_system_tests`
6. Test from the host (same as nightly CI): `./scripts/run-system-tests.sh test_<name>`

---

## 8. Continuous Integration

### 8.1 CI/CD Platform

**Platform:** GitHub Actions

**Workflow Files:** `.github/workflows/`

| Workflow    | File          | Trigger                  | Purpose                    |
| ----------- | ------------- | ------------------------ | -------------------------- |
| **CI**      | `ci.yml`      | Push/PR to main, develop | Fast: Build + Lint + Tests |
| **Nightly** | `nightly.yml` | Daily 2 AM UTC + manual  | Slow: Full system tests    |

### 8.2 Test Execution by Trigger

| Trigger                    | Unit Tests | Integration Tests | System Tests           |
| -------------------------- | ---------- | ----------------- | ---------------------- |
| **Every commit/PR**        | ✅ All     | ✅ All            | ❌ Skip                |
| **Merge to main**          | ✅ All     | ✅ All            | ❌ Skip                |
| **Nightly build**          | ✅ All     | ✅ All            | ✅ All                 |
| **Pre-release**            | ✅ All     | ✅ All            | ✅ All (required)      |
| **Before hardware deploy** | ✅ All     | ✅ All            | ✅ All + manual review |

### 8.3 GitHub Actions Workflows

#### CI Workflow (`.github/workflows/ci.yml`)

Runs on every push and pull request to `main` and `develop` branches.

```yaml
name: CI
on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main, develop]

jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    container:
      image: osrf/ros:humble-ros-base
    steps:
      - uses: actions/checkout@v4
      - name: Install dependencies
        run: apt-get update && apt-get install -y ...
      - name: Build
        run: |
          source /opt/ros/humble/setup.bash
          colcon build --symlink-install
      - name: Lint and Test
        run: |
          source /opt/ros/humble/setup.bash
          source install/setup.bash
          colcon test
          colcon test-result --verbose
      - name: Upload test results
        uses: actions/upload-artifact@v4
        if: always()
        with:
          name: test-results
          path: build/*/test_results/
```

#### Nightly Workflow (`.github/workflows/nightly.yml`)

Runs daily at 2 AM UTC, includes Gazebo for system tests.

```yaml
name: Nightly
on:
  schedule:
    - cron: "0 2 * * *"
  workflow_dispatch: # Allow manual trigger

jobs:
  system-tests:
    runs-on: ubuntu-22.04
    container:
      image: osrf/ros:humble-ros-base
    steps:
      # ... same as CI but includes gazebo dependencies
      # ... extended timeouts for system tests
```

### 8.4 CI Pipeline Commands

All CI runs inside the ROS 2 container for consistency.

```bash
# Fast feedback (PR checks) - inside ci container
colcon test --packages-skip rc_racer_system_tests

# Full test suite (nightly) - inside ci container
colcon test

# System tests only (from host, runs in the simulation image)
./scripts/run-system-tests.sh
```

### 8.5 CI Docker Images

| Container      | Purpose            | Base Image                 |
| -------------- | ------------------ | -------------------------- |
| GitHub Actions | CI/CD pipeline     | `osrf/ros:humble-ros-base` |
| `ci` (local)   | Local CI testing   | `osrf/ros:humble-ros-base` |
| `system-tests` | System test runner | `osrf/ros:humble-desktop`  |

---

## 9. Dependencies

### 9.1 Test Dependencies (per package.xml)

**Standard packages (C++ nodes):**

```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
<test_depend>ament_cmake_gtest</test_depend>
<test_depend>ament_cmake_pytest</test_depend>
<test_depend>ros_testing</test_depend>
```

**Python-only packages (using SPDX license headers):**

```xml
<!-- Use specific linters instead of ament_lint_common -->
<!-- (ament_copyright doesn't recognize SPDX format) -->
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_cmake_flake8</test_depend>
<test_depend>ament_cmake_pep257</test_depend>
<test_depend>ament_cmake_xmllint</test_depend>
<test_depend>ament_cmake_pytest</test_depend>
```

### 9.2 Linter Configuration

**Files in repository root:**

| File          | Purpose                                               |
| ------------- | ----------------------------------------------------- |
| `.flake8`     | Python style checking (line length, ignores)          |
| `.pydocstyle` | Relaxed pep257 - requires docstrings, flexible format |

**pydocstyle configuration (relaxed):**

- ✅ **Enforced:** Docstrings for classes, methods, functions (D101, D102, D103)
- ❌ **Ignored:** Formatting nits (D100, D104, D105, D107, D200, D203, D205, D212, D400, D401, D415)

### 9.3 System Test Dependencies

All included in Docker images:

- Gazebo (simulation)
- ros2bag (replay testing)
- launch_testing (integration tests)

---

## 10. Security & Compliance

### 10.1 SBOM (Software Bill of Materials)

**Purpose:** Track all software components for security auditing, license compliance, and vulnerability management.

**Tool:** [Syft](https://github.com/anchore/syft) (by Anchore)

**Formats Generated:**

| Format    | Standard | File                    | Use Case                   |
| --------- | -------- | ----------------------- | -------------------------- |
| SPDX JSON | ISO      | `sbom-*-spdx.json`      | Compliance, license audits |
| CycloneDX | OWASP    | `sbom-*-cyclonedx.json` | Security tooling, VEX      |

**What's Scanned:**

| Target          | Contents                                     |
| --------------- | -------------------------------------------- |
| Docker CI image | apt packages, pip packages, system libraries |
| Source code     | package.xml, CMakeLists.txt, Python imports  |

**Workflow:** `.github/workflows/sbom.yml` (runs on merge to main)

### 10.2 Vulnerability Scanning

**Tool:** [Grype](https://github.com/anchore/grype) (by Anchore)

**Why Grype?**

- Same ecosystem as Syft (designed to consume Syft-generated SBOMs)
- Lightweight, single-purpose vulnerability scanner
- Supports multiple vulnerability databases (NVD, GitHub Advisory, etc.)

**Severity Levels:**

| Level    | Action              |
| -------- | ------------------- |
| CRITICAL | ❌ Build fails      |
| HIGH     | ❌ Build fails      |
| MEDIUM   | ⚠️ Warning (logged) |
| LOW      | ℹ️ Info (logged)    |

**Output:**

| File                  | Format | Purpose                          |
| --------------------- | ------ | -------------------------------- |
| `vuln-*-report.json`  | JSON   | Machine-readable, CI integration |
| `vuln-*-report.txt`   | Table  | Human-readable summary           |
| `vuln-*-report.sarif` | SARIF  | GitHub Security tab integration  |

**Viewing Results:**

- **GitHub UI:** Repository → Security → Code scanning alerts
- **Artifacts:** Actions → SBOM & Security → Download artifacts

### 10.3 C++ Dependency Tracking

To improve SBOM accuracy for C++ dependencies, enable compile commands export:

```cmake
# In root CMakeLists.txt or via colcon
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
```

**Build with dependency tracking:**

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

This generates `compile_commands.json` which tools can parse for include paths and dependencies.

### 10.4 Security Workflow Integration

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Merge to  │────▶│    Syft     │────▶│   Grype     │
│    main     │     │  (SBOM Gen) │     │   (Scan)    │
└─────────────┘     └─────────────┘     └──────┬──────┘
                                               │
                    ┌──────────────────────────┼──────────────────────────┐
                    │                          │                          │
                    ▼                          ▼                          ▼
             ┌─────────────┐           ┌─────────────┐           ┌─────────────┐
             │  Artifacts  │           │   GitHub    │           │ Build Fail  │
             │  (90 days)  │           │ Security Tab│           │ (if HIGH+)  │
             └─────────────┘           └─────────────┘           └─────────────┘
```
