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

| Feature | Name | Element | Status |
| --- | --- | --- | --- |
| 2.1 | `state-estimation` | VEH.GNC.SE | 🚧 In progress: motion models, filter, ROS node and L2 sim test done (0.13 % drift in sim); ready for review |
| 2.2 | `object-detection` | VEH.PER.OD | ⏳ Planned |
| 3.1 | `localization` | VEH.GNC.LOC | ⏳ Planned (spec to be written first) |
| 3.2 | `mapping` | VEH.GNC.MAP | ⏳ Planned |
| 3.3 | `planning-control` | VEH.GNC.PLN | ⏳ Planned |
| 4.1 | `sim-steering-lag` | TST (DEV-04) | ⏳ Planned |
| 4.2 | `motor-controller` | VEH.ACT.MC | ⏳ Planned |
| 4.3 | `safety` | VEH.FDR.SAF | ⏳ Planned |

Phase 1 was completed before features were numbered. Its branch, `feature/simulation`, keeps its name. Phase 5 features will be numbered when Phase 5 is planned in detail.

**Branch names.** A feature branch is named `feature/<phase>.<feature>-<name>`, using the number and name from the table, e.g. `feature/2.1-state-estimation`. Merged feature branches are kept for later review.

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

**Physics:** Gazebo's DART engine handles gravity, rigid-body dynamics (from each link's mass/inertia), contacts and Coulomb friction (`mu = 1.0` on the tires). Not modelled: tire slip/deformation, suspension, motor torque curves, or terrain-dependent friction.

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
| `/sim/truth/imu`, `/sim/truth/navsat`, `/sim/truth/wheel_odom` | as above | gz → ROS | Gazebo, noise-free (tests only) | as above |
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
```

| Control | Xbox controller (`config/xbox_teleop.yaml`) |
| --- | --- |
| Enable (deadman) | Hold **LB**. Releasing it sends one zero command, which stops the car |
| Speed | Left stick up / down (1 m/s full stick) |
| Turn | Right stick left / right (1 rad/s full stick; the drive clamps it at the steering limit) |
| Turbo | Hold **RB** as well (3 m/s, 1.5 rad/s) |

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
