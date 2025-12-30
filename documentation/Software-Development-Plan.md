# Software Development Plan

## RC Racer - RoboMagellan Competition

---

## 1. Project Overview

This document outlines the software development plan for the RC Racer autonomous vehicle project. The system is built on ROS 2 Humble with a modular microservices architecture.

### 1.1 Package Structure

```
src/
├── state_estimation/    # Sensor fusion and state tracking
├── localization/        # GPS + IMU + camera positioning
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

---

## 2. Testing Strategy

### 2.1 Test Pyramid

```
┌─────────────────────────────┐
│ System / End-to-End Tests   │  ← few, slow, high confidence
├─────────────────────────────┤
│ Integration Tests           │  ← some, medium cost
├─────────────────────────────┤
│ Unit / Node-local Tests     │  ← many, fast
└─────────────────────────────┘
```

### 2.2 Unit Tests (Node-Local)

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

### 2.3 Integration Tests

**Purpose:** Test ROS graphs (not individual nodes) with real DDS communication.

**Location:** `test/integration/` (project root, separate package)

**Package:** `rc_racer_integration_tests`

**Run Command:**

```bash
colcon test --packages-select rc_racer_integration_tests
colcon test-result --verbose
```

#### 2.3.1 What Integration Tests Do

Integration tests launch **multiple real nodes** with **real DDS** to verify:

| Assertion Type          | Description                                       |
| ----------------------- | ------------------------------------------------- |
| **Topics Appear**       | Expected topics exist on the ROS graph            |
| **Messages are Sane**   | Published messages have valid data, correct types |
| **State Converges**     | System reaches expected state within timeout      |
| **Dead Nodes Detected** | Node failures trigger appropriate responses       |

#### 2.3.2 What Integration Tests Do NOT Do

- Test individual node logic (use unit tests)
- Test full end-to-end scenarios (use system tests)
- Test hardware interfaces directly

#### 2.3.3 Framework

- **`launch_testing`** - Launch nodes and run assertions
- **`launch_testing_ros`** - ROS 2 specific test utilities
- **`launch_testing_ament_cmake`** - CMake integration

#### 2.3.4 Integration Test Structure

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

#### 2.3.5 Planned Integration Tests

| Test Name                 | Nodes Under Test                                   | Assertions                                           |
| ------------------------- | -------------------------------------------------- | ---------------------------------------------------- |
| `test_sensor_pipeline`    | state_estimation ↔ localization                    | Data flows, topics exist, messages valid             |
| `test_planning_pipeline`  | localization → planning_control → motor_controller | Commands propagate, timing acceptable                |
| `test_safety_system`      | safety + all critical nodes                        | E-stop triggers, watchdog works, node death detected |
| `test_full_graph_startup` | All nodes                                          | All nodes launch, no crashes, all topics exist       |

#### 2.3.6 Example CMakeLists.txt Entry

```cmake
find_package(launch_testing_ament_cmake REQUIRED)

add_launch_test(
  launch/test_sensor_pipeline.launch.py
  TARGET test_sensor_pipeline
  TIMEOUT 60
)
```

#### 2.3.7 Launch Test File Structure

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

### 2.4 System / End-to-End Tests

**Purpose:** Full system validation in simulation with realistic sensors.

**Location:** `test/system/` (project root, separate package)

**Package:** `rc_racer_system_tests`

#### 2.4.1 Characteristics

| Property      | Value                                            |
| ------------- | ------------------------------------------------ |
| **Speed**     | SLOW (minutes per test)                          |
| **Stability** | BRITTLE (sensitive to timing, environment)       |
| **Value**     | EXTREMELY VALUABLE (catches real-world failures) |

#### 2.4.2 What System Tests Validate

- **Startup order** - Nodes initialize in correct dependency order
- **Failure modes** - System handles crashes and failures gracefully
- **Safety behavior** - E-stop, watchdogs, and safety limits work correctly
- **Real-time-ish constraints** - Control loops and sensor pipelines meet timing requirements

#### 2.4.3 Test Files

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

#### 2.4.4 System Test Descriptions

| Test                    | Validates                                                     | Timeout |
| ----------------------- | ------------------------------------------------------------- | ------- |
| `test_full_stack_sim`   | Full system in Gazebo, realistic sensors, navigation pipeline | 5 min   |
| `test_emergency_stop`   | E-stop triggers, system halts safely, watchdog works          | 2 min   |
| `test_startup_shutdown` | Startup order, clean shutdown, no zombies/leaks               | 2 min   |

#### 2.4.5 When CI/CD Runs System Tests

| Trigger                        | Tests Run                           |
| ------------------------------ | ----------------------------------- |
| **Nightly builds**             | All system tests                    |
| **Pre-release**                | All system tests (required to pass) |
| **Before hardware deployment** | All system tests + manual review    |

#### 2.4.6 Running System Tests

**Local (with ROS 2 + Gazebo installed):**

```bash
colcon test --packages-select rc_racer_system_tests
colcon test-result --verbose
```

**Docker Orchestrated (preferred):**

```bash
cd test/system/docker
docker-compose up --abort-on-container-exit
docker-compose down
```

**CI/CD Pipeline:**

```bash
docker-compose up --abort-on-container-exit --exit-code-from system-tests
```

#### 2.4.7 Docker Test Environment

The system tests run in an isolated Docker environment to ensure reproducibility:

- **Base image:** `osrf/ros:humble-desktop` (includes Gazebo)
- **Isolation:** Dedicated `ROS_DOMAIN_ID=99`
- **Headless:** Gazebo server mode for CI (no GUI)
- **Results:** Mounted volume for test output

---

## 3. Build & Test Commands

### 3.1 Building

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build with tests
colcon build --cmake-args -DBUILD_TESTING=ON
```

### 3.2 Testing

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select <package_name>

# View test results
colcon test-result --verbose

# Run with output on failure
colcon test --event-handlers console_direct+
```

### 3.3 Code Quality

```bash
# Linting (ament_lint)
colcon test --packages-select <package> --ctest-args -R lint
```

---

## 4. Development Workflow

### 4.1 Adding a New Test (C++)

1. Create test file in `src/<package>/test/test_<name>.cpp`
2. Add to `CMakeLists.txt`:
   ```cmake
   ament_add_gtest(test_<name> test/test_<name>.cpp)
   target_include_directories(test_<name> PRIVATE include)
   ament_target_dependencies(test_<name> <dependencies>)
   ```
3. Run: `colcon test --packages-select <package>`

### 4.2 Adding a New Test (Python)

1. Create test file in `src/<package>/test/test_<name>.py`
2. Add to `CMakeLists.txt`:
   ```cmake
   ament_add_pytest_test(test_<name>_py test/test_<name>.py)
   ```
3. Run: `colcon test --packages-select <package>`

### 4.3 Adding a New Integration Test

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

### 4.4 Adding a New System Test

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
5. Test locally: `colcon test --packages-select rc_racer_system_tests`
6. Test in Docker: `cd test/system/docker && docker-compose up --abort-on-container-exit`

---

## 5. Continuous Integration

### 5.1 Test Execution by Trigger

| Trigger                    | Unit Tests | Integration Tests | System Tests           |
| -------------------------- | ---------- | ----------------- | ---------------------- |
| **Every commit/PR**        | ✅ All     | ✅ All            | ❌ Skip                |
| **Merge to main**          | ✅ All     | ✅ All            | ❌ Skip                |
| **Nightly build**          | ✅ All     | ✅ All            | ✅ All                 |
| **Pre-release**            | ✅ All     | ✅ All            | ✅ All (required)      |
| **Before hardware deploy** | ✅ All     | ✅ All            | ✅ All + manual review |

### 5.2 CI Pipeline Commands

```bash
# Fast feedback (PR checks)
colcon test --packages-skip rc_racer_system_tests

# Full test suite (nightly)
colcon test

# System tests only (Docker)
cd test/system/docker && docker-compose up --abort-on-container-exit --exit-code-from system-tests
```

---

## 6. Dependencies

### 6.1 Test Dependencies (per package.xml)

```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
<test_depend>ament_cmake_gtest</test_depend>
<test_depend>ament_cmake_pytest</test_depend>
<test_depend>ros_testing</test_depend>
```

### 6.2 System Test Dependencies

- Gazebo (simulation)
- ros2bag (replay testing)
- launch_testing (integration tests)
