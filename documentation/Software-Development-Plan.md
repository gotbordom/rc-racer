# Software Development Plan

## RC Racer - RoboMagellan Competition

---

## 1. Project Overview

This document outlines the software development plan for the RC Racer autonomous vehicle project. The system is built on ROS 2 Humble with a modular microservices architecture.

**Development Environment:** All development, building, and testing is done inside Docker containers for reproducibility and isolation.

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

### 1.2 License

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

**Docker Orchestrated (preferred):**

```bash
cd test/system/docker
docker compose up --abort-on-container-exit
docker compose down
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

## 4. Docker Development Environment

All development is done inside Docker containers for reproducibility. This ensures:

- Consistent ROS 2 Humble + Gazebo environment
- No "works on my machine" issues
- Identical dev/CI/production environments
- Easy onboarding for new developers

### 4.1 Supported Platforms

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

### 4.2 New Developer Setup (Ubuntu/Debian)

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

### 4.3 Manual Setup (Alternative)

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

### 4.4 Directory Structure

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

### 4.5 Development Workflow with Docker

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

### 4.6 GUI Support (Gazebo, RViz)

For visualization tools, X11 forwarding is configured:

```bash
# On host (Linux) - allow X connections
xhost +local:docker

# Start container with GUI support
docker compose up -d dev

# Inside container - launch Gazebo
ros2 launch gazebo_ros gazebo.launch.py
```

### 4.7 Container Architecture

| Container       | Purpose                 | Base Image                 |
| --------------- | ----------------------- | -------------------------- |
| `dev`           | Interactive development | `osrf/ros:humble-desktop`  |
| `ci`            | CI/CD pipeline          | `osrf/ros:humble-ros-base` |
| `system-tests`  | System test runner      | `osrf/ros:humble-desktop`  |
| `gazebo-server` | Headless simulation     | `osrf/ros:humble-desktop`  |

---

## 5. Build & Test Commands

> **Note:** All commands below are run **inside the Docker container** unless otherwise specified.

### 5.1 Building

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

### 5.2 Testing

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

### 5.3 Code Quality

```bash
# Linting (ament_lint)
colcon test --packages-select <package> --ctest-args -R lint

# Format check (if configured)
ament_clang_format --check src/<package>
```

---

## 6. Adding Tests

> **Note:** All commands below are run **inside the Docker container**.

### 6.1 Adding a New Unit Test (C++)

1. Create test file in `src/<package>/test/test_<name>_logic.cpp` (test the logic class!)
2. Add to `CMakeLists.txt`:
   ```cmake
   ament_add_gtest(test_<name>_logic test/test_<name>_logic.cpp)
   target_include_directories(test_<name>_logic PRIVATE include)
   # Note: minimal dependencies - ideally NO ROS deps for logic tests
   ```
3. Run: `colcon test --packages-select <package>`

### 6.2 Adding a New Unit Test (Python)

1. Create test file in `src/<package>/test/test_<name>_logic.py`
2. Add to `CMakeLists.txt`:
   ```cmake
   ament_add_pytest_test(test_<name>_logic_py test/test_<name>_logic.py)
   ```
3. Run: `colcon test --packages-select <package>`

### 6.3 Adding a New Integration Test

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

### 6.4 Adding a New System Test

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
6. Test via Docker orchestration (from host): `cd test/system/docker && docker compose up --abort-on-container-exit`

---

## 7. Continuous Integration

### 7.1 CI/CD Platform

**Platform:** GitHub Actions

**Workflow Files:** `.github/workflows/`

| Workflow    | File          | Trigger                  | Purpose                    |
| ----------- | ------------- | ------------------------ | -------------------------- |
| **CI**      | `ci.yml`      | Push/PR to main, develop | Fast: Build + Lint + Tests |
| **Nightly** | `nightly.yml` | Daily 2 AM UTC + manual  | Slow: Full system tests    |

### 7.2 Test Execution by Trigger

| Trigger                    | Unit Tests | Integration Tests | System Tests           |
| -------------------------- | ---------- | ----------------- | ---------------------- |
| **Every commit/PR**        | ✅ All     | ✅ All            | ❌ Skip                |
| **Merge to main**          | ✅ All     | ✅ All            | ❌ Skip                |
| **Nightly build**          | ✅ All     | ✅ All            | ✅ All                 |
| **Pre-release**            | ✅ All     | ✅ All            | ✅ All (required)      |
| **Before hardware deploy** | ✅ All     | ✅ All            | ✅ All + manual review |

### 7.3 GitHub Actions Workflows

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

### 7.4 CI Pipeline Commands

All CI runs inside the ROS 2 container for consistency.

```bash
# Fast feedback (PR checks) - inside ci container
colcon test --packages-skip rc_racer_system_tests

# Full test suite (nightly) - inside ci container
colcon test

# System tests only (separate orchestration from host)
cd test/system/docker && docker compose up --abort-on-container-exit --exit-code-from system-tests
```

### 7.5 CI Docker Images

| Container      | Purpose            | Base Image                 |
| -------------- | ------------------ | -------------------------- |
| GitHub Actions | CI/CD pipeline     | `osrf/ros:humble-ros-base` |
| `ci` (local)   | Local CI testing   | `osrf/ros:humble-ros-base` |
| `system-tests` | System test runner | `osrf/ros:humble-desktop`  |

---

## 8. Dependencies

### 8.1 Test Dependencies (per package.xml)

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

### 8.2 Linter Configuration

**Files in repository root:**

| File          | Purpose                                               |
| ------------- | ----------------------------------------------------- |
| `.flake8`     | Python style checking (line length, ignores)          |
| `.pydocstyle` | Relaxed pep257 - requires docstrings, flexible format |

**pydocstyle configuration (relaxed):**

- ✅ **Enforced:** Docstrings for classes, methods, functions (D101, D102, D103)
- ❌ **Ignored:** Formatting nits (D100, D104, D105, D107, D200, D203, D205, D212, D400, D401, D415)

### 8.3 System Test Dependencies

All included in Docker images:

- Gazebo (simulation)
- ros2bag (replay testing)
- launch_testing (integration tests)

---

## 9. Security & Compliance

### 9.1 SBOM (Software Bill of Materials)

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

### 9.2 Vulnerability Scanning

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

### 9.3 C++ Dependency Tracking

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

### 9.4 Security Workflow Integration

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
