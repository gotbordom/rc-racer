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

**Purpose:** Test communication between 2-3 nodes in isolation.

**Location:** `test/integration/` (project root)

**Run Command:**
```bash
colcon test --packages-select rc_racer_integration_tests
```

**Scope:**
- Topic pub/sub between nodes
- Service call/response
- Action client/server interactions
- Parameter synchronization

**Framework:** `launch_testing` with `ament_cmake_ros`

### 2.4 System / End-to-End Tests

**Purpose:** Full system validation in simulation.

**Location:** `test/system/` (project root)

**Scope:**
- Complete navigation scenarios
- Waypoint following
- Obstacle avoidance
- Safety system triggers

**Tools:**
- Gazebo simulation
- Recorded rosbags for replay testing
- Hardware-in-the-loop (HIL) when available

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

---

## 5. Continuous Integration

Tests should be run automatically on:
- Every pull request
- Every merge to main branch
- Nightly builds (full system tests)

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

