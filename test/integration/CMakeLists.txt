cmake_minimum_required(VERSION 3.8)
project(rc_racer_integration_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

#===============================================================================
# Integration Tests
#===============================================================================
# These tests launch multiple real nodes with real DDS to test ROS graphs.
#
# What integration tests verify:
#   - Topics appear on the graph
#   - Messages are published and received correctly
#   - State converges as expected
#   - Dead/crashed nodes are detected
#   - Inter-node communication works
#
# What integration tests do NOT verify:
#   - Individual node logic (use unit tests)
#   - Full system behavior (use system tests)
#
# Run via: colcon test --packages-select rc_racer_integration_tests
#===============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # ---------------------------------------------------------------------------
  # Integration Test: Sensor Pipeline
  # ---------------------------------------------------------------------------
  # Tests: state_estimation <-> localization data flow
  # Asserts: Topics published, messages valid, state converges
  #
  # add_launch_test(
  #   launch/test_sensor_pipeline.launch.py
  #   TARGET test_sensor_pipeline
  #   TIMEOUT 60
  # )

  # ---------------------------------------------------------------------------
  # Integration Test: Planning Pipeline
  # ---------------------------------------------------------------------------
  # Tests: localization -> planning_control -> motor_controller
  # Asserts: Commands flow through, safety checks engaged
  #
  # add_launch_test(
  #   launch/test_planning_pipeline.launch.py
  #   TARGET test_planning_pipeline
  #   TIMEOUT 60
  # )

  # ---------------------------------------------------------------------------
  # Integration Test: Safety System
  # ---------------------------------------------------------------------------
  # Tests: safety node monitors all critical nodes
  # Asserts: E-stop triggers on node death, watchdog works
  #
  # add_launch_test(
  #   launch/test_safety_system.launch.py
  #   TARGET test_safety_system
  #   TIMEOUT 60
  # )

  # ---------------------------------------------------------------------------
  # Integration Test: Full Graph Startup
  # ---------------------------------------------------------------------------
  # Tests: All nodes launch and connect
  # Asserts: All expected topics exist, no crashes on startup
  #
  # add_launch_test(
  #   launch/test_full_graph_startup.launch.py
  #   TARGET test_full_graph_startup
  #   TIMEOUT 120
  # )

endif()

# Install launch files for tests
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

