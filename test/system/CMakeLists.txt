cmake_minimum_required(VERSION 3.8)
project(rc_racer_system_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

#===============================================================================
# System / End-to-End Tests
#===============================================================================
#
# Characteristics:
#   - SLOW (minutes per test)
#   - BRITTLE (sensitive to timing, environment)
#   - EXTREMELY VALUABLE (catch real-world failures)
#
# What they validate:
#   - Startup order and initialization
#   - Failure modes and recovery
#   - Safety behavior (e-stop, watchdogs)
#   - Real-time-ish constraints
#
# When CI/CD runs them:
#   - Nightly builds
#   - Pre-release validation
#   - Before hardware deployment
#
# Run via:
#   colcon test --packages-select rc_racer_system_tests
#
# Or with Docker:
#   cd test/system/docker && docker-compose up --abort-on-container-exit
#
#===============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Linters specified in package.xml (excludes copyright - SPDX format not recognized)
  ament_lint_auto_find_test_dependencies()

  # ---------------------------------------------------------------------------
  # System Test: Full Stack Simulation
  # ---------------------------------------------------------------------------
  # Validates: Complete system in Gazebo with realistic sensors
  # Timeout: Long (5+ minutes)
  #
  # add_launch_test(
  #   launch/test_full_stack_sim.launch.py
  #   TARGET test_full_stack_sim
  #   TIMEOUT 300
  # )

  # ---------------------------------------------------------------------------
  # System Test: Emergency Stop
  # ---------------------------------------------------------------------------
  # Validates: E-stop triggers correctly, system halts safely
  # Timeout: Medium (2 minutes)
  #
  # add_launch_test(
  #   launch/test_emergency_stop.launch.py
  #   TARGET test_emergency_stop
  #   TIMEOUT 120
  # )

  # ---------------------------------------------------------------------------
  # System Test: Startup/Shutdown Sequence
  # ---------------------------------------------------------------------------
  # Validates: Correct startup order, clean shutdown, no zombies
  # Timeout: Medium (2 minutes)
  #
  # ament_add_pytest_test(
  #   test_startup_shutdown
  #   launch/test_startup_shutdown.py
  #   TIMEOUT 120
  # )

endif()

# Install launch files, configs, and worlds
install(DIRECTORY
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

