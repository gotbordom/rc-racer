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
# Or from the host (runs in the simulation image, same as nightly CI):
#   ./scripts/run-system-tests.sh
#
#===============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Linters specified in package.xml (excludes copyright - SPDX format not recognized)
  ament_lint_auto_find_test_dependencies()

  # ---------------------------------------------------------------------------
  # System Test: Simulation Smoke
  # ---------------------------------------------------------------------------
  # Validates: Gazebo starts headless, /clock advances, sensors publish, car drives
  # Timeout: Medium (Gazebo startup + software rendering on CI)
  #
  add_launch_test(
    launch/test_simulation_smoke.launch.py
    TARGET test_simulation_smoke
    TIMEOUT 300
  )

  # ---------------------------------------------------------------------------
  # System Test: Course World
  # ---------------------------------------------------------------------------
  # Validates: practice_course world matches its course file (cones, obstacles,
  #            GPS origin)
  # Timeout: Medium (Gazebo startup)
  #
  add_launch_test(
    launch/test_course_world.launch.py
    TARGET test_course_world
    TIMEOUT 300
  )

  # ---------------------------------------------------------------------------
  # Subsystem Test (L2): state_estimation in simulation
  # ---------------------------------------------------------------------------
  # Validates: /odom vs ground truth over a ~100 m loop of the practice course
  #            (rate, continuity, drift TBD-003, covariance honesty, health)
  # Timeout: Long (Gazebo startup + ~100 s of simulated driving)
  #
  add_launch_test(
    launch/test_gnc_state_estimation_sim.launch.py
    TARGET test_gnc_state_estimation_sim
    TIMEOUT 900
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/support
  )

  # ---------------------------------------------------------------------------
  # Scenario Tests: validation cases (SDP §3.6)
  # ---------------------------------------------------------------------------
  # Validates: one scripted validation case per scenarios/<name>.yaml, each in
  #            its own headless Gazebo (see scenarios/README.md)
  # Timeout: Medium (Gazebo startup + under a minute of sim time)
  #
  file(GLOB SCENARIO_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scenarios/*.yaml)
  foreach(scenario ${SCENARIO_FILES})
    get_filename_component(name ${scenario} NAME_WE)
    add_launch_test(
      launch/test_gnc_state_estimation_scenario.launch.py
      TARGET test_scenario_${name}
      TIMEOUT 300
      ENV SCENARIO_FILE=${scenario}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/support
    )
  endforeach()

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

