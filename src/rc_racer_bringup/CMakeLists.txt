# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(rc_racer_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Pad steering: stick fraction to yaw rate (SDP §3.6 VAL-014)
  ament_add_pytest_test(test_steering_relay test/test_steering_relay.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
    TIMEOUT 60
  )
endif()

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/joy_calibrate.py scripts/se_watch.py
  scripts/steering_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
