# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(rc_racer_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/joy_calibrate.py scripts/se_watch.py
  scripts/steering_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
