# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.8)
project(rc_racer_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Install directories
install(DIRECTORY
  urdf
  meshes
  worlds
  config
  launch
  courses
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Python support code (sensor noise models) and the sensor adapter node
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/sensor_adapter_node.py DESTINATION lib/${PROJECT_NAME})

# Generate a Gazebo world for each course file (courses/<name>.yaml ->
# worlds/<name>.world). The course file is the source of truth; worlds are
# built from it using empty_course.world as the template.
set(COURSE_GENERATOR ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_course_world.py)
set(COURSE_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/worlds/empty_course.world)
file(GLOB COURSE_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/courses/*.yaml)
set(COURSE_WORLDS)
foreach(course_file ${COURSE_FILES})
  get_filename_component(course_name ${course_file} NAME_WE)
  set(world_file ${CMAKE_CURRENT_BINARY_DIR}/worlds/${course_name}.world)
  add_custom_command(
    OUTPUT ${world_file}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/worlds
    COMMAND ${Python3_EXECUTABLE} ${COURSE_GENERATOR}
      --course ${course_file} --template ${COURSE_TEMPLATE} --output ${world_file}
    DEPENDS ${COURSE_GENERATOR} ${COURSE_TEMPLATE} ${course_file}
    COMMENT "Generating Gazebo world for course ${course_name}"
  )
  list(APPEND COURSE_WORLDS ${world_file})
endforeach()
add_custom_target(course_worlds ALL DEPENDS ${COURSE_WORLDS})
install(FILES ${COURSE_WORLDS} DESTINATION share/${PROJECT_NAME}/worlds)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Course world generator: geodetic conversion, course validation, SDF output
  ament_add_pytest_test(test_generate_course_world test/test_generate_course_world.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
    TIMEOUT 60
  )

  # Sensor noise models used by the sensor adapter (DEV-02)
  ament_add_pytest_test(test_sensor_noise test/test_sensor_noise.py
    TIMEOUT 60
  )
endif()

ament_package()
