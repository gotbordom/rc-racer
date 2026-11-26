cmake_minimum_required(VERSION 3.8)
project(state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Logic library (pure C++, no ROS): motion models and filter
add_library(${PROJECT_NAME}_logic
  src/robot_frame.cpp
  src/speed_history.cpp
  src/extended_kalman_filter.cpp
  src/state_estimation_logic.cpp
)
target_include_directories(${PROJECT_NAME}_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_logic PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_logic Eigen3)

# Message conversions and health rules (ROS message types, no node)
add_library(${PROJECT_NAME}_conversions
  src/message_conversions.cpp
)
target_link_libraries(${PROJECT_NAME}_conversions ${PROJECT_NAME}_logic)
ament_target_dependencies(${PROJECT_NAME}_conversions
  builtin_interfaces geometry_msgs nav_msgs sensor_msgs diagnostic_msgs)

# Lifecycle node (thin adapter)
add_executable(state_estimation_node
  src/state_estimation_node.cpp
  src/state_estimation_main.cpp
)
target_link_libraries(state_estimation_node ${PROJECT_NAME}_conversions)
ament_target_dependencies(state_estimation_node
  rclcpp rclcpp_lifecycle lifecycle_msgs nav_msgs sensor_msgs
  diagnostic_msgs diagnostic_updater std_srvs tf2 tf2_ros)

install(TARGETS ${PROJECT_NAME}_logic ${PROJECT_NAME}_conversions
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS state_estimation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

#===============================================================================
# Testing
#===============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # The repo uses SPDX headers, which ament_copyright does not recognise
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # -----------------------------------------------------------------------------
  # L4 unit tests (GTest, no ROS spinning)
  # -----------------------------------------------------------------------------
  # Run via: colcon test --packages-select state_estimation
  ament_add_gtest(test_robot_frame test/test_robot_frame.cpp)
  target_link_libraries(test_robot_frame ${PROJECT_NAME}_logic)
  ament_add_gtest(test_speed_history test/test_speed_history.cpp)
  target_link_libraries(test_speed_history ${PROJECT_NAME}_logic)
  ament_add_gtest(test_spread_correction test/test_spread_correction.cpp)
  target_link_libraries(test_spread_correction ${PROJECT_NAME}_logic)
  ament_add_gtest(test_state_estimation_logic test/test_state_estimation_logic.cpp)
  target_link_libraries(test_state_estimation_logic ${PROJECT_NAME}_logic)
  ament_add_gtest(test_message_conversions test/test_message_conversions.cpp)
  target_link_libraries(test_message_conversions ${PROJECT_NAME}_conversions)

  # -----------------------------------------------------------------------------
  # L3 component test (launch_testing: the node with stub inputs)
  # -----------------------------------------------------------------------------
  add_launch_test(test/test_state_estimation_node.py TIMEOUT 120)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 builtin_interfaces geometry_msgs
  nav_msgs sensor_msgs diagnostic_msgs)
ament_package()
