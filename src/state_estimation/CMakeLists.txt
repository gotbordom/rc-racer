cmake_minimum_required(VERSION 3.8)
project(state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Logic library (pure C++, no ROS): motion models and filter
add_library(${PROJECT_NAME}_logic
  src/robot_frame.cpp
  src/extended_kalman_filter.cpp
  src/state_estimation_logic.cpp
)
target_include_directories(${PROJECT_NAME}_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_logic PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_logic Eigen3)

# Add executable targets here
# add_executable(state_estimation_node src/state_estimation_node.cpp)
# ament_target_dependencies(state_estimation_node rclcpp std_msgs sensor_msgs geometry_msgs)

# Install targets
# install(TARGETS
#   state_estimation_node
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS ${PROJECT_NAME}_logic
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

#===============================================================================
# Testing
#===============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # The repo uses SPDX headers, which ament_copyright does not recognise
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # -----------------------------------------------------------------------------
  # Unit Tests (C++ with GTest)
  # -----------------------------------------------------------------------------
  # Tests pure logic, math, state machines, parameter validation, callbacks
  # Run via: colcon test --packages-select state_estimation
  #
  # Example:
  # ament_add_gtest(test_example test/test_example.cpp)
  # target_include_directories(test_example PRIVATE include)
  # ament_target_dependencies(test_example rclcpp std_msgs sensor_msgs geometry_msgs)
  ament_add_gtest(test_robot_frame test/test_robot_frame.cpp)
  target_link_libraries(test_robot_frame ${PROJECT_NAME}_logic)
  ament_add_gtest(test_state_estimation_logic test/test_state_estimation_logic.cpp)
  target_link_libraries(test_state_estimation_logic ${PROJECT_NAME}_logic)

  # -----------------------------------------------------------------------------
  # Unit Tests (Python with pytest)
  # -----------------------------------------------------------------------------
  # Example:
  # ament_add_pytest_test(test_example_py test/test_example.py)

endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_package()

