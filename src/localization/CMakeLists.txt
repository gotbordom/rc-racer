cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Add executable targets here
# add_executable(localization_node src/localization_node.cpp)
# ament_target_dependencies(localization_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros)

# Install targets
# install(TARGETS
#   localization_node
#   DESTINATION lib/${PROJECT_NAME}
# )

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

  ament_lint_auto_find_test_dependencies()

  # -----------------------------------------------------------------------------
  # Unit Tests (C++ with GTest)
  # -----------------------------------------------------------------------------
  # Tests pure logic, math, state machines, parameter validation, callbacks
  # Run via: colcon test --packages-select localization
  #
  # Example:
  # ament_add_gtest(test_example test/test_example.cpp)
  # target_include_directories(test_example PRIVATE include)
  # ament_target_dependencies(test_example rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros)

  # -----------------------------------------------------------------------------
  # Unit Tests (Python with pytest)
  # -----------------------------------------------------------------------------
  # Example:
  # ament_add_pytest_test(test_example_py test/test_example.py)

endif()

ament_package()

