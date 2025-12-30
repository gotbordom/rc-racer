# Dockerfile for RC Racer System Tests
# 
# Includes: ROS 2 Humble + Gazebo + Full test dependencies
#
# Build:
#   docker build -t rc-racer-system-tests .
#
# Run:
#   docker-compose up --abort-on-container-exit

FROM osrf/ros:humble-desktop

# Avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo and simulation dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ros \
    ros-humble-launch-testing-ament-cmake \
    python3-pip \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws

# Copy source code
COPY src/ /ros2_ws/src/rc-racer/src/
COPY test/ /ros2_ws/src/rc-racer/test/
COPY launch/ /ros2_ws/src/rc-racer/launch/

# Build packages
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source workspace on container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command runs system tests
CMD ["/bin/bash", "-c", \
    "source /opt/ros/humble/setup.bash && \
     source /ros2_ws/install/setup.bash && \
     colcon test --packages-select rc_racer_system_tests && \
     colcon test-result --verbose"]

