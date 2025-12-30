# Dockerfile for RC Racer - RoboMagellan Competition
# Base image for Nvidia Jetson Nano with ROS2

FROM dustynv/ros:humble-pytorch-l4t-r35.4.1

# Set working directory
WORKDIR /workspace

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy project files
COPY . /workspace/

# Install Python dependencies (add requirements.txt as needed)
# RUN pip3 install -r requirements.txt

# Source ROS2 setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]

