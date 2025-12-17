# Edge Kit ROS2 Dockerfile
# Based on ROS2 Humble for Physical AI System Monitoring

FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-std-msgs \
    ros-humble-builtin-interfaces \
    ros-humble-launch \
    ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for edge_health
RUN pip3 install --no-cache-dir \
    psutil \
    pyyaml \
    pytest \
    flask

# Create workspace
WORKDIR /ros2_ws

# Copy the packages
COPY ros2_ws/src /ros2_ws/src

# Create resource directory for ament index
RUN mkdir -p /ros2_ws/src/edge_health/resource && \
    touch /ros2_ws/src/edge_health/resource/edge_health

# Build interfaces package first, then edge_health
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --packages-select edge_health_interfaces && \
    source install/setup.bash && \
    colcon build --packages-select edge_health --symlink-install"

# Source the workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
