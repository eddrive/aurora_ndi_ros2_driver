FROM osrf/ros:humble-desktop

# Install essential development tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    nano \
    vim \
    tree \
    htop \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS2 debugging and visualization tools
RUN apt-get update && apt-get install -y \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-graph \
    ros-humble-rqt-topic \
    ros-humble-tf2-tools \
    ros-humble-rqt-tf-tree \
    ros-humble-rviz2 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Aurora NDI driver dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    libserial-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create ROS2 workspace
ENV COLCON_WS=/workspace
RUN mkdir -p $COLCON_WS/src

WORKDIR $COLCON_WS

# Copy Aurora driver package
COPY resources/aurora_pub $COLCON_WS/src/aurora_pub

# Install ROS2 dependencies
RUN rosdep update && \
    rosdep install --ignore-src --from-paths src -y --rosdistro humble

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Copy and setup entrypoint script
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Set the default entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Default command
CMD ["bash"]