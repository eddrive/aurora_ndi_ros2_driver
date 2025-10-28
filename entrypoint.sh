#!/bin/bash

set -e

# Set the variables for the X11 server
export DISPLAY=${DISPLAY:-":0"}
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-"xcb"}
export QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM:-"1"}

# Allow access to the X server
if command -v xhost &> /dev/null; then
    xhost +local:root
fi

# Source ROS2 Humble environment
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "Sourcing ROS2 Humble environment..."
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS2 Humble setup.bash file not found!"
    exit 1
fi

# Source the workspace environment
if [ -f /workspace/install/setup.bash ]; then
    echo "Sourcing custom workspace environment..."
    source /workspace/install/setup.bash
else
    echo "Error: ROS2 workspace has not been built correctly!"
    exit 1
fi

# Check for Aurora package
echo "Checking for installed packages..."
if ros2 pkg list 2>/dev/null | grep -q "^aurora_ndi_ros2_driver$"; then
    echo "✓ Package 'aurora_ndi_ros2_driver' found"
else
    echo "✗ Package 'aurora_ndi_ros2_driver' missing"
fi

# Setup permanent environment for bash sessions
echo "Setting up permanent ROS2 environment..."
cat >> /root/.bashrc << 'EOF'

# ROS2 Environment Setup
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Set ROS2 environment variables
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
EOF

echo "Starting Aurora NDI tracking system..."
ros2 launch aurora_ndi_ros2_driver aurora_pub.launch.py &

# Wait for Aurora initialization
echo "Waiting for Aurora initialization..."
sleep 3

# If no arguments provided, start an interactive bash shell with ROS2 sourced
if [ $# -eq 0 ]; then
    echo "Starting interactive shell with ROS2 environment..."
    exec /bin/bash
else
    # Pass control to any additional commands specified at runtime
    exec "$@"
fi
