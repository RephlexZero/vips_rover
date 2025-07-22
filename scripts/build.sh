#!/bin/bash

# Script to build the ROS2 workspace
echo "Building ROS2 workspace..."

cd ~/ros2_ws

# Build the workspace
colcon build

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "Build successful! 🎉"
    echo "Run 'source install/setup.bash' to use the workspace"
    echo "Or run the scripts which automatically source the workspace"
else
    echo ""
    echo "Build failed! ❌"
    echo "Check the error messages above"
    exit 1
fi
