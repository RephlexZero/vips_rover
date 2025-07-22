#!/bin/bash

# Script to run the real rover hardware
echo "Starting Real Rover Hardware..."

# Source the workspace
source ~/ros2_ws/install/setup.bash

# Launch the real robot
ros2 launch rover_hardware_interface rover_system.launch.py

echo "Real robot launched. Use Ctrl+C to stop."
