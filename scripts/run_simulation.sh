#!/bin/bash

# Script to run the rover in Gazebo simulation
echo "Starting Rover Simulation..."

# Source the workspace
source ~/ros2_ws/install/setup.bash

# Launch the simulation
ros2 launch rover_hardware_interface rover_simulation.launch.py

echo "Simulation launched. Use Ctrl+C to stop."
