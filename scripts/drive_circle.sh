#!/bin/bash

# Script to make the rover drive in a circle
echo "Making rover drive in a circle..."

# Source the workspace
source /home/jakestewart/ros2_ws/install/setup.bash

# Drive in a circle with 0.8 m/s forward speed and 0.3 rad steering angle
echo "Starting circular motion (Ctrl+C to stop)..."

# Trap Ctrl+C to send stop command before exiting
trap 'echo "Stopping rover..."; ros2 topic pub --once /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 0.0}}"; exit' INT

# Start circular motion
ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.8, steering_angle: 0.3}}" --rate 10
