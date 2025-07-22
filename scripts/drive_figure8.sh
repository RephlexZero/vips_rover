#!/bin/bash

# Script to make the rover drive in a figure-8 pattern
echo "Making rover drive in a figure-8 pattern..."

# Source the workspace
source /home/jakestewart/ros2_ws/install/setup.bash

echo "Starting figure-8 motion (Ctrl+C to stop)..."

# Trap Ctrl+C to send stop command before exiting
trap 'echo "Stopping rover..."; ros2 topic pub --once /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 0.0}}"; exit' INT

# Figure-8 motion: alternate between left and right turns
while true; do
    echo "Turning left..."
    timeout 8s ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.6, steering_angle: 0.4}}" --rate 10
    
    echo "Turning right..."
    timeout 8s ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.6, steering_angle: -0.4}}" --rate 10
done
