#!/bin/bash

# Script to make the rover drive forward and backward
echo "Making rover drive forward and back..."

# Source the workspace
source ~/ros2_ws/install/setup.bash

echo "Starting forward/back motion (Ctrl+C to stop)..."

# Trap Ctrl+C to send stop command before exiting
trap 'echo "Stopping rover..."; ros2 topic pub --once /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 0.0}}"; exit' INT

# Forward and backward motion
while true; do
    echo "Driving forward..."
    timeout 5s ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.0}}" --rate 10
    
    echo "Pausing..."
    timeout 2s ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 0.0}}" --rate 10
    
    echo "Driving backward..."
    timeout 5s ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: -1.0, steering_angle: 0.0}}" --rate 10
    
    echo "Pausing..."
    timeout 2s ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 0.0}}" --rate 10
done
