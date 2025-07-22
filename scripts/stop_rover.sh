#!/bin/bash

# Script to stop the rover immediately
echo "Stopping rover..."

# Source the workspace
source ~/ros2_ws/install/setup.bash

# Send stop command
ros2 topic pub --once /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 0.0}}"

echo "Stop command sent."
