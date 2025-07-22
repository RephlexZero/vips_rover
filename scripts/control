#!/bin/bash

# Interactive rover control
echo "🎮 Rover Control"
echo "==============="
echo ""

# Source the workspace
source ~/ros2_ws/install/setup.bash

echo "Available movement patterns:"
echo "  1) Manual control (keyboard)"
echo "  2) Drive in circle"
echo "  3) Drive figure-8"
echo "  4) Drive forward/backward"
echo "  5) Emergency stop"
echo "  q) Quit"
echo ""

while true; do
    read -p "Select option (1-5, q): " choice
    case $choice in
        1)
            echo "Starting keyboard control... Use WASD keys, Ctrl+C to stop"
            sleep 2
            ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ackermann_steering_controller/reference_unstamped
            ;;
        2)
            echo "Driving in circle for 10 seconds..."
            timeout 10 ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.3}}" --rate 10
            echo "Circle complete."
            ;;
        3)
            echo "Driving figure-8 pattern..."
            # Left turn for 5 seconds
            timeout 5 ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.5}}" --rate 10 &
            sleep 5
            # Right turn for 10 seconds  
            timeout 10 ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: -0.5}}" --rate 10 &
            sleep 10
            # Left turn for 5 seconds
            timeout 5 ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.5}}" --rate 10 &
            sleep 5
            echo "Figure-8 complete."
            ;;
        4)
            echo "Forward 3s, backward 3s..."
            timeout 3 ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.0}}" --rate 10 &
            sleep 3
            timeout 3 ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: -1.0, steering_angle: 0.0}}" --rate 10 &
            sleep 3
            echo "Forward/backward complete."
            ;;
        5)
            echo "Emergency stop!"
            ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 0.0}}" --once
            echo "Rover stopped."
            ;;
        q|Q)
            echo "Exiting control menu."
            break
            ;;
        *)
            echo "Invalid option. Please select 1-5 or q."
            ;;
    esac
    echo ""
done
