#!/bin/bash

# Interactive rover control script
echo "=== Interactive Rover Control ==="
echo "Make sure the rover (simulation or real) is already running!"
echo ""

# Source the workspace
source /home/jakestewart/ros2_ws/install/setup.bash

# Function to send movement command
send_command() {
    local speed=$1
    local steering=$2
    ros2 topic pub --once /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: $speed, steering_angle: $steering}}"
}

# Main menu loop
while true; do
    echo ""
    echo "Choose a movement:"
    echo "1) Forward"
    echo "2) Backward"
    echo "3) Turn left while moving"
    echo "4) Turn right while moving"
    echo "5) Sharp left turn"
    echo "6) Sharp right turn"
    echo "7) Stop"
    echo "8) Drive in circle"
    echo "9) Custom speed/steering"
    echo "0) Exit"
    echo ""
    read -p "Enter choice: " choice

    case $choice in
        1)
            echo "Moving forward..."
            send_command 1.0 0.0
            ;;
        2)
            echo "Moving backward..."
            send_command -1.0 0.0
            ;;
        3)
            echo "Turning left while moving..."
            send_command 0.8 0.3
            ;;
        4)
            echo "Turning right while moving..."
            send_command 0.8 -0.3
            ;;
        5)
            echo "Sharp left turn..."
            send_command 0.5 0.6
            ;;
        6)
            echo "Sharp right turn..."
            send_command 0.5 -0.6
            ;;
        7)
            echo "Stopping..."
            send_command 0.0 0.0
            ;;
        8)
            echo "Starting circular motion (will run for 20 seconds)..."
            timeout 20s ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.8, steering_angle: 0.3}}" --rate 10
            echo "Stopping after circle..."
            send_command 0.0 0.0
            ;;
        9)
            read -p "Enter speed (-2.0 to 2.0): " custom_speed
            read -p "Enter steering angle (-0.7 to 0.7 radians): " custom_steering
            echo "Sending custom command: speed=$custom_speed, steering=$custom_steering"
            send_command $custom_speed $custom_steering
            ;;
        0)
            echo "Stopping rover and exiting..."
            send_command 0.0 0.0
            exit 0
            ;;
        *)
            echo "Invalid choice. Please try again."
            ;;
    esac
done
