#!/bin/bash

#
# Complete rover hardware launcher  
# Includes basic hardware control and optional autonomous navigation
#

set -e

echo "🤖 Rover Hardware Launcher"
echo "=========================="
echo ""
echo "Choose hardware mode:"
echo "  1) Basic hardware control (CAN + VIPS + controllers)"
echo "  2) Navigation hardware (includes SLAM + Nav2 + sensor fusion)"
echo "  q) Quit"
echo ""

while true; do
    read -p "Select option (1-2, q): " choice
    case $choice in
        1)
            echo ""
            echo "🚀 Starting Basic Hardware Control"
            echo "================================="
            echo "📋 This will start:"
            echo "  - CAN bus hardware interface"
            echo "  - VIPS positioning system"
            echo "  - Robot localization sensor fusion"
            echo "  - Controllers (ackermann_steering, joint_state_broadcaster)"
            echo ""
            echo "💡 Control Tips:"
            echo "  - Use './scripts/control.sh' for interactive control"
            echo "  - Monitor with: ros2 topic echo /ackermann_steering_controller/odometry"
            echo "  - Emergency stop: Ctrl+C"
            echo ""
            break
            ;;
        2)
            echo ""
            echo "🧭 Starting Navigation Hardware"
            echo "==============================" 
            echo "📋 This will start:"
            echo "  - Complete hardware stack (CAN + VIPS)"
            echo "  - Robot localization sensor fusion"
            echo "  - SLAM mapping capability"
            echo "  - Nav2 autonomous navigation"
            echo "  - RViz with click-to-navigate interface"
            echo ""
            echo "💡 Navigation Tips:"
            echo "  - System uses conservative speeds for safety"
            echo "  - VIPS provides high-accuracy positioning indoors"
            echo "  - Use '2D Pose Estimate' if localization is lost"
            echo "  - Use '2D Nav Goal' to set navigation targets"
            echo "  - Emergency stop: Ctrl+C"
            echo ""
            break
            ;;
        q|Q)
            echo "Exiting hardware launcher."
            exit 0
            ;;
        *)
            echo "Invalid option. Please select 1-2 or q."
            ;;
    esac
done

# Change to workspace root
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

echo "🔍 Checking hardware prerequisites..."

# Check CAN interface
if ! ip link show can0 >/dev/null 2>&1; then
    echo "❌ CAN interface 'can0' not found"
    echo "   Please run: sudo ./scripts/setup.sh"
    exit 1
fi

if ! ip link show can0 | grep -q "UP"; then
    echo "⚠️  CAN interface is down. Bringing it up..."
    sudo ip link set can0 up type can bitrate 500000 || {
        echo "❌ Failed to bring up CAN interface"
        exit 1
    }
fi

echo "✅ CAN interface ready"

# Check VIPS device (optional)
if [ -e /dev/ttyUSB0 ]; then
    echo "✅ VIPS device found (/dev/ttyUSB0)"
else
    echo "⚠️  VIPS device not found (navigation will use wheel odometry only)"
fi

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built. Run ./scripts/build.sh first"
    exit 1
fi

# Source workspace
echo "📋 Sourcing workspace..."
source install/setup.bash

# Check for required packages
if ! ros2 pkg list | grep -q rover_description; then
    echo "❌ rover_description package not found. Build the workspace first."
    exit 1
fi

if [ "$choice" == "2" ]; then
    if ! ros2 pkg list | grep -q rover_navigation; then
        echo "❌ rover_navigation package not found. Build the workspace first."
        exit 1
    fi
fi

echo "🚀 Launching hardware system..."
echo ""

# Launch appropriate mode
if [ "$choice" == "1" ]; then
    # Basic hardware
    exec ros2 launch rover_description hardware.launch.py
elif [ "$choice" == "2" ]; then
    # Navigation hardware
    exec ros2 launch rover_navigation rover_nav_hw.launch.py
fi
