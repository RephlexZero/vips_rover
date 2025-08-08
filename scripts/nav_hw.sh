#!/bin/bash

#
# Launch rover with autonomous navigation on hardware
# Includes SLAM, Nav2, VIPS, and sensor fusion for real-world navigation
#

set -e

echo "🧭 Starting Rover Navigation on Hardware"
echo "========================================"
echo "🎯 This will launch:"
echo "   • Hardware controllers with CAN bus"
echo "   • VIPS positioning system"
echo "   • Robot localization sensor fusion"
echo "   • SLAM mapping capability"
echo "   • Nav2 autonomous navigation"
echo "   • RViz with click-to-navigate interface"
echo ""

# Change to workspace root
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

# Check hardware prerequisites
echo "🔍 Checking hardware prerequisites..."

# Check CAN interface
if ! ip link show can0 >/dev/null 2>&1; then
    echo "❌ CAN interface 'can0' not found"
    echo "   Please run: sudo ./setup/can-setup.service"
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

# Source workspace
echo "📋 Sourcing workspace..."
source install/setup.bash

echo "🚀 Launching navigation system..."
echo ""
echo "💡 Hardware Navigation Tips:"
echo "   • System uses conservative speeds for safety"
echo "   • VIPS provides high-accuracy positioning indoors"
echo "   • Sensor fusion combines wheel odometry + VIPS + IMU"
echo "   • Use '2D Pose Estimate' if localization is lost"
echo "   • Use '2D Nav Goal' to set navigation targets"
echo "   • Emergency stop: Ctrl+C or use the control script"
echo ""

# Launch the complete navigation system
exec ros2 launch rover_navigation rover_nav_hw.launch.py
