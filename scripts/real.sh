#!/bin/bash

#
# Launch rover hardware mode
# Includes CAN interface, VIPS driver, and sensor fusion
#

set -e

echo "🤖 Starting Rover Hardware"
echo "=========================="

# Change to workspace directory  
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built. Run ./scripts/build.sh first"
    exit 1
fi

# Source the workspace
echo "📋 Sourcing workspace..."
source install/setup.bash

# Check for required packages
if ! ros2 pkg list | grep -q rover_description; then
    echo "❌ rover_description package not found. Build the workspace first."
    exit 1
fi

if ! ros2 pkg list | grep -q rover_hardware_interface; then
    echo "❌ rover_hardware_interface package not found. Build the workspace first."
    exit 1
fi

if ! ros2 pkg list | grep -q vips_driver; then
    echo "❌ vips_driver package not found. Build the workspace first."
    exit 1
fi

# Check CAN interface
echo "🔍 Checking CAN interface..."
if ip link show can0 &>/dev/null; then
    echo "✅ CAN interface can0 found"
else
    echo "⚠️  CAN interface can0 not found. Run setup script first:"
    echo "   sudo ./scripts/setup.sh"
    echo ""
    echo "   Or manually configure:"
    echo "   sudo ip link set can0 up type can bitrate 500000"
fi

# Check VIPS serial port
VIPS_PORT="${VIPS_SERIAL_PORT:-/dev/ttyUSB0}"
if [ -e "$VIPS_PORT" ]; then
    echo "✅ VIPS serial port $VIPS_PORT found"
else
    echo "⚠️  VIPS serial port $VIPS_PORT not found"
    echo "   Connect VIPS device or set VIPS_SERIAL_PORT environment variable"
fi

echo ""
echo "🚀 Launching hardware interface..."
echo ""
echo "This will start:"
echo "  - Hardware CAN interface"
echo "  - VIPS indoor positioning driver"
echo "  - Robot state publisher"
echo "  - Controllers (ackermann_steering, joint_state_broadcaster)"
echo "  - Robot localization EKF (sensor fusion)"
echo ""
echo "💡 Tips:"
echo "  - Monitor CAN traffic: candump can0"
echo "  - Check VIPS data: ros2 topic echo /vips/odometry"
echo "  - View fused odometry: ros2 topic echo /odometry/filtered"
echo "  - Open RViz2: rviz2"
echo ""

# Launch hardware
exec ros2 launch rover_description hardware.launch.py vips_serial_port:="$VIPS_PORT"
