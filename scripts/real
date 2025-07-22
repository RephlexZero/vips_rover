#!/bin/bash

# Run rover on real hardware (no sudo required after setup)
echo "🚀 Starting Real Rover..."

# Source the workspace
source ~/ros2_ws/install/setup.bash

# Check if CAN interface is available
if ! ip link show can0 &>/dev/null; then
    echo "❌ CAN interface can0 not found."
    echo "   Make sure your CAN hardware is connected."
    echo "   Run './scripts/setup' if you haven't done system setup yet."
    exit 1
fi

# Check if CAN interface is up
if ! ip link show can0 | grep -q "UP"; then
    echo "⚠️  CAN interface can0 is down. Attempting to bring it up..."
    if sudo ip link set can0 up type can bitrate 125000 2>/dev/null; then
        echo "✅ CAN interface brought up"
    else
        echo "❌ Failed to bring up CAN interface."
        echo "   Run './scripts/setup' for automatic CAN setup."
        exit 1
    fi
else
    echo "✅ CAN interface can0 is ready"
fi

echo ""
echo "Starting rover system:"
echo "  • VIPS driver (/dev/ttyUSB0)"
echo "  • Hardware interface (CAN)"
echo "  • Controllers (Ackermann steering)"
echo ""
echo "Use Ctrl+C to stop."
echo ""

# Launch real hardware
ros2 launch rover_hardware_interface rover_hardware.launch.py
