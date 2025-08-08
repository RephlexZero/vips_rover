#!/bin/bash

# Rover workspace help and status
echo "🚀 Rover ROS2 Workspace"
echo "======================="
echo ""
echo "Available commands:"
echo "  ./scripts/build    - Build the workspace"
echo "  ./scripts/setup    - One-time system setup (run once)"
echo "  ./scripts/sim      - Start navigation simulation with SLAM + Nav2"
echo "  ./scripts/real     - Start real hardware (basic or navigation modes)"
echo "  ./scripts/control  - Interactive rover control"
echo "  ./scripts/help     - Show this help"
echo ""

# Check workspace status
echo "Status:"
if [ -d ~/ros2_ws/install ]; then
    echo "  ✅ Workspace built"
else
    echo "  ❌ Workspace not built - run './scripts/build'"
fi

# Check if user is in dialout group
if groups $USER | grep -q dialout; then
    echo "  ✅ User in dialout group"
else
    echo "  ❌ User not in dialout group - run './scripts/setup'"
fi

# Check CAN interface
if ip link show can0 &>/dev/null; then
    if ip link show can0 | grep -q "UP"; then
        echo "  ✅ CAN interface ready"
    else
        echo "  ⚠️  CAN interface found but down"
    fi
else
    echo "  ℹ️  CAN interface not found (normal if no hardware)"
fi

# Check VIPS device
if [ -e /dev/ttyUSB0 ]; then
    echo "  ✅ VIPS device found (/dev/ttyUSB0)"
else
    echo "  ℹ️  VIPS device not found (normal if not connected)"
fi

echo ""
echo "📖 See README.md for detailed documentation"
