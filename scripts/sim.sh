#!/bin/bash

#
# Rover navigation simulation launcher
# Always includes SLAM + Nav2 autonomous navigation
#

set -e

echo "🎮 Rover Navigation Simulation"
echo "=============================="
echo ""
echo "🧭 Starting Navigation Simulation"
echo "================================="
echo "📋 This will start:"
echo "  - Gazebo simulation with rover"
echo "  - SLAM mapping capability"
echo "  - Nav2 autonomous navigation"
echo "  - RViz with click-to-navigate interface"
echo ""
echo "💡 Navigation Tips:"
echo "  - Wait for SLAM to initialize (map should appear in RViz)"
echo "  - Use '2D Pose Estimate' to set initial robot pose"
echo "  - Use '2D Nav Goal' to set navigation targets"
echo "  - Click anywhere on the map to navigate there"
echo "  - Robot will avoid obstacles and plan optimal paths"
echo "  - Use teleop for manual control: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""

# Change to workspace root
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

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

if ! ros2 pkg list | grep -q rover_navigation; then
    echo "❌ rover_navigation package not found. Build the workspace first."
    exit 1
fi

echo "🚀 Launching navigation simulation..."
echo ""

# Launch navigation simulation
exec ros2 launch rover_navigation rover_nav_sim.launch.py
