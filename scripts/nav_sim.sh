#!/bin/bash

#
# Launch rover with autonomous navigation in simulation
# Includes SLAM, Nav2, and RViz for click-to-navigate
#

set -e

echo "🧭 Starting Rover Navigation in Simulation"
echo "==========================================="
echo "🎯 This will launch:"
echo "   • Gazebo simulation with rover"
echo "   • SLAM mapping capability"
echo "   • Nav2 autonomous navigation"
echo "   • RViz with click-to-navigate interface"
echo ""

# Change to workspace root
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

# Source workspace
echo "📋 Sourcing workspace..."
source install/setup.bash

echo "🚀 Launching navigation system..."
echo ""
echo "💡 Navigation Tips:"
echo "   • Wait for SLAM to initialize (map should appear in RViz)"
echo "   • Use '2D Pose Estimate' to set initial robot pose"
echo "   • Use '2D Nav Goal' to set navigation targets"
echo "   • Click anywhere on the map to navigate there"
echo "   • Robot will avoid obstacles and plan optimal paths"
echo ""

# Launch the complete navigation system
exec ros2 launch rover_navigation rover_nav_sim.launch.py
