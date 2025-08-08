#!/bin/bash

#
# Launch rover in pure simulation mode
# No hardware components - perfect for development and testing
#

set -e

echo "🎮 Starting Rover Simulation"
echo "============================"

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

echo "🚀 Launching simulation..."
echo ""
echo "This will start:"
echo "  - Gazebo simulation world"
echo "  - Rover robot model" 
echo "  - Robot state publisher"
echo "  - Controllers (ackermann_steering, joint_state_broadcaster)"
echo ""
echo "💡 Tips:"
echo "  - Open RViz2 to visualize: rviz2"
echo "  - Control with teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "  - Monitor topics: ros2 topic list"
echo ""

# Launch simulation
exec ros2 launch rover_description simulation.launch.py
