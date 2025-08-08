#!/bin/bash

#
# Complete rover simulation launcher
# Includes basic simulation and optional autonomous navigation
#

set -e

echo "🎮 Rover Simulation Launcher"
echo "============================"
echo ""
echo "Choose simulation mode:"
echo "  1) Basic simulation (robot + controllers only)"
echo "  2) Navigation simulation (includes SLAM + Nav2 + RViz)"
echo "  q) Quit"
echo ""

while true; do
    read -p "Select option (1-2, q): " choice
    case $choice in
        1)
            echo ""
            echo "🚀 Starting Basic Rover Simulation"
            echo "=================================="
            echo "📋 This will start:"
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
            break
            ;;
        2)
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
            echo ""
            break
            ;;
        q|Q)
            echo "Exiting simulation launcher."
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

echo "🚀 Launching simulation..."
echo ""

# Launch appropriate mode
if [ "$choice" == "1" ]; then
    # Basic simulation
    exec ros2 launch rover_description simulation.launch.py
elif [ "$choice" == "2" ]; then
    # Navigation simulation
    exec ros2 launch rover_navigation rover_nav_sim.launch.py
fi
