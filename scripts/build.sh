#!/bin/bash

#
# Comprehensive rover ROS2 build script
# Builds packages in correct dependency order and validates the build
#

set -e  # Exit on any error

echo "🚀 Building Rover ROS2 Workspace"
echo "=================================="

# Change to workspace directory
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

echo "📁 Workspace: $WORKSPACE_ROOT"

# Check for ROS2 installation
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 is not sourced. Please run:"
    echo "   source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "🔧 ROS2 Distribution: $ROS_DISTRO"

# Install dependencies
echo ""
echo "📦 Installing dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Clean build (optional - uncomment if needed)
# echo ""
# echo "🧹 Cleaning previous build..."
# rm -rf build/ install/ log/

# Build packages in dependency order
echo ""
echo "🔨 Building packages in dependency order..."

# 1. First build core description package (no dependencies)
echo "   Building rover_description..."
colcon build --packages-select rover_description --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2. Then build VIPS driver (independent)
echo "   Building vips_driver..."
colcon build --packages-select vips_driver --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. Finally build hardware interface (depends on rover_description)
echo "   Building rover_hardware_interface..."
colcon build --packages-select rover_hardware_interface --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build all remaining packages
echo "   Building remaining packages..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "🔍 Build Summary:"
echo "=================="

# Check build results
if [ $? -eq 0 ]; then
    echo "✅ Build completed successfully"
    
    # Source the workspace
    source install/setup.bash
    
    # Validate critical packages
    echo ""
    echo "🔍 Validating packages..."
    
    # Check rover_description
    if ros2 pkg list | grep -q rover_description; then
        echo "✅ rover_description package found"
    else
        echo "❌ rover_description package not found"
    fi
    
    # Check rover_hardware_interface
    if ros2 pkg list | grep -q rover_hardware_interface; then
        echo "✅ rover_hardware_interface package found"
    else
        echo "❌ rover_hardware_interface package not found"
    fi
    
    # Check for URDF file
    URDF_PATH="$(ros2 pkg prefix rover_description)/share/rover_description/urdf/rover.urdf.xacro"
    if [ -f "$URDF_PATH" ]; then
        echo "✅ URDF file found at $URDF_PATH"
    else
        echo "❌ URDF file not found"
    fi
    
    # Check for launch files
    SIM_LAUNCH="$(ros2 pkg prefix rover_description)/share/rover_description/launch/simulation.launch.py"
    HW_LAUNCH="$(ros2 pkg prefix rover_description)/share/rover_description/launch/hardware.launch.py"
    
    if [ -f "$SIM_LAUNCH" ]; then
        echo "✅ Simulation launch file found"
    else
        echo "❌ Simulation launch file not found"
    fi
    
    if [ -f "$HW_LAUNCH" ]; then
        echo "✅ Hardware launch file found"
    else
        echo "❌ Hardware launch file not found"
    fi
    
    echo ""
    echo "🎉 Rover workspace built successfully!"
    echo ""
    echo "Quick start commands:"
    echo "  📋 Source workspace: source install/setup.bash"
    echo "  🎮 Run simulation:   ros2 launch rover_description simulation.launch.py"
    echo "  🤖 Run hardware:     ros2 launch rover_description hardware.launch.py"
    echo "  📖 Check URDF:       check_urdf \$(ros2 pkg prefix rover_description)/share/rover_description/urdf/rover.urdf.xacro"
    
else
    echo "❌ Build failed!"
    exit 1
fi
