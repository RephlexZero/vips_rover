#!/bin/bash

# Script to check the URDF after updating rover specifications
echo "Checking URDF with updated rover specifications..."

# Source the workspace
source /home/jakestewart/ros2_ws/install/setup.bash

echo "=== URDF Validation ==="
# Check if the URDF is valid
xacro /home/jakestewart/ros2_ws/rover_hardware_interface/urdf/rover.urdf.xacro > /tmp/rover_test.urdf

if [ $? -eq 0 ]; then
    echo "✅ URDF syntax is valid"
    
    # Check URDF with urdf_parser
    if command -v check_urdf &> /dev/null; then
        echo "=== URDF Structure Check ==="
        check_urdf /tmp/rover_test.urdf
    else
        echo "⚠️  check_urdf not available, install with: sudo apt install liburdfdom-tools"
    fi
    
    echo ""
    echo "=== Key Dimensions ==="
    echo "Wheelbase: 567mm (0.567m)"
    echo "Track width: 435mm (0.435m)" 
    echo "Wheel diameter: 160mm (0.08m radius)"
    echo "Camber angle: 5° (0.0873 rad)"
    echo "Total gear ratio: 11:1"
    
    # Clean up
    rm -f /tmp/rover_test.urdf
else
    echo "❌ URDF has syntax errors"
    exit 1
fi

echo ""
echo "Ready to test! Build and run simulation with:"
echo "  ./scripts/build.sh"
echo "  ./scripts/run_simulation.sh"
