#!/usr/bin/env bash

# Minimal simulation entrypoint: builds if needed, sources, launches sim+nav.

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"

# Source ROS setup (disable strict mode temporarily for setup scripts)
set +u
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi
# Set Gazebo plugin paths for gz_ros2_control
export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}:/opt/ros/jazzy/lib
set -u

# Build if missing install or when SIM_BUILD=1 is set
if [ ! -d install ] || [ "${SIM_BUILD:-0}" = "1" ]; then
    echo "[sim] Building rover_description and rover_navigation..."
    colcon build --packages-select rover_description rover_navigation --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_cohesion+
fi

set +u
source install/setup.bash
set -u

PROFILE="${SIM_PROFILE:-full}"
if [ "$PROFILE" = "empty" ]; then
    echo "[sim] Launching sim + nav (empty world, no SLAM)..."
    exec ros2 launch rover_navigation rover_nav_sim_empty_world.launch.py
else
    echo "[sim] Launching sim + nav..."
    exec ros2 launch rover_navigation rover_nav_sim.launch.py
fi
