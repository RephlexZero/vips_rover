#!/usr/bin/env bash
set -euo pipefail

# Minimal hardware entrypoint: source and launch either basic HW or nav HW.
# Mode is controlled by REAL_MODE env var: basic|nav (default: nav)

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    set +u  # Temporarily disable unbound variable check for ROS setup
    source /opt/ros/jazzy/setup.bash
    set -u  # Re-enable unbound variable check
fi

set +u  # Temporarily disable unbound variable check for workspace setup
source install/setup.bash
set -u  # Re-enable unbound variable check

MODE="${REAL_MODE:-nav}"

case "$MODE" in
    basic)
        echo "[real] Launching basic hardware..."
        exec ros2 launch rover_description hardware.launch.py
        ;;
    nav|navigation)
        echo "[real] Launching navigation hardware..."
        exec ros2 launch rover_navigation rover_nav_hw.launch.py
        ;;
    *)
        echo "Unknown REAL_MODE: $MODE (use 'basic' or 'nav')" >&2
        exit 2
        ;;
esac
