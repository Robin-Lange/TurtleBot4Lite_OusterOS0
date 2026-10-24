#!/usr/bin/env bash
# Turnkey SSH Launcher: Autonomous Frontier SLAM Mapping & Auto Map Saver
# Usage over SSH on Jetson Orin Nano:
#   ./scripts/run_autonomous_mapping.sh [optional_map_prefix]
#
# Press Ctrl+C at any time to stop exploration early — the map is automatically
# saved to <map_prefix>.pgm and <map_prefix>.yaml before exiting.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_PREFIX="${1:-${SCRIPT_DIR}/arena_map}"

source "${SCRIPT_DIR}/setup_env.sh"

echo "=========================================================="
echo " TurtleBot 4 Lite: Autonomous SLAM Mapping (Headless SSH)"
echo " Target Map Output : ${MAP_PREFIX}.yaml / ${MAP_PREFIX}.pgm"
echo " Stop & Save Any Time: Press Ctrl+C"
echo "=========================================================="

cleanup() {
    echo ""
    echo "[Auto-Map] Caught termination signal — ensuring robot is stopped and map is saved..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
    if [ -f "${MAP_PREFIX}.yaml" ] && [ -f "${MAP_PREFIX}.pgm" ]; then
        echo "[Auto-Map] Verified saved map files:"
        ls -lh "${MAP_PREFIX}.yaml" "${MAP_PREFIX}.pgm"
    fi
}
trap cleanup EXIT INT TERM

ros2 launch tb4_ouster_autonomy robot.launch.py \
    mode:=auto_map \
    rviz:=false \
    auto_undock:=true \
    map_output_prefix:="${MAP_PREFIX}"
