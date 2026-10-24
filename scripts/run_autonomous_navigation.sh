#!/usr/bin/env bash
# Turnkey SSH Launcher: Autonomous Navigation (Nav2 + 3D RealSense Voxel Costmap + RPLIDAR S2)
# Usage over SSH on Jetson Orin Nano:
#   ./scripts/run_autonomous_navigation.sh [optional_path_to_map.yaml]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_YAML="${1:-${SCRIPT_DIR}/arena_map.yaml}"

if [ ! -f "${MAP_YAML}" ]; then
    echo "ERROR: Map file not found at ${MAP_YAML}"
    echo "Run ./scripts/run_autonomous_mapping.sh first to generate a map."
    exit 1
fi

source "${SCRIPT_DIR}/setup_env.sh"

echo "=========================================================="
echo " TurtleBot 4 Lite: Autonomous Navigation (Headless SSH)"
echo " Loaded Map : ${MAP_YAML}"
echo " Stop Any Time: Press Ctrl+C"
echo "=========================================================="

CLEANUP_DONE=0
cleanup() {
    if [ "$CLEANUP_DONE" -eq 1 ]; then
        return
    fi
    CLEANUP_DONE=1
    trap - EXIT INT TERM
    echo ""
    echo "[Navigate] Stopping robot wheels and shutting down Nav2 stack..."
    if [ -n "${LAUNCH_PID:-}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        kill -INT "${LAUNCH_PID}" 2>/dev/null || true
        sleep 2
        kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    fi
    timeout 2 ros2 topic pub --once -w 0 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
    pkill -f "component_container|realsense2_camera_node|rplidar_node|cloud_filter|bumper_contact_cloud" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch tb4_ouster_autonomy robot.launch.py \
    mode:=navigate \
    rviz:=false \
    map:="${MAP_YAML}" &
LAUNCH_PID=$!
wait "${LAUNCH_PID}" || true
