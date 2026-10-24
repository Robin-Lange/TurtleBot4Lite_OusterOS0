#!/usr/bin/env bash
# Turnkey SSH Launcher: Autonomous Frontier SLAM Mapping & Auto Map Saver
# Usage over SSH on Jetson Orin Nano:
#   ./scripts/run_autonomous_mapping.sh [optional_map_prefix]
#
# Replaces the legacy 3D Ouster LiDAR map with a fresh 2D RPLIDAR S2 (z=0.118m)
# + RealSense D455 (z=0.176m) SLAM map.
# Press Ctrl+C at any time to stop exploration early — the map is automatically
# saved to <map_prefix>.pgm and <map_prefix>.yaml before exiting.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_PREFIX="${1:-${SCRIPT_DIR}/arena_map}"
INSTALL_MAPS_DIR="${SCRIPT_DIR}/ros2_ws/install/tb4_ouster_autonomy/share/tb4_ouster_autonomy/maps"

source "${SCRIPT_DIR}/setup_env.sh"

# Archive any previous/legacy map before starting a fresh SLAM mapping session
if [ -f "${MAP_PREFIX}.yaml" ] || [ -f "${MAP_PREFIX}.pgm" ]; then
    echo "[Auto-Map] Archiving previous map to ${MAP_PREFIX}.prev.[yaml|pgm]..."
    cp -f "${MAP_PREFIX}.yaml" "${MAP_PREFIX}.prev.yaml" 2>/dev/null || true
    cp -f "${MAP_PREFIX}.pgm" "${MAP_PREFIX}.prev.pgm" 2>/dev/null || true
    rm -f "${MAP_PREFIX}.yaml" "${MAP_PREFIX}.pgm"
fi

echo "=========================================================="
echo " TurtleBot 4 Lite: Fresh Autonomous SLAM Mapping (SSH)"
echo " Sensors           : RPLIDAR S2 (z=0.118m) + D455 (z=0.176m)"
echo " Target Map Output : ${MAP_PREFIX}.yaml & ${MAP_PREFIX}.pgm"
echo " Stop & Save Any Time: Press Ctrl+C"
echo "=========================================================="

CLEANUP_DONE=0
cleanup() {
    if [ "$CLEANUP_DONE" -eq 1 ]; then
        return
    fi
    CLEANUP_DONE=1
    trap - EXIT INT TERM
    echo ""
    echo "[Auto-Map] Stopping robot wheels and finalizing saved map..."
    if [ -n "${LAUNCH_PID:-}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        kill -INT "${LAUNCH_PID}" 2>/dev/null || true
        sleep 2
        kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    fi
    timeout 2 ros2 topic pub --once -w 0 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
    pkill -f "autonomous_explorer|component_container_isolated|async_slam_toolbox_node|realsense2_camera_node|rplidar_node|cloud_filter|bumper_contact_cloud" 2>/dev/null || true
    if [ -f "${MAP_PREFIX}.yaml" ] && [ -f "${MAP_PREFIX}.pgm" ]; then
        if [ -d "${INSTALL_MAPS_DIR}" ]; then
            cp -f "${MAP_PREFIX}.yaml" "${INSTALL_MAPS_DIR}/arena_map.yaml" 2>/dev/null || true
            cp -f "${MAP_PREFIX}.pgm" "${INSTALL_MAPS_DIR}/arena_map.pgm" 2>/dev/null || true
        fi
        echo "[Auto-Map] Successfully saved new RPLIDAR S2 + D455 map:"
        ls -lh "${MAP_PREFIX}.yaml" "${MAP_PREFIX}.pgm"
    else
        echo "[Auto-Map] WARNING: Map files were not generated yet (no /map received before exit)."
    fi
}
trap cleanup EXIT INT TERM

ros2 launch tb4_ouster_autonomy robot.launch.py \
    mode:=auto_map \
    rviz:=false \
    auto_undock:=true \
    map_output_prefix:="${MAP_PREFIX}" &
LAUNCH_PID=$!
wait "${LAUNCH_PID}" || true
