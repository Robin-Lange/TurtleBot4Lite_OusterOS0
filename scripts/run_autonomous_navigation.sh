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
echo "=========================================================="

ros2 launch tb4_ouster_autonomy robot.launch.py \
    mode:=navigate \
    rviz:=false \
    map:="${MAP_YAML}"
