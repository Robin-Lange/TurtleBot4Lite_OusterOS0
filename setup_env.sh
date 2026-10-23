#!/usr/bin/env bash
# Source ROS 2 underlay and workspace overlay
source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/ros2_ws/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/ros2_ws/install/setup.bash"
fi

# Configure direct Domain 0 FastDDS
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/config/fastdds_create3.xml"
export ROS_LOCALHOST_ONLY=0

echo "[TB4 Lite] Environment loaded: ROS_DOMAIN_ID=0, FastDDS profile configured."
