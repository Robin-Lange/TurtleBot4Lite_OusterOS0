#!/usr/bin/env bash
# Source ROS 2 underlay and workspace overlay
source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/ros2_ws/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/ros2_ws/install/setup.bash"
fi

# Configure Domain 0 Unicast FastDDS
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# Auto-select FastDDS unicast profile for AGX Orin vs. Robot Orin Nano
if ip -4 addr show 2>/dev/null | grep -q "192.168.0.10/"; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/config/fastdds_agx_orin.xml"
    echo "[TB4 Lite] AGX Orin Offboard Environment loaded: ROS_DOMAIN_ID=0, Unicast FastDDS (192.168.0.10)."
else
    export FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/config/fastdds_create3.xml"
    echo "[TB4 Lite] Robot Orin Nano Environment loaded: ROS_DOMAIN_ID=0, Unicast FastDDS (Create3 + AGX)."
fi
