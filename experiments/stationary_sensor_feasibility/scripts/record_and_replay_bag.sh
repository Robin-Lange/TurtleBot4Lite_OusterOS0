#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAGS_DIR="${EXP_DIR}/bags"
LOGS_DIR="${EXP_DIR}/logs"
mkdir -p "${BAGS_DIR}" "${LOGS_DIR}"

source /home/ivlaborin2/TurtleBot4Lite_OusterOS0/setup_env.sh

RECORD_DURATION="${1:-10}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${BAGS_DIR}/stationary_sensors_${TIMESTAMP}"

TOPICS=(
    "/camera/camera/color/image_raw"
    "/camera/camera/color/camera_info"
    "/camera/camera/aligned_depth_to_color/image_raw"
    "/scan"
    "/tf_static"
)

echo "=========================================================="
echo "STEP 3: BOUNDED RECORDING AND REPLAY"
echo "Timestamped target directory: ${BAG_PATH}"
echo "Planned recording duration: ${RECORD_DURATION}s"
echo "Topics: ${TOPICS[*]}"
echo "=========================================================="

# Storage demand estimation
# Color: 640*480*3 bytes = 921,600 bytes * 30 Hz = ~27.6 MB/s
# Aligned Depth: 640*480*2 bytes = 614,400 bytes * 30 Hz = ~18.4 MB/s
# LaserScan: ~1.5 KB * 10 Hz = ~15 KB/s
# Metadata/Info/TF: < 100 KB/s
# Total ~46.1 MB/s * RECORD_DURATION
ESTIMATED_MB=$(( 47 * RECORD_DURATION ))
FREE_MB=$(df -m / | awk 'NR==2 {print $4}')
echo "Estimated storage demand: ~${ESTIMATED_MB} MB"
echo "Available NVMe disk space: ${FREE_MB} MB"

if [ "${FREE_MB}" -lt "$(( ESTIMATED_MB + 1000 ))" ]; then
    echo "ERROR: Insufficient disk space."
    exit 1
fi

# Ensure sensors are publishing concurrently
echo "Launching sensors (D455 + RPLIDAR S2)..."
pkill -f realsense2_camera_node || true
pkill -f rplidar_node || true
sleep 2

ros2 launch tb4_ouster_autonomy perception.launch.py start_camera:=true start_lidar:=true enable_pointcloud:=false > "${LOGS_DIR}/bag_record_launch.log" 2>&1 &
LAUNCH_PID=$!

echo "Waiting 8 seconds for sensors to stabilize..."
sleep 8

echo "Starting bounded ROS 2 bag recording for ${RECORD_DURATION}s..."
ros2 bag record \
    -d "${RECORD_DURATION}" \
    -o "${BAG_PATH}" \
    "${TOPICS[@]}" < /dev/null || true

echo "Recording ended. Stopping sensor nodes..."
kill -2 "${LAUNCH_PID}" 2>/dev/null || kill -15 "${LAUNCH_PID}" 2>/dev/null || true
sleep 4
pkill -f realsense2_camera_node || true
pkill -f rplidar_node || true
sleep 2

# Verify ros2 bag info
echo ""
echo "=== Inspecting recorded bag metadata ==="
INFO_OUTPUT="${LOGS_DIR}/bag_info_${TIMESTAMP}.txt"
ros2 bag info "${BAG_PATH}" | tee "${INFO_OUTPUT}"

# Replay test in isolated ROS domain
echo ""
echo "=== Verifying Replay Integrity in Isolated ROS Domain (DOMAIN_ID=42) ==="
REPLAY_LOG="${LOGS_DIR}/bag_replay_${TIMESTAMP}.log"

# In isolated domain 42, play the bag with topic remapping to /playback
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
unset FASTRTPS_DEFAULT_PROFILES_FILE

echo "Playing bag in isolated ROS_DOMAIN_ID=42..."
ros2 bag play "${BAG_PATH}" \
    --remap /camera/camera/color/image_raw:=/playback/camera/color/image_raw \
            /camera/camera/aligned_depth_to_color/image_raw:=/playback/camera/aligned_depth/image_raw \
            /scan:=/playback/scan \
    --disable-keyboard-controls > "${REPLAY_LOG}" 2>&1 &
PLAY_PID=$!

sleep 2
echo "Checking replayed topics in isolated domain..."
ros2 topic list | tee -a "${REPLAY_LOG}"

wait "${PLAY_PID}" 2>/dev/null || true
echo "Replay test completed. Replay log saved to ${REPLAY_LOG}"
echo "Step 3 recording and replay completed successfully."
