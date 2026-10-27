#!/usr/bin/env bash
set -eo pipefail

source /home/ivlaborin2/TurtleBot4Lite_OusterOS0/setup_env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOGS_DIR="${EXP_DIR}/logs"
mkdir -p "${LOGS_DIR}"

CONDITION="$1"
RUN_ID="$2"
WARMUP_SEC="${3:-30}"
RUN_SEC="${4:-120}"

if [ -z "${CONDITION}" ] || [ -z "${RUN_ID}" ]; then
    echo "Usage: $0 <condition: D455_alone|RPLIDAR_alone|Both_concurrent> <run_id: 1|2|3> [warmup_sec] [run_sec]"
    exit 1
fi

echo "=========================================================="
echo "Starting Trial: Condition=${CONDITION}, Run=${RUN_ID}"
echo "Warmup: ${WARMUP_SEC}s, Run: ${RUN_SEC}s"
echo "=========================================================="

# Ensure no dangling processes
pkill -f realsense2_camera_node || true
pkill -f rplidar_node || true
sleep 2

case "${CONDITION}" in
    D455_alone)
        TOPICS="/camera/camera/color/image_raw,/camera/camera/aligned_depth_to_color/image_raw,/camera/camera/depth/image_rect_raw"
        ros2 launch tb4_ouster_autonomy perception.launch.py start_lidar:=false enable_pointcloud:=false > "${LOGS_DIR}/${CONDITION}_run${RUN_ID}_launch.log" 2>&1 &
        LAUNCH_PID=$!
        ;;
    RPLIDAR_alone)
        TOPICS="/scan"
        ros2 launch tb4_ouster_autonomy perception.launch.py start_camera:=false start_lidar:=true > "${LOGS_DIR}/${CONDITION}_run${RUN_ID}_launch.log" 2>&1 &
        LAUNCH_PID=$!
        ;;
    Both_concurrent)
        TOPICS="/camera/camera/color/image_raw,/camera/camera/aligned_depth_to_color/image_raw,/camera/camera/depth/image_rect_raw,/scan"
        ros2 launch tb4_ouster_autonomy perception.launch.py start_camera:=true start_lidar:=true enable_pointcloud:=false > "${LOGS_DIR}/${CONDITION}_run${RUN_ID}_launch.log" 2>&1 &
        LAUNCH_PID=$!
        ;;
    *)
        echo "Unknown condition: ${CONDITION}"
        exit 1
        ;;
esac

echo "Launched driver with PID ${LAUNCH_PID}. Waiting 5 seconds for node initialization..."
sleep 5

# Verify process is still alive
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "ERROR: Driver process died during initialization. Check log:"
    cat "${LOGS_DIR}/${CONDITION}_run${RUN_ID}_launch.log"
    exit 1
fi

OUTPUT_JSON="${LOGS_DIR}/${CONDITION}_run${RUN_ID}_summary.json"
OUTPUT_RAW_CSV="${LOGS_DIR}/${CONDITION}_run${RUN_ID}_raw.csv"
OUTPUT_TEGRA_CSV="${LOGS_DIR}/${CONDITION}_run${RUN_ID}_tegrastats.csv"

python3 "${SCRIPT_DIR}/monitor_acquisition.py" \
    --warmup-seconds "${WARMUP_SEC}" \
    --run-seconds "${RUN_SEC}" \
    --topics "${TOPICS}" \
    --output-json "${OUTPUT_JSON}" \
    --output-raw-csv "${OUTPUT_RAW_CSV}" \
    --output-tegrastats-csv "${OUTPUT_TEGRA_CSV}" \
    --condition-name "${CONDITION}_run${RUN_ID}"

echo "Monitoring completed. Shutting down driver..."
kill -2 "${LAUNCH_PID}" 2>/dev/null || kill -15 "${LAUNCH_PID}" 2>/dev/null || true
sleep 4
pkill -f realsense2_camera_node || true
pkill -f rplidar_node || true
sleep 3
echo "Completed Trial: Condition=${CONDITION}, Run=${RUN_ID}"
