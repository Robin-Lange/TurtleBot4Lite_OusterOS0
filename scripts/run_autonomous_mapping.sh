#!/usr/bin/env bash
# Start a scoped ROS launch session; preserve existing maps until a replacement is saved.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_PREFIX="${1:-${HOME}/maps/arena_map}"
source "${SCRIPT_DIR}/setup_env.sh"
mkdir -p "$(dirname "$MAP_PREFIX")"
for extension in yaml pgm; do
    if [ -f "${MAP_PREFIX}.${extension}" ]; then
        cp -p "${MAP_PREFIX}.${extension}" "${MAP_PREFIX}.$(date +%Y%m%dT%H%M%S).${extension}"
    fi
done
cleanup() {
    trap - EXIT INT TERM
    timeout 2 ros2 service call /safety/emergency_stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
    if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
        for _ in {1..30}; do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid ros2 launch tb4_ouster_autonomy robot.launch.py mode:=auto_map rviz:=false \
    auto_undock:=true map_output_prefix:="$MAP_PREFIX" &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
