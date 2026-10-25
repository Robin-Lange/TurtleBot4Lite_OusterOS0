#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_YAML="${1:-${HOME}/maps/arena_map.yaml}"
if [ ! -f "$MAP_YAML" ]; then
    echo "Map not found: $MAP_YAML" >&2
    exit 1
fi
source "${SCRIPT_DIR}/setup_env.sh"
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
setsid ros2 launch tb4_ouster_autonomy robot.launch.py mode:=navigate rviz:=false map:="$MAP_YAML" &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
