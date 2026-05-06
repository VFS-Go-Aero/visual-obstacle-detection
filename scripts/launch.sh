#!/usr/bin/env bash
set -e

PID_FILE="$(cd "$(dirname "$0")" && pwd)/launch.pids"
ENABLE_LOGGING=true

while [ $# -gt 0 ]; do
    case "$1" in
        --logging=*)
            ENABLE_LOGGING=${1#--logging=}
            shift
            ;;
        -h|--help)
            cat <<'HELP'
Usage: launch.sh [--logging=true|false] [-h|--help]

  --logging=true   enable ROS-level debug logging
  --logging=false  disable ROS-level debug logging (default)
  -h, --help       show this help message
HELP
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

commands=()
if [ "$ENABLE_LOGGING" = "true" ]; then
    commands+=("ros2 run latency_logger monitor_launcher")
fi
commands+=("ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200")
commands+=("ros2 launch launch_files multi_zed.launch.py")
commands+=("ros2 launch visual_obstacle_detection visual_obstacle_detection.launch.py")

pids=()
cleanup() {
    echo "Shutting down subprocesses..."
    for pid in "${pids[@]:-}"; do
        if kill -0 -"$pid" 2>/dev/null; then
            kill -TERM -"$pid" || true
        fi
    done
    rm -f "$PID_FILE"
}
trap cleanup EXIT INT TERM

rm -f "$PID_FILE"
for cmd in "${commands[@]}"; do
    echo "Starting subprocess: $cmd"
    setsid bash -lc "$cmd" &
    pids+=("$!")
done

printf "%s\n" "${pids[@]}" > "$PID_FILE"

exit_code=0
for pid in "${pids[@]}"; do
    if ! wait "$pid"; then
        exit_code=$?
    fi
done

exit "$exit_code"
