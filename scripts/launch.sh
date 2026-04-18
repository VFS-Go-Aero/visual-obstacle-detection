#!/usr/bin/env bash
set -e

ENABLE_LOGGING=true
LOG_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --logging=*)
            ENABLE_LOGGING=${1#--logging=}
            shift
            ;;
        -h|--help)
            cat <<'EOF'
Usage: launch.sh [--logging=true|false] [-h|--help]

  --logging=true   enable ROS-level debug logging
  --logging=false  disable ROS-level debug logging (default)
  -h, --help       show this help message
EOF
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

if [ "$ENABLE_LOGGING" = "true" ]; then
    ros2 run latency_logger monitor_launcher
fi

ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
ros2 launch launch_files multi_zed.launch.py
ros2 launch visual_obstacle_detection visual_obstacle_detection.launch.py