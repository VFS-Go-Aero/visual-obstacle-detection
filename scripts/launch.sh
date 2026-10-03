#!/usr/bin/env bash
# Start the full system. Same as: ros2 launch launch_files launch_all.launch.py [rviz:=false]
# Stop it with: ros2 run launch_files stop
exec ros2 launch launch_files launch_all.launch.py "$@"
