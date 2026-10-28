#!/usr/bin/env bash
set -euo pipefail

SESSION="latency-logger"
DIRECTORY="$HOME/ros2_ws"
COMMAND="ros2 run latency_logger monitor_launcher"

if ! tmux has-session -t "=$SESSION" 2>/dev/null; then
    PANE=$(tmux new-session -d -s "$SESSION" -c "$DIRECTORY" \
        -P -F '#{pane_id}')

    tmux send-keys -t "$PANE" -l "$COMMAND"
    tmux send-keys -t "$PANE" Enter
fi

if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "=$SESSION"
else
    tmux attach-session -t "=$SESSION"
fi