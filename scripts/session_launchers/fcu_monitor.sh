#!/usr/bin/env bash
set -euo pipefail

SESSION="fcu-monitor"
DIRECTORY="$HOME"
COMMAND="fcu_launch"

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