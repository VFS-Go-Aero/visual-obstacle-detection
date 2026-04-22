#!/usr/bin/env bash
set -e

PID_FILE="$(cd "$(dirname "$0")" && pwd)/launch.pids"

if [ ! -f "$PID_FILE" ]; then
    echo "No launch PID file found at $PID_FILE"
    exit 1
fi

pids=()
while IFS= read -r pid; do
    [ -n "$pid" ] && pids+=("$pid")
done < "$PID_FILE"

if [ ${#pids[@]} -eq 0 ]; then
    echo "No PIDs found in $PID_FILE"
    rm -f "$PID_FILE"
    exit 1
fi

echo "Stopping launch subprocesses: ${pids[*]}"
for pid in "${pids[@]}"; do
    if kill -0 -"$pid" 2>/dev/null; then
        echo "Sending SIGINT to process group $pid"
        kill -INT -"$pid" || true
    fi
done

sleep 2
for pid in "${pids[@]}"; do
    if kill -0 -"$pid" 2>/dev/null; then
        echo "Process group $pid still alive; sending SIGTERM"
        kill -TERM -"$pid" || true
    fi
done

sleep 2
for pid in "${pids[@]}"; do
    if kill -0 -"$pid" 2>/dev/null; then
        echo "Process group $pid still alive; sending SIGKILL"
        kill -9 -"$pid" || true
    fi
done

rm -f "$PID_FILE"
echo "Stopped launch subprocesses and removed $PID_FILE"
