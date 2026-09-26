#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/install/setup.bash"

launch_pid=""
cleanup() {
    local status=$?
    trap - EXIT INT TERM HUP
    if [[ -n "${launch_pid}" ]]; then
        # ros2 launch and all nodes run in this private process group.
        kill -TERM -- "-${launch_pid}" 2>/dev/null || true
        for _ in {1..20}; do
            kill -0 -- "-${launch_pid}" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL -- "-${launch_pid}" 2>/dev/null || true
        wait "${launch_pid}" 2>/dev/null || true
    fi
    exit "${status}"
}
trap cleanup EXIT INT TERM HUP

# Detach the launch subtree from the caller's process group so terminal and
# parent-process shutdowns can be handled deterministically by cleanup().
setsid ros2 launch lio_web lio_web.launch.py "$@" &
launch_pid=$!
wait "${launch_pid}" || true
