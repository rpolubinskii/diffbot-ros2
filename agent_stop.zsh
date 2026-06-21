#!/usr/bin/env zsh

set -e
setopt pipefail 2>/dev/null || true

LAUNCH_MATCH="ros2 launch diffbot diffbot.launch.py"
# ROS node executables live under the ROS lib dir; this avoids matching the script.
NODE_MATCH="/opt/ros/humble/lib/"
GRACE="${GRACE:-15}"

have_launch() { pgrep -f "${LAUNCH_MATCH}" >/dev/null 2>&1; }
node_pids()   { pgrep -f "${NODE_MATCH}" 2>/dev/null | grep -vx "$$" || true; }

if ! have_launch && [[ -z "$(node_pids)" ]]; then
  echo "[stop] no running diffbot stack or orphaned nodes found"
  exit 0
fi

if have_launch; then
  echo "[stop] SIGINT -> diffbot launch (graceful; releasing serial devices)..."
  pkill -INT -f "${LAUNCH_MATCH}" || true
  for ((i=0; i<${GRACE}; i++)); do
    have_launch || break
    sleep 1
  done
  if have_launch; then
    echo "[stop] still up after ${GRACE}s; escalating to SIGTERM/SIGKILL on the process group..."
    for pid in $(pgrep -f "${LAUNCH_MATCH}"); do
      pgid=$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')
      [[ -n "${pgid}" ]] && kill -TERM -- "-${pgid}" 2>/dev/null || true
    done
    sleep 3
    for pid in $(pgrep -f "${LAUNCH_MATCH}"); do
      pgid=$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')
      [[ -n "${pgid}" ]] && kill -KILL -- "-${pgid}" 2>/dev/null || true
    done
    sleep 2
  fi
fi

orphans="$(node_pids)"
if [[ -n "${orphans}" ]]; then
  echo "[stop] sweeping orphaned node processes: $(echo ${orphans} | tr '\n' ' ')"
  echo "${orphans}" | xargs -r kill -INT 2>/dev/null || true
  sleep 2
  orphans="$(node_pids)"
  if [[ -n "${orphans}" ]]; then
    echo "${orphans}" | xargs -r kill -KILL 2>/dev/null || true
    sleep 1
  fi
fi

if have_launch || [[ -n "$(node_pids)" ]]; then
  echo "[stop] WARNING: stack still present after escalation:"
  have_launch && pgrep -af "${LAUNCH_MATCH}"
  for p in $(node_pids); do ps -o pid=,cmd= -p "$p" 2>/dev/null | cut -c1-100; done
  exit 1
fi
echo "[stop] diffbot stack stopped (launch + nodes; serial devices released)"
