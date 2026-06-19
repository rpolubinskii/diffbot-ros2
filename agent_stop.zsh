#!/usr/bin/env zsh
# agent_stop.zsh -- cleanly stop the running diffbot ROS 2 stack.
#
# Sends SIGINT to `ros2 launch` first (its designed graceful shutdown: it brings
# down all child nodes so they release /dev/rplidar, /dev/motor-controller, the
# imu, etc.). Only escalates to SIGTERM then SIGKILL of the whole process GROUP
# if it refuses to exit (SIGKILL can't be caught, so the parent can't propagate
# it -- killing the group avoids orphaned nodes left holding the serial devices).
#
# Safe to run when nothing is up (it's a no-op). Returns 0 once stopped, 1 if
# something still matches after escalation.
#
# Usage:   ./agent_stop.zsh
#          GRACE=20 ./agent_stop.zsh    # seconds to wait for graceful exit

set -e
setopt pipefail 2>/dev/null || true

LAUNCH_MATCH="ros2 launch diffbot diffbot.launch.py"
GRACE="${GRACE:-15}"

if ! pgrep -f "${LAUNCH_MATCH}" >/dev/null 2>&1; then
  echo "[stop] no running diffbot stack found"
  exit 0
fi

echo "[stop] SIGINT -> diffbot launch (graceful; releasing serial devices)..."
pkill -INT -f "${LAUNCH_MATCH}" || true

for i in {1..${GRACE}}; do
  pgrep -f "${LAUNCH_MATCH}" >/dev/null 2>&1 || break
  sleep 1
done

# Escalate on the whole process GROUP so child nodes don't orphan.
if pgrep -f "${LAUNCH_MATCH}" >/dev/null 2>&1; then
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

if pgrep -f "${LAUNCH_MATCH}" >/dev/null 2>&1; then
  echo "[stop] WARNING: processes still match '${LAUNCH_MATCH}':"
  pgrep -af "${LAUNCH_MATCH}" || true
  exit 1
fi
echo "[stop] diffbot stack stopped"
