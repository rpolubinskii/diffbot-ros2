#!/usr/bin/env zsh
# agent_stop.zsh -- cleanly stop the running diffbot ROS 2 stack.
#
# Sends SIGINT to `ros2 launch` first (its designed graceful shutdown: it brings
# down all child nodes so they release /dev/rplidar, /dev/motor-controller, the
# imu, etc.). Escalates to SIGTERM then SIGKILL of the whole process GROUP if the
# parent refuses to exit. THEN sweeps any surviving ROS node processes: ros2
# launch's children often sit in their OWN process groups, so they orphan even a
# group-kill of the parent and keep holding the serial devices (two rplidar_nodes
# on one port corrupts the next launch). Verifies BOTH parent and nodes are gone.
#
# Safe to run when nothing is up (it's a no-op). Returns 0 once stopped, 1 if
# something still matches after escalation.
#
# Usage:   ./agent_stop.zsh
#          GRACE=20 ./agent_stop.zsh    # seconds to wait for graceful exit

set -e
setopt pipefail 2>/dev/null || true

LAUNCH_MATCH="ros2 launch diffbot diffbot.launch.py"
# ros2 launch's child NODE processes (C++ and python nodes alike) run under the ROS
# lib dir. This script's argv is the script path -- NOT this string -- so pgrep -f
# never self-matches (and pgrep excludes its own pid). The ros2 daemon lives under
# .../bin/, not .../lib/, so it is not swept.
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
  # Escalate on the whole process GROUP if the launch parent refuses to exit.
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

# Backstop: child nodes frequently survive the parent (their own process groups),
# orphaned and still holding /dev/rplidar, the motor, and the imu -- which corrupts
# the next launch. The old logic never swept these. SIGINT then SIGKILL them.
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

# Verify BOTH the launch parent AND all node processes are gone.
if have_launch || [[ -n "$(node_pids)" ]]; then
  echo "[stop] WARNING: stack still present after escalation:"
  have_launch && pgrep -af "${LAUNCH_MATCH}"
  for p in $(node_pids); do ps -o pid=,cmd= -p "$p" 2>/dev/null | cut -c1-100; done
  exit 1
fi
echo "[stop] diffbot stack stopped (launch + nodes; serial devices released)"
