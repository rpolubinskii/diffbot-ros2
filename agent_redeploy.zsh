#!/usr/bin/env zsh
# agent_redeploy.zsh -- reliable, agent-invokable redeploy of the diffbot ROS 2
# stack ON THE JETSON. Designed for an on-robot agent that edited local files and
# needs those changes APPLIED and VERIFIED, without the slips the old redeploy.sh
# had (it pulled over local edits, used a stale-prone plain build, and relaunched
# without killing the running stack -> the SIFT change never took effect).
#
# What it does:
#   1. Builds with --symlink-install so launch/config/Python edits apply on
#      relaunch with NO rebuild step that can go stale (C++ still recompiles).
#   2. Kills any running stack FIRST (releases /dev/rplidar, motor, imu) and waits
#      for the devices to free.
#   3. Relaunches in the background, logging to a timestamped file (the same
#      timestamped-log format the bag log readers expect).
#   4. VERIFIES by echoing the rtabmap parameters that actually loaded, so the
#      agent can confirm its change took (catches the SIFT-type slip immediately).
#
# Usage:   ./agent_redeploy.zsh            # build local working tree + relaunch
#          PULL=1 ./agent_redeploy.zsh     # also git pull first (dev-host->Jetson flow)
#          VERIFY_WAIT=20 ./agent_redeploy.zsh   # seconds to wait before param check
#
# Returns 0 with the stack running in the background and the log path printed.
# Stop the stack with: pkill -f "ros2 launch diffbot diffbot.launch.py"

set -e
setopt pipefail 2>/dev/null || true

REPO_ROOT="${0:A:h}"                 # directory of this script
WS="${REPO_ROOT}/ros2_ws"
LAUNCH_MATCH="ros2 launch diffbot diffbot.launch.py"
VERIFY_WAIT="${VERIFY_WAIT:-18}"
LOG="${HOME}/diffbot_$(date +%F_%H%M%S).log"

echo "[redeploy] repo=${REPO_ROOT} ws=${WS}"

if [[ "${PULL:-0}" == "1" ]]; then
  echo "[redeploy] git pull (PULL=1)"
  git -C "${REPO_ROOT}" pull
fi

# 1. build (only our package; --symlink-install so config/launch/py changes are live)
echo "[redeploy] colcon build --symlink-install --packages-select diffbot"
cd "${WS}"
colcon build --symlink-install --packages-select diffbot

# 2. stop any running stack CLEANLY via agent_stop.zsh, which SIGINTs the launch
#    AND sweeps orphaned child node processes. A plain pkill on the launch match
#    leaves child nodes (rplidar_node, bt_navigator, camera, imu, ...) orphaned in
#    their own process groups, holding /dev/rplidar/motor/imu and corrupting the
#    relaunch (e.g. two rplidar_nodes contending for one serial port).
echo "[redeploy] stopping any running stack (agent_stop.zsh)..."
"${REPO_ROOT}/agent_stop.zsh"

# 3. relaunch in the background, tee'd to a timestamped log
echo "[redeploy] launching -> ${LOG}"
source "${WS}/install/setup.zsh"
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
nohup ros2 launch diffbot diffbot.launch.py >"${LOG}" 2>&1 &
LAUNCH_PID=$!
disown 2>/dev/null || true
echo "[redeploy] launch pid=${LAUNCH_PID}, waiting ${VERIFY_WAIT}s for params to load..."
sleep "${VERIFY_WAIT}"

# 4. verify: which rtabmap params actually loaded?
echo "[redeploy] ===== loaded rtabmap params (VERIFY your change took effect) ====="
grep -E 'Setting RTAB-Map parameter "(Kp/DetectorStrategy|Vis/FeatureType|Optimizer/Robust|Optimizer/Strategy|RGBD/OptimizeMaxError|RGBD/ProximityMaxGraphDepth|RGBD/ProximityOdomGuess|RGBD/ProximityPathFilteringRadius|Rtabmap/DetectionRate)"' "${LOG}" \
  || echo "[redeploy] (no rtabmap param lines yet -- increase VERIFY_WAIT or check ${LOG})"
echo "[redeploy] ====================================================================="
echo "[redeploy] stack running (pid ${LAUNCH_PID}); log: ${LOG}"
