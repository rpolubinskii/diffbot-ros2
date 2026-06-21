#!/usr/bin/env zsh

set -e
setopt pipefail 2>/dev/null || true

REPO_ROOT="${0:A:h}"
WS="${REPO_ROOT}/ros2_ws"
LAUNCH_MATCH="ros2 launch diffbot diffbot.launch.py"
VERIFY_WAIT="${VERIFY_WAIT:-18}"
LOG="${HOME}/diffbot_$(date +%F_%H%M%S).log"

echo "[redeploy] repo=${REPO_ROOT} ws=${WS}"

if [[ "${PULL:-0}" == "1" ]]; then
  echo "[redeploy] git pull (PULL=1)"
  git -C "${REPO_ROOT}" pull
fi

echo "[redeploy] colcon build --symlink-install --packages-select diffbot"
cd "${WS}"
colcon build --symlink-install --packages-select diffbot

echo "[redeploy] stopping any running stack (agent_stop.zsh)..."
"${REPO_ROOT}/agent_stop.zsh"

echo "[redeploy] launching -> ${LOG}"
source "${WS}/install/setup.zsh"
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
nohup ros2 launch diffbot diffbot.launch.py >"${LOG}" 2>&1 &
LAUNCH_PID=$!
disown 2>/dev/null || true
echo "[redeploy] launch pid=${LAUNCH_PID}, waiting ${VERIFY_WAIT}s for params to load..."
sleep "${VERIFY_WAIT}"

echo "[redeploy] ===== loaded rtabmap params (VERIFY your change took effect) ====="
grep -E 'Setting RTAB-Map parameter "(Kp/DetectorStrategy|Vis/FeatureType|Optimizer/Robust|Optimizer/Strategy|RGBD/OptimizeMaxError|RGBD/ProximityMaxGraphDepth|RGBD/ProximityOdomGuess|RGBD/ProximityPathFilteringRadius|Rtabmap/DetectionRate)"' "${LOG}" \
  || echo "[redeploy] (no rtabmap param lines yet -- increase VERIFY_WAIT or check ${LOG})"
echo "[redeploy] ====================================================================="
echo "[redeploy] stack running (pid ${LAUNCH_PID}); log: ${LOG}"
