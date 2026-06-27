#!/usr/bin/env bash
# Record the diffbot semantic-export bundle for DualMap bring-up / evaluation.
#
# Bring up the robot with the export enabled:
#   ros2 launch diffbot diffbot.launch.py enable_semantic_export:=true
# then run this on the PC (must see the robot's /dualmap/* topics on the ROS 2
# graph, e.g. same ROS_DOMAIN_ID over the LAN). The bundle is throttled +
# compressed and carries map-frame camera Odometry, so it replays directly into
# DualMap. /tf{,_static} are included for debugging / offline re-derivation.
#
# Usage: record_semantic_bag.sh [output_dir]
set -euo pipefail

OUT="${1:-semantic_eval_$(date +%Y%m%d_%H%M%S)}"

exec ros2 bag record -o "${OUT}" \
  /dualmap/color/image_raw/compressed \
  /dualmap/aligned_depth/image_raw/compressedDepth \
  /dualmap/color/camera_info \
  /dualmap/odom \
  /tf \
  /tf_static
