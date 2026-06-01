#!/usr/bin/env bash
# Report autonomous-mapping readiness in two tiers:
#   READY_TO_PLAN  - all hard prerequisites present (map/sensors/Nav2/TF).
#   READY_TO_MOVE  - additionally /cmd_vel_nav and /cmd_vel_safe carry data
#                    (only expected AFTER the first exploration goal is sent).
# Missing /cmd_vel_* before the first goal is a WARN, not a BLOCKER.
set -uo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/.ros/log}"

reasons=()

have_msg() { timeout "${2:-4}" ros2 topic echo "$1" --once >/dev/null 2>&1; }
have_tf()  { timeout 4 ros2 run tf2_ros tf2_echo "$1" "$2" >/dev/null 2>&1; }

# A. Hard prerequisites -> READY_TO_PLAN
for t in /xtm60/left/points /xtm60/right/points /points_merged /imu/data \
         /rtabmap/odom /rtabmap/cloud_map /rtabmap/grid_map /scan \
         /safety_state /exploration/status; do
  if have_msg "$t" 4; then echo "OK   $t"; else echo "MISS $t"; reasons+=("no data on $t"); fi
done
if ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; then
  echo "OK   /navigate_to_pose action server"
else
  echo "MISS /navigate_to_pose action server"; reasons+=("/navigate_to_pose action server not available")
fi
if have_tf map odom; then echo "OK   TF map->odom"; else echo "MISS TF map->odom"; reasons+=("TF map->odom missing (RTAB-Map not localized)"); fi
if have_tf odom base_link; then echo "OK   TF odom->base_link"; else echo "MISS TF odom->base_link"; reasons+=("TF odom->base_link missing (icp_odometry)"); fi

# B. Motion evidence -> READY_TO_MOVE (WARN-only before first goal)
move_ready=true
for t in /cmd_vel_nav /cmd_vel_safe; do
  if have_msg "$t" 4; then echo "OK   $t"; else echo "WARN $t no messages yet; expected before first goal"; move_ready=false; fi
done

echo "----------------------------------------"
if [[ ${#reasons[@]} -ne 0 ]]; then
  echo "BLOCKED"
  for r in "${reasons[@]}"; do echo "  - $r"; done
  exit 2
fi
echo "READY_TO_PLAN"
if [[ "$move_ready" == true ]]; then
  echo "READY_TO_MOVE"
fi
exit 0
