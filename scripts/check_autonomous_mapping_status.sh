#!/usr/bin/env bash
# Report whether the autonomous mapping stack is ready, or BLOCKED with reasons.
set -uo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/.ros/log}"

reasons=()

have_msg() { timeout "${2:-4}" ros2 topic echo "$1" --once >/dev/null 2>&1; }
have_tf()  { timeout 4 ros2 run tf2_ros tf2_echo "$1" "$2" >/dev/null 2>&1; }

for t in /xtm60/left/points /xtm60/right/points /points_merged /imu/data \
         /rtabmap/odom /rtabmap/cloud_map /rtabmap/grid_map /scan \
         /cmd_vel_nav /cmd_vel_safe /safety_state; do
  if have_msg "$t" 4; then echo "OK   $t"; else echo "MISS $t"; reasons+=("no data on $t"); fi
done

if ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; then
  echo "OK   /navigate_to_pose action server"
else
  echo "MISS /navigate_to_pose action server"; reasons+=("/navigate_to_pose action server not available")
fi

if have_tf map odom; then echo "OK   TF map->odom"; else echo "MISS TF map->odom"; reasons+=("TF map->odom missing (RTAB-Map not localized)"); fi
if have_tf odom base_link; then echo "OK   TF odom->base_link"; else echo "MISS TF odom->base_link"; reasons+=("TF odom->base_link missing (base driver / icp odom)"); fi

have_msg /exploration/status 4 && echo "OK   /exploration/status" || echo "WARN /exploration/status (explorer idle or motion disabled)"
have_msg /exploration/frontiers 4 && echo "OK   /exploration/frontiers" || echo "WARN /exploration/frontiers (none yet)"

echo "----------------------------------------"
if [[ ${#reasons[@]} -eq 0 ]]; then
  echo "READY_TO_EXPLORE"
  exit 0
fi
echo "BLOCKED"
for r in "${reasons[@]}"; do echo "  - $r"; done
exit 2
