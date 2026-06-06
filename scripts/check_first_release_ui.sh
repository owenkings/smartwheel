#!/usr/bin/env bash
set -u

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

set +u
source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "BLOCKED: ros2 command not available"
  exit 2
fi

topics="$(ros2 topic list 2>/dev/null || true)"
actions="$(ros2 action list 2>/dev/null || true)"
nav_blockers=()

has_topic() {
  grep -Fxq "$1" <<<"$topics"
}

check_topic() {
  local topic="$1"
  local severity="${2:-warn}"
  if has_topic "$topic"; then
    printf 'OK: %s\n' "$topic"
  else
    printf 'WARN: missing %s\n' "$topic"
    if [[ "$severity" == "block" ]]; then
      nav_blockers+=("$topic")
    fi
  fi
}

if has_topic /rtabmap/grid_map; then
  echo "OK: /rtabmap/grid_map (preferred Web map)"
elif has_topic /map; then
  echo "OK: /map (fallback Web map)"
else
  echo "WARN: missing /rtabmap/grid_map and /map; Web UI will show map not ready"
  nav_blockers+=("map")
fi

check_topic /rtabmap/cloud_map
check_topic /points_merged
check_topic /rgb_cloud_map
check_topic /scan block
check_topic /imu/data
for index in 0 1 2 3; do
  check_topic "/ultrasonic/range_${index}"
done
check_topic /camera/left/image_raw
check_topic /camera/right/image_raw
check_topic /camera/left/camera_info
check_topic /camera/right/camera_info
check_topic /wheel/odom block
check_topic /base/status block
check_topic /safety_state block
check_topic /cmd_vel_safe block

if has_topic /navigation/preview_path || has_topic /plan || has_topic /global_plan || has_topic /received_global_plan; then
  echo "OK: navigation path topic"
else
  echo "WARN: missing navigation path topic"
fi

if grep -Fxq /navigate_to_pose <<<"$actions"; then
  echo "OK: /navigate_to_pose action server"
else
  echo "WARN: missing /navigate_to_pose action server"
  nav_blockers+=("/navigate_to_pose")
fi

echo "READY_FOR_WEB_UI"
if ((${#nav_blockers[@]} == 0)); then
  echo "READY_FOR_NAV_DEMO"
  exit 0
fi

printf 'BLOCKED: navigation demo requires %s\n' "$(IFS=', '; echo "${nav_blockers[*]}")"
exit 1
