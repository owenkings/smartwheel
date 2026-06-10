#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ ! -f install/setup.bash ]]; then
  echo "ERROR: install/setup.bash missing. Run: colcon build --symlink-install" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

stop_smartwheel_service() {
  if systemctl is-active --quiet smartwheel.service 2>/dev/null; then
    if ! sudo -n systemctl stop smartwheel.service 2>/dev/null; then
      echo "ERROR: smartwheel.service is active and could not be stopped non-interactively." >&2
      echo "Run: sudo systemctl stop smartwheel.service" >&2
      return 1
    fi
  fi
  if systemctl --user is-active --quiet smartwheel.service 2>/dev/null; then
    if ! systemctl --user stop smartwheel.service; then
      echo "ERROR: the user smartwheel.service is active and could not be stopped." >&2
      return 1
    fi
  fi
}

stop_smartwheel_service

cat >&2 <<'WARN'
*********************************************************************
  LEFT-LIDAR LAB AUTONOMOUS MAPPING DEMO
  RIGHT XT-M60 IS DISABLED / UNDER REPAIR
  MAX SPEED VERY LOW
  OFF-GROUND TEST FIRST
  PHYSICAL E-STOP REQUIRED

  No passenger. Continuous human supervision is mandatory.
  Stop: bash scripts/stop_autonomous_mapping.sh (or Ctrl+C here)
*********************************************************************
WARN

read -r -p "Type I_UNDERSTAND_LEFT_LIDAR_LAB_MAPPING_RISK to continue: " answer
if [[ "$answer" != "I_UNDERSTAND_LEFT_LIDAR_LAB_MAPPING_RISK" ]]; then
  echo "Confirmation mismatch. Aborted." >&2
  exit 1
fi

left_lidar_ip="${SMARTWHEEL_LEFT_LIDAR_IP:-192.168.0.101}"
echo "Checking left XT-M60 at $left_lidar_ip before starting the motion-capable stack..."
if ! ip route get "$left_lidar_ip" >/dev/null 2>&1; then
  echo "ERROR: no route to left XT-M60 $left_lidar_ip." >&2
  echo "Run: sudo bash scripts/setup_radar_network.sh" >&2
  exit 2
fi
if ! ping -c 3 -W 1 "$left_lidar_ip" >/dev/null 2>&1; then
  echo "ERROR: left XT-M60 $left_lidar_ip is unreachable." >&2
  echo "Check radar power, the Ethernet cable/switch, and the left-radar link LEDs." >&2
  echo "Then run: bash scripts/setup_radar_network.sh --check-only" >&2
  echo "The autonomous mapping stack was NOT started." >&2
  exit 2
fi
echo "Left XT-M60 network preflight passed."

launch_pid=""
database_path="${SMARTWHEEL_RTABMAP_DB:-$HOME/.ros/rtabmap.db}"
mkdir -p "$(dirname "$database_path")"
if [[ -f "$database_path" ]]; then
  backup_path="${database_path%.db}.backup-$(date +%Y%m%d-%H%M%S).db"
  cp --preserve=mode,timestamps "$database_path" "$backup_path"
  echo "Existing RTAB-Map database backed up to $backup_path"
fi
cleanup() {
  set +e
  timeout 2 ros2 topic pub --once /autonomy/enable std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1
  timeout 2 ros2 topic pub --once /emergency_stop_command std_msgs/msg/String "{data: stop}" >/dev/null 2>&1
  timeout 1 ros2 topic pub -r 10 /emergency_stop_sw std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null
    wait "$launch_pid" 2>/dev/null
  fi
  python3 scripts/zlac8030_release.py >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

ros2 launch wheelchair_3d_mapping autonomous_rviz_mapping.launch.py \
  hardware_profile:=left_lidar_lab \
  enable_motion:=true \
  autonomous_exploration:=true \
  exploration_mode:=reactive \
  require_enable_signal:=true \
  max_linear_speed:=0.03 \
  max_angular_speed:=0.18 \
  turn_trigger_distance:=0.60 \
  stop_on_safety_warning:=false \
  use_colorizer:=false \
  delete_db_on_start:=true \
  database_path:="$database_path" \
  rviz:=true "$@" &
launch_pid=$!

set +e
wait "$launch_pid"
status=$?
launch_pid=""
set -e
exit "$status"
