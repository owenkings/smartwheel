#!/usr/bin/env bash
# Start, validate, then explicitly arm autonomous frontier mapping.
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

sudo -n systemctl stop smartwheel.service 2>/dev/null \
  || systemctl --user stop smartwheel.service 2>/dev/null \
  || true

cat >&2 <<'WARN'
*********************************************************************
  HIGH-RISK AUTONOMOUS MAPPING. The wheelchair can move after live
  preflight passes and /autonomy/enable is asserted.
  - Clear at least 1.5 m around the chair. No passenger.
  - Keep the physical E-stop in reach and continuously supervise it.
  - Do not operate near stairs, ramps, glass, people, or traffic.
  Stop: bash scripts/stop_autonomous_mapping.sh (or Ctrl+C here)
*********************************************************************
WARN

if [[ "${SMARTWHEEL_ASSUME_AUTONOMOUS_RISK:-false}" != "true" ]]; then
  read -r -p "Type I_UNDERSTAND_AUTONOMOUS_MAPPING_RISK to continue: " answer
  if [[ "$answer" != "I_UNDERSTAND_AUTONOMOUS_MAPPING_RISK" ]]; then
    echo "Confirmation mismatch. Aborted." >&2
    exit 1
  fi
fi

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
  enable_motion:=true \
  autonomous_exploration:=true \
  exploration_mode:=frontier \
  require_enable_signal:=true \
  delete_db_on_start:=true \
  database_path:="$database_path" \
  max_linear_speed:=0.05 \
  max_angular_speed:=0.22 \
  turn_trigger_distance:=0.50 \
  rviz:=true "$@" &
launch_pid=$!

sleep "${SMARTWHEEL_AUTONOMY_STARTUP_WAIT_SEC:-20}"
preflight_ok=false
for attempt in $(seq 1 "${SMARTWHEEL_AUTONOMY_PREFLIGHT_ATTEMPTS:-3}"); do
  echo "Autonomy preflight attempt $attempt..."
  if bash scripts/check_autonomous_mapping_status.sh --require-motion-enabled; then
    preflight_ok=true
    break
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "ERROR: autonomous mapping launch exited during preflight." >&2
    exit 1
  fi
  sleep 5
done

if [[ "$preflight_ok" != true ]]; then
  echo "ERROR: preflight did not reach READY_TO_ARM. Motion remains disarmed." >&2
  exit 2
fi

timeout 2 ros2 topic pub --once /emergency_stop_command std_msgs/msg/String "{data: release}" >/dev/null
timeout 2 ros2 topic pub --once /autonomy/enable std_msgs/msg/Bool "{data: true}" >/dev/null
echo "AUTONOMY ARMED: frontier exploration is enabled."

set +e
wait "$launch_pid"
status=$?
launch_pid=""
set -e
exit "$status"
