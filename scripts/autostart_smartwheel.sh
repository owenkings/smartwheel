#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_root"

launch_file="${SMARTWHEEL_LAUNCH_FILE:-full_system.launch.py}"
mode="${SMARTWHEEL_MODE:-real}"
ui_port="${SMARTWHEEL_UI_PORT:-8080}"
map_file="${SMARTWHEEL_MAP:-}"
auto_select_map="${SMARTWHEEL_AUTO_SELECT_MAP:-true}"
startup_localization_mode="${SMARTWHEEL_STARTUP_LOCALIZATION_MODE:-disabled}"
startup_named_goal_name="${SMARTWHEEL_STARTUP_NAMED_GOAL:-charging}"
startup_fixed_x="${SMARTWHEEL_STARTUP_FIXED_X:-0.0}"
startup_fixed_y="${SMARTWHEEL_STARTUP_FIXED_Y:-0.0}"
startup_fixed_yaw="${SMARTWHEEL_STARTUP_FIXED_YAW:-0.0}"
startup_anchor_topic="${SMARTWHEEL_STARTUP_ANCHOR_TOPIC:-/localization/anchor_pose}"
ros_domain_id="${ROS_DOMAIN_ID:-${SMARTWHEEL_ROS_DOMAIN_ID:-0}}"
build_on_start="${SMARTWHEEL_BUILD_ON_START:-false}"
setup_radar_network="${SMARTWHEEL_SETUP_RADAR_NETWORK:-true}"
# Whether to start the XT-M60 radar adapters at autostart. Default false so
# the radar stays in standby (powered, not scanning) and only scans on
# explicit demand. Set SMARTWHEEL_ENABLE_XTM60=true (e.g. via systemctl
# --user edit smartwheel.service) when mapping or autonomous navigation is
# needed.
enable_xtm60_radar="${SMARTWHEEL_ENABLE_XTM60:-false}"
launch_pid=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  bash "$workspace_root/scripts/hardware_shutdown.sh" --no-source --quiet || true
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" >/dev/null 2>&1; then
    kill -INT "$launch_pid" >/dev/null 2>&1 || true
    for _ in 1 2 3 4 5 6 7 8; do
      if ! kill -0 "$launch_pid" >/dev/null 2>&1; then
        break
      fi
      sleep 0.5
    done
    if kill -0 "$launch_pid" >/dev/null 2>&1; then
      kill -TERM "$launch_pid" >/dev/null 2>&1 || true
    fi
    wait "$launch_pid" >/dev/null 2>&1 || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found. Install/source ROS2 Humble first." >&2
  exit 1
fi

export ROS_DOMAIN_ID="$ros_domain_id"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_root/.ros/log}"
mkdir -p "$ROS_LOG_DIR"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

if [[ "$build_on_start" == true ]]; then
  colcon build --symlink-install
fi

if [[ "$setup_radar_network" == true ]]; then
  "$workspace_root/scripts/setup_radar_network.sh" --quiet --no-ping || \
    echo "WARN radar network alias was not applied; run 'sudo $workspace_root/scripts/setup_radar_network.sh --install-service'" >&2
fi

if [[ ! -f "$workspace_root/install/setup.bash" ]]; then
  echo "ERROR: install/setup.bash not found. Run colcon build first or set SMARTWHEEL_BUILD_ON_START=true." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source "$workspace_root/install/setup.bash"
set -u

if [[ -z "$map_file" && "$launch_file" == "full_system.launch.py" ]]; then
  case "$(echo "$auto_select_map" | tr '[:upper:]' '[:lower:]')" in
    true|1|yes|on)
      map_file="$(
        python3 -m wheelchair_ui.map_selection --workspace "$workspace_root" \
          2>/dev/null || true
      )"
      if [[ -n "$map_file" ]]; then
        echo "Auto-selected active Nav2 map: $map_file"
      fi
      ;;
  esac
fi

launch_args=()
case "$launch_file" in
  full_system.launch.py)
    launch_args+=("ui_port:=$ui_port")
    case "$(echo "$startup_localization_mode" | tr '[:upper:]' '[:lower:]')" in
      disabled|named_goal|fixed|external_anchor) ;;
      *)
        echo "WARN invalid SMARTWHEEL_STARTUP_LOCALIZATION_MODE; using disabled" >&2
        startup_localization_mode=disabled
        ;;
    esac
    launch_args+=(
      "startup_localization_mode:=$startup_localization_mode"
      "startup_named_goal_name:=$startup_named_goal_name"
      "startup_fixed_x:=$startup_fixed_x"
      "startup_fixed_y:=$startup_fixed_y"
      "startup_fixed_yaw:=$startup_fixed_yaw"
      "startup_anchor_topic:=$startup_anchor_topic"
    )
    # Normalize SMARTWHEEL_ENABLE_XTM60 (true|false|1|0|yes|no) to the
    # lowercase "true"/"false" that ROS launch IfCondition accepts. Anything
    # else falls back to false to keep the radar in standby by default.
    case "$(echo "$enable_xtm60_radar" | tr '[:upper:]' '[:lower:]')" in
      true|1|yes|on)  enable_xtm60_radar=true ;;
      *)              enable_xtm60_radar=false ;;
    esac
    launch_args+=("enable_xtm60_radar:=$enable_xtm60_radar")
    if [[ -n "$map_file" ]]; then
      launch_args+=("map:=$map_file")
    fi
    ;;
  sensors.launch.py|base.launch.py)
    launch_args+=("mode:=$mode")
    ;;
  mapping.launch.py)
    if [[ "$mode" == "mock" ]]; then
      launch_args+=("use_mock:=true")
    else
      launch_args+=("use_mock:=false")
    fi
    launch_args+=("use_rviz:=false" "enable_ui:=true" "ui_port:=$ui_port")
    ;;
esac

echo "SmartWheel autostart"
echo "  workspace: $workspace_root"
echo "  launch: wheelchair_bringup $launch_file ${launch_args[*]}"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  ROS_LOG_DIR: $ROS_LOG_DIR"

ros2 launch wheelchair_bringup "$launch_file" "${launch_args[@]}" &
launch_pid="$!"
wait "$launch_pid"
