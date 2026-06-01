#!/usr/bin/env bash
set -uo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/hardware_shutdown.sh [options]

Publish conservative stop commands for the wheelchair stack. When this is used
from a launch wrapper, the wrapper should terminate the ROS processes after the
commands are published so each node can release its hardware handle.

Options:
  --no-source   Assume ROS has already been sourced by the parent shell.
  --quiet       Suppress informational output.
  --no-direct-zlac-release
               Skip the direct Modbus stop fallback for ZLAC8030.
  -h, --help    Show this help.
EOF
}

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_ros=true
quiet=false
direct_zlac_release=true

while (($#)); do
  case "$1" in
    --no-source)
      source_ros=false
      shift
      ;;
    --quiet)
      quiet=true
      shift
      ;;
    --no-direct-zlac-release)
      direct_zlac_release=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

log() {
  if [[ "$quiet" != true ]]; then
    printf '%s\n' "$*"
  fi
}

source_ros_setup() {
  if [[ "$source_ros" != true ]]; then
    return 0
  fi
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    log "ROS Humble setup not found; skip ROS stop publishes"
    return 1
  fi
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${SMARTWHEEL_ROS_DOMAIN_ID:-0}}"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -f "$workspace_root/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$workspace_root/install/setup.bash"
  fi
  set -u
}

publish_once() {
  local topic="$1"
  local msg_type="$2"
  local payload="$3"
  timeout 2s ros2 topic pub --once "$topic" "$msg_type" "$payload" >/dev/null 2>&1 || true
}

publish_stop_commands() {
  if ! command -v ros2 >/dev/null 2>&1; then
    log "ros2 command not available; skip ROS stop publishes"
    return 0
  fi

  log "Publishing emergency stop and zero velocity commands"
  publish_once /emergency_stop_sw std_msgs/msg/Bool "{data: true}"
  publish_once /emergency_stop_command std_msgs/msg/String "{data: stop}"
  publish_once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  publish_once /cmd_vel_safe geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
}

publish_direct_zlac_release() {
  if [[ "$direct_zlac_release" != true ]]; then
    return 0
  fi
  if [[ ! -x "$workspace_root/scripts/zlac8030_release.py" ]]; then
    return 0
  fi
  if [[ ! -e /dev/smartwheel_zlac8030 ]]; then
    log "ZLAC8030 serial device not found; skip direct release"
    return 0
  fi
  log "Writing direct ZLAC8030 stop command"
  "$workspace_root/scripts/zlac8030_release.py" >/dev/null 2>&1 || \
    log "WARN direct ZLAC8030 stop failed; ROS base may own the serial port"
}

cd "$workspace_root" || exit 1
source_ros_setup || true
publish_stop_commands
publish_direct_zlac_release
