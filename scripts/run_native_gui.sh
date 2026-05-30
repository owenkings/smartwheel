#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_root"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found" >&2
  exit 1
fi

if [[ ! -f "$workspace_root/install/setup.bash" ]]; then
  echo "ERROR: install/setup.bash not found. Run colcon build first." >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${SMARTWHEEL_ROS_DOMAIN_ID:-0}}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_root/.ros/log}"
mkdir -p "$ROS_LOG_DIR"

"$workspace_root/scripts/setup_radar_network.sh" --quiet --no-ping || \
  echo "WARN radar network alias was not applied; run 'sudo $workspace_root/scripts/setup_radar_network.sh --install-service'" >&2

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$workspace_root/install/setup.bash"
set -u

exec ros2 run wheelchair_ui wheelchair_native_gui "$@"
