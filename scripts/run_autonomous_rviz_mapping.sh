#!/usr/bin/env bash
# Launch HIGH-RISK autonomous RViz exploration 3D mapping. The wheelchair MAY MOVE.
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ ! -f install/setup.bash ]]; then
  echo "ERROR: install/setup.bash missing. Run: colcon build --symlink-install" >&2
  exit 1
fi
# ROS setup files reference unset vars; relax -u only while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

# Avoid duplicate hardware nodes from the autostart service (non-interactive).
sudo -n systemctl stop smartwheel.service 2>/dev/null || systemctl --user stop smartwheel.service 2>/dev/null || true

cat >&2 <<'WARN'
*********************************************************************
  HIGH-RISK AUTONOMOUS MAPPING. The wheelchair will try to MOVE by
  itself at low speed and explore unknown areas.
  - FIRST run with drive wheels OFF THE GROUND.
  - Then test in a CLEAR, OPEN area, NO passengers.
  - Keep the PHYSICAL E-STOP in reach. Have a person watching.
  - Do NOT run near stairs, glass doors, ramps, people, or tight spaces.
  Stop anytime: bash scripts/stop_autonomous_mapping.sh  (or Ctrl+C)
*********************************************************************
WARN

if [[ "${SMARTWHEEL_ASSUME_AUTONOMOUS_RISK:-false}" != "true" ]]; then
  read -r -p "Type I_UNDERSTAND_AUTONOMOUS_MAPPING_RISK to continue: " ans
  if [[ "$ans" != "I_UNDERSTAND_AUTONOMOUS_MAPPING_RISK" ]]; then
    echo "Confirmation mismatch. Aborted." >&2
    exit 1
  fi
fi

exec ros2 launch wheelchair_3d_mapping autonomous_rviz_mapping.launch.py \
  enable_motion:=true \
  autonomous_exploration:=true \
  exploration_mode:=reactive \
  max_linear_speed:=0.05 \
  max_angular_speed:=0.22 \
  turn_trigger_distance:=0.30 \
  rviz:=true "$@"
