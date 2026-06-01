#!/usr/bin/env bash
# Launch HIGH-RISK autonomous RViz exploration 3D mapping. The wheelchair MAY MOVE.
set -euo pipefail
cd "$(dirname "$0")/.."

source /opt/ros/humble/setup.bash
if [[ ! -f install/setup.bash ]]; then
  echo "ERROR: install/setup.bash missing. Run: colcon build --symlink-install" >&2
  exit 1
fi
source install/setup.bash

# Avoid duplicate hardware nodes from the autostart service.
sudo systemctl stop smartwheel.service 2>/dev/null || systemctl --user stop smartwheel.service 2>/dev/null || true

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

read -r -p "Type I_UNDERSTAND_AUTONOMOUS_MAPPING_RISK to continue: " ans
if [[ "$ans" != "I_UNDERSTAND_AUTONOMOUS_MAPPING_RISK" ]]; then
  echo "Confirmation mismatch. Aborted." >&2
  exit 1
fi

exec ros2 launch wheelchair_3d_mapping autonomous_rviz_mapping.launch.py \
  enable_motion:=true \
  autonomous_exploration:=true \
  max_linear_speed:=0.10 \
  max_angular_speed:=0.25 \
  rviz:=true "$@"
