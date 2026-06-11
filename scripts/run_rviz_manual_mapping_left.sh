#!/usr/bin/env bash
# Stage 1 launcher: manual-drive mapping (single LEFT XT-M60) with RViz on the
# local/NoMachine desktop. Teleop is an embedded RViz panel.
#
# Motors move only with motion_control_enabled:=true. Default is read-only.
#   MOTION=true bash scripts/run_rviz_manual_mapping_left.sh   # enable motors
#   bash scripts/run_rviz_manual_mapping_left.sh               # read-only

ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_root"

if [[ -z "${DISPLAY:-}" ]]; then
  for d in :1 :0 :1001; do
    if DISPLAY="$d" timeout 3 xdpyinfo >/dev/null 2>&1; then export DISPLAY="$d"; break; fi
  done
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: no usable X DISPLAY. Run inside the NoMachine desktop or 'export DISPLAY=:1'." >&2
  exit 1
fi
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
echo "Using DISPLAY=$DISPLAY"

motion="${MOTION:-false}"
case "$(echo "$motion" | tr '[:upper:]' '[:lower:]')" in
  true|1|yes|on) motion=true ;;
  *) motion=false ;;
esac
echo "motion_control_enabled=$motion (true = motors may move)"

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ws_root/install/setup.bash" 2>/dev/null || true

exec ros2 launch wheelchair_bringup manual_mapping_left.launch.py \
  motion_control_enabled:="$motion" rviz:=true
