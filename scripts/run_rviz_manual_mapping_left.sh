#!/usr/bin/env bash
# Stage 1 launcher: manual-drive mapping (single LEFT XT-M60) with RViz on the
# local/NoMachine desktop. Teleop is an embedded RViz panel.
#
# Starts the mapping stack (sensors+EKF+safety+base+fusion+RTAB-Map) WITHOUT its
# own RViz, then starts RViz separately with the mapping view. Starting RViz as a
# separate process (not inside the big launch) avoids a startup race that left
# RViz not coming up when bundled with ~12 other nodes.
#
# Motors move only with MOTION=true. Default is read-only.
#   MOTION=true bash scripts/run_rviz_manual_mapping_left.sh   # enable motors
#   bash scripts/run_rviz_manual_mapping_left.sh               # read-only

ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_root"

# --- Resolve a usable X display + matching XAUTHORITY (NoMachine/gdm uses
#     /run/user/<uid>/gdm/Xauthority, NOT ~/.Xauthority). ---
detect_display() {
  local uid d xa
  uid="$(id -u)"
  for d in "${DISPLAY:-}" :1 :0 :1001; do
    [[ -z "$d" ]] && continue
    for xa in "${XAUTHORITY:-}" "/run/user/${uid}/gdm/Xauthority" "$HOME/.Xauthority"; do
      [[ -z "$xa" || ! -f "$xa" ]] && continue
      if DISPLAY="$d" XAUTHORITY="$xa" timeout 3 xdpyinfo >/dev/null 2>&1; then
        export DISPLAY="$d" XAUTHORITY="$xa"; return 0
      fi
    done
  done
  return 1
}
if ! detect_display; then
  echo "ERROR: no usable X DISPLAY. In your NoMachine desktop terminal run:" >&2
  echo "  echo \$DISPLAY ; echo \$XAUTHORITY" >&2
  echo "then: DISPLAY=<that> XAUTHORITY=<that> bash $0" >&2
  exit 1
fi
echo "Using DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY"

motion="${MOTION:-false}"
case "$(echo "$motion" | tr '[:upper:]' '[:lower:]')" in
  true|1|yes|on) motion=true ;;
  *) motion=false ;;
esac
echo "motion_control_enabled=$motion (true = motors may move)"

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ws_root/install/setup.bash" 2>/dev/null || true

rviz_cfg="$ws_root/install/wheelchair_bringup/share/wheelchair_bringup/rviz/manual_mapping_left.rviz"
[[ -f "$rviz_cfg" ]] || rviz_cfg="$ws_root/src/wheelchair_bringup/rviz/manual_mapping_left.rviz"

pids=()
cleanup() {
  trap - EXIT INT TERM
  for p in "${pids[@]}"; do kill -INT "$p" >/dev/null 2>&1 || true; done
}
trap cleanup EXIT INT TERM

# 1. Mapping stack, no bundled RViz.
ros2 launch wheelchair_bringup manual_mapping_left.launch.py \
  motion_control_enabled:="$motion" rviz:=false delete_db_on_start:=true &
pids+=("$!")

# 2. RViz separately once the stack has had a moment to publish TF/topics.
sleep 8
echo "Starting RViz (mapping view + embedded TeleopPanel): $rviz_cfg"
rviz2 -d "$rviz_cfg" &
pids+=("$!")

wait -n
