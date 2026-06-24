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
#   MOTION=true bash scripts/run_rviz_manual_mapping_left.sh                # left radar, motors on
#   bash scripts/run_rviz_manual_mapping_left.sh                           # left radar, read-only
#   RADAR=right MOTION=true bash scripts/run_rviz_manual_mapping_left.sh    # right radar only
#   RADAR=both  MOTION=true bash scripts/run_rviz_manual_mapping_left.sh    # both radars

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
radar="${RADAR:-right}"
case "$(echo "$radar" | tr '[:upper:]' '[:lower:]')" in
  right) radar=right ;;
  both) radar=both ;;
  left) radar=left ;;
  *) radar=right ;;
esac
echo "radar=$radar (right default; left is boxed/occluded | both)"

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ws_root/install/setup.bash" 2>/dev/null || true

# Select top-level launch + RViz by radar. RIGHT is the live deployment (left
# radar boxed/occluded). LEFT kept for when it is unboxed.
if [[ "$radar" == "left" ]]; then
  top_launch="manual_mapping_lio_left.launch.py"
  rviz_name="manual_mapping_lio_left.rviz"
else
  radar=right
  top_launch="manual_mapping_lio_right.launch.py"
  rviz_name="manual_mapping_lio_right.rviz"
fi
rviz_cfg="$ws_root/install/wheelchair_bringup/share/wheelchair_bringup/rviz/$rviz_name"
[[ -f "$rviz_cfg" ]] || rviz_cfg="$ws_root/src/wheelchair_bringup/rviz/$rviz_name"

stop_script="$ws_root/scripts/stop_mapping.sh"

# --- Pre-launch cleanup: kill any leftover stack from a previous run so nodes
#     don't pile up (e.g. multiple zlac8030_driver_node fighting the serial
#     port). Without this, an unclean previous exit accumulates duplicates. ---
if [[ -f "$stop_script" ]]; then
  echo "Pre-launch cleanup of any previous mapping stack..."
  bash "$stop_script" >/dev/null 2>&1 || true
fi

# --- Teardown: on exit/INT/TERM, tear down the whole subtree, not just the two
#     direct child PIDs. ros2 launch can leak its children (they reparent to
#     systemd --user and keep running), so we delegate to stop_mapping.sh which
#     pattern-kills every node the stack starts (INT then KILL). ---
cleanup() {
  trap - EXIT INT TERM
  echo ""
  echo "Shutting down mapping stack + RViz..."
  # First INT the direct children we started (fast path for clean shutdown).
  for p in "${pids[@]}"; do kill -INT "$p" >/dev/null 2>&1 || true; done
  # Then sweep the full subtree (handles leaked/reparented launch children).
  if [[ -f "$stop_script" ]]; then
    bash "$stop_script" >/dev/null 2>&1 || true
  fi
}
pids=()
trap cleanup EXIT INT TERM

# Run each background job in its own session/process group (setsid) so the whole
# group can be signalled together and children are easier to reap.
# 1. FAST-LIO mapping stack (LiDAR-inertial), no bundled RViz.
setsid ros2 launch wheelchair_bringup "$top_launch" \
  motion_control_enabled:="$motion" rviz:=false &
pids+=("$!")

# 2. RViz separately once the stack has had a moment to publish TF/topics.
sleep 8
echo "Starting RViz (FAST-LIO mapping view + embedded TeleopPanel): $rviz_cfg"
setsid rviz2 -d "$rviz_cfg" &
pids+=("$!")

# 3. Optional dedicated 2D occupancy-grid window (separate RViz instance, top-down).
#    RViz2 can't dock a 2nd render view in one window, so the 2D map is its own
#    window you can place next to the main one. Disable with MAP2D=false.
map2d="${MAP2D:-true}"
case "$(echo "$map2d" | tr '[:upper:]' '[:lower:]')" in true|1|yes|on) map2d=true ;; *) map2d=false ;; esac
if [[ "$map2d" == true ]]; then
  map2d_cfg="$ws_root/install/wheelchair_bringup/share/wheelchair_bringup/rviz/map_2d.rviz"
  [[ -f "$map2d_cfg" ]] || map2d_cfg="$ws_root/src/wheelchair_bringup/rviz/map_2d.rviz"
  echo "Starting 2D occupancy-grid window: $map2d_cfg"
  setsid rviz2 -d "$map2d_cfg" &
  pids+=("$!")
fi

# Wait for any child to exit (e.g. you close RViz), then cleanup() runs on EXIT.
wait -n
