#!/usr/bin/env bash
# Launcher: RIGHT XT-M60 manual-drive mapping, DIAGNOSTIC route.
#
# Replaces run_rviz_manual_mapping_left.sh for the right radar. That script
# cannot work on this deployment: RADAR=right routes into
# manual_mapping_lio_right.launch.py, which is a fail-closed shell that raises
# "BLOCKED_CONFLICT: right_lidar_stage1_calibration has no runtime-eligible
# transform" before any node starts. It then still started RViz 8 s later and
# immediately SIGINT'd it, which is where the "guard condition / 核心已转储"
# crashes came from.
#
# Differences that matter here:
#   * routes to right_lidar_diag_mapping.launch.py (no gate involved)
#   * RViz is started INSIDE that launch, after a 6 s settle timer, so a dead
#     stack can never leave orphan RViz processes to crash
#   * the launch is health-checked: if it dies early, this script reports the
#     real error instead of continuing
#
# CALIBRATION CAVEAT: the right mount TF is a provisional candidate (x/yaw
# uncalibrated). Good enough to verify hardware and watch mapping run; not a
# metrically approved map source.
#
# Usage:
#   bash scripts/run_right_diag_mapping.sh              # read-only, motors idle
#   MOTION=true bash scripts/run_right_diag_mapping.sh  # W/A/S/D drives the chair
#   CAMERAS=false bash scripts/run_right_diag_mapping.sh          # mapping only
#   CAMERA_ROLES=front,left bash scripts/...                      # subset, lower load
#
# Drive: click the "Wheelchair Teleop" panel in RViz, then W/A/S/D. Space = stop.
# Stop everything: Ctrl-C here, or bash scripts/stop_mapping.sh --force
ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_root"

# NOTE: deliberately no `set -u`. ROS 2's setup.bash dereferences unset variables
# (COLCON_TRACE, AMENT_TRACE_SETUP_FILES, ...), so nounset makes the `source`
# below abort this script silently - no error, no nodes, no log file.

# --- Resolve a usable X display + matching XAUTHORITY. NoMachine/gdm keeps its
#     cookie in /run/user/<uid>/gdm/Xauthority, not ~/.Xauthority. ---
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
  echo "ERROR: no usable X DISPLAY. In the desktop terminal run:" >&2
  echo "  echo \$DISPLAY ; echo \$XAUTHORITY" >&2
  echo "then: DISPLAY=<that> XAUTHORITY=<that> bash $0" >&2
  exit 1
fi
echo "Using DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY"

norm_bool() {
  case "$(echo "${1:-false}" | tr '[:upper:]' '[:lower:]')" in
    true|1|yes|on) echo true ;;
    *) echo false ;;
  esac
}
motion="$(norm_bool "${MOTION:-false}")"
cameras="$(norm_bool "${CAMERAS:-true}")"
ultrasonic="$(norm_bool "${ULTRASONIC:-true}")"
# All four roles. They work over compressed transport (measured 23-30 Hz each);
# the earlier "rear is dead" conclusion was an artefact of using raw transport.
camera_roles="${CAMERA_ROLES:-front,left,right,rear}"

echo "motion_control_enabled=$motion (true = motors may move)"
echo "radar=right (device 192.168.1.101 via eno1 192.168.1.100)"
echo "cameras=$cameras (roles: $camera_roles)  ultrasonic=$ultrasonic"
if [[ "$motion" == true ]]; then
  echo ""
  echo "  !! MOTORS ARMED. Clear area, physical E-stop in reach."
  echo "  !! Space in the RViz teleop panel = immediate stop."
  echo ""
fi

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ws_root/install/setup.bash" 2>/dev/null || true

# --- Radar reachability check up front: a dead link produces an empty map and
#     a confusing "no points" hunt later. ---
if ! ping -c1 -W2 192.168.1.101 >/dev/null 2>&1; then
  echo "ERROR: right radar 192.168.1.101 unreachable on eno1." >&2
  echo "       Check the cable/power, then: bash scripts/setup_radar_network.sh" >&2
  exit 1
fi
echo "Right radar 192.168.1.101 reachable."

stop_script="$ws_root/scripts/stop_mapping.sh"

# --- Pre-launch cleanup so nodes from an unclean previous exit don't pile up
#     (duplicate zlac8030_driver_node instances fight over the serial port). ---
if [[ -f "$stop_script" ]]; then
  echo "Pre-launch cleanup of any previous stack..."
  bash "$stop_script" >/dev/null 2>&1 || true
fi

cleanup() {
  trap - EXIT INT TERM
  echo ""
  echo "Shutting down stack + RViz..."
  [[ -n "${launch_pid:-}" ]] && kill -INT "$launch_pid" >/dev/null 2>&1 || true
  sleep 2
  [[ -f "$stop_script" ]] && bash "$stop_script" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# --- Launch. taskset/nice keep the startup current spike down; this host hard-
#     resets on PMIC brownout under full-core load (steering orin-host-ops). ---
setsid taskset -c 0-5 nice -n 5 \
  ros2 launch wheelchair_bringup right_lidar_diag_mapping.launch.py \
    motion_control_enabled:="$motion" \
    enable_camera:="$cameras" \
    camera_roles:="$camera_roles" \
    enable_ultrasonic:="$ultrasonic" \
    rviz:=true &
launch_pid=$!

# --- Health gate: if the launch aborts (contract gate, missing config, bad
#     param), say so instead of pretending the session is running. ---
for _ in $(seq 1 10); do
  sleep 1
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "" >&2
    echo "ERROR: launch exited during startup. Scroll up for the real cause." >&2
    echo "       Full log: ls -t ~/.ros/log | head -1" >&2
    exit 1
  fi
done
echo ""
echo "Stack is up. RViz appears ~6 s after node startup."
echo "  3D map : /cloud_registered      2D grid : /map_2d_from_3d"
echo "  Drive  : click the SmartWheel Teleop panel, then W/A/S/D (Space = stop)"
echo "  Save   : bash scripts/save_mapping_result.sh <map_name>"
echo ""
echo "  To END the session: close the RViz window, or press Ctrl-C here."
echo "  Either one tears down all nodes. Node logs keep scrolling until then -"
echo "  that is normal, the session is still running."
echo ""

wait "$launch_pid"
