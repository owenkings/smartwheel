#!/usr/bin/env bash
# Launch RViz (sensor view) + a wheelchair teleop panel on the local desktop.
#
# This is meant to run while a sensor stack is already up (e.g.
# scripts/run_real_sensors.sh). It renders to the physical/NoMachine desktop,
# not to the headless SSH session, so it sets DISPLAY if you have not.
#
# Teleop publishes to /cmd_vel_nav (through safety_supervisor). Motors only move
# when the base driver runs with motion_control_enabled:=true.
set -u

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_root"

# Pick a usable X display + matching XAUTHORITY. The NoMachine/gdm session
# usually lives on :1 with auth at /run/user/<uid>/gdm/Xauthority (NOT
# ~/.Xauthority). Override by exporting DISPLAY/XAUTHORITY before running.
detect_display() {
  local uid xauth_candidates d xa
  uid="$(id -u)"
  xauth_candidates=(
    "${XAUTHORITY:-}"
    "/run/user/${uid}/gdm/Xauthority"
    "$HOME/.Xauthority"
  )
  for d in "${DISPLAY:-}" :1 :0 :1001; do
    [[ -z "$d" ]] && continue
    for xa in "${xauth_candidates[@]}"; do
      [[ -z "$xa" || ! -f "$xa" ]] && continue
      if DISPLAY="$d" XAUTHORITY="$xa" timeout 3 xdpyinfo >/dev/null 2>&1; then
        export DISPLAY="$d" XAUTHORITY="$xa"
        return 0
      fi
    done
  done
  return 1
}

if ! detect_display; then
  echo "ERROR: no usable X DISPLAY found. In your NoMachine desktop terminal run:" >&2
  echo "  echo \$DISPLAY ; echo \$XAUTHORITY" >&2
  echo "then re-run:  DISPLAY=<that> XAUTHORITY=<that> bash $0" >&2
  exit 1
fi
echo "Using DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found." >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "$workspace_root/install/setup.bash"
set -u

rviz_cfg="$workspace_root/src/wheelchair_bringup/rviz/sensor_view.rviz"

pids=()
cleanup() {
  trap - EXIT INT TERM
  for p in "${pids[@]}"; do
    kill -INT "$p" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT INT TERM

echo "Starting RViz with $rviz_cfg (teleop is an embedded panel)"
rviz2 -d "$rviz_cfg" &
pids+=("$!")

wait -n
