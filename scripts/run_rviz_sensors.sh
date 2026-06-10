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

# Pick a usable X display. Override by exporting DISPLAY before running.
if [[ -z "${DISPLAY:-}" ]]; then
  for d in :1 :0 :1001; do
    if DISPLAY="$d" timeout 3 xdpyinfo >/dev/null 2>&1; then
      export DISPLAY="$d"
      break
    fi
  done
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: no usable X DISPLAY found. On NoMachine, run this inside the" >&2
  echo "remote desktop terminal, or 'export DISPLAY=:1' then re-run." >&2
  exit 1
fi
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
echo "Using DISPLAY=$DISPLAY"

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
