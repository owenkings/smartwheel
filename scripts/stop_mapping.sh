#!/usr/bin/env bash
# Stop / clean up the manual-mapping stack and RViz.
#
# Why this exists: when the launcher's parent dies, ros2 launch sometimes leaves
# its child nodes behind; they get reparented to systemd --user and keep running
# (and pile up across runs -> multiple zlac8030_driver_node fighting the serial
# port). This script forcibly clears every node the mapping stack starts.
#
# Usage:
#   bash scripts/stop_mapping.sh          # graceful (INT) then force (KILL)
#   bash scripts/stop_mapping.sh --force  # skip grace period, KILL immediately
set -u

force=false
[[ "${1:-}" == "--force" ]] && force=true

# Patterns covering every process the mapping stack launches.
patterns=(
  "manual_mapping_left.launch.py"
  "manual_mapping_lio_left.launch.py"
  "manual_mapping_lio_right.launch.py"
  # Right-radar diagnostic route (scripts/run_right_diag_mapping.sh). Without
  # this pattern its nodes survive teardown and the next run hits a busy serial
  # port / duplicate publishers.
  "right_lidar_diag_mapping.launch.py"
  "fast_lio_mapping.launch.py"
  "rviz2 -d .*manual_mapping_left.rviz"
  "rviz2 -d .*manual_mapping_lio_left.rviz"
  "rviz2 -d .*manual_mapping_lio_right.rviz"
  "rviz2 -d .*right_diag_mapping.rviz"
  "rviz2 -d .*map_2d.rviz"
  # Camera workers run under a per-role namespace, so they do not match the
  # wheelchair_sensors/lib pattern in every ps form.
  "camera_adapter_node"
  "fast_lio/lib"
  "fastlio_mapping"
  "lio_cloud_adapter"
  "wheelchair_sensors/lib"
  "wheelchair_perception/lib"
  "wheelchair_safety/lib"
  "wheelchair_base/lib"
  "wheelchair_3d_mapping/lib"
  "wheelchair_diagnostics/lib"
  "rtabmap_slam/rtabmap"
  # slam_toolbox is started by manual_teleop.launch.py (enable_2d_mapping). It was
  # missing here, so a leftover instance survived teardown and kept publishing
  # map->odom into the next session.
  "slam_toolbox/async_slam_toolbox_node"
  "async_slam_toolbox_node"
  "robot_localization/ekf_node"
  "robot_localization/lib"
  "robot_state_publisher --ros-args"
  "static_transform_publisher"
)

count_alive() {
  local n=0 p c
  for p in "${patterns[@]}"; do
    c="$(pgrep -fc "$p" 2>/dev/null)"
    [[ -z "$c" ]] && c=0
    n=$((n + c))
  done
  echo "$n"
}

signal_all() {
  local sig="$1" p
  for p in "${patterns[@]}"; do
    pkill -"$sig" -f "$p" 2>/dev/null || true
  done
}

echo "[stop_mapping] alive before: $(count_alive)"

if ! $force; then
  echo "[stop_mapping] sending SIGINT (graceful)..."
  signal_all INT
  # Wait up to ~8s for graceful shutdown.
  for _ in $(seq 1 8); do
    [[ "$(count_alive)" == "0" ]] && break
    sleep 1
  done
fi

remaining="$(count_alive)"
if [[ "$remaining" != "0" ]]; then
  echo "[stop_mapping] $remaining still alive, sending SIGKILL..."
  signal_all KILL
  sleep 1
fi

echo "[stop_mapping] alive after: $(count_alive)"
[[ "$(count_alive)" == "0" ]] && echo "[stop_mapping] clean." || echo "[stop_mapping] WARNING: some processes survived."
