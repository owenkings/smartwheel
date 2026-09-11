#!/usr/bin/env bash
# Wheel/IMU-primary indoor mapping. No existing stack is stopped automatically.
# The point cloud remains 3D; the continuous vehicle trajectory is planar.
set -eo pipefail
ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_root"

bool_value() {
  case "${1,,}" in
    true|1|yes|on) echo true ;;
    false|0|no|off) echo false ;;
    *) echo "ERROR: expected true/false, got '$1'" >&2; return 1 ;;
  esac
}
motion="$(bool_value "${MOTION:-false}")"
cameras="$(bool_value "${CAMERAS:-true}")"
ultrasonic="$(bool_value "${ULTRASONIC:-true}")"
shadow="$(bool_value "${FASTLIO_SHADOW:-false}")"
camera_roles="${CAMERA_ROLES:-front,left,right,rear}"

# Refuse instead of killing another terminal, the old RTAB checkout, or a bag
# regression. Delimit process executable names to avoid matching this script.
if pgrep -af '/(xtm60_adapter_node|imu_adapter_node|zlac8030_driver_node|fastlio_mapping|rtabmap)( |$)' ; then
  echo 'ERROR: a hardware/mapping stack is already running. Exit its own terminal first.' >&2
  exit 1
fi

# Do not retain install-old through a previously sourced desktop shell.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$ws_root/install/setup.bash"

if [[ -z "${DISPLAY:-}" ]] || ! timeout 3 xdpyinfo >/dev/null 2>&1; then
  echo 'ERROR: run this command inside the Orin graphical desktop terminal (NoMachine).' >&2
  echo 'No working DISPLAY/Xauthority; no hardware has been started.' >&2
  exit 1
fi
if ! ping -c 1 -W 2 192.168.1.101 >/dev/null 2>&1; then
  echo 'ERROR: right XT-M60 192.168.1.101 is unreachable; no hardware has been started.' >&2
  exit 1
fi
free_kb="$(df -Pk "$ws_root" | awk 'NR==2 {print $4}')"
if (( free_kb < 2097152 )); then
  echo 'ERROR: less than 2 GiB free; archive old bags before a new recorded session.' >&2
  exit 1
fi

mkdir -p "$ws_root/maps"
run_dir="$(mktemp -d "$ws_root/maps/wheel_imu_$(date +%Y%m%d_%H%M%S)_XXXX")"
export ROS_LOG_DIR="$run_dir/ros_logs"
launch_pid=''
owned_pgid=''
record_pid=''

descendants() {
  ps -e -o pid=,ppid=,pgid= | awk -v root="$launch_pid" -v group="$owned_pgid" '
    { parent[$1]=$2; pgid[$1]=$3 }
    END {
      for (pass=0;pass<32;pass++)
        for (pid in parent)
          if (parent[pid]==root || seen[parent[pid]]) seen[pid]=1;
      for (pid in parent)
        if (seen[pid] || (group!="" && pgid[pid]==group)) print pid;
    }'
}
cleanup() {
  trap - EXIT INT TERM
  # Stop only adapters descended from OUR launch, while the parent remains
  # alive; let SDK stop()/shutdown() complete before ROS launch tears down.
  local pid command adapter_pids=() base_pids=() remaining=false
  # End the motor driver's owned session FIRST. Its existing destroy_node()
  # shutdown path zeros both motors before the slower sensor SDK cleanup.
  while read -r pid; do
    [[ -r "/proc/$pid/cmdline" ]] || continue
    command="$(tr '\0' ' ' < "/proc/$pid/cmdline")"
    if [[ "$command" == *'/zlac8030_driver_node '* ]]; then
      base_pids+=("$pid")
      kill -INT "$pid" 2>/dev/null || true
    fi
  done < <([[ -n "$launch_pid" ]] && descendants)
  for _ in $(seq 1 5); do
    remaining=false
    for pid in "${base_pids[@]}"; do
      if [[ -r "/proc/$pid/cmdline" ]] &&
         [[ "$(tr '\0' ' ' < "/proc/$pid/cmdline")" == *'/zlac8030_driver_node '* ]]; then
        remaining=true
      fi
    done
    [[ "$remaining" == false ]] && break
    sleep 1
  done
  if [[ "$remaining" == true ]]; then
    echo 'WARNING: motor driver shutdown did not finish; use the PHYSICAL E-STOP.' >&2
    kill -INT "$launch_pid" 2>/dev/null || true
  fi
  while read -r pid; do
    [[ -r "/proc/$pid/cmdline" ]] || continue
    command="$(tr '\0' ' ' < "/proc/$pid/cmdline")"
    if [[ "$command" == *'/xtm60_adapter_node '* ]]; then
      adapter_pids+=("$pid")
      kill -INT "$pid" 2>/dev/null || true
    fi
  done < <([[ -n "$launch_pid" ]] && descendants)
  for _ in $(seq 1 15); do
    remaining=false
    for pid in "${adapter_pids[@]}"; do
      if [[ -r "/proc/$pid/cmdline" ]] &&
         [[ "$(tr '\0' ' ' < "/proc/$pid/cmdline")" == *'/xtm60_adapter_node '* ]]; then
        remaining=true
      fi
    done
    [[ "$remaining" == false ]] && break
    sleep 1
  done
  if [[ "$remaining" == true ]]; then
    echo 'WARNING: an owned radar adapter did not finish SDK shutdown within 15 s.' >&2
  fi
  if [[ -n "$launch_pid" ]]; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [[ -n "$record_pid" ]]; then
    kill -INT -- "-$record_pid" 2>/dev/null || true
    for _ in $(seq 1 15); do
      kill -0 "$record_pid" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$record_pid" 2>/dev/null; then
      echo 'WARNING: recorder did not finalize; raw bag needs recovery.' >&2
      kill -TERM -- "-$record_pid" 2>/dev/null || true
      for _ in $(seq 1 3); do
        kill -0 "$record_pid" 2>/dev/null || break
        sleep 1
      done
      if kill -0 "$record_pid" 2>/dev/null; then
        kill -KILL -- "-$record_pid" 2>/dev/null || true
      fi
    fi
    wait "$record_pid" 2>/dev/null || true
  fi
  echo "ROS session exited. Database and ROS logs preserved at: $run_dir"
  if [[ "$motion" == true ]]; then
    echo 'IMPORTANT: process exit does NOT verify physical motor stop or successful stop-register writes.' >&2
    echo 'Confirm both wheels have stopped; if not, use the PHYSICAL E-STOP immediately.' >&2
  fi
  # Export only after hardware/mapper shutdown, from a private DB snapshot in
  # an isolated ROS domain. Never copy or open another checkout's database.
  if [[ -s "$run_dir/rtabmap.db" ]]; then
    if python3 "$ws_root/scripts/mapping/wheel_map_bundle.py" export \
        --database "$run_dir/rtabmap.db" --output "$run_dir/products" \
        >"$run_dir/export.log" 2>&1; then
      echo "Map files exported: $run_dir/products"
      echo "Preview: python3 scripts/mapping/wheel_map_bundle.py preview --bundle '$run_dir/products'"
    else
      echo "EXPORT FAILED: keep the database/raw bag; see $run_dir/export.log" >&2
    fi
  else
    echo 'No RTAB database produced; this session did not finish mapping.' >&2
  fi
}
trap cleanup EXIT INT TERM

echo 'POSE: measured wheel speed + H30 yaw rate -> EKF -> freshness gate -> odom->base_link.'
echo 'MAP: raw XYZI + wheel/IMU pose -> RTAB-Map ICP loops -> map->odom.'
echo "FAST-LIO shadow enabled: $shadow; never a vehicle TF source."
echo 'LIMIT: planar indoor trajectory, provisional extrinsics; not slope/navigation acceptance.'
echo "MOTION=$motion; CAMERAS=$cameras; ULTRASONIC=$ultrasonic"
echo "New database: $run_dir/rtabmap.db"
if [[ "$motion" == true ]]; then
  echo 'MOTORS MAY MOVE. Operator must ensure clear area and a reachable physical E-stop.'
fi
# Start before sensors, so initialization and static TF are not lost. Cameras
# and repeated accumulated maps are deliberately excluded from the raw bag.
setsid ros2 bag record -o "$run_dir/raw" --max-bag-duration 300 \
  /xtm60/right/points /xtm60/right/timing /xtm60/right/quality \
  /imu/data /wheel/odom /base/wheel_feedback_healthy /cmd_vel_safe \
  /base/mapping_drive_mode /base/status \
  /wheel_mapping/odometry_gated /tf /tf_static \
  >"$run_dir/recorder.log" 2>&1 &
record_pid=$!
sleep 2
if ! kill -0 "$record_pid" 2>/dev/null; then
  echo "ERROR: raw recorder failed; see $run_dir/recorder.log. No hardware started." >&2
  exit 1
fi
setsid taskset -c 0-5 nice -n 5 \
  ros2 launch wheelchair_bringup right_lidar_diag_mapping.launch.py \
    pose_owner:=wheel_imu \
    enable_fastlio_shadow:="$shadow" \
    motion_control_enabled:="$motion" \
    enable_camera:="$cameras" camera_roles:="$camera_roles" \
    enable_ultrasonic:="$ultrasonic" \
    database_path:="$run_dir/rtabmap.db" rviz:=true &
launch_pid=$!
owned_pgid="$launch_pid"  # setsid makes the owned child its own session/PGID leader.
wait "$launch_pid"
