#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/run_real_sensors.sh [options]

Start the real sensor bringup for:
  - XT-M60 lidar pair at 192.168.0.101 and 192.168.1.101
  - H30 IMU
  - USB-RS485 ultrasonic sensor bus
  - configured USB cameras
  - pointcloud_to_laserscan
  - Web UI

Options:
  --no-build          Do not run colcon build before launching.
  --no-imu           Do not start the H30 IMU adapter.
  --no-ui            Do not start the Web UI.
  --ui-port PORT     Web UI port, default 8080.
  --ros-domain ID    ROS_DOMAIN_ID, default 0.
  -h, --help         Show this help.

This script does not start Nav2, safety_supervisor, or the base driver.
It is for real sensor verification before autonomous driving tests.
EOF
}

run_build=true
enable_imu=true
enable_ui=true
ui_port=8080
ros_domain_id=0
child_pids=()

while (($#)); do
  case "$1" in
    --no-build)
      run_build=false
      shift
      ;;
    --no-imu)
      enable_imu=false
      shift
      ;;
    --no-ui)
      enable_ui=false
      shift
      ;;
    --ui-port)
      ui_port="${2:?missing value for --ui-port}"
      shift 2
      ;;
    --ros-domain)
      ros_domain_id="${2:?missing value for --ros-domain}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_root"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found. Install/source ROS2 Humble first." >&2
  exit 1
fi

export ROS_DOMAIN_ID="$ros_domain_id"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_root/.ros/log}"
mkdir -p "$ROS_LOG_DIR"
set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "$run_build" == true ]]; then
  colcon build --symlink-install
fi

"$workspace_root/scripts/setup_radar_network.sh" --quiet --no-ping || \
  echo "WARN radar network alias was not applied; run 'sudo $workspace_root/scripts/setup_radar_network.sh --install-service'" >&2

set +u
source "$workspace_root/install/setup.bash"
set -u

echo "Workspace: $workspace_root"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "UI enabled: $enable_ui"
echo "IMU enabled: $enable_imu"
echo
echo "Expected topics after startup:"
echo "  /xtm60/right/points"
echo "  /xtm60/left/points   (when the left radar is plugged in)"
echo "  /scan"
echo "  /imu/data"
echo "  /ultrasonic/range_0"
echo "  /camera/left/image_raw"
echo "  /camera/right/image_raw"
echo
echo "In another terminal, verify with:"
echo "  cd $workspace_root"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  bash scripts/check_topics.sh sensors"
echo "  ros2 topic hz /xtm60/right/points"
echo "  ros2 topic hz /xtm60/left/points"
echo "  ros2 topic hz /scan"
echo "  ros2 topic hz /imu/data"
echo "  ros2 topic echo /ultrasonic/range_0 --once"
echo

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  bash "$workspace_root/scripts/hardware_shutdown.sh" --no-source --quiet || true
  for pid in "${child_pids[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -INT "$pid" >/dev/null 2>&1 || true
    fi
  done
  for _ in 1 2 3 4 5 6; do
    local alive=false
    for pid in "${child_pids[@]}"; do
      if kill -0 "$pid" >/dev/null 2>&1; then
        alive=true
      fi
    done
    [[ "$alive" == false ]] && break
    sleep 0.5
  done
  for pid in "${child_pids[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
    wait "$pid" >/dev/null 2>&1 || true
  done
  exit "$status"
}
trap cleanup EXIT INT TERM

ros2 launch wheelchair_bringup sensors.launch.py \
  mode:=real \
  enable_xtm60:=false \
  enable_xtm60_left:=true \
  enable_xtm60_right:=true \
  enable_imu:="$enable_imu" \
  enable_ultrasonic:=true \
  enable_camera:=true &
child_pids+=("$!")

ros2 run wheelchair_perception pointcloud_to_laserscan_node \
  --ros-args \
  -r __node:=pointcloud_to_laserscan_left_node \
  --params-file "$workspace_root/src/wheelchair_bringup/config/pointcloud_to_scan_left.yaml" &
child_pids+=("$!")

ros2 run wheelchair_perception pointcloud_to_laserscan_node \
  --ros-args \
  -r __node:=pointcloud_to_laserscan_right_node \
  --params-file "$workspace_root/src/wheelchair_bringup/config/pointcloud_to_scan_right.yaml" &
child_pids+=("$!")

ros2 run wheelchair_perception scan_merger_node \
  --ros-args \
  --params-file "$workspace_root/src/wheelchair_bringup/config/scan_merger.yaml" &
child_pids+=("$!")

if [[ "$enable_ui" == true ]]; then
  ros2 run wheelchair_ui wheelchair_ui --host 0.0.0.0 --port "$ui_port" &
  child_pids+=("$!")
fi

wait -n
