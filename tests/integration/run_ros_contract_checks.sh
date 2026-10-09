#!/usr/bin/env bash
set -euo pipefail
cd /home/nvidia/wheelchair
test "$(id -un)" = nvidia
test "$(uname -m)" = aarch64
export PYTHONNOUSERSITE=1 PYTHONPATH=/home/nvidia/wheelchair/src ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
set +u
source /opt/ros/humble/setup.bash
source install/main/setup.bash
set -u
run="reports/ros_contracts/$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$run"
exec > >(tee "$run/full.log") 2>&1
python3 tests/imu/check_h30_ros_serialization.py --evidence "$run/h30_cdr.json"
python3 tests/calibration/check_rosbag_roundtrip.py --output-dir "$run/rosbag_roundtrip"
python3 tests/integration/check_fusion_ros_faults.py --output-dir "$run/fusion_faults" --ros-domain-id 84
printf 'PASS\n' > "$run/result"
