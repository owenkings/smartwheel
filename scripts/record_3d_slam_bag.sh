#!/usr/bin/env bash
# Record the topics needed to replay / debug 3D SLAM offline.
# Usage: record_3d_slam_bag.sh [output_path]
set -u
OUT="${1:-bags/3d_slam_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$OUT")"
echo "Recording 3D SLAM bag to: $OUT  (Ctrl-C to stop)"

# Raw sensors + fusion + odometry. /tf + /tf_static are required for replay.
ros2 bag record -o "$OUT" \
  /tf /tf_static \
  /xtm60/left/points /xtm60/right/points \
  /points_merged \
  /imu/data \
  /wheel/odom \
  /main_camera/image_raw /main_camera/camera_info \
  /livo/odom /livo/cloud_registered /livo/path \
  /odometry/filtered \
  /livo_wheel/status /livo_wheel/consistency_score
