#!/usr/bin/env bash
# Quick health check for the 3D SLAM pipeline. Run in a sourced shell.
set -u

hz() { echo "--- hz $1 (Ctrl-C to skip) ---"; timeout "${2:-4}" ros2 topic hz "$1"; }
echo1() { echo "--- echo $1 ---"; timeout 3 ros2 topic echo --once "$1" 2>/dev/null || echo "  (no message)"; }

echo "== topics with 'points' / 'livo' / 'map' =="
ros2 topic list | grep -E 'points|livo|map_2d|odom' || true

echo "== input rates =="
hz /xtm60/left/points 4
hz /xtm60/right/points 4
hz /imu/data 4
hz /wheel/odom 4
hz /main_camera/image_raw 4

echo "== fusion / outputs =="
hz /points_merged 4
echo1 /points_merged/status
echo1 /livo_wheel/status

echo "== LIVO backend (only if backend installed & running) =="
hz /livo/odom 4
hz /livo/cloud_registered 4
hz /map_2d_from_3d 4

echo "== odom->base_link TF (must have exactly one publisher) =="
timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -n 12 || echo "  (no odom->base_link TF)"
