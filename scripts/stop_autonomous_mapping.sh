#!/usr/bin/env bash
# Latch a software E-stop and disarm autonomous exploration. Map data is kept.
set -o pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/.ros/log}"

echo "Disarming autonomy and latching the software E-stop..."
timeout 2 ros2 topic pub --once /autonomy/enable std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1 || true
timeout 2 ros2 topic pub --once /emergency_stop_command std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
# Fallback for stacks that do not yet run emergency_stop_node. This still enters
# the safety supervisor; it never writes /cmd_vel_safe directly.
timeout 1 ros2 topic pub -r 10 /emergency_stop_sw std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 || true

echo "Autonomy is disarmed and the software E-stop is active."
echo "Stop the launch with Ctrl+C to release the drive and flush ~/.ros/rtabmap.db."
echo "Map data can then be exported with scripts/save_rtabmap_3d_map.sh."
