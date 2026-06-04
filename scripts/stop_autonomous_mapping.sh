#!/usr/bin/env bash
# Stop autonomous exploration motion NOW. Does NOT delete map data.
set -o pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/.ros/log}"

echo "Stopping autonomous motion (software e-stop + zero velocity)..."
# Latched software e-stop -> safety_supervisor zeroes /cmd_vel_safe AND
# /safety_state becomes EMERGENCY, which makes frontier_explorer cancel its goal.
timeout 2 ros2 topic pub -r 10 /emergency_stop_sw std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 || true
ros2 topic pub --once /emergency_stop_command std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
timeout 2 ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
timeout 2 ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
python3 scripts/zlac8030_release.py >/dev/null 2>&1 || true

echo "Software e-stop sent. Zero velocity published to /cmd_vel_nav and /cmd_vel_safe."
echo "ZLAC8030 direct release attempted; the chair should be pushable after nodes stop."
echo "NOTE: this does NOT shut down the launch. Press Ctrl+C in the launch terminal"
echo "      to fully stop, then 'ros2 topic pub --once /emergency_stop_sw std_msgs/msg/Bool \"{data: false}\"' to release."
echo "Map data is preserved (~/.ros/rtabmap.db). Save it with scripts/save_rtabmap_3d_map.sh"
