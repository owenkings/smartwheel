#!/usr/bin/env bash
set -euo pipefail

required_topics=(
  "/xtm60/points"
  "/scan"
  "/imu/data"
  "/ultrasonic/range_0"
  "/ultrasonic/range_1"
  "/cmd_vel_safe"
  "/safety_state"
  "/goal_pose"
)

available="$(ros2 topic list)"
missing=0
for topic in "${required_topics[@]}"; do
  if grep -qx "$topic" <<< "$available"; then
    echo "OK $topic"
  else
    echo "MISSING $topic"
    missing=1
  fi
done

exit "$missing"
