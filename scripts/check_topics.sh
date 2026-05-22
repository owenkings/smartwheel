#!/usr/bin/env bash
set -euo pipefail

profile="${1:-full}"

case "$profile" in
  sensors)
    required_topics=(
      "/xtm60/points"
      "/scan"
      "/imu/data"
      "/ultrasonic/range_0"
      "/ultrasonic/range_1"
      "/camera/front/image_raw"
      "/camera/left/image_raw"
    )
    ;;
  full)
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
    ;;
  *)
    echo "Usage: $0 [sensors|full]" >&2
    exit 2
    ;;
esac

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
