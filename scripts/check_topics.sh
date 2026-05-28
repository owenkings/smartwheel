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
      "/camera/front/image_raw"
    )
    ;;
  full)
    required_topics=(
      "/xtm60/points"
      "/scan"
      "/imu/data"
      "/ultrasonic/range_0"
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

timeout_sec="${CHECK_TOPICS_TIMEOUT_SEC:-8}"
deadline=$((SECONDS + timeout_sec))
available=""

while true; do
  available="$(ros2 topic list)"
  all_found=1
  for topic in "${required_topics[@]}"; do
    if ! grep -qx "$topic" <<< "$available"; then
      all_found=0
      break
    fi
  done
  if [[ "$all_found" -eq 1 || "$SECONDS" -ge "$deadline" ]]; then
    break
  fi
  sleep 1
done

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
