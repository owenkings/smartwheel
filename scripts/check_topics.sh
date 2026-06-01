#!/usr/bin/env bash
set -euo pipefail

profile="${1:-full}"

case "$profile" in
  sensors)
    required_topics=(
      "/scan"
      "/imu/data"
      "/ultrasonic/range_0"
      "/camera/left/image_raw"
      "/camera/right/image_raw"
    )
    alternative_groups=(
      "/xtm60/right/points|/xtm60/points"
    )
    ;;
  full)
    required_topics=(
      "/scan"
      "/imu/data"
      "/ultrasonic/range_0"
      "/cmd_vel_safe"
      "/safety_state"
      "/goal_pose"
    )
    alternative_groups=(
      "/xtm60/right/points|/xtm60/points"
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
  for group in "${alternative_groups[@]}"; do
    found_group=0
    IFS='|' read -r -a alternatives <<< "$group"
    for topic in "${alternatives[@]}"; do
      if grep -qx "$topic" <<< "$available"; then
        found_group=1
        break
      fi
    done
    if [[ "$found_group" -eq 0 ]]; then
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

for group in "${alternative_groups[@]}"; do
  found_group=0
  IFS='|' read -r -a alternatives <<< "$group"
  for topic in "${alternatives[@]}"; do
    if grep -qx "$topic" <<< "$available"; then
      echo "OK $topic"
      found_group=1
      break
    fi
  done
  if [[ "$found_group" -eq 0 ]]; then
    echo "MISSING one of: ${group//|/, }"
    missing=1
  fi
done

exit "$missing"
