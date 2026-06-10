#!/usr/bin/env bash
set -o pipefail
cd "$(dirname "$0")/.."

set +e
output="$(bash scripts/check_autonomous_mapping_status.sh --left-lidar-lab "$@" 2>&1)"
status=$?
set -e
printf '%s\n' "$output"

echo "----------------------------------------"
if grep -q '^READY_TO_ARM_LEFT_LIDAR$' <<<"$output"; then
  echo "Left-lidar lab stack is ready to arm."
  echo "Now publish:"
  echo 'ros2 topic pub --once /autonomy/enable std_msgs/msg/Bool "{data: true}"'
elif grep -q '^READY_TO_PLAN_LEFT_LIDAR$' <<<"$output"; then
  echo "Mapping stack is up, but autonomous motion is not armed."
else
  echo "Left-lidar lab mapping is blocked; resolve the MISS/BLOCK reasons above."
fi
exit "$status"
