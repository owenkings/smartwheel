#!/usr/bin/env bash
# Stage 1 verification: is the manual-drive mapping stack healthy?
# Run in a second terminal AFTER starting manual_mapping_left. It does not move
# anything; it only checks topics, the safety chain, and that RTAB-Map outputs
# are alive.
#
# No `set -u`: sourcing ROS setup references unbound vars.

ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ws_root/install/setup.bash" 2>/dev/null || true

pass=0
fail=0

check_hz() {  # topic min_hz
  local topic="$1" min="$2" out rate
  out="$(timeout 8 ros2 topic hz "$topic" 2>/dev/null | grep -oE 'average rate: [0-9.]+' | tail -1)"
  rate="${out##* }"
  if [[ -n "$rate" ]] && awk "BEGIN{exit !($rate >= $min)}"; then
    printf '  OK   %-26s %.2f Hz (>= %s)\n' "$topic" "$rate" "$min"; pass=$((pass+1))
  else
    printf '  FAIL %-26s %s Hz (< %s or silent)\n' "$topic" "${rate:-0}" "$min"; fail=$((fail+1))
  fi
}

check_present() {  # topic
  if timeout 6 ros2 topic echo "$1" --once >/dev/null 2>&1; then
    printf '  OK   %-26s has data\n' "$1"; pass=$((pass+1))
  else
    printf '  FAIL %-26s silent\n' "$1"; fail=$((fail+1))
  fi
}

check_field_false() {  # topic field human  -> pass if value is "false"
  local topic="$1" field="$2"
  local val
  val="$(timeout 5 ros2 topic echo "$topic" --field "$field" --once 2>/dev/null | head -1)"
  if [[ "$val" == "false" ]]; then
    printf '  OK   %-26s %s=false\n' "$topic" "$field"; pass=$((pass+1))
  else
    printf '  FAIL %-26s %s=%s (expected false)\n' "$topic" "$field" "${val:-?}"; fail=$((fail+1))
  fi
}

echo "== Stage 1: manual-drive mapping health =="
echo "-- live inputs --"
check_hz /xtm60/left/points 4
check_hz /points_merged 4
check_hz /odometry/filtered 10
check_hz /scan 5

echo "-- RTAB-Map outputs (the map) --"
check_present /rtabmap/cloud_map
check_present /rtabmap/grid_map

echo "-- safety chain --"
check_field_false /system_stop_required data
echo -n "  safety_state: "
timeout 5 ros2 topic echo /safety_state --field data --once 2>/dev/null | head -1

echo "-- TF map->odom (RTAB-Map) --"
if timeout 5 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "Translation"; then
  printf '  OK   TF map -> odom present\n'; pass=$((pass+1))
else
  printf '  FAIL TF map -> odom missing (RTAB-Map not localizing yet?)\n'; fail=$((fail+1))
fi

echo
echo "PASS=$pass FAIL=$fail"
if [[ "$fail" -eq 0 ]]; then
  echo "STAGE1_OK: mapping stack healthy. Drive slowly to grow the map, then run save_mapping_result.sh."
else
  echo "STAGE1_CHECK: review FAIL items. cloud_map/grid_map/map->odom often appear only after a few seconds of motion."
fi
