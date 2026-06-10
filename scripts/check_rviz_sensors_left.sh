#!/usr/bin/env bash
# Stage 0 verification: are the single-left-radar sensors alive and is TF sane?
# Run this in a second terminal AFTER starting the sensor stack. It does not
# start or move anything; it only checks topics and TF.
#
# Note: no `set -u`. Sourcing ROS setup.bash references unbound vars, which
# would abort under strict mode (same reason other repo scripts wrap source
# with `set +u`).

ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ws_root/install/setup.bash" 2>/dev/null || true

pass=0
fail=0

check_hz() {  # topic min_hz
  local topic="$1" min="$2"
  local out rate
  out="$(timeout 6 ros2 topic hz "$topic" 2>/dev/null | grep -oE 'average rate: [0-9.]+' | tail -1)"
  rate="${out##* }"
  if [[ -n "$rate" ]] && awk "BEGIN{exit !($rate >= $min)}"; then
    printf '  OK   %-26s %.2f Hz (>= %s)\n' "$topic" "$rate" "$min"; pass=$((pass+1))
  else
    printf '  FAIL %-26s %s Hz (< %s or silent)\n' "$topic" "${rate:-0}" "$min"; fail=$((fail+1))
  fi
}

check_tf() {  # parent child
  if timeout 4 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -q "Translation"; then
    printf '  OK   TF %s -> %s\n' "$1" "$2"; pass=$((pass+1))
  else
    printf '  FAIL TF %s -> %s missing\n' "$1" "$2"; fail=$((fail+1))
  fi
}

echo "== Stage 0: single-left-radar sensor visualization =="
echo "-- topic rates --"
check_hz /xtm60/left/points 4
check_hz /scan 5
check_hz /imu/data 50
check_hz /camera/left/image_raw 3

echo "-- ultrasonic (presence) --"
for i in 0 1 2 3; do
  if timeout 3 ros2 topic echo "/ultrasonic/range_$i" --field range --once >/dev/null 2>&1; then
    printf '  OK   /ultrasonic/range_%s has data\n' "$i"; pass=$((pass+1))
  else
    printf '  WARN /ultrasonic/range_%s silent\n' "$i"
  fi
done

echo "-- TF chain --"
check_tf base_link xtm60_left_link
check_tf base_link imu_link
check_tf odom base_link

echo
echo "PASS=$pass FAIL=$fail"
if [[ "$fail" -eq 0 ]]; then
  echo "STAGE0_OK: sensors visible, TF sane."
else
  echo "STAGE0_BLOCKED: fix the FAIL items above before RViz visualization."
fi
