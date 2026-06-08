#!/usr/bin/env bash
# Validate the live autonomous mapping stack before /autonomy/enable is asserted.
set -o pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true
export ROS_LOG_DIR="$PWD/.ros/log"
mkdir -p "$ROS_LOG_DIR"

require_motion=false
if [[ "${1:-}" == "--require-motion-enabled" ]]; then
  require_motion=true
fi

plan_reasons=()
arm_reasons=()

topic_once() {
  timeout "${2:-3}" ros2 topic echo "$1" --once \
    --qos-reliability best_effort 2>/dev/null
}
have_msg() { topic_once "$1" "${2:-3}" >/dev/null; }
have_tf() {
  local output
  output="$(timeout 4 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 || true)"
  grep -q "Translation:" <<<"$output"
}
have_action() {
  local attempt
  for attempt in 1 2 3 4; do
    if ros2 action list 2>/dev/null | grep -Fxq "$1"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

topics=( \
  /xtm60/left/points /xtm60/right/points /points_merged /points_merged/status \
  /imu/data /ultrasonic/range_0 /ultrasonic/range_1 \
  /ultrasonic/range_2 /ultrasonic/range_3 /wheel/odom /odometry/filtered \
  /rtabmap/cloud_map /rtabmap/grid_map /scan /safety_state \
  /hardware/status /system_stop_required /base/status /cmd_vel_safe \
  /exploration/status
)
batch_size=4
for ((start = 0; start < ${#topics[@]}; start += batch_size)); do
  topic_pids=()
  batch_end=$((start + batch_size))
  ((batch_end > ${#topics[@]})) && batch_end=${#topics[@]}
  for ((index = start; index < batch_end; index++)); do
    have_msg "${topics[$index]}" 6 &
    topic_pids+=("$!")
  done
  for ((index = start; index < batch_end; index++)); do
    topic="${topics[$index]}"
    pid_index=$((index - start))
    if wait "${topic_pids[$pid_index]}"; then
      echo "OK   $topic"
    else
      echo "MISS $topic"
      plan_reasons+=("no data on $topic")
    fi
  done
done

if have_action /navigate_to_pose; then
  echo "OK   /navigate_to_pose action server"
else
  echo "MISS /navigate_to_pose action server"
  plan_reasons+=("/navigate_to_pose action server not available")
fi

if have_tf map odom; then
  echo "OK   TF map->odom"
else
  echo "MISS TF map->odom"
  plan_reasons+=("TF map->odom missing")
fi
if have_tf odom base_link; then
  echo "OK   TF odom->base_link"
else
  echo "MISS TF odom->base_link"
  plan_reasons+=("TF odom->base_link missing")
fi

fusion_status="$(topic_once /points_merged/status 3 || true)"
if grep -q 'left_fresh.*true' <<<"$fusion_status" \
    && grep -q 'right_fresh.*true' <<<"$fusion_status" \
    && grep -q 'single_lidar_fallback.*false' <<<"$fusion_status"; then
  echo "OK   dual-lidar fusion is fresh without fallback"
else
  echo "BLOCK dual-lidar fusion is degraded"
  arm_reasons+=("both XT-M60 streams must be fresh and single-lidar fallback must be inactive")
fi

watchdog_status="$(topic_once /system_stop_required 3 || true)"
if grep -q 'data: false' <<<"$watchdog_status"; then
  echo "OK   runtime watchdog permits motion"
else
  echo "BLOCK runtime watchdog requests stop"
  arm_reasons+=("runtime watchdog is requesting a stop")
fi

safety_status="$(topic_once /safety_state 3 || true)"
if grep -Eq 'data: .?(CLEAR|WARNING|WARN|SLOWDOWN):' <<<"$safety_status"; then
  echo "OK   safety state permits controlled motion"
else
  echo "BLOCK safety state does not permit motion"
  arm_reasons+=("safety state is STOP, EMERGENCY_STOP, SENSOR_FAULT, or unavailable")
fi

base_status="$(topic_once /base/status 3 || true)"
if grep -q 'motion_control_enabled=false' <<<"$base_status" \
    && grep -q 'real_motion_enabled=false' <<<"$base_status" \
    && grep -q 'motion_initialized=false' <<<"$base_status"; then
  echo "OK   base is in read-only safe standby"
elif grep -q 'motion_control_enabled=true' <<<"$base_status" \
    && grep -q 'last_command_write_ok=true' <<<"$base_status"; then
  echo "OK   base command channel reports healthy writes"
else
  echo "BLOCK base command channel state is inconsistent or unhealthy"
  arm_reasons+=("base must be safely disabled or report healthy enabled writes")
fi
if [[ "$require_motion" == true ]]; then
  if grep -q 'real_motion_enabled=true' <<<"$base_status" \
      && grep -q 'motion_control_enabled=true' <<<"$base_status"; then
    echo "OK   base motion control is enabled"
  else
    echo "BLOCK base motion control is disabled"
    arm_reasons+=("base motion control is not explicitly enabled")
  fi
fi

echo "----------------------------------------"
if [[ ${#plan_reasons[@]} -ne 0 ]]; then
  echo "BLOCKED_TO_PLAN"
  printf '  - %s\n' "${plan_reasons[@]}"
  exit 2
fi
echo "READY_TO_PLAN"

if [[ ${#arm_reasons[@]} -ne 0 ]]; then
  echo "BLOCKED_TO_ARM"
  printf '  - %s\n' "${arm_reasons[@]}"
  [[ "$require_motion" == true ]] && exit 3
  exit 0
fi
echo "READY_TO_ARM"
exit 0
