#!/usr/bin/env bash
set -Eeuo pipefail

repo=/home/nvidia/smartwheel
duration_sec="${1:-90}"
label="${2:-SMOKE_90S}"
profile="${3:-dual}"
case "$profile" in
  left)
    enable_left=true
    enable_right=false
    expected_adapters=1
    sides=(left)
    ;;
  right)
    enable_left=false
    enable_right=true
    expected_adapters=1
    sides=(right)
    ;;
  dual)
    enable_left=true
    enable_right=true
    expected_adapters=2
    sides=(left right)
    ;;
  *)
    echo "profile must be left, right, or dual" >&2
    exit 2
    ;;
esac
unit="smartwheel-xtm60-quality-${label,,}"
unit="${unit//_/-}"
evidence="$repo/docs/hardware/evidence"
prefix="XT_M60_QUALITY_${label}_20260729"
cleanup_started=0
worker_pids=()

mkdir -p "$evidence"

cleanup() {
  exit_code=$?
  if [[ "$cleanup_started" -eq 1 ]]; then
    exit "$exit_code"
  fi
  cleanup_started=1
  trap - EXIT INT TERM

  if [[ "${#worker_pids[@]}" -gt 0 ]]; then
    kill -TERM "${worker_pids[@]}" 2>/dev/null || true
    wait "${worker_pids[@]}" 2>/dev/null || true
  fi

  mapfile -t adapter_pids < <(
    pgrep -f '/wheelchair_sensors/xtm60_adapter_node( |$)' || true
  )
  if [[ "${#adapter_pids[@]}" -gt 0 ]]; then
    kill -INT "${adapter_pids[@]}" 2>/dev/null || true
    for _attempt in $(seq 1 40); do
      live=0
      for pid in "${adapter_pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
          live=1
        fi
      done
      [[ "$live" -eq 0 ]] && break
      sleep 0.5
    done
  fi

  systemctl --user stop "$unit.service" 2>/dev/null || true
  systemctl --user is-active "$unit.service" > \
    "$evidence/${prefix}_SERVICE_STATE.txt" 2>&1 || true
  journalctl --user -u "$unit.service" --no-pager > \
    "$evidence/${prefix}_SERVICE_LOG.txt" 2>&1 || true

  if pgrep -af '^/usr/bin/python3 .*xtm60_adapter_node' > \
      "$evidence/${prefix}_PROCESS_CHECK.txt"; then
    echo "shutdown_verification=FAIL process_still_running" >> \
      "$evidence/${prefix}_PROCESS_CHECK.txt"
    exit_code=2
  else
    echo "shutdown_verification=PASS no_adapter_process" > \
      "$evidence/${prefix}_PROCESS_CHECK.txt"
  fi

  if ! python3 "$repo/scripts/hardware/xtm60_udp_shutdown_check.py" \
      --duration-sec 3 \
      --output "$evidence/${prefix}_UDP_SHUTDOWN.json"; then
    exit_code=2
  fi
  exit "$exit_code"
}

trap cleanup EXIT INT TERM

if pgrep -f '/wheelchair_sensors/xtm60_adapter_node( |$)' >/dev/null; then
  echo "refusing to start: an XT-M60 adapter is already running" >&2
  exit 2
fi

systemd-run --user \
  --unit="$unit" \
  --property=KillSignal=SIGINT \
  --property=TimeoutStopSec=45s \
  /bin/bash -lc \
  "cd $repo && source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 launch wheelchair_bringup sensors.launch.py mode:=real publish_description:=false enable_xtm60:=false enable_xtm60_left:=$enable_left enable_xtm60_right:=$enable_right enable_imu:=false enable_ultrasonic:=false enable_camera:=false"

for _attempt in $(seq 1 60); do
  if [[ "$(systemctl --user is-active "$unit.service" 2>/dev/null || true)" == "active" ]] \
      && [[ "$(pgrep -fc '/wheelchair_sensors/xtm60_adapter_node( |$)' || true)" -ge "$expected_adapters" ]]; then
    break
  fi
  sleep 0.5
done

sleep 12
set +u
source /opt/ros/humble/setup.bash
source "$repo/install/setup.bash"
set -u

diagnostic_duration="$(python3 -c "print(float('$duration_sec'))")"
for side in "${sides[@]}"; do
  upper_side="${side^^}"
  python3 "$repo/scripts/hardware/xtm60_cloud_diagnostic.py" \
    --topic "/xtm60/$side/points" \
    --duration-sec "$diagnostic_duration" \
    --source "live_quality_soak_$profile" \
    --output "$evidence/${prefix}_${upper_side}_POINTS.json" \
    > "$evidence/${prefix}_${upper_side}_POINTS.stdout" 2>&1 &
  worker_pids+=("$!")
  python3 "$repo/scripts/hardware/xtm60_quality_topic_diagnostic.py" \
    --topic "/xtm60/$side/quality" \
    --duration-sec "$diagnostic_duration" \
    --output "$evidence/${prefix}_${upper_side}_QUALITY.json" \
    > "$evidence/${prefix}_${upper_side}_QUALITY.stdout" 2>&1 &
  worker_pids+=("$!")
done
python3 "$repo/scripts/hardware/xtm60_resource_monitor.py" \
  --duration-sec "$diagnostic_duration" \
  --interval-sec 5 \
  --min-process-count "$expected_adapters" \
  --output "$evidence/${prefix}_RESOURCES.json" \
  > "$evidence/${prefix}_RESOURCES.stdout" 2>&1 &
resource_pid=$!
worker_pids+=("$resource_pid")
for worker_pid in "${worker_pids[@]}"; do
  wait "$worker_pid"
done
worker_pids=()

for side in "${sides[@]}"; do
  upper_side="${side^^}"
  timeout 10 ros2 topic echo --once "/xtm60/$side/status" std_msgs/msg/String > \
    "$evidence/${prefix}_${upper_side}_STATUS.txt"
done
