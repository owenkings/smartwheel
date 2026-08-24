#!/usr/bin/env bash
set -Eeuo pipefail

repo=/home/nvidia/smartwheel
side="${1:-left}"
label="${2:-SIGNAL_CHECK}"
frame_count="${3:-30}"
cleanup_started=0

case "$side" in
  left)
    device_ip=192.168.0.101
    host_ip=192.168.0.100
    ;;
  right)
    device_ip=192.168.1.101
    host_ip=192.168.1.100
    ;;
  *)
    echo "side must be left or right" >&2
    exit 2
    ;;
esac

evidence="$repo/docs/hardware/evidence"
prefix="XT_M60_${label}_20260729"
mkdir -p "$evidence"

cleanup() {
  exit_code=$?
  if [[ "$cleanup_started" -eq 1 ]]; then
    exit "$exit_code"
  fi
  cleanup_started=1
  trap - EXIT INT TERM

  mapfile -t diagnostic_pids < <(
    pgrep -f '/scripts/hardware/xtm60_signal_diagnostic.py( |$)' || true
  )
  if [[ "${#diagnostic_pids[@]}" -gt 0 ]]; then
    kill -TERM "${diagnostic_pids[@]}" 2>/dev/null || true
    sleep 1
  fi

  if pgrep -af '^/usr/bin/python3 .*xtm60' > \
      "$evidence/${prefix}_PROCESS_CHECK.txt"; then
    echo "shutdown_verification=FAIL xtm60_process_still_running" >> \
      "$evidence/${prefix}_PROCESS_CHECK.txt"
    exit_code=2
  else
    echo "shutdown_verification=PASS no_xtm60_process" > \
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

if pgrep -f '^/usr/bin/python3 .*xtm60' >/dev/null; then
  echo "refusing to start: an XT-M60 process is already running" >&2
  exit 2
fi

env \
  LD_PRELOAD="$repo/install/wheelchair_bringup/lib/wheelchair_bringup/libxt_bindshim.so" \
  XT_BIND_IP="$host_ip" \
  XT_BIND_PORT=7687 \
  python3 "$repo/scripts/hardware/xtm60_signal_diagnostic.py" \
  --ip-address "$device_ip" \
  --udp-dest-ip "$host_ip" \
  --udp-dest-port 7687 \
  --frame-count "$frame_count" \
  --output "$evidence/${prefix}.json" \
  --validity-pgm "$evidence/${prefix}.pgm"
