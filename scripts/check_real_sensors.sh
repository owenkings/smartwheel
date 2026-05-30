#!/usr/bin/env bash
set -uo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_root"

radar_ip="${RADAR_IPS:-${RADAR_IP:-192.168.0.101,192.168.1.101}}"
radar_host_cidr="${RADAR_HOST_CIDR:-192.168.0.100/24}"
ultra_port="${ULTRASONIC_PORT:-/dev/smartwheel_ultrasonic}"
imu_port="${IMU_PORT:-/dev/smartwheel_h30_imu}"
radar_iface="${RADAR_IFACE:-auto}"

ok() { printf 'OK   %s\n' "$*"; }
warn() { printf 'WARN %s\n' "$*"; }
fail() { printf 'FAIL %s\n' "$*"; }

source_ros() {
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    warn "ROS Humble setup not found"
    return 1
  fi
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "$workspace_root/install/setup.bash" ]]; then
    source "$workspace_root/install/setup.bash"
  fi
  set -u
}

split_csv() {
  local raw="$1"
  local old_ifs="$IFS"
  IFS=','
  read -r -a radar_ips <<< "$raw"
  IFS="$old_ifs"
}

topic_exists() {
  ros2 topic list 2>/dev/null | grep -qx "$1"
}

topic_hz() {
  local topic="$1"
  local seconds="${2:-6}"
  if ! topic_exists "$topic"; then
    fail "$topic missing"
    return
  fi
  local output
  output="$(timeout "${seconds}s" ros2 topic hz "$topic" 2>&1 || true)"
  local rate
  rate="$(awk '/average rate:/ {rate=$3} END {print rate}' <<<"$output")"
  if [[ -n "$rate" ]]; then
    ok "$topic ${rate} Hz"
  else
    fail "$topic no messages in ${seconds}s"
  fi
}

topic_hz_optional() {
  local topic="$1"
  local seconds="${2:-6}"
  if ! topic_exists "$topic"; then
    warn "$topic missing"
    return
  fi
  local output
  output="$(timeout "${seconds}s" ros2 topic hz "$topic" 2>&1 || true)"
  local rate
  rate="$(awk '/average rate:/ {rate=$3} END {print rate}' <<<"$output")"
  if [[ -n "$rate" ]]; then
    ok "$topic ${rate} Hz"
  else
    warn "$topic no messages in ${seconds}s"
  fi
}

split_csv "$radar_ip"

echo "== Network =="
if scripts/setup_radar_network.sh --iface "$radar_iface" --radar-ip "$radar_ip" --host-cidr "$radar_host_cidr" --check-only --no-ping --quiet; then
  ok "radar route/address check completed"
else
  warn "radar route/address check failed"
fi
radar_ping_ok=0
for item in "${radar_ips[@]}"; do
  ip route get "$item" 2>/dev/null || true
  if ping -c 1 -W 1 "$item" >/dev/null 2>&1; then
    ok "radar ping $item"
    radar_ping_ok=$((radar_ping_ok + 1))
  else
    warn "radar ping $item"
  fi
done
if [[ "$radar_ping_ok" -eq 0 ]]; then
  fail "no radar responded"
fi

echo
echo "== USB =="
if lsusb 2>/dev/null | grep -qi '1a86:7523'; then
  ok "CH340/CH341 USB serial adapter enumerated"
else
  fail "CH340/CH341 USB serial adapter not found in lsusb"
fi
if [[ -e "$ultra_port" ]]; then
  ok "ultrasonic serial port exists: $ultra_port"
else
  fail "ultrasonic serial port missing: $ultra_port"
fi
if [[ -e "$imu_port" ]]; then
  ok "IMU serial port exists: $imu_port"
else
  warn "IMU serial port missing: $imu_port"
fi
if [[ -e /dev/video0 ]]; then
  ok "front camera device exists: /dev/video0"
else
  fail "front camera device missing: /dev/video0"
fi

echo
echo "== ROS topics =="
if source_ros; then
  if topic_exists /xtm60/status; then
    timeout 4s ros2 topic echo /xtm60/status --once 2>/dev/null | sed 's/^/  /'
  else
    warn "/xtm60/status missing; dual mode uses /xtm60/left/status and /xtm60/right/status"
  fi
  topic_hz_optional /xtm60/left/points 6
  topic_hz /xtm60/right/points 6
  topic_hz_optional /xtm60/points 6
  topic_hz /scan 6
  topic_hz /camera/front/image_raw 6
  topic_hz /ultrasonic/range_0 4
  if [[ ! -e "$ultra_port" ]]; then
    warn "/ultrasonic/range_* may be max-range placeholder because $ultra_port is missing"
  else
    timeout 4s ros2 topic echo /ultrasonic/range_0 --once 2>/dev/null | sed 's/^/  /'
  fi
  if topic_exists /imu/data; then
    topic_hz /imu/data 4
  else
    warn "/imu/data missing"
  fi
fi
