#!/usr/bin/env bash
# Read-only health check for the 3D mapping runtime (kills nothing).
# Source ROS first:  source /opt/ros/humble/setup.bash && source install/setup.bash
# Usage: scripts/check_3d_mapping_runtime.sh [measure_seconds]   (default 5)
set -uo pipefail
DUR="${1:-5}"

echo "== duplicate sensor adapters (expect <=1 each; >1 means port contention) =="
for n in imu_adapter_node ultrasonic_adapter_node; do
  c=$(pgrep -fc "[${n:0:1}]${n:1}" 2>/dev/null || true); c=${c:-0}
  flag=""; [ "$c" -gt 1 ] && flag="  <-- DUPLICATE"
  printf '  %-26s %s instance(s)%s\n' "$n" "$c" "$flag"
done

echo "== smartwheel.service (full_system autostart) =="
printf '  is-active: %s\n' "$(systemctl --user is-active smartwheel.service 2>/dev/null || echo unknown)"

echo "== topic presence =="
TOPICS="$(ros2 topic list 2>/dev/null || true)"
for t in /xtm60/left/points /xtm60/right/points /points_merged \
         /rtabmap/cloud_map /rtabmap/grid_map /kiss/map_cloud; do
  if echo "$TOPICS" | grep -qx "$t"; then printf '  %-24s present\n' "$t"; else printf '  %-24s MISSING\n' "$t"; fi
done

echo "== cloud rates over ${DUR}s (best-effort QoS) =="
python3 - "$DUR" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
dur = float(sys.argv[1])
topics = ['/xtm60/left/points', '/xtm60/right/points', '/points_merged',
          '/rtabmap/cloud_map', '/kiss/map_cloud']
c = {}
rclpy.init(); n = Node('rate_check')
def mk(t):
    def cb(_): c[t] = c.get(t, 0) + 1
    return cb
for t in topics:
    n.create_subscription(PointCloud2, t, mk(t), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < dur and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.05)
for t in topics:
    print(f"  {t:24s} {c.get(t, 0) / dur:5.1f} Hz")
n.destroy_node(); rclpy.shutdown()
PY
