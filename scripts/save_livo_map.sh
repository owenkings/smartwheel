#!/usr/bin/env bash
# Save the LIVO 3D map. The exact mechanism depends on the backend; this script
# tries common options and falls back to dumping a cloud topic to PCD.
#
# Usage: save_livo_map.sh [output_dir] [cloud_topic]
set -u
OUT_DIR="${1:-maps/livo_$(date +%Y%m%d_%H%M%S)}"
CLOUD_TOPIC="${2:-/livo/map_cloud}"
mkdir -p "$OUT_DIR"
echo "Saving to: $OUT_DIR  (cloud topic: $CLOUD_TOPIC)"

# 1) Some backends expose a save service. Edit the service name to match yours.
for SVC in /save_map /fast_livo/save_map /r3live/save_map; do
  if ros2 service list 2>/dev/null | grep -q "^${SVC}$"; then
    echo "Calling save service ${SVC} ..."
    ros2 service call "$SVC" std_srvs/srv/Trigger "{}" && exit 0
  fi
done

# 2) Fallback: dump a point cloud topic to PCD via pcl_ros (if installed).
if ros2 pkg executables pcl_ros 2>/dev/null | grep -q pointcloud_to_pcd; then
  echo "No save service found; dumping ${CLOUD_TOPIC} to PCD with pcl_ros."
  echo "Move the wheelchair as needed, then Ctrl-C to stop."
  ( cd "$OUT_DIR" && ros2 run pcl_ros pointcloud_to_pcd --ros-args -r input:="$CLOUD_TOPIC" )
  exit 0
fi

echo "ERROR: no known save service and pcl_ros pointcloud_to_pcd not available."
echo "Check your backend's documented map-save method (see docs/fast_livo2_r3live_integration.md)."
exit 1
