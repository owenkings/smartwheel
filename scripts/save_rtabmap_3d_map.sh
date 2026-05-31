#!/usr/bin/env bash
# Save the RTAB-Map 3D map.
# Primary map = the database (~/.ros/rtabmap.db). This exports the assembled 3D
# point cloud to PLY offline. If the DB has no poses yet (e.g. a stationary
# single-keyframe session -> rtabmap-export "no odometry poses"), it tells you to
# move and remap, or to use --live-pcd to dump the live /rtabmap/cloud_map topic.
#
# Usage:
#   scripts/save_rtabmap_3d_map.sh [-o OUT_DIR] [-d DB] [-t CLOUD_TOPIC] [--live-pcd] [-h]
# Defaults: OUT_DIR=maps/rtabmap_<ts> (maps/ is gitignored), DB=~/.ros/rtabmap.db,
#           CLOUD_TOPIC=/rtabmap/cloud_map. No sudo required.
set -euo pipefail

OUT_DIR=""
DB="${HOME}/.ros/rtabmap.db"
CLOUD_TOPIC="/rtabmap/cloud_map"
LIVE_PCD=0

usage() { sed -n '2,13p' "$0"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    -o|--output-dir) OUT_DIR="$2"; shift 2;;
    -d|--db)         DB="$2"; shift 2;;
    -t|--cloud-topic) CLOUD_TOPIC="$2"; shift 2;;
    --live-pcd)      LIVE_PCD=1; shift;;
    -h|--help)       usage; exit 0;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2;;
  esac
done

OUT_DIR="${OUT_DIR:-maps/rtabmap_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT_DIR"
echo "Output dir: $OUT_DIR   database: $DB   cloud topic: $CLOUD_TOPIC"

live_pcd() {
  if ros2 pkg executables pcl_ros 2>/dev/null | grep -q pointcloud_to_pcd; then
    echo "Dumping $CLOUD_TOPIC to PCD in $OUT_DIR (mapping must be running; Ctrl-C to stop) ..."
    ( cd "$OUT_DIR" && ros2 run pcl_ros pointcloud_to_pcd --ros-args -r input:="$CLOUD_TOPIC" )
  else
    echo "ERROR: pcl_ros pointcloud_to_pcd unavailable; install ros-humble-pcl-ros." >&2
    return 1
  fi
}

if [[ "$LIVE_PCD" -eq 1 ]]; then
  live_pcd
  exit $?
fi

if ! command -v rtabmap-export >/dev/null 2>&1; then
  echo "ERROR: rtabmap-export not found. Install: sudo apt install ros-humble-rtabmap-ros" >&2
  exit 1
fi
if [[ ! -f "$DB" ]]; then
  echo "ERROR: database $DB not found - run a mapping session first." >&2
  exit 1
fi

echo "Exporting assembled 3D cloud via rtabmap-export ..."
if rtabmap-export --cloud --output rtabmap_cloud --output_dir "$OUT_DIR" "$DB" \
   && ls "$OUT_DIR"/rtabmap_cloud.ply >/dev/null 2>&1; then
  echo "Wrote $OUT_DIR/rtabmap_cloud.ply"
else
  echo "WARN: no cloud exported. A stationary single-keyframe DB has no odometry" >&2
  echo "      poses to optimize. Drive/push the wheelchair to build a real map, or" >&2
  echo "      run with --live-pcd to dump the live $CLOUD_TOPIC topic instead." >&2
  exit 3
fi
