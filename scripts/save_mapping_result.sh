#!/usr/bin/env bash
# Save the FAST-LIO mapping result (point cloud PLY + 2D occupancy grid).
# (spec fastlio-narrow-fov-mapping, Task 13 — replaces the old RTAB-Map db export.)
#
# Usage:
#   bash scripts/save_mapping_result.sh [map_name] [accumulate_seconds]
#   map_name           : output basename (default: lab_map_YYYYmmdd_HHMMSS)
#   accumulate_seconds : how long to accumulate /cloud_registered (default 6)
#
# Output -> <repo>/maps/<map_name>/:
#   <map_name>_cloud.ply   accumulated FAST-LIO world cloud (xyz+intensity)
#   <map_name>.pgm/.yaml   2D occupancy grid from /map_2d_from_3d (Nav2)
#
# Run this WHILE a manual_mapping_lio_left session is up (FAST-LIO publishing).
# If the optional RTAB-Map loop backend was enabled (enable_loop_backend:=true),
# its ~/.ros/rtabmap.db is ALSO copied for loop-closure/export reuse.

ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
map_name="${1:-lab_map_$(date +%Y%m%d_%H%M%S)}"
accum_sec="${2:-6}"
out_dir="$ws_root/maps/$map_name"

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ws_root/install/setup.bash" 2>/dev/null || true

mkdir -p "$out_dir"

# 1. Accumulate the FAST-LIO registered world cloud into a merged PLY.
if ros2 topic list 2>/dev/null | grep -q "/cloud_registered"; then
  echo "Accumulating /cloud_registered for ${accum_sec}s -> PLY ..."
  python3 "$ws_root/scripts/lio_save_cloud.py" \
    "$out_dir/${map_name}_cloud.ply" --seconds "$accum_sec" --voxel 0.05 \
    --topic /cloud_registered || echo "WARN cloud save failed."
  ls "$out_dir/${map_name}_cloud.ply" >/dev/null 2>&1 && \
    echo "  PLY ok: $out_dir/${map_name}_cloud.ply" || echo "  WARN no PLY produced."
else
  echo "  (skip 3D cloud: /cloud_registered not available; is FAST-LIO running?)"
fi

# 2. Save the 2D occupancy grid (/map_2d_from_3d) as PGM + YAML for Nav2.
if ros2 topic list 2>/dev/null | grep -q "/map_2d_from_3d"; then
  echo "Saving 2D grid (PGM+YAML) from /map_2d_from_3d ..."
  ( cd "$out_dir" && timeout 25 ros2 run nav2_map_server map_saver_cli \
      -t /map_2d_from_3d -f "$map_name" \
      --ros-args -p map_subscribe_transient_local:=true \
                 -p save_map_timeout:=20.0 2>&1 | tail -5 ) || \
    echo "  WARN 2D grid save failed."
  ls "$out_dir/$map_name.pgm" >/dev/null 2>&1 && echo "  PGM ok" || echo "  WARN no PGM"
else
  echo "  (skip 2D grid: /map_2d_from_3d not available; run while mapping is up)"
fi

# 3. Optional: copy the RTAB-Map loop-backend db if it exists (enable_loop_backend).
db_path="$HOME/.ros/rtabmap.db"
if [[ -f "$db_path" ]]; then
  cp -f "$db_path" "$out_dir/$map_name.db"
  echo "Copied optional RTAB-Map loop-backend db: $out_dir/$map_name.db"
fi

echo
echo "Result in: $out_dir"
ls -lh "$out_dir" 2>/dev/null | sed 's/^/  /'
