#!/usr/bin/env bash
# Stage 1: save the RTAB-Map mapping result (db + PLY/PCD point cloud).
#
# Usage:
#   bash scripts/save_mapping_result.sh [map_name] [db_path]
#   map_name : output basename (default: lab_map_YYYYmmdd_HHMMSS)
#   db_path  : source rtabmap database (default: ~/.ros/rtabmap.db)
#
# Output goes to <repo>/maps/<map_name>/:
#   <map_name>.db   copy of the live database (always)
#   <map_name>.ply  exported assembled cloud (if rtabmap-export is available)
#
# Run this while OR after a manual_mapping_left session. The db is updated live
# by RTAB-Map; copying it is safe.

ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
map_name="${1:-lab_map_$(date +%Y%m%d_%H%M%S)}"
db_path="${2:-$HOME/.ros/rtabmap.db}"
out_dir="$ws_root/maps/$map_name"

source /opt/ros/humble/setup.bash 2>/dev/null || true

if [[ ! -f "$db_path" ]]; then
  echo "ERROR: rtabmap database not found: $db_path" >&2
  echo "Is manual_mapping_left running? Check database_path:= argument." >&2
  exit 1
fi

mkdir -p "$out_dir"
cp -f "$db_path" "$out_dir/$map_name.db"
echo "Saved database: $out_dir/$map_name.db ($(du -h "$out_dir/$map_name.db" | cut -f1))"

# Export an assembled point cloud (PLY) from the LiDAR SCANS. The XT-M60 data is
# stored as scan clouds (no depth/stereo), so --scan is required, otherwise
# rtabmap-export reports "empty cloud". --opt 2 uses the db's optimized poses.
if command -v rtabmap-export >/dev/null 2>&1; then
  echo "Exporting assembled cloud with rtabmap-export (--scan) ..."
  rtabmap-export --scan --opt 2 --output "$map_name" --output_dir "$out_dir" \
    "$out_dir/$map_name.db" 2>&1 | tail -6 || \
    echo "WARN rtabmap-export failed; the .db is still saved and re-exportable later."
  if ls "$out_dir"/*.ply >/dev/null 2>&1; then
    echo "  PLY ok: $(ls "$out_dir"/*.ply)"
  else
    echo "  WARN no PLY produced."
  fi
else
  echo "WARN rtabmap-export not found (install ros-humble-rtabmap). The .db is"
  echo "     saved; export later with: rtabmap-export --scan <db>"
fi

# Save the 2D occupancy grid (/rtabmap/grid_map) as PGM + YAML for Nav2.
# /rtabmap/grid_map is latched (transient_local), so map_saver must use a
# matching durability or it times out ("Failed to spin map subscription").
if command -v ros2 >/dev/null 2>&1; then
  source "$ws_root/install/setup.bash" 2>/dev/null || true
  if ros2 topic list 2>/dev/null | grep -q "/rtabmap/grid_map"; then
    echo "Saving 2D grid map (PGM+YAML) from /rtabmap/grid_map ..."
    # map_saver_cli takes -t <topic> and -f <output>. The grid is latched
    # (transient_local); pass the QoS overrides after --ros-args.
    ( cd "$out_dir" && timeout 25 ros2 run nav2_map_server map_saver_cli \
        -t /rtabmap/grid_map -f "$map_name" \
        --ros-args -p map_subscribe_transient_local:=true \
                   -p save_map_timeout:=20.0 2>&1 | tail -5 ) || \
      echo "  WARN 2D grid save failed."
    ls "$out_dir/$map_name.pgm" >/dev/null 2>&1 && echo "  PGM ok" || echo "  WARN no PGM"
  else
    echo "  (skip 2D grid: /rtabmap/grid_map not available; run while mapping is up)"
  fi
fi

echo
echo "Result in: $out_dir"
ls -lh "$out_dir" 2>/dev/null | sed 's/^/  /'
