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

# Export an assembled point cloud (PLY). rtabmap-export ships with rtabmap.
if command -v rtabmap-export >/dev/null 2>&1; then
  echo "Exporting assembled cloud with rtabmap-export ..."
  rtabmap-export --output "$out_dir/$map_name" --output_dir "$out_dir" \
    "$out_dir/$map_name.db" 2>&1 | tail -8 || \
    echo "WARN rtabmap-export failed; the .db is still saved and re-exportable later."
else
  echo "WARN rtabmap-export not found (install ros-humble-rtabmap). The .db is"
  echo "     saved; export later with: rtabmap-export --output <name> <db>"
fi

echo
echo "Result in: $out_dir"
ls -lh "$out_dir" 2>/dev/null | sed 's/^/  /'
