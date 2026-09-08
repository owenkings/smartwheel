#!/usr/bin/env bash
# Finalize the formal map-products session.
#
# This compatibility entry point no longer starts a short end-of-run cloud
# window.  map_products_node must have been started before mapping and must
# have received the whole session.  It stops that session first, then asks the
# node to atomically export its products and manifest.
#
# Usage:
#   bash scripts/save_mapping_result.sh
#
# Configure map_name/output_root/cloud topics when starting
# map_export.launch.py (or the relevant mapping launch).  The old positional
# map_name and accumulate_seconds arguments are rejected to avoid implying
# that a late six-second capture is a complete map.

set -Eeuo pipefail

ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if (( $# != 0 )); then
  echo "ERROR: positional map_name/accumulate_seconds arguments are no longer supported." >&2
  echo "       Start map_products_node before mapping, then run this script with no arguments." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
if [[ -f "$ws_root/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ws_root/install/setup.bash"
else
  echo "ERROR: workspace overlay is missing: $ws_root/install/setup.bash" >&2
  exit 2
fi

services="$(ros2 service list 2>/dev/null)" || {
  echo "ERROR: unable to query ROS services; is the mapping session running?" >&2
  exit 2
}
for service in /map_session/stop /map_export/export; do
  if ! grep -Fxq "$service" <<<"$services"; then
    echo "ERROR: required service is unavailable: $service" >&2
    echo "       Start map_products_node before mapping; no late cloud fallback is performed." >&2
    exit 3
  fi
done

stop_reply="$(ros2 service call /map_session/stop std_srvs/srv/Trigger "{}" 2>&1)" || {
  printf '%s\n' "$stop_reply" >&2
  echo "ERROR: map session stop service call failed." >&2
  exit 4
}
printf '%s\n' "$stop_reply"
if ! grep -qE 'success:[[:space:]]*True' <<<"$stop_reply"; then
  echo "ERROR: map session did not stop successfully; export was not requested." >&2
  exit 5
fi

export_reply="$(ros2 service call /map_export/export std_srvs/srv/Trigger "{}" 2>&1)" || {
  printf '%s\n' "$export_reply" >&2
  echo "ERROR: map export service call failed." >&2
  exit 6
}
printf '%s\n' "$export_reply"
if ! grep -qE 'success:[[:space:]]*True' <<<"$export_reply"; then
  echo "ERROR: map export did not report success." >&2
  exit 7
fi

echo "Formal map session export completed.  The service response above contains the version directory."
