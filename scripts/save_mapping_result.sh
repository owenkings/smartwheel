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

# Freeze RTAB-Map before defining the exporter stop boundary.  A current
# Humble RTAB-Map CoreWrapper exposes this as std_srvs/Empty.  Older/custom
# builds may omit it; the post-stop snapshot and trajectory bounds enforced
# by map_products_node remain the fail-closed guard in that case.
if grep -Fxq /rtabmap/pause <<<"$services"; then
  pause_type="$(ros2 service type /rtabmap/pause 2>/dev/null | head -n 1)"
  if [[ "$pause_type" != "std_srvs/srv/Empty" ]]; then
    echo "ERROR: unexpected /rtabmap/pause type: ${pause_type:-<unknown>}" >&2
    exit 4
  fi
  if ! pause_reply="$(
    timeout 10 ros2 service call /rtabmap/pause std_srvs/srv/Empty "{}" 2>&1
  )"; then
    printf '%s\n' "$pause_reply" >&2
    echo "ERROR: RTAB-Map pause service call failed." >&2
    exit 4
  fi
  printf '%s\n' "$pause_reply"
else
  echo "WARNING: /rtabmap/pause is unavailable; post-stop snapshot checks remain mandatory." >&2
fi

stop_reply="$(timeout 10 ros2 service call /map_session/stop std_srvs/srv/Trigger "{}" 2>&1)" || {
  printf '%s\n' "$stop_reply" >&2
  echo "ERROR: map session stop service call failed." >&2
  exit 4
}
printf '%s\n' "$stop_reply"
if ! grep -qE 'success[=:][[:space:]]*True' <<<"$stop_reply"; then
  echo "ERROR: map session did not stop successfully; export was not requested." >&2
  exit 5
fi

# The optimized cloud/path publisher polls RTAB-Map.  STOP can therefore be
# acknowledged before the first matching post-stop snapshot arrives.  Retry
# only this read/finalize request for a bounded interval; each failed response
# explains which product is still unavailable.
export_deadline=$((SECONDS + 30))
export_reply=""
while (( SECONDS < export_deadline )); do
  if export_reply="$(
    timeout 10 ros2 service call /map_export/export std_srvs/srv/Trigger "{}" 2>&1
  )"; then
    printf '%s\n' "$export_reply"
    if grep -qE 'success[=:][[:space:]]*True' <<<"$export_reply"; then
      break
    fi
  else
    printf '%s\n' "$export_reply" >&2
  fi
  sleep 1
done
if ! grep -qE 'success[=:][[:space:]]*True' <<<"$export_reply"; then
  echo "ERROR: no fresh, matching backend cloud/path became exportable within 30 seconds." >&2
  exit 6
fi

# The completion publisher is transient-local and is emitted from the same
# synchronous export transaction.  Read the exact path and ensure it is also
# present in this service response, preventing an unrelated/stale path from
# being accepted.
if ! completed_reply="$(
  timeout 10 ros2 topic echo --once --full-length --qos-durability transient_local \
    --field data /map_export/completed std_msgs/msg/String 2>&1
)"; then
  printf '%s\n' "$completed_reply" >&2
  echo "ERROR: timed out waiting for /map_export/completed." >&2
  exit 7
fi
completed_path="$(sed -n '1p' <<<"$completed_reply")"
if [[ -z "$completed_path" ]] || ! grep -Fq -- "$completed_path" <<<"$export_reply"; then
  echo "ERROR: completion topic path does not match this export response." >&2
  exit 7
fi

python3 - "$completed_path" <<'PY'
import json
import sys
from pathlib import Path

bundle = Path(sys.argv[1])
manifest_path = bundle / "manifest.json"
if not bundle.is_dir():
    raise SystemExit(f"ERROR: exported bundle directory is missing: {bundle}")
if (bundle / ".incomplete").exists():
    raise SystemExit(f"ERROR: exported bundle still has .incomplete: {bundle}")
try:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
except (OSError, UnicodeError, json.JSONDecodeError) as exc:
    raise SystemExit(f"ERROR: cannot read completed manifest: {exc}")
if not isinstance(manifest, dict) or manifest.get("complete") is not True:
    raise SystemExit("ERROR: manifest complete flag is not true")
if not isinstance(manifest.get("files"), list) or not manifest["files"]:
    raise SystemExit("ERROR: completed manifest has no file entries")
PY

echo "Formal map session export completed: $completed_path"
