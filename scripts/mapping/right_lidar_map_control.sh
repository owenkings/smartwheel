#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 save <absolute-output-prefix> | reload <absolute-posegraph-prefix>" >&2
  exit 2
}

[[ $# -eq 2 ]] || usage
action="$1"
prefix="$2"
[[ "$prefix" = /* ]] || {
  echo "output prefix must be absolute: $prefix" >&2
  exit 2
}

case "$action" in
  save)
    mkdir -p "$(dirname "$prefix")"

    # Humble slam_toolbox's in-process SaveMap callback can starve its own
    # /map publisher and return result=255 ("Failed to spin map subscription").
    # Use the external Nav2 saver so /map is consumed by an independent process.
    timeout 15s ros2 run nav2_map_server map_saver_cli \
      -t /map \
      -f "${prefix}_2d"
    [[ -s "${prefix}_2d.pgm" && -s "${prefix}_2d.yaml" ]] || {
      echo "occupancy map files were not created" >&2
      exit 1
    }

    serialize_output="$(
      timeout 20s ros2 service call \
        /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph \
        "{filename: '${prefix}_posegraph'}" 2>&1
    )"
    printf '%s\n' "${serialize_output}"
    grep -q 'result=0' <<<"${serialize_output}"
    [[ -s "${prefix}_posegraph.posegraph" && -s "${prefix}_posegraph.data" ]] || {
      echo "serialized pose graph files were not created" >&2
      exit 1
    }

    export_output="$(
      timeout 20s ros2 service call \
        /map_export/export \
        std_srvs/srv/Trigger \
        "{}" 2>&1
    )"
    printf '%s\n' "${export_output}"
    grep -q 'success=True' <<<"${export_output}"
    echo "Saved occupancy prefix: ${prefix}_2d"
    echo "Saved pose graph prefix: ${prefix}_posegraph"
    ;;
  reload)
    [[ -s "${prefix}.posegraph" && -s "${prefix}.data" ]] || {
      echo "pose graph files do not exist for prefix: ${prefix}" >&2
      exit 1
    }
    reload_output="$(
      timeout 20s ros2 service call \
        /slam_toolbox/deserialize_map \
        slam_toolbox/srv/DeserializePoseGraph \
        "{filename: '${prefix}', match_type: 1, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}" 2>&1
    )"
    printf '%s\n' "${reload_output}"
    grep -q 'DeserializePoseGraph_Response()' <<<"${reload_output}"
    ;;
  *)
    usage
    ;;
esac
