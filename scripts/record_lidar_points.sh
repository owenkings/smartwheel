#!/usr/bin/env bash
# Four-stage sensor capture followed by offline picker preparation; no browser.
set -euo pipefail
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export WHEELCHAIR_PROJECT_ROOT="$project" PYTHONDONTWRITEBYTECODE=1
# Help works on any host and never imports a device preflight.
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf '%s\n' 'Usage: bash scripts/record_lidar_points.sh [ASCII_scene_name]' \
    '       bash scripts/record_lidar_points.sh --prepare-only reports/lidar_stability/STUDY' \
    'Default scene: manual. Capture takes about 3-4 minutes; keep the rig and scene still.' \
    'Device settings are preserved. Existing recordings and picker inputs are retained.' \
    'Preparation does not start a browser. Run the printed pick_lidar_points command afterwards.'
  exit 0
fi
cd "$project"
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$project/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -s -m wc_runtime.prepare_picker_input "$@"
