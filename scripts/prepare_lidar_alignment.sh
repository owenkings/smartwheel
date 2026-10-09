#!/usr/bin/env bash
# Offline conversion only: never starts ROS, sensors, control or live TF.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$ROOT"
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -s -m wc_runtime.prepare_organized_picker "$@"
