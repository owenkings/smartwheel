#!/usr/bin/env bash
# Frozen cloud alignment only; no sensor, TF or motor is started.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export WHEELCHAIR_PROJECT_ROOT="$PROJECT_ROOT"
export PYTHONDONTWRITEBYTECODE=1
cd "$PROJECT_ROOT"
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -s -m wc_runtime.calibration_picker --alignment --port 8767 "$@"
