#!/usr/bin/env bash
# Frozen cloud alignment only; no sensor, TF or motor is started.
set -euo pipefail
cd /home/nvidia/wheelchair
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="/home/nvidia/wheelchair/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -s -m wc_runtime.calibration_picker --alignment --port 8767 "$@"
