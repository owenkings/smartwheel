#!/usr/bin/env bash
# Offline frozen cloud picking; no sensor driver or motor command is started.
set -euo pipefail
cd /home/nvidia/wheelchair
export PYTHONNOUSERSITE=1
export PYTHONPATH="/home/nvidia/wheelchair/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -s -m wc_runtime.calibration_picker "$@"
