#!/usr/bin/env bash
# Bounded Orin-only preview. Each lidar stays in its own sensor frame/RViz.
set -euo pipefail
cd /home/nvidia/wheelchair
export PYTHONNOUSERSITE=1
export PYTHONPATH="/home/nvidia/wheelchair/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -s -m wc_runtime.sensor_viewer "$@"
