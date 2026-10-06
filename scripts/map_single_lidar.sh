#!/usr/bin/env bash
set -eo pipefail
cd /home/nvidia/wheelchair
source /opt/ros/humble/setup.bash
source install/main/setup.bash
export PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/src:$PYTHONPATH" OPENBLAS_NUM_THREADS=1
exec python3 -s -m wc_runtime.single_mapping "$@"
