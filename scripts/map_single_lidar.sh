#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export WHEELCHAIR_PROJECT_ROOT="$PROJECT_ROOT"
export PYTHONDONTWRITEBYTECODE=1
cd "$PROJECT_ROOT"
export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
WC_ROS_SETUP="$(python3 -s -c 'from wc_runtime.project_paths import ros_setup_path; print(ros_setup_path())')"
source "$WC_ROS_SETUP"
source install/main/setup.bash
export PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/src:$PYTHONPATH" OPENBLAS_NUM_THREADS=1
exec python3 -s -m wc_runtime.single_mapping "$@"
