#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export WHEELCHAIR_PROJECT_ROOT="$PROJECT_ROOT"
export PYTHONDONTWRITEBYTECODE=1
cd "$PROJECT_ROOT"
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
python3 -s -c 'from wc_runtime.project_paths import require_linux_runtime; require_linux_runtime()'
REPORT_ROOT="$(python3 -s -m wc_runtime.storage_policy reports)"
mkdir -p "$REPORT_ROOT/builds" .phase1_runtime/locks
export PYTHONNOUSERSITE=1 CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
WC_ROS_SETUP="$(python3 -s -c 'from wc_runtime.project_paths import ros_setup_path; print(ros_setup_path())')"
source "$WC_ROS_SETUP"
source install/main/setup.bash
exec 9>.phase1_runtime/locks/heavy_build.lock
flock -n 9 || exit 75
colcon --log-base "$REPORT_ROOT/builds/driver_exit_log" build --base-paths src/wc_xt_driver --build-base build/main --install-base install/main --executor sequential --event-handlers console_direct+
colcon --log-base "$REPORT_ROOT/builds/test_tools_log" build --base-paths tests/integration/ros_tools --build-base build/test_tools --install-base install/test_tools --executor sequential --packages-select wc_test_tools --event-handlers console_direct+
colcon --log-base "$REPORT_ROOT/builds/driver_exit_test_log" test --build-base build/main --packages-select wc_xt_driver --event-handlers console_direct+ --return-code-on-test-failure
PYTHONPATH=src python3 -m unittest discover -s tests/integration/ros_tools/wc_test_tools/test -v
