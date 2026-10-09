#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export WHEELCHAIR_PROJECT_ROOT="$PROJECT_ROOT"
export PYTHONDONTWRITEBYTECODE=1
cd "$PROJECT_ROOT"
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
python3 -s -c 'from wc_runtime.project_paths import require_linux_runtime; require_linux_runtime()'
python3 -s scripts/check_layout.py
TASK_ROOT="$(python3 -s -m wc_runtime.development_archive build_verify)"
REPORT_ROOT="$TASK_ROOT/validation"
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=2
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
set +u
WC_ROS_SETUP="$(python3 -s -c 'from wc_runtime.project_paths import ros_setup_path; print(ros_setup_path())')"
source "$WC_ROS_SETUP"
set -u
mkdir -p "$REPORT_ROOT/builds" .phase1_runtime/locks
exec 9>.phase1_runtime/locks/heavy_build.lock
flock -n 9 || exit 75
python3 -s - "$REPORT_ROOT/RTABMapConfig.cmake.txt" <<'PY'
import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
prefix = Path(get_package_prefix('rtabmap'))
paths = sorted(prefix.glob('lib/**/rtabmap-0.23/RTABMapConfig.cmake'))
if len(paths) != 1:
    raise RuntimeError('Expected one installed RTABMap 0.23 configuration: ' + repr(paths))
Path(sys.argv[1]).write_bytes(paths[0].read_bytes())
PY
test -f /usr/include/openssl/sha.h && printf 'OPENSSL_SHA_HEADER_PRESENT\n'
colcon --log-base "$REPORT_ROOT/builds/current_log" build --base-paths src --build-base build/main --install-base install/main --executor sequential --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON 2>&1 | tee "$REPORT_ROOT/builds/current.log"
set +u
source install/main/setup.bash
set -u
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
colcon --log-base "$REPORT_ROOT/builds/current_test_log" test --build-base build/main --install-base install/main --packages-select wc_xt_driver wc_slam wc_bringup wc_camera_panel --event-handlers console_direct+ 2>&1 | tee "$REPORT_ROOT/builds/native_tests.log"
colcon test-result --test-result-base build/main --verbose
export WC_PANEL_SAVE_CHOICE_DRIVER="$PROJECT_ROOT/build/main/wc_bringup/panel_save_choice_driver"
test -x "$WC_PANEL_SAVE_CHOICE_DRIVER"
bash tests/run_target_tests.sh
