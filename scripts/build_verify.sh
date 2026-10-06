set -euo pipefail
cd /home/nvidia/wheelchair
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
REPORT_ROOT="$(python3 -s -m wc_runtime.storage_policy reports)"
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=2
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
set +u
source /opt/ros/humble/setup.bash
set -u
mkdir -p "$REPORT_ROOT/builds" .phase1_runtime/locks
exec 9>.phase1_runtime/locks/heavy_build.lock
flock -n 9 || exit 75
cat /opt/ros/humble/lib/aarch64-linux-gnu/rtabmap-0.23/RTABMapConfig.cmake > "$REPORT_ROOT/RTABMapConfig.cmake.txt"
test -f /usr/include/openssl/sha.h && printf 'OPENSSL_SHA_HEADER_PRESENT\n'
colcon --log-base "$REPORT_ROOT/builds/current_log" build --base-paths src/wc_interfaces src/wc_xt_driver src/wc_bringup src/wc_slam --build-base build/main --install-base install/main --executor sequential --event-handlers console_direct+ 2>&1 | tee "$REPORT_ROOT/builds/current.log"
set +u
source install/main/setup.bash
set -u
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
colcon --log-base "$REPORT_ROOT/builds/current_test_log" test --build-base build/main --install-base install/main --packages-select wc_xt_driver wc_slam --event-handlers console_direct+ 2>&1 | tee "$REPORT_ROOT/builds/native_tests.log"
colcon test-result --test-result-base build/main --verbose
bash tests/run_target_tests.sh
