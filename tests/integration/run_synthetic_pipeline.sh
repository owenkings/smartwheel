#!/usr/bin/env bash
set -eo pipefail
cd /home/nvidia/wheelchair
test "$(id -un)" = nvidia
test "$(uname -m)" = aarch64
export PYTHONNOUSERSITE=1 PYTHONPATH=/home/nvidia/wheelchair/src ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=2
source /opt/ros/humble/setup.bash
source install/main/setup.bash
source install/test_tools/setup.bash
fixture_args=()
prefix=synthetic
if test "${WC_WHEEL_FILTER_FIXTURE:-0}" = 1; then
  fixture_args+=(--with-wheel-filter-fixture)
  prefix=synthetic_wheel
fi
session="${prefix}_$(date -u +%Y%m%dT%H%M%SZ)"
config="$(python3 -m wc_runtime.synthetic_source --prepare "$session" --period-s "${WC_TEST_PERIOD_S:-0.5}" "${fixture_args[@]}")"
root="$(dirname "$config")"
report="reports/synthetic/$session"
mkdir -p "$report"
evidence="/home/nvidia/wheelchair/$report"
printf '%s\n' "$session" > reports/synthetic/latest_session.txt
printf '%s\n' "$config" > "$report/config_path.txt"
install/test_tools/wc_test_tools/lib/wc_test_tools/tf_ownership_probe --ros-args -p output:="$evidence/tf_native.json" -p duration_s:=100.0 -p session_id:="$session" > "$report/tf_probe.log" 2>&1 &
tf_probe=$!
python3 tests/integration/check_icp_pipeline.py --session-config "$config" --output "$evidence/ros_pipeline.json" --tf-evidence "$evidence/tf_native.json" --duration 110 --close-snapshot > "$report/observer.log" 2>&1 &
observer=$!
wheel_observer=''
private_probe=''
if test "${WC_WHEEL_FILTER_FIXTURE:-0}" = 1; then
  install/test_tools/wc_test_tools/lib/wc_test_tools/tf_ownership_probe --ros-args -p output:="$evidence/private_tf_native.json" -p duration_s:=90.0 -p session_id:="$session" -p wheel_private_mode:=true > "$report/private_tf_probe.log" 2>&1 &
  private_probe=$!
  python3 tests/integration/check_wheel_guess.py --session-config "$config" --output "$evidence/wheel_guess.json" --private-tf-evidence "$evidence/private_tf_native.json" --duration 100 > "$report/wheel_guess.log" 2>&1 &
  wheel_observer=$!
fi
cleanup() {
  python3 -s scripts/wc_phase1 stop --session "$session" > "$report/stop.json" 2>&1 || true
  if kill -0 "$observer" 2>/dev/null; then kill -INT "$observer"; fi
  if kill -0 "$tf_probe" 2>/dev/null; then kill -INT "$tf_probe"; fi
  if test -n "$wheel_observer" && kill -0 "$wheel_observer" 2>/dev/null; then kill -INT "$wheel_observer"; fi
  if test -n "$private_probe" && kill -0 "$private_probe" 2>/dev/null; then kill -INT "$private_probe"; fi
}
trap cleanup EXIT
ready=0
for attempt in $(seq 1 100); do
  if grep -q OBSERVER_READY "$report/observer.log" && grep -q TF_PROBE_READY "$report/tf_probe.log"; then
    if test -z "$wheel_observer" || { grep -q WHEEL_GUESS_OBSERVER_READY "$report/wheel_guess.log" && grep -q TF_PROBE_READY "$report/private_tf_probe.log"; }; then ready=1; break; fi
  fi
  if ! kill -0 "$observer" 2>/dev/null; then cat "$report/observer.log"; exit 2; fi
  sleep 0.1
done
test "$ready" = 1 || { cat "$report/observer.log"; exit 3; }
python3 -s scripts/wc_phase1 map --session-config "$config" --with-synthetic-source --duration 125 > "$report/start.json"
python3 -s tests/integration/sample_resources.py --session "$session" --duration 100 --output "$evidence/resources.json" > "$report/resources.log" 2>&1 &
resource_sampler=$!
set +e
wait "$observer"
result=$?
wait "$tf_probe"
tf_result=$?
wait "$resource_sampler"
resource_result=$?
wheel_result=0
private_result=0
if test -n "$wheel_observer"; then
  wait "$wheel_observer"
  wheel_result=$?
  wait "$private_probe"
  private_result=$?
  printf '%s\n' "$wheel_result" > "$report/wheel_exit_code"
  printf '%s\n' "$private_result" > "$report/private_tf_exit_code"
fi
set -e
printf '%s\n' "$tf_result" > "$report/tf_exit_code"
printf '%s\n' "$resource_result" > "$report/resources_exit_code"
if test "$wheel_result" != 0 || test "$private_result" != 0 || test "$tf_result" != 0 || test "$resource_result" != 0; then result=1; fi
printf '%s\n' "$result" > "$report/exit_code"
cat "$report/observer.log"
exit "$result"
