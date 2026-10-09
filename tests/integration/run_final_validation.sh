#!/usr/bin/env bash
# Root-scheduled target checks. Hardware is explicitly opt-in through --static.
set -eo pipefail
cd /home/nvidia/wheelchair
test "$(hostname)" = ubuntu
test "$(id -un)" = nvidia
test "$(uname -m)" = aarch64
export PYTHONNOUSERSITE=1 PYTHONPATH=/home/nvidia/wheelchair/src OPENBLAS_NUM_THREADS=2
source /opt/ros/humble/setup.bash
source install/main/setup.bash
job="reports/final_validation/$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$job"
printf '%s\n' "$job" > reports/final_validation/latest_job.txt
exec > >(tee "$job/full.log") 2>&1
build="$(cat reports/builds/latest_job.txt)"
test "$(cat "$build/exit_code")" = 0
printf 'BUILD_PREREQUISITE %s\n' "$build"
if test "${1:-}" = --static; then
  session="static_$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p reports/live_static
  printf '%s\n' "$session" > reports/live_static/latest_record_session.txt
  printf '%s\n' "$session" > "$job/static_session.txt"
  # CLI performs current identity, network-address, port and lock checks before
  # starting the reviewed native sensor driver and read-only H30 collector.
  python3 -s scripts/wc_phase1 record --session "$session" --mode dual --duration 30 > "$job/static_start.json"
  deadline=$((SECONDS+65))
  while test "$SECONDS" -lt "$deadline"; do
    state="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["state"])' ".phase1_runtime/sessions/$session/record/manifest.json")"
    if test "$state" != RUNNING; then break; fi
    sleep 1
  done
  python3 -s scripts/wc_phase1 status --session "$session" > "$job/static_status.json"
  python3 - "$session" <<'PY'
import json,pathlib,sys
p=pathlib.Path('.phase1_runtime/sessions')/sys.argv[1]/'record/manifest.json'
d=json.loads(p.read_text());assert d['state']=='STOPPED' and d['exit_code']==0,d
PY
  python3 tests/sensors/check_live_bag.py --bag "/home/nvidia/wheelchair/data/bags/$session" --output "/home/nvidia/wheelchair/reports/live_static/$session-bag_audit.json" --mode dual --require-h30 --expected-imu H30-0000000015 > "$job/static_bag_audit.log" 2>&1
  printf 'STATIC_VERIFIED %s\n' "$session"
elif test "$#" != 0; then
  printf 'Only optional --static is supported\n' >&2
  exit 2
fi
python3 -s scripts/wc_phase1 build > "$job/operator_build.log" 2>&1
printf 'OPERATOR_BUILD_PASS\n'
bash tests/integration/run_ros_contract_checks.sh > "$job/ros_contracts.log" 2>&1
printf 'ROS_CONTRACTS_PASS\n'
set +e
bash tests/integration/run_synthetic_pipeline.sh > "$job/synthetic.log" 2>&1
result=$?
set -e
printf '%s\n' "$result" > "$job/synthetic_exit_code"
printf 'DONE\n' > "$job/done"
printf 'FINAL_SYNTHETIC_EXIT %s\n' "$result"
exit "$result"
