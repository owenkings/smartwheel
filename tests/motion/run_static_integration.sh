#!/usr/bin/env bash
set -eo pipefail
cd /home/nvidia/wheelchair
export PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/src" OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source install/main/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
profile="${1:-monitor_320}"
session="encoder_integrated_$(date -u +%Y%m%dT%H%M%SZ)"
job="reports/encoder/$session"
mkdir -p "$job"
printf '%s\n' "$job" > reports/encoder/latest_integrated.txt
printf '%s\n' "$session" > "$job/session.txt"
python3 -s tests/operations/capture_runtime_state.py --output "$job/before.json" > "$job/before.log"
trap 'python3 -s -m wc_runtime.cli stop --session "$session" > "$job/cleanup.json"' EXIT
python3 -s -m wc_runtime.cli record --session "$session" --duration 35 --imu-poll-period-ms 2.5 > "$job/record_start.json"
python3 -s -m wc_runtime.cli cameras --session "$session" --duration 55 --profile "$profile" > "$job/camera_start.json"
python3 -s -m wc_runtime.cli encoder --session "$session" --duration 25 --rate-hz 10 --allow-read-queries --preview-history-calibration > "$job/encoder_start.json"
python3 -s tests/cameras/check_ros_camera_images.py --duration 22 --min-frames 90 --output "$job/camera_topics.json" > "$job/camera_observe.log"
python3 -s tests/cameras/check_rviz_cameras.py --output-dir "$job/rviz" > "$job/rviz.log"
python3 -s - "$session" "$job" <<'PY'
import json,pathlib,sys,time
session,job=sys.argv[1:];root=pathlib.Path.cwd();p=root/'.phase1_runtime/sessions'/session
deadline=time.monotonic()+45
while True:
 states=[json.loads((p/role/'manifest.json').read_text()) for role in ('record','cameras','encoder')]
 if all(s['state'] in ('STOPPED','FAILED') for s in states):break
 if time.monotonic()>deadline:raise RuntimeError('Managed combined capture did not finish within bound')
 time.sleep(.5)
summary=[{k:s.get(k) for k in ('role','state','exit_code','cleanup_errors')} for s in states]
(pathlib.Path(job)/'completed.json').write_text(json.dumps(summary,indent=2))
assert all(s['state']=='STOPPED' and s['exit_code']==0 for s in summary),summary
records=[json.loads(x) for x in (root/'data/wheel_feedback'/(session+'.transactions.jsonl')).read_text().splitlines()]
good=[r for r in records if r['event']=='transaction_complete']
assert len(good)==250,len(good)
assert all(r['register_words_u16']==[0,0] and r['control_transmissions']==0 for r in good)
assert any(r['event']=='lease_closed' for r in records)
result={'status':'STATIC_CAPTURE_PASS','session':session,'feedback_responses':len(good),'zero_speed_registers':True,'control_transmissions':0,'formal_odometry_eligible':False,'states':summary}
(pathlib.Path(job)/'capture_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
PY
python3 -s tests/timing/check_arrival_basis.py --bag "data/bags/$session" --output "$job/arrival.json" > "$job/arrival.log"
python3 -s tests/sensors/check_live_bag.py --bag "data/bags/$session" --output "$job/data_audit.json" --require-h30 --expected-imu H30-0000000015 --manifest ".phase1_runtime/sessions/$session/record/manifest.json" > "$job/data_audit.log" 2>&1
python3 -s tests/sensors/check_filtered_bag.py --bag "data/bags/$session" --output "$job/filtered_audit.json" > "$job/filtered_audit.log" 2>&1
python3 -s tests/motion/check_recorded_feedback.py --bag "data/bags/$session" --output "$job/feedback_audit.json"
python3 -s tests/operations/capture_runtime_state.py --output "$job/after.json" > "$job/after.log"
printf 'INTEGRATED_RECORD_READY %s\n' "$job"
