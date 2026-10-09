#!/usr/bin/env bash
set -eo pipefail
cd /home/nvidia/wheelchair
export PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/src" OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source install/main/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
job="reports/cameras/exit_budget_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$job"
printf '%s\n' "$job" > reports/cameras/latest_exit_budget.txt
session="camera_exit_$(date -u +%Y%m%dT%H%M%SZ)"
printf '%s\n' "$session" > "$job/session.txt"
python3 -s -m wc_runtime.runtime_snapshot --output "$job/before.json" > "$job/before.log"
python3 -s -m wc_runtime.cli cameras --session "$session" --duration 40 --profile monitor_320 > "$job/start.json"
trap 'python3 -s -m wc_runtime.cli stop --session "$session" > "$job/stop.json"' EXIT
python3 -s tests/cameras/check_ros_camera_images.py --duration 18 --min-frames 90 --output "$job/topics.json" > "$job/topics.log"
python3 -s tests/cameras/check_rviz_cameras.py --output-dir "$job/rviz" > "$job/rviz.log"
python3 -s - "$session" "$job" <<'PY'
import pathlib,sys,json,time
s,j=sys.argv[1:];r=pathlib.Path.cwd();f=r/'.phase1_runtime/sessions'/s/'cameras/manifest.json';deadline=time.monotonic()+35
while True:
 d=json.loads(f.read_text())
 if d['state'] in ('STOPPED','FAILED'):break
 if time.monotonic()>deadline:raise RuntimeError('Camera completion deadline exceeded')
 time.sleep(.5)
logs={}
for p in f.parent.glob('process-*.log'):
 rows=p.read_text(errors='replace').splitlines();logs[p.name]={'jpeg_warnings':sum('Corrupt JPEG' in x for x in rows),'stop_event':json.loads(rows[-1])}
out={'session':s,'state':d['state'],'exit_code':d.get('exit_code'),'sigint_grace_s':d['sigint_grace_s'],'cameras':logs}
(pathlib.Path(j)/'result.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
assert d['state']=='STOPPED' and d['exit_code']==0
assert all(v['stop_event']['cleanup']['termination']=='normal' for v in logs.values())
PY
python3 -s -m wc_runtime.runtime_snapshot --output "$job/after.json" > "$job/after.log"
