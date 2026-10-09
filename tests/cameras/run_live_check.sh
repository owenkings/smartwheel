#!/usr/bin/env bash
set -eo pipefail
cd /home/nvidia/wheelchair
export PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/src" OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source install/main/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
job="reports/cameras/live_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$job"
printf '%s\n' "$job" > reports/cameras/latest_live.txt
python3 -s tests/cameras/check_ros_message_abi.py > "$job/message_abi.json"
session="camera320_$(date -u +%Y%m%dT%H%M%SZ)"
printf '%s\n' "$session" > "$job/session.txt"
python3 -s -m wc_runtime.cli cameras --session "$session" --duration 110 > "$job/start.json"
trap 'python3 -s -m wc_runtime.cli stop --session "$session" > "$job/stop.json"' EXIT
python3 -s tests/cameras/check_ros_camera_images.py --duration 18 --min-frames 70 --output "$job/topics.json" > "$job/observe.log"
python3 -s tests/cameras/check_rviz_cameras.py --output-dir "$job/rviz" > "$job/gui.log"
python3 -s - "$job" <<'PY'
import json,pathlib,sys
p=pathlib.Path(sys.argv[1]);r=json.loads((p/'topics.json').read_text())
print(json.dumps({'job':str(p),'status':r['status'],'cameras':{k:{'frames':v['frames'],'hz':v['observed_hz'],'shape':v['last_shape']} for k,v in r['cameras'].items()}}))
print((p/'rviz/result.json').read_text())
PY
