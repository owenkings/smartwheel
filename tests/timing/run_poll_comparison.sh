#!/usr/bin/env bash
set -eo pipefail
cd /home/nvidia/wheelchair
export PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/src" OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
source /opt/ros/humble/setup.bash
source install/main/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
job="reports/timing/poll_parameters_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$job"
printf '%s\n' "$job" > reports/timing/latest_parameters.txt
flock -n .phase1_runtime/locks/heavy_build.lock colcon --log-base "$job/colcon" build --base-paths src --build-base build/main --install-base install/main --executor sequential --packages-select wc_bringup > "$job/build.log" 2>&1
python3 -s -m pytest -q tests/imu tests/motion tests/operations/test_camera_encoder_cli.py --junitxml="$job/tests.xml" 2>&1 | tee "$job/tests.log"
python3 -s - "$job" <<'PY'
import json,pathlib,subprocess,sys,time
root=pathlib.Path.cwd();p=pathlib.Path(sys.argv[1]);summary=[]
for poll in (10,2.5):
 session='poll'+str(poll).replace('.','p')+'_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
 with (p/(session+'.start.log')).open('x') as log:
  rc=subprocess.call([sys.executable,'-m','wc_runtime.cli','record','--session',session,'--duration','20','--imu-poll-period-ms',str(poll)],stdout=log,stderr=subprocess.STDOUT)
 if rc:summary.append({'session':session,'poll_ms':poll,'state':'START_FAILED','exit_code':rc});break
 manifest=root/'.phase1_runtime/sessions'/session/'record/manifest.json'
 deadline=time.monotonic()+65
 while time.monotonic()<deadline:
  m=json.loads(manifest.read_text())
  if m['state']!='RUNNING':break
  time.sleep(.5)
 else:
  subprocess.run([sys.executable,'-m','wc_runtime.cli','stop','--session',session],timeout=45,capture_output=True)
  m=json.loads(manifest.read_text())
 (p/(session+'.manifest.json')).write_text(json.dumps(m,indent=2))
 summary.append({'session':session,'poll_ms':poll,'state':m['state'],'exit_code':m.get('exit_code')})
 if m['state']!='STOPPED' or m.get('exit_code')!=0:break
(p/'sessions.json').write_text(json.dumps(summary,indent=2));print(json.dumps({'job':str(p),'sessions':summary}))
PY
