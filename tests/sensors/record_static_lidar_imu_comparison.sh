#!/usr/bin/env bash
# Root-managed four-stage static capture with H30. No motion commands.
set -eo pipefail
if [ "${1:-}" = --help ] || [ "${1:-}" = -h ]; then
  printf '%s\n' 'Usage: bash tests/sensors/record_static_lidar_imu_comparison.sh scene_name "static scene declaration"' \
    'Four stages: dual, left only, right only, dual; H30 is recorded in every stage.' \
    'Each stage has a 90-second device limit and a 25-second observation after 5 seconds warmup.' \
    'All sensors stop before full offline bag audits. No browser or default input is changed.'
  exit 0
fi
if [ "$#" -ne 2 ]; then printf '%s\n' 'Expected scene name and static declaration; use --help.' >&2; exit 2; fi
case "$1" in ''|*[!a-zA-Z0-9_-]*) printf '%s\n' 'Scene name must use ASCII letters, numbers, underscore or hyphen.' >&2; exit 2;; esac
if [ "${#1}" -gt 20 ]; then printf '%s\n' 'Scene name exceeds 20 characters.' >&2; exit 2; fi
cd /home/nvidia/wheelchair
export PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/src" OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
python3 -s - <<'PY'
from pathlib import Path
from wc_runtime.cli import ROOT, target, device_preflight, imu_preflight
target()
imu_preflight()
device_preflight()
for relative in ('reports/lidar_stability', 'data/bags', '.phase1_runtime/sessions'):
    path = ROOT/relative
    if not path.resolve().is_relative_to(ROOT) or any(p.is_symlink() for p in (path, *path.parents)):
        raise RuntimeError('Redirected project evidence/runtime path: '+str(path))
PY
source /opt/ros/humble/setup.bash
source install/main/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
study="reports/lidar_stability/${1}_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p reports/lidar_stability
mkdir "$study"
printf '%s\n' "$2" 'Scene static status is a caller declaration, not independently measured.' > "$study/scene_declaration.txt"
cp tests/sensors/record_static_lidar_imu_comparison.sh "$study/recording_procedure.sh"
git rev-parse HEAD > "$study/source_commit.txt"
python3 -s tests/operations/capture_runtime_state.py --output "$study/before.json" > /dev/null
active_session=''
cleanup() {
  original_status=$?
  trap - EXIT INT TERM HUP
  if [ -n "$active_session" ]; then
    if ! python3 -s scripts/wc_phase1 stop --session "$active_session" > "$study/interrupted_cleanup.log" 2>&1; then
      original_status=1
    fi
  fi
  exit "$original_status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

snapshot_network() {
  python3 -s - "$1" <<'PY'
import json, pathlib, subprocess, sys, time
interface = pathlib.Path('/sys/class/net/eno1')
out = {'observed_ns':time.time_ns(),
    'udp_7687':subprocess.check_output(['ss','-H','-lunp','sport = :7687'],text=True), 'net':{}}
for key in ('speed','duplex','mtu','statistics/rx_bytes','statistics/rx_packets','statistics/rx_errors',
            'statistics/rx_dropped','statistics/rx_crc_errors','statistics/rx_frame_errors',
            'statistics/rx_missed_errors','statistics/tx_errors','statistics/tx_dropped'):
    try:
        out['net'][key] = (interface/key).read_text().strip()
    except (OSError, TypeError, UnicodeError) as error:
        out['net'][key] = {'unavailable':type(error).__name__}
with pathlib.Path(sys.argv[1]).open('x') as stream:
    json.dump(out, stream, indent=2)
    stream.write('\n')
PY
}

for spec in dual_A:dual:left,right left_single:single_left:left right_single:single_right:right dual_B:dual:left,right; do
  IFS=: read -r label mode side_csv <<< "$spec"
  IFS=, read -r -a sides <<< "$side_csv"
  active_session="${1}_${label}_$(date -u +%Y%m%dT%H%M%SZ)"
  printf 'CAPTURE_STAGE_START %s %s\n' "$label" "$active_session"
  python3 -s scripts/wc_phase1 record --session "$active_session" --mode "$mode" --duration 90 \
    --device-config-policy preserve_current --imu-poll-period-ms 2.5 > "$study/${label}_start.log" 2>&1
  snapshot_network "$study/${label}_network_before.json"
  # Observe H30 beside the unchanged lidar observer; neither observer owns hardware.
  python3 -s - "$active_session" "$mode" "$study/$label" "${sides[@]}" <<'PY'
import json, math, pathlib, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from wc_interfaces.msg import H30Frame
from wc_runtime.cli import ROOT

session, mode, output_value, *sides = sys.argv[1:]
out = ROOT/output_value
runtime = ROOT/'.phase1_runtime/sessions'/session/'record'
bag = ROOT/'data/bags'/session
for path in (out, runtime, bag):
    if not path.resolve().is_relative_to(ROOT) or any(p.is_symlink() for p in (path, *path.parents)):
        raise RuntimeError('Unsafe evidence path')
plan = json.loads((runtime/'plan.json').read_text())
recording = plan.get('recording', {})
if (plan.get('session_id') != session or plan.get('role') != 'record' or
        recording.get('sensor_mode') != mode or recording.get('imu_included') is not True or
        recording.get('device_config_policy') != 'preserve_current' or
        '/wc_mapping/imu/source_frame' not in recording.get('topics', [])):
    raise RuntimeError('Managed recording plan does not match required session, sensors or policy')
bag_commands = [c for c in plan['commands'] if 'ros2' in c and 'bag' in c and 'record' in c]
if len(bag_commands) != 1 or bag_commands[0][bag_commands[0].index('-o')+1] != str(bag):
    raise RuntimeError('Managed recorder does not target the exact session bag')
deadline = time.monotonic()+12
while True:
    manifest = json.loads((runtime/'manifest.json').read_text())
    if manifest.get('session_id') != session or manifest.get('state') != 'RUNNING':
        raise RuntimeError('Managed recording stopped before observation')
    if bag.is_dir() and any(bag.glob('*.db3')):
        break
    if time.monotonic() > deadline:
        raise RuntimeError('Managed bag storage did not become ready within 12 seconds')
    time.sleep(.1)

samples, errors, keys, epochs = [], [], set(), set()
started = time.monotonic()
rclpy.init()
node = Node('wc_static_h30_observer_'+session)
def receive(message):
    try:
        if (message.session_id != session or message.sensor_id != 'H30-0000000015' or
                message.coordinate_convention != 'H30_NATIVE_UNVALIDATED'):
            raise ValueError('H30 source session, identity or coordinates mismatch')
        if not message.linear_acceleration_valid or not message.angular_velocity_valid:
            raise ValueError('H30 acceleration or angular velocity unavailable')
        a, w = message.imu.linear_acceleration, message.imu.angular_velocity
        if not all(math.isfinite(v) for v in (a.x,a.y,a.z,w.x,w.y,w.z)):
            raise ValueError('H30 nonfinite acceleration or angular velocity')
        key = (message.stream_epoch, int(message.frame_sequence))
        if key in keys or not key[0]:
            raise ValueError('H30 duplicate source identity or empty epoch')
        keys.add(key)
        epochs.add(key[0])
        if len(epochs) != 1 or len(samples) >= 100000:
            raise ValueError('H30 epoch changed or sample budget exceeded')
        samples.append({'elapsed_s':time.monotonic()-started, 'sequence':key[1],
            'host_ns':int(message.host_receive_time.sec)*10**9+int(message.host_receive_time.nanosec)})
    except (ValueError, TypeError) as error:
        if len(errors) < 20:
            errors.append(str(error))
subscription = node.create_subscription(H30Frame, '/wc_mapping/imu/source_frame', receive, qos_profile_sensor_data)
child = None
try:
    child = subprocess.Popen([sys.executable, '-s', 'tests/sensors/capture_lidar_stability.py',
        '--session', session, '--sides', *sides, '--duration', '25', '--warmup', '5', '--output', str(out)],
        cwd=ROOT, stdin=subprocess.DEVNULL)
    while child.poll() is None:
        rclpy.spin_once(node, timeout_sec=.05)
        elapsed = time.monotonic()-started
        # The child's 25+5 seconds bound data collection only. Its process also
        # shuts down ROS and compresses two sides of raw/filtered XYZ/intensity
        # while rosbag is writing. Allow that bounded finalization separately;
        # the independently supervised sensor duration remains 90 seconds.
        if elapsed > 72:
            raise RuntimeError('Lidar observer startup/30-second collection/finalization exceeded 72 seconds')
        if elapsed > 5 and (not samples or elapsed-samples[-1]['elapsed_s'] > 2):
            raise RuntimeError('No continuously fresh valid H30 observations')
        if errors:
            raise RuntimeError(errors[0])
    if child.returncode != 0:
        raise RuntimeError('Lidar observer failed: '+str(child.returncode))
    if len(samples) < 100 or samples[-1]['elapsed_s']-samples[0]['elapsed_s'] < 20:
        raise RuntimeError('Insufficient sustained valid H30 observations')
    report = {'schema_version':1, 'status':'VALID_H30_OBSERVED', 'source_session':session,
        'sensor_id':'H30-0000000015', 'topic':'/wc_mapping/imu/source_frame', 'sample_count':len(samples),
        'observer_process_elapsed_s':time.monotonic()-started, 'observer_process_limit_s':72,
        'lidar_collection_duration_s':25, 'lidar_collection_warmup_s':5,
        'first':samples[0], 'last':samples[-1], 'stream_epoch':next(iter(epochs)), 'errors':errors,
        'bag_uri':str(bag), 'record_manifest':str(runtime/'manifest.json'),
        'physical_static_verified':False, 'time_model_validated':False, 'imu_mount_extrinsic_validated':False,
        'note':'Arrival-only source observations; full raw packet and bag audit runs after all sensors stop.'}
    with (out/'imu_observation.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
finally:
    if child is not None and child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
PY
  snapshot_network "$study/${label}_network_after.json"
  python3 -s scripts/wc_phase1 stop --session "$active_session" > "$study/${label}_stop.log" 2>&1
  python3 -s - "$active_session" "$mode" "$study/$label" <<'PY'
import hashlib, json, pathlib, sys
from wc_runtime.cli import ROOT
session, mode, output_value = sys.argv[1:]
out = ROOT/output_value
source = ROOT/'.phase1_runtime/sessions'/session
manifest_path, plan_path = source/'record/manifest.json', source/'record/plan.json'
manifest = json.loads(manifest_path.read_text())
capture = json.loads((out/'capture.json').read_text())
imu = json.loads((out/'imu_observation.json').read_text())
if (manifest.get('session_id') != session or manifest.get('role') != 'record' or
        manifest.get('state') != 'STOPPED' or manifest.get('exit_code') != 0 or manifest.get('cleanup_errors') or
        capture.get('session') != session or capture.get('status') != 'CAPTURE_AUDIT_PASS' or
        imu.get('source_session') != session or imu.get('status') != 'VALID_H30_OBSERVED'):
    raise RuntimeError('Capture/IMU/normal shutdown evidence does not match this session')
for side in capture['sides'].values():
    if any(side[edge].get('session_id') != session for edge in ('first','last')):
        raise RuntimeError('Lidar endpoint source session mismatch')
bag = ROOT/'data/bags'/session
for path in (out, source, bag):
    if not path.resolve().is_relative_to(ROOT) or any(p.is_symlink() for p in (path, *path.parents)):
        raise RuntimeError('Redirected stage evidence path')
if not (bag/'metadata.yaml').is_file():
    raise RuntimeError('Bag metadata absent after normal stop')
for role in ('left','right','drivers','record'):
    for path in (source/role).rglob('*'):
        if path.is_file() and not path.is_symlink() and path.suffix in ('.txt','.json','.log') and path.stat().st_size < 2_000_000:
            destination = out/'device_records'/path.relative_to(source)
            destination.parent.mkdir(parents=True, exist_ok=True)
            with destination.open('xb') as stream:
                stream.write(path.read_bytes())
provenance = {'schema_version':1, 'status':'CLOSED_PENDING_OFFLINE_AUDIT', 'source_session':session,
    'sensor_mode':mode, 'bag_uri':str(bag), 'record_manifest':str(manifest_path), 'record_plan':str(plan_path),
    'record_manifest_sha256':hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
    'record_plan_sha256':hashlib.sha256(plan_path.read_bytes()).hexdigest(),
    'metadata_sha256':hashlib.sha256((bag/'metadata.yaml').read_bytes()).hexdigest(),
    'imu_included':True, 'device_config_policy':'preserve_current', 'time_model_validated':False}
with (out/'bag_provenance.json').open('x') as stream:
    json.dump(provenance, stream, indent=2)
    stream.write('\n')
PY
  active_session=''
  python3 -s tests/operations/capture_runtime_state.py --output "$study/${label}_after.json" > /dev/null
  printf 'CAPTURE_STAGE_STOPPED %s\n' "$label"
done
python3 -s tests/operations/capture_runtime_state.py --output "$study/after.json" > /dev/null
printf 'ALL_CAPTURE_STOPPED %s; offline bag auditing follows\n' "$study"

# Offline only: checked bags are closed before the audited reader opens SQLite.
python3 -s - "$study" <<'PY'
import json, pathlib, subprocess, sys
from wc_runtime.cli import ROOT, name
study = ROOT/sys.argv[1]
stages = {}
for label in ('dual_A','left_single','right_single','dual_B'):
    directory = study/label
    provenance = json.loads((directory/'bag_provenance.json').read_text())
    session, mode = name(provenance['source_session']), provenance['sensor_mode']
    expected_mode = {'dual_A':'dual', 'left_single':'single_left', 'right_single':'single_right', 'dual_B':'dual'}[label]
    if mode != expected_mode:
        raise RuntimeError('Stage mode differs from the four-stage procedure')
    bag = ROOT/'data/bags'/session
    manifest = ROOT/'.phase1_runtime/sessions'/session/'record/manifest.json'
    if provenance['bag_uri'] != str(bag) or provenance['record_manifest'] != str(manifest):
        raise RuntimeError('Stage bag or manifest does not match its source session')
    for path in (directory, bag, manifest):
        if not path.resolve().is_relative_to(ROOT) or any(p.is_symlink() for p in (path, *path.parents)):
            raise RuntimeError('Redirected offline audit evidence path')
    output = directory/'bag_audit.json'
    subprocess.run([sys.executable, '-s', 'tests/sensors/check_live_bag.py', '--bag', str(bag),
        '--mode', mode, '--require-h30', '--expected-imu', 'H30-0000000015',
        '--manifest', str(manifest), '--output', str(output)], cwd=ROOT, check=True, timeout=120)
    audit = json.loads(output.read_text())
    if audit.get('status') != 'DATA_REVIEW_OK' or audit.get('h30_recorded') is not True:
        raise RuntimeError('Offline bag audit failed')
    streams = audit.get('streams')
    expected_sides = {'imu', 'left', 'right'} if mode == 'dual' else {'imu', 'left' if mode == 'single_left' else 'right'}
    if not isinstance(streams, list) or not streams or {s.get('side') for s in streams} != expected_sides:
        raise RuntimeError('Required source streams absent from offline audit')
    for stream in streams:
        if set(stream.get('sessions', {})) != {session}:
            raise RuntimeError('Recorded source session differs from stage session')
    stages[label] = dict(provenance, bag_audit=str(output), audit_status=audit['status'])
with (study/'imu_bags.json').open('x') as stream:
    json.dump({'schema_version':1, 'status':'LIDAR_AND_H30_CAPTURE_AUDITED', 'stages':stages,
        'physical_static_independently_verified':False, 'time_model_validated':False}, stream, indent=2)
    stream.write('\n')
PY
printf 'STATIC_COMPARISON_COMPLETE %s\n' "$study"
