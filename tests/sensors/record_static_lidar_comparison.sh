#!/usr/bin/env bash
# Root-managed, bounded sensor-only comparison. Caller must hold the scene still.
set -eo pipefail
cd /home/nvidia/wheelchair
export PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/src" OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
python3 -s - <<'PY'
from wc_runtime.cli import target, device_preflight
target()
device_preflight()
PY
if [ "$#" -ne 2 ]; then
  printf '%s\n' 'Usage: bash tests/sensors/record_static_lidar_comparison.sh scene_name "scene description and static assumption"' >&2
  exit 2
fi
case "$1" in ''|*[!a-zA-Z0-9_-]*) printf '%s\n' 'Scene name must use ASCII letters, numbers, underscore or hyphen' >&2; exit 2;; esac
if [ "${#1}" -gt 20 ]; then printf '%s\n' 'Scene name exceeds 20 characters' >&2; exit 2; fi
source /opt/ros/humble/setup.bash
source install/main/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
study=$(python3 -s -c 'import sys; from pathlib import Path; from wc_runtime.storage_policy import resolve_storage_path; print(resolve_storage_path(Path.cwd(), sys.argv[1]))' "reports/lidar_stability/${1}_$(date -u +%Y%m%dT%H%M%SZ)")
mkdir -p "$(dirname "$study")"
mkdir "$study"
recapture=$(python3 -s -c 'from pathlib import Path; from wc_runtime.storage_policy import resolve_storage_path; print(resolve_storage_path(Path.cwd(), "reports/recapture"))')
mkdir -p "$recapture"
printf '%s\n' "$study" > "$recapture/latest_static_comparison.txt"
printf '%s\n' "$2" 'Scene static status is a caller declaration, not independently measured.' > "$study/scene_declaration.txt"
cp tests/sensors/record_static_lidar_comparison.sh "$study/recording_procedure.sh"
git rev-parse HEAD > "$study/source_commit.txt"
python3 -s tests/operations/capture_runtime_state.py --output "$study/before.json"
active_session=''
cleanup() {
  if [ -n "$active_session" ]; then
    python3 -s scripts/wc_phase1 stop --session "$active_session" > "$study/interrupted_cleanup.log" 2>&1
  fi
}
trap cleanup EXIT
snapshot_network() {
  python3 -s - "$1" <<'PY'
import json,pathlib,subprocess,sys,time
root=pathlib.Path('/sys/class/net/eno1')
out={'observed_ns':time.time_ns(),'udp_7687':subprocess.check_output(['ss','-H','-lunp','sport = :7687'],text=True),'net':{}}
for key in ('speed','duplex','mtu','statistics/rx_bytes','statistics/rx_packets','statistics/rx_errors','statistics/rx_dropped','statistics/rx_crc_errors','statistics/rx_frame_errors','statistics/rx_missed_errors','statistics/tx_errors','statistics/tx_dropped'):
    try:out['net'][key]=(root/key).read_text().strip()
    except (OSError,TypeError,UnicodeError) as error:out['net'][key]={'unavailable':type(error).__name__}
pathlib.Path(sys.argv[1]).write_text(json.dumps(out,indent=2)+'\n')
PY
}
for spec in dual_A:dual:left,right left_single:single_left:left right_single:single_right:right dual_B:dual:left,right; do
  IFS=: read -r label mode side_csv <<< "$spec"
  IFS=, read -r -a sides <<< "$side_csv"
  active_session="${1}_${label}_$(date -u +%Y%m%dT%H%M%SZ)"
  python3 -s scripts/wc_phase1 drivers --session "$active_session" --mode "$mode" --duration 65 --device-config-policy preserve_current > "$study/${label}_start.log" 2>&1
  snapshot_network "$study/${label}_network_before.json"
  python3 -s tests/sensors/capture_lidar_stability.py --session "$active_session" --sides "${sides[@]}" --duration 25 --warmup 5 --output "$study/$label"
  snapshot_network "$study/${label}_network_after.json"
  python3 -s scripts/wc_phase1 stop --session "$active_session" > "$study/${label}_stop.log" 2>&1
  python3 -s - "$active_session" "$study/$label/device_records" <<'PY'
import pathlib,sys
source=pathlib.Path('.phase1_runtime/sessions')/sys.argv[1]
out=pathlib.Path(sys.argv[2])
for side in ('left','right','drivers'):
    for path in (source/side).rglob('*'):
        if path.is_file() and not path.is_symlink() and path.suffix in ('.txt','.json','.log') and path.stat().st_size<2_000_000:
            destination=out/path.relative_to(source)
            destination.parent.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(path.read_bytes())
PY
  active_session=''
  python3 -s tests/operations/capture_runtime_state.py --output "$study/${label}_after.json" > /dev/null
done
python3 -s tests/operations/capture_runtime_state.py --output "$study/after.json"
printf 'STATIC_COMPARISON_COMPLETE %s\n' "$study"
