#!/usr/bin/env bash
# This runner intentionally refuses Windows/x86: local runs are not Orin evidence.
set -euo pipefail
test "$(id -un)" = nvidia || { printf 'Wrong target user\n' >&2; exit 41; }
test "$(uname -m)" = aarch64 || { printf 'Wrong target architecture\n' >&2; exit 42; }
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
test "$PROJECT_ROOT" = /home/nvidia/wheelchair || { printf 'Unexpected project root\n' >&2; exit 43; }
cd "$PROJECT_ROOT"
export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
REPORT_ROOT="$(python3 -s -m wc_runtime.storage_policy reports)"
mkdir -p "$REPORT_ROOT/test_runs"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-$$"
export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
set +e
python3 -m pytest tests -v --junitxml="$REPORT_ROOT/test_runs/$RUN_ID.xml" 2>&1 | tee "$REPORT_ROOT/test_runs/$RUN_ID.log"
TEST_RC=${PIPESTATUS[0]}
set -e
python3 - "$RUN_ID" "$TEST_RC" <<'PY'
import datetime, hashlib, json, pathlib, platform, socket, subprocess, sys
from wc_runtime.storage_policy import resolve_storage_path
root=pathlib.Path.cwd()
run,rc=sys.argv[1],int(sys.argv[2])
hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for folder in ('src','tests','config') for p in sorted((root/folder).rglob('*')) if p.is_file() and '__pycache__' not in str(p)}
data={'run_id':run,'command':'python3 -m pytest tests -v --junitxml=reports/test_runs/'+run+'.xml', 'exit_code':rc,'status':'PASS' if rc==0 else 'FAIL','level':'SYNTHETIC','hostname':socket.gethostname(),'architecture':platform.machine(),'python':platform.python_version(),'project_root':str(root),'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_sha256':hashes,'log':'reports/test_runs/'+run+'.log'}
resolve_storage_path(root, pathlib.Path('reports/test_runs')/f'{run}.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
PY
exit "$TEST_RC"
