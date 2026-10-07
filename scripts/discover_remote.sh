#!/usr/bin/env bash
# Read-only. Do not source historical workspaces or start any sensor program.
set -u
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export WHEELCHAIR_PROJECT_ROOT="$PROJECT_ROOT" PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
cd "$PROJECT_ROOT" || exit 43
python3 -s -c 'from wc_runtime.project_paths import require_linux_runtime; require_linux_runtime()' || exit 42
hostname
id
pwd -P
for path in "$PROJECT_ROOT/AGENTS.md" "$PROJECT_ROOT/docs/task_pack/AGENTS.md"; do
  if test -f "$path"; then
    printf 'INSTRUCTIONS_FILE %s\n' "$path"
    cat "$path"
  fi
done
cat /etc/os-release
test ! -f /etc/nv_tegra_release || cat /etc/nv_tegra_release
df -h "$PROJECT_ROOT"
free -h
ls -la "$PROJECT_ROOT"
find "$PROJECT_ROOT" -maxdepth 5 -type f \( -name AGENTS.md -o -name package.xml -o -name metadata.yaml -o -name '*.xtcfg' -o -name 'communication_config.json' -o -iname '*calibration*.yaml' -o -iname '*sdk*.zip' -o -name README.md \) -print | head -160
ps -eo pid,user,comm --sort=-rss | head -45
ss -luntp
ip -brief addr
ip neigh
ls -l /dev/serial/by-id /dev/serial/by-path /dev/v4l/by-path 2>/dev/null
ls -l /dev/smartwheel* 2>/dev/null
command -v lsusb >/dev/null && lsusb
loginctl list-sessions --no-legend
for command_name in python3 cmake gcc g++ colcon git rviz2; do
  command -v "$command_name" || true
done
python3 - <<'PY'
import importlib.util, json, platform
print(json.dumps({'python':platform.python_version(), 'modules':{name:bool(importlib.util.find_spec(name)) for name in ('numpy','scipy','pytest','yaml','serial','rclpy','rosbag2_py')}}, sort_keys=True))
PY
ls -d /opt/ros/* 2>/dev/null
if WC_ROS_SETUP="$(python3 -s -c 'from wc_runtime.project_paths import ros_setup_path; print(ros_setup_path())')"; then
  set +u
  source "$WC_ROS_SETUP"
  python3 - <<'PY'
import json
from ament_index_python.packages import get_packages_with_prefixes
p=get_packages_with_prefixes()
print(json.dumps({k:v for k,v in p.items() if any(x in k for x in ('rtabmap','xtsdk','yesense','rviz2','rosbag2','octomap'))},sort_keys=True))
PY
fi
