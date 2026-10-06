#!/usr/bin/env bash
# Read-only. Do not source historical workspaces or start any sensor program.
set -u
test "$(id -un)" = nvidia || exit 41
test "$(uname -m)" = aarch64 || exit 42
cd /home/nvidia/wheelchair || exit 43
hostname
id
pwd -P
for path in /AGENTS.md /home/AGENTS.md /home/nvidia/AGENTS.md /home/nvidia/wheelchair/AGENTS.md; do
  if test -f "$path"; then
    printf 'INSTRUCTIONS_FILE %s\n' "$path"
    cat "$path"
  fi
done
cat /etc/os-release
test ! -f /etc/nv_tegra_release || cat /etc/nv_tegra_release
df -h /home/nvidia/wheelchair
free -h
ls -la /home/nvidia/wheelchair
find /home/nvidia -maxdepth 1 -type d -name '*ws*' -print
find /home/nvidia/wheelchair -maxdepth 5 -type f \( -name AGENTS.md -o -name package.xml -o -name metadata.yaml -o -name '*.xtcfg' -o -name 'communication_config.json' -o -iname '*calibration*.yaml' -o -iname '*sdk*.zip' -o -name README.md \) -print | head -160
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
if test -f /opt/ros/humble/setup.bash; then
  set +u
  source /opt/ros/humble/setup.bash
  python3 - <<'PY'
import json
from ament_index_python.packages import get_packages_with_prefixes
p=get_packages_with_prefixes()
print(json.dumps({k:v for k,v in p.items() if any(x in k for x in ('rtabmap','xtsdk','yesense','rviz2','rosbag2','octomap'))},sort_keys=True))
PY
fi
