#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

host_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
host_ip="${host_ip:-<orin-ip>}"

echo "http://localhost:8080"
echo "http://${host_ip}:8080"
echo "This script starts the Web UI only. It does not enable motor motion."

exec ros2 launch wheelchair_ui web_user_map.launch.py "$@"
