#!/usr/bin/env bash
# Run from Orin's desktop terminal; closing RViz stops this owned camera session.
set -eo pipefail
cd /home/nvidia/wheelchair
session="cameras_$(date -u +%Y%m%dT%H%M%SZ)_$$"
profile="${1:-monitor_320}"
python3 -s scripts/wc_phase1 cameras --session "$session" --profile "$profile" --duration 3600
trap 'python3 -s scripts/wc_phase1 stop --session "$session"' EXIT
python3 -s scripts/wc_phase1 rviz --session "$session" --view cameras
