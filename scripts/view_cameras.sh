#!/usr/bin/env bash
# Run from the configured Linux desktop terminal; closing RViz stops this owned camera session.
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export WHEELCHAIR_PROJECT_ROOT="$PROJECT_ROOT"
export PYTHONDONTWRITEBYTECODE=1
cd "$PROJECT_ROOT"
session="cameras_$(date -u +%Y%m%dT%H%M%SZ)_$$"
profile="${1:-monitor_320}"
python3 -s scripts/wc_phase1 cameras --session "$session" --profile "$profile" --duration 3600
trap 'python3 -s scripts/wc_phase1 stop --session "$session"' EXIT
python3 -s scripts/wc_phase1 rviz --session "$session" --view cameras
