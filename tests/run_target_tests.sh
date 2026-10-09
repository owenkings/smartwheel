#!/usr/bin/env bash
# Run software regressions on supported Linux hosts; retain actual platform evidence.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export WHEELCHAIR_PROJECT_ROOT="$PROJECT_ROOT"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONNOUSERSITE=1
exec python3 -s "$PROJECT_ROOT/scripts/run_software_tests.py" --project-root "$PROJECT_ROOT" -- "$@"
