#!/usr/bin/env bash
# Retain target-acceptance checks; code root follows this checkout on another Orin.
set -euo pipefail
test "$(id -un)" = nvidia || { printf 'Wrong target user: expected nvidia\n' >&2; exit 41; }
test "$(uname -m)" = aarch64 || { printf 'Wrong target architecture: expected aarch64\n' >&2; exit 42; }
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONNOUSERSITE=1
exec python3 -s "$PROJECT_ROOT/scripts/run_software_tests.py" --project-root "$PROJECT_ROOT" -- "$@"
