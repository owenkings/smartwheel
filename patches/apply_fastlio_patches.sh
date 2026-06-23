#!/usr/bin/env bash
# Re-apply local modifications to the vendored FAST-LIO (Ericsii/FAST_LIO_ROS2)
# after a fresh clone or `git submodule update` (which would silently revert
# them). The critical one is LASER_POINT_COV 0.001 -> 100.0, WITHOUT which
# FAST-LIO ignores the IMU on the narrow-FOV XT-M60 (yaw stops tracking, map
# swirls). See docs/fastlio_mapping.md §6.2 and auto_test/20260622_lio_rootcause_fix.
#
# Usage: bash patches/apply_fastlio_patches.sh
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
FL="$WS/src/third_party/FAST_LIO_ROS2"
PATCH="$WS/patches/fastlio_laser_point_cov.patch"

if [[ ! -d "$FL" ]]; then
  echo "ERROR: vendored FAST_LIO_ROS2 not found at $FL"; exit 1
fi

# Idempotent: if already applied (value is 100.0), do nothing.
if grep -q "define LASER_POINT_COV     (100.0)" "$FL/src/laserMapping.cpp" 2>/dev/null; then
  echo "LASER_POINT_COV already = 100.0; patch already applied. Nothing to do."
  exit 0
fi

echo "Applying $PATCH ..."
git -C "$FL" apply --verbose "$PATCH" && \
  echo "OK. Rebuild: colcon build --packages-select fast_lio" || {
    echo "git apply failed; applying the critical LASER_POINT_COV change directly via sed."
    sed -i 's/#define LASER_POINT_COV     (0.001)/#define LASER_POINT_COV     (100.0)/' \
      "$FL/src/laserMapping.cpp"
    grep -n "LASER_POINT_COV" "$FL/src/laserMapping.cpp" | head -1
  }
