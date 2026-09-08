#!/usr/bin/env bash
# Re-apply local modifications to the vendored FAST-LIO (Ericsii/FAST_LIO_ROS2)
# after a fresh clone or `git submodule update` (which would silently revert
# them). The critical one is LASER_POINT_COV 0.001 -> 100.0, WITHOUT which
# FAST-LIO ignores the IMU on the narrow-FOV XT-M60 (yaw stops tracking, map
# swirls). See docs/fastlio_mapping.md §6.2 and auto_test/20260622_lio_rootcause_fix.
#
# The second patch adds an opt-in ZUPT (zero-velocity update) mitigation for
# the drift-after-teleop diagnosis (20260903): see docs/fastlio_mapping.md §8.1
# and xtm60_right_lio.yaml's "zupt:" block. It is dormant unless a route's yaml
# sets zupt.enabled: true (only xtm60_right_lio.yaml does), so re-applying it
# does not change behaviour on the left/dual-radar/mock routes.
#
# Usage: bash patches/apply_fastlio_patches.sh
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
FL="$WS/src/third_party/FAST_LIO_ROS2"
PATCH_COV="$WS/patches/fastlio_laser_point_cov.patch"
PATCH_ZUPT="$WS/patches/fastlio_zupt.patch"

if [[ ! -d "$FL" ]]; then
  echo "ERROR: vendored FAST_LIO_ROS2 not found at $FL"; exit 1
fi

# Idempotent: if already applied (value is 100.0), do nothing.
if grep -q "define LASER_POINT_COV     (100.0)" "$FL/src/laserMapping.cpp" 2>/dev/null; then
  echo "LASER_POINT_COV already = 100.0; patch already applied. Nothing to do."
else
  echo "Applying $PATCH_COV ..."
  git -C "$FL" apply --verbose "$PATCH_COV" && \
    echo "OK." || {
      echo "git apply failed; applying the critical LASER_POINT_COV change directly via sed."
      sed -i 's/#define LASER_POINT_COV     (0.001)/#define LASER_POINT_COV     (100.0)/' \
        "$FL/src/laserMapping.cpp"
      grep -n "LASER_POINT_COV" "$FL/src/laserMapping.cpp" | head -1
    }
fi

# Idempotent: if already applied, do nothing.
if grep -q "zupt_en = false" "$FL/src/laserMapping.cpp" 2>/dev/null; then
  echo "ZUPT mitigation already present; patch already applied. Nothing to do."
else
  echo "Applying $PATCH_ZUPT ..."
  git -C "$FL" apply --verbose "$PATCH_ZUPT" || {
    echo "ERROR: git apply failed for $PATCH_ZUPT. Apply it manually (patch -p1 < $PATCH_ZUPT" \
         "from $FL) after checking laserMapping.cpp still matches the expected upstream shape."
    exit 1
  }
fi

echo "OK. Rebuild: colcon build --packages-select fast_lio"
