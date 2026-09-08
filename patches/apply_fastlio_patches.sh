#!/usr/bin/env bash
# Reproduce SmartWheel's complete FAST-LIO hardening on the pinned upstream tree.
# The nested FAST-LIO checkout is intentionally ignored by the parent repository,
# so this script and the canonical patch are the durable delivery mechanism.
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
FL="$WS/src/third_party/FAST_LIO_ROS2"
PATCH="$WS/patches/fastlio_smartwheel_hardening.patch"
BASE_COMMIT="2fffc570a25d0df172720bac034fbdb6a13d2162"
EXPECTED_PATCH_SHA256="89912f73b932a33c1fe5c7f56575f13c7f8940b840e3191c6d9007f271f93c74"
EXPECTED_LASER_MAPPING_SHA256="cff8187e4e4cfb2c7c09d6f6478f933c1c2465e687ea66e4f7d212a1ef6e5d10"
EXPECTED_IMU_PROCESSING_SHA256="a024e7040d189e9e75d12211610c33fa5d5829add229829a28a9f70c36622c72"

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

sha256_matches() {
  local expected="$1"
  local file="$2"
  local actual
  actual="$(sha256sum "$file" | awk '{print $1}')"
  [[ "$actual" == "$expected" ]]
}

validate_hardened_tree() {
  local source="$FL/src/laserMapping.cpp"
  local imu="$FL/src/IMU_Processing.hpp"

  sha256_matches "$EXPECTED_LASER_MAPPING_SHA256" "$source" &&
    sha256_matches "$EXPECTED_IMU_PROCESSING_SHA256" "$imu" &&
    grep -Eq 'laser_point_cov[[:space:]]*=[[:space:]]*0\.001' "$source" &&
    grep -q 'project_degenerate_directions' "$source" &&
    grep -q 'velocity_measurement_variance' "$source" &&
    grep -q 'wheel_feedback_healthy' "$source" &&
    grep -q 'path_max_poses' "$source" &&
    grep -q 'adaptive_retry_covariance_scale' "$source" &&
    grep -q 'set_init_requirements' "$imu" &&
    ! grep -Eq 'LASER_POINT_COV.*100\.0|laser_point_cov[[:space:]]*=[[:space:]]*100\.0' "$source"
}

[[ -d "$FL/.git" ]] || fail "FAST_LIO_ROS2 git checkout not found at $FL"
[[ -f "$PATCH" ]] || fail "canonical hardening patch not found at $PATCH"
sha256_matches "$EXPECTED_PATCH_SHA256" "$PATCH" ||
  fail "canonical hardening patch checksum differs; review it before applying"

if validate_hardened_tree; then
  printf 'FAST-LIO hardening is already present and LASER_POINT_COV remains 0.001.\n'
  exit 0
fi

actual_commit="$(git -C "$FL" rev-parse HEAD)"
[[ "$actual_commit" == "$BASE_COMMIT" ]] ||
  fail "FAST-LIO HEAD is $actual_commit, expected $BASE_COMMIT; review/rebase the patch instead of forcing it"

[[ -z "$(git -C "$FL" status --porcelain --untracked-files=no)" ]] ||
  fail "FAST-LIO checkout has tracked changes; preserve/review them before applying the canonical patch"

git -C "$FL" apply --check "$PATCH" ||
  fail "canonical patch does not apply cleanly to the pinned FAST-LIO commit"
git -C "$FL" apply "$PATCH"

validate_hardened_tree ||
  fail "post-apply validation failed; refusing to report success"

printf 'FAST-LIO hardening applied successfully. Rebuild with:\n'
printf '  colcon build --packages-select fast_lio\n'
