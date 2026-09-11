#!/usr/bin/env bash
# Reproduce SmartWheel's complete FAST-LIO hardening on the pinned upstream tree.
# The nested FAST-LIO checkout is intentionally ignored by the parent repository,
# so this script and the canonical patch are the durable delivery mechanism.
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
FL="$WS/src/third_party/FAST_LIO_ROS2"
PATCH="$WS/patches/fastlio_smartwheel_hardening.patch"
BASE_COMMIT="2fffc570a25d0df172720bac034fbdb6a13d2162"
EXPECTED_PATCH_SHA256="57c8352ba839d9a7f9421985f3f0538f15d272c43eb6540a944eb6c3c1b17a9b"
EXPECTED_LASER_MAPPING_SHA256="ae9df41f67cb62ed6d9dcd27601e3a2aa06eddb0a663b0fbde94c3bee78d3b08"
EXPECTED_IMU_PROCESSING_SHA256="c5f504012b884f09e70a9e6696cb09407687c7333892d531abfd13888f3cee4a"
EXPECTED_WHEEL_HEADER_SHA256="093fef1256aa65482e2f2a5a4cb50c917b96ae818ec20ea41e18d44fdf762898"
EXPECTED_WHEEL_TEST_SHA256="69e89f19095c26e0b9ed084a0606cb2283febd854dabf6f4d9af759c4104737a"
EXPECTED_AIDING_HEADER_SHA256="a3560ccb15b53c3f96b49cff2744618b1f13e7bfa50c6aa1ae09e03288629fed"
EXPECTED_AIDING_TEST_SHA256="bbda1b430ba22287a793a53dd8d7f07bc22d4eb4752fc346430b57617bf492a8"

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
    sha256_matches "$EXPECTED_WHEEL_HEADER_SHA256" "$FL/include/wheel_velocity_update.hpp" &&
    sha256_matches "$EXPECTED_WHEEL_TEST_SHA256" "$FL/tests/test_wheel_velocity_update.cpp" &&
    sha256_matches "$EXPECTED_AIDING_HEADER_SHA256" "$FL/include/state_aiding_safety.hpp" &&
    sha256_matches "$EXPECTED_AIDING_TEST_SHA256" "$FL/tests/test_state_aiding_safety.cpp" &&
    grep -q 'publish.tf_en' "$source" &&
    grep -q 'validate_post_aid' "$source" &&
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
