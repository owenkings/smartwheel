#!/usr/bin/env bash
# Extract ABI-matched Ubuntu packages inside the project. This never installs
# packages, runs maintainer scripts, or changes the system's OpenCV selection.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
test "$PROJECT_ROOT" = /home/nvidia/wheelchair
test "$(id -un)" = nvidia
test "$(uname -m)" = aarch64
cd "$PROJECT_ROOT"
export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
REPORT_ROOT="$(python3 -s -m wc_runtime.storage_policy reports)"
mkdir -p SDKs/ubuntu_opencv45/packages SDKs/ubuntu_opencv45/sysroot "$REPORT_ROOT/dependencies" .phase1_runtime/locks
exec 9>.phase1_runtime/locks/heavy_build.lock
flock -n 9 || exit 75
cd SDKs/ubuntu_opencv45/packages
components=(core calib3d imgproc highgui stitching photo video videoio features2d flann imgcodecs objdetect dnn ml contrib shape superres videostab viz)
arguments=(libopencv-dev=4.5.4+dfsg-9ubuntu4)
for component in "${components[@]}"; do
  arguments+=("libopencv-$component-dev=4.5.4+dfsg-9ubuntu4" "libopencv-${component}4.5d=4.5.4+dfsg-9ubuntu4")
done
apt-get download "${arguments[@]}"
for package in ./*.deb; do dpkg-deb -x "$package" ../sysroot; done
sha256sum ./*.deb > "$REPORT_ROOT/dependencies/opencv45_packages.sha256"
printf 'Extracted matching OpenCV development/runtime packages under SDKs/ubuntu_opencv45.\n'
