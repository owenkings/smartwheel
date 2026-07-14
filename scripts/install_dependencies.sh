#!/usr/bin/env bash
set -euo pipefail

mode="dry-run"
if [[ "${1:-}" == "--apply" ]]; then
  mode="apply"
elif [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--apply]" >&2
  exit 2
fi

packages=(
  build-essential
  cmake
  git
  libeigen3-dev
  libpcl-dev
  python3-numpy
  python3-opencv
  python3-pytest
  python3-yaml
  python3-vcstool
  ros-humble-diagnostic-updater
  ros-humble-robot-localization
  ros-humble-robot-state-publisher
  ros-humble-rosbag2
  ros-humble-rtabmap-ros
  ros-humble-slam-toolbox
  ros-humble-tf2-ros
  ros-humble-xacro
)

echo "Stage A dependency command (not executed by default):"
printf 'sudo apt-get update\nsudo apt-get install -y'
printf ' %q' "${packages[@]}"
printf '\n'

if [[ "$mode" == "apply" ]]; then
  echo "--apply was explicitly requested; installing packages"
  sudo apt-get update
  sudo apt-get install -y "${packages[@]}"
else
  echo "Dry run only. Review the command, then rerun with --apply outside Stage A if authorized."
fi
