#!/usr/bin/env bash
# Show all FOUR cameras live in RViz (no SLAM, no motors).
# Brings up camera_adapter_node with the 4-camera config + RViz with 4 Image
# panels (/camera/{front,left,right,rear}/image_raw).
#
#   bash experiments/quad_cam/run_quad_cam.sh
#
# Device indices come from camera_quad.yaml (defaults 0/2/4/6). If two panels
# show the SAME camera, edit config/camera_quad.yaml *_device to use distinct
# capture nodes (run `python3 auto_test/probe_cameras.py` to list them).
#
# Do NOT use set -u (sourcing ROS setup.bash references unbound vars).

ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ws_root"

detect_display() {
  local uid d xa; uid="$(id -u)"
  for d in "${DISPLAY:-}" :1 :0 :1001; do
    [[ -z "$d" ]] && continue
    for xa in "${XAUTHORITY:-}" "/run/user/${uid}/gdm/Xauthority" "$HOME/.Xauthority"; do
      [[ -z "$xa" || ! -f "$xa" ]] && continue
      if DISPLAY="$d" XAUTHORITY="$xa" timeout 3 xdpyinfo >/dev/null 2>&1; then
        export DISPLAY="$d" XAUTHORITY="$xa"; return 0
      fi
    done
  done
  return 1
}
detect_display && echo "Using DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY" || echo "WARN: no X display"

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ws_root/install/setup.bash" 2>/dev/null || true

cfg="$ws_root/install/wheelchair_bringup/share/wheelchair_bringup/config/camera_quad.yaml"
[[ -f "$cfg" ]] || cfg="$ws_root/src/wheelchair_bringup/config/camera_quad.yaml"
rviz_cfg="$ws_root/experiments/quad_cam/quad_cam.rviz"

pids=()
cleanup() {
  trap - EXIT INT TERM
  echo ""; echo "Shutting down quad cam..."
  for p in "${pids[@]:-}"; do kill -INT "$p" >/dev/null 2>&1 || true; done
  pkill -f camera_adapter_node >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "Starting camera_adapter_node with: $cfg"
ros2 run wheelchair_sensors camera_adapter_node --ros-args --params-file "$cfg" &
pids+=("$!")
sleep 4

echo "Starting RViz (4 camera panels): $rviz_cfg"
rviz2 -d "$rviz_cfg" &
pids+=("$!")

echo ""
echo "Four-camera live view running."
echo "  Topics: /camera/{front,left,right,rear}/image_raw"
echo "  If two panels look identical, edit config/camera_quad.yaml *_device."
echo "Ctrl+C to stop."
wait
