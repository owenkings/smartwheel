#!/usr/bin/env bash
# Live PointCloud+Amp view (no SLAM, no motors): brings up ONE XT-M60 sensor +
# the amp_image_node + RViz showing the amplitude/depth images and the
# amp-coloured point cloud. Mirrors the upper-computer's live single-frame view.
#
#   bash experiments/amp_view/run_amp_view.sh           # LEFT radar (default)
#   SIDE=right bash experiments/amp_view/run_amp_view.sh # RIGHT radar
#
# Do NOT use set -u (sourcing ROS setup.bash references unbound vars).

ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ws_root"

side="${SIDE:-left}"
case "$(echo "$side" | tr '[:upper:]' '[:lower:]')" in
  right) side=right ;;
  *) side=left ;;
esac
if [[ "$side" == "right" ]]; then
  en_left=false; en_right=true
  points_topic="/xtm60/right/points"
else
  en_left=true; en_right=false
  points_topic="/xtm60/left/points"
fi
echo "amp_view SIDE=$side  topic=$points_topic"

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

[[ -f "$ws_root/scripts/stop_mapping.sh" ]] && bash "$ws_root/scripts/stop_mapping.sh" >/dev/null 2>&1 || true
pkill -f amp_image_node >/dev/null 2>&1 || true

pids=()
cleanup() {
  trap - EXIT INT TERM
  echo ""; echo "Shutting down amp view..."
  for p in "${pids[@]:-}"; do kill -INT "$p" >/dev/null 2>&1 || true; done
  pkill -f amp_image_node >/dev/null 2>&1 || true
  [[ -f "$ws_root/scripts/stop_mapping.sh" ]] && bash "$ws_root/scripts/stop_mapping.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# 1. Sensors only: the chosen XT-M60 (organized cloud), no motors, no mapping.
ros2 launch wheelchair_bringup sensors.launch.py \
  mode:=real enable_xtm60:=false enable_xtm60_left:="$en_left" enable_xtm60_right:="$en_right" \
  enable_imu:=false enable_ultrasonic:=false enable_camera:=false \
  publish_description:=true &
pids+=("$!")
sleep 6

# 1b. base_link -> xtm60_{left,right}_link static TF.
# BOTH radar fixed joints were REMOVED from the URDF (those TF edges are now
# owned by ekf-pose-mapping's ground_plane_calibrator_node, which amp_view does
# NOT start). So robot_state_publisher no longer emits base_link->xtm60_*_link
# and the cloud (frame=xtm60_*_link) has no transform to the Fixed Frame -> 3D
# shows nothing. Republish the nominal edge here so amp_view works standalone.
# Heights = the lower (range-sensing) lens: right = 0.52 m, left ~1.5 cm lower
# = 0.505 m. rpy = pi/2 0 pi/2 is the verified z-fwd/y-up/x-left convention.
# (These are nominal display values; the mapping launch auto-calibrates instead.)
if [[ "$side" == "left" ]]; then
  ros2 run tf2_ros static_transform_publisher \
    --x 0.45 --y 0.24 --z 0.505 --roll 1.5708 --pitch 0 --yaw 1.5708 \
    --frame-id base_link --child-frame-id xtm60_left_link &
  pids+=("$!")
  echo "Published static TF base_link -> xtm60_left_link (z=0.505; URDF joint removed; calibrator not run here)."
else
  ros2 run tf2_ros static_transform_publisher \
    --x 0.606 --y -0.24 --z 0.52 --roll 1.5708 --pitch 0 --yaw 1.5708 \
    --frame-id base_link --child-frame-id xtm60_right_link &
  pids+=("$!")
  echo "Published static TF base_link -> xtm60_right_link (x=0.606 y=-0.24 z=0.52; URDF joint removed; calibrator not run here)."
fi

# 2. amp/depth image renderer on the chosen topic.
python3 "$ws_root/experiments/amp_view/amp_image_node.py" \
  --ros-args -p input_topic:="$points_topic" &
pids+=("$!")
sleep 2

# 3. RViz with the amp-view layout.
rviz_cfg="$ws_root/experiments/amp_view/amp_view.rviz"
echo "Starting RViz: $rviz_cfg"
rviz2 -d "$rviz_cfg" &
pids+=("$!")

echo ""
echo "Live PointCloud+Amp view running ($side radar)."
echo "  Image panels: amp_image (jet) / amp_mono (gray) / depth_image"
echo "  3D: cloud points coloured by amp/depth"
echo "Ctrl+C to stop."
wait
