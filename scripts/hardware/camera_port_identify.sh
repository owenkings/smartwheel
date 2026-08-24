#!/usr/bin/env bash
# Interactively identify SmartWheel UVC cameras by physical USB path.
# This tool is read-only with respect to camera and system configuration.

set -euo pipefail

BY_PATH_DIR=/dev/v4l/by-path

camera_links() {
  local link
  shopt -s nullglob
  for link in "$BY_PATH_DIR"/*video-index0; do
    printf '%s\n' "$link"
  done
}

camera_fields() {
  local link=$1 real node device_target interface_name usb_device speed product
  real=$(readlink -f "$link")
  node=$(basename "$real")
  device_target=$(readlink -f "/sys/class/video4linux/$node/device")
  interface_name=$(basename "$device_target")
  usb_device=${interface_name%%:*}
  speed=$(cat "/sys/bus/usb/devices/$usb_device/speed" 2>/dev/null || printf '?')
  product=$(cat "/sys/bus/usb/devices/$usb_device/product" 2>/dev/null || printf '?')
  printf '%s|%s|%s|%s|%s\n' "$usb_device" "$speed" "$node" "$product" "$link"
}

list_cameras() {
  local count=0 link fields usb_device speed node product
  printf 'Time: %s\n' "$(date -Is)"
  printf '%-10s %-8s %-10s %-20s %s\n' \
    'USB path' 'speed' 'node' 'product' 'stable by-path'
  while IFS= read -r link; do
    fields=$(camera_fields "$link")
    IFS='|' read -r usb_device speed node product link <<<"$fields"
    printf '%-10s %-8s %-10s %-20s %s\n' \
      "$usb_device" "${speed}M" "/dev/$node" "$product" "$link"
    count=$((count + 1))
  done < <(camera_links)
  printf 'Physical capture devices: %d\n' "$count"
}

detect_display() {
  local uid display_candidate authority_candidate
  uid=$(id -u)
  for display_candidate in "${DISPLAY:-}" :1 :0 :1001; do
    [[ -n "$display_candidate" ]] || continue
    for authority_candidate in \
      "${XAUTHORITY:-}" \
      "/run/user/${uid}/gdm/Xauthority" \
      "$HOME/.Xauthority"; do
      [[ -n "$authority_candidate" && -f "$authority_candidate" ]] || continue
      if DISPLAY="$display_candidate" XAUTHORITY="$authority_candidate" \
        timeout 3 xdpyinfo >/dev/null 2>&1; then
        export DISPLAY=$display_candidate
        export XAUTHORITY=$authority_candidate
        return 0
      fi
    done
  done
  printf 'No usable X display found. Run from the Orin desktop terminal.\n' >&2
  return 1
}

resolve_camera() {
  local selector=$1 link fields usb_device speed node product
  while IFS= read -r link; do
    fields=$(camera_fields "$link")
    IFS='|' read -r usb_device speed node product link <<<"$fields"
    if [[ "$selector" == "$usb_device" ||
          "$selector" == "$node" ||
          "$selector" == "/dev/$node" ||
          "$selector" == "$link" ||
          "$selector" == "$(basename "$link")" ]]; then
      printf '%s\n' "$fields"
      return 0
    fi
  done < <(camera_links)
  printf 'Camera selector not found: %s\n' "$selector" >&2
  return 1
}

show_camera_fields() {
  local fields=$1 usb_device speed node product link label
  IFS='|' read -r usb_device speed node product link <<<"$fields"
  label="USB ${usb_device} ${speed}M /dev/${node}"
  printf 'Opening %s\n' "$label"
  printf 'Device: %s\n' "$link"
  gst-launch-1.0 -e \
    v4l2src device="$link" \
    ! image/jpeg,width=640,height=480,framerate=30/1 \
    ! jpegdec \
    ! videoconvert \
    ! textoverlay text="$label" valignment=top halignment=left \
        shaded-background=true font-desc='Sans Bold 20' \
    ! ximagesink sync=false
}

usage() {
  cat <<'EOF'
Usage:
  camera_port_identify.sh list
  camera_port_identify.sh watch
  camera_port_identify.sh show <USB-path|video-node|by-path>
  camera_port_identify.sh show-all

Examples:
  bash scripts/hardware/camera_port_identify.sh list
  bash scripts/hardware/camera_port_identify.sh watch
  bash scripts/hardware/camera_port_identify.sh show 2-3.1
  bash scripts/hardware/camera_port_identify.sh show /dev/video0
  bash scripts/hardware/camera_port_identify.sh show-all

Use Ctrl+C to stop watch or live display.
EOF
}

command=${1:-list}
case "$command" in
  list)
    list_cameras
    ;;
  watch)
    while true; do
      printf '\033[2J\033[H'
      list_cameras
      printf '\nUSB tree:\n'
      lsusb -t
      sleep 1
    done
    ;;
  show)
    [[ $# -eq 2 ]] || {
      usage
      exit 2
    }
    detect_display
    show_camera_fields "$(resolve_camera "$2")"
    ;;
  show-all)
    detect_display
    mapfile -t links < <(camera_links)
    ((${#links[@]} > 0)) || {
      printf 'No video-index0 devices found.\n' >&2
      exit 1
    }
    pids=()
    cleanup() {
      trap - EXIT INT TERM
      local pid
      for pid in "${pids[@]:-}"; do
        kill -INT "$pid" >/dev/null 2>&1 || true
      done
      wait "${pids[@]:-}" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM
    for link in "${links[@]}"; do
      show_camera_fields "$(camera_fields "$link")" &
      pids+=("$!")
    done
    wait
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    usage
    exit 2
    ;;
esac
