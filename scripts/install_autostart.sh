#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/install_autostart.sh [--start]

Install the SmartWheel user systemd service. The service is enabled for future
user sessions by default. Use --start to restart it immediately after install.
EOF
}

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
start_now=false

while (($#)); do
  case "$1" in
    --start)
      start_now=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

service_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
service_path="$service_dir/smartwheel.service"
template_path="$workspace_root/systemd/smartwheel.service"

mkdir -p "$service_dir"
sed "s|@WORKSPACE_ROOT@|$workspace_root|g" "$template_path" > "$service_path"
systemctl --user daemon-reload
systemctl --user enable smartwheel.service

if [[ "$start_now" == true ]]; then
  systemctl --user restart smartwheel.service
fi

echo "Installed $service_path"
echo "Service enabled. It will start in future user sessions."
echo "Use 'systemctl --user start smartwheel.service' to start it now."
