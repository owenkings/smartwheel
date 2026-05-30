#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/setup_radar_network.sh [options]

<<<<<<< HEAD
Configure non-default-route Ethernet aliases for the XT-M60 radar pair.

Defaults:
  radar IPs     192.168.0.101,192.168.1.101
  Orin address  192.168.0.100/24,192.168.1.100/24

The two radars sit on isolated subnets so the vendor SDK does not see
broadcast traffic from both at once. The Orin Ethernet interface gets one
host alias per subnet.
=======
Configure a non-default-route Ethernet alias for the XT-M60 radar.

Defaults:
  radar IPs     10.55.231.101
  Orin address  10.55.231.100/24
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b

Options:
  --iface IFACE        Ethernet interface, or "auto" to detect. Default: auto.
  --radar-ip IP[,IP]   Radar IP or comma-separated radar IPs.
<<<<<<< HEAD
                       Default: 192.168.0.101,192.168.1.101.
  --host-cidr CIDR[,CIDR]
                       Orin radar-side address(es), comma-separated when more
                       than one subnet is needed.
                       Default: 192.168.0.100/24,192.168.1.100/24.
  --gateway IP         Documented radar gateway. Not installed as default route.
                       Default: 192.168.0.1.
=======
                       Default: 10.55.231.101.
  --host-cidr CIDR     Orin radar-side address. Default: 10.55.231.100/24.
  --gateway IP         Documented radar gateway. Not installed as default route.
                       Default: 10.55.0.1.
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
  --check-only         Only print current route/address/ping status.
  --dry-run            Print commands without changing the system.
  --no-ping            Do not ping the radar after configuration.
  --install-service    Install a root systemd oneshot service for boot-time setup.
  --uninstall-service  Remove that systemd service.
  --quiet              Reduce informational output.
  -h, --help           Show this help.

This script intentionally does not set or replace the default gateway, so WiFi
or another internet connection keeps working.
EOF
}

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
iface="${RADAR_IFACE:-auto}"
<<<<<<< HEAD
radar_ip="${RADAR_IPS:-${RADAR_IP:-192.168.0.101,192.168.1.101}}"
host_cidr="${RADAR_HOST_CIDRS:-${RADAR_HOST_CIDR:-192.168.0.100/24,192.168.1.100/24}}"
radar_gateway="${RADAR_GATEWAY:-192.168.0.1}"
=======
radar_ip="${RADAR_IP:-10.55.231.101}"
host_cidr="${RADAR_HOST_CIDR:-10.55.231.100/24}"
radar_gateway="${RADAR_GATEWAY:-10.55.0.1}"
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
check_only=false
dry_run=false
ping_after=true
install_service=false
uninstall_service=false
quiet=false
service_name="smartwheel-radar-network.service"
service_path="/etc/systemd/system/$service_name"

while (($#)); do
  case "$1" in
    --iface)
      iface="${2:?missing value for --iface}"
      shift 2
      ;;
    --radar-ip)
      radar_ip="${2:?missing value for --radar-ip}"
      shift 2
      ;;
    --host-cidr)
      host_cidr="${2:?missing value for --host-cidr}"
      shift 2
      ;;
    --gateway)
      radar_gateway="${2:?missing value for --gateway}"
      shift 2
      ;;
    --check-only)
      check_only=true
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    --no-ping)
      ping_after=false
      shift
      ;;
    --install-service)
      install_service=true
      shift
      ;;
    --uninstall-service)
      uninstall_service=true
      shift
      ;;
    --quiet)
      quiet=true
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

log() {
  if [[ "$quiet" != true ]]; then
    printf '%s\n' "$*"
  fi
}

warn() {
  printf 'WARN %s\n' "$*" >&2
}

fail() {
  printf 'ERROR %s\n' "$*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "missing command: $1"
}

host_ip="${host_cidr%/*}"

split_csv() {
  local raw="$1"
  local old_ifs="$IFS"
  IFS=','
  read -r -a radar_ips <<< "$raw"
  IFS="$old_ifs"
}

<<<<<<< HEAD
split_cidr_csv() {
  local raw="$1"
  local old_ifs="$IFS"
  IFS=','
  read -r -a host_cidrs <<< "$raw"
  IFS="$old_ifs"
}

=======
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
first_radar_ip() {
  printf '%s\n' "${radar_ips[0]}"
}

<<<<<<< HEAD
# Pick the host source IP that shares a /24 with the given radar IP.
# Falls back to the first configured host IP if no /24 matches.
host_src_for_radar() {
  local radar="$1"
  local cidr cidr_ip
  for cidr in "${host_cidrs[@]}"; do
    cidr_ip="${cidr%/*}"
    if [[ "${radar%.*}" == "${cidr_ip%.*}" ]]; then
      printf '%s\n' "$cidr_ip"
      return 0
    fi
  done
  cidr_ip="${host_cidrs[0]%/*}"
  printf '%s\n' "$cidr_ip"
}

=======
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
run_root() {
  if [[ "$dry_run" == true ]]; then
    printf '+'
    printf ' %q' "$@"
    printf '\n'
    return 0
  fi
  if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
    "$@"
    return
  fi
  if sudo -n true >/dev/null 2>&1; then
    sudo "$@"
    return
  fi
  fail "root permission required because radar network is not configured. Run: sudo $workspace_root/scripts/setup_radar_network.sh --iface $iface"
}

same_24() {
  local a="$1"
  local b="$2"
  [[ "${a%.*}" == "${b%.*}" ]]
}

detect_iface() {
  if [[ "$iface" != "auto" ]]; then
    printf '%s\n' "$iface"
    return 0
  fi

  local routed
  routed="$(ip route get "$(first_radar_ip)" 2>/dev/null | awk '
    {for (i=1; i<=NF; i++) if ($i == "dev") {print $(i+1); exit}}
  ' || true)"
  if [[ -n "$routed" && "$routed" != "lo" && "$routed" != wl* && "$routed" != wlan* && -e "/sys/class/net/$routed" ]]; then
    local routed_type
    routed_type="$(cat "/sys/class/net/$routed/type" 2>/dev/null || true)"
    if [[ "$routed_type" == "1" ]]; then
      printf '%s\n' "$routed"
      return 0
    fi
  fi

  local name carrier type
  for name in eno1 eth0 enp0s31f6 enp1s0 enp2s0 enx* en* eth*; do
    [[ -e "/sys/class/net/$name" ]] || continue
    [[ "$name" == "lo" || "$name" == wl* || "$name" == wlan* ]] && continue
    type="$(cat "/sys/class/net/$name/type" 2>/dev/null || true)"
    [[ "$type" == "1" ]] || continue
    carrier="$(cat "/sys/class/net/$name/carrier" 2>/dev/null || echo 1)"
    if [[ "$carrier" == "1" ]]; then
      printf '%s\n' "$name"
      return 0
    fi
  done

  for name in /sys/class/net/*; do
    name="${name##*/}"
    [[ "$name" == "lo" || "$name" == wl* || "$name" == wlan* || "$name" == docker* || "$name" == br-* ]] && continue
    type="$(cat "/sys/class/net/$name/type" 2>/dev/null || true)"
    [[ "$type" == "1" ]] || continue
    printf '%s\n' "$name"
    return 0
  done

  fail "could not auto-detect an Ethernet interface. Pass --iface eno1 or --iface eth0."
}

show_status() {
  local dev="$1"
  log "Radar network status"
  log "  iface:      $dev"
<<<<<<< HEAD
  log "  host CIDRs: ${host_cidrs[*]}"
=======
  log "  host CIDR:  $host_cidr"
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
  log "  radar IPs:  ${radar_ips[*]}"
  log "  gateway:    $radar_gateway (not installed as default route)"
  if [[ "$quiet" != true ]]; then
    ip -br addr show dev "$dev" 2>/dev/null || true
    for item in "${radar_ips[@]}"; do
      ip route get "$item" 2>/dev/null || true
    done
  fi
  if [[ "$ping_after" == true ]]; then
<<<<<<< HEAD
    local ok_count=0
    for item in "${radar_ips[@]}"; do
      if ping -c 1 -W 1 -I "$dev" "$item" >/dev/null 2>&1; then
        log "  ping $item: OK"
        ok_count=$((ok_count + 1))
      else
        warn "radar ping failed: $item via $dev"
      fi
    done
    [[ "$ok_count" -gt 0 ]]
=======
    local failed=false
    for item in "${radar_ips[@]}"; do
      if ping -c 1 -W 1 -I "$dev" "$item" >/dev/null 2>&1; then
        log "  ping $item: OK"
      else
        warn "radar ping failed: $item via $dev"
        failed=true
      fi
    done
    [[ "$failed" == false ]]
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
  fi
}

address_configured() {
  local dev="$1"
<<<<<<< HEAD
  local cidr
  for cidr in "${host_cidrs[@]}"; do
    ip -4 addr show dev "$dev" 2>/dev/null | grep -Fq " $cidr" || return 1
  done
=======
  ip -4 addr show dev "$dev" 2>/dev/null | grep -Fq " $host_cidr"
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
}

route_configured() {
  local dev="$1"
<<<<<<< HEAD
  local item route src
  for item in "${radar_ips[@]}"; do
    route="$(ip route get "$item" 2>/dev/null || true)"
    src="$(host_src_for_radar "$item")"
    [[ "$route" == *" dev $dev "* && "$route" == *" src $src "* ]] || return 1
=======
  local item route
  for item in "${radar_ips[@]}"; do
    route="$(ip route get "$item" 2>/dev/null || true)"
    [[ "$route" == *" dev $dev "* && "$route" == *" src $host_ip "* ]] || return 1
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
  done
}

network_ready() {
  local dev="$1"
  address_configured "$dev" && route_configured "$dev"
}

configure_network() {
  local dev="$1"
<<<<<<< HEAD
=======
  local prefix="${host_cidr#*/}"
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
  if [[ "$dry_run" != true ]] && network_ready "$dev"; then
    log "Radar network already configured on $dev"
    return 0
  fi

<<<<<<< HEAD
  for cidr in "${host_cidrs[@]}"; do
    local cidr_ip="${cidr%/*}"
    local prefix="${cidr#*/}"
    if [[ "$prefix" == "24" ]] && ! same_24 "$cidr_ip" "$radar_gateway"; then
      :  # gateway docs are okay outside the host's /24; nothing to warn about per-cidr
    fi
  done

  log "Configuring radar access on $dev without changing default route"
  run_root ip link set dev "$dev" up
  for cidr in "${host_cidrs[@]}"; do
    if [[ "$dry_run" == true ]]; then
      run_root ip addr add "$cidr" dev "$dev"
    elif ! ip -4 addr show dev "$dev" | grep -Fq " $cidr"; then
      run_root ip addr add "$cidr" dev "$dev"
    fi
  done
  local item src
  for item in "${radar_ips[@]}"; do
    src="$(host_src_for_radar "$item")"
    run_root ip route replace "$item/32" dev "$dev" src "$src"
=======
  if [[ "$prefix" == "24" ]] && ! same_24 "$host_ip" "$radar_gateway"; then
    warn "gateway $radar_gateway is outside $host_cidr; leaving default gateway unchanged"
  fi

  log "Configuring radar access on $dev without changing default route"
  run_root ip link set dev "$dev" up
  if [[ "$dry_run" == true ]]; then
    run_root ip addr add "$host_cidr" dev "$dev"
  elif ! ip -4 addr show dev "$dev" | grep -Fq " $host_cidr"; then
    run_root ip addr add "$host_cidr" dev "$dev"
  fi
  local item
  for item in "${radar_ips[@]}"; do
    run_root ip route replace "$item/32" dev "$dev" src "$host_ip"
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
  done
}

install_systemd_service() {
  local dev="$1"
  local tmp
  tmp="$(mktemp)"
  cat > "$tmp" <<EOF
[Unit]
Description=SmartWheel XT-M60 radar network alias
After=network-online.target NetworkManager.service
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=$workspace_root/scripts/setup_radar_network.sh --iface $dev --radar-ip $radar_ip --host-cidr $host_cidr --gateway $radar_gateway --no-ping --quiet
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
  run_root install -m 0644 "$tmp" "$service_path"
  rm -f "$tmp"
  run_root systemctl daemon-reload
  run_root systemctl enable --now "$service_name"
  log "Installed and started $service_path"
}

uninstall_systemd_service() {
  run_root systemctl disable --now "$service_name" >/dev/null 2>&1 || true
  if [[ -e "$service_path" ]]; then
    run_root rm -f "$service_path"
  fi
  run_root systemctl daemon-reload
  log "Removed $service_path"
}

require_cmd ip
require_cmd awk
split_csv "$radar_ip"
<<<<<<< HEAD
split_cidr_csv "$host_cidr"
=======
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b

if [[ "$uninstall_service" == true ]]; then
  uninstall_systemd_service
  exit 0
fi

selected_iface="$(detect_iface)"
if [[ ! -e "/sys/class/net/$selected_iface" && "$dry_run" != true ]]; then
  fail "interface does not exist: $selected_iface"
fi

if [[ "$install_service" == true ]]; then
  install_systemd_service "$selected_iface"
fi

if [[ "$check_only" == true ]]; then
  show_status "$selected_iface"
  exit $?
fi

configure_network "$selected_iface"
show_status "$selected_iface"
