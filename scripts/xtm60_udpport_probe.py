"""Direct-drive one XT-M60 with explicit ordering to test setUdpDestIp(port).

argv: ip [seconds] [dest_ip dest_port] [--reset]
If dest_ip/dest_port given, calls setUdpDestIp(dest_ip, dest_port) after TCP
connect and before starting measurement.
If --reset is given, reboots the device after reading its information and exits.
"""
import sys
import time
import os

sys.path.insert(0, "/home/nvidia/smartwheel/src/wheelchair_sensors")
from wheelchair_sensors.xtm60_adapter_node import (
    find_xtsdk_root,
    configure_xtsdk_import_path,
    _import_xintan_sdk,
)

ip = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
dest_ip = sys.argv[3] if len(sys.argv) > 4 else ""
dest_port = int(sys.argv[4]) if len(sys.argv) > 4 else 0
reset_device = "--reset" in sys.argv[5:]

root = find_xtsdk_root("/home/nvidia/smartwheel/xtsdk_py")
configure_xtsdk_import_path(root)
xsdk = _import_xintan_sdk(root)

frames = [0]
events = []
sdk = xsdk.XtSdk()


def finish(code):
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


def on_event(event):
    event_text = str(getattr(event, "eventstr", ""))
    cmd_id = getattr(event, "cmdid", None)
    events.append((event_text, cmd_id))
    print(f"[{ip}] event={event_text!r} cmdid={cmd_id}", flush=True)


sdk.setCallback(
    on_event,
    lambda frame: frames.__setitem__(
        0, frames[0] + (1 if getattr(frame, "hasPointcloud", False) else 0)
    ),
)
print(f"[{ip}] configuring TCP control connection", flush=True)
print(f"[{ip}] setConnectIpaddress -> {sdk.setConnectIpaddress(ip)}", flush=True)
print(f"[{ip}] startup begin", flush=True)
sdk.startup()
print(f"[{ip}] startup returned", flush=True)

for _ in range(100):
    if sdk.isconnect():
        break
    time.sleep(0.1)

connected = bool(sdk.isconnect())
print(f"[{ip}] connected={connected} state={sdk.getStateStr()!r}", flush=True)
if not connected:
    print(
        f"RESULT ip={ip} connected=false frames=0 error=tcp_handshake_timeout ",
        f"events={events!r}",
        flush=True,
    )
    finish(2)

info_ok, info = sdk.getDevInfo()
if info_ok:
    print(
        f"[{ip}] device fw={getattr(info, 'fwVersion', '')!r} "
        f"sn={getattr(info, 'sn', '')!r} chip={getattr(info, 'chipidStr', '')!r}",
        flush=True,
    )
else:
    print(f"[{ip}] getDevInfo -> false", flush=True)

config_ok, config = sdk.getDevConfig()
if config_ok:
    fields = (
        "version",
        "modFreq",
        "hdrMode",
        "integrationTimeGs",
        "integrationTimes",
        "miniAmp",
        "isFilterOn",
        "roi",
        "maxfps",
        "freqChannel",
        "setmaxfps",
        "endianType",
    )
    values = " ".join(
        f"{field}={getattr(config, field, None)!r}" for field in fields
    )
    print(f"[{ip}] config {values}", flush=True)
else:
    print(f"[{ip}] getDevConfig -> false", flush=True)

if reset_device:
    print(f"[{ip}] resetDev begin", flush=True)
    reset_ok = sdk.resetDev()
    print(f"RESULT ip={ip} reset={reset_ok}", flush=True)
    finish(0 if reset_ok else 4)

if dest_ip and dest_port:
    print(f"[{ip}] setUdpDestIp begin", flush=True)
    ok = sdk.setUdpDestIp(dest_ip, dest_port)
    print(f"[{ip}] setUdpDestIp({dest_ip},{dest_port}) -> {ok}", flush=True)

print(f"[{ip}] measurement start begin", flush=True)
start_ok = sdk.start(xsdk.ImageType(4), False)
print(f"[{ip}] measurement start -> {start_ok}", flush=True)
time.sleep(seconds)
print(
    f"RESULT ip={ip} dest_port={dest_port or 7687} "
    f"connected={sdk.isconnect()} frames={frames[0]}",
    flush=True,
)
finish(0 if frames[0] > 0 else 3)
