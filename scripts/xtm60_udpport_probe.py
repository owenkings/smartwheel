"""Direct-drive one XT-M60 with explicit ordering to test setUdpDestIp(port).

argv: ip [seconds] [dest_ip dest_port]
If dest_ip/dest_port given, calls setUdpDestIp(dest_ip, dest_port) after TCP
connect and before starting measurement.
"""
import sys
import time

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

root = find_xtsdk_root("/home/nvidia/smartwheel/xtsdk_py")
configure_xtsdk_import_path(root)
xsdk = _import_xintan_sdk(root)

frames = [0]
sdk = xsdk.XtSdk()
sdk.setCallback(lambda e: None, lambda f: frames.__setitem__(0, frames[0] + (1 if getattr(f, "hasPointcloud", False) else 0)))
sdk.setConnectIpaddress(ip)
sdk.startup()

for _ in range(100):
    if sdk.isconnect():
        break
    time.sleep(0.1)

if dest_ip and dest_port:
    ok = sdk.setUdpDestIp(dest_ip, dest_port)
    print(f"[{ip}] setUdpDestIp({dest_ip},{dest_port}) -> {ok}", flush=True)

sdk.start(xsdk.ImageType(4), False)
time.sleep(seconds)
print(f"RESULT ip={ip} dest_port={dest_port or 7687} connected={sdk.isconnect()} frames={frames[0]}", flush=True)
sys.stdout.flush()
import os
os._exit(0)
