"""Connect one XT-M60 by IP, count frames for a few seconds, print result.

Run two copies in parallel to verify simultaneous dual-radar reception.
"""
import logging
import sys
import time

sys.path.insert(0, "/home/nvidia/smartwheel/src/wheelchair_sensors")
from wheelchair_sensors.xtm60_adapter_node import XTM60SdkAdapter, XTM60SdkConfig

ip = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
log = logging.getLogger(ip)
logging.basicConfig(level=logging.INFO, format=f"[{ip}] %(message)s")

cfg = XTM60SdkConfig(sdk_root="/home/nvidia/smartwheel/xtsdk_py", ip_address=ip)
ad = XTM60SdkAdapter(cfg, log)
ad.start()

frames = 0
t0 = time.monotonic()
while time.monotonic() - t0 < seconds:
    ad.poll()
    pts, _ = ad.take_latest_points()
    if pts:
        frames += 1
    time.sleep(0.02)

elapsed = time.monotonic() - t0
print(f"RESULT ip={ip} connected={ad.connected} frames={frames} fps={frames/elapsed:.1f}", flush=True)
sys.stdout.flush()
ad.stop_and_exit_process(0)
