#!/usr/bin/env python3
"""Identify and label USB cameras using GStreamer + MJPG.

This is a rewrite of identify_cameras.py that uses gst-launch-1.0 to capture
JPEG frames instead of OpenCV. GStreamer with explicit ``image/jpeg`` caps:

  - Forces the camera to negotiate MJPG instead of YUYV. MJPG is roughly 5x
    smaller, so multiple cameras on the same USB 2.0 hub do not exceed the
    isochronous bandwidth budget.
  - Runs each capture in an isolated subprocess. If a device misbehaves and
    the pipeline hangs, ``timeout`` kills it cleanly and the parent script
    keeps going.
  - Adds a settle delay between captures so the USB controller has time to
    reclaim previously allocated bandwidth before the next device opens.
  - Gracefully skips devices that do not respond and reports them so you
    can swap cables or rebind drivers.

If you see all four devices fail in a row, the V4L2 stack is probably wedged
from earlier failed STREAMON attempts. Easiest fix is ``sudo reboot``.

Usage:
  cd ~/smartwheel
  python3 scripts/identify_cameras_v2.py --auto-stop-service
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path


POSITION_LABELS = {
    "1": "front_left",
    "2": "front_right",
    "3": "side_left",
    "4": "side_right",
    "5": "rear",
    "0": "skip",
}

POSITION_PROMPT = """位置选项:
  [1] front_left   左前
  [2] front_right  右前
  [3] side_left    左侧方
  [4] side_right   右侧方
  [5] rear         后方
  [0] skip         不用这个相机
"""


def usb_position(video_index: int) -> str | None:
    sysfs = f"/sys/class/video4linux/video{video_index}/device"
    if not os.path.islink(sysfs):
        return None
    real = os.path.realpath(sysfs)
    matches = re.findall(r"/([0-9]+-[0-9]+(?:\.[0-9]+)*)/", real)
    return matches[-1] if matches else None


def is_video_capture(video_index: int) -> bool:
    """Use VIDIOC_QUERYCAP to confirm the node is a capture device."""
    import ctypes
    import fcntl

    class V4L2Capability(ctypes.Structure):
        _fields_ = [
            ("driver", ctypes.c_char * 16),
            ("card", ctypes.c_char * 32),
            ("bus_info", ctypes.c_char * 32),
            ("version", ctypes.c_uint32),
            ("capabilities", ctypes.c_uint32),
            ("device_caps", ctypes.c_uint32),
            ("reserved", ctypes.c_uint32 * 3),
        ]

    VIDIOC_QUERYCAP = 0x80685600
    V4L2_CAP_VIDEO_CAPTURE = 0x00000001
    path = f"/dev/video{video_index}"
    try:
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        cap = V4L2Capability()
        fcntl.ioctl(fd, VIDIOC_QUERYCAP, cap)
        return bool(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)
    except Exception:
        return False
    finally:
        os.close(fd)


def find_capture_devices() -> list[dict]:
    out: list[dict] = []
    for i in range(40):
        path = f"/dev/video{i}"
        if not os.path.exists(path):
            continue
        if not is_video_capture(i):
            continue
        out.append({"index": i, "path": path, "usb_pos": usb_position(i)})
    return out


def kill_orphan_holders(video_path: str) -> int:
    """Best-effort kill of stale processes holding /dev/videoN open."""
    try:
        result = subprocess.run(
            ["lsof", video_path],
            capture_output=True,
            text=True,
            timeout=3,
        )
    except Exception:
        return 0
    pids: set[int] = set()
    for line in result.stdout.splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 2:
            try:
                pids.add(int(parts[1]))
            except ValueError:
                pass
    if not pids:
        return 0
    self_pid = os.getpid()
    killed = 0
    for pid in pids:
        if pid == self_pid:
            continue
        try:
            os.kill(pid, 15)
            killed += 1
        except OSError:
            pass
    return killed


def capture_jpeg_via_gst(video_index: int, out_path: Path, timeout: float, fps: int = 15) -> tuple[bool, str]:
    """Use gst-launch to grab one JPEG frame via the MJPG pipeline.

    Returns (success, stderr_text). The temporary JPEG is converted to the
    output PNG via the same pipeline.
    """
    if shutil.which("gst-launch-1.0") is None:
        return False, "gst-launch-1.0 not installed"
    pipeline = (
        f"v4l2src device=/dev/video{video_index} num-buffers=3 "
        f"! image/jpeg,framerate={fps}/1 "
        "! jpegdec ! videoconvert ! pngenc "
        f"! filesink location={out_path}"
    )
    try:
        result = subprocess.run(
            ["gst-launch-1.0", "-e"] + pipeline.split(),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as exc:
        return False, f"timeout after {timeout}s: {exc.stderr or ''}"
    ok = result.returncode == 0 and out_path.exists() and out_path.stat().st_size > 0
    return ok, (result.stderr or result.stdout)[-400:]


def has_display() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def open_in_viewer(path: Path) -> bool:
    if not has_display():
        return False
    for cmd in ("eog", "xdg-open", "gio open"):
        first = cmd.split()[0]
        if shutil.which(first) is None:
            continue
        try:
            subprocess.Popen(
                cmd.split() + [str(path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            return True
        except Exception:
            continue
    return False


def ask(prompt: str, valid: set[str]) -> str:
    while True:
        try:
            ans = input(prompt).strip()
        except EOFError:
            print()
            return "0"
        if ans in valid:
            return ans
        print(f"  invalid input; expected one of: {sorted(valid)}")


def stop_service_if_needed(auto_stop: bool) -> bool:
    try:
        active = subprocess.run(
            ["systemctl", "--user", "is-active", "smartwheel.service"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except Exception:
        return True
    if active.stdout.strip() != "active":
        return True
    if auto_stop:
        print("Stopping smartwheel.service to release /dev/video0 ...")
        subprocess.run(["systemctl", "--user", "stop", "smartwheel.service"], timeout=15)
        time.sleep(3)
        return True
    print("WARN: smartwheel.service is active; /dev/video0 may be busy.")
    print("Either run:    systemctl --user stop smartwheel.service")
    print("Or rerun this script with --auto-stop-service")
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshots-dir", default="docs/camera_snapshots")
    parser.add_argument("--output", default="docs/camera_mapping.yaml")
    parser.add_argument("--timeout", type=float, default=8.0,
                        help="Per-device gst-launch timeout in seconds (default 8).")
    parser.add_argument("--settle-sec", type=float, default=1.5,
                        help="Sleep between captures so USB controller can reclaim bandwidth.")
    parser.add_argument("--auto-stop-service", action="store_true")
    parser.add_argument("--restart-service", action="store_true")
    parser.add_argument("--kill-orphans", action="store_true",
                        help="Kill stale processes holding /dev/videoN before opening.")
    args = parser.parse_args()

    workspace = Path(__file__).resolve().parent.parent
    snapshots_dir = (workspace / args.snapshots_dir).resolve()
    output_path = (workspace / args.output).resolve()
    snapshots_dir.mkdir(parents=True, exist_ok=True)

    if not stop_service_if_needed(args.auto_stop_service):
        return 1

    devices = find_capture_devices()
    if not devices:
        print("No capture devices found under /dev/videoN")
        return 1

    print(f"\nFound {len(devices)} capture devices (V4L2 VIDEO_CAPTURE bit set):")
    for d in devices:
        print(f"  /dev/video{d['index']:<2}  USB={d['usb_pos']}")

    print(
        f"\n--- Capturing one MJPG frame from each device "
        f"(per-device timeout {args.timeout}s, settle {args.settle_sec}s) ---"
    )

    failures: list[tuple[int, str]] = []
    for d in devices:
        if args.kill_orphans:
            n = kill_orphan_holders(d["path"])
            if n > 0:
                print(f"  killed {n} orphan(s) holding {d['path']}")
                time.sleep(0.5)
        out = snapshots_dir / (
            f"cam_video{d['index']}_usb{(d['usb_pos'] or 'unknown').replace('.', '_')}.png"
        )
        if out.exists():
            try:
                out.unlink()
            except OSError:
                pass
        ok, info = capture_jpeg_via_gst(d["index"], out, args.timeout)
        d["captured"] = ok
        d["snapshot"] = str(out) if ok else None
        if ok:
            print(f"  /dev/video{d['index']:<2}  OK  -> {out}")
        else:
            failures.append((d["index"], info.strip()))
            print(f"  /dev/video{d['index']:<2}  FAILED")
        time.sleep(args.settle_sec)

    if failures:
        print("\nFAILED captures (may need reboot or driver reload):")
        for idx, info in failures:
            short = info.replace("\n", " | ")[:200]
            print(f"  video{idx}: {short}")
        if len(failures) == len(devices):
            print(
                "\nAll cameras failed. The V4L2 subsystem is probably wedged.\n"
                "Recovery options (most reliable first):\n"
                "  1. sudo reboot\n"
                "  2. sudo modprobe -r uvcvideo && sudo modprobe uvcvideo\n"
                "  3. Physically unplug all cameras for 5 seconds, then replug\n"
                "After recovery, rerun this script."
            )
            return 2

    print("\n--- Labeling phase ---")
    print(POSITION_PROMPT)

    mapping: list[dict] = []
    valid_pos = set(POSITION_LABELS.keys())
    valid_rot = {"0", "90", "180", "270"}
    for d in devices:
        if not d["captured"]:
            print(f"\n=== /dev/video{d['index']} (USB {d['usb_pos']}) — capture FAILED, auto-skip ===")
            mapping.append(
                {
                    "video_index": d["index"],
                    "usb_pos": d["usb_pos"],
                    "snapshot": None,
                    "physical": "skip",
                    "rotation": 0,
                    "captured": False,
                }
            )
            continue
        print(f"\n=== /dev/video{d['index']}  USB={d['usb_pos']} ===")
        print(f"  snapshot: {d['snapshot']}")
        if not open_in_viewer(Path(d["snapshot"])):
            print("  (no GUI viewer launched; please open the PNG in another terminal/file-manager)")
        pos_ans = ask("  Position [0-5]: ", valid_pos)
        rot_ans = ask("  Rotation needed [0/90/180/270]: ", valid_rot)
        mapping.append(
            {
                "video_index": d["index"],
                "usb_pos": d["usb_pos"],
                "snapshot": d["snapshot"],
                "physical": POSITION_LABELS[pos_ans],
                "rotation": int(rot_ans),
                "captured": True,
            }
        )

    try:
        import yaml  # noqa: WPS433
    except ImportError:
        print("ERROR: pyyaml is required (pip3 install pyyaml)", file=sys.stderr)
        return 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        yaml.safe_dump({"cameras": mapping}, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    print(f"\n=> mapping saved to {output_path}")
    print("=> snapshots saved under", snapshots_dir)

    print("\nNext step:")
    print(f"  cat {output_path}")

    if args.restart_service:
        print("\nRestarting smartwheel.service ...")
        subprocess.run(["systemctl", "--user", "start", "smartwheel.service"], timeout=15)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
