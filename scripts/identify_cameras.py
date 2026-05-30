#!/usr/bin/env python3
"""Identify and label USB cameras by visual inspection.

For each plugged-in capture device, this script:
  1. Captures one frame using an isolated subprocess (with timeout) so any
     hanging device does not block the whole tool
  2. Saves the snapshot to docs/camera_snapshots/cam_videoN_usbX-X.X.png
  3. Tries to open the image with eog / xdg-open (falls back to printing path)
  4. Asks you to label the physical position and required rotation
  5. Writes the result to docs/camera_mapping.yaml

The output YAML can be handed to me (or pasted in chat) to update
src/wheelchair_bringup/config/camera.yaml.

Prerequisites:
  - smartwheel.service must be stopped (otherwise /dev/video0 may be busy).
    Either stop it manually or pass --auto-stop-service.
  - Display required only for image preview; snapshots are also saved as PNG
    so you can view them with any file manager if pop-up viewers fail.

Usage:
  cd ~/smartwheel
  python3 scripts/identify_cameras.py --auto-stop-service
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
    """Return USB physical path like '1-1.3' for /dev/videoN, via sysfs."""
    sysfs = f"/sys/class/video4linux/video{video_index}/device"
    if not os.path.islink(sysfs):
        return None
    real = os.path.realpath(sysfs)
    matches = re.findall(r"/([0-9]+-[0-9]+(?:\.[0-9]+)*)/", real)
    return matches[-1] if matches else None


def find_capture_devices() -> list[dict]:
    """List even-indexed /dev/videoN entries; UVC capture is on even indices."""
    out: list[dict] = []
    for i in range(40):
        path = f"/dev/video{i}"
        if not os.path.exists(path):
            continue
        if i % 2 != 0:
            continue
        out.append({"index": i, "path": path, "usb_pos": usb_position(i)})
    return out


def capture_frame_isolated(video_index: int, out_path: Path, timeout: float) -> bool:
    """Capture a single frame in a child Python process.

    Subprocess isolation is required because cv2.VideoCapture.read() has no
    built-in timeout and can block indefinitely on misbehaving devices. We
    enforce timeout via subprocess.run timeout instead, killing the child if
    it gets stuck.
    """
    code = f"""
import cv2
import sys
import time

cap = cv2.VideoCapture({video_index}, cv2.CAP_V4L2)
if not cap.isOpened():
    sys.exit(10)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
deadline = time.time() + max(0.2, {timeout - 1.5})
ok, frame = False, None
while time.time() < deadline:
    ok, frame = cap.read()
    if ok and frame is not None and frame.size > 0:
        break
    time.sleep(0.05)
cap.release()
if ok and frame is not None and frame.size > 0:
    cv2.imwrite({str(out_path)!r}, frame)
    sys.exit(0)
sys.exit(20)
"""
    try:
        result = subprocess.run(
            [sys.executable, "-c", code],
            timeout=timeout,
            capture_output=True,
            text=True,
        )
        return result.returncode == 0 and out_path.exists()
    except subprocess.TimeoutExpired:
        return False


def has_display() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def open_in_viewer(path: Path) -> bool:
    """Best-effort image preview without blocking."""
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
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Per-device frame capture timeout in seconds (default 5).",
    )
    parser.add_argument("--auto-stop-service", action="store_true")
    parser.add_argument(
        "--restart-service",
        action="store_true",
        help="Restart smartwheel.service after labeling completes.",
    )
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

    print(f"\nFound {len(devices)} capture devices:")
    for d in devices:
        print(f"  /dev/video{d['index']:<2}  USB={d['usb_pos']}")

    print(f"\n--- Capturing one frame from each device (per-device timeout {args.timeout}s) ---")
    for d in devices:
        out = snapshots_dir / f"cam_video{d['index']}_usb{(d['usb_pos'] or 'unknown').replace('.', '_')}.png"
        if out.exists():
            try:
                out.unlink()
            except OSError:
                pass
        ok = capture_frame_isolated(d["index"], out, args.timeout)
        d["captured"] = ok
        d["snapshot"] = str(out) if ok else None
        flag = "OK" if ok else "FAILED"
        print(f"  /dev/video{d['index']:<2}  {flag}{('  -> ' + str(out)) if ok else ''}")

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
    print("  把这个 YAML 内容贴给我，或告诉我每行的 physical/rotation，")
    print("  我会用它去更新 src/wheelchair_bringup/config/camera.yaml，")
    print("  并在 camera_adapter_node.py 加入 rotation 参数（如果有非 0 旋转）。")

    if args.restart_service:
        print("\nRestarting smartwheel.service ...")
        subprocess.run(["systemctl", "--user", "start", "smartwheel.service"], timeout=15)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
