#!/usr/bin/env python3
"""Bounded sequential/concurrent diagnostics for physical V4L2 camera nodes."""

import argparse
import hashlib
import json
import statistics
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


def summarize(values):
    if not values:
        return {"count": 0}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "mean": float(np.mean(array)),
        "max": float(np.max(array)),
        "stddev": float(np.std(array)),
    }


def capture_device(
    device,
    duration_sec,
    sample_dir,
    phase,
    width,
    height,
    fps,
    fourcc,
    start_barrier=None,
):
    path = str(device)
    numeric = int(path) if path.isdigit() else path
    result = {
        "device": path,
        "phase": phase,
        "opened": False,
        "frames": 0,
        "read_failures": 0,
        "sample_path": None,
        "sample_sha256": None,
        "error": "",
    }
    cap = cv2.VideoCapture(numeric, cv2.CAP_V4L2)
    if not cap.isOpened():
        result["error"] = "open failed"
        cap.release()
        return result

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    result["opened"] = True
    result["reported"] = {
        "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "fps": float(cap.get(cv2.CAP_PROP_FPS)),
        "fourcc_u32": int(cap.get(cv2.CAP_PROP_FOURCC)),
    }

    if start_barrier is not None:
        try:
            start_barrier.wait(timeout=10.0)
        except threading.BrokenBarrierError:
            result["error"] = "concurrent start barrier failed"
            cap.release()
            return result

    started = time.monotonic()
    times = []
    brightness = []
    spatial_stddev = []
    laplacian_variance = []
    frame_differences = []
    previous_small = None
    sample_frame = None
    shapes = set()

    while time.monotonic() - started < duration_sec:
        ok, frame = cap.read()
        if not ok or frame is None:
            result["read_failures"] += 1
            # A disconnected V4L2 node can return immediately instead of
            # blocking for the next frame. Avoid a tight loop that consumes a
            # CPU core and inflates the failure counter by millions.
            time.sleep(0.02)
            continue
        now = time.monotonic()
        result["frames"] += 1
        times.append(now)
        shapes.add(tuple(int(v) for v in frame.shape))
        # Keep the capture-rate measurement representative of the ROS adapter:
        # decode every frame, but run quality metrics only on every tenth frame
        # and at 80x60 so diagnostics do not become the bottleneck.
        if result["frames"] % 10 == 0:
            small_bgr = cv2.resize(frame, (80, 60), interpolation=cv2.INTER_AREA)
            small = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2GRAY)
            brightness.append(float(np.mean(small)))
            spatial_stddev.append(float(np.std(small)))
            laplacian_variance.append(float(cv2.Laplacian(small, cv2.CV_64F).var()))
            if previous_small is not None:
                frame_differences.append(
                    float(np.mean(cv2.absdiff(previous_small, small)))
                )
            previous_small = small
        if sample_frame is None and result["frames"] >= 5:
            sample_frame = frame.copy()

    cap.release()
    elapsed = time.monotonic() - started
    deltas = [b - a for a, b in zip(times, times[1:])]
    result.update(
        {
            "elapsed_sec": elapsed,
            "measured_fps": (
                1.0 / statistics.fmean(deltas) if deltas else None
            ),
            "frame_delta_sec": summarize(deltas),
            "brightness": summarize(brightness),
            "spatial_gray_stddev": summarize(spatial_stddev),
            "laplacian_variance": summarize(laplacian_variance),
            "frame_difference": summarize(frame_differences),
            "near_duplicate_fraction": (
                sum(value < 0.05 for value in frame_differences)
                / len(frame_differences)
                if frame_differences
                else None
            ),
            "shapes": [list(shape) for shape in sorted(shapes)],
        }
    )
    if sample_frame is not None:
        sample_dir.mkdir(parents=True, exist_ok=True)
        safe_name = path.replace("/", "_").replace("\\", "_")
        sample_path = sample_dir / f"{phase}_{safe_name}.jpg"
        cv2.imwrite(str(sample_path), sample_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        result["sample_path"] = str(sample_path)
        result["sample_sha256"] = hashlib.sha256(sample_path.read_bytes()).hexdigest()

    brightness_mean = result["brightness"].get("mean")
    spatial_mean = result["spatial_gray_stddev"].get("mean")
    result["checks"] = {
        "opened": result["opened"],
        "frames_present": result["frames"] > 0,
        "at_least_two_thirds_requested_fps": (
            result["measured_fps"] is not None
            and result["measured_fps"] >= max(1.0, fps * 2.0 / 3.0)
        ),
        "requested_resolution": [height, width, 3] in result["shapes"],
        "not_black_or_saturated": (
            brightness_mean is not None and 1.0 < brightness_mean < 254.0
        ),
        "spatial_content_present": spatial_mean is not None and spatial_mean > 2.0,
        "not_frozen": (
            result["near_duplicate_fraction"] is not None
            and result["near_duplicate_fraction"] < 0.95
        ),
        "sample_saved": result["sample_path"] is not None,
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--devices", nargs="+", default=["0", "2", "4"])
    parser.add_argument("--expected-count", type=int, default=4)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--sequential-duration-sec", type=float, default=6.0)
    parser.add_argument("--concurrent-duration-sec", type=float, default=10.0)
    parser.add_argument("--sample-dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    sample_dir = args.sample_dir or Path("bags/hardware") / (
        "camera_b4_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    )

    sequential = [
        capture_device(
            device,
            args.sequential_duration_sec,
            sample_dir,
            "sequential",
            args.width,
            args.height,
            args.fps,
            args.fourcc,
        )
        for device in args.devices
    ]

    barrier = threading.Barrier(len(args.devices))
    concurrent = [None] * len(args.devices)

    def worker(index, device):
        concurrent[index] = capture_device(
            device,
            args.concurrent_duration_sec,
            sample_dir,
            "concurrent",
            args.width,
            args.height,
            args.fps,
            args.fourcc,
            start_barrier=barrier,
        )

    threads = [
        threading.Thread(target=worker, args=(index, device), daemon=True)
        for index, device in enumerate(args.devices)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(args.concurrent_duration_sec + 15.0)

    concurrent = [
        item
        if item is not None
        else {
            "device": args.devices[index],
            "phase": "concurrent",
            "opened": False,
            "frames": 0,
            "error": "worker timeout",
            "checks": {"worker_completed": False},
        }
        for index, item in enumerate(concurrent)
    ]

    all_capture_checks = all(
        item.get("opened") is True
        and bool(item.get("checks"))
        and all(item["checks"].values())
        for item in sequential + concurrent
    )
    result = {
        "schema": "smartwheel.camera_array_diagnostic.v1",
        "read_only": True,
        "expected_physical_camera_count": args.expected_count,
        "tested_physical_capture_nodes": args.devices,
        "tested_count": len(args.devices),
        "sample_dir": str(sample_dir),
        "requested_mode": {
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
            "fourcc": args.fourcc,
        },
        "sequential": sequential,
        "concurrent": concurrent,
        "checks": {
            "expected_camera_count_present": len(args.devices) == args.expected_count,
            "all_tested_cameras_pass_sequential_and_concurrent": all_capture_checks,
        },
    }

    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if all(result["checks"].values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
