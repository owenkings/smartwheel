#!/usr/bin/env python3
"""Collect bounded XT-M60 SDK frame metadata without changing imaging settings."""

import argparse
import json
import statistics
import sys
import threading
import time
from pathlib import Path

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "src" / "wheelchair_sensors"))

from wheelchair_sensors.xtm60_adapter_node import (  # noqa: E402
    _import_xintan_sdk,
    _with_sdk_example_cwd,
    configure_xtsdk_import_path,
    find_xtsdk_root,
)


def _stats(values):
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "max": max(values),
        "stddev": statistics.pstdev(values),
    }


def _distribution_stats(values):
    if not values:
        return {"count": 0}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "p05": float(np.percentile(array, 5)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
        "stddev": float(np.std(array)),
    }


def collect(args):
    sdk_root = find_xtsdk_root(args.sdk_root)
    configure_xtsdk_import_path(sdk_root)
    xintan_sdk = _import_xintan_sdk(sdk_root)
    sdk = _with_sdk_example_cwd(sdk_root, xintan_sdk.XtSdk)
    connected = threading.Event()
    enough_frames = threading.Event()
    lock = threading.Lock()
    events = []
    frames = []
    point_distance_ratios = []
    raw_distance_mm = []
    point_range_m = []
    valid_amplitude = []
    valid_pixel_fractions = []
    center_distance_mm = []
    frame_delta_mm = []
    frame_changed_fractions = []
    previous_distance = None

    def on_event(event):
        cmd_id = int(getattr(event, "cmdid", -1))
        events.append({"event": str(getattr(event, "eventstr", "")), "cmd_id": cmd_id})
        try:
            if cmd_id == 0xFE and sdk.isconnect():
                connected.set()
        except Exception:
            pass

    def on_frame(frame):
        nonlocal previous_distance
        with lock:
            point_objects = getattr(frame, "points", [])
            distance_values = getattr(frame, "distData", [])
            amplitude_values = getattr(frame, "amplData", [])
            width = int(getattr(frame, "width", 0))
            height = int(getattr(frame, "height", 0))
            frames.append(
                {
                    "frame_id": int(getattr(frame, "frame_id", -1)),
                    "width": width,
                    "height": height,
                    "timestamp_sec": int(getattr(frame, "timeStampS", 0)),
                    "timestamp_nsec": int(getattr(frame, "timeStampNS", 0)),
                    "temperature_raw": int(getattr(frame, "temperature", 0)),
                    "vcsel_temperature_raw": int(getattr(frame, "vcseltemperature", 0)),
                    "point_count": len(point_objects),
                    "distance_count": len(distance_values),
                    "amplitude_count": len(amplitude_values),
                    "has_pointcloud": bool(getattr(frame, "hasPointcloud", False)),
                }
            )
            # The SDK exposes point coordinates and filtered device distance for
            # the same organized pixels. Compare them directly to verify the
            # point-unit conversion without relying on an unknown room target.
            if len(frames) <= 5 and len(point_objects) == len(distance_values):
                xyz = np.asarray(
                    [(point.x, point.y, point.z) for point in point_objects],
                    dtype=np.float64,
                )
                distance = np.asarray(distance_values, dtype=np.float64)
                norms = np.linalg.norm(xyz, axis=1)
                valid = np.isfinite(norms) & (norms > 0.0) & (distance > 0.0)
                point_distance_ratios.extend((norms[valid] / distance[valid]).tolist())
            if (
                len(frames) <= args.analysis_frame_count
                and len(point_objects) == len(distance_values)
                and len(distance_values) > 0
            ):
                xyz = np.asarray(
                    [(point.x, point.y, point.z) for point in point_objects],
                    dtype=np.float64,
                )
                distance = np.asarray(distance_values, dtype=np.float64)
                amplitude = np.asarray(amplitude_values, dtype=np.float64)
                norms = np.linalg.norm(xyz, axis=1)
                valid = (
                    np.isfinite(norms)
                    & (norms > 0.0)
                    & np.isfinite(distance)
                    & (distance > 0.0)
                    & (distance < 64000.0)
                )
                valid_pixel_fractions.append(float(np.count_nonzero(valid) / distance.size))
                raw_distance_mm.extend(distance[valid].tolist())
                point_range_m.extend(norms[valid].tolist())
                if amplitude.size == distance.size:
                    amplitude_valid = valid & np.isfinite(amplitude) & (amplitude < 64000.0)
                    valid_amplitude.extend(amplitude[amplitude_valid].tolist())
                if width > 0 and height > 0 and width * height == distance.size:
                    center_distance_mm.append(float(distance[(height // 2) * width + width // 2]))
                if previous_distance is not None and previous_distance.size == distance.size:
                    pair_valid = (
                        np.isfinite(previous_distance)
                        & (previous_distance > 0.0)
                        & (previous_distance < 64000.0)
                        & np.isfinite(distance)
                        & (distance > 0.0)
                        & (distance < 64000.0)
                    )
                    if np.any(pair_valid):
                        delta = np.abs(distance[pair_valid] - previous_distance[pair_valid])
                        frame_delta_mm.extend(delta.tolist())
                        frame_changed_fractions.append(
                            float(np.count_nonzero(delta >= 10.0) / delta.size)
                        )
                previous_distance = distance.copy()
            if len(frames) >= args.frame_count:
                enough_frames.set()

    started = time.monotonic()
    result = {
        "schema": "smartwheel.xtm60_sdk_frame_diagnostic.v1",
        "ip_address": args.ip_address,
        "udp_destination": f"{args.udp_dest_ip}:{args.udp_dest_port}",
        "image_type": args.image_type,
        "device_configuration_written": False,
        "runtime_udp_destination_set": True,
        "imaging_parameters_source": "device_saved_configuration",
    }
    try:
        sdk.setCallback(on_event, on_frame)
        result["set_connect_ip_return"] = bool(sdk.setConnectIpaddress(args.ip_address))
        sdk.startup()
        if not connected.wait(args.connect_timeout_sec):
            result["error"] = "SDK connection timeout"
            return result
        result["connected"] = True
        result["set_udp_destination_return"] = bool(
            sdk.setUdpDestIp(args.udp_dest_ip, args.udp_dest_port)
        )
        image_type = xintan_sdk.ImageType(args.image_type)
        try:
            result["start_return"] = bool(sdk.start(image_type, False))
        except TypeError:
            result["start_return"] = bool(sdk.start(image_type))
        enough_frames.wait(args.frame_timeout_sec)
    finally:
        try:
            sdk.stop()
        except Exception as exc:
            result["stop_error"] = str(exc)
        try:
            sdk.shutdown()
        except Exception as exc:
            result["shutdown_error"] = str(exc)

    with lock:
        captured = frames[: args.frame_count]
    sdk_timestamps = [
        frame["timestamp_sec"] * 1_000_000_000 + frame["timestamp_nsec"]
        for frame in captured
    ]
    sdk_deltas = [
        (current - previous) / 1_000_000_000.0
        for previous, current in zip(sdk_timestamps, sdk_timestamps[1:])
    ]
    result.update(
        {
            "elapsed_sec": time.monotonic() - started,
            "events": events,
            "frame_count": len(captured),
            "widths": sorted({frame["width"] for frame in captured}),
            "heights": sorted({frame["height"] for frame in captured}),
            "point_counts": sorted({frame["point_count"] for frame in captured}),
            "distance_counts": sorted({frame["distance_count"] for frame in captured}),
            "amplitude_counts": sorted({frame["amplitude_count"] for frame in captured}),
            "temperature_c": _stats(
                [frame["temperature_raw"] / 100.0 for frame in captured]
            ),
            "vcsel_temperature_c": _stats(
                [frame["vcsel_temperature_raw"] / 100.0 for frame in captured]
            ),
            "point_norm_m_per_distance_unit": _stats(point_distance_ratios),
            "sdk_timestamp": {
                "first_sec": captured[0]["timestamp_sec"] if captured else None,
                "last_sec": captured[-1]["timestamp_sec"] if captured else None,
                "nonpositive_deltas": sum(delta <= 0 for delta in sdk_deltas),
                "delta_sec": _stats(sdk_deltas),
            },
        }
    )
    result["checks"] = {
        "requested_frames_received": len(captured) == args.frame_count,
        "organized_160x60": result["widths"] == [160] and result["heights"] == [60],
        "full_frame_arrays": (
            result["point_counts"] == [9600]
            and result["distance_counts"] == [9600]
            and result["amplitude_counts"] == [9600]
        ),
        "sdk_timestamps_strictly_increasing": (
            len(sdk_deltas) > 0 and result["sdk_timestamp"]["nonpositive_deltas"] == 0
        ),
        "point_coordinates_match_millimeter_distance_scale": (
            point_distance_ratios
            and 0.00099
            <= statistics.median(point_distance_ratios)
            <= 0.00101
        ),
    }
    result["raw_distance_mm"] = _distribution_stats(raw_distance_mm)
    result["point_range_m"] = _distribution_stats(point_range_m)
    result["amplitude"] = _distribution_stats(valid_amplitude)
    result["valid_pixel_fraction"] = _stats(valid_pixel_fractions)
    result["center_distance_mm"] = _distribution_stats(center_distance_mm)
    result["consecutive_frame_absolute_delta_mm"] = _distribution_stats(frame_delta_mm)
    result["consecutive_frame_changed_fraction_ge_10mm"] = _stats(
        frame_changed_fractions
    )
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip-address", default="192.168.0.101")
    parser.add_argument("--udp-dest-ip", default="192.168.0.100")
    parser.add_argument("--udp-dest-port", type=int, default=7687)
    parser.add_argument("--image-type", type=int, default=4)
    parser.add_argument("--sdk-root", default="/home/nvidia/smartwheel/xtsdk_py")
    parser.add_argument("--connect-timeout-sec", type=float, default=15.0)
    parser.add_argument("--frame-timeout-sec", type=float, default=20.0)
    parser.add_argument("--frame-count", type=int, default=100)
    parser.add_argument(
        "--analysis-frame-count",
        type=int,
        default=10,
        help="Number of initial frames used for distance/amplitude distributions.",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = collect(args)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result.get("checks", {}).get("requested_frames_received") else 2


if __name__ == "__main__":
    raise SystemExit(main())
