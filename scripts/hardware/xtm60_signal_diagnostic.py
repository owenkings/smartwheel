#!/usr/bin/env python3
"""Collect bounded XT-M60 signal/invalid-pixel evidence without config writes."""

import argparse
import json
import statistics
import sys
import threading
import time
from collections import Counter
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
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0:
        return {"count": 0}
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


def _top_counts(values, limit=12):
    return [
        {"value": int(value), "count": int(count)}
        for value, count in Counter(int(item) for item in values).most_common(limit)
    ]


def _region_grid(pixel_sum, frame_count, width, height, block=10):
    if frame_count <= 0 or pixel_sum.size != width * height:
        return []
    image = (pixel_sum / frame_count).reshape(height, width)
    rows = []
    for y0 in range(0, height, block):
        row = []
        for x0 in range(0, width, block):
            region = image[y0 : min(y0 + block, height), x0 : min(x0 + block, width)]
            row.append(round(float(np.mean(region)), 4))
        rows.append(row)
    return rows


def _write_pgm(path, pixel_sum, frame_count, width, height):
    if frame_count <= 0 or pixel_sum.size != width * height:
        return False
    image = np.clip(np.rint(pixel_sum / frame_count * 255.0), 0, 255).astype(np.uint8)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + image.tobytes())
    return True


def collect(args):
    sdk_root = find_xtsdk_root(args.sdk_root)
    configure_xtsdk_import_path(sdk_root)
    xintan_sdk = _import_xintan_sdk(sdk_root)
    sdk = _with_sdk_example_cwd(sdk_root, xintan_sdk.XtSdk)
    connected = threading.Event()
    enough_frames = threading.Event()
    lock = threading.Lock()
    events = []
    frame_metadata = []
    public_frame_fields = []
    dist_invalid_values = []
    rawdist_invalid_values = []
    amplitude_invalid_values = []
    amplitude_all_valid_encoding = []
    amplitude_for_point_valid = []
    amplitude_for_point_invalid = []
    grayscale_values = []
    level_values = []
    reflectivity_values = []
    point_ranges = []
    dist_ranges = []
    rawdist_ranges = []
    consecutive_distance_delta_mm = []
    consecutive_changed_fraction_ge_50mm = []
    point_valid_fractions = []
    mapping_valid_fractions = []
    dist_valid_fractions = []
    rawdist_valid_fractions = []
    temperatures_c = []
    vcsel_temperatures_c = []
    width = 0
    height = 0
    point_valid_sum = np.asarray([], dtype=np.float64)
    mapping_valid_sum = np.asarray([], dtype=np.float64)
    amplitude_valid_sum = np.asarray([], dtype=np.float64)
    previous_distance = None
    previous_point_valid = None

    def on_event(event):
        cmd_id = int(getattr(event, "cmdid", -1))
        events.append({"event": str(getattr(event, "eventstr", "")), "cmd_id": cmd_id})
        try:
            if cmd_id == 0xFE and sdk.isconnect():
                connected.set()
        except Exception:
            pass

    def on_frame(frame):
        nonlocal width, height, point_valid_sum, mapping_valid_sum
        nonlocal amplitude_valid_sum, previous_distance, previous_point_valid
        with lock:
            if len(frame_metadata) >= args.frame_count:
                return
            width = int(getattr(frame, "width", 0))
            height = int(getattr(frame, "height", 0))
            points = list(getattr(frame, "points", []))
            distance = np.asarray(getattr(frame, "distData", []), dtype=np.float64)
            raw_distance = np.asarray(getattr(frame, "rawdistData", []), dtype=np.float64)
            amplitude = np.asarray(getattr(frame, "amplData", []), dtype=np.float64)
            grayscale = np.asarray(getattr(frame, "grayscaledata", []), dtype=np.float64)
            level = np.asarray(getattr(frame, "leveldata", []), dtype=np.float64)
            reflectivity = np.asarray(getattr(frame, "reflectivity", []), dtype=np.float64)
            xyz = np.asarray(
                [(point.x, point.y, point.z) for point in points], dtype=np.float64
            )
            norms = np.linalg.norm(xyz, axis=1) if xyz.size else np.asarray([])
            count = width * height

            if not public_frame_fields:
                for name in dir(frame):
                    if name.startswith("_"):
                        continue
                    try:
                        value = getattr(frame, name)
                    except Exception:
                        continue
                    if callable(value):
                        continue
                    try:
                        value_length = len(value)
                    except (TypeError, AttributeError):
                        value_length = None
                    public_frame_fields.append(
                        {
                            "name": name,
                            "type": type(value).__name__,
                            "length": value_length,
                        }
                    )

            if (
                count <= 0
                or len(points) != count
                or distance.size != count
                or amplitude.size != count
            ):
                frame_metadata.append(
                    {
                        "width": width,
                        "height": height,
                        "point_count": len(points),
                        "distance_count": int(distance.size),
                        "raw_distance_count": int(raw_distance.size),
                        "amplitude_count": int(amplitude.size),
                        "array_shape_ok": False,
                    }
                )
                if len(frame_metadata) >= args.frame_count:
                    enough_frames.set()
                return

            amplitude_encoding_valid = (
                np.isfinite(amplitude) & (amplitude >= 0.0) & (amplitude < 64000.0)
            )
            dist_vendor_valid = (
                np.isfinite(distance) & (distance >= 0.0) & (distance <= 964000.0)
            )
            rawdist_vendor_valid = (
                raw_distance.size == count
            ) and (
                np.isfinite(raw_distance)
                & (raw_distance >= 0.0)
                & (raw_distance <= 964000.0)
            )
            point_valid = (
                np.isfinite(norms)
                & (norms > 0.0)
                & amplitude_encoding_valid
            )
            mapping_valid = point_valid & (norms >= args.range_min_m) & (
                norms <= args.range_max_m
            )

            point_valid_fractions.append(float(np.count_nonzero(point_valid) / count))
            mapping_valid_fractions.append(float(np.count_nonzero(mapping_valid) / count))
            dist_valid_fractions.append(float(np.count_nonzero(dist_vendor_valid) / count))
            if isinstance(rawdist_vendor_valid, np.ndarray):
                rawdist_valid_fractions.append(
                    float(np.count_nonzero(rawdist_vendor_valid) / count)
                )
            temperatures_c.append(float(getattr(frame, "temperature", 0)) / 100.0)
            vcsel_temperatures_c.append(
                float(getattr(frame, "vcseltemperature", 0)) / 100.0
            )

            if point_valid_sum.size != count:
                point_valid_sum = np.zeros(count, dtype=np.float64)
                mapping_valid_sum = np.zeros(count, dtype=np.float64)
                amplitude_valid_sum = np.zeros(count, dtype=np.float64)
            point_valid_sum += point_valid.astype(np.float64)
            mapping_valid_sum += mapping_valid.astype(np.float64)
            amplitude_valid_sum += amplitude_encoding_valid.astype(np.float64)

            point_ranges.extend(norms[point_valid].tolist())
            dist_ranges.extend(distance[dist_vendor_valid].tolist())
            if isinstance(rawdist_vendor_valid, np.ndarray):
                rawdist_ranges.extend(raw_distance[rawdist_vendor_valid].tolist())
                rawdist_invalid_values.extend(raw_distance[~rawdist_vendor_valid].tolist())
            amplitude_all_valid_encoding.extend(amplitude[amplitude_encoding_valid].tolist())
            amplitude_for_point_valid.extend(amplitude[point_valid].tolist())
            amplitude_for_point_invalid.extend(
                amplitude[(~point_valid) & amplitude_encoding_valid].tolist()
            )
            if grayscale.size == count:
                grayscale_values.extend(grayscale[np.isfinite(grayscale)].tolist())
            if level.size == count:
                level_values.extend(level[np.isfinite(level)].tolist())
            if reflectivity.size == count:
                reflectivity_values.extend(
                    reflectivity[np.isfinite(reflectivity)].tolist()
                )
            dist_invalid_values.extend(distance[~dist_vendor_valid].tolist())
            amplitude_invalid_values.extend(amplitude[~amplitude_encoding_valid].tolist())

            if (
                previous_distance is not None
                and previous_distance.size == count
                and previous_point_valid is not None
            ):
                pair_valid = point_valid & previous_point_valid
                if np.any(pair_valid):
                    delta = np.abs(distance[pair_valid] - previous_distance[pair_valid])
                    consecutive_distance_delta_mm.extend(delta.tolist())
                    consecutive_changed_fraction_ge_50mm.append(
                        float(np.count_nonzero(delta >= 50.0) / delta.size)
                    )
            previous_distance = distance.copy()
            previous_point_valid = point_valid.copy()

            frame_metadata.append(
                {
                    "frame_id": int(getattr(frame, "frame_id", -1)),
                    "receive_monotonic_sec": time.monotonic(),
                    "sdk_timestamp_sec": int(getattr(frame, "timeStampS", 0)),
                    "sdk_timestamp_nsec": int(getattr(frame, "timeStampNS", 0)),
                    "width": width,
                    "height": height,
                    "point_count": len(points),
                    "distance_count": int(distance.size),
                    "raw_distance_count": int(raw_distance.size),
                    "amplitude_count": int(amplitude.size),
                    "array_shape_ok": True,
                    "point_valid_fraction": point_valid_fractions[-1],
                    "mapping_valid_fraction": mapping_valid_fractions[-1],
                    "distance_valid_fraction": dist_valid_fractions[-1],
                    "raw_distance_valid_fraction": (
                        rawdist_valid_fractions[-1] if rawdist_valid_fractions else None
                    ),
                    "temperature_c": temperatures_c[-1],
                    "vcsel_temperature_c": vcsel_temperatures_c[-1],
                    "dust_percent": int(getattr(frame, "dust_percent", -1)),
                    "frame_label": str(getattr(frame, "frame_label", "")),
                }
            )
            if len(frame_metadata) >= args.frame_count:
                enough_frames.set()

    started = time.monotonic()
    result = {
        "schema": "smartwheel.xtm60_signal_diagnostic.v1",
        "read_only": True,
        "device_configuration_written": False,
        "extra_sdk_filters_applied": False,
        "ip_address": args.ip_address,
        "udp_destination": f"{args.udp_dest_ip}:{args.udp_dest_port}",
        "image_type": args.image_type,
        "mapping_range_m": [args.range_min_m, args.range_max_m],
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
        if args.start_at_monotonic > 0.0:
            remaining = args.start_at_monotonic - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
        result["start_call_monotonic_sec"] = time.monotonic()
        result["requested_start_monotonic_sec"] = (
            args.start_at_monotonic if args.start_at_monotonic > 0.0 else None
        )
        result["start_lateness_sec"] = (
            result["start_call_monotonic_sec"] - args.start_at_monotonic
            if args.start_at_monotonic > 0.0
            else None
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

    captured_count = min(len(frame_metadata), args.frame_count)
    result.update(
        {
            "elapsed_sec": time.monotonic() - started,
            "events": events,
            "frame_count": captured_count,
            "width": width,
            "height": height,
            "frame_public_fields": sorted(public_frame_fields, key=lambda item: item["name"]),
            "point_valid_fraction": _stats(point_valid_fractions),
            "mapping_valid_fraction": _stats(mapping_valid_fractions),
            "distance_valid_fraction": _stats(dist_valid_fractions),
            "raw_distance_valid_fraction": _stats(rawdist_valid_fractions),
            "temperature_c": _stats(temperatures_c),
            "vcsel_temperature_c": _stats(vcsel_temperatures_c),
            "point_range_m": _stats(point_ranges),
            "distance_vendor_valid_values": _stats(dist_ranges),
            "raw_distance_vendor_valid_values": _stats(rawdist_ranges),
            "amplitude_all_non_sentinel": _stats(amplitude_all_valid_encoding),
            "amplitude_for_point_valid": _stats(amplitude_for_point_valid),
            "amplitude_for_point_invalid": _stats(amplitude_for_point_invalid),
            "grayscale": _stats(grayscale_values),
            "level_top_values": _top_counts(level_values),
            "reflectivity": _stats(reflectivity_values),
            "consecutive_distance_delta_mm": _stats(consecutive_distance_delta_mm),
            "consecutive_changed_fraction_ge_50mm": _stats(
                consecutive_changed_fraction_ge_50mm
            ),
            "distance_invalid_top_values": _top_counts(dist_invalid_values),
            "raw_distance_invalid_top_values": _top_counts(rawdist_invalid_values),
            "amplitude_invalid_top_values": _top_counts(amplitude_invalid_values),
            "point_valid_region_grid_10x10": _region_grid(
                point_valid_sum, captured_count, width, height
            ),
            "mapping_valid_region_grid_10x10": _region_grid(
                mapping_valid_sum, captured_count, width, height
            ),
            "amplitude_non_sentinel_region_grid_10x10": _region_grid(
                amplitude_valid_sum, captured_count, width, height
            ),
            "point_valid_row_fraction": (
                np.mean(
                    (point_valid_sum / captured_count).reshape(height, width), axis=1
                ).round(6).tolist()
                if captured_count > 0 and point_valid_sum.size == width * height
                else []
            ),
            "point_valid_column_fraction": (
                np.mean(
                    (point_valid_sum / captured_count).reshape(height, width), axis=0
                ).round(6).tolist()
                if captured_count > 0 and point_valid_sum.size == width * height
                else []
            ),
            "frame_metadata": frame_metadata,
        }
    )
    result["checks"] = {
        "requested_frames_received": captured_count == args.frame_count,
        "organized_160x60": width == 160 and height == 60,
        "full_point_distance_amplitude_arrays": all(
            item.get("array_shape_ok") for item in frame_metadata
        ),
        "raw_distance_exposed": any(
            item.get("raw_distance_count") == width * height for item in frame_metadata
        ),
    }
    if args.validity_pgm is not None:
        result["validity_pgm_written"] = _write_pgm(
            args.validity_pgm, point_valid_sum, captured_count, width, height
        )
        result["validity_pgm"] = str(args.validity_pgm)
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
    parser.add_argument("--frame-count", type=int, default=30)
    parser.add_argument("--range-min-m", type=float, default=0.3)
    parser.add_argument("--range-max-m", type=float, default=12.0)
    parser.add_argument(
        "--start-at-monotonic",
        type=float,
        default=0.0,
        help="Optional host CLOCK_MONOTONIC time for the sdk.start call.",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--validity-pgm", type=Path)
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
