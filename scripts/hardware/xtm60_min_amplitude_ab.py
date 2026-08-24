#!/usr/bin/env python3
"""Guarded XT-M60 minimum-amplitude A/B test with mandatory restoration."""

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


def _public_values(obj):
    values = {}
    for name in dir(obj):
        if name.startswith("_"):
            continue
        try:
            value = getattr(obj, name)
        except Exception as exc:
            values[name] = f"<read failed: {exc}>"
            continue
        if callable(value):
            continue
        if isinstance(value, (bool, int, float, str)):
            values[name] = value
        elif isinstance(value, (list, tuple)):
            values[name] = list(value)
        else:
            try:
                values[name] = int(value)
            except (TypeError, ValueError):
                values[name] = str(value)
    return values


def _analyse(frames, range_min_m, range_max_m):
    valid_fractions = []
    mapping_fractions = []
    amplitudes = []
    ranges = []
    consecutive_delta_mm = []
    consecutive_changed_fraction_ge_50mm = []
    pixel_valid_sum = None
    previous_distance = None
    previous_valid = None

    for frame in frames:
        distance = frame["distance"]
        amplitude = frame["amplitude"]
        norms = frame["norms"]
        count = distance.size
        amp_encoding_valid = (
            np.isfinite(amplitude) & (amplitude >= 0.0) & (amplitude < 64000.0)
        )
        valid = np.isfinite(norms) & (norms > 0.0) & amp_encoding_valid
        mapping = valid & (norms >= range_min_m) & (norms <= range_max_m)
        valid_fractions.append(float(np.count_nonzero(valid) / count))
        mapping_fractions.append(float(np.count_nonzero(mapping) / count))
        amplitudes.extend(amplitude[valid].tolist())
        ranges.extend(norms[mapping].tolist())
        if pixel_valid_sum is None:
            pixel_valid_sum = np.zeros(count, dtype=np.float64)
        pixel_valid_sum += valid.astype(np.float64)
        if previous_distance is not None:
            pair_valid = valid & previous_valid
            if np.any(pair_valid):
                delta = np.abs(distance[pair_valid] - previous_distance[pair_valid])
                consecutive_delta_mm.extend(delta.tolist())
                consecutive_changed_fraction_ge_50mm.append(
                    float(np.count_nonzero(delta >= 50.0) / delta.size)
                )
        previous_distance = distance
        previous_valid = valid

    width = frames[0]["width"] if frames else 0
    height = frames[0]["height"] if frames else 0
    region_grid = []
    if frames and pixel_valid_sum is not None and width * height == pixel_valid_sum.size:
        image = (pixel_valid_sum / len(frames)).reshape(height, width)
        for y0 in range(0, height, 10):
            row = []
            for x0 in range(0, width, 10):
                row.append(
                    round(
                        float(
                            np.mean(
                                image[
                                    y0 : min(y0 + 10, height),
                                    x0 : min(x0 + 10, width),
                                ]
                            )
                        ),
                        4,
                    )
                )
            region_grid.append(row)
    return {
        "frame_count": len(frames),
        "point_valid_fraction": _stats(valid_fractions),
        "mapping_valid_fraction": _stats(mapping_fractions),
        "point_amplitude": _stats(amplitudes),
        "mapping_range_m": _stats(ranges),
        "consecutive_distance_delta_mm": _stats(consecutive_delta_mm),
        "consecutive_changed_fraction_ge_50mm": _stats(
            consecutive_changed_fraction_ge_50mm
        ),
        "point_valid_region_grid_10x10": region_grid,
    }


def collect(args):
    sdk_root = find_xtsdk_root(args.sdk_root)
    configure_xtsdk_import_path(sdk_root)
    xintan_sdk = _import_xintan_sdk(sdk_root)
    sdk = _with_sdk_example_cwd(sdk_root, xintan_sdk.XtSdk)
    connected = threading.Event()
    phase_done = threading.Event()
    lock = threading.Lock()
    events = []
    current_phase = None
    phase_frames = {}
    original_min_amplitude = None
    write_attempted = False

    def on_event(event):
        cmd_id = int(getattr(event, "cmdid", -1))
        events.append({"event": str(getattr(event, "eventstr", "")), "cmd_id": cmd_id})
        try:
            if cmd_id == 0xFE and sdk.isconnect():
                connected.set()
        except Exception:
            pass

    def on_frame(frame):
        with lock:
            if current_phase is None:
                return
            frames = phase_frames[current_phase]
            if len(frames) >= args.frame_count:
                return
            width = int(getattr(frame, "width", 0))
            height = int(getattr(frame, "height", 0))
            points = list(getattr(frame, "points", []))
            distance = np.asarray(getattr(frame, "distData", []), dtype=np.float64)
            amplitude = np.asarray(getattr(frame, "amplData", []), dtype=np.float64)
            count = width * height
            if (
                count <= 0
                or len(points) != count
                or distance.size != count
                or amplitude.size != count
            ):
                return
            xyz = np.asarray(
                [(point.x, point.y, point.z) for point in points], dtype=np.float64
            )
            frames.append(
                {
                    "width": width,
                    "height": height,
                    "distance": distance.copy(),
                    "amplitude": amplitude.copy(),
                    "norms": np.linalg.norm(xyz, axis=1),
                }
            )
            if len(frames) >= args.frame_count:
                phase_done.set()

    def read_config():
        config_ok, config = sdk.getDevConfig()
        if not config_ok:
            raise RuntimeError("getDevConfig failed")
        return config, _public_values(config)

    def capture_phase(name):
        nonlocal current_phase
        phase_frames[name] = []
        phase_done.clear()
        with lock:
            current_phase = name
        image_type = xintan_sdk.ImageType(args.image_type)
        try:
            start_ok = bool(sdk.start(image_type, False))
        except TypeError:
            start_ok = bool(sdk.start(image_type))
        if not start_ok:
            raise RuntimeError(f"{name}: sdk.start returned false")
        complete = phase_done.wait(args.frame_timeout_sec)
        sdk.stop()
        with lock:
            current_phase = None
        time.sleep(args.settle_sec)
        if not complete:
            raise RuntimeError(
                f"{name}: received {len(phase_frames[name])}/{args.frame_count} frames"
            )
        return _analyse(
            phase_frames[name], args.range_min_m, args.range_max_m
        )

    started = time.monotonic()
    result = {
        "schema": "smartwheel.xtm60_min_amplitude_ab.v1",
        "ip_address": args.ip_address,
        "expected_serial": args.expected_serial,
        "image_type": args.image_type,
        "requested_sequence": [
            args.expected_initial,
            args.test_value,
            args.expected_initial,
        ],
        "vendor_documented_typical_range": [50, 100],
        "motor_writes": False,
        "imaging_write_scope": "minimum amplitude only; restored before shutdown",
        "calls": [],
    }
    try:
        sdk.setCallback(on_event, on_frame)
        result["set_connect_ip_return"] = bool(sdk.setConnectIpaddress(args.ip_address))
        sdk.startup()
        if not connected.wait(args.connect_timeout_sec):
            raise RuntimeError("SDK connection timeout")
        result["connected"] = True
        result["set_udp_destination_return"] = bool(
            sdk.setUdpDestIp(args.udp_dest_ip, args.udp_dest_port)
        )
        info_ok, info = sdk.getDevInfo()
        if not info_ok:
            raise RuntimeError("getDevInfo failed")
        serial = str(getattr(info, "sn", ""))
        result["serial"] = serial
        if serial != args.expected_serial:
            raise RuntimeError(
                f"serial gate failed: expected {args.expected_serial}, got {serial}"
            )
        before_config, before_values = read_config()
        original_min_amplitude = int(getattr(before_config, "miniAmp"))
        result["before_config"] = before_values
        if original_min_amplitude != args.expected_initial:
            raise RuntimeError(
                "minimum-amplitude gate failed: "
                f"expected {args.expected_initial}, got {original_min_amplitude}"
            )

        result["phases"] = {}
        result["phases"][f"control_{original_min_amplitude}"] = capture_phase(
            f"control_{original_min_amplitude}"
        )

        write_attempted = True
        test_ok = bool(sdk.setMinAmplitude(args.test_value))
        result["calls"].append(
            {"name": "setMinAmplitude_test", "value": args.test_value, "ok": test_ok}
        )
        if not test_ok:
            raise RuntimeError("test setMinAmplitude returned false")
        time.sleep(args.settle_sec)
        test_config, test_values = read_config()
        result["test_config"] = test_values
        if int(getattr(test_config, "miniAmp")) != args.test_value:
            raise RuntimeError("test minimum-amplitude readback mismatch")
        result["phases"][f"test_{args.test_value}"] = capture_phase(
            f"test_{args.test_value}"
        )

        restore_ok = bool(sdk.setMinAmplitude(original_min_amplitude))
        result["calls"].append(
            {
                "name": "setMinAmplitude_restore",
                "value": original_min_amplitude,
                "ok": restore_ok,
            }
        )
        if not restore_ok:
            raise RuntimeError("restore setMinAmplitude returned false")
        time.sleep(args.settle_sec)
        restored_config, restored_values = read_config()
        result["restored_config"] = restored_values
        if int(getattr(restored_config, "miniAmp")) != original_min_amplitude:
            raise RuntimeError("restored minimum-amplitude readback mismatch")
        result["phases"][f"restored_{original_min_amplitude}"] = capture_phase(
            f"restored_{original_min_amplitude}"
        )
        result["restoration_verified"] = True
        result["success"] = True
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        with lock:
            current_phase = None
        try:
            sdk.stop()
        except Exception as exc:
            result["stop_error"] = str(exc)
        if write_attempted and original_min_amplitude is not None:
            try:
                emergency_restore_ok = bool(
                    sdk.setMinAmplitude(original_min_amplitude)
                )
                result["emergency_restore_call"] = {
                    "value": original_min_amplitude,
                    "ok": emergency_restore_ok,
                }
                time.sleep(args.settle_sec)
                final_config, final_values = read_config()
                result["final_config"] = final_values
                result["final_min_amplitude"] = int(
                    getattr(final_config, "miniAmp")
                )
                result["final_restoration_verified"] = (
                    result["final_min_amplitude"] == original_min_amplitude
                )
            except Exception as exc:
                result["emergency_restore_error"] = str(exc)
                result["final_restoration_verified"] = False
        try:
            sdk.shutdown()
        except Exception as exc:
            result["shutdown_error"] = str(exc)
        result["elapsed_sec"] = time.monotonic() - started
        result["events"] = events
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip-address", default="192.168.0.101")
    parser.add_argument("--udp-dest-ip", default="192.168.0.100")
    parser.add_argument("--udp-dest-port", type=int, default=7687)
    parser.add_argument("--expected-serial", required=True)
    parser.add_argument("--expected-initial", type=int, default=70)
    parser.add_argument("--test-value", type=int, default=50)
    parser.add_argument("--image-type", type=int, default=4)
    parser.add_argument("--frame-count", type=int, default=60)
    parser.add_argument("--range-min-m", type=float, default=0.3)
    parser.add_argument("--range-max-m", type=float, default=12.0)
    parser.add_argument("--connect-timeout-sec", type=float, default=15.0)
    parser.add_argument("--frame-timeout-sec", type=float, default=20.0)
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--sdk-root", default="/home/nvidia/smartwheel/xtsdk_py")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = collect(args)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result.get("success") and result.get("final_restoration_verified") else 2


if __name__ == "__main__":
    raise SystemExit(main())
