#!/usr/bin/env python3
"""Apply the supported XT-M60 upper-computer export fields with before/after evidence."""

import argparse
import configparser
import hashlib
import json
import statistics
import sys
import threading
import time
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "src" / "wheelchair_sensors"))

from wheelchair_sensors.xtm60_adapter_node import (  # noqa: E402
    _import_xintan_sdk,
    _with_sdk_example_cwd,
    configure_xtsdk_import_path,
    find_xtsdk_root,
)


def _json_safe(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "tolist"):
        return _json_safe(value.tolist())
    try:
        return int(value)
    except (TypeError, ValueError):
        return str(value)


def _public_values(obj):
    result = {}
    for name in dir(obj):
        if name.startswith("_"):
            continue
        try:
            value = getattr(obj, name)
        except Exception as exc:
            result[name] = f"<read failed: {exc}>"
            continue
        if not callable(value):
            result[name] = _json_safe(value)
    return result


def _load_export(path):
    parser = configparser.ConfigParser()
    with path.open("r", encoding="utf-8-sig") as stream:
        parser.read_file(stream)
    setting = parser["Setting"]
    filters = parser["Filters"]
    return {
        "image_type": setting.getint("imgType"),
        "intgs": setting.getint("intgs"),
        "int1": setting.getint("int1"),
        "int2": setting.getint("int2"),
        "int3": setting.getint("int3"),
        "int4": setting.getint("int4"),
        "hdr": setting.getint("HDR"),
        "min_lsb": setting.getint("minLSB"),
        "max_fps": setting.getint("maxfps"),
        "freq1": setting.getint("freq1"),
        "freq2": setting.getint("freq2"),
        "freq3": setting.getint("freq3"),
        "freq4": setting.getint("freq4"),
        "export_freq5": setting.getint("freq5"),
        "median_size": filters.getint("medianSize"),
        "kalman_enable": filters.getboolean("kalmanEnable"),
        "kalman_factor": round(filters.getfloat("kalmanFactor") * 1000),
        "kalman_threshold": filters.getint("kalmanThreshold"),
        "edge_enable": filters.getboolean("edgeEnable"),
        "edge_threshold": filters.getint("edgeThreshold"),
        "dust_enable": filters.getboolean("dustEnable"),
        "dust_threshold": filters.getint("dustThreshold"),
        "dust_frames": filters.getint("dustFrames"),
        "postprocess_enable": filters.getboolean("postprocessEnable"),
        "postprocess_threshold": filters.getfloat("postprocessThreshold"),
        "dynamics_enabled": filters.getint("dynamicsEnabled"),
        "dynamics_winsize": filters.getint("dynamicsWinsize"),
        "reflective_enable": filters.getboolean("reflectiveEnable"),
        "reflective_min": filters.getfloat("ref_th_min"),
        "reflective_max": filters.getfloat("ref_th_max"),
    }


def _call(calls, name, function, *args):
    entry = {"name": name, "arguments": [_json_safe(value) for value in args]}
    try:
        value = function(*args)
        entry["return"] = _json_safe(value)
        entry["ok"] = value is not False
    except Exception as exc:
        entry["ok"] = False
        entry["error"] = str(exc)
    calls.append(entry)
    return entry["ok"]


def apply(args):
    export = _load_export(args.export)
    export_sha256_actual = hashlib.sha256(args.export.read_bytes()).hexdigest().upper()
    sdk_root = find_xtsdk_root(args.sdk_root)
    configure_xtsdk_import_path(sdk_root)
    xs = _import_xintan_sdk(sdk_root)
    sdk = _with_sdk_example_cwd(sdk_root, xs.XtSdk)
    connected = threading.Event()
    enough_frames = threading.Event()
    events = []
    frame_times = []
    calls = []

    def on_event(event):
        cmd_id = int(getattr(event, "cmdid", -1))
        events.append({"event": str(getattr(event, "eventstr", "")), "cmd_id": cmd_id})
        try:
            if cmd_id == 0xFE and sdk.isconnect():
                connected.set()
        except Exception:
            pass

    def on_frame(frame):
        frame_times.append(
            int(getattr(frame, "timeStampS", 0))
            + int(getattr(frame, "timeStampNS", 0)) / 1_000_000_000.0
        )
        if len(frame_times) >= args.frame_count:
            enough_frames.set()

    result = {
        "schema": "smartwheel.xtm60_apply_export_config.v1",
        "ip_address": args.ip_address,
        "udp_destination": f"{args.udp_dest_ip}:{args.udp_dest_port}",
        "export_path": str(args.export),
        "export_sha256_expected": args.export_sha256.upper(),
        "export_sha256_actual": export_sha256_actual,
        "device_configuration_written": True,
        "authorized_by_user": True,
        "export_values": export,
        "unsupported_or_intentionally_unmapped_export_fields": {
            "int5": "not consumed by the official M60 SDK example",
            "freq5": (
                f"export value {export['export_freq5']} is not consumed; official example "
                "passes ModulationFreq(2) as API argument five"
            ),
            "spatial_filter": "not exposed by this SDK build/example",
            "average_filter": "disabled in export and not exposed by this SDK example",
        },
    }
    started = time.monotonic()
    try:
        sdk.setCallback(on_event, on_frame)

        # These SDK-side stream filters mirror the official example and export.
        if export["kalman_enable"]:
            _call(calls, "setSdkKalmanFilter", sdk.setSdkKalmanFilter,
                  export["kalman_factor"], export["kalman_threshold"], 2000)
        if export["median_size"] > 0:
            _call(calls, "setSdkMedianFilter", sdk.setSdkMedianFilter, export["median_size"])
        if export["edge_enable"]:
            _call(calls, "setSdkEdgeFilter", sdk.setSdkEdgeFilter, export["edge_threshold"])
        if export["dust_enable"]:
            _call(calls, "setSdkDustFilter", sdk.setSdkDustFilter,
                  export["dust_threshold"], export["dust_frames"])
        if export["postprocess_enable"]:
            _call(calls, "setPostProcess", sdk.setPostProcess,
                  export["postprocess_threshold"], export["dynamics_enabled"],
                  export["dynamics_winsize"])
        if export["reflective_enable"]:
            _call(calls, "setSdkReflectiveFilter", sdk.setSdkReflectiveFilter,
                  export["reflective_min"], export["reflective_max"])

        result["set_connect_ip_return"] = bool(sdk.setConnectIpaddress(args.ip_address))
        sdk.startup()
        if not connected.wait(args.connect_timeout_sec):
            result["error"] = "SDK connection timeout"
            return result

        result["connected"] = True
        before_ok, before = sdk.getDevConfig()
        result["before_readback_ok"] = bool(before_ok)
        if before_ok:
            result["before_readback"] = _public_values(before)

        _call(calls, "stop_before_config", sdk.stop)
        _call(calls, "setIntTimesus", sdk.setIntTimesus,
              export["intgs"], export["int1"], export["int2"], export["int3"],
              export["int4"], 0)
        _call(calls, "setMultiModFreq", sdk.setMultiModFreq,
              xs.ModulationFreq(export["freq1"]), xs.ModulationFreq(export["freq2"]),
              xs.ModulationFreq(export["freq3"]), xs.ModulationFreq(export["freq4"]),
              xs.ModulationFreq(2))
        _call(calls, "setHdrMode", sdk.setHdrMode, xs.HDRMode(export["hdr"]))
        _call(calls, "setMinAmplitude", sdk.setMinAmplitude, export["min_lsb"])
        _call(calls, "setMaxFps", sdk.setMaxFps, export["max_fps"])
        _call(calls, "setUdpDestIp", sdk.setUdpDestIp,
              args.udp_dest_ip, args.udp_dest_port)
        _call(calls, "start_measurement", sdk.start,
              xs.ImageType(export["image_type"]), False)

        enough_frames.wait(args.frame_timeout_sec)
        time.sleep(args.readback_settle_sec)
        after_ok, after = sdk.getDevConfig()
        result["after_readback_ok"] = bool(after_ok)
        if after_ok:
            result["after_readback"] = _public_values(after)
    finally:
        result["calls"] = calls
        result["events"] = events
        result["frame_count"] = len(frame_times)
        deltas = [b - a for a, b in zip(frame_times, frame_times[1:])]
        result["frame_interval_sec"] = {
            "count": len(deltas),
            "min": min(deltas) if deltas else None,
            "mean": statistics.fmean(deltas) if deltas else None,
            "max": max(deltas) if deltas else None,
        }
        try:
            sdk.stop()
        except Exception as exc:
            result["stop_error"] = str(exc)
        try:
            sdk.shutdown()
        except Exception as exc:
            result["shutdown_error"] = str(exc)
        result["elapsed_sec"] = time.monotonic() - started

    after = result.get("after_readback", {})
    expected_integrations = [export["int1"], export["int2"], export["int3"], export["int4"]]
    result["checks"] = {
        "export_hash_matches": export_sha256_actual == args.export_sha256.upper(),
        "all_sdk_calls_succeeded": all(call["ok"] for call in calls),
        "requested_frames_received": len(frame_times) >= args.frame_count,
        "integration_time_gs_matches": after.get("integrationTimeGs") == export["intgs"],
        "integration_times_match": after.get("integrationTimes") == expected_integrations,
        "hdr_matches": after.get("hdrMode") == export["hdr"],
        "minimum_amplitude_matches": after.get("miniAmp") == export["min_lsb"],
        "maximum_fps_matches": after.get("maxfps") == export["max_fps"],
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip-address", required=True)
    parser.add_argument("--udp-dest-ip", required=True)
    parser.add_argument("--udp-dest-port", type=int, default=7687)
    parser.add_argument("--sdk-root", default="/home/nvidia/smartwheel/xtsdk_py")
    parser.add_argument(
        "--export",
        type=Path,
        default=WORKSPACE / "docs/hardware/vendor/xintan_windows_export_20260722.xtcfg",
    )
    parser.add_argument(
        "--export-sha256",
        default="1611F653AA13B35AEA38DE844A130840342A73CA8AA1DB47910B7C8588E0D9F4",
    )
    parser.add_argument("--connect-timeout-sec", type=float, default=15.0)
    parser.add_argument("--frame-timeout-sec", type=float, default=15.0)
    parser.add_argument("--readback-settle-sec", type=float, default=2.0)
    parser.add_argument("--frame-count", type=int, default=20)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    result = apply(args)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result.get("checks") and all(result["checks"].values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
