#!/usr/bin/env python3
"""Read and compare both XT-M60 configurations without writing or measuring."""

import argparse
import json
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
    if hasattr(value, "tolist"):
        return _json_safe(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    try:
        return int(value)
    except (TypeError, ValueError):
        return str(value)


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
        if not callable(value):
            values[name] = _json_safe(value)
    return values


def _read_device(ip_address, expected_serial, sdk_root, timeout_sec):
    xs = _import_xintan_sdk(sdk_root)
    sdk = _with_sdk_example_cwd(sdk_root, xs.XtSdk)
    connected = threading.Event()
    events = []

    def on_event(event):
        cmd_id = int(getattr(event, "cmdid", -1))
        events.append(
            {"event": str(getattr(event, "eventstr", "")), "cmd_id": cmd_id}
        )
        try:
            if cmd_id == 0xFE and sdk.isconnect():
                connected.set()
        except Exception:
            pass

    sdk.setCallback(on_event, lambda _frame: None)
    set_ip_ok = bool(sdk.setConnectIpaddress(ip_address))
    started = time.monotonic()
    measurement_started = False
    sdk.startup()
    try:
        if not connected.wait(timeout_sec):
            raise RuntimeError(f"{ip_address}: connection timeout")
        info_ok, info = sdk.getDevInfo()
        config_ok, config = sdk.getDevConfig()
        if not info_ok or not config_ok:
            raise RuntimeError(
                f"{ip_address}: getDevInfo/getDevConfig failed "
                f"({bool(info_ok)}/{bool(config_ok)})"
            )
        info_values = _public_values(info)
        serial = info_values.get("sn")
        if serial != expected_serial:
            raise RuntimeError(
                f"{ip_address}: expected serial {expected_serial}, got {serial}"
            )
        return {
            "ip_address": ip_address,
            "expected_serial": expected_serial,
            "set_connect_ip_return": set_ip_ok,
            "device_info": info_values,
            "device_config": _public_values(config),
            "events": events,
            "measurement_started": measurement_started,
            "elapsed_sec": time.monotonic() - started,
        }
    finally:
        try:
            sdk.stop()
        except Exception:
            pass
        sdk.shutdown()


def _stable_imaging(config):
    fields = (
        "integrationTimeGs",
        "integrationTimes",
        "hdrMode",
        "miniAmp",
        "maxfps",
        "modFreq",
        "freq",
        "roi",
        "bCompensateOn",
        "bBinningH",
        "bBinningV",
        "imgType",
    )
    return {name: config.get(name) for name in fields}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-ip", default="192.168.0.101")
    parser.add_argument("--right-ip", default="192.168.1.101")
    parser.add_argument(
        "--expected-left-sn", default="XTM60B20250324000151"
    )
    parser.add_argument(
        "--expected-right-sn", default="XTM60B20250324000134"
    )
    parser.add_argument("--sdk-root", default="/home/nvidia/smartwheel/xtsdk_py")
    parser.add_argument("--connect-timeout-sec", type=float, default=15.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    sdk_root = find_xtsdk_root(args.sdk_root)
    configure_xtsdk_import_path(sdk_root)
    started = time.monotonic()
    left = _read_device(
        args.left_ip, args.expected_left_sn, sdk_root, args.connect_timeout_sec
    )
    right = _read_device(
        args.right_ip, args.expected_right_sn, sdk_root, args.connect_timeout_sec
    )
    left_stable = _stable_imaging(left["device_config"])
    right_stable = _stable_imaging(right["device_config"])
    differences = {
        name: {"left": left_stable[name], "right": right_stable[name]}
        for name in left_stable
        if left_stable[name] != right_stable[name]
    }
    result = {
        "schema": "smartwheel.xtm60_readonly_config_snapshot.v1",
        "read_only": True,
        "measurement_started": False,
        "excluded_from_stable_comparison": {
            "identity_and_network": "must differ between devices",
            "isFilterOn": (
                "observed to change between read-only connections and is not "
                "treated as a stable persistent setting"
            ),
        },
        "left": left,
        "right": right,
        "left_stable_imaging": left_stable,
        "right_stable_imaging": right_stable,
        "stable_imaging_differences": differences,
        "checks": {
            "read_only": True,
            "measurement_never_started": (
                not left["measurement_started"]
                and not right["measurement_started"]
            ),
            "stable_imaging_matches": not differences,
        },
        "elapsed_sec": time.monotonic() - started,
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
