#!/usr/bin/env python3
"""Read XT-M60 identity and saved configuration without starting measurement."""

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
        except Exception as exc:  # pragma: no cover - vendor object edge case
            values[name] = f"<read failed: {exc}>"
            continue
        if callable(value):
            continue
        values[name] = _json_safe(value)
    return values


def collect(ip_address: str, sdk_root_value: str, timeout_sec: float):
    sdk_root = find_xtsdk_root(sdk_root_value)
    lib_dir = configure_xtsdk_import_path(sdk_root)
    xintan_sdk = _import_xintan_sdk(sdk_root)
    sdk = _with_sdk_example_cwd(sdk_root, xintan_sdk.XtSdk)
    connected = threading.Event()
    event_log = []

    def on_event(event):
        cmd_id = int(getattr(event, "cmdid", -1))
        event_log.append(
            {
                "event": str(getattr(event, "eventstr", "")),
                "cmd_id": cmd_id,
            }
        )
        try:
            if cmd_id == 0xFE and sdk.isconnect():
                connected.set()
        except Exception:
            pass

    def on_frame(_frame):
        # Measurement is never started; this callback exists only for SDK API shape.
        return None

    started = time.monotonic()
    result = {
        "schema": "smartwheel.xtm60_device_info.v1",
        "read_only": True,
        "measurement_started": False,
        "ip_address": ip_address,
        "sdk_root": str(sdk_root),
        "sdk_library_dir": str(lib_dir),
    }
    try:
        sdk.setCallback(on_event, on_frame)
        result["set_connect_ip_return"] = bool(sdk.setConnectIpaddress(ip_address))
        sdk.startup()
        if not connected.wait(timeout_sec):
            result["connected"] = bool(sdk.isconnect())
            result["error"] = f"connection timeout after {timeout_sec:.1f}s"
            return result

        result["connected"] = True
        info_ok, device_info = sdk.getDevInfo()
        config_ok, device_config = sdk.getDevConfig()
        result["device_info_ok"] = bool(info_ok)
        result["device_config_ok"] = bool(config_ok)
        if info_ok:
            result["device_info"] = _public_values(device_info)
        if config_ok:
            result["device_config"] = _public_values(device_config)
        return result
    finally:
        result["elapsed_sec"] = time.monotonic() - started
        result["events"] = event_log
        try:
            sdk.shutdown()
        except Exception as exc:
            result["shutdown_error"] = str(exc)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip-address", default="192.168.0.101")
    parser.add_argument("--sdk-root", default="/home/nvidia/smartwheel/xtsdk_py")
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = collect(args.ip_address, args.sdk_root, args.timeout_sec)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result.get("device_info_ok") and result.get("device_config_ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
