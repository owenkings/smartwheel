#!/usr/bin/env python3
"""Mirror stable XT-M60 imaging fields from a source unit to a target unit.

This deliberately excludes identity/network fields and RespDevConfig.isFilterOn.
Repeated read-only observations showed isFilterOn changing on every connection,
so it is not treated as a persistent device setting. Measurement is never started.
"""

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


def _open_sdk(ip_address, sdk_root, timeout_sec):
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
    sdk.startup()
    if not connected.wait(timeout_sec):
        try:
            sdk.shutdown()
        finally:
            raise RuntimeError(f"{ip_address}: connection timeout")
    return xs, sdk, events, set_ip_ok


def _read_device(ip_address, sdk_root, timeout_sec):
    xs, sdk, events, set_ip_ok = _open_sdk(
        ip_address, sdk_root, timeout_sec
    )
    try:
        info_ok, info = sdk.getDevInfo()
        config_ok, config = sdk.getDevConfig()
        if not info_ok or not config_ok:
            raise RuntimeError(
                f"{ip_address}: getDevInfo/getDevConfig failed "
                f"({bool(info_ok)}/{bool(config_ok)})"
            )
        return {
            "ip_address": ip_address,
            "set_connect_ip_return": set_ip_ok,
            "device_info": _public_values(info),
            "device_config": _public_values(config),
            "events": events,
        }
    finally:
        try:
            sdk.stop()
        except Exception:
            pass
        sdk.shutdown()


def _call(calls, name, function, *arguments):
    entry = {
        "name": name,
        "arguments": [_json_safe(value) for value in arguments],
    }
    try:
        value = function(*arguments)
        entry["return"] = _json_safe(value)
        entry["ok"] = value is not False
    except Exception as exc:
        entry["ok"] = False
        entry["error"] = str(exc)
    calls.append(entry)
    if not entry["ok"]:
        raise RuntimeError(f"{name} failed: {entry.get('error', entry.get('return'))}")


def _stable_imaging(config):
    return {
        "integrationTimeGs": int(config["integrationTimeGs"]),
        "integrationTimes": [int(value) for value in config["integrationTimes"]],
        "hdrMode": int(config["hdrMode"]),
        "miniAmp": int(config["miniAmp"]),
        "maxfps": int(config["maxfps"]),
        "modFreq": int(config["modFreq"]),
        "freq": config["freq"],
        "roi": [int(value) for value in config["roi"]],
        "bCompensateOn": int(config["bCompensateOn"]),
        "bBinningH": int(config["bBinningH"]),
        "bBinningV": int(config["bBinningV"]),
    }


def mirror(args):
    sdk_root = find_xtsdk_root(args.sdk_root)
    configure_xtsdk_import_path(sdk_root)
    started = time.monotonic()
    source = _read_device(
        args.source_ip, sdk_root, args.connect_timeout_sec
    )
    source_sn = source["device_info"].get("sn")
    if source_sn != args.expected_source_sn:
        raise RuntimeError(
            f"source serial mismatch: expected {args.expected_source_sn}, got {source_sn}"
        )

    xs, sdk, events, set_ip_ok = _open_sdk(
        args.target_ip, sdk_root, args.connect_timeout_sec
    )
    calls = []
    result = {
        "schema": "smartwheel.xtm60_mirror_stable_imaging.v1",
        "authorized_by_user": True,
        "measurement_started": False,
        "source": source,
        "target_ip": args.target_ip,
        "excluded_fields": {
            "identity_and_network": "must remain unique per physical device/subnet",
            "isFilterOn": (
                "excluded because repeated read-only getDevConfig calls returned "
                "different values without any write"
            ),
            "imgType": (
                "stored value is unchanged; production start explicitly requests "
                "ImageType 4"
            ),
        },
        "calls": calls,
        "target_events": events,
        "set_connect_ip_return": set_ip_ok,
    }
    try:
        info_ok, target_info = sdk.getDevInfo()
        before_ok, before_obj = sdk.getDevConfig()
        if not info_ok or not before_ok:
            raise RuntimeError("target getDevInfo/getDevConfig failed")
        target_info = _public_values(target_info)
        before = _public_values(before_obj)
        result["target_info"] = target_info
        result["target_before"] = before
        if target_info.get("sn") != args.expected_target_sn:
            raise RuntimeError(
                "target serial mismatch: expected "
                f"{args.expected_target_sn}, got {target_info.get('sn')}"
            )

        source_stable = _stable_imaging(source["device_config"])
        before_stable = _stable_imaging(before)
        result["source_stable_imaging"] = source_stable
        result["target_before_stable_imaging"] = before_stable
        result["differences_before"] = {
            key: {"source": source_stable[key], "target": before_stable[key]}
            for key in source_stable
            if source_stable[key] != before_stable[key]
        }

        integrations = source_stable["integrationTimes"]
        if len(integrations) != 4:
            raise RuntimeError(f"unexpected source integrationTimes: {integrations}")

        _call(calls, "stop_before_config", sdk.stop)
        _call(
            calls,
            "setIntTimesus",
            sdk.setIntTimesus,
            source_stable["integrationTimeGs"],
            integrations[0],
            integrations[1],
            integrations[2],
            integrations[3],
            0,
        )
        _call(
            calls,
            "setHdrMode",
            sdk.setHdrMode,
            xs.HDRMode(source_stable["hdrMode"]),
        )
        _call(
            calls,
            "setMinAmplitude",
            sdk.setMinAmplitude,
            source_stable["miniAmp"],
        )
        _call(calls, "setMaxFps", sdk.setMaxFps, source_stable["maxfps"])
        time.sleep(args.readback_settle_sec)

        after_ok, after_obj = sdk.getDevConfig()
        if not after_ok:
            raise RuntimeError("target after-write getDevConfig failed")
        after = _public_values(after_obj)
        after_stable = _stable_imaging(after)
        result["target_after"] = after
        result["target_after_stable_imaging"] = after_stable
        result["differences_after"] = {
            key: {"source": source_stable[key], "target": after_stable[key]}
            for key in source_stable
            if source_stable[key] != after_stable[key]
        }
        result["checks"] = {
            "source_serial_matches": source_sn == args.expected_source_sn,
            "target_serial_matches": (
                target_info.get("sn") == args.expected_target_sn
            ),
            "all_calls_succeeded": all(call["ok"] for call in calls),
            "stable_imaging_matches_source": not result["differences_after"],
            "third_exposure_matches_source": (
                after_stable["integrationTimes"][2] == integrations[2]
            ),
            "measurement_never_started": True,
        }
    finally:
        try:
            sdk.stop()
        except Exception as exc:
            result["final_stop_error"] = str(exc)
        try:
            sdk.shutdown()
        except Exception as exc:
            result["shutdown_error"] = str(exc)
        result["elapsed_sec"] = time.monotonic() - started
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-ip", default="192.168.1.101")
    parser.add_argument("--target-ip", default="192.168.0.101")
    parser.add_argument(
        "--expected-source-sn", default="XTM60B20250324000134"
    )
    parser.add_argument(
        "--expected-target-sn", default="XTM60B20250324000151"
    )
    parser.add_argument("--sdk-root", default="/home/nvidia/smartwheel/xtsdk_py")
    parser.add_argument("--connect-timeout-sec", type=float, default=15.0)
    parser.add_argument("--readback-settle-sec", type=float, default=2.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    result = mirror(args)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    checks = result.get("checks", {})
    return 0 if checks and all(checks.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
