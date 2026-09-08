#!/usr/bin/env python3
"""Read-only dual XT-M60 raw capture with an independent H30 recorder.

The XT-M60 SDK runs in one process per sensor and never shares an rclpy
event loop.  Each frame is written as fixed-size binary arrays plus JSONL
metadata, while H30 samples are recorded with the same CLOCK_MONOTONIC clock.
No device configuration or motor command is issued.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import select
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "src" / "wheelchair_sensors"))


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _sensor_child(args: argparse.Namespace) -> int:
    from wheelchair_sensors.xtm60_adapter_node import (
        _import_xintan_sdk,
        _with_sdk_example_cwd,
        configure_xtsdk_import_path,
        find_xtsdk_root,
    )

    sensor = args.sensor
    device_ip, bind_ip = (
        ("192.168.0.101", "192.168.0.100")
        if sensor == "left"
        else ("192.168.1.101", "192.168.1.100")
    )
    out = Path(args.output_dir) / sensor
    out.mkdir(parents=True, exist_ok=True)
    status_path = out / "status.json"
    meta_path = out / "frames.jsonl"
    points_path = out / "points_xyz_f32.bin"
    distance_path = out / "distance_f32.bin"
    amplitude_path = out / "amplitude_f32.bin"
    stop_path = Path(args.stop_file)
    stop_event = threading.Event()
    connected = threading.Event()
    frame_queue: queue.Queue[dict] = queue.Queue(maxsize=64)
    state = {"sensor": sensor, "state": "starting", "frame_count": 0, "dropped_frames": 0}
    _write_json(status_path, state)

    try:
        sdk_root = find_xtsdk_root(args.sdk_root)
        configure_xtsdk_import_path(sdk_root)
        xintan_sdk = _import_xintan_sdk(sdk_root)
        sdk = _with_sdk_example_cwd(sdk_root, xintan_sdk.XtSdk)

        def on_event(event):
            try:
                if int(getattr(event, "cmdid", -1)) == 0xFE and sdk.isconnect():
                    connected.set()
            except Exception:
                pass

        def on_frame(frame):
            if stop_event.is_set():
                return
            width = int(getattr(frame, "width", 0))
            height = int(getattr(frame, "height", 0))
            points = list(getattr(frame, "points", []))
            xyz = np.asarray([(p.x, p.y, p.z) for p in points], dtype="<f4")
            distance = np.asarray(getattr(frame, "distData", []), dtype="<f4")
            amplitude = np.asarray(getattr(frame, "amplData", []), dtype="<f4")
            count = width * height
            if count <= 0 or xyz.shape != (count, 3) or distance.size != count or amplitude.size != count:
                return
            norms = np.linalg.norm(xyz, axis=1)
            valid = np.isfinite(norms) & (norms > 0) & np.isfinite(amplitude) & (amplitude >= 0)
            item = {
                "frame_id": int(getattr(frame, "frame_id", -1)),
                "host_monotonic_ns": time.monotonic_ns(),
                "host_wall_ns": time.time_ns(),
                "sdk_timestamp_sec": int(getattr(frame, "timeStampS", 0)),
                "sdk_timestamp_nsec": int(getattr(frame, "timeStampNS", 0)),
                "width": width,
                "height": height,
                "point_count": int(xyz.shape[0]),
                "valid_fraction": float(np.count_nonzero(valid) / count),
                "temperature_c": float(getattr(frame, "temperature", 0)) / 100.0,
                "vcsel_temperature_c": float(getattr(frame, "vcseltemperature", 0)) / 100.0,
                "xyz": xyz,
                "distance": distance,
                "amplitude": amplitude,
            }
            try:
                frame_queue.put_nowait(item)
            except queue.Full:
                state["dropped_frames"] += 1

        sdk.setCallback(on_event, on_frame)
        state["set_connect_ip_return"] = bool(sdk.setConnectIpaddress(device_ip))
        sdk.startup()
        if not connected.wait(float(args.connect_timeout_sec)):
            state.update({"state": "connect_timeout", "error": f"no connection to {device_ip}"})
            _write_json(status_path, state)
            return 2
        state["connected_monotonic_ns"] = time.monotonic_ns()
        state["set_udp_destination_return"] = bool(sdk.setUdpDestIp(bind_ip, int(args.udp_port)))
        target_ns = int(float(args.start_at_monotonic) * 1_000_000_000)
        while time.monotonic_ns() < target_ns and not stop_event.is_set():
            time.sleep(0.001)
        state["start_call_monotonic_ns"] = time.monotonic_ns()
        try:
            started = bool(sdk.start(xintan_sdk.ImageType(4), False))
        except TypeError:
            started = bool(sdk.start(xintan_sdk.ImageType(4)))
        state.update({"state": "capturing" if started else "start_failed", "start_return": started})
        _write_json(status_path, state)

        writer_stop = threading.Event()

        def writer():
            count = 0
            with meta_path.open("w", encoding="utf-8") as meta, points_path.open("wb") as pf, distance_path.open("wb") as df, amplitude_path.open("wb") as af:
                while not writer_stop.is_set() or not frame_queue.empty():
                    try:
                        item = frame_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    xyz = item.pop("xyz")
                    distance = item.pop("distance")
                    amplitude = item.pop("amplitude")
                    pf.write(xyz.tobytes(order="C"))
                    df.write(distance.tobytes(order="C"))
                    af.write(amplitude.tobytes(order="C"))
                    count += 1
                    item["sequence"] = count - 1
                    meta.write(json.dumps(item, ensure_ascii=False, sort_keys=True) + "\n")
                    if count % 20 == 0:
                        meta.flush()
                state["frame_count"] = count

        writer_thread = threading.Thread(target=writer, name=f"{sensor}-writer", daemon=True)
        writer_thread.start()
        deadline = time.monotonic() + float(args.duration_sec)
        while time.monotonic() < deadline and not stop_path.exists():
            time.sleep(0.05)
    except KeyboardInterrupt:
        state["interrupted"] = True
    except Exception as exc:
        state.update({"state": "error", "error": repr(exc)})
    finally:
        stop_event.set()
        try:
            sdk.stop()
        except Exception as exc:
            state["stop_error"] = repr(exc)
        try:
            sdk.shutdown()
        except Exception as exc:
            state["shutdown_error"] = repr(exc)
        if "writer_stop" in locals():
            writer_stop.set()
            writer_thread.join(timeout=10.0)
        state["state"] = "stopped" if state.get("state") not in {"error", "connect_timeout", "start_failed"} else state["state"]
        state["stopped_monotonic_ns"] = time.monotonic_ns()
        _write_json(status_path, state)
    return 0 if state.get("state") == "stopped" and state.get("frame_count", 0) > 0 else 2


def _h30_capture(output_dir: Path, stop_event: threading.Event, status: dict) -> None:
    from wheelchair_sensors.imu_adapter_node import H30ImuAdapter

    path = output_dir / "imu_h30.jsonl"
    sample_count = 0
    try:
        adapter = H30ImuAdapter(port="/dev/smartwheel_h30_imu", baud_rate=460800, timeout_sec=0.01)
        status["state"] = "capturing"
        with path.open("w", encoding="utf-8") as stream:
            while not stop_event.is_set():
                samples = adapter.read_samples()
                for sample in samples:
                    sample_count += 1
                    receive_mono = getattr(sample, "host_receive_monotonic_ns", None)
                    receive_wall = getattr(sample, "host_receive_time_ns", None)
                    interpolated_mono = getattr(sample, "host_interpolated_monotonic_ns", None)
                    interpolated_wall = getattr(sample, "host_interpolated_time_ns", None)
                    record = {
                        # Keep the established fields on the estimated
                        # per-sample timeline used by the analyzer. Preserve
                        # the raw read boundary separately for diagnostics.
                        "host_monotonic_ns": interpolated_mono or receive_mono or time.monotonic_ns(),
                        "host_wall_ns": interpolated_wall or receive_wall or time.time_ns(),
                        "host_receive_monotonic_ns": receive_mono,
                        "host_receive_wall_ns": receive_wall,
                        "sample_timestamp_us": sample.sample_timestamp_us,
                        "accel_mps2": sample.accel_mps2,
                        "gyro_rps": sample.gyro_rps,
                        "euler_rad": sample.euler_rad,
                        "quat_xyzw": sample.quat_xyzw,
                    }
                    stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
                if samples:
                    stream.flush()
        adapter.close()
    except Exception as exc:
        status.update({"state": "error", "error": repr(exc)})
    finally:
        status["sample_count"] = sample_count
        if status.get("state") != "error":
            status["state"] = "stopped"


def _coordinator(args: argparse.Namespace) -> int:
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    stop_file = out / "STOP"
    if stop_file.exists():
        stop_file.unlink()
    marker_path = out / "markers.jsonl"
    manifest = {
        "schema": "smartwheel.h2_direct_dual_capture.v1",
        "read_only": True,
        "device_configuration_written": False,
        "rclpy_loaded_in_radar_children": False,
        "sensors": {"left": {"device_ip": "192.168.0.101", "bind_ip": "192.168.0.100"}, "right": {"device_ip": "192.168.1.101", "bind_ip": "192.168.1.100"}},
        "udp_port": int(args.udp_port),
        "start_target_monotonic_sec": time.monotonic() + float(args.ready_delay_sec),
        "duration_sec": float(args.duration_sec),
        "markers": str(marker_path),
    }
    _write_json(out / "manifest.json", manifest)
    child_env = os.environ.copy()
    bind_shim = WORKSPACE / "install" / "wheelchair_bringup" / "lib" / "wheelchair_bringup" / "libxt_bindshim.so"
    children = []
    target = manifest["start_target_monotonic_sec"]
    for sensor, bind_ip in (("left", "192.168.0.100"), ("right", "192.168.1.100")):
        env = child_env.copy()
        env.update({"LD_PRELOAD": str(bind_shim), "XT_BIND_IP": bind_ip, "XT_BIND_PORT": str(args.udp_port)})
        log = (out / f"{sensor}.stderr.log").open("w", encoding="utf-8")
        command = [sys.executable, str(Path(__file__).resolve()), "--sensor", sensor, "--output-dir", str(out), "--stop-file", str(stop_file), "--start-at-monotonic", f"{target:.9f}", "--duration-sec", str(args.duration_sec), "--udp-port", str(args.udp_port)]
        children.append((sensor, subprocess.Popen(command, cwd=WORKSPACE, env=env, stdout=log, stderr=log, text=True), log))

    h30_stop = threading.Event()
    h30_status = {"state": "starting"}
    h30_thread = threading.Thread(target=_h30_capture, args=(out, h30_stop, h30_status), daemon=True)
    h30_thread.start()
    start_marker = {"marker": "capture_started", "host_monotonic_ns": time.monotonic_ns(), "host_wall_ns": time.time_ns()}
    with marker_path.open("w", encoding="utf-8") as marker_stream:
        marker_stream.write(json.dumps(start_marker, ensure_ascii=False) + "\n")
        marker_stream.flush()
        print("CAPTURE_STARTED", flush=True)
        if args.interactive:
            print("输入 marker 文本记录时刻；输入 STOP 结束。示例：static_start / left_pose / wall_corner / yaw_left_start", flush=True)
        deadline = time.monotonic() + float(args.duration_sec)
        while time.monotonic() < deadline and not stop_file.exists():
            if args.interactive:
                ready, _, _ = select.select([sys.stdin], [], [], 0.2)
                if ready:
                    line = sys.stdin.readline().strip()
                    if line:
                        if line.upper() == "STOP":
                            stop_file.touch()
                            break
                        record = {"marker": line, "host_monotonic_ns": time.monotonic_ns(), "host_wall_ns": time.time_ns()}
                        marker_stream.write(json.dumps(record, ensure_ascii=False) + "\n")
                        marker_stream.flush()
                        print(f"MARKER_RECORDED {line}", flush=True)
            else:
                time.sleep(0.2)
    stop_file.touch()
    h30_stop.set()
    h30_thread.join(timeout=10.0)
    for _, process, log in children:
        try:
            process.send_signal(signal.SIGINT)
        except Exception:
            pass
    child_results = []
    for sensor, process, log in children:
        try:
            returncode = process.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            process.kill()
            returncode = process.wait(timeout=5.0)
        log.close()
        child_results.append({"sensor": sensor, "returncode": returncode})
    manifest.update({"h30": h30_status, "children": child_results, "stopped_monotonic_ns": time.monotonic_ns()})
    _write_json(out / "manifest.json", manifest)
    return 0 if all(item["returncode"] == 0 for item in child_results) else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sensor", choices=("left", "right"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path)
    parser.add_argument("--start-at-monotonic", type=float, default=0.0)
    parser.add_argument("--duration-sec", type=float, default=180.0)
    parser.add_argument("--ready-delay-sec", type=float, default=8.0)
    parser.add_argument("--connect-timeout-sec", type=float, default=20.0)
    parser.add_argument("--udp-port", type=int, default=7687)
    parser.add_argument("--sdk-root", default="/home/nvidia/smartwheel/xtsdk_py")
    parser.add_argument("--interactive", action="store_true")
    args = parser.parse_args()
    if args.sensor:
        if args.stop_file is None:
            parser.error("--stop-file is required in sensor mode")
        return _sensor_child(args)
    return _coordinator(args)


if __name__ == "__main__":
    raise SystemExit(main())
