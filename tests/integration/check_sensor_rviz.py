#!/usr/bin/env python3
"""Inspect existing live single-frame lidar streams in owned Orin RViz windows.

Starts only RViz, never a sensor, SLAM, wheel interface or map publisher.
Example during an already running sensor capture:
  PYTHONPATH=src python3 tests/integration/check_sensor_rviz.py \
    --session sensor_check_001 --output-dir reports/sensor_rviz/run_001
"""

import argparse
from collections import OrderedDict
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from check_rviz_display import capture_owned_window, desktop_environment
from wc_runtime.cli import ROOT, name, target
from wc_runtime.component import descendants, process, signal_descendant


def timestamp(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def cloud_record(message):
    return {"stamp_ns": timestamp(message.header.stamp), "frame_id": message.header.frame_id,
            "width": int(message.width), "height": int(message.height),
            "point_count": int(message.width) * int(message.height),
            "payload_sha256": hashlib.sha256(bytes(message.data)).hexdigest()}


def cloud_key(record):
    return record["frame_id"], record["stamp_ns"], record["payload_sha256"]


def bounded_add(rows, key, value, maximum=64):
    rows[key] = value
    while len(rows) > maximum:
        rows.popitem(last=False)


def stop_owned(child, descriptor, row):
    """Let the existing subreaping wrapper clean only its own descendants."""
    try:
        if child.poll() is None:
            signal.pidfd_send_signal(descriptor, signal.SIGINT)
        try:
            child.wait(timeout=18)
        except subprocess.TimeoutExpired:
            row["cleanup_timeout"] = True
            owner = process(child.pid)
            if owner:
                for record in descendants(owner):
                    signal_descendant(record, owner, signal.SIGKILL)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                signal.pidfd_send_signal(descriptor, signal.SIGKILL)
                child.wait(timeout=3)
        row["wrapper_exit_code"] = child.returncode
    finally:
        os.close(descriptor)
        row["wrapper_alive_after_cleanup"] = child.poll() is None
        saved = row.get("rviz_process_identity")
        remaining = process(saved["pid"]) if saved else None
        row["rviz_identity_alive_after_cleanup"] = bool(remaining and remaining.start_ticks == saved["start_ticks"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=name, help="existing live capture session; no source is started")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--settle-seconds", type=float, default=4.)
    parser.add_argument("--view-timeout", type=float, default=12.)
    args = parser.parse_args(argv)
    target()
    output = args.output_dir.absolute()
    if not output.resolve().is_relative_to(ROOT / "reports") or any(p.is_symlink() for p in (output, *output.parents)):
        raise ValueError("evidence requires a new nonsymlink directory below project reports")
    output.mkdir(parents=True, exist_ok=False)
    evidence = {"test": "actual_orin_owned_rviz_lidar_single_frames", "status": "FAIL",
                "scope": "LIVE_SINGLE_FRAME_NOT_MAP", "source_mode": "real", "session_id": args.session,
                "ros_domain_id": 83, "started_wall_time_ns": time.time_ns(), "views": {},
                "starts_sensors": False, "starts_slam": False, "map_validated": False,
                "extrinsics_validated": False, "filter_quality_validated": False,
                "time_synchronization_validated": False,
                "visual_review": "PENDING_INDEPENDENT_SCREENSHOT_INSPECTION"}
    lock = probe = ros = None
    old_handlers = {}

    def interrupted(signum, frame):
        raise InterruptedError("display test interrupted by signal " + str(signum))

    try:
        if not 0 <= args.settle_seconds <= 10 or not args.settle_seconds < args.view_timeout <= 30:
            raise ValueError("require 0<=settle<=10 and settle<view-timeout<=30 seconds")
        if os.environ.get("ROS_DOMAIN_ID") != "83" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
            raise RuntimeError("requires existing live capture on localhost ROS domain 83")
        if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
            raise RuntimeError("owned cleanup requires Linux pidfds")
        lock = (ROOT / ".phase1_runtime/locks/domain-83-sensor-display.lock").open("a")
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        env = desktop_environment()
        evidence["display"] = env["DISPLAY"]
        expected_ids = json.loads((ROOT / "config/live_unvalidated.json").read_text())["sensor_ids"]
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, interrupted)
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from wc_interfaces.msg import SourceFrame
        ros = rclpy
        ros.init()
        probe = Node("wc_sensor_rviz_probe_" + str(os.getpid()))
        for side in ("left", "right"):
            cloud_topic = "/wc_mapping/lidar_" + side + "/points_filtered"
            source_topic = "/wc_mapping/lidar_" + side + "/source_frame_filtered"
            row = {"status": "FAIL", "cloud_topic": cloud_topic, "source_topic": source_topic,
                   "expected_frame_id": "lidar_" + side, "received_clouds": 0, "received_source_frames": 0}
            evidence["views"][side] = row
            clouds, frames, cloud_stamps = OrderedDict(), OrderedDict(), OrderedDict()
            callback_errors = []

            def on_cloud(message):
                record = cloud_record(message)
                record["received_monotonic_ns"] = time.monotonic_ns()
                row["received_clouds"] += 1
                if record["frame_id"] != "lidar_" + side or not record["point_count"]:
                    callback_errors.append("empty cloud or unexpected native frame")
                    return
                bounded_add(clouds, cloud_key(record), record)
                bounded_add(cloud_stamps, record["stamp_ns"], True)
                row.setdefault("first_cloud", record)
                row["last_cloud"] = record

            def on_frame(message):
                row["received_source_frames"] += 1
                if message.session_id != args.session or message.side != side or message.sensor_id != expected_ids[side]:
                    callback_errors.append("live SourceFrame session/side/device identity mismatch")
                    return
                if "representation=host_filtered" not in message.diagnostic_flags:
                    callback_errors.append("SourceFrame does not identify host_filtered representation")
                    return
                record = cloud_record(message.cloud)
                record.update(session_id=message.session_id, sensor_id=message.sensor_id, side=message.side,
                    stream_epoch=message.stream_epoch, frame_sequence=int(message.frame_sequence),
                    source_config_hash=message.source_config_hash, source_header_stamp_ns=timestamp(message.header.stamp),
                    host_receive_time_ns=timestamp(message.host_receive_time), host_monotonic_ns=int(message.host_monotonic_ns),
                    device_timestamp_raw=int(message.device_timestamp_raw), device_timestamp_unit=message.device_timestamp_unit,
                    common_time_valid=bool(message.common_time_valid), common_time_ns=int(message.common_time_ns),
                    time_source=message.time_source, coordinate_convention=message.coordinate_convention, units=message.units)
                bounded_add(frames, cloud_key(record), record)
                row["last_source_frame"] = record

            subscriptions = [probe.create_subscription(PointCloud2, cloud_topic, on_cloud, qos_profile_sensor_data),
                             probe.create_subscription(SourceFrame, source_topic, on_frame, qos_profile_sensor_data)]
            node_name = "wc_sensor_rviz_" + side + "_" + str(os.getpid())
            command = [sys.executable, "-m", "wc_runtime.component", "--parent", str(os.getpid()), "--",
                       "rviz2", "-d", str(ROOT / "config/rviz" / ("sensor_" + side + ".rviz")),
                       "--ros-args", "-r", "__node:=" + node_name, "-r", "__ns:=/"]
            child = None
            descriptor = None
            try:
                with (output / (side + ".log")).open("x") as log:
                    child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                             env=env, start_new_session=True)
                    descriptor = os.pidfd_open(child.pid)
                    row["wrapper_pid"] = child.pid
                    deadline, ready_since = time.monotonic() + args.view_timeout, None
                    ready = False
                    while time.monotonic() < deadline:
                        if child.poll() is not None:
                            raise RuntimeError("owned RViz exited before its display checks")
                        ros.spin_once(probe, timeout_sec=.1)
                        if callback_errors:
                            raise RuntimeError(callback_errors[0])
                        infos = probe.get_subscriptions_info_by_topic(cloud_topic)
                        row["rviz_subscriptions"] = [{"node_name": item.node_name, "node_namespace": item.node_namespace}
                            for item in infos if item.node_name == node_name and item.node_namespace == "/"]
                        matching = [key for key in clouds if key in frames]
                        now_ns = time.monotonic_ns()
                        latest_cloud = row.get("last_cloud", {})
                        source_age = (now_ns - frames[matching[-1]]["host_monotonic_ns"]) if matching else None
                        row["latest_matching_source_age_ns"] = source_age
                        ready = (len(cloud_stamps) >= 3 and bool(row["rviz_subscriptions"]) and bool(matching)
                                 and 0 <= now_ns - latest_cloud.get("received_monotonic_ns", 0) <= 1_000_000_000
                                 and source_age is not None and 0 <= source_age <= 2_000_000_000)
                        if ready:
                            ready_since = ready_since or time.monotonic()
                            row["matched_source_frame"] = frames[matching[-1]]
                            if time.monotonic() - ready_since >= args.settle_seconds:
                                break
                        else:
                            ready_since = None
                    row["distinct_cloud_stamps_observed"] = len(cloud_stamps)
                    if not ready or ready_since is None or time.monotonic() - ready_since < args.settle_seconds:
                        raise RuntimeError("live cloud count, exact source reference, RViz subscription or settle condition not met")
                    row.update(capture_owned_window(child, output / (side + ".png"), env))
                    owner = process(row["rviz_pid"])
                    if owner:
                        row["rviz_process_identity"] = {"pid": owner.pid, "start_ticks": owner.start_ticks}
                    row["status"] = "DDS_AND_OWNED_WINDOW_CAPTURE_PASS"
            finally:
                # Ignore repeated interruptions only while cleaning our own processes.
                for signum in old_handlers:
                    signal.signal(signum, signal.SIG_IGN)
                if child is not None:
                    if descriptor is not None:
                        stop_owned(child, descriptor, row)
                    else:
                        row["wrapper_exit_code"] = child.poll()
                        row["cleanup_error"] = "pidfd unavailable; wrapper parent-death handler remains active"
                for subscription in subscriptions:
                    probe.destroy_subscription(subscription)
                for signum in old_handlers:
                    signal.signal(signum, interrupted)
            if row.get("wrapper_exit_code") != 0 or row.get("wrapper_alive_after_cleanup") or row.get("rviz_identity_alive_after_cleanup") or row.get("cleanup_timeout"):
                raise RuntimeError("owned RViz cleanup did not complete normally")
        evidence["status"] = "PASS"
    except BaseException as error:
        evidence["error"] = type(error).__name__ + ": " + str(error)
    finally:
        for signum, previous in old_handlers.items():
            signal.signal(signum, previous)
        if probe is not None:
            probe.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
        if lock is not None:
            lock.close()
        evidence["finished_wall_time_ns"] = time.time_ns()
        with (output / "result.json").open("x") as stream:
            json.dump(evidence, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    print(json.dumps(evidence, ensure_ascii=False, indent=2))
    return 0 if evidence["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
