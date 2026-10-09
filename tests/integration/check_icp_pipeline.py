#!/usr/bin/env python3
"""Observe the actual ROS dual-lidar -> ICP -> RTABMap -> map pipeline.

Run this observer FIRST in the same sourced ROS environment and ROS_DOMAIN_ID
as the fixture. Wait for OBSERVER_READY on stdout, then start wc_phase1 map.
Example (replace every path/session with the actual prepared session):

  python3 tests/integration/check_icp_pipeline.py \
    --session-config /absolute/session/session.json \
    --output /absolute/session/evidence/ros_pipeline.json \
    --duration 45 --close-snapshot

This test never publishes sensor, odometry, TF, navigation or motor messages.
The optional flag calls ONLY the session-checked software graph close service.
No graph edges, truth poses or measured points are injected by the observer.
Exit 0: all required integration observations passed. Exit 1: failed/missing
integration evidence. Exit 2: observer/configuration failure. Evidence is still
written on callback failures and ordinary interruption. Accuracy is MEASURED,
not an accuracy acceptance claim: this fixture defines no accuracy threshold.
"""

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import sys
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation


TOPICS = {
    "bundle": "/wc_mapping/fusion/bundle",
    "accepted": "/wc_mapping/accepted",
    "odom": "/wc_mapping/odom",
    "graph": "/wc_mapping/graph/snapshot",
    "status": "/wc_mapping/status",
    "cloud": "/wc_mapping/map/cloud",
    "grid": "/wc_mapping/map/grid",
    "tf": "/tf",
    "tf_static": "/tf_static",
}
CLOSE_SERVICE = "/wc_mapping/close_graph_snapshot"


def safe_path(value, *, existing=True):
    path = Path(value)
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError("an absolute path without '..' is required: " + str(path))
    for component in (path, *path.parents):
        if component.is_symlink():
            raise ValueError("symlink path refused: " + str(component))
    if existing and not path.exists():
        raise ValueError("missing path: " + str(path))
    return path


def read_json(path):
    path = safe_path(path)
    if path.stat().st_size > 64 * 1024 * 1024:
        raise ValueError("JSON evidence input exceeds 64 MiB")
    return json.loads(path.read_text(encoding="utf-8"))


def sha(data):
    return hashlib.sha256(data).hexdigest()


def file_sha(path):
    digest = hashlib.sha256()
    with safe_path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stamp(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def pose_values(pose):
    return [float(pose.position.x), float(pose.position.y), float(pose.position.z),
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w)]


def transform_values(transform):
    return [float(transform.translation.x), float(transform.translation.y), float(transform.translation.z),
            float(transform.rotation.x), float(transform.rotation.y),
            float(transform.rotation.z), float(transform.rotation.w)]


def matrix(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (7,) or not np.all(np.isfinite(values)):
        raise ValueError("pose must contain seven finite values")
    if abs(np.linalg.norm(values[3:]) - 1.0) > 1e-4:
        raise ValueError("quaternion is not normalized; observer will not silently repair it")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    result[:3, 3] = values[:3]
    return result


def valid_matrix(value):
    value = np.asarray(value, dtype=float)
    if (value.shape != (4, 4) or not np.all(np.isfinite(value))
            or not np.allclose(value[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(value[:3, :3]), 1.0, atol=1e-6, rtol=0)):
        raise ValueError("truth pose is not a finite rigid SE(3) transform")
    return value


def compact_cloud(cloud):
    return {
        "stamp_ns": stamp(cloud.header), "frame_id": cloud.header.frame_id,
        "count": int(cloud.width) * int(cloud.height), "width": int(cloud.width), "height": int(cloud.height),
        "point_step": int(cloud.point_step), "row_step": int(cloud.row_step),
        "is_bigendian": bool(cloud.is_bigendian), "is_dense": bool(cloud.is_dense),
        "fields": [[f.name, int(f.offset), int(f.datatype), int(f.count)] for f in cloud.fields],
        "data_bytes": len(cloud.data), "data_sha256": sha(bytes(cloud.data)),
    }


def compact_bundle(message):
    codes = bytes(message.source_codes)
    counts = Counter(codes)
    result = {name: getattr(message, name) for name in (
        "session_id", "bundle_id", "source_mode", "sensor_mode", "calibration_id", "left_clock_model_id",
        "right_clock_model_id", "left_raw_key", "right_raw_key", "left_time_ns", "right_time_ns",
        "compensation_mode", "temporal_error_bound_valid", "temporal_error_bound_m", "left_retained_count",
        "right_retained_count", "calibration_valid", "time_valid", "dual_valid")}
    result.update(stamp_ns=stamp(message.header), frame_id=message.header.frame_id,
                  cloud=compact_cloud(message.cloud), rejection_reasons=list(message.rejection_reasons),
                  source_codes_count=len(codes), source_codes_sha256=sha(codes),
                  source_code_counts={str(k): v for k, v in sorted(counts.items())},
                  left_contributing_points=counts[1] + counts[3], right_contributing_points=counts[2] + counts[3])
    for name in ("t_ref_left_sensor", "t_ref_right_sensor", "t_ref_rig_at_left_time", "t_ref_rig_at_right_time"):
        result[name] = transform_values(getattr(message, name))
    # Comparison includes the complete cloud payload hash and every bundle field.
    result["fingerprint"] = sha(json.dumps(result, sort_keys=True, allow_nan=False).encode())
    return result


def compact_odom(message):
    values = pose_values(message.pose.pose)
    matrix(values)
    return dict(stamp_ns=stamp(message.header), frame_id=message.header.frame_id,
                child_frame_id=message.child_frame_id, pose_xyz_xyzw=values,
                covariance=[float(x) for x in message.pose.covariance])


def gid(info):
    value = getattr(info, "publisher_gid", None)
    if value is None:
        return None
    try:
        data = bytes(value)
        return data.hex() if data and any(data) else None
    except (TypeError, ValueError):
        return None


def endpoint_name(endpoint):
    return endpoint.node_namespace.rstrip("/") + "/" + endpoint.node_name


def result(status, detail, **evidence):
    return dict(status=status, detail=detail, **evidence)


def trajectory_metrics(estimated, truth):
    common = sorted(set(estimated) & set(truth))
    if len(common) < 2:
        return result("BLOCKED", "At least two exact timestamp matches are required.", matched_count=len(common))
    first = common[0]
    alignment = truth[first] @ np.linalg.inv(estimated[first])
    errors = []
    for key in common:
        aligned = alignment @ estimated[key]
        error = np.linalg.inv(truth[key]) @ aligned
        errors.append(dict(stamp_ns=key, translation_error_m=float(np.linalg.norm(error[:3, 3])),
                           rotation_error_deg=float(np.rad2deg(Rotation.from_matrix(error[:3, :3]).magnitude()))))
    translation = np.array([e["translation_error_m"] for e in errors])
    angular = np.array([e["rotation_error_deg"] for e in errors])
    return result("PASS", "Actual estimates compared with recorded synthetic truth; numerical accuracy has no acceptance threshold.",
                  accuracy_acceptance="NOT_RUN", matched_count=len(common), truth_count=len(truth),
                  alignment_policy="One rigid SE(3) transform fixed from the first matching pose (6 DoF); no fit to subsequent poses, no scale adjustment.",
                  alignment_scale=1.0, alignment_first_stamp_ns=first, T_truth_odom_alignment=alignment.tolist(),
                  translation_rmse_m=float(np.sqrt(np.mean(translation ** 2))),
                  translation_median_m=float(np.median(translation)), translation_max_m=float(translation.max()),
                  rotation_rmse_deg=float(np.sqrt(np.mean(angular ** 2))), rotation_max_deg=float(angular.max()),
                  missing_truth_stamps=sorted(set(truth) - set(estimated)), errors=errors)


class Recorder:
    """Own compact evidence only; no sensor/graph implementation dependencies."""

    def __init__(self, config, close_requested):
        self.config = config
        self.start = time.monotonic()
        self.start_unix_ns = time.time_ns()
        self.counts = Counter()
        self.foreign = Counter()
        self.errors = []
        self.bundles, self.accepted, self.odometry, self.graphs = {}, {}, {}, {}
        self.statuses, self.clouds, self.grids = [], {}, {}
        self.endpoints = defaultdict(dict)
        self.observed_publishers = defaultdict(set)
        self.tf = defaultdict(list)
        self.discovery_errors = []
        self.close = dict(requested=close_requested, status="NOT_RUN", detail="Flag not supplied." if not close_requested else "Waiting for complete fixture graph and provider identity.")
        self.close_future = None
        self.interrupted = False
        self.native_tf_summary = None

    def elapsed(self):
        return round(time.monotonic() - self.start, 6)

    def problem(self, kind, detail):
        if len(self.errors) < 200:
            self.errors.append(dict(kind=kind, detail=str(detail), elapsed_s=self.elapsed()))

    def callback(self, name, handler):
        # The inspected target's Humble executor calls subscription callbacks
        # with only the message. Newer executors may additionally pass info.
        def receive(message, info=None):
            self.counts[name] += 1
            publisher = gid(info)
            self.observed_publishers[name].add(publisher or "UNAVAILABLE")
            try:
                handler(message, publisher)
            except Exception as failure:
                self.problem("callback_" + name, str(failure))
        return receive

    def own(self, message, name):
        if message.session_id != self.config["session_id"]:
            self.foreign[name] += 1
            return False
        return True

    def put(self, destination, key, value, name):
        if key in destination and destination[key] != value:
            self.problem(name + "_conflicting_duplicate", key)
        else:
            destination[key] = value

    def bundle(self, message, _publisher):
        if self.own(message, "bundle"):
            self.put(self.bundles, int(message.bundle_id), compact_bundle(message), "bundle")

    def accept(self, message, _publisher):
        if not self.own(message.bundle, "accepted"):
            return
        tracking = message.tracking
        value = dict(bundle=compact_bundle(message.bundle), tracking={name: getattr(tracking, name) for name in (
            "session_id", "bundle_id", "odom_epoch", "tracking_state", "accepted", "left_solver_input_count",
            "right_solver_input_count", "inliers_available", "inliers", "residual_available", "residual",
            "degeneracy_available", "degeneracy_metric")})
        value["tracking"].update(stamp_ns=stamp(tracking.header), frame_id=tracking.header.frame_id,
                                 rejection_reasons=list(tracking.rejection_reasons), odometry=compact_odom(tracking.odometry))
        self.put(self.accepted, int(message.bundle.bundle_id), value, "accepted")

    def odom(self, message, _publisher):
        self.put(self.odometry, stamp(message.header), compact_odom(message), "odom")

    def graph(self, message, _publisher):
        if not self.own(message, "graph"):
            return
        value = dict(stamp_ns=stamp(message.header), frame_id=message.header.frame_id,
                     session_id=message.session_id, graph_epoch=message.graph_epoch, revision=int(message.revision),
                     source_mode=message.source_mode, calibration_id=message.calibration_id,
                     left_clock_model_id=message.left_clock_model_id, right_clock_model_id=message.right_clock_model_id,
                     raw_index_complete=bool(message.raw_index_complete), raw_index_hash=message.raw_index_hash,
                     nodes=[dict(node_id=int(n.node_id), bundle_id=int(n.bundle_id), odom_epoch=n.odom_epoch,
                                 pose_xyz_xyzw=pose_values(n.optimized_pose), raw_keys=list(n.raw_keys),
                                 t_node_sensor=[transform_values(t) for t in n.t_node_sensor]) for n in message.nodes],
                     edges=[dict(from_id=int(e.from_id), to_id=int(e.to_id), type=e.type, synthetic=bool(e.synthetic)) for e in message.edges])
        for node in value["nodes"]:
            matrix(node["pose_xyz_xyzw"])
        self.put(self.graphs, int(message.revision), value, "graph")

    def status(self, message, _publisher):
        if not self.own(message, "status"):
            return
        value = {name: getattr(message, name) for name in (
            "session_id", "source_mode", "sensor_mode", "state", "left_fresh", "right_fresh", "calibration_valid",
            "time_valid", "tracking_valid", "latest_accepted_time_ns", "graph_epoch", "graph_revision",
            "completed_map_revision", "rebuilding", "consistent_snapshot", "map_id", "map_version", "navigation_validated")}
        value["pause_reasons"] = list(message.pause_reasons)
        # Header is publication wall time, NOT graph reference time or revision.
        if not self.statuses or value != self.statuses[-1]["value"]:
            self.statuses.append(dict(elapsed_s=self.elapsed(), value=value))

    def cloud(self, message, _publisher):
        self.put(self.clouds, stamp(message.header), compact_cloud(message), "map_cloud")

    def grid(self, message, _publisher):
        data = np.asarray(message.data, dtype=np.int8)
        value = dict(stamp_ns=stamp(message.header), frame_id=message.header.frame_id,
                     width=int(message.info.width), height=int(message.info.height), resolution=float(message.info.resolution),
                     origin_xyz_xyzw=pose_values(message.info.origin), data_sha256=sha(data.tobytes()),
                     cell_count=len(data), occupied_cells=int(np.count_nonzero(data > 0)),
                     free_cells=int(np.count_nonzero(data == 0)), unknown_cells=int(np.count_nonzero(data < 0)))
        self.put(self.grids, stamp(message.header), value, "map_grid")

    def transforms(self, message, publisher, *, static=False):
        frames = self.config.get("graph", {})
        wanted = {(frames.get("map_frame", "map"), frames.get("odom_frame", "odom")),
                  (frames.get("odom_frame", "odom"), frames.get("rig_frame", "rig_link"))}
        for item in message.transforms:
            edge = (item.header.frame_id, item.child_frame_id)
            if edge in wanted:
                matrix(transform_values(item.transform))
                self.tf["->".join(edge)].append(dict(stamp_ns=stamp(item.header), publisher_gid=publisher,
                                                   static=static, received_unix_ns=time.time_ns(),
                                                   transform_xyz_xyzw=transform_values(item.transform)))

    def discover(self, node):
        for name, topic in TOPICS.items():
            try:
                for endpoint in node.get_publishers_info_by_topic(topic):
                    endpoint_gid = bytes(endpoint.endpoint_gid).hex()
                    self.endpoints[name][endpoint_gid] = dict(node=endpoint_name(endpoint),
                        reliability=str(endpoint.qos_profile.reliability), durability=str(endpoint.qos_profile.durability))
            except Exception as failure:
                text = name + ": " + str(failure)
                if text not in self.discovery_errors:
                    self.discovery_errors.append(text)

    def maybe_close(self, node, client):
        if not self.close["requested"] or self.close_future is not None or not self.graphs:
            return
        expected = int(self.config.get("synthetic", {}).get("frame_count", 0))
        latest = self.graphs[max(self.graphs)]
        if expected < 1 or len(latest["nodes"]) < expected or len(self.accepted) < expected:
            return  # Never close a partial pipeline just to make this test pass.
        if self.foreign["graph"] or latest["graph_epoch"] != self.config["graph_epoch"] or latest["source_mode"] != self.config["source_mode"]:
            self.close.update(status="BLOCKED", detail="Graph topic has foreign or mismatching session evidence; service not called.")
            return
        # Trigger has no session argument. Establish one current endpoint and
        # a unique service provider with the same node, alongside the complete
        # session/epoch/source-matched graph above. Old Humble omits MessageInfo;
        # missing GIDs must not be fabricated as per-message provenance.
        try:
            publishers = node.get_publishers_info_by_topic(TOPICS["graph"])
            observed = self.observed_publishers["graph"]
            known_observed = observed - {"UNAVAILABLE"}
            if (len(publishers) != 1 or not observed
                    or known_observed and known_observed != {bytes(publishers[0].endpoint_gid).hex()}):
                self.close.update(status="BLOCKED", detail="Unique graph publisher identity is not established.")
                return
            graph_node = endpoint_name(publishers[0])
            providers = []
            for name, namespace in node.get_node_names_and_namespaces():
                services = node.get_service_names_and_types_by_node(name, namespace)
                if any(service == CLOSE_SERVICE and "std_srvs/srv/Trigger" in types for service, types in services):
                    providers.append(namespace.rstrip("/") + "/" + name)
            if providers != [graph_node] or not client.service_is_ready():
                self.close.update(status="BLOCKED", detail="Close service owner is not uniquely the observed session graph publisher.", providers=providers)
                return
            from std_srvs.srv import Trigger
            self.close.update(status="BLOCKED", detail="Asynchronous session graph close requested; awaiting real response.",
                              provider=graph_node, graph_revision=latest["revision"], requested_at_elapsed_s=self.elapsed(),
                              authorization_evidence="Unique current graph publisher endpoint and unique same-node close service; received graph session/epoch/source and expected frame count match.",
                              per_message_publisher_gid_available="UNAVAILABLE" not in observed,
                              graph_publisher_endpoint_gid=bytes(publishers[0].endpoint_gid).hex())
            self.close_future = client.call_async(Trigger.Request())
        except Exception as failure:
            self.close.update(status="BLOCKED", detail="Service identity check failed: " + str(failure))

    def close_result(self):
        if self.close_future is None or not self.close_future.done():
            return
        try:
            response = self.close_future.result()
            self.close.update(status="PASS" if response.success else "FAIL", response_success=bool(response.success),
                              response_message=response.message, detail="Received software graph close service response.")
        except Exception as failure:
            self.close.update(status="FAIL", detail="Close service failed: " + str(failure))


def native_tf_checks(recorder):
    """Correlate optional independent C++ GIDs with actual session/pose data.

    The native session_id is explicitly just an operator supplied label. It is
    never sufficient for acceptance without cross-process sample, DDS endpoint,
    time window and accepted-odom/optimized-graph pose checks below.
    """
    path = recorder.config.get("_observer_tf_evidence")
    if not path:
        return None
    evidence_path = Path(path)
    if not evidence_path.exists():
        return {"error": result("BLOCKED", "Native TF evidence is not yet available; finish the native probe before this observer's duration ends.", path=path)}
    try:
        document = read_json(evidence_path)
        if (document.get("schema_version") != 1 or document.get("test") != "native_tf_ownership_probe"
                or document.get("session_id") != recorder.config["session_id"]
                or document.get("hostname") != socket.gethostname()
                or document.get("ros_domain_id") != os.environ.get("ROS_DOMAIN_ID", "0")
                or document.get("topic") != "/tf" or document.get("rmw_gid_storage_size") != 24
                or document.get("navigation_validated") is not False):
            raise ValueError("native probe schema/session/host/domain/topic/GID format mismatch")
        if document.get("status") == "BLOCKED" and not document.get("errors"):
            return {"error": result("BLOCKED", "Native TF probe lacks a complete observation window or one of the required edges.", path=path)}
        if document.get("status") != "PASS" or document.get("errors") or document.get("interrupted"):
            raise ValueError("native probe did not complete with PASS: " + str(document.get("status")))
        start, end = document["start_unix_ns"], document["end_unix_ns"]
        now = time.time_ns()
        if (type(start) is not int or type(end) is not int or not start < end <= now
                or end < recorder.start_unix_ns or start > now):
            raise ValueError("native and Python observation windows do not overlap")
        if not recorder.accepted or not recorder.graphs or recorder.foreign:
            raise ValueError("actual accepted/graph evidence for one unambiguous session is required")
        bindings = defaultdict(set)
        for endpoint in document["endpoint_history"]:
            if endpoint["node"] != "_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_":
                bindings[endpoint["publisher_gid"]].add(endpoint["node"])
        recorder.native_tf_summary = dict(path=path, sha256=file_sha(evidence_path),
            session_id=document["session_id"], hostname=document["hostname"], ros_domain_id=document["ros_domain_id"],
            start_unix_ns=start, end_unix_ns=end, status=document["status"], messages_received=document["messages_received"],
            session_binding="Verified below against independently observed actual session messages/TF samples; label alone is insufficient.")
        checks = {}
        motion_owner = 'bundle' if recorder.config.get('motion_prior', {}).get('enabled') else 'odom'
        for edge, owner_topic in (("map->odom", "graph"), ("odom->rig_link", motion_owner)):
            native = document["edges"][edge]
            rows = recorder.tf.get(edge, [])
            samples = native["observations"]
            if not rows or not samples:
                checks[edge] = result("BLOCKED", "Both Python and native actual TF samples are required.")
                continue
            if any(row["static"] for row in rows):
                raise ValueError("a static publication was observed for a dynamic TF edge: " + edge)
            counts = Counter(sample["publisher_gid"] for sample in samples)
            if len(counts) != 1 or dict(counts) != native["publisher_gids"] or len(samples) != native["transform_count"]:
                raise ValueError("native edge publisher uniqueness or per-message count mismatch: " + edge)
            identity = next(iter(counts))
            if len(identity) != 48 or identity.lower() != identity or len(bytes.fromhex(identity)) != 24 or not any(bytes.fromhex(identity)):
                raise ValueError("native per-message GID is unavailable or invalid")
            names = bindings[identity]
            source_endpoints = recorder.endpoints[owner_topic]
            source_names = {entry["node"] for entry in source_endpoints.values()}
            python_tf_endpoint = recorder.endpoints["tf"].get(identity)
            if (len(names) != 1 or len(source_endpoints) != 1 or names != source_names
                    or not python_tf_endpoint or python_tf_endpoint["node"] not in names):
                raise ValueError("native TF GID must independently resolve to the sole actual source topic node in both observers: " + edge)
            by_stamp = defaultdict(list)
            for sample in samples:
                if type(sample["stamp_ns"]) is not int or not start <= sample["received_unix_ns"] <= end:
                    raise ValueError("native sample timestamp/window invalid")
                matrix(sample["transform_xyz_xyzw"])
                by_stamp[sample["stamp_ns"]].append(sample)
            if min(by_stamp) != native["min_stamp_ns"] or max(by_stamp) != native["max_stamp_ns"]:
                raise ValueError("native sample min/max timestamp summary mismatch")
            unmatched = []
            for row in rows:
                if not start <= row["received_unix_ns"] <= end:
                    unmatched.append(row["stamp_ns"])
                    continue
                matches = [sample for sample in by_stamp[row["stamp_ns"]]
                           if np.allclose(matrix(sample["transform_xyz_xyzw"]), matrix(row["transform_xyz_xyzw"]), atol=1e-10, rtol=0)]
                if not matches or row["publisher_gid"] not in (None, identity):
                    unmatched.append(row["stamp_ns"])
            if unmatched:
                checks[edge] = result("BLOCKED", "Native probe does not cover every independently observed Python TF sample exactly.", unmatched_stamps=sorted(set(unmatched)))
                continue
            # Bind the native edge's data to this actual session, not just to
            # the operator-supplied session_id or a same-named ROS node.
            missing, pose_mismatch = [], []
            for accepted in recorder.accepted.values():
                bundle, tracking = accepted["bundle"], accepted["tracking"]
                reference = bundle["stamp_ns"]
                actual = by_stamp.get(reference, [])
                if not actual:
                    missing.append(reference)
                    continue
                expected = matrix(tracking["odometry"]["pose_xyz_xyzw"])
                if owner_topic == "graph":
                    graphs = [g for g in recorder.graphs.values() if g["stamp_ns"] == reference]
                    if len(graphs) != 1:
                        missing.append(reference)
                        continue
                    nodes = [n for n in graphs[0]["nodes"] if n["bundle_id"] == bundle["bundle_id"]]
                    if len(nodes) != 1:
                        missing.append(reference)
                        continue
                    expected = matrix(nodes[0]["pose_xyz_xyzw"]) @ np.linalg.inv(expected)
                if any(not np.allclose(matrix(sample["transform_xyz_xyzw"]), expected, atol=1e-6, rtol=0) for sample in actual):
                    pose_mismatch.append(reference)
            if pose_mismatch:
                checks[edge] = result("FAIL", "Native TF disagrees with actual accepted odometry/optimized graph at exact session timestamps.", mismatching_stamps=pose_mismatch)
            elif missing:
                checks[edge] = result("BLOCKED", "Native TF or graph revisions do not cover every accepted session timestamp.", missing_stamps=missing)
            else:
                checks[edge] = result("PASS", "Independent native per-message GIDs establish one edge publisher; samples match Python TF and this session's actual accepted odometry/graph.",
                    evidence_source="native_rclcpp_MessageInfo", native_evidence_sha256=recorder.native_tf_summary["sha256"],
                    publisher_gids=[identity], publishers=sorted(names), native_transform_count=len(samples),
                    matched_python_tf_samples=len(rows), matched_accepted_stamps=len(recorder.accepted),
                    native_min_stamp_ns=native["min_stamp_ns"], native_max_stamp_ns=native["max_stamp_ns"],
                    pose_serialization_atol=1e-6,
                    scope="Only native observed /tf messages and this exact session's independently matched samples; no assertion about unobserved publishers.")
        return checks
    except Exception as failure:
        return {"error": result("FAIL", "Native TF evidence rejected: " + str(failure), path=path)}


def tf_checks(recorder):
    frames = recorder.config.get("graph", {})
    map_frame, odom_frame, rig_frame = (frames.get("map_frame", "map"), frames.get("odom_frame", "odom"), frames.get("rig_frame", "rig_link"))
    checks = {}
    motion_owner = 'bundle' if recorder.config.get('motion_prior', {}).get('enabled') else 'odom'
    for edge, owner_topic in ((map_frame + "->" + odom_frame, "graph"), (odom_frame + "->" + rig_frame, motion_owner)):
        rows = recorder.tf.get(edge, [])
        observed_gids = {row["publisher_gid"] for row in rows}
        tf_endpoints = dict(recorder.endpoints["tf"])
        tf_endpoints.update(recorder.endpoints["tf_static"])
        names = sorted({tf_endpoints[x]["node"] for x in observed_gids if x in tf_endpoints})
        owner_gids = recorder.observed_publishers[owner_topic]
        owner_names = {recorder.endpoints[owner_topic][x]["node"] for x in owner_gids if x in recorder.endpoints[owner_topic]}
        if not rows or None in observed_gids or len(names) != len(observed_gids) or not owner_names:
            status, detail = "BLOCKED", "Cannot establish each observed TF edge publisher and its source node from DDS GIDs."
        elif len(observed_gids) != 1 or len(owner_names) != 1 or set(names) != owner_names or any(row["static"] for row in rows):
            status, detail = "FAIL", "Edge has conflicting publishers, wrong source node, or a static publication for dynamic motion."
        else:
            status, detail = "PASS", "Exactly one observed edge publisher matches the actual source topic node during this observation window."
        checks[edge] = result(status, detail, message_count=len(rows), publishers=names,
                             publisher_gids=sorted(x or "UNAVAILABLE" for x in observed_gids),
                             expected_source_topic=TOPICS[owner_topic], observed_source_nodes=sorted(owner_names),
                             discovered_source_topic_nodes=sorted({e["node"] for e in recorder.endpoints[owner_topic].values()}),
                             discovered_tf_topic_nodes=sorted({e["node"] for e in tf_endpoints.values()}),
                             endpoint_only_evidence_note="Topic endpoints alone do not prove which publisher sent a particular TF edge; unavailable per-message GID stays BLOCKED.",
                             scope="Observed messages and discovered endpoints only; no assertion about unobserved publishers or earlier/later execution.")
    native = native_tf_checks(recorder)
    if native is not None:
        for edge, check in list(checks.items()):
            # A real Python-side contradiction is never erased by a second
            # observer. Only missing per-message provenance can be completed.
            replacement = native.get(edge, native.get("error"))
            if replacement is not None and check["status"] != "FAIL":
                if check["status"] == "BLOCKED" or replacement["status"] == "FAIL":
                    checks[edge] = dict(replacement, python_messageinfo_status="UNAVAILABLE_OR_INCOMPLETE",
                                       python_observation_evidence=check)
    return checks


def build_report(recorder):
    config = recorder.config
    checks = {}
    expected = int(config.get("synthetic", {}).get("frame_count", 0))
    accepted_ids, bundle_ids = set(recorder.accepted), set(recorder.bundles)
    missing_bundles = sorted(accepted_ids - bundle_ids)
    mismatches, invalid_dual, invalid_tracking, odom_mismatch = [], [], [], []
    contribution_rows = []
    for identifier, entry in sorted(recorder.accepted.items()):
        bundle, tracking = entry["bundle"], entry["tracking"]
        raw = recorder.bundles.get(identifier)
        if raw is not None and raw["fingerprint"] != bundle["fingerprint"]:
            mismatches.append(identifier)
        if (bundle["source_mode"] != config["source_mode"] or bundle["sensor_mode"] != "dual"
                or not all(bundle[k] for k in ("dual_valid", "calibration_valid", "time_valid"))
                or bundle["left_retained_count"] <= 0 or bundle["right_retained_count"] <= 0
                or bundle["left_contributing_points"] <= 0 or bundle["right_contributing_points"] <= 0
                or bundle["source_codes_count"] != bundle["cloud"]["count"]
                or set(bundle["source_code_counts"]) - {"1", "2", "3"}
                or bundle["stamp_ns"] != bundle["cloud"]["stamp_ns"]):
            invalid_dual.append(identifier)
        if (tracking["session_id"] != config["session_id"] or tracking["bundle_id"] != identifier
                or not tracking["accepted"] or tracking["stamp_ns"] != bundle["stamp_ns"]
                or tracking["odometry"]["stamp_ns"] != bundle["stamp_ns"] or not tracking["odom_epoch"]
                or tracking["left_solver_input_count"] <= 0 or tracking["right_solver_input_count"] <= 0):
            invalid_tracking.append(identifier)
        observed_odom = recorder.odometry.get(bundle["stamp_ns"])
        if observed_odom is None or observed_odom != tracking["odometry"]:
            odom_mismatch.append(identifier)
        contribution_rows.append(dict(bundle_id=identifier, stamp_ns=bundle["stamp_ns"],
            left_raw_key=bundle["left_raw_key"], right_raw_key=bundle["right_raw_key"],
            left_retained_count=bundle["left_retained_count"], right_retained_count=bundle["right_retained_count"],
            left_contributing_points=bundle["left_contributing_points"], right_contributing_points=bundle["right_contributing_points"],
            left_solver_input_count=tracking["left_solver_input_count"], right_solver_input_count=tracking["right_solver_input_count"],
            inliers=tracking["inliers"] if tracking["inliers_available"] else None,
            residual=tracking["residual"] if tracking["residual_available"] else None,
            degeneracy_metric=tracking["degeneracy_metric"] if tracking["degeneracy_available"] else None))
    checks["bundle_accepted_exact_match"] = result("FAIL" if missing_bundles or mismatches else "PASS" if accepted_ids else "BLOCKED",
        "Accepted bundles must equal an independently observed fusion bundle, including ID, stamp, metadata and cloud payload hash.",
        missing_bundle_ids=missing_bundles, mismatching_bundle_ids=mismatches, accepted_count=len(accepted_ids))
    checks["both_sensors_in_same_estimator_input"] = result("FAIL" if invalid_dual or invalid_tracking else "PASS" if accepted_ids else "BLOCKED",
        "Both source masks and the frontend's actual solver input counts are positive for each accepted ICP observation; no per-sensor influence/ablation claim.",
        invalid_dual_ids=invalid_dual, invalid_tracking_ids=invalid_tracking, contribution_rows=contribution_rows)
    checks["actual_icp_odometry_association"] = result("FAIL" if odom_mismatch else "PASS" if accepted_ids else "BLOCKED",
        "Tracking odometry must equal an independently observed actual /wc_mapping/odom message at the exact bundle stamp.",
        missing_or_different_odom_bundle_ids=odom_mismatch)
    checks["complete_fixture"] = result("PASS" if expected > 0 and len(accepted_ids) == expected and len(bundle_ids) == expected else "BLOCKED",
        "Full configured synthetic frame count required; dropped/rejected frames remain visible.",
        expected_frames=expected, observed_fusion_bundles=len(bundle_ids), accepted_bundles=len(accepted_ids),
        not_accepted_bundle_ids=sorted(bundle_ids - accepted_ids))
    latest = recorder.graphs[max(recorder.graphs)] if recorder.graphs else None
    graph_errors, graph_missing, closure_edges = [], [], []
    stm_size = int(config.get("graph", {}).get("stm_size", 10))
    for revision, graph in sorted(recorder.graphs.items()):
        nodes = graph["nodes"]
        node_ids = [node["node_id"] for node in nodes]
        if (graph["graph_epoch"] != config["graph_epoch"] or graph["source_mode"] != config["source_mode"]
                or not graph["raw_index_complete"] or len(graph["raw_index_hash"]) != 64
                or len(node_ids) != len(set(node_ids)) or not nodes):
            graph_errors.append(dict(revision=revision, reason="graph metadata, raw index or node identity invalid"))
        for node in nodes:
            entry = recorder.accepted.get(node["bundle_id"])
            if entry is None:
                graph_missing.append(dict(revision=revision, bundle_id=node["bundle_id"]))
                continue
            bundle, tracking = entry["bundle"], entry["tracking"]
            original_transforms = [bundle["t_ref_left_sensor"], bundle["t_ref_right_sensor"]]
            # RTAB-Map Transform stores float32; compare rigid matrices with a
            # serialization tolerance, not a physical calibration tolerance.
            transforms_match = (len(node["t_node_sensor"]) == 2 and all(
                np.allclose(matrix(actual), matrix(original), atol=1e-6, rtol=0)
                for actual, original in zip(node["t_node_sensor"], original_transforms)))
            if (node["node_id"] != node["bundle_id"] or node["odom_epoch"] != tracking["odom_epoch"]
                    or node["raw_keys"] != [bundle["left_raw_key"], bundle["right_raw_key"]]
                    or not transforms_match):
                graph_errors.append(dict(revision=revision, reason="node/raw/extrinsic association mismatch", node_id=node["node_id"]))
        for edge in graph["edges"]:
            if edge["from_id"] not in node_ids or edge["to_id"] not in node_ids or edge["synthetic"] != (config["source_mode"] == "synthetic"):
                graph_errors.append(dict(revision=revision, reason="edge endpoint or source label mismatch", edge=edge))
            if edge["type"] in ("user_closure", "virtual_closure") or edge["type"].startswith("unknown_type_"):
                graph_errors.append(dict(revision=revision, reason="manual, virtual or unknown edge cannot establish native geometric closure", edge=edge))
            if edge["type"] in ("global_closure", "local_space_closure") and abs(edge["from_id"] - edge["to_id"]) > stm_size:
                candidate = dict(edge, first_observed_revision=revision)
                if not any((e["from_id"], e["to_id"], e["type"]) == (edge["from_id"], edge["to_id"], edge["type"]) for e in closure_edges):
                    closure_edges.append(candidate)
    checks["graph_exact_association"] = result("FAIL" if graph_errors or graph_missing else "PASS" if latest else "BLOCKED",
        "Observed optimized nodes preserve accepted IDs, odom epochs, both raw keys and both original sensor transforms; edges preserve native types/source labels.",
        graph_errors=graph_errors, graph_nodes_without_observed_acceptance=graph_missing, transform_serialization_atol=1e-6,
        latest_revision=latest["revision"] if latest else None, latest_node_count=len(latest["nodes"]) if latest else 0)
    checks["complete_graph"] = result("PASS" if latest and expected > 0 and {n["bundle_id"] for n in latest["nodes"]} == accepted_ids and len(accepted_ids) == expected else "BLOCKED",
        "Final observed graph must contain every configured accepted fixture bundle.")
    checks["native_geometric_revisit"] = result("PASS" if closure_edges else "BLOCKED",
        "Observed native global/local-space constraints with endpoint separation greater than the configured STM; synthetic observations do not establish a real-world loop closure.",
        source_mode=config["source_mode"], stm_size=stm_size, constraints=closure_edges,
        latest_edge_type_counts=dict(Counter(e["type"] for e in latest["edges"])) if latest else {})
    # PointCloud2/Grid have no graph revision field. Their common reference stamp
    # is useful only when it maps to one graph revision and completed MapStatus.
    graphs_by_stamp = defaultdict(list)
    for graph in recorder.graphs.values():
        graphs_by_stamp[graph["stamp_ns"]].append(graph)
    map_matches, bad_status = [], []
    for event in recorder.statuses:
        status = event["value"]
        if status["source_mode"] != config["source_mode"] or status["navigation_validated"]:
            bad_status.append(dict(elapsed_s=event["elapsed_s"], reason="source label mismatch or unsolicited navigation validation"))
        if not status["consistent_snapshot"]:
            continue
        graph = recorder.graphs.get(status["completed_map_revision"])
        if (status["graph_revision"] != status["completed_map_revision"] or status["rebuilding"]
                or status["graph_epoch"] != config["graph_epoch"]):
            bad_status.append(dict(elapsed_s=event["elapsed_s"], reason="inconsistent completed map status"))
            continue
        if graph is None:
            continue
        reference = graph["stamp_ns"]
        cloud, grid = recorder.clouds.get(reference), recorder.grids.get(reference)
        if (len(graphs_by_stamp[reference]) == 1 and cloud and grid and cloud["count"] > 0
                and grid["occupied_cells"] > 0 and grid["cell_count"] == grid["width"] * grid["height"]
                and cloud["frame_id"] == grid["frame_id"] == graph["frame_id"]):
            if not any(match["revision"] == graph["revision"] for match in map_matches):
                map_matches.append(dict(revision=graph["revision"], stamp_ns=reference, cloud_points=cloud["count"],
                    occupied_cells=grid["occupied_cells"], free_cells=grid["free_cells"], unknown_cells=grid["unknown_cells"]))
    checks["map_revision_consistency"] = result("FAIL" if bad_status else "PASS" if latest and any(m["revision"] == latest["revision"] for m in map_matches) else "BLOCKED",
        "Final graph revision must have a unique reference timestamp shared by actual nonempty map cloud/grid and a completed consistent MapStatus. Status header time is not used as graph time.",
        matched_revisions=map_matches, invalid_statuses=bad_status, message_revision_fields="PointCloud2/Grid do not contain revision IDs; correlation is conditional on unique observed graph reference stamps.")
    checks["tf_publishers"] = tf_checks(recorder)
    truth_path = Path(config["session_root"]) / "synthetic_truth.json"
    metrics = dict(icp=result("BLOCKED", "No recorded synthetic truth file."), optimized_graph=result("BLOCKED", "No recorded synthetic truth file."))
    truth_evidence = None
    if config["source_mode"] == "synthetic" and truth_path.exists():
        try:
            document = read_json(truth_path)
            if document["source_mode"] != "synthetic":
                raise ValueError("truth source_mode mismatch")
            rows = document["poses"]
            truth = {int(row["stamp_ns"]): valid_matrix(row["T_map_rig"]) for row in rows}
            if len(truth) != len(rows) or len({int(row["frame_sequence"]) for row in rows}) != len(rows):
                raise ValueError("truth contains duplicate stamps or frame IDs")
            if (sorted(int(row["frame_sequence"]) for row in rows) != list(range(1, len(rows) + 1))
                    or document.get("seed") != config["synthetic"]["seed"]):
                raise ValueError("truth frame sequence or fixture seed disagrees with session configuration")
            truth_evidence = dict(path=str(truth_path), sha256=file_sha(truth_path), pose_count=len(rows),
                                  source_mode=document["source_mode"], seed=document.get("seed"), observations_model=document.get("observations_model"))
            metrics["icp"] = trajectory_metrics({t: matrix(odom["pose_xyz_xyzw"]) for t, odom in recorder.odometry.items()}, truth)
            graph_poses = {}
            if latest:
                for node in latest["nodes"]:
                    accepted = recorder.accepted.get(node["bundle_id"])
                    if accepted:
                        graph_poses[accepted["bundle"]["stamp_ns"]] = matrix(node["pose_xyz_xyzw"])
            metrics["optimized_graph"] = trajectory_metrics(graph_poses, truth)
            checks["truth_coverage"] = result("PASS" if expected > 0 and len(truth) == expected and metrics["icp"].get("matched_count") == expected and metrics["optimized_graph"].get("matched_count") == expected else "BLOCKED",
                "Actual ICP and final optimized graph both require exact timestamps for every recorded truth pose.")
        except Exception as failure:
            checks["truth_coverage"] = result("FAIL", str(failure))
    else:
        checks["truth_coverage"] = result("BLOCKED", "Recorded synthetic truth is unavailable; no generated substitute is used.")
    recorder.close_result()
    if recorder.close["requested"]:
        if recorder.close["status"] == "PASS":
            try:
                barrier = read_json(Path(config["session_root"]) / "graph" / "closed_snapshot.json")
                if (barrier["session_id"] != config["session_id"] or barrier["graph_epoch"] != config["graph_epoch"]
                        or barrier["state"] != "CLOSED_FOR_SNAPSHOT" or not latest or barrier["revision"] != latest["revision"]):
                    raise ValueError("closed snapshot barrier does not match final observed session graph")
                for path_key, hash_key in (("graph_file", "graph_sha256"), ("database_file", "database_sha256")):
                    relative = Path(barrier[path_key])
                    if relative.is_absolute() or ".." in relative.parts:
                        raise ValueError("unsafe closed snapshot member")
                    if file_sha(Path(config["session_root"]) / relative) != barrier[hash_key]:
                        raise ValueError("closed snapshot member SHA256 mismatch: " + path_key)
                recorder.close["barrier"] = barrier
            except Exception as failure:
                recorder.close.update(status="FAIL", detail="Barrier verification failed: " + str(failure))
        checks["close_snapshot"] = recorder.close
    checks["observer_integrity"] = result("FAIL" if recorder.errors or recorder.foreign else "BLOCKED" if recorder.interrupted else "PASS",
        "Callback errors, conflicting duplicates and foreign session traffic are retained as evidence.", errors=recorder.errors,
        foreign_session_message_counts=dict(recorder.foreign), interrupted=recorder.interrupted)
    flat_status = [value["status"] for key, value in checks.items() if key != "tf_publishers"]
    flat_status.extend(value["status"] for value in checks["tf_publishers"].values())
    overall = "FAIL" if "FAIL" in flat_status else "PASS" if all(s == "PASS" for s in flat_status) else "BLOCKED"
    return dict(schema_version=1, test="actual_ros_dual_icp_graph_map_observer", status=overall,
        verification_level="SYNTHETIC" if config["source_mode"] == "synthetic" else "LIVE_UNCLASSIFIED",
        session_id=config["session_id"], source_mode=config["source_mode"], elapsed_s=recorder.elapsed(),
        runtime=dict(hostname=socket.gethostname(), user=os.environ.get("USER"), cwd=str(Path.cwd()),
                     ros_domain_id=os.environ.get("ROS_DOMAIN_ID", "0 (environment unset)"), rmw_implementation=os.environ.get("RMW_IMPLEMENTATION")),
        session_config_sha256=config["_observer_config_sha256"], message_counts=dict(recorder.counts),
        topic_names=TOPICS, endpoint_evidence=dict(recorder.endpoints), discovery_errors=recorder.discovery_errors,
        checks=checks, trajectory_metrics=metrics, truth_evidence=truth_evidence,
        native_tf_evidence=recorder.native_tf_summary,
        native_graph_latest=latest, graph_revision_summary=[dict(revision=g["revision"], stamp_ns=g["stamp_ns"], nodes=len(g["nodes"]), edges=len(g["edges"])) for _, g in sorted(recorder.graphs.items())],
        map_status_history=recorder.statuses, map_clouds=list(recorder.clouds.values()), map_grids=list(recorder.grids.values()),
        tf_observations=dict(recorder.tf), odometry_observations=list(recorder.odometry.values()), close_snapshot=recorder.close,
        separate_acceptance=dict(per_sensor_ablation=result("NOT_RUN", "This observer does not run left-only/right-only/dual ablation."),
            numerical_accuracy=result("NOT_RUN", "Metrics are measured; no predeclared accuracy threshold exists for this fixture."),
            real_devices=result("NOT_RUN", "Synthetic pipeline evidence does not establish real device, timing or calibration performance."),
            real_dynamic_loop=result("NOT_RUN", "No physical movement or real-world loop is performed by this program."),
            rviz_visual_review=result("NOT_RUN", "Receiving map messages does not establish successful RViz rendering or map quality.")),
        navigation_validated=False)


def write_report(path, report):
    path = safe_path(path, existing=False)
    if path.exists():
        raise ValueError("existing evidence is immutable; choose a new --output path")
    path.parent.mkdir(parents=True, exist_ok=True)
    safe_path(path.parent)
    temporary = path.parent / ("." + path.name + "." + str(os.getpid()) + ".tmp")
    owns_temporary = False
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            owns_temporary = True
            json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)  # Linux same-filesystem NOREPLACE publication.
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if owns_temporary and temporary.exists():
            temporary.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session-config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--tf-evidence", type=Path, help="Optional immutable native tf_ownership_probe JSON from this exact session; must finish before this observer ends.")
    parser.add_argument("--close-snapshot", action="store_true", help="After every expected synthetic frame reaches the graph, close ONLY its proven software graph service and verify the immutable barrier.")
    args, ros_args = parser.parse_known_args(argv)
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 600:
        parser.error("--duration must be finite and between 1 and 600 seconds")
    config_path = safe_path(args.session_config)
    config = read_json(config_path)
    for key in ("session_id", "session_root", "graph_epoch", "source_mode"):
        if not isinstance(config.get(key), str) or not config[key]:
            parser.error("session config requires nonempty string " + key)
    safe_path(config["session_root"])
    if args.close_snapshot and (config["source_mode"] != "synthetic" or config.get("execution_mode") != "synthetic"):
        parser.error("this fixture observer's --close-snapshot is restricted to explicit synthetic software sessions")
    safe_path(args.output, existing=False)
    if args.output.exists():
        parser.error("--output already exists; preserve evidence and choose a new path")
    config["_observer_config_sha256"] = file_sha(config_path)
    if args.tf_evidence:
        config["_observer_tf_evidence"] = str(safe_path(args.tf_evidence, existing=False))
    recorder = Recorder(config, args.close_snapshot)
    node = None
    initialized = False
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry, OccupancyGrid
        from sensor_msgs.msg import PointCloud2
        from tf2_msgs.msg import TFMessage
        from std_srvs.srv import Trigger
        from wc_interfaces.msg import FusionBundle, AcceptedBundle, GraphSnapshot, MapStatus
        rclpy.init(args=ros_args)
        initialized = True
        node = Node("wc_icp_pipeline_observer_" + str(os.getpid()))
        reliable = QoSProfile(depth=128, reliability=ReliabilityPolicy.RELIABLE)
        durable = QoSProfile(depth=128, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        best_effort = QoSProfile(depth=512, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions = []
        for name, message_type, handler, qos in (
            ("bundle", FusionBundle, recorder.bundle, reliable),
            ("accepted", AcceptedBundle, recorder.accept, reliable),
            ("odom", Odometry, recorder.odom, best_effort),
            ("graph", GraphSnapshot, recorder.graph, durable),
            ("status", MapStatus, recorder.status, durable),
            ("cloud", PointCloud2, recorder.cloud, durable),
            ("grid", OccupancyGrid, recorder.grid, durable),
            ("tf", TFMessage, recorder.transforms, best_effort),
            ("tf_static", TFMessage, lambda m, g: recorder.transforms(m, g, static=True), durable),
        ):
            subscriptions.append(node.create_subscription(message_type, TOPICS[name], recorder.callback(name, handler), qos))
        client = node.create_client(Trigger, CLOSE_SERVICE) if args.close_snapshot else None
        print("OBSERVER_READY " + json.dumps(dict(session_id=config["session_id"], output=str(args.output), duration_s=args.duration)), flush=True)
        deadline, discovery_at = time.monotonic() + args.duration, 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= discovery_at:
                recorder.discover(node)
                discovery_at = time.monotonic() + 0.5
                if client:
                    recorder.maybe_close(node, client)
            recorder.close_result()
        recorder.discover(node)
    except KeyboardInterrupt:
        recorder.interrupted = True
    except Exception as failure:
        recorder.problem("observer_runtime", str(failure))
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
    try:
        report = build_report(recorder)
    except Exception as failure:
        recorder.problem("evidence_assembly", str(failure))
        report = dict(schema_version=1, test="actual_ros_dual_icp_graph_map_observer", status="FAIL",
                      session_id=config["session_id"], source_mode=config["source_mode"],
                      message_counts=dict(recorder.counts), errors=recorder.errors, navigation_validated=False)
    write_report(args.output, report)
    print(json.dumps(dict(status=report["status"], evidence=str(args.output), message_counts=dict(recorder.counts))), flush=True)
    return 0 if report["status"] == "PASS" else 2 if any(e["kind"] == "observer_runtime" for e in recorder.errors) else 1


if __name__ == "__main__":
    raise SystemExit(main())
