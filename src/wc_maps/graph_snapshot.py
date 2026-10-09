"""Join real RTAB-Map graph revisions with exact accepted raw observations.

Live revision rebuilding never opens the database. Final snapshot assembly
requires the C++ graph owner's immutable close/database barrier.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import sys
import tempfile

import numpy as np

from .builder import MapError, build_map, integer, rigid
from .store import (_hash, _json, _no_symlinks, _read_bytes, _rename_no_replace,
                    _snapshot_validate, _sqlite_backup, _sync_directory, _write_json)


def _relative(root: Path, value: str) -> Path:
    candidate = Path(value)
    if candidate.is_absolute() or ".." in candidate.parts or "\\" in value or ":" in value:
        raise MapError("unsafe path in graph snapshot")
    return _no_symlinks(root / candidate)


def revision_path(session_root, revision: int) -> Path:
    integer(revision, "graph revision")
    return _no_symlinks(Path(session_root) / "graph" / "revisions" / f"r{revision:020d}.json")


def rebuild_revision(session_root, graph: dict, config: dict, calibration: dict) -> dict:
    """Validate every graph/raw association and rebuild one immutable revision."""
    root = _no_symlinks(Path(session_root))
    for key in ("session_id", "graph_epoch", "source_mode", "revision", "frame_id", "calibration_id", "time_model_id",
                "left_clock_model_id", "right_clock_model_id", "raw_index_complete", "raw_index_hash", "nodes", "edges"):
        if key not in graph:
            raise MapError(f"graph revision missing {key}")
    if graph["raw_index_complete"] is not True or graph["source_mode"] not in ("synthetic", "real"):
        raise MapError("graph raw index/source mode is invalid")
    if calibration.get("calibration_id") != graph["calibration_id"]:
        raise MapError("calibration file does not match accepted graph calibration ID")
    if calibration.get("source_mode", graph["source_mode"]) != graph["source_mode"]:
        raise MapError("calibration and graph source modes disagree")
    observations = []
    seen_ids = set()
    source_index = []
    total_points = 0
    for node in sorted(graph["nodes"], key=lambda item: item["node_id"]):
        node_id = integer(node.get("node_id"), "node_id")
        if node_id < 1 or node_id > 2**31 - 1 or node_id != node.get("bundle_id") or node_id in seen_ids:
            raise MapError("graph node/bundle identity is invalid or reused")
        seen_ids.add(node_id)
        if node.get("raw_file") != f"raw_observations/{node_id}.json":
            raise MapError("graph raw filename does not match explicit bundle ID")
        path = _relative(root, node["raw_file"])
        if path.stat().st_size > config.get("max_raw_file_bytes", 128 * 1024 * 1024):
            raise MapError("raw observation archive exceeds configured file-size bound")
        raw_bytes = _read_bytes(path)
        checksum = hashlib.sha256(raw_bytes).hexdigest()
        if checksum != node.get("raw_file_sha256"):
            raise MapError("raw observation archive hash differs from accepted graph")
        source_index.append(f"{node_id}\t{checksum}\n")
        raw = json.loads(raw_bytes.decode("utf-8"), parse_constant=lambda value: (_ for _ in ()).throw(MapError(f"nonfinite raw JSON: {value}")))
        for key, expected in (("schema_version", 1), ("session_id", graph["session_id"]), ("source_mode", graph["source_mode"]),
                              ("sensor_mode", "dual"), ("bundle_id", node_id), ("t_ref_ns", node["stamp_ns"]),
                              ("calibration_id", graph["calibration_id"]), ("time_model_id", graph["time_model_id"])):
            if raw.get(key) != expected:
                raise MapError(f"raw archive {key} does not match the accepted graph")
        frames = raw.get("observations")
        if not isinstance(frames, list) or len(frames) != 2 or {frame.get("side") for frame in frames} != {"left", "right"}:
            raise MapError("each graph node requires exactly one original frame per side")
        if len(node.get("raw_keys", [])) != 2 or len(node.get("t_node_sensor", [])) != 2:
            raise MapError("graph node lacks two sensor origins and raw keys")
        by_side = {frame["side"]: frame for frame in frames}
        for index, side in enumerate(("left", "right")):
            frame = by_side[side]
            if frame.get("raw_key") != node["raw_keys"][index]:
                raise MapError("accepted raw key differs from archived original frame")
            T_ref_sensor = rigid(frame.get("T_ref_sensor"), "raw T_ref_sensor")
            T_node_sensor = rigid(node["t_node_sensor"][index], "graph T_node_sensor")
            # RTAB-Map's Transform uses float32. This compares serialization
            # precision only, not a physical calibration acceptance tolerance.
            if not np.allclose(T_ref_sensor, T_node_sensor, atol=1e-6, rtol=0):
                raise MapError("archived original sensor origin differs from accepted graph transform")
            points = frame.get("points")
            if not isinstance(points, list) or len(points) != frame.get("valid_count"):
                raise MapError("raw point count differs from valid_count")
            total_points += len(points)
            if total_points > config.get("max_points", 5000000):
                raise MapError("raw graph points exceed configured resource bound")
            if frame.get("origin_in_ref") is not None and not np.allclose(frame["origin_in_ref"], T_ref_sensor[:3, 3], atol=1e-8, rtol=0):
                raise MapError("raw sensor origin metadata disagrees with T_ref_sensor")
            observations.append({"raw_key": frame["raw_key"], "sensor_id": frame["sensor_id"], "side": side,
                                 "session_id": graph["session_id"], "graph_epoch": graph["graph_epoch"],
                                 "node_id": node_id, "odom_epoch": node["odom_epoch"], "points": points,
                                 "T_node_sensor": T_ref_sensor.tolist(), "stamp_ns": frame.get("stamp_ns"),
                                 "source_archive_sha256": checksum})
    if hashlib.sha256("".join(source_index).encode("ascii")).hexdigest() != graph["raw_index_hash"]:
        raise MapError("complete graph/raw index SHA256 mismatch")
    config = dict(config, sensor_mode="dual")
    mapped = build_map(graph, {"observations": observations}, config)
    trajectory = {"session_id": graph["session_id"], "graph_epoch": graph["graph_epoch"], "graph_revision": graph["revision"],
                  "body_frame_id": "rig_link", "poses": [{"node_id": node["node_id"], "T_map_rig": node["T_map_node"]}
                                                         for node in sorted(graph["nodes"], key=lambda item: item["node_id"])]}
    return {"graph": graph, "observations": {"observations": observations}, "trajectory": trajectory, "mapped": mapped,
            "config": config, "calibration": calibration}


def rebuild_message_revision(session_root, message, config: dict, calibration: dict) -> dict:
    """Validate GraphSnapshot metadata against its exact immutable disk revision."""
    graph = _json(revision_path(session_root, int(message.revision)))
    for name in ("session_id", "graph_epoch", "source_mode", "calibration_id", "left_clock_model_id", "right_clock_model_id", "revision", "raw_index_hash", "raw_index_complete"):
        if getattr(message, name) != graph[name]:
            raise MapError(f"ROS GraphSnapshot {name} disagrees with immutable revision")
    if message.header.frame_id != graph["frame_id"] or len(message.nodes) != len(graph["nodes"]):
        raise MapError("ROS GraphSnapshot frame/node count differs from immutable revision")
    if not 0 <= message.header.stamp.nanosec < 1000000000 or message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec != graph["stamp_ns"]:
        raise MapError("ROS GraphSnapshot timestamp differs from immutable revision")

    def ros_matrix(translation, rotation):
        quaternion = np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=float)
        if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion) - 1) > 1e-5:
            raise MapError("ROS GraphSnapshot quaternion is invalid")
        x, y, z, w = quaternion / np.linalg.norm(quaternion)
        matrix = np.eye(4)
        matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                          [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                          [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
        matrix[:3, 3] = [translation.x, translation.y, translation.z]
        return rigid(matrix, "ROS GraphSnapshot pose")
    nodes = {node["node_id"]: node for node in graph["nodes"]}
    if len({node.node_id for node in message.nodes}) != len(nodes):
        raise MapError("ROS GraphSnapshot has duplicate node IDs")
    for node in message.nodes:
        record = nodes.get(node.node_id)
        if record is None or node.bundle_id != record["bundle_id"] or node.odom_epoch != record["odom_epoch"] or list(node.raw_keys) != record["raw_keys"]:
            raise MapError("ROS GraphSnapshot node identity/association mismatch")
        if not np.allclose(ros_matrix(node.optimized_pose.position, node.optimized_pose.orientation), record["T_map_node"], atol=1e-6, rtol=0):
            raise MapError("ROS optimized pose differs from immutable graph revision")
        if len(node.t_node_sensor) != 2:
            raise MapError("ROS graph node lacks two original sensor transforms")
        for index, transform in enumerate(node.t_node_sensor):
            if not np.allclose(ros_matrix(transform.translation, transform.rotation), record["t_node_sensor"][index], atol=1e-6, rtol=0):
                raise MapError("ROS graph sensor transform differs from immutable revision")
    expected_edges = sorted((edge["from_id"], edge["to_id"], edge["type"], edge["synthetic"]) for edge in graph["edges"])
    actual_edges = sorted((edge.from_id, edge.to_id, edge.type, edge.synthetic) for edge in message.edges)
    if actual_edges != expected_edges:
        raise MapError("ROS graph constraint types/identities differ from immutable revision")
    return rebuild_revision(session_root, graph, config, calibration)


def assemble_closed_snapshot(session_root, output_dir, config: dict, calibration: dict) -> dict:
    """Create a new strict snapshot only after the unique graph owner closed DB."""
    root = _no_symlinks(Path(session_root))
    output = _no_symlinks(Path(output_dir))
    if root not in output.parents or output.exists():
        raise MapError("snapshot output must be a new directory inside this session")
    marker_path = root / "graph" / "closed_snapshot.json"
    marker_bytes = _read_bytes(marker_path)
    marker = _json(marker_path)
    if marker.get("schema_version") != 1 or marker.get("state") != "CLOSED_FOR_SNAPSHOT":
        raise MapError("graph writer has not produced a closed database snapshot barrier")
    graph_path = _relative(root, marker["graph_file"])
    database_path = _relative(root, marker["database_file"])
    if marker["database_file"] != "slam/rtabmap.db" or _hash(graph_path)["sha256"] != marker["graph_sha256"]:
        raise MapError("closed graph path/hash mismatch")
    if _hash(database_path)["sha256"] != marker["database_sha256"]:
        raise MapError("database changed after the graph snapshot barrier")
    graph = _json(graph_path)
    for field in ("session_id", "graph_epoch", "revision", "raw_index_hash"):
        if marker.get(field) != graph.get(field):
            raise MapError(f"closed database/graph barrier {field} mismatch")
    reconstructed = rebuild_revision(root, graph, config, calibration)
    snapshot = {"schema_version": 1, "session_id": graph["session_id"], "source_mode": graph["source_mode"],
                "sensor_mode": "dual", "graph_epoch": graph["graph_epoch"], "graph_revision": graph["revision"],
                "completed_map_revision": reconstructed["mapped"]["revision"], "frame_id": graph["frame_id"],
                "ground_valid": config["ground_valid"], "calibration_id": graph["calibration_id"], "time_model_id": graph["time_model_id"],
                "rtabmap_version": graph.get("rtabmap_version"), "left_clock_model_id": graph["left_clock_model_id"],
                "right_clock_model_id": graph["right_clock_model_id"], "closed_graph_sha256": marker["graph_sha256"],
                "closed_database_sha256": marker["database_sha256"], "graph_edge_noise_model": graph.get("graph_edge_noise_model"),
                "navigation_validated": False}
    final_graph = copy.deepcopy(graph)
    final_graph["database_closed_at_graph_publication"] = final_graph.get("database_closed")
    final_graph["database_closed"] = True
    final_graph["closed_snapshot_barrier"] = marker
    inputs = {"snapshot.json": snapshot, "graph.json": final_graph, "observations.json": reconstructed["observations"],
              "trajectory.json": reconstructed["trajectory"], "config.json": reconstructed["config"], "calibration.json": calibration}
    _snapshot_validate(inputs)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".assembling-", dir=output.parent))
    try:
        for name, content in inputs.items():
            _write_json(temporary / name, content)
        _sqlite_backup(database_path, temporary / "slam" / "rtabmap.db")
        if _read_bytes(marker_path) != marker_bytes or _hash(database_path)["sha256"] != marker["database_sha256"]:
            raise MapError("closed source database/barrier changed during snapshot assembly")
        for node in graph["nodes"]:
            if _hash(_relative(root, node["raw_file"]))["sha256"] != node["raw_file_sha256"]:
                raise MapError("raw archive changed while assembling snapshot")
        _write_json(temporary / "rebuild_summary.json", {"status": "COMPLETE", "revision": graph["revision"],
            "source_mode": graph["source_mode"], "source_point_counts": reconstructed["mapped"]["source_point_counts"],
            "voxel_count": len(reconstructed["mapped"]["occupancy"]["voxels"]), "navigation_validated": False})
        _sync_directory(temporary / "slam")
        _sync_directory(temporary)
        _rename_no_replace(temporary, output)
        _sync_directory(output.parent)
        return {"path": str(output), "snapshot": snapshot, "source_point_counts": reconstructed["mapped"]["source_point_counts"]}
    finally:
        if temporary.exists():
            checked = _no_symlinks(temporary)
            if checked.parent != output.parent or not checked.name.startswith(".assembling-"):
                raise MapError("refusing snapshot cleanup outside this invocation's staging directory")
            shutil.rmtree(checked)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Assemble an immutable offline snapshot after RTAB-Map closes its database")
    parser.add_argument("--session-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--map-config", required=True)
    parser.add_argument("--calibration", required=True)
    args = parser.parse_args(argv)
    try:
        result = assemble_closed_snapshot(args.session_root, args.output, _json(Path(args.map_config)), _json(Path(args.calibration)))
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
        return 0
    except (MapError, ValueError, KeyError, TypeError, OSError, sqlite3.Error) as error:
        print(json.dumps({"status": "ERROR", "error": str(error)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
