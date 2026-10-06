#!/usr/bin/env python3
"""Exercise persistence against a CLOSED_FOR_SNAPSHOT synthetic ROS session.

Run on the authorized Orin after check_icp_pipeline.py has closed its software
graph and the session's software processes have stopped. The mutable frontend
status cache is also hashed; a still-running writer correctly fails preservation.
This program does not start ROS, publish messages, close a live database,
or alter input observations. It reserves one NEW map ID, assembles the actual
closed graph, and saves v1/v2 from that SAME graph revision. Versioning is a
storage lifecycle test; a second graph revision is never fabricated.

  PYTHONPATH=src PYTHONNOUSERSITE=1 python3 \
    tests/integration/check_saved_graph_map.py \
    --session-config /home/nvidia/wheelchair/data/synthetic/SESSION/session.json \
    --map-id synthetic_saved_SESSION \
    --output /home/nvidia/wheelchair/reports/integration/SESSION_saved.json

Artifacts are retained on failure. The output is atomic and cannot replace an
existing report. Exit 0 means this synthetic persistence test passed; exit 1
means a check failed, and exit 2 means prerequisites were unavailable. Neither
success nor failure claims physical accuracy, navigation, or RViz review.
"""

import argparse
from collections import Counter
import getpass
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shlex
import socket
import sqlite3
import stat
import struct
import sys
import tempfile
import time
import traceback

import numpy as np

from wc_maps.builder import MapError, rigid
from wc_maps.graph_snapshot import assemble_closed_snapshot
from wc_maps.store import MapStore


PROJECT = Path(__file__).absolute().parents[2]
NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")


class PrerequisiteError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def safe_path(value, *, exists=True, within=None):
    path = Path(value)
    if not path.is_absolute() or ".." in path.parts:
        raise PrerequisiteError("require an absolute path without traversal: " + str(path))
    for part in (path, *path.parents):
        if part.is_symlink():
            raise PrerequisiteError("symbolic links are forbidden: " + str(part))
    if within is not None and not path.is_relative_to(within):
        raise PrerequisiteError("path is outside the allowed directory: " + str(path))
    if exists and not path.exists():
        raise PrerequisiteError("required input does not exist: " + str(path))
    return path


def regular_open(path, mode="rb"):
    path = safe_path(path)
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            raise PrerequisiteError("input must be a regular file: " + str(path))
        stream = os.fdopen(descriptor, mode, **({"encoding": "utf-8"} if mode == "r" else {}))
    except BaseException:
        os.close(descriptor)
        raise
    return stream


def read_json(path):
    with regular_open(path, "r") as stream:
        return json.load(stream, parse_constant=lambda value: (_ for _ in ()).throw(
            ValueError("nonfinite JSON constant: " + value)))


def file_hash(path):
    digest = hashlib.sha256()
    with regular_open(path) as stream:
        before = os.fstat(stream.fileno())
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
        after = os.fstat(stream.fileno())
    require((before.st_ino, before.st_size, before.st_mtime_ns) ==
            (after.st_ino, after.st_size, after.st_mtime_ns), "file changed while hashing: " + str(path))
    return {"sha256": digest.hexdigest(), "bytes": before.st_size, "mtime_ns": before.st_mtime_ns}


def inventory(directory, *, omit=None):
    """Hash every regular file, rejecting redirects and unusual object types."""
    directory = safe_path(directory)
    result = {}
    for parent, dirs, files in os.walk(directory, followlinks=False):
        for name in list(dirs):
            child = safe_path(Path(parent) / name)
            require(child.is_dir(), "non-directory in directory inventory")
            if omit is not None and child == omit:
                dirs.remove(name)
        for name in sorted(files):
            path = safe_path(Path(parent) / name)
            result[path.relative_to(directory).as_posix()] = file_hash(path)
    return dict(sorted(result.items()))


def stable_inventory(label, directory, before, *, omit=None):
    after = inventory(directory, omit=omit)
    changed = sorted(name for name in set(before) | set(after) if before.get(name) != after.get(name))
    require(not changed, label + " changed: " + ", ".join(changed[:20]))
    return {"unchanged": True, "file_count": len(before), "files": before}


def atomic_report(path, report):
    path = safe_path(path, exists=False, within=PROJECT / "reports")
    if path.exists():
        raise FileExistsError("evidence already exists: " + str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    safe_path(path.parent)
    descriptor, name = tempfile.mkstemp(prefix=".saved-map-evidence-", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        safe_path(path, exists=False)
        os.link(temporary, path)  # Same filesystem, atomic and NOREPLACE.
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        temporary.unlink(missing_ok=True)  # Only this invocation's mkstemp file.


def canonical(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(",", ":"), allow_nan=False)


def quoted(identifier):
    return '"' + identifier.replace('"', '""') + '"'


def sqlite_evidence(path, graph):
    """Read-only integrity, native identities, and a full-table logical digest.

SQLite backup may change header bytes or journal mode. Compare every SQL table
as a multiset of typed row digests, plus its actual schema, instead of assuming
byte equality between databases. No schema version or scan column is invented.
"""
    path = safe_path(path)
    require(path.stat().st_size > 0, "empty RTABMap database")
    for suffix in ("-wal", "-journal"):
        sibling = safe_path(str(path) + suffix, exists=False)
        require(not sibling.exists() or sibling.stat().st_size == 0,
                "closed snapshot still has a nonempty journal: " + str(sibling))
    connection = sqlite3.connect(path.as_uri() + "?mode=ro&immutable=1", uri=True)
    try:
        connection.execute("PRAGMA query_only=ON")
        integrity = connection.execute("PRAGMA integrity_check").fetchall()
        require(integrity == [("ok",)], "SQLite integrity_check failed: " + repr(integrity))
        foreign = connection.execute("PRAGMA foreign_key_check").fetchall()
        require(not foreign, "SQLite declared foreign key integrity failed")
        schema = connection.execute(
            "SELECT type,name,tbl_name,sql FROM sqlite_master WHERE name NOT LIKE 'sqlite_%' ORDER BY type,name"
        ).fetchall()
        tables = {}
        columns = {}
        for kind, name, _, sql in schema:
            if kind != "table":
                continue
            info = connection.execute("PRAGMA table_info(" + quoted(name) + ")").fetchall()
            columns[name] = {row[1].lower(): row[1] for row in info}
            row_hashes = Counter()
            for row in connection.execute("SELECT * FROM " + quoted(name)):
                digest = hashlib.sha256()
                for value in row:
                    if value is None:
                        tag, data = b"n", b""
                    elif isinstance(value, bytes):
                        tag, data = b"b", value
                    elif isinstance(value, str):
                        tag, data = b"s", value.encode("utf-8")
                    elif isinstance(value, int):
                        tag, data = b"i", str(value).encode("ascii")
                    elif isinstance(value, float):
                        tag, data = b"f", struct.pack(">d", value)
                    else:
                        raise AssertionError("unexpected SQLite storage type")
                    digest.update(tag + struct.pack(">Q", len(data)) + data)
                row_hashes[digest.hexdigest()] += 1
            tables[name] = {"row_count": sum(row_hashes.values()), "columns": [row[1] for row in info],
                            "rows_sha256": hashlib.sha256(canonical(sorted(row_hashes.items())).encode()).hexdigest()}
        by_name = {name.lower(): name for name in tables}
        require("node" in by_name and "link" in by_name and "data" in by_name,
                "database lacks the actual native RTABMap Node/Link/Data tables")
        node_table, link_table, data_table = (by_name[name] for name in ("node", "link", "data"))
        require("id" in columns[node_table] and "id" in columns[data_table], "native node/data ID column unavailable")
        require({"from_id", "to_id", "type"}.issubset(columns[link_table]), "native constraint columns unavailable")
        node_ids = [row[0] for row in connection.execute(
            "SELECT " + quoted(columns[node_table]["id"]) + " FROM " + quoted(node_table))]
        expected_ids = sorted(node["node_id"] for node in graph["nodes"])
        require(sorted(node_ids) == expected_ids, "database Node IDs differ from the complete closed graph")
        data_ids = [row[0] for row in connection.execute(
            "SELECT " + quoted(columns[data_table]["id"]) + " FROM " + quoted(data_table))]
        require(sorted(data_ids) == expected_ids, "database Data rows do not retain every graph node")
        fields = ",".join(quoted(columns[link_table][key]) for key in ("from_id", "to_id", "type"))
        links = connection.execute("SELECT " + fields + " FROM " + quoted(link_table)).fetchall()
        native_edges = {(min(a, b), max(a, b), kind) for a, b, kind in links}
        graph_edges = {(min(edge["from_id"], edge["to_id"]), max(edge["from_id"], edge["to_id"]), edge["type_code"])
                       for edge in graph["edges"]}
        require(native_edges == graph_edges, "native database constraints and graph edges disagree")
        logical = {"schema": schema, "tables": tables}
        return {"path": str(path), "file": file_hash(path), "integrity_check": "ok", "foreign_key_violations": 0,
                "node_ids": expected_ids, "data_node_count": len(data_ids), "stored_link_rows": len(links),
                "undirected_constraint_count": len(native_edges), "tables": tables,
                "logical_sha256": hashlib.sha256(canonical(logical).encode()).hexdigest()}
    finally:
        connection.close()


def validate_raw_graph(root, graph, config, truth):
    count = config["synthetic"]["frame_count"]
    require(type(count) is int and count > 1, "invalid explicit synthetic frame_count")
    expected_ids = list(range(1, count + 1))
    nodes = sorted(graph["nodes"], key=lambda node: node["node_id"])
    require([node["node_id"] for node in nodes] == expected_ids, "closed graph does not contain every synthetic frame exactly once")
    require(truth.get("source_mode") == "synthetic", "truth source is not explicitly synthetic")
    truth_rows = truth["poses"]
    require(sorted(row["frame_sequence"] for row in truth_rows) == expected_ids, "truth archive is incomplete or duplicated")
    truth_stamps = {row["frame_sequence"]: row["stamp_ns"] for row in truth_rows}
    counts = {"left": 0, "right": 0}
    frames = []
    keys = set()
    index_digest = hashlib.sha256()
    for node in nodes:
        node_id = node["node_id"]
        require(node["bundle_id"] == node_id and node["stamp_ns"] == truth_stamps[node_id],
                "node/bundle ID or timestamp differs from actual fixture archive")
        require(node["raw_file"] == f"raw_observations/{node_id}.json", "raw filename is not the exact node ID")
        raw_path = safe_path(root / node["raw_file"], within=root)
        checksum = file_hash(raw_path)["sha256"]
        require(checksum == node["raw_file_sha256"], "raw archive SHA mismatch")
        index_digest.update(f"{node_id}\t{checksum}\n".encode("ascii"))
        raw = read_json(raw_path)
        for field, expected in (("schema_version", 1), ("source_mode", "synthetic"), ("sensor_mode", "dual"),
                                ("session_id", config["session_id"]), ("bundle_id", node_id),
                                ("t_ref_ns", node["stamp_ns"]), ("calibration_id", graph["calibration_id"]),
                                ("time_model_id", graph["time_model_id"])):
            require(raw.get(field) == expected, "raw archive identity mismatch: " + field)
        require(len(raw["observations"]) == 2 and {f["side"] for f in raw["observations"]} == {"left", "right"},
                "node is missing an independent original observation from one side")
        require(len(node["raw_keys"]) == 2 and len(node["t_node_sensor"]) == 2, "node lacks both raw associations/origins")
        by_side = {frame["side"]: frame for frame in raw["observations"]}
        pose = rigid(node["T_map_node"], "optimized native node pose")
        for index, side in enumerate(("left", "right")):
            frame = by_side[side]
            key = canonical(frame["raw_key"])
            require(key not in keys and frame["raw_key"] == node["raw_keys"][index], "raw key is reused or mismatched")
            keys.add(key)
            require(frame["sensor_id"] == config["sensor_ids"][side], "sensor identity changed across graph")
            points = np.asarray(frame["points"], dtype=np.float64)
            require(points.ndim == 2 and points.shape[1] == 3 and len(points) > 0 and np.isfinite(points).all(),
                    "raw points are not a nonempty finite Nx3 original observation")
            require(type(frame["valid_count"]) is int and len(points) == frame["valid_count"] <= frame["raw_count"],
                    "raw point counts are invalid")
            valid_indices = frame["valid_indices"]
            require(len(valid_indices) == len(points) and len(set(valid_indices)) == len(points)
                    and all(type(i) is int and 0 <= i < frame["raw_count"] for i in valid_indices),
                    "original valid point indices are incomplete or reused")
            origin = rigid(frame["T_ref_sensor"], "original T_ref_sensor")
            require(np.allclose(origin, node["t_node_sensor"][index], atol=1e-6, rtol=0),
                    "node sensor transform disagrees beyond RTABMap float serialization tolerance")
            require(np.allclose(origin[:3, 3], frame["origin_in_ref"], atol=1e-8, rtol=0), "original ray origin metadata mismatch")
            transform = pose @ origin
            mapped = points @ transform[:3, :3].T + transform[:3, 3]
            counts[side] += len(points)
            frames.append({"raw_key": frame["raw_key"], "node_id": node_id, "odom_epoch": node["odom_epoch"],
                           "session_id": config["session_id"], "graph_epoch": graph["graph_epoch"],
                           "side": side, "sensor_id": frame["sensor_id"], "point_count": len(points),
                           "T_node_sensor": origin.tolist(), "origin_map": transform[:3, 3].tolist(),
                           "mapped_points": mapped})
    require(graph.get("raw_index_complete") is True and index_digest.hexdigest() == graph["raw_index_hash"],
            "full graph raw index completeness/hash mismatch")
    require(counts["left"] > 0 and counts["right"] > 0, "one side has zero contribution")
    require(all(edge.get("synthetic") is True for edge in graph["edges"]), "native constraints are not labeled synthetic")
    require(all(edge["type"] != "user_closure" for edge in graph["edges"]), "user-injected graph closure is outside this test")
    require(all(type(edge.get("type_code")) is int for edge in graph["edges"]), "native edge type codes missing")
    stm = int(graph["parameters"]["Mem/STMSize"])
    require(stm > 0, "native short-term memory setting unavailable")
    closures = [edge for edge in graph["edges"] if edge["type"] in ("global_closure", "local_space_closure")
                and abs(edge["from_id"] - edge["to_id"]) > stm]
    require(closures, "no native geometric closure beyond the configured short-term memory window")
    return frames, {"node_ids": expected_ids, "original_frame_count": len(frames), "raw_key_count": len(keys),
                    "source_point_counts": counts, "native_edge_types": dict(Counter(e["type"] for e in graph["edges"])),
                    "nonadjacent_native_closures": closures, "native_stm_size": stm,
                    "raw_index_sha256": index_digest.hexdigest(), "rtabmap_version": graph["rtabmap_version"],
                    "accuracy_evaluation": "NOT_PERFORMED; truth timestamps associate observations only"}


def read_ply(path):
    with regular_open(path, "r") as stream:
        header = []
        for _ in range(32):
            line = stream.readline().rstrip("\n")
            header.append(line)
            if line == "end_header":
                break
        require(header[:2] == ["ply", "format ascii 1.0"] and header[-1] == "end_header", "unsupported/incomplete saved PLY")
        revisions = [int(line.split()[-1]) for line in header if line.startswith("comment graph_revision ")]
        counts = [int(line.split()[-1]) for line in header if line.startswith("element vertex ")]
        require(len(revisions) == len(counts) == 1, "PLY revision/count is ambiguous")
        require([line for line in header if line.startswith("property ")] ==
                ["property double x", "property double y", "property double z"], "PLY fields differ from supported XYZ")
        cloud = np.loadtxt(stream, dtype=np.float64, ndmin=2)
    require(cloud.shape == (counts[0], 3) and np.isfinite(cloud).all(), "PLY vertex payload/count mismatch")
    return revisions[0], cloud


def validate_reload(store, map_id, version, graph, frames, counts, profile):
    manifest = store.verify(map_id, version)
    payload = store.load(map_id, version)
    path = safe_path(payload["path"])
    revision = graph["revision"]
    require(payload["state"] == manifest["state"] == "LOADED_VIEW_ONLY" and payload["current_pose"] is None,
            "loading a saved map manufactured current localization")
    require(manifest["source_mode"] == payload["quality"]["source_mode"] == "synthetic"
            and manifest["sensor_mode"] == payload["quality"]["sensor_mode"] == "dual", "saved source labels changed")
    require(manifest["navigation_validated"] is False and payload["quality"]["navigation_validated"] is False,
            "synthetic saved map incorrectly claims navigation validation")
    require(manifest["map_id"] == map_id and manifest["version"] == version, "saved version binding changed")
    for value in (manifest["graph_revision"], manifest["completed_map_revision"], payload["graph"]["revision"],
                  payload["grid"]["revision"], payload["occupancy"]["revision"], payload["trajectory"]["graph_revision"]):
        require(value == revision, "saved/reloaded component belongs to a different graph revision")
    require(payload["graph"]["nodes"] == graph["nodes"] and payload["graph"]["edges"] == graph["edges"],
            "save changed native optimized nodes or constraints")
    require(payload["graph"]["database_closed"] is True, "saved graph lacks its closed database barrier")
    require(payload["quality"]["source_point_counts"] == counts, "saved map lost one side's original points")
    for side in ("left", "right"):
        require(payload["quality"]["sensor_ids"][side] == next(frame["sensor_id"] for frame in frames if frame["side"] == side),
                "saved quality associates a different sensor ID")
    ply_revision, cloud = read_ply(path / "cloud" / "map.ply")
    expected_cloud = np.concatenate([frame["mapped_points"] for frame in frames])
    require(ply_revision == revision and np.array_equal(cloud, expected_cloud),
            "reloaded PLY differs from every original endpoint transformed through the native optimized graph")
    raw_index = read_json(path / "raw_data_index.json")
    require((raw_index["session_id"], raw_index["graph_epoch"], raw_index["revision"]) ==
            (graph["session_id"], graph["graph_epoch"], revision), "saved raw index belongs to a different graph")
    expected_index = [{key: value for key, value in frame.items() if key not in ("origin_map", "mapped_points")} for frame in frames]
    require(raw_index["frames"] == expected_index, "saved raw frame/index association or counts changed")
    expected_origins = [{"raw_key": frame["raw_key"], "side": frame["side"], "origin_map": frame["origin_map"]} for frame in frames]
    require(raw_index["origins"] == expected_origins, "saved per-frame ray origins changed")
    trajectory = payload["trajectory"]
    require((trajectory["session_id"], trajectory["graph_epoch"], trajectory["body_frame_id"]) ==
            (graph["session_id"], graph["graph_epoch"], "rig_link"), "saved trajectory frame/session mismatch")
    nodes = sorted(graph["nodes"], key=lambda item: item["node_id"])
    require([pose["node_id"] for pose in trajectory["poses"]] == [node["node_id"] for node in nodes],
            "saved trajectory has missing, duplicated, or reordered graph poses")
    for pose, node in zip(trajectory["poses"], nodes):
        require(np.array_equal(rigid(pose["T_map_rig"], "loaded path pose"), np.asarray(node["T_map_node"])),
                "saved path was not taken from this optimized graph revision")
    occupancy = payload["occupancy"]
    resolution = float(profile["resolution"])
    require(occupancy["resolution"] == resolution and occupancy["format"] == "sparse_binary_evidence_v1",
            "saved sparse occupancy format/resolution mismatch")
    cells = {}
    for item in occupancy["voxels"]:
        index = tuple(item["index"])
        require(len(index) == 3 and all(type(i) is int for i in index) and index not in cells
                and type(item["state"]) is int and item["state"] in (0, 100), "invalid or duplicated saved voxel")
        cells[index] = item["state"]
    occupied = {index for index, state in cells.items() if state == 100}
    endpoint_voxels = {tuple(int(i) for i in row) for row in np.floor(cloud / resolution)}
    require(occupied == endpoint_voxels, "3D occupied evidence differs from all original optimized endpoints")
    require(0 < len(cells) <= profile.get("max_voxels", 4000000), "saved occupancy exceeds the global voxel bound")
    lower = [min(index[axis] for index in cells) for axis in (0, 1)]
    upper = [max(index[axis] for index in cells) for axis in (0, 1)]
    grid = payload["grid"]
    require(grid["frame_id"] == manifest["frame_id"] == graph["frame_id"] == "map", "saved grid/frame mismatch")
    require(grid["width"] == upper[0] - lower[0] + 1 and grid["height"] == upper[1] - lower[1] + 1
            and grid["resolution"] == resolution and grid["origin"] == [lower[0] * resolution, lower[1] * resolution, 0.0],
            "2D extent/origin/resolution differs from saved 3D evidence")
    require(grid["inflated"] is False and grid["row_order"] == "y_increasing_x_fastest", "unsupported grid layout/extra inflation")
    ground = float(profile["ground_z"])
    tolerance = float(profile["ground_tolerance"])
    support = {tuple(int(i) for i in voxel[:2]) for point, voxel in zip(cloud, np.floor(cloud / resolution))
               if abs(float(point[2]) - ground) <= tolerance}
    z_min = math.floor((ground + profile["collision_min_height"]) / resolution)
    z_max = math.ceil((ground + profile["collision_max_height"]) / resolution)
    expected_grid = []
    for y in range(lower[1], upper[1] + 1):
        for x in range(lower[0], upper[0] + 1):
            column = [cells.get((x, y, z)) for z in range(z_min, z_max)]
            expected_grid.append(100 if 100 in column else 0 if profile["ground_valid"] and
                                 (x, y) in support and all(state == 0 for state in column) else -1)
    require(grid["data"] == expected_grid, "2D projection cleared unknown/obstacle evidence or lost required ground support")
    width, height = grid["width"], grid["height"]
    with regular_open(path / "occupancy" / "map.pgm") as stream:
        require(stream.readline() == b"P5\n" and stream.readline().strip() == f"{width} {height}".encode()
                and stream.readline() == b"255\n", "saved PGM header/dimensions mismatch")
        expected_pixels = bytes({-1: 205, 0: 254, 100: 0}[expected_grid[y * width + x]]
                                for y in reversed(range(height)) for x in range(width))
        require(stream.read() == expected_pixels, "saved PGM pixel encoding or Y direction mismatch")
    with regular_open(path / "occupancy" / "map.yaml", "r") as stream:
        yaml_values = dict(line.strip().split(":", 1) for line in stream if line.strip())
    require(yaml_values["image"].strip() == "map.pgm" and float(yaml_values["resolution"]) == resolution
            and json.loads(yaml_values["origin"]) == grid["origin"] and yaml_values["mode"].strip() == "trinary"
            and int(yaml_values["negate"]) == 0 and float(yaml_values["occupied_thresh"]) == 0.65
            and float(yaml_values["free_thresh"]) == 0.196,
            "saved YAML does not bind the verified grid image/geometry")
    return {"path": str(path), "manifest": file_hash(path / "manifest.json"), "graph_revision": revision,
            "completed_map_revision": revision, "cloud_revision": ply_revision, "trajectory_revision": trajectory["graph_revision"],
            "grid_revision": grid["revision"], "occupancy_revision": occupancy["revision"],
            "cloud_point_count": len(cloud), "voxel_count": len(cells), "grid_cells": len(expected_grid),
            "grid_state_counts": {str(key): value for key, value in Counter(expected_grid).items()},
            "source_point_counts": counts, "raw_frames": len(frames), "trajectory_pose_count": len(nodes),
            "database": sqlite_evidence(path / "slam" / "rtabmap.db", graph), "view_only": True,
            "ros_publication": "NOT_TESTED_BY_THIS_PROGRAM", "navigation_validated": False,
            "viewer_command": shlex.join(["python3", "-m", "wc_maps.ros_view", "--root", str(store.root),
                                            "--map-id", map_id, "--version", version]),
            "artifacts": {name: str(path / name) for name in ("cloud/map.ply", "occupancy/voxels.json", "occupancy/grid.json",
                "occupancy/map.yaml", "occupancy/map.pgm", "trajectory.json", "graph.json", "raw_data_index.json", "slam/rtabmap.db")}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session-config", required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    report = {"schema_version": 1, "test": "actual_ros_synthetic_closed_graph_map_persistence", "status": "BLOCKED",
              "started_unix_ns": time.time_ns(), "source_mode": "synthetic", "execution_mode": "synthetic",
              "host": socket.gethostname(), "user": getpass.getuser(), "architecture": platform.machine(),
              "project_root": str(PROJECT), "map_id": args.map_id, "checks": [], "artifacts": {},
              "scope": "SYNTHETIC persistence/reload checks using archived native ICP/RTABMap output",
              "physical_validation": "NOT_PERFORMED", "accuracy_validation": "NOT_PERFORMED",
              "dynamic_hardware_validation": "NOT_PERFORMED", "rviz_visual_review": "NOT_PERFORMED",
              "navigation_validated": False,
              "versions_note": "v1 and v2 intentionally serialize the same closed graph revision; no graph optimization is fabricated"}
    output = None
    original = None
    assembled_inventory = None
    version_inventories = {}
    root = assembled = map_directory = None
    exit_code = 2

    def passed(name, **facts):
        report["checks"].append({"name": name, "status": "PASS", **facts})
        print(json.dumps({"check": name, "status": "PASS"}), flush=True)

    try:
        output = safe_path(args.output, exists=False, within=PROJECT / "reports")
        if output.exists() or output.suffix.lower() != ".json":
            output = None
            raise PrerequisiteError("--output must be a new absolute reports/*.json file")
        if (PROJECT != Path("/home/nvidia/wheelchair") or platform.machine() != "aarch64"
                or socket.gethostname() != "ubuntu" or getpass.getuser() != "nvidia"):
            raise PrerequisiteError("execute only on the verified ubuntu/nvidia/aarch64 Orin project")
        if NAME.fullmatch(args.map_id) is None:
            raise PrerequisiteError("map ID must be a simple ASCII identifier, 1..64 characters")
        config_path = safe_path(args.session_config, within=PROJECT)
        config = read_json(config_path)
        if config.get("source_mode") != "synthetic" or config.get("execution_mode") != "synthetic" or config.get("sensor_mode") != "dual":
            raise PrerequisiteError("this test requires explicit synthetic source/execution modes and dual sensors")
        root = safe_path(config["session_root"], within=PROJECT / "data" / "synthetic")
        require(root.is_dir() and config_path.is_relative_to(root), "session configuration is outside its declared synthetic session")
        require(isinstance(config["session_id"], str) and NAME.fullmatch(config["session_id"]), "invalid session identity")
        require(set(config["sensor_ids"]) == {"left", "right"} and len(set(config["sensor_ids"].values())) == 2,
                "two distinct synthetic sensor IDs are required")
        report["session_id"] = config["session_id"]
        report["session_config"] = str(config_path)
        report["session_root"] = str(root)
        assembled = safe_path(root / "assembled_snapshot", exists=False)
        map_directory = safe_path(PROJECT / "maps" / args.map_id, exists=False)
        annotations = safe_path(PROJECT / "maps" / ".annotations" / args.map_id, exists=False)
        if assembled.exists() or map_directory.exists() or annotations.exists():
            raise PrerequisiteError("assembly or map/annotation ID already exists; keep it and select a new synthetic session/map ID")
        marker_path = safe_path(root / "graph" / "closed_snapshot.json")
        marker = read_json(marker_path)
        if marker.get("schema_version") != 1 or marker.get("state") != "CLOSED_FOR_SNAPSHOT":
            raise PrerequisiteError("the session owner has not produced its CLOSED_FOR_SNAPSHOT barrier")
        graph_path = safe_path(root / marker["graph_file"], within=root / "graph" / "revisions")
        require(marker["database_file"] == "slam/rtabmap.db", "barrier points to an unexpected database")
        source_database = safe_path(root / marker["database_file"], within=root)
        graph = read_json(graph_path)
        require(graph["source_mode"] == "synthetic" and graph["sensor_mode"] == "dual", "closed graph labels differ from synthetic session")
        for name in ("session_id", "graph_epoch"):
            require(marker[name] == graph[name] == config[name], "barrier/graph/session mismatch: " + name)
        for name in ("revision", "raw_index_hash"):
            require(marker[name] == graph[name], "barrier/graph mismatch: " + name)
        require(marker["graph_sha256"] == file_hash(graph_path)["sha256"]
                and marker["database_sha256"] == file_hash(source_database)["sha256"], "closed barrier source hashes mismatch")
        profile_path = safe_path(root / "map_config.json")
        calibration_path = safe_path(root / "calibration.json")
        profile, calibration = read_json(profile_path), read_json(calibration_path)
        require(calibration == config["calibration"] and calibration.get("source_mode") == "synthetic"
                and calibration["calibration_id"] == graph["calibration_id"]
                and calibration["sensor_ids"] == config["sensor_ids"], "saved calibration differs from accepted synthetic calibration")
        require(graph["time_model_id"] == config["time_model_id"] and graph["left_clock_model_id"] == config["clock_model_ids"]["left"]
                and graph["right_clock_model_id"] == config["clock_model_ids"]["right"], "accepted synthetic clock IDs changed")
        require(profile.get("sensor_mode") == "dual" and isinstance(profile.get("ground_valid"), bool), "map profile must explicitly declare dual mode/ground status")
        original = inventory(root)
        frames, graph_evidence = validate_raw_graph(root, graph, config, read_json(root / "synthetic_truth.json"))
        report["graph"] = graph_evidence
        report["graph_epoch"], report["graph_revision"] = graph["graph_epoch"], graph["revision"]
        report["barrier"] = {"path": str(marker_path), **marker}
        report["source_database"] = sqlite_evidence(source_database, graph)
        passed("closed_native_graph_all_nodes_both_original_sensors", **{key: graph_evidence[key] for key in ("original_frame_count", "source_point_counts")})
        # Atomic mkdir reserves this previously unused business map ID. All
        # subsequent writes are through the real store or new assembly API.
        map_directory.parent.mkdir(parents=True, exist_ok=True)
        safe_path(map_directory.parent)
        map_directory.mkdir(exist_ok=False)
        report["artifacts"].update(assembled_snapshot=str(assembled), map_directory=str(map_directory))
        exit_code = 1
        started = time.monotonic()
        assembly = assemble_closed_snapshot(root, assembled, profile, calibration)
        report["assembly"] = {**assembly, "duration_s": time.monotonic() - started}
        require(assembly["snapshot"]["graph_revision"] == assembly["snapshot"]["completed_map_revision"] == graph["revision"],
                "assembly did not complete the closed graph revision")
        require(assembly["source_point_counts"] == graph_evidence["source_point_counts"], "assembly point counts differ from original archives")
        assembled_inventory = inventory(assembled)
        report["assembly_database"] = sqlite_evidence(assembled / "slam" / "rtabmap.db", graph)
        require(report["assembly_database"]["logical_sha256"] == report["source_database"]["logical_sha256"],
                "assembly SQLite backup changed logical database contents")
        passed("assembled_from_actual_closed_barrier", graph_revision=graph["revision"])
        store = MapStore(PROJECT / "maps")
        report["versions"] = {}
        for version in ("v1", "v2"):
            started = time.monotonic()
            saved = store.save_snapshot(assembled, args.map_id, version)
            version_path = safe_path(map_directory / version)
            require(Path(saved["path"]) == version_path, "store saved outside the reserved map/version")
            version_inventories[version] = inventory(version_path)
            evidence = validate_reload(store, args.map_id, version, graph, frames, graph_evidence["source_point_counts"], profile)
            require(evidence["database"]["logical_sha256"] == report["source_database"]["logical_sha256"],
                    "saved SQLite backup changed logical database contents")
            evidence["save_and_verify_duration_s"] = time.monotonic() - started
            report["versions"][version] = evidence
            passed("save_verify_reload_" + version, **{key: evidence[key] for key in ("cloud_point_count", "voxel_count", "trajectory_pose_count")})
            try:
                store.save_snapshot(assembled, args.map_id, version)
            except MapError as error:
                require("already exists" in str(error) and "immutable" in str(error),
                        "repeat save failed for a reason other than rejecting an immutable version")
                report["versions"][version]["overwrite_rejection"] = str(error)
            else:
                raise AssertionError("saving an existing map version unexpectedly succeeded")
            stable_inventory(version + " immutable package", version_path, version_inventories[version])
            passed("existing_" + version + "_overwrite_rejected")
        listed = store.list_versions(args.map_id)
        require([entry["version"] for entry in listed] == ["v1", "v2"] and
                all(entry["graph_revision"] == graph["revision"] for entry in listed), "list_versions contains missing/unexpected revisions")
        report["listed_versions"] = listed
        # This is a synthetic annotation at the first optimized pose, not an
        # approved destination. Never impersonate an operator review here.
        position = rigid(sorted(graph["nodes"], key=lambda node: node["node_id"])[0]["T_map_node"], "first optimized pose")[:3, 3].tolist()
        goal = store.set_goal(args.map_id, "v1", {"name": "SYNTHETIC_STORAGE_TEST_ONLY", "position": position,
                                                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]})
        require(goal["map_id"] == args.map_id and goal["version"] == "v1" and goal["review_state"] == "UNVERIFIED", "new goal is not bound/unverified")
        before_goals = store.get_goals(args.map_id, "v1")
        require(store.get_goals(args.map_id, "v2")["goals"] == [], "new version unexpectedly inherited mutable goals")
        migrated = store.migrate_goals(args.map_id, "v1", "v2")
        require(store.get_goals(args.map_id, "v1") == before_goals, "migration altered old version's annotations")
        require(migrated["map_id"] == args.map_id and migrated["version"] == "v2" and len(migrated["goals"]) == 1,
                "migration target identity/count mismatch")
        migrated_goal = migrated["goals"][0]
        require(migrated_goal["version"] == "v2" and migrated_goal["map_id"] == args.map_id
                and migrated_goal["frame_id"] == graph["frame_id"] and migrated_goal["position"] == goal["position"]
                and migrated_goal["orientation_xyzw"] == goal["orientation_xyzw"]
                and migrated_goal["review_state"] == "UNVERIFIED" and migrated_goal["review_reason"] == "map_version_changed"
                and migrated_goal["inherited_from"] == {"map_id": args.map_id, "version": "v1"}, "migrated goal binding/review state changed")
        require(store.load(args.map_id, "v1")["goals"] == before_goals and store.load(args.map_id, "v2")["goals"] == migrated,
                "reload did not select the requested version's external goal annotations")
        report["goals"] = {"v1": before_goals, "v2": migrated, "operator_review_performed": False,
                           "artifacts": {version: str(PROJECT / "maps" / ".annotations" / args.map_id / version / "goals.json") for version in ("v1", "v2")}}
        passed("goal_map_version_binding_and_migration_unverified")
        report["status"] = "PASS"
        exit_code = 0
    except PrerequisiteError as error:
        report["status"] = "BLOCKED"
        report["error"] = str(error)
        exit_code = 2
    except Exception as error:
        report["status"] = "FAIL"
        report["error"] = str(error)
        report["traceback"] = traceback.format_exc()
        exit_code = 1
    finally:
        # Run preservation checks even if a save or verification above failed.
        # New assembled_snapshot is the sole intentionally added session tree.
        for name, directory, baseline, omit in (
            [("original_session", root, original, assembled), ("assembled_snapshot", assembled, assembled_inventory, None)] +
            [("immutable_" + version, map_directory / version, data, None) for version, data in version_inventories.items()]
        ):
            if baseline is None:
                continue
            try:
                report[name + "_preservation"] = stable_inventory(name, directory, baseline, omit=omit)
                passed(name + "_unchanged")
            except Exception as error:
                report[name + "_preservation"] = {"unchanged": False, "error": str(error)}
                report["status"] = "FAIL"
                exit_code = 1
        report["finished_unix_ns"] = time.time_ns()
        if output is not None:
            try:
                atomic_report(output, report)
            except Exception as error:
                print(json.dumps({"status": "FAIL", "evidence_write_error": str(error), "requested_output": str(output)}), file=sys.stderr)
                report["status"] = "FAIL"
                exit_code = 1
        print(json.dumps({"status": report["status"], "output": str(output) if output is not None else None,
                          "artifacts": report["artifacts"], "error": report.get("error"),
                          "scope": report["scope"]}, ensure_ascii=False), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
