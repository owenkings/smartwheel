"""Hash-checked reading of a frozen single-lidar frontend cell (left or right).

A cell is ``.../robot_localization_hz5_filter_on`` written by ``wc_phase1 compare``:
``runtime_config.json`` (mode, frozen mounts), ``frontend/index.jsonl`` (one row per
frame) and the CDR files it names. Every cloud CDR is checked against the index
SHA-256 before use; the prepared cloud is in ``mapping_reference`` (origin at the
lidar, axes aligned with the axle), so base_link = point + ``t_axle_lidar_m``.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

SIDES = ("left", "right")


def cell_side(runtime):
    """The single lidar a frozen cell was prepared from, or ValueError."""
    mode = runtime.get("mode")
    selected = (runtime.get("offline_source_selection") or {}).get("selected_sides")
    if mode not in SIDES or selected != [mode]:
        raise ValueError("a frozen single-lidar cell is required (mode left or right)")
    return mode


def mount(runtime, side):
    """Frozen lever arm (3,) of ``side`` in base_link, from the cell's runtime config."""
    lever = np.asarray(runtime["mounts"][side]["t_axle_lidar_m"], dtype=float)
    if lever.shape != (3,) or not np.isfinite(lever).all() or np.linalg.norm(lever) > 3.0:
        raise ValueError("finite frozen lever arm within 3 m required")
    return lever


def read_cell(cell, *, hashes=None, start_ns=None, end_ns=None):
    """Yield (stamp_ns, Nx3 float32 finite points, 4x4 T_prior_reference) of a frozen cell.

    ``hashes`` (dict) receives the SHA-256 of every file read, keyed by path, so a
    caller can confirm afterwards that the sources did not change.
    """
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from ..rebuild import pointcloud_xyz
    cell = Path(cell).expanduser().resolve(strict=True)
    runtime_raw = (cell / "runtime_config.json").read_bytes()
    index_raw = (cell / "frontend/index.jsonl").read_bytes()
    if hashes is not None:
        hashes[str(cell / "runtime_config.json")] = hashlib.sha256(runtime_raw).hexdigest()
        hashes[str(cell / "frontend/index.jsonl")] = hashlib.sha256(index_raw).hexdigest()
    cell_side(json.loads(runtime_raw))
    previous = None
    for line in index_raw.decode("utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        stamp = int(row["stamp_ns"])
        if previous is not None and stamp <= previous:
            raise ValueError("frontend stamps must increase")
        previous = stamp
        if (start_ns is not None and stamp < start_ns) or (end_ns is not None and stamp > end_ns):
            continue
        name = row["cloud_cdr_file"]
        if Path(name).name != name:
            raise ValueError("cloud CDR path must be a single file name")
        path = cell / "frontend" / name
        if path.is_symlink() or path.stat().st_size > 64 * 1024 ** 2:
            raise ValueError("linked or oversized cloud CDR")
        raw = path.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if digest != row["cloud_cdr_sha256"]:
            raise ValueError("cloud CDR identity mismatch: " + name)
        if hashes is not None:
            hashes[str(path)] = digest
        cloud = deserialize_message(raw, PointCloud2)
        if (int(cloud.header.stamp.sec) * 10 ** 9 + int(cloud.header.stamp.nanosec) != stamp
                or cloud.header.frame_id != "mapping_reference"):
            raise ValueError("cloud stamp or frame differs from the index")
        xyz = pointcloud_xyz(cloud)
        xyz = xyz[np.isfinite(xyz).all(axis=1)].astype(np.float32)
        pose = np.asarray(row["T_prior_reference"], dtype=float)
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            raise ValueError("finite 4x4 frame pose required")
        yield stamp, xyz, pose


def load_runtime(cell):
    cell = Path(cell).expanduser().resolve(strict=True)
    runtime = json.loads((cell / "runtime_config.json").read_text(encoding="utf-8"))
    return cell, runtime, cell_side(runtime)
