"""Immutable map packages with Linux flock, SQLite backup and atomic commit.

The caller must capture/quiesce graph sidecars and the RTAB-Map writer at one
application revision. SQLite backup provides a database transaction snapshot;
it cannot independently prove a third-party database's graph/sidecar relation.
"""

from __future__ import annotations

import contextlib
import ctypes
import datetime as dt
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sqlite3
import stat
import tempfile
import time

import numpy as np

from .builder import MapError, build_map, identity, rigid


_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")
_INPUTS = ("snapshot.json", "observations.json", "graph.json", "trajectory.json", "calibration.json", "config.json")
_REQUIRED = set(_INPUTS) | {"slam/rtabmap.db", "cloud/map.ply", "occupancy/voxels.json", "occupancy/grid.json",
                            "occupancy/map.pgm", "occupancy/map.yaml", "raw_data_index.json", "goals.json", "map_quality.json"}


def _name(value: str, label: str) -> str:
    if not isinstance(value, str) or not _NAME.fullmatch(value):
        raise MapError(f"{label}: use 1-64 ASCII letters, digits, underscore, dot or hyphen, starting with a letter/digit")
    return value


def _no_symlinks(path: Path) -> Path:
    absolute = Path(os.path.abspath(path))
    for component in reversed((absolute, *absolute.parents)):
        try:
            mode = component.lstat().st_mode
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(mode):
            raise MapError(f"symbolic links are forbidden: {component}")
    return absolute


def _read_bytes(path: Path) -> bytes:
    path = _no_symlinks(path)
    # O_NONBLOCK prevents a replaced FIFO/device path from blocking before its
    # descriptor is inspected. It has no effect for regular snapshot files.
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
    descriptor = os.open(path, flags)
    with os.fdopen(descriptor, "rb") as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise MapError(f"not a regular file: {path}")
        return stream.read()


def _decode(data: bytes, label: str):
    try:
        return json.loads(data.decode("utf-8"), parse_constant=lambda item: (_ for _ in ()).throw(ValueError(item)))
    except (ValueError, UnicodeError) as exc:
        raise MapError(f"invalid strict UTF-8 JSON: {label}") from exc


def _json(path: Path):
    return _decode(_read_bytes(path), str(path))


def _write(path: Path, data: bytes):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())


def _write_json(path: Path, value):
    _write(path, (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8"))


def _hash(path: Path) -> dict:
    path = _no_symlinks(path)
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0))
    checksum, size = hashlib.sha256(), 0
    with os.fdopen(descriptor, "rb") as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise MapError(f"not a regular file: {path}")
        while block := stream.read(1024 * 1024):
            checksum.update(block)
            size += len(block)
    return {"sha256": checksum.hexdigest(), "bytes": size}


def _sync_directory(path: Path):
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _rename_no_replace(source: Path, destination: Path):
    # Linux renameat2 is an atomic directory publication that also refuses an
    # independently-created empty destination. Plain os.rename can overwrite it.
    libc = ctypes.CDLL(None, use_errno=True)
    operation = getattr(libc, "renameat2", None)
    if operation is None:
        raise MapError("atomic no-replace directory commit requires Linux renameat2")
    operation.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    operation.restype = ctypes.c_int
    if operation(-100, os.fsencode(source), -100, os.fsencode(destination), 1) != 0:
        code = ctypes.get_errno()
        if code == errno.EEXIST:
            raise MapError("map version already exists and is immutable")
        if code in (errno.EINVAL, errno.EOPNOTSUPP, errno.ENOSYS):
            from wc_runtime.storage_atomic import rename_on_configured_archive
            return rename_on_configured_archive(source, destination)
        raise OSError(code, os.strerror(code), str(destination))


def _sqlite_backup(source: Path, destination: Path):
    source = _no_symlinks(source)
    if not source.is_file():
        raise MapError("slam/rtabmap.db is required; cloud-only save is incomplete")
    # WAL/SHM are part of a live SQLite database and must not redirect elsewhere.
    for suffix in ("-wal", "-shm"):
        _no_symlinks(Path(str(source) + suffix))
    uri = source.as_uri() + "?mode=ro"
    destination.parent.mkdir(parents=True, exist_ok=True)
    src = sqlite3.connect(uri, uri=True, timeout=5)
    dst = sqlite3.connect(destination)
    try:
        if src.execute("PRAGMA quick_check").fetchall() != [("ok",)]:
            raise MapError("source SQLite integrity check failed")
        deadline = time.monotonic() + 120

        def progress(status, remaining, total):
            if time.monotonic() > deadline:
                raise MapError("SQLite backup exceeded 120 seconds; quiesce the map writer and retry")

        src.backup(dst, pages=256, progress=progress, sleep=0.01)
        if dst.execute("PRAGMA quick_check").fetchall() != [("ok",)]:
            raise MapError("backed-up SQLite integrity check failed")
        dst.execute("PRAGMA journal_mode=DELETE")
        dst.commit()
    finally:
        dst.close()
        src.close()
    with destination.open("rb") as stream:
        os.fsync(stream.fileno())


def _validate_goal(goal: dict) -> dict:
    name = goal.get("name")
    if not isinstance(name, str) or not name.strip() or len(name) > 128:
        raise MapError("goal name must be a nonempty UTF-8 string of at most 128 characters")
    position, orientation = goal.get("position"), goal.get("orientation_xyzw")
    if not isinstance(position, list) or len(position) != 3 or not isinstance(orientation, list) or len(orientation) != 4:
        raise MapError("goal requires position xyz and orientation_xyzw")
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in position + orientation):
        raise MapError("goal position/quaternion must be finite numbers")
    if not math.isclose(sum(v * v for v in orientation), 1.0, rel_tol=0, abs_tol=1e-6):
        raise MapError("goal quaternion must be normalized")
    return {"name": name, "position": position, "orientation_xyzw": orientation}


def _snapshot_validate(inputs: dict):
    snapshot, graph, config = inputs["snapshot.json"], inputs["graph.json"], inputs["config.json"]
    mandatory = {"schema_version", "session_id", "source_mode", "sensor_mode", "graph_epoch", "graph_revision",
                 "completed_map_revision", "frame_id", "ground_valid", "calibration_id", "time_model_id"}
    if mandatory - snapshot.keys():
        raise MapError(f"snapshot fields missing: {sorted(mandatory - snapshot.keys())}")
    if snapshot["schema_version"] != 1 or snapshot["source_mode"] not in ("synthetic", "real"):
        raise MapError("unsupported snapshot schema/source_mode")
    if not isinstance(snapshot["session_id"], str) or not snapshot["session_id"]:
        raise MapError("snapshot session_id must be a nonempty string")
    # A top-level label must not rebrand a synthetic graph/calibration as real.
    # Calibration and map configuration may be reusable across sessions, but
    # an explicit session/epoch declaration is never allowed to contradict the
    # imported snapshot. The graph and each original frame are session-bound.
    calibration = inputs["calibration.json"]
    for label, document in (("graph", graph), ("calibration", calibration)):
        if not isinstance(document, dict) or document.get("source_mode") != snapshot["source_mode"]:
            raise MapError(f"{label} source_mode is missing or disagrees with the snapshot")
    if calibration.get("calibration_id") != snapshot["calibration_id"]:
        raise MapError("calibration calibration_id disagrees with the snapshot")
    for label, document in (("graph", graph), ("calibration", calibration), ("config", config),
                            ("observations", inputs["observations.json"]), ("trajectory", inputs["trajectory.json"])):
        if not isinstance(document, dict):
            raise MapError(f"{label} must be a JSON object")
        for field in ("source_mode", "session_id", "graph_epoch"):
            if field in document and document[field] != snapshot[field]:
                raise MapError(f"{label} {field} disagrees with the snapshot")
    for field in ("frame_id", "sensor_mode", "calibration_id", "time_model_id", "left_clock_model_id", "right_clock_model_id"):
        if field in graph and field in snapshot and graph[field] != snapshot[field]:
            raise MapError(f"graph {field} disagrees with the snapshot")
    frames = inputs["observations.json"].get("observations")
    if not isinstance(frames, list):
        raise MapError("observations must contain an original frame list")
    for frame in frames:
        if not isinstance(frame, dict):
            raise MapError("each original observation must be a JSON object")
        for field in ("session_id", "graph_epoch"):
            if frame.get(field) != snapshot[field]:
                raise MapError(f"original observation {field} is missing or disagrees with the snapshot")
        if "source_mode" in frame and frame["source_mode"] != snapshot["source_mode"]:
            raise MapError("original observation source_mode disagrees with the snapshot")
    if snapshot["graph_revision"] != snapshot["completed_map_revision"]:
        raise MapError("snapshot has an incomplete map revision")
    if (snapshot["session_id"], snapshot["graph_epoch"], snapshot["graph_revision"]) != (graph.get("session_id"), graph.get("graph_epoch"), graph.get("revision")):
        raise MapError("snapshot and graph revision/session/epoch disagree")
    if not isinstance(snapshot["frame_id"], str) or not snapshot["frame_id"]:
        raise MapError("snapshot frame_id is required")
    if not isinstance(snapshot["ground_valid"], bool):
        raise MapError("snapshot ground_valid must be boolean")
    for key in ("ground_valid", "sensor_mode"):
        if key in config and config[key] != snapshot[key]:
            raise MapError(f"snapshot and config {key} disagree")
    trajectory = inputs["trajectory.json"]
    trajectory_fields = {"session_id", "graph_epoch", "graph_revision", "body_frame_id", "poses"}
    if not isinstance(trajectory, dict) or trajectory_fields - trajectory.keys():
        raise MapError("trajectory requires session_id, graph_epoch, graph_revision, body_frame_id and poses")
    for key, expected in (("graph_revision", snapshot["graph_revision"]), ("session_id", snapshot["session_id"]), ("graph_epoch", snapshot["graph_epoch"])):
        if trajectory[key] != expected:
            raise MapError(f"trajectory {key} disagrees with the snapshot")
    if trajectory["body_frame_id"] != "rig_link":
        raise MapError("this exporter only supports the rig_link trajectory")
    if not isinstance(trajectory["poses"], list) or not trajectory["poses"]:
        raise MapError("trajectory poses must be a nonempty list")
    node_poses = {}
    for node in graph.get("nodes", []):
        key = identity(node.get("node_id"), "graph node_id")
        if key in node_poses:
            raise MapError("duplicate graph node ID")
        node_poses[key] = rigid(node.get("T_map_node"), "graph T_map_node")
    trajectory_ids = set()
    for pose in trajectory["poses"]:
        if not isinstance(pose, dict) or not {"node_id", "T_map_rig"}.issubset(pose):
            raise MapError("trajectory pose requires node_id and T_map_rig")
        key = identity(pose["node_id"], "trajectory node_id")
        if key in trajectory_ids:
            raise MapError("duplicate trajectory node ID")
        trajectory_ids.add(key)
        if key not in node_poses:
            raise MapError("trajectory references an unknown graph node")
        matrix = rigid(pose["T_map_rig"], "trajectory T_map_rig")
        if not np.allclose(matrix, node_poses[key], atol=1e-8, rtol=0):
            raise MapError("trajectory pose disagrees with optimized graph pose")
    if trajectory_ids != set(node_poses):
        raise MapError("trajectory must cover every graph node exactly once")


class MapStore:
    """All public paths remain underneath an explicitly chosen project map root."""

    def __init__(self, root):
        self.root = _no_symlinks(Path(root))
        self.root.mkdir(parents=True, exist_ok=True)
        if not self.root.is_dir():
            raise MapError("map root must be a directory")

    @contextlib.contextmanager
    def _lock(self):
        try:
            import fcntl
        except ImportError as exc:
            raise MapError("MapStore mutation requires Linux fcntl/flock") from exc
        _no_symlinks(self.root)
        path = _no_symlinks(self.root / ".map_store.lock")
        descriptor = os.open(path, os.O_RDWR | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0), 0o600)
        try:
            if not stat.S_ISREG(os.fstat(descriptor).st_mode):
                raise MapError("map lock is not a regular file")
            fcntl.flock(descriptor, fcntl.LOCK_EX)
            yield
        finally:
            os.close(descriptor)

    def _version(self, map_id: str, version: str) -> Path:
        return _no_symlinks(self.root / _name(map_id, "map_id") / _name(version, "version"))

    def save_snapshot(self, snapshot_dir, map_id: str, version: str) -> dict:
        source = _no_symlinks(Path(snapshot_dir))
        final = self._version(map_id, version)
        with self._lock():
            if final.exists():
                raise MapError("map version already exists and is immutable")
            final.parent.mkdir(parents=True, exist_ok=True)
            source_bytes = {name: _read_bytes(source / name) for name in _INPUTS}
            inputs = {name: _decode(data, name) for name, data in source_bytes.items()}
            _snapshot_validate(inputs)
            snapshot = inputs["snapshot.json"]
            config = dict(inputs["config.json"], ground_valid=snapshot["ground_valid"], sensor_mode=snapshot["sensor_mode"])
            mapped = build_map(inputs["graph.json"], inputs["observations.json"], config)
            temporary = Path(tempfile.mkdtemp(prefix=".pending-", dir=final.parent))
            try:
                for name, data in source_bytes.items():
                    _write(temporary / name, data)
                _sqlite_backup(source / "slam" / "rtabmap.db", temporary / "slam" / "rtabmap.db")
                # Refuse a sidecar set that changed while the database was captured.
                if any(_read_bytes(source / name) != data for name, data in source_bytes.items()):
                    raise MapError("snapshot sidecars changed during save; obtain a quiesced snapshot")
                _write_json(temporary / "occupancy" / "voxels.json", dict(mapped["occupancy"], revision=mapped["revision"]))
                grid = dict(mapped["grid"], revision=mapped["revision"], frame_id=snapshot["frame_id"])
                _write_json(temporary / "occupancy" / "grid.json", grid)
                header = f"ply\nformat ascii 1.0\ncomment graph_revision {mapped['revision']}\nelement vertex {len(mapped['cloud'])}\nproperty double x\nproperty double y\nproperty double z\nend_header\n"
                cloud_text = header + "".join(" ".join(format(coordinate, ".17g") for coordinate in point) + "\n" for point in mapped["cloud"])
                _write(temporary / "cloud" / "map.ply", cloud_text.encode("ascii"))
                width, height = grid["width"], grid["height"]
                pixels = bytes({-1: 205, 0: 254, 100: 0}[grid["data"][y * width + x]] for y in reversed(range(height)) for x in range(width))
                _write(temporary / "occupancy" / "map.pgm", f"P5\n{width} {height}\n255\n".encode("ascii") + pixels)
                yaml_text = f"image: map.pgm\nresolution: {grid['resolution']:.17g}\norigin: {json.dumps(grid['origin'])}\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
                _write(temporary / "occupancy" / "map.yaml", yaml_text.encode("utf-8"))
                _write_json(temporary / "raw_data_index.json", {"session_id": mapped["session_id"], "graph_epoch": mapped["graph_epoch"],
                                                               "revision": mapped["revision"], "frames": mapped["raw_data_index"], "origins": mapped["origins"]})
                incoming_goals = {"goals": []}
                if (source / "goals.json").exists() or (source / "goals.json").is_symlink():
                    incoming_goals = _json(source / "goals.json")
                goals = []
                names = set()
                for goal in incoming_goals.get("goals", []):
                    clean = _validate_goal(goal)
                    if clean["name"] in names:
                        raise MapError("duplicate goal names")
                    names.add(clean["name"])
                    clean.update(map_id=map_id, version=version, frame_id=snapshot["frame_id"], review_state="UNVERIFIED",
                                 review_reason="new_map_version_requires_review")
                    goals.append(clean)
                _write_json(temporary / "goals.json", {"map_id": map_id, "version": version, "goals": goals})
                _write_json(temporary / "map_quality.json", {"navigation_validated": False, "source_mode": snapshot["source_mode"],
                                                           "sensor_mode": mapped["sensor_mode"], "sensor_ids": mapped["sensor_ids"],
                                                           "source_point_counts": mapped["source_point_counts"], **mapped["quality"]})
                artifacts = {path.relative_to(temporary).as_posix(): _hash(path) for path in sorted(temporary.rglob("*")) if path.is_file()}
                manifest = {"schema_version": 1, "map_id": map_id, "version": version,
                            "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(), "snapshot": snapshot,
                            "source_mode": snapshot["source_mode"], "sensor_mode": mapped["sensor_mode"],
                            "state": "LOADED_VIEW_ONLY", "navigation_validated": False, "frame_id": snapshot["frame_id"],
                            "graph_revision": mapped["revision"], "completed_map_revision": mapped["revision"],
                            "formats": {"cloud": "PLY_ASCII", "occupancy": "sparse_binary_evidence_v1", "grid": "ROS_map_yaml_PGM"},
                            "database_consistency": "sqlite_read_only_backup; caller_owns_graph_snapshot_barrier",
                            "artifacts": artifacts}
                _write_json(temporary / "manifest.json", manifest)
                _write(temporary / "manifest.sha256", (hashlib.sha256(_read_bytes(temporary / "manifest.json")).hexdigest() + "\n").encode("ascii"))
                self._verify_path(temporary, map_id, version)
                for directory in sorted((p for p in temporary.rglob("*") if p.is_dir()), reverse=True):
                    _sync_directory(directory)
                _sync_directory(temporary)
                _no_symlinks(final.parent)
                _rename_no_replace(temporary, final)
                _sync_directory(final.parent)
                return {"path": str(final), "manifest": manifest}
            except OSError as exc:
                if exc.errno == errno.ENOSPC:
                    raise MapError("disk full: map version was not committed") from exc
                raise
            finally:
                if temporary.exists():
                    # Only this invocation's generated staging directory is removed.
                    checked = _no_symlinks(temporary)
                    if checked.parent != final.parent or not checked.name.startswith(".pending-"):
                        raise MapError("refusing cleanup outside this save's staging directory")
                    shutil.rmtree(checked)

    def _verify_path(self, directory: Path, map_id: str, version: str) -> dict:
        _no_symlinks(directory)
        raw_manifest = _read_bytes(directory / "manifest.json")
        expected = _read_bytes(directory / "manifest.sha256").decode("ascii").strip()
        if hashlib.sha256(raw_manifest).hexdigest() != expected:
            raise MapError("manifest hash mismatch")
        manifest = _decode(raw_manifest, "manifest.json")
        if manifest.get("schema_version") != 1 or manifest.get("map_id") != map_id or manifest.get("version") != version:
            raise MapError("manifest identity/schema mismatch")
        artifacts = manifest.get("artifacts")
        if not isinstance(artifacts, dict) or not _REQUIRED.issubset(artifacts):
            raise MapError("manifest is missing required snapshot artifacts")
        actual = set()
        for path in directory.rglob("*"):
            _no_symlinks(path)
            if path.is_file():
                actual.add(path.relative_to(directory).as_posix())
            elif not path.is_dir():
                raise MapError("map package contains a non-regular filesystem object")
        if actual != set(artifacts) | {"manifest.json", "manifest.sha256"}:
            raise MapError("map package inventory differs from manifest")
        for relative, expected_hash in artifacts.items():
            candidate = Path(relative)
            if candidate.is_absolute() or ".." in candidate.parts or "\\" in relative or ":" in relative:
                raise MapError("unsafe artifact path in manifest")
            if _hash(directory / candidate) != expected_hash:
                raise MapError(f"artifact hash mismatch: {relative}")
        inputs = {name: _json(directory / name) for name in _INPUTS}
        _snapshot_validate(inputs)
        revision = inputs["snapshot.json"]["graph_revision"]
        if manifest.get("graph_revision") != revision or manifest.get("completed_map_revision") != revision:
            raise MapError("manifest revision disagrees with snapshot")
        if manifest.get("snapshot") != inputs["snapshot.json"]:
            raise MapError("manifest snapshot metadata disagreement")
        for field in ("source_mode", "sensor_mode", "frame_id"):
            if manifest.get(field) != inputs["snapshot.json"][field]:
                raise MapError(f"manifest {field} disagrees with the snapshot")
        quality = _json(directory / "map_quality.json")
        for field in ("source_mode", "sensor_mode"):
            if quality.get(field) != inputs["snapshot.json"][field]:
                raise MapError(f"map quality {field} disagrees with the snapshot")
        for filename in ("occupancy/grid.json", "occupancy/voxels.json", "raw_data_index.json"):
            if _json(directory / filename).get("revision") != revision:
                raise MapError(f"map component revision mismatch: {filename}")
        raw_index = _json(directory / "raw_data_index.json")
        for field in ("session_id", "graph_epoch"):
            if raw_index.get(field) != inputs["snapshot.json"][field]:
                raise MapError(f"saved raw index {field} disagrees with the snapshot")
        db = sqlite3.connect((directory / "slam" / "rtabmap.db").as_uri() + "?mode=ro&immutable=1", uri=True)
        try:
            if db.execute("PRAGMA quick_check").fetchall() != [("ok",)]:
                raise MapError("saved SQLite database integrity failure")
        finally:
            db.close()
        return manifest

    def verify(self, map_id: str, version: str) -> dict:
        return self._verify_path(self._version(map_id, version), map_id, version)

    def load(self, map_id: str, version: str) -> dict:
        path = self._version(map_id, version)
        manifest = self.verify(map_id, version)
        return {"state": "LOADED_VIEW_ONLY", "current_pose": None, "path": str(path), "manifest": manifest,
                "cloud_path": str(path / "cloud" / "map.ply"), "grid": _json(path / "occupancy" / "grid.json"),
                "occupancy": _json(path / "occupancy" / "voxels.json"), "trajectory": _json(path / "trajectory.json"),
                "graph": _json(path / "graph.json"), "quality": _json(path / "map_quality.json"),
                "goals": self.get_goals(map_id, version, verify=False)}

    def list_versions(self, map_id: str) -> list[dict]:
        parent = _no_symlinks(self.root / _name(map_id, "map_id"))
        if not parent.exists():
            return []
        results = []
        for child in sorted(parent.iterdir()):
            _no_symlinks(child)
            if child.name.startswith(".pending-"):
                continue
            if child.is_dir():
                manifest = self.verify(map_id, child.name)
                results.append({"map_id": map_id, "version": child.name, "path": str(child), "graph_revision": manifest["graph_revision"]})
        return results

    def _annotation_path(self, map_id: str, version: str) -> Path:
        return _no_symlinks(self.root / ".annotations" / _name(map_id, "map_id") / _name(version, "version") / "goals.json")

    def get_goals(self, map_id: str, version: str, verify: bool = True) -> dict:
        if verify:
            self.verify(map_id, version)
        annotations = self._annotation_path(map_id, version)
        data = _json(annotations if annotations.exists() else self._version(map_id, version) / "goals.json")
        if data.get("map_id") != map_id or data.get("version") != version:
            raise MapError("goal annotations bound to a different map/version")
        for goal in data.get("goals", []):
            _validate_goal(goal)
            if goal.get("map_id") != map_id or goal.get("version") != version or goal.get("review_state") not in ("UNVERIFIED", "VERIFIED"):
                raise MapError("goal identity/review state mismatch")
        return data

    def _write_annotations(self, map_id: str, version: str, data: dict):
        target = self._annotation_path(map_id, version)
        target.parent.mkdir(parents=True, exist_ok=True)
        descriptor, filename = tempfile.mkstemp(prefix=".goals-", dir=target.parent)
        temporary = Path(filename)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(data, stream, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            _no_symlinks(target)
            os.replace(temporary, target)
            _sync_directory(target.parent)
        finally:
            if temporary.exists():
                temporary.unlink()

    def set_goal(self, map_id: str, version: str, goal: dict) -> dict:
        clean = _validate_goal(goal)
        with self._lock():
            manifest = self.verify(map_id, version)
            data = self.get_goals(map_id, version, verify=False)
            clean.update(map_id=map_id, version=version, frame_id=manifest["frame_id"], review_state="UNVERIFIED", review_reason="new_or_edited_annotation")
            data["goals"] = [item for item in data["goals"] if item["name"] != clean["name"]] + [clean]
            self._write_annotations(map_id, version, data)
            return clean

    def review_goal(self, map_id: str, version: str, name: str, verified: bool) -> dict:
        if not isinstance(verified, bool):
            raise MapError("verified must be a boolean")
        with self._lock():
            data = self.get_goals(map_id, version)
            matches = [item for item in data["goals"] if item["name"] == name]
            if len(matches) != 1:
                raise MapError("goal name not found or ambiguous")
            matches[0].update(review_state="VERIFIED" if verified else "UNVERIFIED", review_reason="explicit_operator_review")
            self._write_annotations(map_id, version, data)
            return matches[0]

    def migrate_goals(self, map_id: str, old_version: str, new_version: str) -> dict:
        if old_version == new_version:
            raise MapError("goal migration requires a distinct map version")
        with self._lock():
            source = self.get_goals(map_id, old_version)
            target_manifest = self.verify(map_id, new_version)
            if self.verify(map_id, old_version)["frame_id"] != target_manifest["frame_id"]:
                raise MapError("coordinate frame changed; explicit transformed annotation input is required")
            migrated = []
            for goal in source["goals"]:
                updated = dict(goal, version=new_version, frame_id=target_manifest["frame_id"], review_state="UNVERIFIED",
                               review_reason="map_version_changed", inherited_from={"map_id": map_id, "version": old_version})
                migrated.append(updated)
            result = {"map_id": map_id, "version": new_version, "goals": migrated}
            self._write_annotations(map_id, new_version, result)
            return result
