import json
import hashlib
import math
from pathlib import Path


REQUIRED_EXPORTS = (
    "map_geometry.pcd",
    "map_geometry.ply",
    "map_2d.pgm",
    "map_2d.png",
    "map_2d.yaml",
    "trajectory.tum",
    "poses.csv",
    "quality_report.json",
    "manifest.json",
)


def validate_quality_bundle(
    directory: str | Path,
    backend: str,
    minimum_points: int,
    minimum_poses: int,
    maximum_rmse_m: float,
) -> dict[str, bool]:
    output = Path(directory)
    checks = {
        "export_directory": output.is_dir(),
        "required_export_files": False,
        "manifest_complete": False,
        "minimum_map_points": False,
        "minimum_trajectory_poses": False,
        "occupancy_has_occupied": False,
        "occupancy_has_free": False,
        "occupancy_has_unknown": False,
        "trajectory_rmse_available": False,
        "trajectory_rmse_within_limit": False,
        "geometry_source_valid": False,
    }
    if not checks["export_directory"]:
        return checks
    checks["required_export_files"] = all(
        (output / name).is_file() and (output / name).stat().st_size > 0 for name in REQUIRED_EXPORTS
    )
    try:
        manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        quality = json.loads((output / "quality_report.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return checks
    checks["manifest_complete"] = (
        manifest.get("complete") is True and not (output / ".incomplete").exists()
    )
    if "files" in manifest:
        checks["manifest_integrity"] = _manifest_files_intact(output, manifest)
    if "session" in quality:
        checks["session_complete"] = _session_quality_is_complete(quality["session"])
    checks["minimum_map_points"] = int(quality.get("point_count", 0)) >= minimum_points
    checks["minimum_trajectory_poses"] = int(quality.get("trajectory_pose_count", 0)) >= minimum_poses
    checks["occupancy_has_occupied"] = int(quality.get("occupied_cells", 0)) > 0
    checks["occupancy_has_free"] = int(quality.get("free_cells", 0)) > 0
    checks["occupancy_has_unknown"] = int(quality.get("unknown_cells", 0)) > 0
    rmse = quality.get("trajectory_ground_truth_rmse_m")
    checks["trajectory_rmse_available"] = (
        isinstance(rmse, (int, float))
        and not isinstance(rmse, bool)
        and math.isfinite(rmse)
        and rmse >= 0.0
    )
    checks["trajectory_rmse_within_limit"] = (
        checks["trajectory_rmse_available"] and float(rmse) <= maximum_rmse_m
    )
    source = str(quality.get("geometry_source", ""))
    checks["geometry_source_valid"] = (
        source.startswith("backend:") if backend == "rtabmap" else source == "local_odometry_accumulator"
    )
    return checks


def _manifest_files_intact(output: Path, manifest: dict) -> bool:
    entries = manifest.get("files")
    if not isinstance(entries, list) or not entries:
        return False
    for entry in entries:
        if not isinstance(entry, dict):
            return False
        relative = entry.get("path")
        expected_bytes = entry.get("bytes")
        expected_sha = entry.get("sha256")
        if not isinstance(relative, str):
            return False
        relative_path = Path(relative)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            return False
        path = output / relative_path
        if not path.is_file():
            return False
        try:
            if path.stat().st_size != int(expected_bytes):
                return False
        except (TypeError, ValueError, OSError):
            return False
        digest = hashlib.sha256()
        try:
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
        except OSError:
            return False
        if digest.hexdigest() != expected_sha:
            return False
    return True


def _session_quality_is_complete(session: object) -> bool:
    if not isinstance(session, dict):
        return False
    if session.get("complete") is not True or session.get("state") != "STOPPED":
        return False
    if session.get("failure_reason"):
        return False
    try:
        frame_count = int(session.get("frame_count", 0))
        raw_point_count = int(session.get("raw_point_count", 0))
        rejected_frame_count = int(session.get("rejected_frame_count", 0))
        first = float(session["first_cloud_stamp"])
        last = float(session["last_cloud_stamp"])
        span = float(session["cloud_time_span_sec"])
    except (KeyError, TypeError, ValueError):
        return False
    return (
        frame_count > 0
        and raw_point_count > 0
        and math.isfinite(first)
        and math.isfinite(last)
        and math.isfinite(span)
        and last >= first
        and abs(span - (last - first)) <= 1e-6
        and isinstance(session.get("frame_ids"), list)
        and len(session["frame_ids"]) == 1
        and rejected_frame_count >= 0
    )
