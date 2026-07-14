import json
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
    checks["manifest_complete"] = manifest.get("complete") is True
    checks["minimum_map_points"] = int(quality.get("point_count", 0)) >= minimum_points
    checks["minimum_trajectory_poses"] = int(quality.get("trajectory_pose_count", 0)) >= minimum_poses
    checks["occupancy_has_occupied"] = int(quality.get("occupied_cells", 0)) > 0
    checks["occupancy_has_free"] = int(quality.get("free_cells", 0)) > 0
    checks["occupancy_has_unknown"] = int(quality.get("unknown_cells", 0)) > 0
    rmse = quality.get("trajectory_ground_truth_rmse_m")
    checks["trajectory_rmse_available"] = isinstance(rmse, (int, float)) and math.isfinite(rmse)
    checks["trajectory_rmse_within_limit"] = (
        checks["trajectory_rmse_available"] and float(rmse) <= maximum_rmse_m
    )
    source = str(quality.get("geometry_source", ""))
    checks["geometry_source_valid"] = (
        source.startswith("backend:") if backend == "rtabmap" else source == "local_odometry_accumulator"
    )
    return checks
