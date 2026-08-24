import csv
import hashlib
import json
import math
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml

from smartwheel_map_products.occupancy import OccupancyGridData


def _validated_intensity(
    points: np.ndarray,
    intensity: np.ndarray | None,
) -> np.ndarray | None:
    if intensity is None:
        return None
    amplitudes = np.asarray(intensity, dtype=np.float32).reshape(-1)
    if amplitudes.shape[0] != points.shape[0]:
        raise ValueError("intensity length must match points")
    if not np.isfinite(amplitudes).all():
        raise ValueError("intensity must be finite")
    return amplitudes


def write_pcd(
    path: Path,
    points: np.ndarray,
    intensity: np.ndarray | None = None,
) -> None:
    xyz = np.asarray(points, dtype=np.float64)
    amplitudes = _validated_intensity(xyz, intensity)
    fields = "x y z intensity" if amplitudes is not None else "x y z"
    values_per_point = 4 if amplitudes is not None else 3
    sizes = " ".join(["4"] * values_per_point)
    types = " ".join(["F"] * values_per_point)
    counts = " ".join(["1"] * values_per_point)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\nFIELDS {fields}\nSIZE {sizes}\nTYPE {types}\n"
        f"COUNT {counts}\nWIDTH {xyz.shape[0]}\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {xyz.shape[0]}\nDATA ascii\n"
    )
    with path.open("w", encoding="ascii") as stream:
        stream.write(header)
        if amplitudes is None:
            np.savetxt(stream, xyz, fmt="%.6f %.6f %.6f")
        else:
            np.savetxt(
                stream,
                np.column_stack((xyz, amplitudes)),
                fmt="%.6f %.6f %.6f %.6f",
            )


def write_ply(
    path: Path,
    points: np.ndarray,
    colors: np.ndarray | None = None,
    intensity: np.ndarray | None = None,
) -> None:
    xyz = np.asarray(points, dtype=np.float64)
    amplitudes = _validated_intensity(xyz, intensity)
    rgb = None
    if colors is not None:
        rgb = np.asarray(colors, dtype=np.uint8)
        if rgb.shape != xyz.shape:
            raise ValueError("colors must match points")
    with path.open("w", encoding="ascii") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {xyz.shape[0]}\n")
        stream.write("property float x\nproperty float y\nproperty float z\n")
        if amplitudes is not None:
            stream.write("property float intensity\n")
        if rgb is not None:
            stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        stream.write("end_header\n")
        if rgb is None and amplitudes is None:
            np.savetxt(stream, xyz, fmt="%.6f %.6f %.6f")
            return
        for index, point in enumerate(xyz):
            values = [f"{point[0]:.6f}", f"{point[1]:.6f}", f"{point[2]:.6f}"]
            if amplitudes is not None:
                values.append(f"{float(amplitudes[index]):.6f}")
            if rgb is not None:
                values.extend(str(int(value)) for value in rgb[index])
            stream.write(" ".join(values) + "\n")


def _map_image(cells: np.ndarray) -> np.ndarray:
    image = np.full(cells.shape, 205, dtype=np.uint8)
    image[cells == 0] = 254
    image[cells == 100] = 0
    return np.flipud(image)


def write_occupancy(directory: Path, grid: OccupancyGridData) -> None:
    image = _map_image(grid.cells)
    with (directory / "map_2d.pgm").open("wb") as stream:
        stream.write(f"P5\n{grid.width} {grid.height}\n255\n".encode("ascii"))
        stream.write(image.tobytes())
    if not cv2.imwrite(str(directory / "map_2d.png"), image):
        raise RuntimeError("failed to write map_2d.png")
    metadata = {
        "image": "map_2d.pgm",
        "resolution": grid.resolution,
        "origin": [grid.origin_x, grid.origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with (directory / "map_2d.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)


def write_trajectory(directory: Path, poses: list[tuple[float, float, float, float]]) -> None:
    with (directory / "trajectory.tum").open("w", encoding="ascii") as tum:
        for stamp, x, y, yaw in poses:
            tum.write(
                f"{stamp:.9f} {x:.6f} {y:.6f} 0.000000 0.000000 0.000000 "
                f"{math.sin(yaw / 2.0):.9f} {math.cos(yaw / 2.0):.9f}\n"
            )
    with (directory / "poses.csv").open("w", encoding="ascii", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(("timestamp", "x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"))
        for stamp, x, y, yaw in poses:
            writer.writerow((f"{stamp:.9f}", f"{x:.6f}", f"{y:.6f}", "0.0", "0.0", "0.0", f"{yaw:.9f}"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _replace_none_for_markdown(value):
    if value is None:
        return "UNAVAILABLE"
    if isinstance(value, dict):
        return {
            str(key): _replace_none_for_markdown(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_replace_none_for_markdown(item) for item in value]
    return value


def _markdown_value(value) -> str:
    if value is None:
        return "UNAVAILABLE"
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(
            _replace_none_for_markdown(value),
            sort_keys=True,
            allow_nan=False,
        )
    return str(value)


def _flatten_markdown_items(value: dict, prefix: str = ""):
    for key in sorted(value, key=str):
        item = value[key]
        path = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(item, dict) and item:
            yield from _flatten_markdown_items(item, path)
        else:
            yield path, _markdown_value(item)


def export_map_bundle(
    directory: str | Path,
    points: np.ndarray,
    grid: OccupancyGridData,
    poses: list[tuple[float, float, float, float]],
    colors: np.ndarray | None,
    hardware_profile_path: str,
    algorithm_profile: dict,
    bag_path: str,
    quality: dict,
    intensity_points: np.ndarray | None = None,
    intensity: np.ndarray | None = None,
) -> Path:
    quality_json = json.dumps(
        quality,
        indent=2,
        sort_keys=True,
        allow_nan=False,
    ) + "\n"
    markdown_lines = [
        "# Map Quality Report",
        "",
        (
            f"Validation stage: {_markdown_value(quality.get('stage', 'UNSPECIFIED'))}; "
            f"hardware validated: {bool(quality.get('hardware_validated', False))}."
        ),
        "",
    ]
    markdown_lines.extend(
        f"- `{key}`: `{value}`" for key, value in _flatten_markdown_items(quality)
    )
    quality_markdown = "\n".join(markdown_lines) + "\n"

    output = Path(directory)
    output.mkdir(parents=True, exist_ok=True)
    (output / "logs").mkdir(exist_ok=True)
    write_pcd(output / "map_geometry.pcd", points)
    write_ply(output / "map_geometry.ply", points)
    if colors is not None:
        write_ply(output / "map_colored.ply", points, colors)
    if intensity_points is not None and intensity is not None:
        write_pcd(output / "map_pointcloud_amp.pcd", intensity_points, intensity)
        write_ply(
            output / "map_pointcloud_amp.ply",
            intensity_points,
            intensity=intensity,
        )
    write_occupancy(output, grid)
    write_trajectory(output, poses)
    profile_source = Path(hardware_profile_path)
    if not profile_source.is_file():
        raise FileNotFoundError(f"hardware profile does not exist: {profile_source}")
    shutil.copyfile(profile_source, output / "hardware_profile_used.yaml")
    with (output / "algorithm_profile_used.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(algorithm_profile, stream, sort_keys=True)
    (output / "bag_path.txt").write_text((bag_path or "NOT_RECORDED") + "\n", encoding="utf-8")
    (output / "quality_report.json").write_text(quality_json, encoding="utf-8")
    (output / "quality_report.md").write_text(quality_markdown, encoding="utf-8")

    files = []
    externally_managed_files = []
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            relative = path.relative_to(output)
            if path.name == "rtabmap.db" or "raw_bag" in relative.parts:
                externally_managed_files.append(str(relative))
                continue
            files.append(
                {
                    "path": str(relative),
                    "bytes": path.stat().st_size,
                    "sha256": _sha256(path),
                }
            )
    manifest = {
        "format_version": 1,
        "stage": quality.get("stage", "UNSPECIFIED"),
        "complete": True,
        "files": files,
        "externally_managed_files": externally_managed_files,
    }
    manifest_json = json.dumps(
        manifest,
        indent=2,
        sort_keys=True,
        allow_nan=False,
    ) + "\n"
    (output / "manifest.json").write_text(manifest_json, encoding="utf-8")
    return output
