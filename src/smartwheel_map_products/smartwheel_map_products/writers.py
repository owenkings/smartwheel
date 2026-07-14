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


def write_pcd(path: Path, points: np.ndarray) -> None:
    xyz = np.asarray(points, dtype=np.float64)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {xyz.shape[0]}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {xyz.shape[0]}\nDATA ascii\n"
    )
    with path.open("w", encoding="ascii") as stream:
        stream.write(header)
        np.savetxt(stream, xyz, fmt="%.6f %.6f %.6f")


def write_ply(path: Path, points: np.ndarray, colors: np.ndarray | None = None) -> None:
    xyz = np.asarray(points, dtype=np.float64)
    with path.open("w", encoding="ascii") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {xyz.shape[0]}\n")
        stream.write("property float x\nproperty float y\nproperty float z\n")
        if colors is not None:
            stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        stream.write("end_header\n")
        if colors is None:
            np.savetxt(stream, xyz, fmt="%.6f %.6f %.6f")
        else:
            rgb = np.asarray(colors, dtype=np.uint8)
            if rgb.shape != xyz.shape:
                raise ValueError("colors must match points")
            for point, color in zip(xyz, rgb):
                stream.write(
                    f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                    f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
                )


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
) -> Path:
    output = Path(directory)
    output.mkdir(parents=True, exist_ok=True)
    (output / "logs").mkdir(exist_ok=True)
    write_pcd(output / "map_geometry.pcd", points)
    write_ply(output / "map_geometry.ply", points)
    if colors is not None:
        write_ply(output / "map_colored.ply", points, colors)
    write_occupancy(output, grid)
    write_trajectory(output, poses)
    profile_source = Path(hardware_profile_path)
    if not profile_source.is_file():
        raise FileNotFoundError(f"hardware profile does not exist: {profile_source}")
    shutil.copyfile(profile_source, output / "hardware_profile_used.yaml")
    with (output / "algorithm_profile_used.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(algorithm_profile, stream, sort_keys=True)
    (output / "bag_path.txt").write_text((bag_path or "NOT_RECORDED") + "\n", encoding="utf-8")
    with (output / "quality_report.json").open("w", encoding="utf-8") as stream:
        json.dump(quality, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with (output / "quality_report.md").open("w", encoding="utf-8") as stream:
        stream.write("# Map Quality Report\n\n")
        stream.write("Stage A synthetic result; this is not real hardware validation.\n\n")
        for key, value in sorted(quality.items()):
            stream.write(f"- `{key}`: `{value}`\n")
    files = []
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            files.append(
                {
                    "path": str(path.relative_to(output)),
                    "bytes": path.stat().st_size,
                    "sha256": _sha256(path),
                }
            )
    manifest = {
        "format_version": 1,
        "stage": "A_SYNTHETIC",
        "complete": True,
        "files": files,
    }
    with (output / "manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return output

