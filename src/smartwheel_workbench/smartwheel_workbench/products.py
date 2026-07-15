import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


REQUIRED_PRODUCTS = (
    "map_geometry.pcd",
    "map_geometry.ply",
    "map_2d.pgm",
    "map_2d.png",
    "map_2d.yaml",
    "trajectory.tum",
    "quality_report.json",
    "quality_report.md",
    "manifest.json",
)
OPTIONAL_PRODUCTS = ("map_colored.ply", "rtabmap.db")


@dataclass(frozen=True)
class LoadedMap:
    directory: Path
    points: np.ndarray
    colors: np.ndarray | None
    cells: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    trajectory: list[tuple[float, float, float, float]]
    files: list[str]
    quality_summary: str


def list_map_versions(root: str | Path) -> list[Path]:
    directory = Path(root).expanduser().resolve()
    if not directory.is_dir():
        return []
    return sorted(
        (item for item in directory.iterdir() if item.is_dir() and (item / "manifest.json").is_file()),
        key=lambda item: item.stat().st_mtime,
        reverse=True,
    )


def read_ascii_pcd(path: Path) -> np.ndarray:
    fields = None
    points = None
    lines = []
    with path.open(encoding="ascii") as stream:
        for line in stream:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if lines:
                lines.append(stripped)
                continue
            key, *values = stripped.split()
            if key == "FIELDS":
                fields = values
            elif key == "POINTS":
                points = int(values[0])
            elif key == "DATA":
                if values != ["ascii"]:
                    raise ValueError("map preview supports ASCII PCD only")
                lines.append("")
    if fields is None or not {"x", "y", "z"}.issubset(fields) or points is None:
        raise ValueError("PCD header does not define x/y/z and POINTS")
    rows = [row for row in lines[1:] if row]
    if len(rows) != points:
        raise ValueError(f"PCD point count mismatch: header={points}, rows={len(rows)}")
    matrix = np.array([[float(value) for value in row.split()] for row in rows], dtype=np.float32)
    indices = [fields.index(axis) for axis in ("x", "y", "z")]
    xyz = matrix[:, indices]
    if not np.isfinite(xyz).all():
        raise ValueError("PCD contains non-finite coordinates")
    return xyz


def read_ascii_colored_ply(path: Path) -> tuple[np.ndarray, np.ndarray]:
    properties = []
    vertex_count = None
    rows = []
    in_header = True
    with path.open(encoding="ascii") as stream:
        for line in stream:
            values = line.strip().split()
            if in_header:
                if values[:2] == ["format", "ascii"] and values[2:] != ["1.0"]:
                    raise ValueError("unsupported ASCII PLY version")
                if values[:2] == ["element", "vertex"]:
                    vertex_count = int(values[2])
                elif values[:1] == ["property"] and vertex_count is not None:
                    properties.append(values[-1])
                elif values[:1] == ["end_header"]:
                    in_header = False
                continue
            if values:
                rows.append(values)
    required = ("x", "y", "z", "red", "green", "blue")
    if vertex_count is None or not set(required).issubset(properties):
        raise ValueError("colored PLY does not define vertex x/y/z/red/green/blue")
    if len(rows) != vertex_count:
        raise ValueError(f"PLY vertex count mismatch: header={vertex_count}, rows={len(rows)}")
    matrix = np.asarray(rows)
    xyz = np.column_stack(
        [matrix[:, properties.index(axis)].astype(np.float32) for axis in ("x", "y", "z")]
    )
    rgb = np.column_stack(
        [matrix[:, properties.index(channel)].astype(np.uint8) for channel in ("red", "green", "blue")]
    )
    if not np.isfinite(xyz).all():
        raise ValueError("colored PLY contains non-finite coordinates")
    return xyz, rgb


def _load_occupancy(directory: Path) -> tuple[np.ndarray, float, float, float, float]:
    metadata = yaml.safe_load((directory / "map_2d.yaml").read_text(encoding="utf-8"))
    image_path = directory / metadata["image"]
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None or image.size == 0:
        raise ValueError(f"cannot read occupancy image {image_path}")
    image = np.flipud(image)
    cells = np.full(image.shape, -1, dtype=np.int8)
    negate = int(metadata.get("negate", 0))
    probability = image.astype(np.float64) / 255.0 if negate else (255.0 - image) / 255.0
    cells[probability > float(metadata.get("occupied_thresh", 0.65))] = 100
    cells[probability < float(metadata.get("free_thresh", 0.196))] = 0
    origin = metadata.get("origin", [0.0, 0.0, 0.0])
    return cells, float(metadata["resolution"]), float(origin[0]), float(origin[1]), float(origin[2])


def _load_trajectory(path: Path) -> list[tuple[float, float, float, float]]:
    poses = []
    if not path.is_file():
        return poses
    for number, line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        if not line.strip():
            continue
        values = [float(value) for value in line.split()]
        if len(values) != 8:
            raise ValueError(f"trajectory line {number} must contain 8 TUM fields")
        stamp, x, y, _z, _qx, _qy, qz, qw = values
        yaw = 2.0 * np.arctan2(qz, qw)
        poses.append((stamp, x, y, float(yaw)))
    return poses


def load_map_version(directory: str | Path) -> LoadedMap:
    root = Path(directory).expanduser().resolve()
    if not root.is_dir():
        raise ValueError(f"map version is not a directory: {root}")
    points = read_ascii_pcd(root / "map_geometry.pcd")
    colors = None
    colored_path = root / "map_colored.ply"
    if colored_path.is_file():
        colored_points, colors = read_ascii_colored_ply(colored_path)
        if colored_points.shape != points.shape or not np.allclose(colored_points, points, atol=1e-4):
            raise ValueError("map_colored.ply geometry does not match map_geometry.pcd")
    cells, resolution, origin_x, origin_y, origin_yaw = _load_occupancy(root)
    quality = json.loads((root / "quality_report.json").read_text(encoding="utf-8"))
    keys = ("point_count", "loop_closure_position_error_m", "trajectory_rmse_m", "hardware_validated")
    summary = " | ".join(f"{key}={quality.get(key, 'N/A')}" for key in keys)
    files = [name for name in REQUIRED_PRODUCTS + OPTIONAL_PRODUCTS if (root / name).is_file()]
    return LoadedMap(
        root,
        points,
        colors,
        cells,
        resolution,
        origin_x,
        origin_y,
        origin_yaw,
        _load_trajectory(root / "trajectory.tum"),
        files,
        summary,
    )
