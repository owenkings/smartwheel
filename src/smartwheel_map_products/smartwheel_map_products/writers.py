import csv
import hashlib
import json
import math
import os
import shutil
import tempfile
from contextlib import contextmanager
from pathlib import Path

import cv2
import numpy as np
import yaml

from smartwheel_map_products.formal_acceptance import (
    template as formal_acceptance_template,
    validate_hardware_evidence_binding,
)
from smartwheel_map_products.occupancy import OccupancyGridData


def _fsync_directory(directory: Path) -> None:
    """Persist directory entries when the platform exposes directory fsync."""

    if os.name == "nt":
        return
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    try:
        descriptor = os.open(str(directory), flags)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


@contextmanager
def _atomic_path(path: Path, mode: str, encoding: str | None = None):
    """Yield a temporary file stream and atomically publish it on success."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=str(path.parent),
    )
    temporary = Path(temporary_name)
    stream = None
    try:
        if "b" in mode:
            stream = os.fdopen(descriptor, mode)
        else:
            stream = os.fdopen(descriptor, mode, encoding=encoding or "utf-8")
        descriptor = -1
        yield stream
        stream.flush()
        os.fsync(stream.fileno())
        stream.close()
        stream = None
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    except BaseException:
        if stream is not None:
            stream.close()
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
        raise


@contextmanager
def _atomic_binary_path(path: Path):
    with _atomic_path(path, "wb") as stream:
        yield stream


def _atomic_copy(source: Path, destination: Path) -> None:
    with _atomic_binary_path(destination) as stream:
        with source.open("rb") as input_stream:
            shutil.copyfileobj(input_stream, stream)


def publish_latest_path(output_root: str | Path, directory: str | Path) -> Path:
    """Publish the latest completed bundle pointer atomically."""

    root = Path(output_root)
    root.mkdir(parents=True, exist_ok=True)
    pointer = root / "latest_path.txt"
    with _atomic_path(pointer, "w", encoding="utf-8") as stream:
        stream.write(str(Path(directory).resolve()) + "\n")
    return pointer


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


def _validated_xyz(points: np.ndarray) -> np.ndarray:
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if not np.isfinite(xyz).all():
        raise ValueError("points must be finite")
    return xyz


def write_pcd(
    path: Path,
    points: np.ndarray,
    intensity: np.ndarray | None = None,
) -> None:
    xyz = _validated_xyz(points)
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
    def write(stream) -> None:
        stream.write(header)
        if amplitudes is None:
            np.savetxt(stream, xyz, fmt="%.6f %.6f %.6f")
        else:
            np.savetxt(
                stream,
                np.column_stack((xyz, amplitudes)),
                fmt="%.6f %.6f %.6f %.6f",
            )
    with _atomic_path(path, "w", encoding="ascii") as stream:
        write(stream)


def write_ply(
    path: Path,
    points: np.ndarray,
    colors: np.ndarray | None = None,
    intensity: np.ndarray | None = None,
) -> None:
    xyz = _validated_xyz(points)
    amplitudes = _validated_intensity(xyz, intensity)
    rgb = None
    if colors is not None:
        rgb = np.asarray(colors, dtype=np.uint8)
        if rgb.shape != xyz.shape:
            raise ValueError("colors must match points")
    def write(stream) -> None:
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
    with _atomic_path(path, "w", encoding="ascii") as stream:
        write(stream)


def _map_image(cells: np.ndarray) -> np.ndarray:
    image = np.full(cells.shape, 205, dtype=np.uint8)
    image[cells == 0] = 254
    image[cells == 100] = 0
    return np.flipud(image)


def write_occupancy(directory: Path, grid: OccupancyGridData) -> None:
    image = _map_image(grid.cells)
    with _atomic_binary_path(directory / "map_2d.pgm") as stream:
        stream.write(f"P5\n{grid.width} {grid.height}\n255\n".encode("ascii"))
        stream.write(image.tobytes())
    png_path = directory / "map_2d.png"
    descriptor, temporary_name = tempfile.mkstemp(
        # OpenCV selects the encoder from the temporary path extension.  Keep
        # the final suffix as `.png` while the hidden prefix still marks the
        # file as an in-progress artifact until the atomic rename.
        prefix=f".{png_path.name}.", suffix=".png", dir=str(png_path.parent)
    )
    os.close(descriptor)
    temporary_png = Path(temporary_name)
    try:
        if not cv2.imwrite(str(temporary_png), image):
            raise RuntimeError("failed to write map_2d.png")
        try:
            with temporary_png.open("r+b") as stream:
                os.fsync(stream.fileno())
        except OSError:
            # Some Windows file handles do not permit fsync after an external
            # encoder has closed and reopened the path; the atomic rename still
            # prevents a partially written final filename.
            pass
        os.replace(temporary_png, png_path)
        _fsync_directory(png_path.parent)
    finally:
        temporary_png.unlink(missing_ok=True)
    metadata = {
        "image": "map_2d.pgm",
        "resolution": grid.resolution,
        "origin": [grid.origin_x, grid.origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with _atomic_path(directory / "map_2d.yaml", "w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)


def _trajectory_sample(sample) -> tuple[float, float, float, float, float, float, float, float]:
    """Normalize legacy planar or full 6-DoF trajectory samples.

    The historical exporter accepted ``(stamp, x, y, yaw)``.  Formal RTAB-Map
    products use ``(stamp, x, y, z, qx, qy, qz, qw)`` so the optimized graph's
    roll, pitch and height are not silently replaced with zeros.
    """

    values = tuple(float(value) for value in sample)
    if len(values) == 4:
        stamp, x, y, yaw = values
        z = qx = qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
    elif len(values) == 8:
        stamp, x, y, z, qx, qy, qz, qw = values
    else:
        raise ValueError("trajectory samples must contain 4 planar or 8 full-pose values")
    result = (stamp, x, y, z, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("trajectory samples must be finite")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1.0e-9:
        raise ValueError("trajectory quaternion must be non-zero")
    return stamp, x, y, z, qx / norm, qy / norm, qz / norm, qw / norm


def _rpy_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    roll = math.atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    pitch_term = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, pitch_term)))
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return roll, pitch, yaw


def write_trajectory(directory: Path, poses) -> None:
    normalized = [_trajectory_sample(sample) for sample in poses]
    if any(current[0] <= previous[0] for previous, current in zip(normalized, normalized[1:])):
        raise ValueError("trajectory timestamps must be strictly increasing")
    with _atomic_path(directory / "trajectory.tum", "w", encoding="ascii") as tum:
        for stamp, x, y, z, qx, qy, qz, qw in normalized:
            tum.write(
                f"{stamp:.9f} {x:.6f} {y:.6f} {z:.6f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )
    with _atomic_path(directory / "poses.csv", "w", encoding="ascii") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(("timestamp", "x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"))
        for stamp, x, y, z, qx, qy, qz, qw in normalized:
            roll, pitch, yaw = _rpy_from_quaternion(qx, qy, qz, qw)
            writer.writerow(
                (
                    f"{stamp:.9f}",
                    f"{x:.6f}",
                    f"{y:.6f}",
                    f"{z:.6f}",
                    f"{roll:.9f}",
                    f"{pitch:.9f}",
                    f"{yaw:.9f}",
                )
            )


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
    trajectory_poses=None,
    calibration_contract_path: str = "",
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
    manifest_path = output / "manifest.json"
    incomplete_path = output / ".incomplete"
    if manifest_path.exists():
        raise FileExistsError(f"map bundle is already complete: {output}")
    existing_symlinks = [path for path in output.rglob("*") if path.is_symlink()]
    if existing_symlinks:
        raise ValueError(
            f"refusing map bundle containing a symlink: {existing_symlinks[0]}"
        )
    existing_files = [
        path
        for path in output.rglob("*")
        if path.is_file()
        and path.name != ".incomplete"
        and path.name != "rtabmap.db"
        and "raw_bag" not in path.relative_to(output).parts
    ]
    if existing_files and not incomplete_path.exists():
        raise FileExistsError(
            f"refusing to overwrite non-empty map bundle without {incomplete_path.name}: {output}"
        )
    with _atomic_path(incomplete_path, "w", encoding="ascii") as stream:
        stream.write("map export in progress\n")
    (output / "logs").mkdir(exist_ok=True)
    _fsync_directory(output)
    try:
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
        write_trajectory(output, poses if trajectory_poses is None else trajectory_poses)
        profile_source = Path(hardware_profile_path)
        if not profile_source.is_file():
            raise FileNotFoundError(f"hardware profile does not exist: {profile_source}")
        _atomic_copy(profile_source, output / "hardware_profile_used.yaml")
        if calibration_contract_path:
            contract_source = Path(calibration_contract_path).expanduser()
            if not contract_source.is_file():
                raise FileNotFoundError(
                    f"calibration contract does not exist: {contract_source}"
                )
            _atomic_copy(contract_source, output / "calibration_contract_used.json")
        with _atomic_path(output / "algorithm_profile_used.yaml", "w", encoding="utf-8") as stream:
            yaml.safe_dump(algorithm_profile, stream, sort_keys=True)
        with _atomic_path(output / "bag_path.txt", "w", encoding="utf-8") as stream:
            stream.write((bag_path or "NOT_RECORDED") + "\n")
        with _atomic_path(output / "quality_report.json", "w", encoding="utf-8") as stream:
            stream.write(quality_json)
        formal_evidence = quality.get("formal_acceptance")
        if not isinstance(formal_evidence, dict):
            formal_evidence = formal_acceptance_template()
        with _atomic_path(output / "formal_acceptance.json", "w", encoding="utf-8") as stream:
            json.dump(formal_evidence, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        if quality.get("hardware_validated") is True:
            provenance_ok, provenance_reason = validate_hardware_evidence_binding(
                formal_evidence,
                hardware_profile_path=output / "hardware_profile_used.yaml",
                calibration_contract_path=output
                / "calibration_contract_used.json",
            )
            if not provenance_ok:
                raise ValueError(
                    "copied formal provenance snapshot is invalid: "
                    + provenance_reason
                )
        with _atomic_path(output / "quality_report.md", "w", encoding="utf-8") as stream:
            stream.write(quality_markdown)
    except BaseException:
        _fsync_directory(output)
        raise

    files = []
    externally_managed_files = []
    for path in sorted(output.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"refusing to include a symlink in map manifest: {path}")
        if path.is_file() and path.name != "manifest.json":
            relative = path.relative_to(output)
            if path.name in ("manifest.json", ".incomplete") or "raw_bag" in relative.parts:
                continue
            if path.name == "rtabmap.db":
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
    # Publish the manifest while the in-progress marker still exists.  If the
    # process dies between these operations, validators can distinguish an
    # interrupted export and a complete bundle instead of leaving an orphaned
    # directory that appears resumable but has no marker.
    with _atomic_path(manifest_path, "w", encoding="utf-8") as stream:
        stream.write(manifest_json)
    incomplete_path.unlink(missing_ok=True)
    _fsync_directory(output)
    return output
