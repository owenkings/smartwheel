from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    image_rgb: np.ndarray
    intrinsic: np.ndarray
    camera_from_map: np.ndarray
    stamp_sec: float = 0.0

    def __post_init__(self) -> None:
        if self.image_rgb.ndim != 3 or self.image_rgb.shape[2] != 3:
            raise ValueError("camera image must be HxWx3 RGB")
        if self.intrinsic.shape != (3, 3) or self.camera_from_map.shape != (4, 4):
            raise ValueError("invalid camera matrix dimensions")


def _image_quality(image: np.ndarray) -> tuple[float, float]:
    gray = image.astype(np.float64).mean(axis=2)
    if min(gray.shape) > 1:
        gradient = np.mean(np.abs(np.diff(gray, axis=0))) + np.mean(np.abs(np.diff(gray, axis=1)))
    else:
        gradient = 0.0
    sharpness = min(1.0, 0.1 + gradient / 32.0)
    mean = float(gray.mean()) / 255.0
    exposure = max(0.1, 1.0 - abs(mean - 0.5) * 1.8)
    return sharpness, exposure


def colorize_points(
    points_map: np.ndarray,
    frames: list[CameraFrame],
    default_color=(128, 128, 128),
    occlusion_tolerance_m: float = 0.08,
) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points_map, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_map must have shape (N, 3)")
    colors = np.tile(np.asarray(default_color, dtype=np.uint8), (points.shape[0], 1))
    best_score = np.full(points.shape[0], -np.inf, dtype=np.float64)
    colored = np.zeros(points.shape[0], dtype=bool)
    homogeneous = np.column_stack((points, np.ones(points.shape[0])))
    for frame in frames:
        camera_points = (homogeneous @ frame.camera_from_map.T)[:, :3]
        depth = camera_points[:, 2]
        in_front = depth > 1e-4
        if not np.any(in_front):
            continue
        projection = camera_points @ frame.intrinsic.T
        u = np.rint(projection[:, 0] / np.maximum(depth, 1e-9)).astype(np.int64)
        v = np.rint(projection[:, 1] / np.maximum(depth, 1e-9)).astype(np.int64)
        height, width = frame.image_rgb.shape[:2]
        valid = in_front & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        candidates = np.nonzero(valid)[0]
        if candidates.size == 0:
            continue
        pixel_id = v[candidates] * width + u[candidates]
        nearest = {}
        for index, pixel in zip(candidates, pixel_id):
            nearest[pixel] = min(nearest.get(pixel, np.inf), depth[index])
        visible = np.array(
            [index for index, pixel in zip(candidates, pixel_id) if depth[index] <= nearest[pixel] + occlusion_tolerance_m],
            dtype=np.int64,
        )
        distance = np.linalg.norm(camera_points[visible], axis=1)
        view_cosine = depth[visible] / np.maximum(distance, 1e-9)
        sharpness, exposure = _image_quality(frame.image_rgb)
        score = view_cosine * sharpness * exposure / np.maximum(distance, 0.25)
        better = score > best_score[visible]
        selected = visible[better]
        if selected.size:
            colors[selected] = frame.image_rgb[v[selected], u[selected]]
            best_score[selected] = score[better]
            colored[selected] = True
    return colors, colored

