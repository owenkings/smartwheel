import math
from dataclasses import dataclass

import numpy as np

from smartwheel_sensor_api.pointcloud import transform_matrix


def _grid(values_min: float, values_max: float, spacing: float) -> np.ndarray:
    count = max(2, int(math.ceil((values_max - values_min) / spacing)) + 1)
    return np.linspace(values_min, values_max, count)


def _wall_x(x: float, y0: float, y1: float, height: float, spacing: float) -> np.ndarray:
    y, z = np.meshgrid(_grid(y0, y1, spacing), _grid(0.0, height, spacing))
    return np.column_stack((np.full(y.size, x), y.ravel(), z.ravel()))


def _wall_y(y: float, x0: float, x1: float, height: float, spacing: float) -> np.ndarray:
    x, z = np.meshgrid(_grid(x0, x1, spacing), _grid(0.0, height, spacing))
    return np.column_stack((x.ravel(), np.full(x.size, y), z.ravel()))


def _box(cx: float, cy: float, sx: float, sy: float, height: float, spacing: float) -> np.ndarray:
    return np.vstack(
        (
            _wall_x(cx - sx / 2.0, cy - sy / 2.0, cy + sy / 2.0, height, spacing),
            _wall_x(cx + sx / 2.0, cy - sy / 2.0, cy + sy / 2.0, height, spacing),
            _wall_y(cy - sy / 2.0, cx - sx / 2.0, cx + sx / 2.0, height, spacing),
            _wall_y(cy + sy / 2.0, cx - sx / 2.0, cx + sx / 2.0, height, spacing),
        )
    )


def _column(cx: float, cy: float, radius: float, height: float, spacing: float) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * math.pi, max(16, int(2.0 * math.pi * radius / spacing)), endpoint=False)
    z = _grid(0.0, height, spacing)
    aa, zz = np.meshgrid(angles, z)
    return np.column_stack((cx + radius * np.cos(aa.ravel()), cy + radius * np.sin(aa.ravel()), zz.ravel()))


class IndoorScene:
    features = (
        "rectangular_room",
        "l_shaped_corridor",
        "doorway",
        "column",
        "table_legs",
        "boxes_with_varied_height",
    )

    def __init__(self, spacing_m: float = 0.14) -> None:
        if spacing_m <= 0.0:
            raise ValueError("scene spacing must be positive")
        room = [
            _wall_x(0.0, 0.0, 6.0, 2.6, spacing_m),
            _wall_y(0.0, 0.0, 8.0, 2.6, spacing_m),
            _wall_y(6.0, 0.0, 8.0, 2.6, spacing_m),
            _wall_x(8.0, 0.0, 2.2, 2.6, spacing_m),
            _wall_x(8.0, 3.8, 6.0, 2.6, spacing_m),
        ]
        corridor = [
            _wall_y(1.8, 8.0, 14.2, 2.6, spacing_m),
            _wall_y(4.2, 8.0, 11.8, 2.6, spacing_m),
            _wall_x(11.8, 4.2, 9.5, 2.6, spacing_m),
            _wall_x(14.2, 1.8, 9.5, 2.6, spacing_m),
            _wall_y(9.5, 11.8, 14.2, 2.6, spacing_m),
        ]
        objects = [
            _column(3.2, 3.0, 0.35, 2.2, spacing_m),
            _box(5.0, 1.8, 0.9, 0.7, 0.8, spacing_m),
            _box(5.7, 4.2, 0.7, 0.7, 1.5, spacing_m),
            _box(12.7, 6.5, 0.8, 0.8, 1.1, spacing_m),
        ]
        for x in (5.8, 6.8):
            for y in (4.5, 5.3):
                objects.append(_box(x, y, 0.12, 0.12, 0.75, spacing_m / 2.0))
        floor_x, floor_y = np.meshgrid(_grid(0.0, 14.2, spacing_m * 2.5), _grid(0.0, 9.5, spacing_m * 2.5))
        floor = np.column_stack((floor_x.ravel(), floor_y.ravel(), np.zeros(floor_x.size)))
        self.points = np.vstack((*room, *corridor, *objects, floor)).astype(np.float64)


@dataclass(frozen=True)
class LidarModel:
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    horizontal_fov_deg: float = 120.0
    vertical_fov_deg: float = 45.0
    min_range_m: float = 0.2
    max_range_m: float = 12.0
    noise_stddev_m: float = 0.008
    max_points: int = 900

    def observe(
        self,
        world_points: np.ndarray,
        base_pose,
        seed: int,
        frame_index: int,
        occluded: bool = False,
    ) -> tuple[np.ndarray, np.ndarray]:
        world_from_base = transform_matrix(
            [base_pose.x, base_pose.y, 0.0], [0.0, 0.0, base_pose.yaw]
        )
        base_from_sensor = transform_matrix(self.xyz, self.rpy)
        world_from_sensor = world_from_base @ base_from_sensor
        rotation = world_from_sensor[:3, :3]
        translation = world_from_sensor[:3, 3]
        sensor_points = (world_points - translation) @ rotation
        ranges = np.linalg.norm(sensor_points, axis=1)
        horizontal = np.arctan2(sensor_points[:, 1], sensor_points[:, 0])
        vertical = np.arctan2(sensor_points[:, 2], np.hypot(sensor_points[:, 0], sensor_points[:, 1]))
        keep = (
            (sensor_points[:, 0] > 0.0)
            & (ranges >= self.min_range_m)
            & (ranges <= self.max_range_m)
            & (np.abs(horizontal) <= math.radians(self.horizontal_fov_deg) / 2.0)
            & (np.abs(vertical) <= math.radians(self.vertical_fov_deg) / 2.0)
        )
        if occluded:
            keep &= ~((horizontal > -0.12) & (horizontal < 0.22) & (vertical < 0.15))
        visible = sensor_points[keep]
        visible_ranges = ranges[keep]
        rng = np.random.default_rng(seed + frame_index * 1009)
        if visible.shape[0] > self.max_points:
            selected = rng.choice(visible.shape[0], self.max_points, replace=False)
            visible = visible[selected]
            visible_ranges = visible_ranges[selected]
        visible = visible + rng.normal(0.0, self.noise_stddev_m, visible.shape)
        intensity = np.clip(255.0 - visible_ranges * 12.0 + visible[:, 2] * 8.0, 0.0, 255.0)
        return visible, intensity.astype(np.float32)

