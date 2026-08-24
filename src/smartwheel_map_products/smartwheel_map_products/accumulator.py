import numpy as np


class VoxelAccumulator:
    def __init__(self, voxel_size_m: float, maximum_points: int) -> None:
        if voxel_size_m <= 0.0:
            raise ValueError("voxel_size_m must be positive")
        if maximum_points <= 0:
            raise ValueError("maximum_points must be positive")
        self._voxel_size = float(voxel_size_m)
        self._maximum_points = int(maximum_points)
        self._samples: dict[
            tuple[int, int, int],
            tuple[np.ndarray, np.ndarray, float | None],
        ] = {}

    def add(
        self,
        points: np.ndarray,
        origins: np.ndarray,
        intensity: np.ndarray | None = None,
    ) -> int:
        xyz = np.asarray(points, dtype=np.float64)
        ray_origins = np.asarray(origins, dtype=np.float64)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or ray_origins.shape != xyz.shape:
            raise ValueError("points and origins must have matching Nx3 shapes")
        if not np.isfinite(xyz).all() or not np.isfinite(ray_origins).all():
            raise ValueError("points and origins must be finite")
        amplitudes = None
        if intensity is not None:
            amplitudes = np.asarray(intensity, dtype=np.float32).reshape(-1)
            if amplitudes.shape[0] != xyz.shape[0]:
                raise ValueError("intensity length must match points")
            if not np.isfinite(amplitudes).all():
                raise ValueError("intensity must be finite")
        if xyz.shape[0] == 0:
            return 0

        keys = np.floor(xyz / self._voxel_size).astype(np.int64)
        _, first = np.unique(keys, axis=0, return_index=True)
        additions = [
            (tuple(int(value) for value in keys[index]), index)
            for index in first
            if tuple(int(value) for value in keys[index]) not in self._samples
        ]
        if len(self._samples) + len(additions) > self._maximum_points:
            raise OverflowError(
                f"voxel accumulator would exceed configured limit {self._maximum_points}"
            )
        for key, index in additions:
            amplitude = None if amplitudes is None else float(amplitudes[index])
            self._samples[key] = (
                xyz[index].copy(),
                ray_origins[index].copy(),
                amplitude,
            )
        return len(additions)

    def arrays(self) -> tuple[np.ndarray, np.ndarray]:
        if not self._samples:
            empty = np.empty((0, 3), dtype=np.float64)
            return empty, empty.copy()
        points, origins, _ = zip(*self._samples.values())
        return np.asarray(points), np.asarray(origins)

    def arrays_with_intensity(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        points, origins = self.arrays()
        if not self._samples:
            return points, origins, None
        amplitudes = [sample[2] for sample in self._samples.values()]
        if any(amplitude is None for amplitude in amplitudes):
            return points, origins, None
        return points, origins, np.asarray(amplitudes, dtype=np.float32)

    def __len__(self) -> int:
        return len(self._samples)
