"""Independent ray/plane intersections for synthetic regression, never real data."""
import numpy as np
from scipy.spatial.transform import Rotation


TRUE_R = Rotation.from_euler('xyz', [.8, -.6, 1.2], degrees=True).as_matrix()
TRUE_T = np.array([.018, -.462, .012])


def make_capture(identity='synthetic_train', role='train', seed=17, *, width=120, height=60,
                 wall_distances=(2.8, 1.65, .65), noise=.004, frame_count=12, directions=3):
    rng = np.random.default_rng(seed)
    horizontal, vertical = np.meshgrid(np.linspace(np.radians(60), -np.radians(60), width),
                                      np.linspace(np.radians(22), -np.radians(22), height))
    rays = np.stack([np.cos(vertical)*np.cos(horizontal),
                     np.cos(vertical)*np.sin(horizontal), np.sin(vertical)], axis=-1).reshape(-1, 3)
    planes = [(np.array([-1., 0, 0]), wall_distances[0]),
              (np.array([0., -1, 0]), wall_distances[1]),
              (np.array([0., 0, 1]), wall_distances[2])][:directions]
    source = {'capture_id': identity, 'role': role, 'source_mode': 'synthetic', 'units': 'm',
              'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'},
              'sensor_ids': {'left': 'SYNTHETIC_LEFT', 'right': 'SYNTHETIC_RIGHT'},
              'width': width, 'height': height, 'clouds': {}, 'frames': {},
              'provenance': {'kind': 'ANALYTICAL_RAY_PLANE_INTERSECTION_SYNTHETIC_ONLY',
                             'physical_static_independently_verified': False,
                             'hardware_synchronization_claimed': False}}
    for side in ('left', 'right'):
        origin = np.zeros(3) if side == 'left' else TRUE_T
        directions_left = rays if side == 'left' else rays @ TRUE_R.T
        intersection = []
        for normal, d in planes:
            denominator = directions_left @ normal
            with np.errstate(divide='ignore', invalid='ignore'):
                distance = -(normal @ origin + d)/denominator
            distance[(distance <= 0) | ~np.isfinite(distance)] = np.inf
            intersection.append(distance)
        ranges = np.min(intersection, axis=0)
        ranges[ranges > 9] = np.nan
        frames = rays[None, :, :] * (ranges[None, :] + rng.normal(0, noise, (frame_count, len(rays))))[:, :, None]
        source['frames'][side] = frames
        median = rays * np.nanmedian(np.linalg.norm(frames, axis=2), axis=0)[:, None]
        source['clouds'][side] = [[float(v) if np.isfinite(v) else None for v in p] for p in median]
    return source
