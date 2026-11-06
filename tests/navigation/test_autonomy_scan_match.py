"""Bounded scan-to-map correction: recovery, degeneracy, rejection."""
import math

import numpy as np
import pytest

from wc_navigation.autonomy.occupancy import LogOddsGrid
from wc_navigation.autonomy.scan_match import IcpConfig, correct_pose, map_points, scan_normals, scan_points


def wall_points(segments, spacing=0.05):
    points = []
    for (x0, y0), (x1, y1) in segments:
        n = int(round(math.hypot(x1 - x0, y1 - y0) / spacing)) + 1
        points.append(np.column_stack([np.linspace(x0, x1, n), np.linspace(y0, y1, n)]))
    return np.vstack(points)


def synthetic_scan(segments, pose, beams=160, half_fov=1.0, max_range=8.0, noise=0.003, seed=0):
    """Ranges from ``pose`` to line segments; returns sensor-frame endpoints."""
    rng = np.random.default_rng(seed)
    x, y, yaw = pose
    angles = np.linspace(-half_fov, half_fov, beams)
    ranges = np.full(beams, np.inf)
    for k, angle in enumerate(angles):
        dx, dy = math.cos(yaw + angle), math.sin(yaw + angle)
        for (ax, ay), (bx, by) in segments:
            ex, ey = bx - ax, by - ay
            det = dx * (-ey) - dy * (-ex)
            if abs(det) < 1e-12:
                continue
            t = ((ax - x) * (-ey) - (ay - y) * (-ex)) / det
            u = (dx * (ay - y) - dy * (ax - x)) / det
            if 0.0 < t <= max_range and 0.0 <= u <= 1.0:
                ranges[k] = min(ranges[k], t)
    finite = np.isfinite(ranges)
    ranges[finite] += rng.normal(0.0, noise, int(finite.sum()))
    return scan_points(angles, ranges, 0.3, max_range, 400)


ROOM = [((-1.0, 2.0), (3.0, 2.0)), ((3.0, 2.0), (3.0, -2.0)), ((3.0, -2.0), (-1.0, -2.0))]
CORRIDOR = [((-10.0, 1.2), (10.0, 1.2)), ((-10.0, -1.2), (10.0, -1.2))]


def test_recovers_pose_from_offset_prior_in_a_room():
    truth = (0.2, -0.1, 0.25)
    points = synthetic_scan(ROOM, truth)
    prior = (truth[0] + 0.05, truth[1] - 0.04, truth[2] + math.radians(1.5))
    result = correct_pose(wall_points(ROOM), prior, points)
    assert result["status"] == "CORRECTED"
    x, y, yaw = result["pose"]
    assert math.hypot(x - truth[0], y - truth[1]) < 0.01
    assert abs(yaw - truth[2]) < math.radians(0.2)
    assert result["rms_m"] < 0.01 and result["correspondences"] >= 30


def test_unconstrained_corridor_direction_keeps_the_odometry_prediction():
    truth = (0.0, 0.0, 0.0)
    points = synthetic_scan(CORRIDOR, truth)
    prior = (0.10, 0.05, math.radians(1.0))
    result = correct_pose(wall_points(CORRIDOR), prior, points)
    assert result["status"] == "CORRECTED" and result["constrained_directions"] == 2
    x, y, yaw = result["pose"]
    assert abs(y - truth[1]) < 0.01 and abs(yaw - truth[2]) < math.radians(0.2)
    assert abs(x - prior[0]) < 1e-9          # along the corridor the prediction is kept exactly
    eigenvalues = sorted(result["scan_information_eigenvalues"])
    assert eigenvalues[0] < 1e-3 * eigenvalues[-1]


def test_points_on_one_wall_do_not_pull_along_a_perpendicular_wall():
    # Mapped: only a wall at x = 2.0. Scanned: a wall along y = 0.1 crossing near it.
    map_xy = wall_points([((2.0, -1.0), (2.0, 1.0))])
    segment = [((1.0, 0.1), (3.0, 0.1))]
    truth = (0.0, -1.0, math.atan2(1.1, 2.0))
    points = synthetic_scan(segment, truth, half_fov=0.35)
    result = correct_pose(map_xy, truth, points)
    assert result["status"] == "REJECTED_FEW_CORRESPONDENCES" and result["pose"] == list(truth)


def test_scan_normals_follow_lines_and_skip_gaps():
    line = np.column_stack([np.linspace(1.0, 2.0, 21), np.full(21, 0.5)])
    normals = scan_normals(line, 2, 0.25, 0.3)
    inner = normals[2:-2]
    assert np.allclose(np.abs(inner[:, 1]), 1.0, atol=1e-6) and np.isnan(normals[:2]).all()
    gapped = line.copy()
    gapped[10:, 0] += 1.0
    assert np.isnan(scan_normals(gapped, 2, 0.25, 0.3)[8:12]).all()


def test_large_offsets_are_rejected_and_prediction_kept():
    truth = (0.0, 0.0, 0.0)
    points = synthetic_scan(ROOM, truth)
    prior = (0.3, 0.0, math.radians(8.0))
    result = correct_pose(wall_points(ROOM), prior, points)
    assert result["status"].startswith("REJECTED")
    assert result["pose"] == list(prior)


def test_empty_map_and_sparse_scans_are_skipped():
    points = synthetic_scan(ROOM, (0.0, 0.0, 0.0))
    assert correct_pose(np.zeros((0, 2)), (0, 0, 0), points)["status"] == "SKIPPED_EMPTY_MAP"
    assert correct_pose(wall_points(ROOM), (0, 0, 0), points[:5])["status"] == "SKIPPED_FEW_POINTS"


def test_scan_far_from_mapped_structure_is_rejected():
    points = synthetic_scan(ROOM, (0.0, 0.0, 0.0))
    far_map = wall_points([((20.0, 20.0), (25.0, 20.0))])
    result = correct_pose(far_map, (0.0, 0.0, 0.0), points)
    assert result["status"] == "REJECTED_FEW_CORRESPONDENCES" and result["pose"] == [0.0, 0.0, 0.0]


def test_map_points_reads_occupied_cells_near_the_sensor():
    grid = LogOddsGrid(resolution=0.05, size_m=20.0)
    for _ in range(3):
        grid.integrate((0.0, 0.0, 0.0), np.linspace(-0.5, 0.5, 41), np.full(41, 2.0), 0.3, 8.0)
    near = map_points(grid, (0.0, 0.0), 3.0)
    assert len(near) >= 20 and np.all(np.hypot(near[:, 0], near[:, 1]) > 1.9)
    assert len(map_points(grid, (8.0, 8.0), 1.0)) == 0


@pytest.mark.parametrize("kwargs", [{"iterations": 1}, {"prior_sigma_xy_m": 0}, {"max_rms_m": -1},
                                    {"final_correspondence_m": 0.5}, {"min_normal_agreement": 1.5},
                                    {"scan_normal_window": 0}])
def test_invalid_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        IcpConfig(**kwargs)
