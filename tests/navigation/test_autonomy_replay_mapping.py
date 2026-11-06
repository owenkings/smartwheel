"""Offline replay mapping: hold-out evaluation and missing-bin semantics on synthetic scans."""
import math

import numpy as np
import pytest

from wc_navigation.autonomy.replay_mapping import ANGLES, planar, replay


def room_scan(pose, size=(8.0, 5.0)):
    """Ranges from a sensor pose to the walls of an axis-aligned room [0,w]x[0,h]."""
    x, y, yaw = pose
    width, height = size
    ranges = np.full(len(ANGLES), np.inf)
    for k, angle in enumerate(ANGLES):
        dx, dy = math.cos(yaw + angle), math.sin(yaw + angle)
        hits = []
        if dx > 1e-9:
            hits.append((width - x) / dx)
        if dx < -1e-9:
            hits.append(-x / dx)
        if dy > 1e-9:
            hits.append((height - y) / dy)
        if dy < -1e-9:
            hits.append(-y / dy)
        distance = min(hits)
        if 0.3 <= distance <= 8.0:
            ranges[k] = distance
    return ranges


def trajectory(frames=60):
    for k in range(frames):
        t = k / (frames - 1)
        yield (1.5 + 4.0 * t, 2.5 + 0.5 * math.sin(3 * t), 0.4 * math.sin(4 * t))


def test_consistent_odometry_gives_small_holdout_distances():
    scans = [(k, pose, room_scan(pose)) for k, pose in enumerate(trajectory())]
    for matching in (False, True):
        grid, rows, evaluation = replay(scans, scan_matching=matching, holdout_every=5)
        stats = evaluation["endpoint_to_map_m"]
        assert evaluation["holdout_frames"] == 12 and stats["median"] < 0.04 and stats["within_0_10_m"] > 0.9
        assert sum(1 for row in rows if row[1] == "HOLDOUT") == 12
    assert {row[1] for row in rows} - {"HOLDOUT"} <= {"CORRECTED", "SKIPPED_EMPTY_MAP", "SKIPPED_DEGENERATE",
                                                      "REJECTED_FEW_CORRESPONDENCES", "REJECTED_RESIDUAL",
                                                      "REJECTED_LARGE_CORRECTION", "SKIPPED_FEW_LINE_POINTS"}


def test_missing_bins_never_clear_space():
    pose = (2.0, 2.5, 0.0)
    ranges = np.full(len(ANGLES), np.inf)
    ranges[80] = 2.0
    # Three observations: a free cell needs log-odds <= -1.0 at -0.4 per miss.
    grid, _, _ = replay([(k, pose, ranges) for k in range(3)], scan_matching=False, holdout_every=0)
    data = grid.trinary()
    col = int(math.floor((5.0 - grid.origin[0]) / grid.resolution))
    row = int(math.floor((2.5 + 1.0 - grid.origin[1]) / grid.resolution))
    assert data[row, col] == -1                      # an empty bin direction stays unknown
    assert (data == 0).sum() > 0                     # the single return still clears its own ray


def test_planar_pose_from_transform():
    yaw = 0.7
    transform = np.eye(4)
    transform[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    transform[:3, 3] = [1.0, -2.0, 0.5]
    assert np.allclose(planar(transform), (1.0, -2.0, yaw))


def test_bin_scan_reproduces_the_amcl_scan_and_refines_it():
    from wc_navigation.amcl_replay import build_scan
    from wc_navigation.autonomy.replay_mapping import bin_scan
    rng = np.random.default_rng(3)
    points = np.column_stack([rng.uniform(-1, 9, 4000), rng.uniform(-8, 8, 4000), rng.uniform(-0.8, 0.8, 4000)])
    points[:5] = np.nan
    reference, _ = build_scan(points, -0.5, 0.5)
    ranges, angles = bin_scan(points, (-0.5, 0.5), 0.75)
    assert np.allclose(angles, ANGLES) and np.array_equal(np.isinf(ranges), np.isinf(reference))
    assert np.allclose(ranges[np.isfinite(ranges)], reference[np.isfinite(reference)], rtol=1e-6)
    fine, fine_angles = bin_scan(points, (-0.5, 0.5), 0.25)
    assert len(fine) == 481 and fine_angles[0] == pytest.approx(-math.pi / 3)
    assert np.isfinite(fine).sum() > np.isfinite(ranges).sum()
    with pytest.raises(ValueError):
        bin_scan(points, (-0.5, 0.5), 0.3)


def test_unknown_pockets_count_enclosed_pinholes_only():
    from wc_navigation.autonomy.occupancy import LogOddsGrid
    from wc_navigation.autonomy.replay_mapping import unknown_pockets
    grid = LogOddsGrid(resolution=0.05, size_m=10.0)
    grid.logodds[60:140, 60:140] = -2.0              # an observed free square
    grid.logodds[100, 100] = 0.0                     # one unobserved pinhole inside it
    grid.logodds[80:95, 120:135] = 0.0               # a larger enclosed unknown region (225 cells)
    pockets = unknown_pockets(grid)
    assert pockets["enclosed_components"] == 2 and pockets["pinholes"] == 1 and pockets["pinhole_cells"] == 1
