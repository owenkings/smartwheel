"""Obstacle filter on synthetic prepared clouds: floor, height window, far-floor gate, body, unsupported clusters."""
import math

import numpy as np
import pytest

from wc_navigation.perception.consensus import Witness, move, planar_relative
from wc_navigation.perception.obstacle_cloud import (ObstacleFilter, body_outline, clearing_rays, filter_frame,
                                                     filter_points, grow_convex, inside_convex, scan_ranges)

LEVER = np.array([0.323, 0.22875, 0.5915])          # left lidar in base_link (frozen mount)
FLOOR = -LEVER[2] - 0.185                            # floor z in the lidar frame
MEASURED = [(0.81, 0.2425), (0.2, 0.35), (-0.255, 0.35), (-0.255, -0.35), (0.2, -0.35), (0.81, -0.2425)]


def block(x0, x1, y0, y1, h0, h1, step=0.02):
    """Solid box surface samples in base_link heights above the floor, returned in the lidar frame."""
    xs, ys, hs = np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step), np.arange(h0, h1 + 1e-9, step)
    grid = np.array(np.meshgrid(xs, ys, hs, indexing="ij")).reshape(3, -1).T
    base = np.column_stack([grid[:, 0], grid[:, 1], grid[:, 2] - 0.185])
    return base - LEVER


def floor(x0, x1, y0, y1, step=0.05, height=0.0):
    return block(x0, x1, y0, y1, height, height, step)


def base_of(points):
    return np.asarray(points, dtype=float) + LEVER


def test_floor_and_overhead_are_removed_and_a_box_is_kept():
    cloud = np.vstack([floor(1.5, 3.0, -1, 1), block(1.2, 1.3, -0.1, 0.1, 0.0, 0.35), block(1.0, 1.1, -0.1, 0.1, 1.7, 1.9)])
    kept, stats = filter_points(cloud, LEVER)
    h = base_of(kept)[:, 2] + 0.185
    assert len(kept) and h.min() >= 0.05 - 1e-6 and h.max() <= 1.6
    assert np.all((base_of(kept)[:, 0] > 1.15) & (base_of(kept)[:, 0] < 1.35))
    assert stats["removed_height"] > 0 and stats["kept_points"] > 0


def test_low_returns_count_only_near_the_lidar_but_tall_structure_is_kept_far_away():
    low_far = block(3.0, 3.1, -0.1, 0.1, 0.05, 0.40)          # 2.7 m from the lidar: far-floor zone
    wall = block(3.4, 3.42, -0.5, 0.5, 0.0, 1.2)
    kept, stats = filter_points(np.vstack([low_far, wall]), LEVER)
    b = base_of(kept)
    assert not np.any(b[:, 0] < 3.3), "low points beyond low_max_range_m must be dropped"
    assert np.any(b[:, 2] + 0.185 > 0.5) and (b[:, 2] + 0.185).min() >= 0.5 - 0.05
    assert stats["removed_far_low"] > 0
    near_low = block(1.6, 1.7, 0.1, 0.3, 0.05, 0.30)           # 1.3 m from the lidar: kept
    kept, _ = filter_points(near_low, LEVER)
    assert len(kept) > 0


def test_points_inside_the_body_outline_are_removed():
    knees = block(0.45, 0.60, 0.05, 0.25, 0.50, 0.62)          # occupant's knees above the footrest
    outside = block(0.86, 0.90, -0.1, 0.1, 0.60, 0.70)         # 5 cm ahead of the measured front edge
    kept, stats = filter_points(np.vstack([knees, outside]), LEVER, body=MEASURED)
    b = base_of(kept)
    assert np.all(b[:, 0] > 0.81) and stats["removed_body"] > 0
    assert not inside_convex(b[:, :2], MEASURED).any()


def test_small_unsupported_cluster_is_dropped_but_supported_and_wide_ones_stay():
    ghost = block(2.3, 2.39, 0.2, 0.29, 0.85, 0.91, step=0.03)  # 36 points, nothing beneath, floor in view
    ground = floor(1.6, 3.0, -1.0, 1.4)
    kept, stats = filter_points(np.vstack([ghost, ground]), LEVER)
    assert len(kept) == 0 and stats["removed_floating"] > 0
    person = np.vstack([block(2.3, 2.39, 0.2, 0.29, 0.0, 0.84, step=0.03), ghost])
    kept, stats = filter_points(np.vstack([person, ground]), LEVER)
    assert stats["removed_floating"] == 0 and (base_of(kept)[:, 2] + 0.185).max() > 0.85
    table = block(2.3, 2.34, -0.4, 0.5, 0.72, 0.76)            # 0.9 m wide edge with nothing beneath
    kept, stats = filter_points(np.vstack([table, ground]), LEVER)
    assert stats["removed_floating"] == 0 and len(kept) > 0


def test_cluster_close_to_the_lower_edge_of_the_view_is_not_called_floating():
    # 0.9 m from the lidar the lowest ray is at 0.78 - 0.9 * tan(28 deg) = 0.30 m above the floor
    near = block(1.2, 1.25, 0.2, 0.25, 0.45, 0.50)
    kept, stats = filter_points(near, LEVER)
    assert stats["removed_floating"] == 0 and len(kept) > 0


def test_witness_removes_what_it_saw_through_and_keeps_what_it_also_hit():
    # 90 points: too many for the single-lidar size rule, so only the other lidar can reject it
    ghost = block(2.3, 2.42, 0.15, 0.30, 0.85, 0.91, step=0.03)
    assert 60 < len(ghost) <= 400
    other_lever = np.array([0.41173, -0.23331, 0.55265])
    # the other lidar looks through the ghost to a wall about 6 m away
    target = base_of(ghost).mean(axis=0)
    direction = (target - other_lever) / np.linalg.norm(target - other_lever)
    fan = []
    for dy in np.linspace(-0.12, 0.12, 25):
        for dz in np.linspace(-0.08, 0.08, 17):
            d = direction + np.array([0.0, dy, dz])
            fan.append(other_lever + 6.0 * d / np.linalg.norm(d))
    witness = Witness(np.asarray(fan), other_lever)
    kept, stats = filter_points(ghost, LEVER)
    assert len(kept) > 0 and stats["removed_floating"] == 0
    kept, stats = filter_points(ghost, LEVER, witness=witness)
    assert len(kept) == 0 and stats["removed_contradicted"] > 0
    # the same cluster hit by the other lidar is real, even though it is small
    seen = Witness(base_of(ghost), other_lever)
    kept, stats = filter_points(ghost, LEVER, witness=seen)
    assert stats["removed_contradicted"] == 0 and stats["removed_floating"] == 0 and len(kept) > 0


def test_clearing_rays_stop_short_of_surfaces_and_of_the_floor_layer():
    wall = block(4.0, 4.0, -0.5, 0.5, 0.3, 1.2, step=0.05)
    ground = floor(1.6, 3.5, -0.5, 0.5, step=0.05)
    cloud = np.vstack([wall, ground])
    cfg = ObstacleFilter()
    rays = clearing_rays(cloud, LEVER, None, cfg)
    assert len(rays) > 0
    end_h = rays[:, 2] + LEVER[2] + 0.185
    assert end_h.min() >= cfg.clear_min_end_height_m - 1e-6
    # every ray end lies on the line to a return and at least 0.2 m (or 10 %) before it
    r_end = np.linalg.norm(rays, axis=1)
    r_src = np.linalg.norm(cloud, axis=1)
    unit_src = cloud / r_src[:, None]
    for end, r in zip(rays[::7], r_end[::7]):
        k = int(np.argmax(unit_src @ (end / r)))
        assert r <= r_src[k] - max(0.2, 0.1 * r_src[k]) + 1e-4
    # excluded points (artefacts) give no rays
    none = clearing_rays(cloud, LEVER, np.ones(len(cloud), dtype=bool), cfg)
    assert len(none) == 0


def test_filter_returns_clearing_rays_without_artefact_directions():
    ghost = block(2.3, 2.39, 0.2, 0.29, 0.85, 0.91, step=0.03)
    ground = floor(1.6, 3.0, -1.0, 1.4)
    kept, stats, rays = filter_points(np.vstack([ghost, ground]), LEVER, clearing=True)
    assert stats["removed_floating"] > 0 and stats["clearing_rays"] == len(rays)
    ghost_dirs = ghost / np.linalg.norm(ghost, axis=1)[:, None]
    ray_dirs = rays / np.linalg.norm(rays, axis=1)[:, None]
    assert (ray_dirs @ ghost_dirs.T).max() < np.cos(np.radians(0.5)) or len(rays) == 0


def test_filtered_scan_keeps_low_near_objects_and_far_walls_but_not_the_floor_curl():
    low_box = block(1.6, 1.62, -0.1, 0.1, 0.05, 0.20)              # 1.3 m from the lidar, below the old band
    curl = block(3.2, 3.22, 0.0, 0.2, 0.25, 0.40)                  # far low returns: the floor curl
    far_wall = block(7.5, 7.52, -0.5, 0.5, 0.3, 1.2, step=0.05)    # 7.2 m: beyond the cloud's 4 m
    result = filter_frame(np.vstack([low_box, curl, far_wall]), LEVER, scan=True)
    ranges = result["scan"]
    assert ranges.shape == (161,) and ranges.dtype == np.float32
    finite = ranges[np.isfinite(ranges)]
    assert finite.min() == pytest.approx(1.6 - LEVER[0], abs=0.03)
    assert not np.any((finite > 2.5) & (finite < 3.5)), "floor-curl returns must not reach the scan"
    assert np.any(finite > 7.0), "walls up to 8 m stay in the scan for clearing"
    assert (base_of(result["points"])[:, 0] < LEVER[0] + 4.0 + 1e-6).all(), "the cloud keeps its 4 m range"
    assert result["stats"]["beyond_cloud_range"] > 0 and result["stats"]["scan_bins"] > 0
    plain = filter_frame(np.vstack([low_box, curl, far_wall]), LEVER)
    assert "scan" not in plain and len(plain["points"]) == len(result["points"])


def test_scan_ranges_use_the_shadow_scan_bins():
    pts = np.array([[2.0, 0.0, 0.0], [1.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.2, 0.0, 0.0]])
    ranges = scan_ranges(pts)
    assert ranges[80] == pytest.approx(2.0)                         # straight ahead is the middle bin
    assert ranges[int(round((math.pi / 4 + math.pi / 3) / (math.pi / 240)))] == pytest.approx(math.sqrt(2))
    assert np.isfinite(ranges).sum() == 2                           # behind and too close are dropped


def test_voxel_reduction_and_lidar_frame_output():
    box = block(1.5, 1.6, 0.0, 0.1, 0.3, 0.4, step=0.01)
    kept, stats = filter_points(box, LEVER)
    keys = np.unique(np.floor(base_of(box) / 0.05).astype(int), axis=0)
    assert stats["voxels"] == len(kept) == len(keys)
    assert kept.dtype == np.float32
    assert np.allclose(base_of(kept).mean(axis=0), base_of(box).mean(axis=0), atol=0.03)


def test_witness_geometry_helpers_round_trip():
    a, b = (1.0, 2.0, 0.3), (1.5, 1.0, -0.4)
    rel = planar_relative(a, b)
    p = np.array([[0.5, 0.2, 0.1]])
    world_a = move(p, (a[0], a[1], a[2]))
    world_b = move(move(p, rel), (b[0], b[1], b[2]))
    assert np.allclose(world_a, world_b)


def test_outline_helpers():
    grown = grow_convex(MEASURED, 0.05)
    assert inside_convex(np.array([[0.84, 0.0]]), grown)[0] and not inside_convex(np.array([[0.84, 0.0]]), MEASURED)[0]
    circle = body_outline((), 0.55)
    assert len(circle) == 32 and math.isclose(max(math.hypot(*p) for p in circle), 0.55)


@pytest.mark.parametrize("bad", [{"min_height_m": 1.7}, {"max_range_m": 20.0}, {"low_max_range_m": 5.0},
                                 {"witness_min_share": 0.0}, {"floating_max_points": -1}, {"lowest_ray_deg": 95},
                                 {"scan_max_range_m": 9.0}])
def test_invalid_settings_are_rejected(bad):
    with pytest.raises(ValueError):
        ObstacleFilter(**bad)


def test_lever_must_be_finite_and_near():
    with pytest.raises(ValueError):
        filter_points(np.zeros((1, 3)), [np.nan, 0, 0])
    with pytest.raises(ValueError):
        filter_points(np.zeros((1, 3)), [5.0, 0, 0])
