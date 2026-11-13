"""Obstacle filter on synthetic prepared clouds: floor, height window, far-floor gate, body, unsupported clusters."""
import math

import numpy as np
import pytest

from wc_navigation.perception.consensus import EchoWitness, Recent, RecentFrames, Witness, move, planar_relative
from wc_navigation.perception.obstacle_cloud import (EXPLAIN_BEYOND, EXPLAIN_KEPT, EXPLAIN_NOT_FINITE, WRAP_REASONS,
                                                     ObstacleFilter, body_outline, clearing_rays, filter_frame,
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


def _through(points_base, other_lever, reach_m=6.0):
    """A witness whose rays pass through every given point to returns ``reach_m`` from the other lidar."""
    d = points_base - other_lever
    return Witness(other_lever + reach_m * d / np.linalg.norm(d, axis=1)[:, None], other_lever)


OTHER = np.array([0.41173, -0.23331, 0.55265])        # right lidar in base_link (frozen mount)


def test_witness_scope_all_drops_a_supported_cluster_the_other_lidar_saw_through():
    # a phase-wrap ghost hangs above the floor; low enough above the visible floor, stage 5 calls it supported
    ghost = block(1.9, 1.94, 0.0, 0.15, 0.27, 1.0, step=0.03)
    assert len(ghost) <= ObstacleFilter().unsupported_max_points
    witness = _through(base_of(ghost), OTHER)
    kept, stats = filter_points(ghost, LEVER, witness=witness)
    assert len(kept) > 0 and stats["removed_contradicted"] == 0 and "removed_seen_through" not in stats
    cfg = ObstacleFilter(witness_scope="all")
    result = filter_frame(ghost, LEVER, cfg, witness=witness, clearing=True, scan=True)
    stats = result["stats"]
    assert len(result["points"]) == 0 and stats["removed_seen_through"] > 0 and stats["clusters_judged_by_witness"] == 1
    assert not np.isfinite(result["scan"]).any() and len(result["rays"]) == 0
    # the same cluster hit by the other lidar is real and stays; without a witness nothing is judged
    kept, stats = filter_points(ghost, LEVER, cfg, witness=Witness(base_of(ghost), OTHER))
    assert len(kept) > 0 and stats["removed_seen_through"] == 0
    kept, stats = filter_points(ghost, LEVER, cfg)
    assert len(kept) > 0 and stats["removed_seen_through"] == 0 and stats["clusters_judged_by_witness"] == 0


def test_witness_scope_all_keeps_what_stands_on_the_floor_and_large_clusters():
    cfg = ObstacleFilter(witness_scope="all")
    # standing on the floor near the chair, seen through anyway (a wrong far reading of the other lidar)
    near = block(1.9, 1.94, 0.0, 0.3, 0.06, 1.0, step=0.03)
    kept, stats = filter_points(near, LEVER, cfg, witness=_through(base_of(near), OTHER))
    assert len(kept) > 0 and stats["removed_seen_through"] == 0 and stats["clusters_judged_by_witness"] == 0
    # 2.4 m from the lidar stage 3 drops everything below 0.5 m, yet the legs still show it stands on the floor
    far = block(2.7, 2.76, -0.1, 0.2, 0.0, 1.6, step=0.03)
    kept, stats = filter_points(far, LEVER, cfg, witness=_through(base_of(far), OTHER))
    assert stats["removed_far_low"] > 0 and len(kept) > 0 and stats["removed_seen_through"] == 0
    # more points than unsupported_max_points: never judged
    big = block(1.9, 1.99, -0.5, 0.5, 0.27, 1.0, step=0.03)
    assert len(big) > cfg.unsupported_max_points
    kept, stats = filter_points(big, LEVER, cfg, witness=_through(base_of(big), OTHER))
    assert len(kept) > 0 and stats["removed_seen_through"] == 0 and stats["clusters_judged_by_witness"] == 0


def test_witness_scope_all_keeps_a_person_the_other_lidar_partly_sees_past():
    person = block(1.9, 2.1, 0.0, 0.3, 0.0, 1.6, step=0.03)
    pb = base_of(person)
    edge = pb[pb[:, 1] > 0.24]                                      # the other lidar sees past one side of it
    returns = np.vstack([pb[pb[:, 1] <= 0.24], _through(edge, OTHER)._inputs[0]])
    kept, stats = filter_points(person, LEVER, ObstacleFilter(witness_scope="all"), witness=Witness(returns, OTHER))
    assert stats["removed_seen_through"] == 0 and len(kept) > 0


def test_witness_scope_setting():
    assert ObstacleFilter().witness_scope == "unsupported" and "witness_scope" not in ObstacleFilter().as_dict()
    assert ObstacleFilter(witness_scope="all").as_dict()["witness_scope"] == "all"
    assert ObstacleFilter(**ObstacleFilter(witness_scope="all").as_dict()).witness_scope == "all"
    assert "seen_through" not in ObstacleFilter().stage_order
    assert "seen_through_floor_m" not in ObstacleFilter().as_dict()
    assert ObstacleFilter(witness_scope="all").as_dict()["seen_through_floor_m"] == 0.25
    order = ObstacleFilter(witness_scope="all").stage_order
    assert order.index("floating") < order.index("seen_through") < order.index("transient")
    for bad in ({"witness_scope": "everything"}, {"seen_through_floor_m": 0.05}, {"seen_through_floor_m": 1.5}):
        with pytest.raises(ValueError):
            ObstacleFilter(**bad)


# -- stage 5b phase_wrap: prediction and confirmation (status/C.md 2026-11-13) ----------------------------------
def wrap_ghost():
    """A phase-wrap ghost as on 11-11: 0.27-1.0 m above the floor, 300 points; stage 5 keeps it."""
    return block(1.9, 1.94, 0.0, 0.15, 0.27, 1.0, step=0.03)


def echo(points_base, empty=False, reach_m=6.0):
    """The other lidar's frame with a ray through every given point: to a return ``reach_m`` from it, or
    (``empty``) from a pixel without a return."""
    d = points_base - OTHER
    units = d / np.linalg.norm(d, axis=1)[:, None]
    if empty:
        return EchoWitness(np.zeros((0, 3)), OTHER, units)
    return EchoWitness(OTHER + reach_m * units, OTHER)


def wrap_inputs(n, witness, trusted=True, fold=None, amplitude=None):
    return {"fold": np.ones(n, dtype=bool) if fold is None else fold, "trusted": trusted, "witness": witness,
            "amplitude": np.full(n, 200.0) if amplitude is None else amplitude}


def test_phase_wrap_stage_is_off_by_default_and_changes_nothing_then():
    cfg = ObstacleFilter()
    assert not cfg.wrap_delete and "phase_wrap" not in cfg.stage_order
    assert not any(name.startswith("wrap_") for name in cfg.as_dict())
    ghost = wrap_ghost()
    plain = filter_frame(ghost, LEVER, cfg, clearing=True, scan=True)
    given = filter_frame(ghost, LEVER, cfg, clearing=True, scan=True,
                         wrap=wrap_inputs(len(ghost), echo(base_of(ghost))))
    assert plain["stats"] == given["stats"] and "removed_phase_wrap" not in given["stats"]
    assert "phase_wrap" not in given and "dropped_by" not in given and len(given["points"]) > 0
    for key in ("points", "rays", "scan"):
        np.testing.assert_array_equal(plain[key], given[key])


@pytest.mark.parametrize("empty", [False, True])
def test_phase_wrap_deletes_a_predicted_dim_cluster_the_other_lidar_saw_through_or_had_no_echo_at(empty):
    ghost = wrap_ghost()
    cfg = ObstacleFilter(wrap_delete=True)
    order = cfg.stage_order
    assert order.index("floating") < order.index("phase_wrap") < order.index("transient")
    result = filter_frame(ghost, LEVER, cfg, clearing=True, scan=True,
                          wrap=wrap_inputs(len(ghost), echo(base_of(ghost), empty=empty)))
    stats, [deleted] = result["stats"], result["phase_wrap"]
    assert stats["removed_phase_wrap"] == len(ghost) and stats["phase_wrap_clusters"] == 1
    assert stats["phase_wrap_trusted"] == 1 and all(stats["phase_wrap_kept_" + r] == 0 for r in WRAP_REASONS)
    # gone from the cloud and the scan, and no clearing ray stops at it
    assert len(result["points"]) == 0 and not np.isfinite(result["scan"]).any() and len(result["rays"]) == 0
    assert deleted["points"] == len(ghost) and deleted["predicted"] == 1.0 and deleted["amplitude_max"] == 200.0
    assert deleted["hit"] == 0.0 and (deleted["empty"] if empty else deleted["through"]) >= cfg.wrap_unconfirmed_share
    assert deleted["through" if empty else "empty"] == 0.0
    assert 0.25 < deleted["height_m"][0] < deleted["height_m"][1] < 1.05


def test_phase_wrap_keeps_a_cluster_and_names_the_first_condition_it_fails():
    cfg = ObstacleFilter(wrap_delete=True)
    ghost = wrap_ghost()
    gb, n = base_of(ghost), len(ghost)

    def reasons(cloud, wrap, body=()):
        result = filter_frame(cloud, LEVER, cfg, body, wrap=wrap)
        stats = result["stats"]
        assert stats["removed_phase_wrap"] == 0 and result["phase_wrap"] == [] and len(result["points"]) > 0
        return [r for r in WRAP_REASONS if stats["phase_wrap_kept_" + r]]

    fold = np.ones(n, dtype=bool)
    fold[: int(0.15 * n)] = False                                   # 85 % of its points predicted
    assert reasons(ghost, wrap_inputs(n, echo(gb), fold=fold)) == ["not_predicted"]
    amplitude = np.full(n, 200.0)
    amplitude[0] = cfg.wrap_max_amplitude                           # one point as bright as a real surface
    assert reasons(ghost, wrap_inputs(n, echo(gb), amplitude=amplitude)) == ["bright"]
    standing = block(1.9, 1.94, 0.0, 0.15, 0.06, 1.0, step=0.03)    # reaches down to the floor
    assert reasons(standing, wrap_inputs(len(standing), echo(base_of(standing)))) == ["on_floor"]
    near = block(0.82, 0.88, -0.1, 0.1, 0.3, 0.9, step=0.03)        # 1 cm ahead of the measured front edge
    assert reasons(near, wrap_inputs(len(near), echo(base_of(near))), MEASURED) == ["stop_zone"]
    assert reasons(ghost, wrap_inputs(n, EchoWitness(gb, OTHER))) == ["confirmed"]       # the other lidar hit it
    assert reasons(ghost, wrap_inputs(n, None)) == ["no_verdict"]                       # no frame of the other lidar
    assert reasons(ghost, wrap_inputs(n, echo(gb, reach_m=0.8))) == ["no_verdict"]      # behind a nearer return
    elsewhere = echo(base_of(block(1.0, 1.1, -0.8, -0.7, 0.85, 0.91)))                  # outside its rays
    assert reasons(ghost, wrap_inputs(n, elsewhere)) == ["no_verdict"]


def test_phase_wrap_judges_nothing_while_the_localization_is_not_trusted():
    cfg = ObstacleFilter(wrap_delete=True)
    ghost = wrap_ghost()
    for wrap in (wrap_inputs(len(ghost), echo(base_of(ghost)), trusted=False), None):
        result = filter_frame(ghost, LEVER, cfg, wrap=wrap)
        stats = result["stats"]
        assert stats["phase_wrap_trusted"] == 0 and stats["removed_phase_wrap"] == 0 and result["phase_wrap"] == []
        assert all(stats["phase_wrap_kept_" + r] == 0 for r in WRAP_REASONS) and len(result["points"]) > 0


def test_phase_wrap_settings():
    cfg = ObstacleFilter(wrap_delete=True)
    values = cfg.as_dict()
    assert values["wrap_delete"] is True and values["wrap_min_share"] == 0.9 and values["wrap_max_amplitude"] == 400.0
    assert values["wrap_unconfirmed_share"] == 0.5 and values["wrap_stop_margin_m"] == 0.02
    assert values["seen_through_floor_m"] == 0.25                   # its floor guard is recorded with it
    assert ObstacleFilter(**values) == cfg
    order = ObstacleFilter(wrap_delete=True, witness_scope="all").stage_order
    assert order.index("seen_through") < order.index("phase_wrap") < order.index("transient")
    assert ObstacleFilter(wrap_min_share=0.8).as_dict()["wrap_min_share"] == 0.8   # changed values are recorded
    for bad in ({"wrap_delete": 1}, {"wrap_min_share": 0.0}, {"wrap_min_share": 1.5}, {"wrap_unconfirmed_share": 0.0},
                {"wrap_max_amplitude": 0.0}, {"wrap_max_amplitude": float("nan")}, {"wrap_max_amplitude": 5000.0},
                {"wrap_stop_margin_m": -0.01}, {"wrap_stop_margin_m": 0.6}):
        with pytest.raises(ValueError):
            ObstacleFilter(**bad)


def test_explain_names_the_stage_that_dropped_each_point():
    ghost = wrap_ghost()
    parts = [floor(1.5, 3.0, -1, 1), block(1.2, 1.3, -0.1, 0.1, 0.0, 0.35), ghost,
             block(4.6, 4.62, -0.5, 0.5, 0.6, 1.2)]                 # the last beyond max_range_m: in the scan only
    cloud = np.vstack(parts + [np.full((1, 3), np.nan)])
    n = len(cloud)
    fold = np.zeros(n, dtype=bool)
    start = len(parts[0]) + len(parts[1])
    fold[start:start + len(ghost)] = True
    cfg = ObstacleFilter(wrap_delete=True)
    result = filter_frame(cloud, LEVER, cfg, scan=True, explain=True,
                          wrap=wrap_inputs(n, echo(base_of(ghost)), fold=fold))
    codes, names, stats = result["dropped_by"], result["stage_names"], result["stats"]
    assert len(codes) == n and names == list(cfg.stage_order) and codes[-1] == EXPLAIN_NOT_FINITE
    for k, name in enumerate(names):
        assert (codes == k).sum() == stats["removed_" + name], name
    assert (codes[start:start + len(ghost)] == names.index("phase_wrap")).all()
    assert (codes == EXPLAIN_BEYOND).sum() == stats["beyond_cloud_range"] > 0
    assert (codes == EXPLAIN_KEPT).sum() == stats["kept_points"] > 0


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


def test_transient_stage_is_off_by_default_and_drops_clusters_absent_earlier():
    ghost = block(2.3, 2.42, 0.15, 0.30, 0.85, 0.91, step=0.03)     # 90 points: survives the size rule
    cloud = np.vstack([ghost, floor(1.6, 3.0, -1.0, 1.4)])
    elsewhere = Recent(base_of(block(1.0, 1.1, -0.8, -0.7, 0.85, 0.91)))
    result = filter_frame(cloud, LEVER, ObstacleFilter(), recent=elsewhere)
    assert result["stats"]["removed_transient"] == 0 and "candidates" not in result
    cfg = ObstacleFilter(persist_min_age_s=0.6)
    result = filter_frame(cloud, LEVER, cfg)                          # no earlier frames: no verdict
    assert result["stats"]["removed_transient"] == 0 and len(result["candidates"]) > 0
    result = filter_frame(cloud, LEVER, cfg, recent=elsewhere, clearing=True)
    assert result["stats"]["removed_transient"] == len(ghost) and len(result["points"]) == 0
    rays = result["rays"]
    if len(rays):                                                   # no clearing rays toward the artefact
        ghost_dirs = ghost / np.linalg.norm(ghost, axis=1)[:, None]
        ray_dirs = rays / np.linalg.norm(rays, axis=1)[:, None]
        assert (ray_dirs @ ghost_dirs.T).max() < np.cos(np.radians(0.5))
    result = filter_frame(cloud, LEVER, cfg, recent=Recent(base_of(ghost)))   # seen there before: real
    assert result["stats"]["removed_transient"] == 0 and len(result["points"]) > 0


def test_supported_objects_are_never_judged_by_persistence():
    person = block(2.3, 2.39, 0.2, 0.29, 0.0, 1.2, step=0.03)
    cfg = ObstacleFilter(persist_min_age_s=0.6)
    result = filter_frame(np.vstack([person, floor(1.6, 3.0, -1.0, 1.4)]), LEVER, cfg, recent=Recent(np.zeros((0, 3))))
    assert result["stats"]["removed_transient"] == 0 and len(result["points"]) > 0


def test_recent_frames_place_earlier_candidates_in_the_world():
    def to_base(world, pose):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        dx, dy = world[:, 0] - pose[0], world[:, 1] - pose[1]
        return np.column_stack([c * dx + s * dy, -s * dx + c * dy, world[:, 2]])
    frames = RecentFrames(0.6, 2.0)
    obj = np.array([[3.0, 1.0, 0.9], [3.05, 1.0, 0.9]])
    pose0, pose1 = (0.0, 0.0, 0.0), (0.5, 0.2, math.radians(20))
    frames.add(10.0, pose0, to_base(obj, pose0))
    assert frames.recent(10.3, pose1) is None                       # younger than 0.6 s
    recent = frames.recent(11.0, pose1)
    assert recent.share(to_base(obj, pose1)) == 1.0
    assert recent.share(to_base(obj + np.array([0.5, 0.0, 0.0]), pose1)) == 0.0
    frames.add(13.5, pose1, np.zeros((0, 3)))                         # entries older than 2 s are forgotten
    assert frames.recent(14.2, pose1).points == 0


@pytest.mark.parametrize("bad", [{"min_height_m": 1.7}, {"max_range_m": 20.0}, {"low_max_range_m": 5.0},
                                 {"witness_min_share": 0.0}, {"floating_max_points": -1}, {"lowest_ray_deg": 95},
                                 {"scan_max_range_m": 9.0}, {"persist_min_age_s": -1.0},
                                 {"persist_min_age_s": 2.0, "persist_max_age_s": 1.0},
                                 {"persist_min_age_s": 0.5, "persist_max_age_s": 6.0}, {"persist_min_share": 0.0}])
def test_invalid_settings_are_rejected(bad):
    with pytest.raises(ValueError):
        ObstacleFilter(**bad)


def test_lever_must_be_finite_and_near():
    with pytest.raises(ValueError):
        filter_points(np.zeros((1, 3)), [np.nan, 0, 0])
    with pytest.raises(ValueError):
        filter_points(np.zeros((1, 3)), [5.0, 0, 0])
