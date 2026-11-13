"""Phase-wrap ghosts predicted from a surface model, on a synthetic corridor (no device, no recording)."""
import math
import struct

import numpy as np
import pytest

from wc_sensors import phase_wrap_model as pm

U = pm.UNAMBIGUOUS_12MHZ_M
VOXEL = 0.1
ORIGIN = (-2.0, -1.6, -1.0)          # voxel faces at round numbers: x = -2.0 + 0.1 i


def corridor(end_x=15.0, blocks=()):
    """Floor z = -1.0..-0.9, ceiling 2.5..2.6, side walls at |y| 1.5..1.6, end walls at x -2.0 and end_x."""
    shape = (int(round((end_x + 0.1 - ORIGIN[0]) / VOXEL)), 32, 36)
    occ = np.zeros(shape, dtype=bool)
    occ[:, :, 0] = True                          # floor
    occ[:, :, 35] = True                         # ceiling (z 2.5..2.6)
    occ[:, 0, :] = occ[:, 31, :] = True          # y -1.6..-1.5 and 1.5..1.6
    occ[0, :, :] = occ[-1, :, :] = True          # x -2.0..-1.9 and end_x..end_x + 0.1
    for (x0, x1), (y0, y1), (z0, z1) in blocks:
        occ[cell(x0, 0):cell(x1, 0), cell(y0, 1):cell(y1, 1), cell(z0, 2):cell(z1, 2)] = True
    return pm.SurfaceModel.from_occupancy(occ, ORIGIN, VOXEL, {"axle_z_m": -0.6, "variant": "synthetic"})


def cell(value, axis):
    return int(round((value - ORIGIN[axis]) / VOXEL))


IDENTITY = (np.eye(3), np.zeros(3))


def pixels(*rows):
    """(xyz, amplitude) of pixels given as (range, amplitude, direction) in the lidar frame."""
    xyz, amp = [], []
    for r, a, direction in rows:
        u = np.asarray(direction, dtype=float)
        xyz.append(r * u / np.linalg.norm(u))
        amp.append(a)
    return np.array(xyz), np.array(amp)


def test_a_dim_near_return_from_a_surface_one_ambiguity_further_on_is_folded():
    model = corridor()
    xyz, amp = pixels((15.0 - U, 200.0, (1, 0, 0)), (15.0 - U + 0.45, 200.0, (1, 0, 0)),
                      (15.0 - U - 0.55, 200.0, (1, 0, 0)))
    before = (xyz.copy(), amp.copy())
    folded, d = pm.predict(xyz, amp, *IDENTITY, model)
    np.testing.assert_array_equal(xyz, before[0])
    np.testing.assert_array_equal(amp, before[1])
    assert folded.tolist() == [True, True, False]
    assert d == pytest.approx([15.0, 15.0, 15.0], abs=0.06)    # the end wall's face, at most half a voxel late


def test_bright_far_invalid_and_mismatched_pixels_are_not_folded():
    model = corridor()
    xyz, amp = pixels((15.0 - U, 400.0, (1, 0, 0)),          # bright: the firmware unwraps it
                      (6.6, 200.0, (1, 0, 0)),                # beyond max_range_m
                      (3.6, 200.0, (1, 0, 0)),                # 3.6 + U is 1.1 m short of the wall
                      (15.0 - U, float("nan"), (1, 0, 0)))
    xyz = np.vstack([xyz, [np.nan, 0, 0], [0, 0, 0]])
    amp = np.r_[amp, 100.0, 100.0]
    folded, d = pm.predict(xyz, amp, *IDENTITY, model)
    assert not folded.any()
    assert np.isnan(d[[0, 1, 3, 4, 5]]).all() and d[2] == pytest.approx(15.0, abs=0.06)


def test_a_nearer_surface_along_the_ray_hides_the_far_one():
    model = corridor(blocks=[((5.0, 5.2), (-0.2, 0.2), (-0.2, 0.2))])
    xyz, amp = pixels((15.0 - U, 200.0, (1, 0, 0)), (15.0 - U, 200.0, (1, 0.1, 0)))
    folded, d = pm.predict(xyz, amp, *IDENTITY, model)
    assert d[0] == pytest.approx(5.0, abs=0.06) and not folded[0]
    assert folded[1]                                   # this ray passes the block on its left


def test_two_ambiguities_and_the_folds_setting():
    model = corridor(end_x=28.0)
    xyz, amp = pixels((28.0 - 2 * U, 200.0, (1, 0, 0)))
    assert pm.predict(xyz, amp, *IDENTITY, model)[0][0]
    assert not pm.predict(xyz, amp, *IDENTITY, model, pm.FoldPrediction(folds=(1,)))[0][0]


def test_the_ray_follows_the_lidar_pose():
    """Pitched up, the ray meets the ceiling; moved and yawed, the same pixel looks down the corridor."""
    model = corridor()
    e = math.asin((2.5 - 0.0) / 13.5)                 # elevation reaching the ceiling face 13.5 m out
    pitch = np.array([[math.cos(e), 0, -math.sin(e)], [0, 1, 0], [math.sin(e), 0, math.cos(e)]])
    xyz, amp = pixels((13.5 - U, 150.0, (1, 0, 0)))
    folded, d = pm.predict(xyz, amp, pitch, np.zeros(3), model)
    assert folded[0] and d[0] == pytest.approx(13.5, abs=0.3)   # a ceiling met at 11 degrees: 0.05 m is 0.27 m
    rot, t = pm.lidar_pose((1.0, 0.2, math.pi / 2), [[0, 1, 0], [-1, 0, 0], [0, 0, 1]], (0.3, 0.2, 0.6), -0.6)
    np.testing.assert_allclose(rot, np.eye(3), atol=1e-12)       # the mount's -90 deg yaw undoes the pose's +90
    np.testing.assert_allclose(t, (1.0 - 0.2, 0.2 + 0.3, 0.0), atol=1e-12)
    xyz, amp = pixels((14.2 - U, 150.0, (1, 0, 0)))            # the lidar is 0.8 m along: the end wall 14.2 m off
    folded, d = pm.predict(xyz, amp, rot, t, model)
    assert folded[0] and d[0] == pytest.approx(14.2, abs=0.06)


def brute_force(origin, u, model, min_m, max_m, step=0.002):
    s = np.arange(min_m, max_m, step)
    p = np.asarray(origin) + s[:, None] * np.asarray(u)
    cell = np.floor((p - model.origin) / model.voxel_m).astype(int)
    inside = ((cell >= 0) & (cell < model.shape)).all(axis=1)
    hit = np.zeros(len(s), dtype=bool)
    hit[inside] = model.occupied[cell[inside, 0], cell[inside, 1], cell[inside, 2]]
    if not inside.all():
        hit[np.argmin(inside):] = False
    return s[np.argmax(hit)] if hit.any() else np.nan


def test_the_sphere_tracing_finds_the_first_voxel_a_fine_march_finds():
    model = corridor(blocks=[((4.0, 4.3), (0.5, 0.9), (-1.0, 0.8)), ((9.0, 9.1), (-1.5, 0.0), (0.5, 2.5))])
    rng = np.random.default_rng(3)
    u = rng.normal(size=(300, 3))
    u[:, 0] = np.abs(u[:, 0]) * 3
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    origin = np.array([0.1, 0.05, 0.3])
    d = pm.first_surface(origin, u, model, 0.3, 40.0)
    expected = np.array([brute_force(origin, ray, model, 0.3, 40.0) for ray in u])
    assert np.isfinite(d).all() and np.isfinite(expected).all()
    # never early; at most one half-voxel step late, except where a step only grazed a voxel's corner
    assert (d >= expected - 0.003).all()
    late = d - expected
    assert np.mean(late <= 0.052) > 0.97 and late.max() < 0.5


def test_save_load_and_hash(tmp_path):
    model = corridor()
    path = tmp_path / "model.npz"
    sha = model.save(path)
    loaded = pm.SurfaceModel.load(path, sha)
    np.testing.assert_array_equal(loaded.distance, model.distance)
    np.testing.assert_array_equal(loaded.origin, model.origin)
    assert loaded.voxel_m == model.voxel_m and loaded.metadata == model.metadata and loaded.axle_z_m == -0.6
    with pytest.raises(ValueError, match="sha256"):
        pm.SurfaceModel.load(path, "0" * 64)
    with pytest.raises(FileExistsError):
        model.save(path)
    with pytest.raises(ValueError):
        pm.SurfaceModel(model.distance.astype(np.int16), ORIGIN, VOXEL)
    with pytest.raises(ValueError):
        pm.SurfaceModel(model.distance, ORIGIN, VOXEL, {"axle_z_m": "low"})


def test_settings_validation_and_round_trip():
    cfg = pm.FoldPrediction()
    assert (cfg.max_range_m, cfg.max_amplitude, cfg.tolerance_m, cfg.folds, cfg.min_surface_m, cfg.floor_block_m,
            cfg.known_surface_m, cfg.max_steps) == (6.5, 400.0, 0.5, (1, 2), 0.3, 6.0, 0.2, 400)
    assert cfg.search_m == pytest.approx(6.5 + 2 * U + 0.5)
    assert pm.FoldPrediction.from_dict(cfg.as_dict()) == cfg
    assert pm.FoldPrediction.from_dict({"folds": [2, 1]}).folds == (1, 2)
    off = pm.FoldPrediction(floor_block_m=None)
    assert off.floor_block_m is None and pm.FoldPrediction.from_dict(off.as_dict()) == off
    for bad in ({"max_range_m": 0}, {"max_range_m": 12.6}, {"max_amplitude": float("nan")}, {"tolerance_m": 2.5},
                {"folds": []}, {"folds": [0]}, {"folds": [1, 1]}, {"folds": "12"}, {"folds": [True]},
                {"max_steps": 5}, {"max_steps": 10.5}, {"min_surface_m": 7.0}, {"unambiguous_range_m": 4.0},
                {"floor_block_m": 0.0}, {"floor_block_m": float("inf")}, {"floor_block_m": True},
                {"known_surface_m": 0.0}, {"known_surface_m": 1.5}):
        with pytest.raises(ValueError):
            pm.FoldPrediction(**bad)
    with pytest.raises(ValueError, match="unknown"):
        pm.FoldPrediction.from_dict({"tolerance": 0.5})


def floorless(end_x=15.0, floor_z=-1.0):
    """The corridor without its floor voxels: side and end walls reach below the floor plane, as a mirrored model."""
    model = corridor(end_x)
    occ = model.occupied.copy()
    occ[:, :, 0] = False
    occ[:, 1:31, 0] = occ[:, 1:31, 1]                 # walls go on through the floor layer
    return pm.SurfaceModel.from_occupancy(occ, ORIGIN, VOXEL, {"axle_z_m": -0.6, "floor_z_m": floor_z})


def test_the_floor_plane_stops_steep_rays_only():
    """A grazing ray passes the floor plane (the walls' mirror image is behind it); a steep one stops there."""
    model = floorless(end_x=15.0)
    origin = np.array([0.0, 0.0, 0.0])                # 1.0 m above the floor plane z = -1.0
    grazing = math.atan2(1.0, 30.0)                   # meets the floor plane 30 m out: beyond floor_block_m
    steep = math.atan2(1.0, 4.0)                      # meets it 4.1 m out
    rows = [(15.0 / math.cos(grazing) - U, 200.0, (math.cos(grazing), 0, -math.sin(grazing))),
            (3.0, 200.0, (math.cos(steep), 0, -math.sin(steep)))]
    xyz, amp = pixels(*rows)
    folded, d = pm.predict(xyz, amp, np.eye(3), origin, model)
    assert folded.tolist() == [True, False]
    assert d[0] == pytest.approx(15.0 / math.cos(grazing), abs=0.06)
    assert np.isnan(d[1])                             # stopped by the floor before any fold distance: not searched
    # without the floor block the steep ray goes on below the floor: nothing there in this model
    folded, d = pm.predict(xyz, amp, np.eye(3), origin, model, pm.FoldPrediction(floor_block_m=None))
    assert folded[0] and not folded[1] and np.isnan(d[1])
    # a model without floor_z_m has no floor plane
    bare = pm.SurfaceModel(model.distance, model.origin, model.voxel_m, {"axle_z_m": -0.6})
    assert bare.floor_z_m is None and pm.predict(xyz, amp, np.eye(3), origin, bare)[0][0]
    with pytest.raises(ValueError):
        pm.SurfaceModel(model.distance, model.origin, model.voxel_m, {"floor_z_m": "low"})


def test_a_point_on_a_known_surface_is_not_searched():
    """A dim wall return is that wall's (d stays NaN, not folded) even where the ray would go on to a far surface."""
    model = corridor(end_x=15.0)
    a = math.radians(20)
    xyz, amp = pixels((1.45 / math.sin(a), 200.0, (math.cos(a), math.sin(a), 0)),   # in the voxel next to the wall
                      (15.0 - U, 200.0, (1, 0, 0)))
    folded, d = pm.predict(xyz, amp, *IDENTITY, model)
    assert folded.tolist() == [False, True] and np.isnan(d[0])
    folded, d = pm.predict(xyz, amp, *IDENTITY, model, pm.FoldPrediction(known_surface_m=0.05))
    assert not folded[0] and d[0] == pytest.approx(1.5 / math.sin(a), abs=0.2)     # searched: the wall itself


def test_first_surface_takes_one_limit_per_ray():
    model = corridor(end_x=15.0)
    u = np.array([[1.0, 0.0, 0.0]] * 3)
    d = pm.first_surface(np.zeros(3), u, model, 0.3, np.array([14.0, 15.2, 40.0]))
    assert np.isnan(d[0]) and d[1:] == pytest.approx([15.0, 15.0], abs=0.06)


def test_source_frame_predict_reads_the_frame_in_cloud_order():
    model = corridor()
    rows = [(15.0 - U, 200.0, (1, 0, 0)), (3.0, 200.0, (1, 0.2, 0)), (15.0 - U, 900.0, (1, 0, 0))] * 3200
    xyz, amp = pixels(*rows)
    data = b"".join(struct.pack("<ffff", *p, a) for p, a in zip(xyz, amp))
    message = {"cloud": {"width": 9600, "height": 1, "point_step": 16, "row_step": 16 * 9600, "is_bigendian": False,
                         "data": data, "fields": [{"name": n, "offset": 4 * i, "datatype": 7, "count": 1}
                                                  for i, n in enumerate(("x", "y", "z", "intensity"))]},
               "diagnostic_flags": ["sdk_width=160", "sdk_height=60", "sdk_point_order=row_major"]}
    folded, d = pm.source_frame_predict(message, *IDENTITY, model)
    expected = pm.predict(xyz.astype(np.float32).astype(float), amp, *IDENTITY, model)
    np.testing.assert_array_equal(folded, expected[0])
    np.testing.assert_array_equal(d, expected[1])
    assert folded[0::3].all() and not folded[1::3].any() and not folded[2::3].any()
