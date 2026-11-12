"""Phase-wrap ghost mask on synthetic organized frames (no device, no recording)."""
import struct

import numpy as np
import pytest

from wc_sensors import phase_wrap as pw

U = pw.UNAMBIGUOUS_12MHZ_M
ROWS, COLS = 60, 160


def scene(near=1.30, amplitude=300.0):
    """A dim near wall everywhere."""
    return np.full((ROWS, COLS), near), np.full((ROWS, COLS), amplitude)


def with_anchors(r, a, rows, cols, value, amplitude=450.0):
    r[rows, cols] = value
    a[rows, cols] = amplitude
    return r, a


def test_the_constant_is_c_over_twice_12_mhz_and_the_defaults_are_the_evaluated_rule():
    assert U == pytest.approx(12.4913524, abs=1e-6)
    cfg = pw.PhaseWrapFilter()
    assert (cfg.window_px, cfg.tolerance_m, cfg.max_amplitude, cfg.max_near_range_m, cfg.min_anchor_range_m) == \
        (3, 0.6, 450.0, 6.5, 11.0)
    assert cfg.unambiguous_range_m == U


def test_dim_near_pixels_within_three_pixels_of_a_resolved_far_surface_are_ghosts():
    r, a = with_anchors(*scene(), slice(20, 30), slice(50, 60), 1.30 + U + 0.05)
    before = (r.copy(), a.copy())
    mask = pw.ghost_mask(r, a)
    np.testing.assert_array_equal(r, before[0])
    np.testing.assert_array_equal(a, before[1])
    expected = np.zeros((ROWS, COLS), dtype=bool)
    expected[17:33, 47:63] = True          # +-3 rows and columns around the anchors
    expected[20:30, 50:60] = False         # the resolved pixels themselves stay
    np.testing.assert_array_equal(mask, expected)
    assert mask.sum() == 16 * 16 - 100


def test_bright_far_or_unmatched_near_pixels_are_kept():
    r, a = with_anchors(*scene(), slice(20, 30), slice(50, 60), 1.30 + U)
    a[18, 50:60] = 450.0                   # bright: the firmware would have resolved it
    r[19, 50:60] = 6.5                     # not near
    r[30, 50:60] = 3.0                     # 3.0 + U is 1.7 m beyond the anchors
    mask = pw.ghost_mask(r, a)
    assert not mask[18, 50:60].any() and not mask[19, 50:60].any() and not mask[30, 50:60].any()
    assert mask[17, 50:60].all() and mask[31, 50:60].all()


@pytest.mark.parametrize("offset, flagged", [(0.59, True), (-0.59, True), (0.61, False), (-0.61, False)])
def test_tolerance(offset, flagged):
    r, a = with_anchors(*scene(), slice(10, 11), slice(80, 81), 1.30 + U + offset)
    assert pw.ghost_mask(r, a)[10, 79] == flagged


@pytest.mark.parametrize("dy, dx, flagged", [(3, 3, True), (-3, 0, True), (4, 0, False), (0, -4, False)])
def test_window(dy, dx, flagged):
    r, a = with_anchors(*scene(), slice(30, 31), slice(80, 81), 1.30 + U)
    assert pw.ghost_mask(r, a)[30 + dy, 80 + dx] == flagged


def test_settings_change_the_rule():
    r, a = with_anchors(*scene(), slice(30, 31), slice(80, 81), 1.30 + U)
    assert pw.ghost_mask(r, a, pw.PhaseWrapFilter(window_px=5))[30, 85]
    assert not pw.ghost_mask(r, a, pw.PhaseWrapFilter(min_anchor_range_m=13.9))[30, 81]
    assert not pw.ghost_mask(r, a, pw.PhaseWrapFilter(max_amplitude=300.0))[30, 81]
    assert not pw.ghost_mask(r, a, pw.PhaseWrapFilter(max_near_range_m=1.3))[30, 81]
    assert pw.ghost_mask(r + 0.2, a, pw.PhaseWrapFilter(unambiguous_range_m=U - 0.2))[30, 81]


def test_invalid_pixels_never_count_and_the_image_does_not_wrap_at_its_edges():
    r, a = scene()
    r[:, 159] = 1.30 + U                    # anchors in the last column
    r[0, :] = 1.30 + U                      # and in the first row
    r[59, 0] = 1.30
    mask = pw.ghost_mask(r, a)
    assert not mask[4:60, 0:3].any()        # columns 0-2 do not see column 159
    assert not mask[59, 10]                 # row 59 does not see row 0
    assert mask[1:4, 10].all() and mask[10, 156:159].all()
    r2, a2 = with_anchors(*scene(), slice(30, 31), slice(80, 81), 1.30 + U)
    r2[30, 79], a2[30, 81] = np.nan, np.nan
    r2[29, 80], r2[31, 80] = 0.0, -1.30
    mask2 = pw.ghost_mask(r2, a2)
    assert not mask2[30, 79] and not mask2[30, 81] and not mask2[29, 80] and not mask2[31, 80]
    assert mask2[30, 78] and mask2[29, 79]
    r2[30, 80] = np.inf                      # a non-finite anchor is no anchor
    assert not pw.ghost_mask(r2, a2).any()


def literal_rule(r, a, cfg):
    """The rule written out pixel by pixel, for comparison."""
    rows, cols = r.shape
    k = cfg.window_px
    out = np.zeros(r.shape, dtype=bool)
    for i in range(rows):
        for j in range(cols):
            p = r[i, j]
            if not (np.isfinite(p) and 0 < p < cfg.max_near_range_m and np.isfinite(a[i, j])
                    and a[i, j] < cfg.max_amplitude):
                continue
            window = r[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1]
            with np.errstate(invalid="ignore"):
                anchors = np.isfinite(window) & (window > cfg.min_anchor_range_m)
                out[i, j] = bool((anchors & (np.abs(window - (p + cfg.unambiguous_range_m)) < cfg.tolerance_m)).any())
    return out


@pytest.mark.parametrize("seed, window", [(1, 3), (2, 3), (3, 1), (4, 5)])
def test_the_vectorised_mask_is_the_literal_rule(seed, window):
    rng = np.random.default_rng(seed)
    near = rng.uniform(0.1, 7.0, (24, 40))
    r = np.where(rng.random(near.shape) < 0.15, near + U + rng.normal(0, 0.5, near.shape), near)
    r[rng.random(r.shape) < 0.05] = np.nan
    r[rng.random(r.shape) < 0.02] = 0.0
    a = rng.uniform(50, 900, r.shape)
    a[rng.random(a.shape) < 0.02] = np.nan
    cfg = pw.PhaseWrapFilter(window_px=window)
    mask = pw.ghost_mask(r, a, cfg)
    np.testing.assert_array_equal(mask, literal_rule(r, a, cfg))
    assert 0 < mask.sum() < (r < 6.5).sum()


def test_frames_without_anchors_or_candidates_give_an_empty_mask():
    r, a = scene()
    assert not pw.ghost_mask(r, a).any()
    r[:] = 13.8
    assert not pw.ghost_mask(r, a).any()
    with pytest.raises(ValueError):
        pw.ghost_mask(r, a[:, :10])
    with pytest.raises(ValueError):
        pw.ghost_mask(r.reshape(-1), a.reshape(-1))


def test_cloud_mask_is_the_image_mask_in_row_major_order():
    r, a = with_anchors(*scene(), slice(20, 30), slice(50, 60), 1.30 + U)
    rng = np.random.default_rng(3)
    direction = rng.normal(size=(ROWS * COLS, 3))
    direction /= np.linalg.norm(direction, axis=1, keepdims=True)
    xyz = direction * r.reshape(-1, 1)
    flat = pw.cloud_ghost_mask(xyz, a.reshape(-1), (ROWS, COLS))
    np.testing.assert_array_equal(flat.reshape(ROWS, COLS), pw.ghost_mask(r, a))
    with pytest.raises(ValueError):
        pw.cloud_ghost_mask(xyz[:-1], a.reshape(-1)[:-1], (ROWS, COLS))


def source_frame(r, a, *, big=False, organized=False, flags=None, amplitude_name="intensity", point_order="row_major"):
    """A dict SourceFrame with an x, y, z, <amplitude> float32 cloud; points along +x at the given ranges."""
    order = ">" if big else "<"
    data = b"".join(struct.pack(order + "ffff", float(rr), 0.0, 0.0, float(aa))
                    for rr, aa in zip(r.reshape(-1), a.reshape(-1)))
    height, width = (ROWS, COLS) if organized else (1, ROWS * COLS)
    cloud = {"width": width, "height": height, "point_step": 16, "row_step": 16 * width, "is_bigendian": big,
             "data": data, "fields": [{"name": n, "offset": 4 * i, "datatype": 7, "count": 1}
                                      for i, n in enumerate(("x", "y", "z", amplitude_name))]}
    if flags is None:
        flags = [] if organized else ["sdk_width=%d" % COLS, "sdk_height=%d" % ROWS, "sdk_point_order=" + point_order,
                                      "representation=host_filtered"]
    return {"cloud": cloud, "diagnostic_flags": flags}


@pytest.mark.parametrize("big, organized, name", [(False, False, "intensity"), (True, False, "intensity"),
                                                   (False, True, "amplitude")])
def test_source_frame_mask_reads_the_layout_and_the_amplitude_field(big, organized, name):
    r, a = with_anchors(*scene(), slice(20, 30), slice(50, 60), 1.30 + U)
    r[5, 5] = np.nan
    message = source_frame(r, a, big=big, organized=organized, amplitude_name=name)
    data = bytes(message["cloud"]["data"])
    mask = pw.source_frame_mask(message)
    assert mask.shape == (ROWS * COLS,)
    np.testing.assert_array_equal(mask.reshape(ROWS, COLS), pw.ghost_mask(r, a))
    assert message["cloud"]["data"] == data


@pytest.mark.parametrize("change", ["no_flags", "wrong_size", "column_major", "twice", "no_amplitude"])
def test_source_frame_mask_refuses_frames_it_cannot_lay_out(change):
    r, a = scene()
    flags = {"no_flags": [], "wrong_size": ["sdk_width=150", "sdk_height=60", "sdk_point_order=row_major"],
             "twice": ["sdk_width=160", "sdk_width=160", "sdk_height=60", "sdk_point_order=row_major"]}.get(change)
    message = source_frame(r, a, flags=flags, point_order="column_major" if change == "column_major" else "row_major",
                           amplitude_name="ring" if change == "no_amplitude" else "intensity")
    with pytest.raises(ValueError):
        pw.source_frame_mask(message)


def test_settings_validation_and_round_trip():
    cfg = pw.PhaseWrapFilter(window_px=5, tolerance_m=0.8)
    assert pw.PhaseWrapFilter.from_dict(cfg.as_dict()) == cfg
    assert pw.PhaseWrapFilter.from_dict({"window_px": 2}) == pw.PhaseWrapFilter(window_px=2)
    assert pw.PhaseWrapFilter(window_px=np.int64(3)).window_px == 3
    for bad in ({"window_px": 0}, {"window_px": 11}, {"window_px": 2.5}, {"window_px": True},
                {"tolerance_m": 0}, {"tolerance_m": float("nan")}, {"tolerance_m": 2.5}, {"max_amplitude": -1},
                {"max_near_range_m": 12.6}, {"min_anchor_range_m": 6.0}, {"min_anchor_range_m": 30.0},
                {"unambiguous_range_m": 4.0}, {"max_amplitude": "450"}):
        with pytest.raises(ValueError):
            pw.PhaseWrapFilter(**bad)
    with pytest.raises(ValueError, match="unknown"):
        pw.PhaseWrapFilter.from_dict({"window": 3})
    with pytest.raises(ValueError):
        pw.PhaseWrapFilter.from_dict([("window_px", 3)])
