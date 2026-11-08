"""Obstacle tracks: clustering, constant-velocity tracking, classes, map removal and the message schema."""
import math

import numpy as np
import pytest

from wc_navigation.perception.tracks import SCHEMA, StaticMap, Tracker, TrackerConfig, clusters


def cylinder(x, y, r=0.2, h0=0.05, h1=1.6, step=0.05):
    """Surface points of an upright cylinder (a standing person) as (x, y, h)."""
    out = []
    for a in np.linspace(0, 2 * math.pi, 24, endpoint=False):
        for h in np.arange(h0, h1 + 1e-9, step):
            out.append((x + r * math.cos(a), y + r * math.sin(a), h))
    return np.asarray(out)


def wall_map(wall_y=1.0, size=12.0, resolution=0.05):
    """A map with one wall along y = wall_y (cells at and above it occupied)."""
    n = int(size / resolution)
    data = np.zeros((n, n), dtype=np.int8)
    row = int((wall_y + size / 2) / resolution)
    data[row:row + 4, :] = 100
    return StaticMap(data, resolution, (-size / 2, -size / 2))


def test_clusters_split_objects_and_drop_small_or_out_of_band_points():
    pts = np.vstack([cylinder(0.0, 0.0), cylinder(2.0, 0.0), [[5.0, 5.0, 0.5]] * 3, cylinder(-2.0, 0.0, h0=1.9, h1=2.2)])
    found = clusters(pts)
    centres = sorted(round(float(c["xy"][0]), 1) for c in found)
    assert centres == [0.0, 2.0]
    c = found[0]
    assert 0.2 <= c["radius_m"] <= 0.3 and c["height_m"][0] == pytest.approx(0.05) and c["elongation"] < 1.5


def test_a_walking_person_becomes_one_moving_track_with_its_velocity():
    tracker = Tracker()
    for k in range(15):                                  # 3 s at 5 Hz, 1.0 m/s along -x, 0.3 m/s along +y
        t = 100.0 + 0.2 * k
        out = tracker.update(t, cylinder(4.0 - 1.0 * (t - 100.0), 0.3 * (t - 100.0)))
    assert len(out) == 1
    track = out[0]
    assert track["class"] == "moving" and track["state"] == "confirmed" and track["has_moved"]
    assert track["vx"] == pytest.approx(-1.0, abs=0.15) and track["vy"] == pytest.approx(0.3, abs=0.15)
    assert track["id"] == 1                               # no identity switch


def test_static_unmapped_objects_and_floating_artefacts_never_move():
    tracker = Tracker()
    box = np.array([(1.5 + 0.05 * i, -0.2 + 0.05 * j, 0.05 + 0.05 * k)
                    for i in range(6) for j in range(8) for k in range(6)])
    for k in range(10):
        t = 0.2 * k
        # an artefact that rides along 1.8 m ahead of a chair driving 0.5 m/s, with nothing near the floor
        ghost = cylinder(1.8 + 0.5 * t, 3.0, r=0.1, h0=0.9, h1=1.2)
        out = tracker.update(t, np.vstack([box, ghost]))
    classes = {round(tr["x"]): tr["class"] for tr in out}
    assert classes[2] == "static_unknown"
    assert all(tr["class"] != "moving" for tr in out)


def test_person_candidate_needs_a_person_shape_away_from_the_walls():
    walls = wall_map(wall_y=1.0)
    tracker = Tracker(static_map=walls)
    out = []
    for k in range(5):
        out = tracker.update(0.2 * k, np.vstack([cylinder(0.0, -1.0), cylinder(3.0, 0.75)]))
    by_x = {round(tr["x"]): tr for tr in out}
    assert by_x[0]["class"] == "person_candidate" and by_x[0]["wall_gap_m"] > 0.15
    assert by_x[3]["class"] == "static_unknown"           # standing against the wall: no person margin


def test_walls_and_door_frames_in_the_map_make_no_tracks_and_a_door_leaf_stays_static():
    walls = wall_map(wall_y=1.0)
    tracker = Tracker(static_map=walls)
    on_wall = np.array([(x, 1.02, h) for x in np.arange(-2, 2, 0.05) for h in np.arange(0.05, 1.6, 0.1)])
    leaf = np.array([(1.0 + 0.04 * i, 0.15 + 0.05 * j, h) for i in range(2) for j in range(16)
                     for h in np.arange(0.05, 1.6, 0.1)])   # 0.8 m long, 4 cm thick, from the wall
    for k in range(5):
        out = tracker.update(0.2 * k, np.vstack([on_wall, leaf]))
    assert len(out) == 1 and out[0]["class"] == "static_unknown"


def test_track_states_and_message():
    tracker = Tracker()
    tracker.update(0.0, cylinder(1.0, 0.0))
    assert tracker.current(0.0) == []                     # tentative tracks are not reported
    for k in range(1, 4):
        tracker.update(0.2 * k, cylinder(1.0, 0.0))
    message = tracker.message(0.6)
    assert message["schema"] == SCHEMA and message["frame"] == "odom" and len(message["tracks"]) == 1
    keys = {"id", "x", "y", "vx", "vy", "speed", "speed_std", "radius_m", "height_m", "points", "age_s", "seen_by",
            "state", "class", "wall_gap_m", "has_moved"}
    assert set(message["tracks"][0]) == keys
    tracker.update(1.8, np.zeros((0, 3)))
    assert tracker.current(1.8)[0]["state"] == "lost"
    tracker.update(3.0, np.zeros((0, 3)))
    assert tracker.current(3.0) == []
    with pytest.raises(ValueError):
        tracker.update(2.0, np.zeros((0, 3)))


@pytest.mark.parametrize("bad", [{"cell_m": 0.0}, {"lost_after_s": 3.0}, {"min_height_m": 2.0},
                                 {"confirm_hits": 2.5}])
def test_bad_tracker_settings_are_rejected(bad):
    with pytest.raises(ValueError):
        TrackerConfig(**bad)
