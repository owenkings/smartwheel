"""Obstacle tracks: clustering, constant-velocity tracking, classes, map removal and the message schema."""
import math

import numpy as np
import pytest

from wc_navigation.perception.tracks import (ENTRY_KEYS, SCHEMA, StaticMap, Tracker, TrackerConfig, check_message,
                                             clusters, track_entry, tracks_message)


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
    pts = np.vstack([cylinder(0.0, 0.0), cylinder(2.0, 0.0), [[5.0, 5.0, 0.5]] * 3,
                     cylinder(-2.0, 0.0, h0=1.9, h1=2.2)])
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


def test_objects_riding_along_with_the_chair_or_sliding_along_a_wall_are_not_moving():
    walls = wall_map(wall_y=1.0)
    plain, informed = Tracker(), Tracker()
    walled = Tracker(static_map=walls)
    for k in range(15):
        t = 0.2 * k
        # something on the floor that stays 2 m ahead of the chair driving 0.6 m/s along +x
        rider = cylinder(2.0 + 0.6 * t, -2.0)
        plain.update(t, rider)
        informed.update(t, rider, chair_velocity=(0.6, 0.0))
        # the visible part of an unmapped skirting board sliding along the wall's edge
        walled.update(t, cylinder(0.6 * t, 0.85, r=0.15, h1=0.4), chair_velocity=(0.0, 0.0))
    assert plain.current(2.8)[0]["class"] == "moving"
    assert informed.current(2.8)[0]["class"] != "moving"
    assert walled.current(2.8)[0]["class"] != "moving"


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
    tracker.update(0.0, cylinder(1.0, 0.0), source="left")
    assert tracker.current(0.0) == []                     # tentative tracks are not reported
    for k in range(1, 4):
        tracker.update(0.2 * k, cylinder(1.0, 0.0), source=("left", "right"))
    message = tracker.message(0.6)
    assert message["schema"] == SCHEMA and message["frame"] == "odom" and message["source"] == "perception"
    assert message["stamp"] == 0.6 and len(message["tracks"]) == 1
    track = message["tracks"][0]
    assert set(track) == set(ENTRY_KEYS) and track["seen_by"] == ["left", "right"]
    assert track["last_seen_s"] == 0.0 and track["vx_std"] > 0 and track["vy_std"] > 0
    assert track["speed_std"] == pytest.approx(math.hypot(track["vx_std"], track["vy_std"]), abs=0.002)
    assert check_message(message) is message
    tracker.update(1.8, np.zeros((0, 3)))
    assert tracker.current(1.8)[0]["state"] == "lost"
    tracker.update(3.0, np.zeros((0, 3)))
    assert tracker.current(3.0) == []
    with pytest.raises(ValueError):
        tracker.update(2.0, np.zeros((0, 3)))


def test_tentative_tracks_are_never_published_not_even_when_lost():
    tracker = Tracker()
    tracker.update(0.0, cylinder(1.0, 0.0))
    tracker.update(0.2, cylinder(1.0, 0.0))               # two hits: still tentative
    for t in (1.4, 1.8):
        assert tracker.update(t, np.zeros((0, 3))) == []


def test_a_lost_walker_keeps_its_class_and_last_velocity_until_dropped():
    tracker = Tracker()
    for k in range(15):                                  # 1.0 m/s along +x, then the walker leaves the view
        tracker.update(0.2 * k, cylinder(1.0 + 0.2 * k, 0.0))
    seen = tracker.current(2.8)[0]
    assert seen["class"] == "moving" and seen["state"] == "confirmed"
    tracker.update(3.0, np.zeros((0, 3)))
    still = tracker.current(3.0)[0]
    assert still["state"] == "confirmed" and still["last_seen_s"] == pytest.approx(0.2)
    tracker.update(4.0, np.zeros((0, 3)))
    lost = tracker.current(4.0)[0]
    assert lost["state"] == "lost" and lost["class"] == "moving" and lost["has_moved"]
    assert lost["last_seen_s"] == pytest.approx(1.2) and lost["age_s"] == pytest.approx(4.0)
    assert (lost["vx"], lost["vy"]) == (seen["vx"], seen["vy"])          # constant-velocity extrapolation
    assert lost["x"] - lost["vx"] * lost["last_seen_s"] == pytest.approx(seen["x"], abs=0.01)
    assert lost["vx_std"] > seen["vx_std"] and lost["speed_std"] > seen["speed_std"]
    tracker.update(4.9, np.zeros((0, 3)))
    assert tracker.current(4.9) == []                     # dropped 2 s after the last match


def test_the_message_builder_serves_truth_tracks_and_the_check_rejects_broken_messages():
    truth = track_entry(1, 2.0, -0.5, -1.0, 0.0, 0.05, 0.05, 0.25, "moving", age_s=3.0, seen_by=["truth"])
    assert set(truth) == set(ENTRY_KEYS) and truth["height_m"] is None and truth["points"] == 0
    assert truth["has_moved"] and truth["speed"] == 1.0 and truth["speed_std"] == pytest.approx(0.071, abs=0.001)
    message = tracks_message(12.3456, [truth], source="truth")
    assert message["stamp"] == 12.346 and check_message(message) is message
    extra = dict(truth, label="walker")                   # unknown keys are ignored by consumers
    check_message(tracks_message(1.0, [extra], source="truth"))
    broken = [dict(truth, **{"class": "person"}), dict(truth, state="tentative"), dict(truth, vx=float("nan")),
              dict(truth, vx_std=-0.1), dict(truth, speed=3.0), dict(truth, last_seen_s=5.0),
              dict(truth, height_m=[1.0]), dict(truth, id=0), {k: v for k, v in truth.items() if k != "vy_std"}]
    for entry in broken:
        with pytest.raises(ValueError):
            check_message({"schema": SCHEMA, "stamp": 1.0, "frame": "odom", "source": "truth", "tracks": [entry]})
    with pytest.raises(ValueError):
        tracks_message(1.0, [truth, truth])               # duplicate ids
    for bad in ({"schema": "other"}, {"schema": SCHEMA, "stamp": 1.0, "frame": "odom", "source": "x", "tracks": []},
                {"schema": SCHEMA, "stamp": None, "frame": "odom", "source": "truth", "tracks": []}):
        with pytest.raises(ValueError):
            check_message(bad)


@pytest.mark.parametrize("bad", [{"cell_m": 0.0}, {"lost_after_s": 3.0}, {"min_height_m": 2.0},
                                 {"confirm_hits": 2.5}])
def test_bad_tracker_settings_are_rejected(bad):
    with pytest.raises(ValueError):
        TrackerConfig(**bad)
