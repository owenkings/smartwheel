"""Reaction to people (planning/people.py): prediction, gaps, speed limit with yield, keepout masks."""
import math

import numpy as np
import pytest

from wc_navigation.planning import people as pp
from wc_navigation.planning.body import Body

OUTLINE = ((0.81, 0.2425), (0.2, 0.35), (-0.255, 0.35), (-0.255, -0.35), (0.2, -0.35), (0.81, -0.2425))  # MEASURED_20261107
CFG = pp.PeopleConfig()


def body():
    return Body(outline=OUTLINE, source="POLYGON_MEASURED_20261107")


def track(track_id, x, y, vx=0.0, vy=0.0, cls="moving", state="confirmed", radius=0.25, std=0.05, has_moved=None):
    """A ``wc_obstacle_tracks/1`` entry with every key (perception/tracks.py layout)."""
    return {"id": track_id, "x": x, "y": y, "vx": vx, "vy": vy, "vx_std": std, "vy_std": std,
            "speed": round(math.hypot(vx, vy), 3), "speed_std": round(math.hypot(std, std), 3), "radius_m": radius,
            "height_m": None, "points": 0, "age_s": 2.0, "last_seen_s": 0.0, "seen_by": ["truth"], "state": state,
            "class": cls, "wall_gap_m": None, "elongation": None,
            "has_moved": cls == "moving" if has_moved is None else has_moved}


def message(stamp, tracks):
    return {"schema": "wc_obstacle_tracks/1", "stamp": stamp, "frame": "odom", "source": "truth", "tracks": tracks}


def test_people_are_moving_or_person_shaped_tracks_brought_to_now():
    m = message(10.0, [track(1, 3.0, 0.0, vx=-1.0), track(2, 2.0, 1.0, cls="person_candidate"),
                       track(3, 4.0, 0.0, cls="static_unknown"), track(4, 5.0, 0.0, cls="static_unknown", has_moved=True)])
    people, age = pp.people_from_message(m, 10.2, CFG)
    assert age == pytest.approx(0.2) and [p.id for p in people] == [1, 2, 4]
    walker = people[0]
    # brought forward over the age (0.2 s) plus the range lag of the filtered lidar frames (0.07 s)
    assert walker.moving and walker.x == pytest.approx(3.0 - 0.27) and walker.radius_m > 0.25
    assert walker.growth_mps == pytest.approx(max(CFG.min_growth_mps, CFG.growth_sigma * 0.071))
    assert not people[1].moving and (people[1].vx, people[1].vy) == (0.0, 0.0)
    assert pp.people_from_message(m, 11.5, CFG)[0] == []             # stale message: nobody


def test_range_lag_brings_an_oncoming_person_9_cm_closer():
    """Filtered ranges lag about 70 ms: 1 m/s against 0.3 m/s is (1.0 + 0.3) x 0.07 = 0.091 m less margin."""
    b = body()
    chair = ((0.0, 0.0, 0.0), (0.3, 0.0))
    m = message(10.0, [track(1, 4.0, 0.0, vx=-1.0, std=0.0)])
    margins = []
    for lag in (0.0, 0.07):
        cfg = pp.PeopleConfig(range_lag_s=lag, min_growth_mps=0.0, growth_sigma=0.0)
        people, _ = pp.people_from_message(m, 10.0, cfg)
        a = pp.assess(people, *chair, b, cfg)[0]
        margins.append(a["gap_now_m"] - a["clearance_m"])
    assert margins[0] - margins[1] == pytest.approx((1.0 + 0.3) * 0.07)
    assert pp.lag_margin((0.3, 0.2), CFG) == pytest.approx(0.021) and pp.lag_margin((-0.2, 0.0), CFG) == pytest.approx(0.014)
    with pytest.raises(ValueError):
        pp.PeopleConfig(range_lag_s=0.6)


def test_chair_arc_and_frames():
    assert pp.chair_at((0.0, 0.0, 0.0), 0.3, 0.0, 2.0) == pytest.approx((0.6, 0.0, 0.0))
    x, y, yaw = pp.chair_at((0.0, 0.0, 0.0), 0.5, 0.5, math.pi)   # quarter circle of radius 1
    assert (x, y, yaw) == pytest.approx((1.0, 1.0, math.pi / 2), abs=1e-9)
    assert pp.chair_at((0.0, 0.0, 0.0), 0.5, 0.5, 2 * math.pi) == pytest.approx((0.0, 2.0, math.pi), abs=1e-9)
    moved = pp.transform_people([pp.Person(1, 1.0, 0.0, 1.0, 0.0, 0.25, True)], (2.0, 3.0, math.pi / 2))[0]
    assert (moved.x, moved.y, moved.vx, moved.vy) == pytest.approx((2.0, 4.0, 0.0, 1.0))


def test_body_gap_measures_from_the_outline():
    b = body()
    assert pp.body_gap(b, (0.0, 0.0, 0.0), 2.0, 0.0, 0.25) == pytest.approx(2.0 - 0.81 - 0.25)
    assert pp.body_gap(b, (0.0, 0.0, 0.0), 0.0, 1.0, 0.25) == pytest.approx(1.0 - 0.35 - 0.25)
    assert pp.body_gap(b, (0.0, 0.0, 0.0), 0.5, 0.0, 0.25) < 0.0


def test_speed_limit_slows_near_people_and_yields_before_the_clearance():
    b = body()
    chair = ((0.0, 0.0, 0.0), (0.3, 0.0))
    far = pp.assess([pp.Person(1, 6.0, 0.0, 0.0, 0.0, 0.25, False)], *chair, b)
    assert pp.speed_decision(far)[:2] == (None, "CLEAR")
    beside = pp.assess([pp.Person(1, 1.0, 1.4, 0.0, 0.0, 0.25, False)], *chair, b)
    assert pp.speed_decision(beside)[:2] == (CFG.slow_near_mps, "SLOW_NEAR")
    side = pp.assess([pp.Person(1, 1.0, 2.2, 0.0, 0.0, 0.25, False)], *chair, b)
    assert pp.speed_decision(side)[:2] == (CFG.slow_far_mps, "SLOW_FAR")
    # Oncoming at 1 m/s, 3.5 m ahead: within 1.5 s the gap (2.44 m now) closes below 0.5 m -> stop now.
    oncoming = pp.assess([pp.Person(1, 3.5, 0.0, -1.0, 0.0, 0.25, True, 0.1)], *chair, b)
    limit, state, who = pp.speed_decision(oncoming)
    assert state == "YIELD" and limit == CFG.stop_limit_mps and who["gap_now_m"] > 2.0
    # A standing person needs only 0.3 m: passing 0.4 m beside them is slow, not a stop.
    passing = pp.assess([pp.Person(1, 1.5, 0.35 + 0.25 + 0.4, 0.0, 0.0, 0.25, False)], *chair, b)
    assert pp.speed_decision(passing)[1] == "SLOW_NEAR"


def test_yield_releases_after_the_hold_and_waits_at_most_max_wait():
    b = body()
    cfg = pp.PeopleConfig(max_wait_s=5.0, release_s=1.0)
    reaction = pp.Reaction(cfg)
    still = ((0.0, 0.0, 0.0), (0.0, 0.0))
    blocker = [pp.Person(7, 1.6, 0.0, -0.05, 0.0, 0.25, True, 0.1)]     # creeping towards the chair
    out = reaction.update(0.0, blocker, *still, b)
    assert out["state"] == "YIELD" and out["limit_mps"] == cfg.stop_limit_mps
    assert reaction.update(4.0, blocker, *still, b)["state"] == "YIELD"
    # After 5 s the creeping person counts as standing (0.3 m, no prediction): 0.54 m ahead is room enough.
    out = reaction.update(5.0, blocker, *still, b)
    assert out["treated_as_standing"] == [7] and out["state"] == "SLOW_NEAR"
    # Once the person has left (farther than 2 m) the normal treatment returns.
    gone = [pp.Person(7, 4.5, 0.0, 0.0, 0.0, 0.25, True, 0.1)]
    assert reaction.update(6.0, gone, *still, b)["treated_as_standing"] == []
    # A yield is held for release_s after the way is clear.
    reaction = pp.Reaction(cfg)
    reaction.update(0.0, blocker, *still, b)
    away = [pp.Person(7, 1.6, 0.0, 1.0, 0.0, 0.25, True, 0.1)]
    assert reaction.update(0.5, away, *still, b)["state"] == "YIELD_RELEASING"
    assert reaction.update(1.6, away, *still, b)["state"] != "YIELD_RELEASING"
    # Yields less than wait_reset_s apart are one wait: someone crossing back and forth in front of the chair
    # is treated as standing after max_wait_s in total, not never.
    reaction = pp.Reaction(cfg)
    t = 0.0
    while t < 4.6:
        state = reaction.update(t, blocker if int(t) % 2 == 0 else away, *still, b)
        t += 0.2
    assert state["yield_s"] is not None and state["yield_s"] >= 4.0 and state["treated_as_standing"] == []
    assert reaction.update(5.0, blocker, *still, b)["treated_as_standing"] == [7]
    # After wait_reset_s without a yield the next yield starts a new wait.
    reaction = pp.Reaction(cfg)
    reaction.update(0.0, blocker, *still, b)
    reaction.update(1.0, away, *still, b)
    reaction.update(6.5, away, *still, b)
    assert reaction.update(6.6, blocker, *still, b)["yield_s"] == 0.0


def test_keepout_mask_is_lethal_near_people_and_repeats_the_inflation_curve():
    shape, res, origin = (120, 120), 0.05, (-3.0, -3.0)
    walker = pp.Person(1, 1.0, 0.0, 1.0, 0.0, 0.25, True, 0.1)
    data = pp.keepout_mask(shape, res, origin, [walker], CFG, inscribed_m=0.355, circumscribed_m=0.946,
                           cost_scaling_factor=3.0)
    assert data.shape == shape and data.dtype == np.int8

    def at(x, y):
        return int(data[int((y - origin[1]) // res), int((x - origin[0]) // res)])
    assert at(1.0, 0.0) == 100 and at(1.0, 0.70) == 100            # radius 0.25 + 0.5 m clearance
    assert at(2.2, 0.0) == 100                                     # predicted zone along the velocity
    assert at(1.0, 0.95) == 99                                     # inscribed ring: centre cost forces the check
    assert at(-2.5, -2.5) == -1                                    # untouched cells: the filter skips them
    ring = at(1.0, 1.65)
    d = 1.675 - 0.75                                               # cell centre to the zone edge
    nav2 = (pp.NAV2_INSCRIBED - 1) * math.exp(-3.0 * (d - 0.355))
    assert 0 < ring < 99 and round(ring * 254 / 100) >= nav2
    flat = pp.keepout_mask(shape, res, origin, [walker], CFG, inscribed_m=0.355, circumscribed_m=0.946,
                           cost_scaling_factor=3.0, center_point=True)
    assert int(flat[int((0.95 - origin[1]) // res), int((1.0 - origin[0]) // res)]) == 100
    standing = pp.keepout_mask(shape, res, origin, [walker.as_standing()], CFG, inscribed_m=0.355,
                               circumscribed_m=0.946, cost_scaling_factor=3.0)
    # cell centre (1.575, 0.025) is 0.576 m from the person: outside the standing zone 0.25 + 0.3 m ...
    assert int(standing[int((0.01 - origin[1]) // res), int((1.59 - origin[0]) // res)]) == 99
    wider = pp.keepout_mask(shape, res, origin, [walker.as_standing()], CFG, inscribed_m=0.355,
                            circumscribed_m=0.946, cost_scaling_factor=3.0, extra_m=0.06)
    # ... and inside it once the zone is 0.06 m wider
    assert int(wider[int((0.01 - origin[1]) // res), int((1.59 - origin[0]) // res)]) == 100


def test_config_is_checked():
    with pytest.raises(ValueError):
        pp.PeopleConfig(standing_clearance_m=0.6)
    with pytest.raises(ValueError):
        pp.PeopleConfig(stop_limit_mps=0.0)
    with pytest.raises(ValueError):
        pp.PeopleConfig(yield_horizon_s=5.0)
