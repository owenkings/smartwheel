"""Planner bench scoring (no ROS): recorded requests, route categories, true-body path scores, summaries."""
import math

import numpy as np
import pytest

from wc_navigation.planning import bench
from wc_navigation.planning.body import Body

OUTLINE = ((0.81, 0.2425), (0.2, 0.35), (-0.255, 0.35), (-0.255, -0.35), (0.2, -0.35), (0.81, -0.2425))  # MEASURED_20261107


def corridor(width=1.2, length=8.0, resolution=0.05):
    """Free corridor along x of the given width centred on y = 1.0, walls 0.2 m, unknown outside."""
    rows, cols = int(round(2.0 / resolution)), int(round(length / resolution))
    data = np.full((rows, cols), -1, dtype=np.int8)
    lo, hi = int(round((1.0 - width / 2) / resolution)), int(round((1.0 + width / 2) / resolution))
    data[lo - 4:hi + 4, :] = 100
    data[lo:hi, 4:-4] = 0
    return data


def body():
    return Body(outline=OUTLINE, source="POLYGON_MEASURED_20261107")


def test_requests_follow_the_recording():
    times = np.arange(0.0, 40.0, 0.2)
    poses = np.column_stack([0.5 + 0.2 * times, np.ones_like(times), np.zeros_like(times)])
    requests = bench.make_requests(times, poses, (0.0, 30.0), lead_s=15.0, every_s=4.0)
    assert [r["t"] for r in requests][:3] == [0.0, 4.0, 8.0]
    first = requests[0]
    assert first["start"][0] == pytest.approx(0.5) and first["goal"][0] == pytest.approx(0.5 + 0.2 * 15.0)
    assert requests[-1]["goal_t"] == pytest.approx(30.0)                     # clipped at the window end
    assert np.all(np.diff(first["driver"][:, 0]) > 0.0)


def test_true_body_gap_and_overlap_of_a_planned_path():
    geo = bench.MapGeometry(corridor(width=1.2), 0.05, (0.0, 0.0))
    centre = np.column_stack([np.linspace(1.0, 6.0, 60), np.full(60, 1.0)])
    score = bench.score_path(geo, body(), centre)
    assert score["min_gap_m"] == pytest.approx(0.6 - 0.35, abs=1e-6) and score["poses_overlapping_blocked"] == 0
    hugging = centre + [0.0, 0.3]                                             # 0.05 m past the wall at y = 1.6
    assert bench.score_path(geo, body(), hugging)["poses_overlapping_blocked"] > 0
    turning = np.column_stack([np.r_[np.linspace(1.0, 3.0, 20), np.full(20, 3.0)], np.r_[np.full(20, 1.0),
                                                                                         np.linspace(1.0, 1.4, 20)]])
    assert bench.score_path(geo, body(), turning)["sharp_turns"] >= 1


def test_planner_headings_reveal_rotation_in_place_and_reversing():
    geo = bench.MapGeometry(corridor(width=1.6), 0.05, (0.0, 0.0))
    xy = np.array([[2.0, 1.0], [2.0, 1.0], [2.0, 1.0], [2.5, 1.0], [3.0, 1.0]])
    spin = bench.score_path(geo, body(), xy, np.array([0.0, 0.8, 0.0, 0.0, 0.0]))
    assert spin["in_place_rotation_rad"] == pytest.approx(1.6) and spin["reverse_m"] == 0.0
    backwards = bench.score_path(geo, body(), xy[2:], np.array([math.pi, math.pi, math.pi]))
    assert backwards["reverse_m"] == pytest.approx(1.0)


def test_route_category_separates_map_conflicts_from_tight_spots():
    assert bench.route_category(0, 0) == "clear"
    assert bench.route_category(0, 3) == "tight"
    assert bench.route_category(2, 5) == "conflict"
    geo = bench.MapGeometry(corridor(width=0.85), 0.05, (0.0, 0.0))
    request = {"start": [1.5, 1.0, 0.0], "goal": [6.0, 1.0, 0.0],
               "driver": np.column_stack([np.linspace(1.5, 6.0, 30), np.ones(30), np.zeros(30)])}
    context = bench.request_context(geo, body(), 0.10, request)
    assert context["category"] == "tight"                     # 0.70 m body fits 0.85 m, the 0.10 m margin does not
    assert context["min_passage_width_m"] == pytest.approx(0.85, abs=0.06)
    narrow = bench.request_context(bench.MapGeometry(corridor(width=0.65), 0.05, (0.0, 0.0)), body(), 0.10, request)
    assert narrow["category"] == "conflict" and not narrow["start_fits_padded_body"]


def test_summary_counts_only_safe_plans_as_usable():
    context = {"min_passage_width_m": 1.0, "category": "tight"}
    good = {"length_m": 3.0, "min_gap_m": 0.1, "poses_overlapping_occupied": 0, "poses_overlapping_blocked": 0,
            "sharp_turns": 0, "in_place_rotation_rad": 0.0, "reverse_m": 0.0, "max_deviation_from_driver_m": 0.2}
    bad = dict(good, min_gap_m=0.0, poses_overlapping_blocked=4, poses_overlapping_occupied=2)
    rows = [{"planner": "a", "status": "SUCCEEDED", "planning_time_s": 0.01, "context": context,
             "driver_length_m": 3.0, "score": good},
            {"planner": "a", "status": "SUCCEEDED", "planning_time_s": 0.02, "context": context,
             "driver_length_m": 3.0, "score": bad},
            {"planner": "a", "status": "ABORTED", "planning_time_s": 0.0, "context": context, "driver_length_m": 3.0}]
    summary = bench.summarize(rows, ["a"])["a"]
    assert summary["all"]["success_fraction"] == pytest.approx(2 / 3)
    assert summary["all"]["safe_success_fraction"] == pytest.approx(1 / 3) and summary["all"]["unsafe_succeeded"] == 1
    assert summary["tight"]["requests"] == 3 and summary["narrow"]["requests"] == 3 and summary["clear"]["requests"] == 0
