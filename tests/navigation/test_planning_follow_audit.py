"""Follow-mode audit (no ROS): scans contradicting the driver's route, turning in place, speed by gap."""
import csv
import json

import numpy as np
import pytest

from wc_navigation.planning import follow_audit as fa
from wc_navigation.planning.body import Body

OUTLINE = ((0.81, 0.2425), (0.2, 0.35), (-0.255, 0.35), (-0.255, -0.35), (0.2, -0.35), (0.81, -0.2425))


def straight_recording(phantom_at=None):
    """Driver along +x at 0.5 m/s; scans see walls at y = +-1.1 and optionally a return ``phantom_at`` m ahead."""
    times = np.arange(0.0, 20.0, 0.2)
    poses = np.column_stack([0.5 * times, np.zeros_like(times), np.zeros_like(times)])
    lever = np.array([0.323, 0.229])
    angle_min, increment, bins = -np.pi / 3, np.pi / 240, 161
    angles = angle_min + increment * np.arange(bins)
    ranges = np.full((len(times), bins), np.inf, dtype=np.float32)
    for k in range(len(times)):
        for j, a in enumerate(angles):
            s = np.sin(a)
            if abs(s) > 1e-6:
                wall = (1.1 - lever[1]) / s if s > 0 else (-1.1 - lever[1]) / s
                if 0.3 <= wall <= 8.0:
                    ranges[k, j] = wall
        if phantom_at is not None:
            ranges[k, bins // 2 - 2:bins // 2 + 3] = phantom_at
    lidar = poses + np.column_stack([np.full(len(times), lever[0]), np.full(len(times), lever[1]), np.zeros(len(times))])
    return {"times_s": times, "base_poses": poses, "lidar_poses": lidar, "ranges": ranges, "lever_xy": lever,
            "angle_min": np.float64(angle_min), "angle_increment": np.float64(increment)}


def write_run(tmp_path, recording, outcomes):
    run = tmp_path / "run"
    run.mkdir(parents=True)
    np.savez(run / "recording.npz", **recording)
    profile = {"radius_m": 0.55, "margin_m": 0.10, "footprint": [list(p) for p in OUTLINE]}
    (run / "run_config.json").write_text(json.dumps({"profile": profile, "body_model": "POLYGON_MEASURED_20261107",
                                                     "planning": None, "window_s": [0.0, 19.8]}))
    with (run / "samples.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["recording_time_s", "recorded_linear", "recorded_angular", "recording_moving", "nav_linear",
                         "nav_angular", "gated_linear", "outcome"])
        for k, outcome in enumerate(outcomes):
            writer.writerow([round(0.1 * k, 3), 0.5, 0.3 if k % 2 else 0.0, 1, 0.0 if outcome != "DRIVES_FORWARD" else 0.3,
                             0.3, 0.0, outcome])
    return run


def test_scan_points_follow_the_capture_pose():
    recording = straight_recording(phantom_at=1.2)
    points = fa.world_scan_points(recording, 10)
    x0 = recording["base_poses"][10, 0]
    assert np.isclose(points[:, 1], 1.1, atol=1e-6).any() and np.isclose(points[:, 1], -1.1, atol=1e-6).any()
    ahead = points[np.abs(points[:, 1] - 0.229) < 0.05]
    assert np.allclose(ahead[:, 0], x0 + 0.323 + 1.2, atol=0.02)


def test_stops_are_split_by_whether_the_scan_contradicts_the_route(tmp_path):
    outcomes = ["DRIVES_FORWARD"] * 60 + ["CONTROLLER_ZERO"] * 5 + ["DRIVES_FORWARD"] * 30 + ["ROTATES_IN_PLACE"] * 4
    clean = fa.audit(write_run(tmp_path / "a", straight_recording(), outcomes))
    assert clean["controller_stops"] == 1 and clean["stops_without_scan_points_on_driver_route"] == 1
    phantom = fa.audit(write_run(tmp_path / "b", straight_recording(phantom_at=0.9), outcomes))
    assert phantom["stops_with_scan_points_on_driver_route"] == 1
    assert phantom["stops"][0]["scan_points_in_body_on_driver_route"] > 0
    assert phantom["turn_in_place_while_moving_s"] == pytest.approx(0.4)
    assert phantom["turn_in_place_while_driver_turns_forward_s"] == pytest.approx(0.2)


def test_points_inside_the_body_along_a_route():
    body = Body(outline=OUTLINE)
    route = np.column_stack([np.linspace(0.0, 0.6, 13), np.zeros(13), np.zeros(13)])
    assert fa.points_in_body_along(route, np.array([[1.3, 0.0], [1.0, 0.5], [3.0, 0.0]]), body, 0.0) == 1
    assert fa.points_in_body_along(route, np.zeros((0, 2)), body, 0.0) == 0
