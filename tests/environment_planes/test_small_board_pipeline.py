"""Analytic finite-board rays through the complete controlled-board pipeline.

All geometry and sensor origins below are synthetic. The ray/rectangle
intersection is independent of production fitting, segmentation, and matching.
This tests software geometry and rejection rules, not real sensor accuracy.
"""
from __future__ import annotations

import copy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from wc_calibration.small_board_calibration import analyse_board_captures


WIDTH, HEIGHT, FRAME_COUNT = 160, 60, 8


def _analytic_rotation(angles_deg):
    x, y, z = np.radians(angles_deg)
    cx, cy, cz, sx, sy, sz = np.cos(x), np.cos(y), np.cos(z), np.sin(x), np.sin(y), np.sin(z)
    rx = np.array([[1., 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1., 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1.]])
    return rz @ ry @ rx


def _rays():
    horizontal, vertical = np.meshgrid(np.radians(np.linspace(45., -45., WIDTH)),
                                      np.radians(np.linspace(20., -20., HEIGHT)))
    return np.column_stack((np.cos(vertical).ravel() * np.cos(horizontal).ravel(),
                            np.cos(vertical).ravel() * np.sin(horizontal).ravel(),
                            np.sin(vertical).ravel()))


def _finite_board_capture(cid, role, rotation, translation, *, yaw_deg=0., pitch_deg=0.,
                          centre=(1.1, -.23, 0.), seed=101):
    rays = _rays()
    centre = np.asarray(centre, dtype=float)
    yaw, pitch = np.radians([yaw_deg, pitch_deg])
    normal = np.array([-np.cos(yaw)*np.cos(pitch), np.sin(yaw)*np.cos(pitch), np.sin(pitch)])
    tangent_u = np.cross([0., 0., 1.], normal)
    tangent_u /= np.linalg.norm(tangent_u)
    tangent_v = np.cross(normal, tangent_u)
    rng = np.random.default_rng(seed)
    capture = {
        "capture_id": cid, "role": role, "source_mode": "synthetic", "units": "m",
        "coordinate_conventions": {"left": "FLU", "right": "FLU"},
        "sensor_ids": {"left": "SYNTHETIC_FINITE_BOARD_LEFT", "right": "SYNTHETIC_FINITE_BOARD_RIGHT"},
        "width": WIDTH, "height": HEIGHT, "clouds": {}, "frames": {},
        "provenance": {"kind": "ANALYTIC_RAYS_FINITE_50CM_RECTANGLE_SYNTHETIC_ONLY",
            "board_dimensions_m": [.5, .5], "board_centre_left_m": centre.tolist(),
            "background_plane_left_x_m": 3.4, "range_noise_std_m": .0006,
            "physical_static_independently_verified": False,
            "hardware_synchronization_claimed": False},
    }
    for side in ("left", "right"):
        origin = np.zeros(3) if side == "left" else np.asarray(translation)
        directions_left = rays if side == "left" else rays @ rotation.T
        # Solve n dot (origin + s * ray - centre) = 0 in the left frame.
        with np.errstate(divide="ignore", invalid="ignore"):
            board_range = float(normal @ (centre-origin)) / (directions_left @ normal)
            background_range = (3.4-origin[0]) / directions_left[:, 0]
        intersections = origin + board_range[:, None] * directions_left
        local = intersections-centre
        inside = (np.isfinite(board_range) & (board_range > 0)
                  & (np.abs(local @ tangent_u) <= .25)
                  & (np.abs(local @ tangent_v) <= .25))
        ranges = np.where(inside & (board_range < background_range), board_range, background_range)
        assert np.isfinite(ranges).all() and np.min(ranges) > 0
        # Geometry preconditions are independent of the automatic detector.
        hits = np.flatnonzero(inside)
        rows, cols = hits // WIDTH, hits % WIDTH
        assert len(hits) > 100
        assert rows.min() > 0 and rows.max() < HEIGHT-1
        assert cols.min() > 0 and cols.max() < WIDTH-1
        noisy_ranges = ranges[None, :] + rng.normal(0., .0006, (FRAME_COUNT, len(rays)))
        frames = noisy_ranges[:, :, None] * rays[None, :, :]
        cloud = np.median(noisy_ranges, axis=0)[:, None] * rays
        capture["frames"][side] = frames
        capture["clouds"][side] = cloud.tolist()
    return capture


def _sequence(rotation, translation, *, yaw_only=False):
    train = [(0., 0., 1.05), (28., 0., 1.10), (0., 28., 1.15)]
    holdout = [(-9., 5., 1.10), (25., -5., 1.15), (-5., 30., 1.05)]
    if yaw_only:
        train = [(-28., 0., 1.05), (0., 0., 1.10), (28., 0., 1.15)]
        holdout = [(-23., 0., 1.10), (4., 0., 1.15), (25., 0., 1.05)]
    captures = []
    for role, poses in (("train", train), ("holdout", holdout)):
        for i, (yaw, pitch, distance) in enumerate(poses):
            captures.append(_finite_board_capture(f"synthetic_{role}_board_{i}", role,
                rotation, translation, yaw_deg=yaw, pitch_deg=pitch,
                centre=(distance, -.23, 0.), seed=301 + len(captures)))
    return captures


def _diagnostic_failure(report):
    return {"result": report["result"], "captures": [
        {"capture_id": c["capture_id"], **{side: {
            "diagnostic_status": c["controlled_board_diagnostics"][side]["status"],
            "diagnostic_reasons": c["controlled_board_diagnostics"][side]["reasons"],
            "adapter_status": c[side]["status"], "adapter_reasons": c[side]["reasons"]}
            for side in ("left", "right")}} for c in report["captures"]]}


@pytest.mark.parametrize("angles,translation", [
    ([1.3, -1.7, 2.2], [.025, -.450, .012]),
    ([-1.8, .9, -2.7], [-.015, -.476, -.009]),
])
def test_analytic_finite_board_pipeline_recovers_known_two_sensor_installations(angles, translation):
    rotation, translation = _analytic_rotation(angles), np.asarray(translation)
    report = analyse_board_captures(_sequence(rotation, translation), same_board_face_confirmed=True)
    result = report["result"]
    assert result["status"] == "CANDIDATE", _diagnostic_failure(report)
    assert report["verification_level"] == result["verification_level"] == "SYNTHETIC"
    assert result["validation"]["holdout_passed"] is True
    assert result["conditioning"]["rank"] == 3
    assert result["holdout_summary"]["conditioning"]["rank"] == 3
    assert result["train_summary"]["independent_capture_count"] == 3
    assert result["holdout_summary"]["independent_capture_count"] == 3
    matrix = np.asarray(result["transform_right_to_left"])
    angle = np.degrees(np.arccos(np.clip((np.trace(matrix[:3, :3] @ rotation.T)-1)/2, -1, 1)))
    assert angle < .15, (angle, _diagnostic_failure(report))
    np.testing.assert_allclose(matrix[:3, 3], translation, atol=.003)
    assert report["live_eligible"] is result["live_eligible"] is False
    assert result["validation"]["absolute_accuracy_certified"] is False
    for capture in report["captures"]:
        for side in ("left", "right"):
            assert capture["controlled_board_diagnostics"][side]["status"] == "DIAGNOSTIC_AVAILABLE"
            assert capture[side]["status"] == "PASS"
            assert len(capture[side]["planes"]) == 1
            plane = capture[side]["planes"][0]
            assert plane["temporal"]["status"] == "PASS"
            assert plane["physical_target_identity_verified"] is False
            assert plane["projected_mesh_area_m2"] >= .08


def test_finite_board_yaw_only_does_not_fake_full_translation_observability():
    rotation = _analytic_rotation([1.3, -1.7, 2.2])
    translation = np.array([.025, -.450, .012])
    report = analyse_board_captures(_sequence(rotation, translation, yaw_only=True),
                                   same_board_face_confirmed=True)
    result = report["result"]
    assert result["status"] == "REJECTED", _diagnostic_failure(report)
    assert result["transform_right_to_left"] is None
    # Noisy near-coplanarity may produce numerical rank 3, but must fail the
    # declared condition-number gate rather than invent a vertical translation.
    conditioning = result["conditioning"]
    assert conditioning["rank"] < 3 or conditioning["condition"] > 20., conditioning


def test_finite_board_capture_reuse_or_mixed_source_is_rejected():
    rotation = _analytic_rotation([1.3, -1.7, 2.2])
    translation = np.array([.025, -.450, .012])
    first = _finite_board_capture("synthetic_reused", "train", rotation, translation)
    duplicate = copy.deepcopy(first)
    duplicate["role"] = "holdout"
    with pytest.raises(ValueError, match="repeated capture"):
        analyse_board_captures([first, duplicate], same_board_face_confirmed=True)
    mixed = copy.deepcopy(first)
    mixed["capture_id"], mixed["source_mode"], mixed["role"] = "synthetic_mislabelled", "real", "holdout"
    with pytest.raises(ValueError, match="consistently labelled"):
        analyse_board_captures([first, mixed], same_board_face_confirmed=True)


def test_operator_same_face_confirmation_is_required_before_processing():
    with pytest.raises(ValueError, match="same single physical board face"):
        analyse_board_captures([], same_board_face_confirmed=False)

