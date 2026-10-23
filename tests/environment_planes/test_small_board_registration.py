"""One finite 50 cm board per capture, independent multi-pose geometry checks.

These tests bypass extraction intentionally: a solver test cannot demonstrate
that a 50 cm real board survives the existing large-plane extraction gates.
Analytic planes come from a 38 cm square interior of a 50 cm board. Geometry is
synthetic and remains labelled synthetic, even with simulated temporal PASS.
"""
from __future__ import annotations

import copy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from wc_calibration.environment_registration import solve_environment


def _rotation_xyz_deg(angles):
    """Construct the truth independently of solver/SciPy rotation helpers."""
    x, y, z = np.radians(angles)
    rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
    ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    return rz @ ry @ rx


def _board_capture(cid, role, yaw_deg, pitch_deg, rotation, translation, distance=1.1):
    """Same physical finite board, transformed independently into both frames."""
    yaw, pitch = np.radians([yaw_deg, pitch_deg])
    normal = np.array([-np.cos(yaw) * np.cos(pitch), np.sin(yaw) * np.cos(pitch), np.sin(pitch)])
    centre = np.array([distance, 0.0, 0.0])
    tangent_u = np.cross([0.0, 0.0, 1.0], normal)
    tangent_u /= np.linalg.norm(tangent_u)
    tangent_v = np.cross(normal, tangent_u)
    # The full board has +/-0.25 m bounds; omit 6 cm around its four edges.
    coords = np.linspace(-0.19, 0.19, 12)
    uv = np.array([(u, v) for u in coords for v in coords])
    points_left = centre + uv[:, :1] * tangent_u + uv[:, 1:] * tangent_v
    points_right = (rotation.T @ (points_left - translation).T).T
    normals = {"left": normal, "right": rotation.T @ normal}
    points = {"left": points_left, "right": points_right}
    record = {"capture_id": cid, "role": role, "source_mode": "synthetic",
              "sensor_ids": {"left": "SYNTHETIC_BOARD_LEFT", "right": "SYNTHETIC_BOARD_RIGHT"}}
    for side in ("left", "right"):
        cloud, n = points[side], normals[side]
        centroid = cloud.mean(axis=0)
        d = -float(n @ centroid)
        assert d > 0
        assert np.max(np.abs(cloud @ n + d)) < 1e-12
        record[side] = {"planes": [{"id": "board_interior", "normal": n.tolist(),
            "d_m": d, "centroid": centroid.tolist(), "span_m": [0.342, 0.342],
            "rmse_m": 0.0, "p95_m": 0.0, "point_count": len(cloud),
            "indices": list(range(len(cloud))), "quality": {"status": "PASS"},
            "status": "PASS", "temporal": {"status": "PASS", "independent_validation": False}}]}
    return record


def _six_captures(rotation, translation):
    training_poses = [(0, 0), (32, 0), (0, 30)]
    holdout_poses = [(-10, 5), (27, -7), (-5, 34)]
    records = []
    for role, poses in (("train", training_poses), ("holdout", holdout_poses)):
        for index, (yaw, pitch) in enumerate(poses):
            records.append(_board_capture(f"{role}_board_{index}", role, yaw, pitch,
                                          rotation, translation, distance=1.05 + index * .05))
    return records


def _truth_matrix(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, translation
    return matrix


@pytest.mark.parametrize("angles,translation", [
    ([1.5, -2.0, 3.0], [0.035, -0.440, -0.018]),
    ([-2.5, 1.0, -1.8], [-0.026, -0.486, 0.023]),
])
def test_three_training_and_three_holdout_single_planes_recover_full_transform(angles, translation):
    rotation, translation = _rotation_xyz_deg(angles), np.asarray(translation)
    records = _six_captures(rotation, translation)
    assert all(len(c["left"]["planes"]) == len(c["right"]["planes"]) == 1 for c in records)
    result = solve_environment(records)
    assert result["status"] == "CANDIDATE", result  # Synthetic is never a real-capture pass.
    assert result["validation"]["holdout_passed"] is True
    assert result["conditioning"]["rank"] == 3
    assert result["holdout_summary"]["conditioning"]["rank"] == 3
    assert result["train_summary"]["independent_capture_count"] == 3
    assert result["holdout_summary"]["independent_capture_count"] == 3
    assert len(result["matches"]) == 6
    np.testing.assert_allclose(result["transform_right_to_left"], _truth_matrix(rotation, translation), atol=1e-9)
    assert result["live_eligible"] is False
    assert result["uncertainty"]["status"] == "NOT_ESTIMATED"


def test_three_training_single_planes_need_no_same_capture_three_plane_requirement():
    rotation = _rotation_xyz_deg([1.5, -2.0, 3.0])
    translation = np.array([0.035, -0.440, -0.018])
    result = solve_environment(_six_captures(rotation, translation)[:3])
    assert result["status"] == "CANDIDATE", result
    assert result["validation"]["independent_holdout_performed"] is False
    np.testing.assert_allclose(result["transform_right_to_left"], _truth_matrix(rotation, translation), atol=1e-9)


def test_only_yaw_board_poses_leave_vertical_translation_unobservable():
    rotation = _rotation_xyz_deg([1.5, -2.0, 3.0])
    translation = np.array([0.035, -0.440, -0.018])
    captures = [_board_capture(f"yaw_{i}", "train", yaw, 0, rotation, translation)
                for i, yaw in enumerate([-30, 0, 30])]
    result = solve_environment(captures)
    assert result["status"] == "REJECTED", result
    assert result["conditioning"]["rank"] == 2
    assert result["transform_right_to_left"] is None


def test_holdout_plane_offset_errors_reject_validation_without_changing_training():
    rotation = _rotation_xyz_deg([-2.5, 1.0, -1.8])
    translation = np.array([-0.026, -0.486, 0.023])
    records = _six_captures(rotation, translation)
    training = solve_environment(records[:3])
    changed = copy.deepcopy(records)
    p = changed[3]["right"]["planes"][0]
    p["d_m"] += .18
    # Maintain a valid plane representation while moving the measured plane.
    p["centroid"] = (np.asarray(p["centroid"]) - .18 * np.asarray(p["normal"])).tolist()
    validation = solve_environment(changed)
    assert validation["status"] == "REJECTED", validation
    assert validation["validation"]["holdout_passed"] is False
    np.testing.assert_allclose(validation["transform_right_to_left"], training["transform_right_to_left"], atol=1e-12)
    assert validation["validation"]["holdout_used_for_fitting"] is False


def test_only_yaw_holdout_does_not_independently_validate_vertical_component():
    rotation = _rotation_xyz_deg([1.5, -2.0, 3.0])
    translation = np.array([0.035, -0.440, -0.018])
    captures = _six_captures(rotation, translation)[:3]
    captures += [_board_capture(f"validation_yaw_{i}", "holdout", yaw, 0, rotation, translation)
                 for i, yaw in enumerate([-30, 0, 30])]
    result = solve_environment(captures)
    assert result["status"] == "REJECTED", result
    assert result["holdout_summary"]["conditioning"]["rank"] == 2
    assert "HOLDOUT_DOES_NOT_INDEPENDENTLY_COVER_THREE_WELL_CONDITIONED_DIRECTIONS" in result["reasons"]
    np.testing.assert_allclose(result["transform_right_to_left"], _truth_matrix(rotation, translation), atol=1e-9)


def test_initial_transform_is_a_declared_correspondence_search_window_not_a_fitted_prior():
    rotation = _rotation_xyz_deg([0, 0, 35])
    translation = np.array([0.035, -0.440, -0.018])
    captures = _six_captures(rotation, translation)
    narrow_default = solve_environment(captures)
    assert narrow_default["status"] == "REJECTED", narrow_default
    initial = _truth_matrix(_rotation_xyz_deg([0, 0, 32]), np.array([.01, -.4575, .01]))
    result = solve_environment(captures, initial_transform=initial)
    assert result["status"] == "CANDIDATE", result
    assert result["search_parameters"]["prior_in_optimization"] is False
    np.testing.assert_allclose(result["transform_right_to_left"], _truth_matrix(rotation, translation), atol=1e-9)

