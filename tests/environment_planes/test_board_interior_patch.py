"""Independent analytic pinhole rays and controlled-interior rejection cases."""
from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from wc_calibration.board_interior_patch import controlled_interior_patch, INTERIOR_PATCH_POLICY
from wc_calibration.small_board_planes import CONTROLLED_BOARD_POLICY


def _scene(*, width=80, height=60, frame_count=6):
    """Ray hits of a .5 m square at X=1.1, against the X=3 m background."""
    slope_y, slope_z = np.meshgrid(np.linspace(.6, -.6, width), np.linspace(.45, -.45, height))
    rays = np.column_stack((np.ones(width*height), slope_y.ravel(), slope_z.ravel()))
    rays /= np.linalg.norm(rays, axis=1)[:, None]
    board_range = 1.1/rays[:, 0]
    board_hits = board_range[:, None]*rays
    inside = (np.abs(board_hits[:, 1]) <= .25) & (np.abs(board_hits[:, 2]) <= .25)
    ranges = np.where(inside, board_range, 3./rays[:, 0])
    cloud = ranges[:, None]*rays
    frames = np.repeat(cloud[None, :, :], frame_count, axis=0)
    rows, cols = np.flatnonzero(inside)//width, np.flatnonzero(inside)%width
    # The bottom ROI bound is on the board's final observed row. Its outside
    # ring contains background, which must be retained but must not gate fit.
    roi = [int(cols.min()+2), int(rows.min()+2), int(cols.max()-2), int(rows.max())]
    return cloud, frames, width, height, roi


def _run(scene, *, operator_confirmed=True):
    cloud, frames, width, height, roi = scene
    return controlled_interior_patch(cloud, frames=frames, width=width, height=height,
                                     roi_xyxy=roi, operator_confirmed=operator_confirmed)


def test_confirmed_patch_uses_all_fixed_support_and_retains_context_residuals():
    cloud, frames, width, height, roi = scene = _scene()
    result = _run(scene)
    assert result["status"] == "PASS", result["reasons"]
    assert len(result["planes"]) == 1
    plane, diag = result["planes"][0], result["diagnostic"]
    np.testing.assert_allclose(plane["normal"], [-1, 0, 0], atol=1e-12)
    assert abs(plane["d_m"]-1.1) < 1e-12
    assert plane["point_count"] == len(plane["indices"]) == len(diag["interior_points_m"])
    assert plane["point_count"] >= 80 and plane["projected_mesh_area_m2"] >= .08
    assert plane["temporal"]["status"] == "PASS"
    assert plane["physical_target_identity_verified"] is False
    assert plane["board_completeness_verified"] is False
    assert plane["residual_trimming_performed"] is False
    assert diag["raw_roi_fit"]["point_count"] == len(diag["raw_roi_finite_indices"])
    assert diag["raw_roi_fit"]["residual_trimming_performed"] is False
    assert len(diag["raw_roi_indices"]) == (roi[2]-roi[0]+1)*(roi[3]-roi[1]+1)
    assert set(diag["interior_indices"]).issubset(diag["raw_roi_indices"])
    assert set(diag["interior_indices"]).isdisjoint(diag["roi_depth_edge_excluded_indices"])
    # Real non-board context is retained without contaminating the board gate.
    assert diag["outside_ring_residual_to_interior_plane"]["statistics"]["p95_m"] > 1.
    assert diag["whole_image_residual_to_interior_plane"]["statistics"]["p95_m"] > 1.
    assert len(diag["whole_image_residual_to_interior_plane"]["finite_indices"]) == width*height
    assert diag["context_is_board_accuracy_gate"] is False
    json.dumps(result, allow_nan=False)


def test_unconfirmed_roi_still_computes_full_diagnostic_but_no_qualified_plane():
    result = _run(_scene(), operator_confirmed=False)
    assert result["status"] == "REJECTED" and not result["planes"]
    assert "OPERATOR_ROI_SAME_BOARD_FACE_NOT_CONFIRMED" in result["reasons"]
    assert result["diagnostic"]["status"] == "DIAGNOSTIC_AVAILABLE"
    assert result["diagnostic"]["raw_roi_fit"] is not None
    assert result["diagnostic"]["interior_fit"] is not None
    assert result["rejected_planes"][0]["quality"]["status"] == "PASS"
    assert result["identity_assumption"]["operator_confirmed"] is False


def test_roi_containing_box_is_rejected_without_residual_based_point_selection():
    cloud, frames, width, height, roi = _scene()
    x0, y0, x1, y1 = roi
    mask = np.zeros((height, width), dtype=bool)
    mask[(y0+y1)//2:y1+1, x0:x1+1] = True
    box_ids = np.flatnonzero(mask)
    # Distinct box plane X=.8 in the same image rectangle; both sides of the
    # depth edge remain fixed-support candidates away from the edge itself.
    cloud[box_ids] *= .8/1.1
    frames[:, box_ids] = cloud[box_ids]
    result = _run((cloud, frames, width, height, roi))
    assert result["status"] == "REJECTED" and not result["planes"]
    assert any(r in result["reasons"] for r in (
        "INTERIOR_RMSE_EXCEEDED", "INTERIOR_P95_EXCEEDED", "RAW_ROI_P95_EXCEEDED")), result["reasons"]
    diag = result["diagnostic"]
    assert len(diag["raw_roi_finite_indices"]) == len(diag["raw_roi_indices"])
    assert diag["raw_roi_fit"]["point_count"] == len(diag["raw_roi_indices"])
    assert set(diag["interior_indices"]) & set(box_ids)
    assert diag["interior_fit"]["point_count"] == len(diag["interior_indices"])
    assert diag["residual_trimming_performed"] is False


def test_holes_cannot_use_convex_hull_to_invent_continuous_mesh_area():
    cloud, frames, width, height, roi = _scene(width=120, height=90)
    x0, y0, x1, y1 = roi
    hole = np.zeros((height, width), dtype=bool)
    hole[y0+5:y1-4, x0+5:x1-4] = True
    holes = np.flatnonzero(hole)
    cloud[holes] = np.nan
    frames[:, holes] = np.nan
    result = _run((cloud, frames, width, height, roi))
    assert result["status"] == "REJECTED" and not result["planes"]
    assert "INSUFFICIENT_PROJECTED_INTERNAL_MESH_AREA" in result["reasons"], result["reasons"]
    diag = result["diagnostic"]
    assert diag["projected_mesh_area_m2"] < .08
    assert diag["hull_area_m2"] > diag["projected_mesh_area_m2"]
    assert set(holes).issubset(diag["raw_roi_invalid_indices"])
    assert set(holes).isdisjoint(diag["interior_indices"])
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("frame_case", ["missing", "too_few", "shape", "unstable", "all_missing", "infinite"])
def test_missing_invalid_or_unstable_frames_fail_closed(frame_case):
    cloud, frames, width, height, roi = _scene()
    if frame_case == "missing":
        frames = None
    elif frame_case == "too_few":
        frames = frames[:2]
    elif frame_case == "shape":
        frames = frames[0]
    elif frame_case == "unstable":
        frames[1::2, :, 0] += .12
    elif frame_case == "all_missing":
        frames[:] = np.nan
    else:
        frames[0, 0, 0] = np.inf
    result = _run((cloud, frames, width, height, roi))
    assert result["status"] == "REJECTED" and not result["planes"], result["reasons"]
    assert "FIXED_PIXEL_TEMPORAL_GATES_NOT_PASSED" in result["reasons"]
    assert result["diagnostic"]["interior_fit"] is not None
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("bad_roi", [[-1, 0, 20, 20], [1, 1, 999, 20], [20, 10, 10, 20],
                                      [1, 1, 1, 20], [1., 1., 20., 20.], [1, 2, 3], None])
def test_invalid_or_out_of_bounds_roi_is_explicitly_rejected(bad_roi):
    cloud, frames, width, height, _ = _scene()
    result = controlled_interior_patch(cloud, frames=frames, width=width, height=height,
                                       roi_xyxy=bad_roi, operator_confirmed=True)
    assert result["status"] == "REJECTED" and not result["planes"]
    assert result["reasons"]
    json.dumps(result, allow_nan=False)


def test_patch_policy_reuses_existing_numeric_and_temporal_gates():
    for key in ("minimum_interior_points", "minimum_span_m", "minimum_projected_mesh_area_m2",
                "maximum_interior_rmse_m", "maximum_interior_p95_m", "maximum_raw_candidate_p95_m",
                "raw_candidate_outlier_distance_m", "maximum_raw_candidate_outlier_fraction", "temporal"):
        assert INTERIOR_PATCH_POLICY[key] == CONTROLLED_BOARD_POLICY[key]

