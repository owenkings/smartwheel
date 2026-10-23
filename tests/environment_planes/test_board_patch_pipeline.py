"""Independent synthetic rays exercise interior-region input and joint solving."""
import json
import numpy as np
import pytest

from test_small_board_pipeline import _analytic_rotation, _sequence
from wc_calibration.small_board_calibration import analyse_board_captures


def _contained_rectangle(mask):
    best, bounds = 0, None
    for top in range(mask.shape[0]):
        columns = np.ones(mask.shape[1], dtype=bool)
        for bottom in range(top, mask.shape[0]):
            columns &= mask[bottom]
            edges = np.diff(np.r_[False, columns, False].astype(int))
            for start, stop in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
                area = (stop-start)*(bottom-top+1)
                if area > best:
                    best, bounds = area, [int(start),top,int(stop-1),bottom]
    assert bounds is not None
    x0,y0,x1,y1 = bounds
    assert mask[y0:y1+1,x0:x1+1].all()
    return bounds


def _captures_and_regions():
    rotation = _analytic_rotation([1.3, -1.7, 2.2])
    translation = np.array([.025, -.450, .012])
    captures = _sequence(rotation, translation)
    regions = {}
    for capture in captures:
        regions[capture['capture_id']] = {}
        for side in ('left', 'right'):
            points = np.asarray(capture['clouds'][side])
            # Analytic scene has only the declared finite board below X=2 m.
            # A tilted board projects to a quadrilateral. Its bounding-box
            # corners may contain background even after a fixed inset.
            mask = (points[:, 0] < 2.).reshape(60,160)
            regions[capture['capture_id']][side] = _contained_rectangle(mask)
    return captures, regions, rotation, translation


def test_confirmed_interior_regions_recover_synthetic_relative_transform():
    captures, regions, rotation, translation = _captures_and_regions()
    report = analyse_board_captures(captures, same_board_face_confirmed=True,
        interior_rois=regions, interior_regions_confirmed=True)
    result = report['result']
    assert result['status'] == 'CANDIDATE', result
    assert result['validation']['holdout_passed'] is True
    assert result['verification_level'] == 'SYNTHETIC'
    matrix = np.asarray(result['transform_right_to_left'])
    error = np.degrees(np.arccos(np.clip((np.trace(matrix[:3,:3] @ rotation.T)-1)/2,-1,1)))
    assert error < .15
    np.testing.assert_allclose(matrix[:3,3], translation, atol=.003)
    assert result['live_eligible'] is False
    json.dumps(report, allow_nan=False)


def test_unconfirmed_regions_never_produce_joint_candidate():
    captures, regions, _, _ = _captures_and_regions()
    report = analyse_board_captures(captures, same_board_face_confirmed=True,
        interior_rois=regions, interior_regions_confirmed=False)
    assert report['result']['status'] == 'REJECTED'
    assert report['result']['transform_right_to_left'] is None
    for capture in report['captures']:
        assert not capture['left']['planes'] and not capture['right']['planes']


def test_partial_or_unknown_capture_region_configuration_is_rejected():
    captures, regions, _, _ = _captures_and_regions()
    with pytest.raises(ValueError, match='require explicit'):
        analyse_board_captures(captures, same_board_face_confirmed=True,
            interior_regions_confirmed=True)
    regions.pop(captures[0]['capture_id'])
    with pytest.raises(ValueError, match='every and only'):
        analyse_board_captures(captures, same_board_face_confirmed=True,
            interior_rois=regions, interior_regions_confirmed=True)
