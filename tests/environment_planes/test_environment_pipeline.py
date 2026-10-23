"""Known geometry, degeneracy and independent validation regressions."""
import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from wc_calibration.environment_calibration import analyse_captures, export_report, render_report
from wc_calibration.environment_planes import extract_planes
from wc_calibration.environment_registration import solve_environment
from scene_fixture import make_capture, TRUE_R, TRUE_T


def transformation(result):
    for key in ('transform_right_to_left', 'T_left_right', 'initial_T_left_right'):
        matrix = result.get(key)
        if matrix is not None:
            return np.asarray(matrix)
    raise AssertionError(result)


@pytest.fixture(scope='module')
def analysed():
    train = make_capture()
    holdout = make_capture('synthetic_holdout', 'holdout', 43, wall_distances=(3.2, 1.85, .65))
    return analyse_captures([train, holdout])


def test_complete_three_surface_recovery(analysed):
    result = analysed['result']
    assert result['status'] in ('CANDIDATE', 'OFFLINE_VALIDATED_CANDIDATE'), result
    matrix = transformation(result)
    angle = np.degrees(np.arccos(np.clip((np.trace(matrix[:3, :3] @ TRUE_R.T)-1)/2, -1, 1)))
    assert angle < .25, (angle, result)
    np.testing.assert_allclose(matrix[:3, 3], TRUE_T, atol=.008)
    assert abs(np.linalg.det(matrix[:3, :3])-1) < 1e-8
    assert analysed['verification_level'] == 'SYNTHETIC'
    assert analysed['live_eligible'] is False and result['live_eligible'] is False
    for c in analysed['captures']:
        assert len(c['left']['planes']) >= 3 and len(c['right']['planes']) >= 3
    json.dumps(analysed, allow_nan=False)


def test_independent_holdout_never_changes_fitted_transform(analysed):
    records = copy.deepcopy(analysed['captures'])
    training = solve_environment([records[0]])
    for plane in records[1]['right']['planes']:
        plane['d_m'] += .18
        normal = np.asarray(plane['normal'])
        plane['centroid'] = (-plane['d_m'] * normal).tolist()
    changed = solve_environment(records)
    np.testing.assert_allclose(transformation(changed), transformation(training), atol=1e-10)
    assert changed['status'] != 'OFFLINE_VALIDATED_CANDIDATE', changed


def test_two_directions_cannot_determine_full_translation():
    c = make_capture(directions=2)
    report = analyse_captures([c])
    assert report['result']['status'] == 'REJECTED', report['result']


def test_temporal_data_missing_is_not_validated(analysed):
    records = copy.deepcopy(analysed['captures'])
    for c in records:
        for side in ('left', 'right'):
            for plane in c[side]['planes']:
                plane['temporal'] = {'status': 'NOT_EVALUATED'}
    result = solve_environment(records)
    assert result['status'] != 'OFFLINE_VALIDATED_CANDIDATE', result


def test_capture_reuse_and_swapped_identity_rejected():
    c = make_capture()
    duplicate = copy.deepcopy(c)
    duplicate['role'] = 'holdout'
    with pytest.raises(ValueError, match='身份缺失或重复'):
        analyse_captures([c, duplicate])
    swapped = make_capture('another_capture', 'holdout')
    swapped['sensor_ids'] = {'left': 'SYNTHETIC_RIGHT', 'right': 'SYNTHETIC_LEFT'}
    with pytest.raises(ValueError, match='设备身份'):
        analyse_captures([c, swapped])


def test_coordinate_units_must_be_explicit():
    c = make_capture()
    c['units'] = 'mm'
    with pytest.raises(ValueError, match='FLU'):
        analyse_captures([c])


def test_reflection_cannot_be_used_as_mounting_rotation(analysed):
    initial = np.eye(4)
    initial[1, 1] = -1
    result = solve_environment([analysed['captures'][0]], initial_transform=initial)
    assert result['status'] == 'REJECTED', result


def test_constant_depth_edge_excluded_and_original_indices_retained():
    c = make_capture(width=100, height=50, noise=.003)
    extraction = extract_planes(c['clouds']['left'], width=100, height=50, frames=c['frames']['left'])
    assert len(extraction['planes']) >= 3
    ids = [index for p in extraction['planes'] for index in p['indices']]
    assert len(ids) == len(set(ids))
    assert all(0 <= index < 5000 for index in ids)
    assert len(ids) < 5000
    for p in extraction['planes']:
        normal = np.asarray(p['normal'])
        assert abs(np.linalg.norm(normal)-1) < 1e-8
        assert p['d_m'] > 0
        assert abs(normal @ np.asarray(p['centroid']) + p['d_m']) < 1e-6


def test_export_safe_json_and_html_no_overwrite(analysed, tmp_path):
    report = copy.deepcopy(analysed)
    report['captures'][0]['capture_id'] = '</script><script>alert(1)</script>'
    document = render_report(report)
    assert '</script><script>alert(1)' not in document
    assert 'connect-src' in document and 'SYNTHETIC' in document
    receipt = export_report(report, tmp_path/'report')
    assert receipt['live_eligible'] is False
    stored = json.loads((tmp_path/'report/environment_calibration.json').read_text(encoding='utf-8'))
    assert stored['captures'][0]['capture_id'] == report['captures'][0]['capture_id']
    with pytest.raises(FileExistsError):
        export_report(report, tmp_path/'report')


def test_wall_floor_boundary_retains_the_second_physical_surface():
    # Independent ray intersections: a wall and a real floor with enough area.
    # The old greedy extraction lost their shared boundary support and reduced
    # the floor's short span below 30cm, despite adequate original observations.
    width, height = 160, 60
    yaw, pitch = np.meshgrid(np.deg2rad(np.linspace(-15., 15., width)),
                            np.deg2rad(np.linspace(22., -22., height)))
    rays = np.stack([np.cos(pitch)*np.cos(yaw), np.cos(pitch)*np.sin(yaw),
                     np.sin(pitch)], axis=-1).reshape(-1, 3)
    wall_ranges = 2.06 / rays[:, 0]
    floor_ranges = np.full(width*height, np.inf)
    downward = rays[:, 2] < 0
    floor_ranges[downward] = -.65 / rays[downward, 2]
    cloud = rays * np.minimum(wall_ranges, floor_ranges)[:, None]
    result = extract_planes(cloud, width=width, height=height, seed=0)
    floors = [p for p in result['planes'] if np.dot(p['normal'], [0, 0, 1]) > .999]
    assert len(floors) == 1, result
    floor = floors[0]
    assert floor['quality']['status'] == 'PASS'
    assert abs(floor['d_m']-.65) < 1e-6
    assert floor['span_m'][1] >= .30
    assert floor['temporal']['status'] == 'NOT_EVALUATED'
    ids = [i for p in result['planes'] for i in p['indices']]
    assert len(ids) == len(set(ids))
