"""Unsaved calculations must not create or rewrite evidence or enable calibration."""
import copy
import hashlib
import http.client
import json
import threading

import numpy as np
import pytest

from wc_calibration.assessment import assess_result
from wc_calibration import alignment_workspace
from wc_calibration.picker import PickerSession, PickerServer
from wc_calibration.core import CalibrationError


def snapshot(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            if p.is_file() else 'directory' for p in root.rglob('*')}


@pytest.fixture
def session(tmp_path):
    right = np.array([[1, 0, 0], [2, 0, 0], [1, 1, 0], [1, 0, 1]], dtype=float)
    data = {'schema_version': 1, 'status': 'PREPARED_NOT_VALIDATED', 'source_mode': 'synthetic',
            'units': 'm', 'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'},
            'sensor_ids': {'left': 'SYN-L', 'right': 'SYN-R'},
            'training': [{'id': 'SYN-PREVIEW', 'left': (right + [.2, -.3, .1]).tolist(),
                          'right': right.tolist(), 'input_files': {s: {'units': 'm',
                          'coordinate_convention': 'FLU'} for s in ('left', 'right')}}]}
    source = tmp_path/'prepared.json'
    source.write_text(json.dumps(data), encoding='utf-8')
    return PickerSession(source, tmp_path/'manual_points')


def manual_body(session):
    return {'input_hash': session.input_hash, 'scene_id': session.scene()['scene_id'],
            'pairs': [{'left_id': i, 'right_id': i} for i in range(4)]}


def alignment_body(session):
    return {'input_hash': session.input_hash, 'scene_id': session.scene()['scene_id'],
            'initial_T_left_right': np.eye(4).tolist(), 'operation': 'refine'}


def test_manual_preview_is_same_solver_with_no_files_or_promotion(session, tmp_path):
    before = snapshot(tmp_path)
    response = session.calculate(manual_body(session))
    assert snapshot(tmp_path) == before
    assert response['export_dir'] is None and response['saved'] is False
    assert response['result']['live_eligible'] is False
    assert response['assessment']['level'] == 'EXPLORATORY'
    assert response['assessment']['formal_use'] is False
    assert np.allclose(np.array(response['result']['initial_T_left_right'])[:3, 3], [.2, -.3, .1])
    saved = session.export(manual_body(session), solve=True)
    assert saved['result'] == response['result'] and saved['assessment'] == response['assessment']
    assert saved['saved'] is True and saved['export_dir']
    before = snapshot(tmp_path)
    bootstrap = session.alignment_workspace().bootstrap()
    assert bootstrap['initial_assessment'] == response['assessment']
    assert snapshot(tmp_path) == before  # old records gain display assessments, never rewrites


def test_bad_pairs_are_rejected_without_storage(session, tmp_path):
    body = manual_body(session)
    body['pairs'][2]['right_id'], body['pairs'][3]['right_id'] = 3, 2
    before = snapshot(tmp_path)
    response = session.calculate(body)
    assert response['assessment']['level'] == 'REJECTED'
    assert response['result']['rejection_reasons'] == ['PAIR_RESIDUAL_LIMIT_EXCEEDED']
    assert snapshot(tmp_path) == before
    body['pairs'][1]['left_id'] = 0
    with pytest.raises(CalibrationError):
        session.calculate(body)
    assert snapshot(tmp_path) == before


def fake_icp(left, right, initial):
    return {'kind': 'offline_icp_candidate', 'status': 'CANDIDATE',
            'initial_T_left_right': initial.tolist(), 'refined_T_left_right': initial.tolist(),
            'final_metrics': {'forward_overlap_fraction': .75, 'reverse_overlap_fraction': .74,
                              'nn_rmse_m': .055, 'nn_p95_m': .105},
            'transform_change': {'translation_m': .39, 'rotation_deg': 7.8}, 'reasons': [],
            'correspondences': [{'left_index': 0, 'right_index': 0, 'distance_m': .1}]}


def test_icp_preview_and_save_have_equal_results_without_hidden_export(session, tmp_path, monkeypatch):
    monkeypatch.setattr(alignment_workspace, 'refine_preview', fake_icp)
    workspace = session.alignment_workspace()
    before = snapshot(tmp_path)
    preview = workspace.calculate(alignment_body(session))
    assert preview['saved'] is False and preview['export_dir'] is None
    assert snapshot(tmp_path) == before
    assert preview['result']['live_eligible'] is False
    assert preview['assessment']['formal_use'] is False
    assert any('5.50 cm' in line for line in preview['assessment']['details'])
    saved = workspace.export(alignment_body(session))
    assert saved['result'] == preview['result'] and saved['assessment'] == preview['assessment']
    before = snapshot(tmp_path)
    assert workspace.bootstrap()['saved_candidate'] == saved
    assert snapshot(tmp_path) == before
    with workspace._lock, pytest.raises(alignment_workspace.AlignmentBusy):
        workspace.calculate(alignment_body(session))


@pytest.mark.parametrize('route', ['/api/calculate', '/api/alignment-preview'])
def test_preview_http_authentication_and_no_writes(session, tmp_path, monkeypatch, route):
    monkeypatch.setattr(alignment_workspace, 'refine_preview', fake_icp)
    server = PickerServer(session, 0)
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    thread.start()
    before = snapshot(tmp_path)
    try:
        body = manual_body(session) if route.endswith('/calculate') else alignment_body(session)
        for token, expected in [('wrong', 403), (session.token, 200)]:
            connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
            try:
                connection.request('POST', route, json.dumps(body),
                    {'Content-Type': 'application/json', 'X-Picker-Token': token, 'Origin': server.origin})
                response = connection.getresponse()
                payload = json.loads(response.read())
                assert response.status == expected
                if expected == 200:
                    assert payload['saved'] is False and payload['export_dir'] is None
                    assert payload['assessment']['formal_use'] is False
            finally:
                connection.close()
        assert snapshot(tmp_path) == before
    finally:
        server.shutdown(); server.server_close(); thread.join(2)


def test_assessment_does_not_promote_even_perfect_fit_or_change_result():
    result = fake_icp([], [], np.eye(4))
    original = copy.deepcopy(result)
    result['final_metrics']['nn_rmse_m'] = 0.
    assert assess_result(result)['formal_use'] is False
    result['final_metrics']['nn_rmse_m'] = .055
    assert result == original
    result['status'] = 'REJECTED'
    result['reasons'] = ['DEGENERATE_GEOMETRY']
    assessment = assess_result(result)
    assert assessment['level'] == 'REJECTED' and assessment['formal_use'] is False
    assert any('DEGENERATE_GEOMETRY' in line for line in assessment['details'])
