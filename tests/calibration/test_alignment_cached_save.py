"""Save the exact reviewed calculation, without running ICP a second time."""
import copy
import hashlib
import http.client
import json
from pathlib import Path
import threading

import numpy as np
import pytest

from wc_calibration import alignment_workspace as alignment
from wc_calibration.core import CalibrationError
from wc_calibration.picker import PickerServer, PickerSession


def snapshot(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            if p.is_file() else 'directory' for p in root.rglob('*')}


@pytest.fixture
def session(tmp_path):
    right = [[1., 0., 0.], [2., 0., 0.], [1., 1., 0.], [1., 0., 1.]]
    data = {'schema_version': 1, 'status': 'PREPARED_NOT_VALIDATED',
            'source_mode': 'synthetic', 'units': 'm',
            'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'},
            'sensor_ids': {'left': 'CACHE-L', 'right': 'CACHE-R'},
            'training': [{'id': 'SYN-CACHED-SAVE', 'left': (np.array(right) + [.2, -.3, .1]).tolist(),
                          'right': right, 'input_files': {s: {'units': 'm', 'coordinate_convention': 'FLU'}
                                                        for s in ('left', 'right')}}]}
    source = tmp_path / 'prepared.json'
    source.write_text(json.dumps(data), encoding='utf-8')
    return PickerSession(source, tmp_path / 'manual_points')


@pytest.fixture
def solver_calls(monkeypatch):
    calls = []
    def solve(left, right, initial):
        calls.append(np.asarray(initial).tolist())
        refined = np.array(initial, copy=True)
        refined[0, 3] += .01234567890123
        return {'schema_version': 1, 'kind': 'offline_icp_candidate', 'status': 'CANDIDATE',
                'initial_T_left_right': np.asarray(initial).tolist(),
                'refined_T_left_right': refined.tolist(),
                'initial_metrics': {'nn_rmse_m': .04},
                'final_metrics': {'forward_overlap_fraction': .8, 'reverse_overlap_fraction': .79,
                                  'nn_rmse_m': .012},
                'transform_change': {'translation_m': .01234567890123, 'rotation_deg': 0.},
                'policy': {'role': 'test_double'}, 'history': [], 'reasons': [],
                'correspondences': [{'left_index': 1, 'right_index': 2, 'distance_m': .012}]}
    monkeypatch.setattr(alignment, 'refine_preview', solve)
    return calls


def calculate_body(session, dx=0.):
    T = np.eye(4)
    T[0, 3] = dx
    return {'input_hash': session.input_hash, 'scene_id': session.scene()['scene_id'],
            'initial_T_left_right': T.tolist(), 'operation': 'refine'}


def save_body(session, preview):
    return {'input_hash': session.input_hash, 'scene_id': session.scene()['scene_id'],
            'calculation_id': preview['calculation_id']}


def assert_manifest(saved):
    directory = Path(saved['export_dir'])
    manifest = json.loads((directory / 'manifest.json').read_text(encoding='utf-8'))
    for name, record in manifest['files'].items():
        raw = (directory / name).read_bytes()
        assert len(raw) == record['bytes']
        assert hashlib.sha256(raw).hexdigest() == record['sha256']
    assert json.loads((directory / 'result.json').read_text(encoding='utf-8')) == saved['result']
    return directory, manifest


def test_cache_is_memory_only_and_saving_does_not_refine_again(session, tmp_path, solver_calls):
    workspace = session.alignment_workspace()
    before = snapshot(tmp_path)
    preview = workspace.calculate(calculate_body(session))
    assert snapshot(tmp_path) == before
    assert isinstance(preview['calculation_id'], str) and preview['calculation_id']
    assert preview['saved'] is False and preview['export_dir'] is None
    assert preview['result']['live_eligible'] is False
    expected = copy.deepcopy(preview['result'])
    saved = workspace.save_result(save_body(session, preview))
    assert len(solver_calls) == 1
    assert saved['saved'] is True and saved['result'] == expected
    assert saved['assessment'] == preview['assessment']
    directory, manifest = assert_manifest(saved)
    pairs = json.loads((directory / 'correspondences.json').read_text(encoding='utf-8'))
    assert pairs['pairs'] == [{'left_id': 1, 'right_id': 2, 'distance_m': .012, 'verified': False}]
    assert manifest['scene_id'] == session.scene()['scene_id']
    assert manifest['live_eligible'] is False and manifest['independent_validation_performed'] is False


def test_returned_preview_object_cannot_change_the_server_cached_result(session, solver_calls):
    workspace = session.alignment_workspace()
    preview = workspace.calculate(calculate_body(session))
    expected = copy.deepcopy(preview['result'])
    body = save_body(session, preview)
    preview['result']['refined_T_left_right'][0][3] = 9.
    preview['result']['final_metrics']['nn_rmse_m'] = 0.
    preview['assessment']['formal_use'] = True
    saved = workspace.save_result(body)
    assert saved['result'] == expected
    assert saved['assessment']['formal_use'] is False
    assert len(solver_calls) == 1


def test_repeated_save_of_same_id_returns_same_artifact_without_rewrites(session, tmp_path, solver_calls):
    workspace = session.alignment_workspace()
    preview = workspace.calculate(calculate_body(session))
    first = workspace.save_result(save_body(session, preview))
    before = snapshot(tmp_path)
    second = workspace.save_result(save_body(session, preview))
    assert first['export_dir'] == second['export_dir']
    assert first['result'] == second['result']
    assert snapshot(tmp_path) == before and len(solver_calls) == 1
    second['result']['refined_T_left_right'][0][3] = 9.
    third = workspace.save_result(save_body(session, preview))
    assert third['result'] == first['result']


@pytest.mark.parametrize('fault', ['unknown_id', 'wrong_input', 'wrong_scene', 'client_result',
                                 'client_matrix', 'output_path', 'missing_id', 'null_id'])
def test_untrusted_save_request_never_writes_or_runs_solver(session, tmp_path, solver_calls, fault):
    workspace = session.alignment_workspace()
    preview = workspace.calculate(calculate_body(session))
    body = save_body(session, preview)
    if fault == 'unknown_id': body['calculation_id'] = 'f' * 32
    elif fault == 'wrong_input': body['input_hash'] = '0' * 64
    elif fault == 'wrong_scene': body['scene_id'] = 'OTHER-SCENE'
    elif fault == 'client_result': body['result'] = preview['result']
    elif fault == 'client_matrix': body['initial_T_left_right'] = np.eye(4).tolist()
    elif fault == 'output_path': body['output_root'] = str(tmp_path / 'outside')
    elif fault == 'missing_id': body.pop('calculation_id')
    else: body['calculation_id'] = None
    before = snapshot(tmp_path)
    with pytest.raises((CalibrationError, ValueError)):
        workspace.save_result(body)
    assert snapshot(tmp_path) == before and len(solver_calls) == 1


def test_new_calculation_expires_old_calculation_id(session, tmp_path, solver_calls):
    workspace = session.alignment_workspace()
    first = workspace.calculate(calculate_body(session))
    second = workspace.calculate(calculate_body(session, dx=.1))
    assert first['calculation_id'] != second['calculation_id']
    before = snapshot(tmp_path)
    with pytest.raises((CalibrationError, ValueError)):
        workspace.save_result(save_body(session, first))
    assert snapshot(tmp_path) == before
    saved = workspace.save_result(save_body(session, second))
    assert saved['result'] == second['result']
    assert len(solver_calls) == 2


def test_failed_new_solver_does_not_leave_previous_result_saveable(session, tmp_path, solver_calls, monkeypatch):
    workspace = session.alignment_workspace()
    first = workspace.calculate(calculate_body(session))
    def broken(*args):
        raise RuntimeError('injected solver failure')
    monkeypatch.setattr(alignment, 'refine_preview', broken)
    before = snapshot(tmp_path)
    with pytest.raises(RuntimeError, match='injected solver failure'):
        workspace.calculate(calculate_body(session, dx=.1))
    with pytest.raises((CalibrationError, ValueError)):
        workspace.save_result(save_body(session, first))
    assert snapshot(tmp_path) == before


def test_cache_is_not_restored_by_a_new_workspace_for_the_same_scene(session, tmp_path, solver_calls):
    first = alignment.AlignmentWorkspace(session)
    preview = first.calculate(calculate_body(session))
    replacement = alignment.AlignmentWorkspace(session)
    before = snapshot(tmp_path)
    with pytest.raises((CalibrationError, ValueError)):
        replacement.save_result(save_body(session, preview))
    assert snapshot(tmp_path) == before and len(solver_calls) == 1


def test_save_respects_alignment_busy_without_partial_export(session, tmp_path, solver_calls):
    workspace = session.alignment_workspace()
    preview = workspace.calculate(calculate_body(session))
    before = snapshot(tmp_path)
    with workspace._lock, pytest.raises(alignment.AlignmentBusy):
        workspace.save_result(save_body(session, preview))
    assert snapshot(tmp_path) == before and len(solver_calls) == 1


def test_http_cached_save_requires_token_and_preserves_calculation(session, tmp_path, solver_calls):
    workspace = session.alignment_workspace()
    preview = workspace.calculate(calculate_body(session))
    server = PickerServer(session, 0)
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    thread.start()
    try:
        before = snapshot(tmp_path)
        for token, expected_status in [('wrong', 403), (session.token, 200)]:
            connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=5)
            try:
                connection.request('POST', '/api/alignment-save-result', json.dumps(save_body(session, preview)),
                                   {'Content-Type': 'application/json', 'X-Picker-Token': token, 'Origin': server.origin})
                response = connection.getresponse()
                payload = json.loads(response.read())
                assert response.status == expected_status
                if expected_status == 403:
                    assert snapshot(tmp_path) == before
                    assert 'export_dir' not in payload
                else:
                    assert payload['saved'] is True
                    assert payload['result'] == preview['result']
                    assert_manifest(payload)
            finally:
                connection.close()
        assert len(solver_calls) == 1
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2)
