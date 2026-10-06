"""Frozen synthetic clouds and local files only; no ROS, hardware or real ICP."""
import copy
import hashlib
import http.client
import json
from pathlib import Path
import threading

import numpy as np
import pytest

from wc_calibration import alignment_workspace as alignment
from wc_calibration.core import CalibrationError, _hash
from wc_calibration.picker import PickerServer, PickerSession


def prepared(*, holes=False, inconsistent_matches=False):
    right = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.],
                      [0., 0., 1.], [.3, .6, 1.1], [-.4, .8, .5]])
    left = right + [.18, -.21, .04]
    if inconsistent_matches:
        left[0, 0] += .8
    rows = {'left': left.tolist(), 'right': right.tolist()}
    if holes:
        rows['left'].insert(1, [None, 0, 0])
        rows['left'].insert(5, [0, None, 0])
        rows['right'].insert(0, [0, 0, None])
        rows['right'].insert(4, [None, None, None])
    metadata = {side: {'units': 'm', 'coordinate_convention': 'FLU',
        'sensor_id': sid, 'raw_key': ['SYN-SESSION', sid, 'SYN-EPOCH', 17],
        'source_config_hash': ('c' if side == 'left' else 'd') * 64,
        'cloud_data_sha256': ('a' if side == 'left' else 'b') * 64}
        for side, sid in (('left', 'SYN-L'), ('right', 'SYN-R'))}
    return {'schema_version': 1, 'status': 'PREPARED_NOT_VALIDATED',
        'source_mode': 'synthetic', 'units': 'm',
        'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'},
        'sensor_ids': {'left': 'SYN-L', 'right': 'SYN-R'},
        'training': [{'id': 'SYN-ALIGNMENT', **rows, 'selected_raw_frames': metadata,
            'time_quality': 'ARRIVAL_ONLY_NOT_SYNCHRONIZED',
            'static_suggestion': {'static_validated': False}}], 'validation': [],
        'input_preparation': {'kind': 'synthetic_fixture_only'}}


def make_session(tmp_path, data=None):
    data = prepared() if data is None else data
    source = tmp_path / 'prepared.json'
    source.write_text(json.dumps(data, allow_nan=False), encoding='utf8')
    return PickerSession(source, tmp_path / 'manual_pairs')


def request(session, operation='save_manual', transform=None):
    return {'input_hash': session.input_hash, 'scene_id': session.scene()['scene_id'],
        'initial_T_left_right': np.eye(4).tolist() if transform is None else transform,
        'operation': operation}


def fake_result(left, right, transform):
    return {'schema_version': 1, 'kind': 'offline_icp_candidate',
        'status': 'SYNTHETIC_SOLVER_DOUBLE', 'live_eligible': True,
        'initial_T_left_right': np.asarray(transform).tolist(),
        'refined_T_left_right': np.asarray(transform).tolist(),
        'initial_metrics': {'source_points': len(right), 'target_points': len(left)},
        'final_metrics': {'nn_rmse_m': .01}, 'history': [], 'reasons': [],
        'correspondences': [{'left_index': 1, 'right_index': 2,
                             'distance_m': .01, 'verified': False}]}


@pytest.fixture(autouse=True)
def numerical_doubles(monkeypatch):
    # These are workspace/storage tests. Accuracy and actual ICP are tested separately.
    monkeypatch.setattr(alignment, 'refine_preview', fake_result)
    monkeypatch.setattr(alignment, 'evaluate_preview', lambda left, right, transform:
        {'source_points': len(right), 'target_points': len(left), 'nn_rmse_m': .02})


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf8'))


def complete_manual(session):
    return session.export({'input_hash': session.input_hash,
        'scene_id': session.scene()['scene_id'],
        'pairs': [{'left_id': i, 'right_id': i} for i in range(6)]}, solve=True)


def assert_complete(response, session):
    target = Path(response['export_dir'])
    assert target.parent == session.output_root.parent / 'alignment_candidates'
    assert set(p.name for p in target.iterdir()) == {
        'result.json', 'correspondences.json', 'manifest.json'}
    manifest = read_json(target / 'manifest.json')
    result = read_json(target / 'result.json')
    pairs = read_json(target / 'correspondences.json')
    assert result == response['result']
    assert manifest['kind'] == 'offline_alignment_export'
    assert pairs['kind'] == 'unverified_nearest_neighbours'
    for value in (manifest, result, pairs):
        assert value['prepared_input_hash'] == session.input_hash
        assert value['scene_id'] == session.scene()['scene_id']
        assert value['live_eligible'] is False
    assert set(manifest['files']) == {'result.json', 'correspondences.json'}
    for name, meta in manifest['files'].items():
        raw = (target / name).read_bytes()
        assert meta['bytes'] == len(raw)
        assert meta['sha256'] == hashlib.sha256(raw).hexdigest()
    assert result['provenance'] == session._provenance
    assert result['sensor_ids'] == {'left': 'SYN-L', 'right': 'SYN-R'}
    assert result['source_mode'] == 'synthetic' and result['units'] == 'm'
    assert result['independent_validation_performed'] is False
    assert manifest['independent_validation_performed'] is False
    assert 'T_left_right' not in result and 'calibration_id' not in result
    assert 'correspondences' not in result
    assert result['correspondence_count'] == len(pairs['pairs'])
    assert all(pair['verified'] is False for pair in pairs['pairs'])
    assert all(session.token not in p.read_text(encoding='utf8') for p in target.iterdir())
    return manifest, result, pairs


def test_bootstrap_identity_is_explicitly_unvalidated_and_has_only_browser_token(tmp_path):
    session = make_session(tmp_path)
    workspace = alignment.AlignmentWorkspace(session)
    shown = workspace.bootstrap()
    assert set(shown) == {'scene', 'initial_T_left_right', 'initial_source', 'saved_candidate'}
    assert shown['scene'] == session.scene()
    assert shown['scene']['token'] == session.token
    assert shown['initial_source'] == {'kind': 'identity_for_manual_adjustment_only',
        'path': None, 'status': 'UNKNOWN_INSTALLATION'}
    assert shown['initial_T_left_right'] == np.eye(4).tolist()
    assert shown['saved_candidate'] is None
    assert not (session.output_root.parent / 'alignment_candidates').exists()
    shown['scene']['clouds']['left'][0]['xyz'][0] = 99
    assert session.scene()['clouds']['left'][0]['xyz'][0] != 99


def test_prepared_initial_is_frozen_source_bound_and_independent_of_saved_candidate(tmp_path):
    data = prepared()
    initial = np.eye(4)
    initial[1, 3] = -.4575
    data['initial_T_left_right'] = initial.tolist()
    session = make_session(tmp_path, data)
    workspace = alignment.AlignmentWorkspace(session)
    shown = workspace.bootstrap()
    assert shown['initial_T_left_right'] == initial.tolist()
    assert shown['initial_source'] == {'kind': 'PREPARED_INPUT_INITIAL',
        'path': str((tmp_path / 'prepared.json').absolute()),
        'field': 'initial_T_left_right', 'status': 'PREPARED_NOT_VALIDATED'}
    assert shown['saved_candidate'] is None and shown['scene']['live_eligible'] is False
    # Neither returned objects nor a source edit can change an active session.
    shown['initial_T_left_right'][1][3] = 99
    (tmp_path / 'prepared.json').write_text('{}', encoding='utf8')
    assert workspace.bootstrap()['initial_T_left_right'] == initial.tolist()
    changed = initial.copy()
    changed[1, 3] = -.4
    saved = workspace.export(request(session, transform=changed.tolist()))
    reloaded = workspace.bootstrap()
    assert reloaded['saved_candidate'] == saved
    assert reloaded['initial_T_left_right'] == initial.tolist()
    assert reloaded['initial_source']['kind'] == 'PREPARED_INPUT_INITIAL'


def test_matching_manual_initial_remains_preferred_over_prepared_initial(tmp_path):
    data = prepared()
    initial = np.eye(4)
    initial[1, 3] = -.4575
    data['initial_T_left_right'] = initial.tolist()
    session = make_session(tmp_path, data)
    saved = complete_manual(session)
    shown = alignment.AlignmentWorkspace(session).bootstrap()
    assert shown['initial_source']['kind'] == 'saved_manual_initial'
    assert shown['initial_T_left_right'] == saved['result']['initial_T_left_right']
    assert shown['initial_T_left_right'] != initial.tolist()


def test_null_prepared_initial_preserves_legacy_identity_fallback(tmp_path):
    data = prepared()
    data['initial_T_left_right'] = None
    shown = alignment.AlignmentWorkspace(make_session(tmp_path, data)).bootstrap()
    assert shown['initial_T_left_right'] == np.eye(4).tolist()
    assert shown['initial_source']['kind'] == 'identity_for_manual_adjustment_only'


@pytest.mark.parametrize('fault', ['scale', 'reflection', 'bottom_row', 'wrong_shape', 'empty'])
def test_invalid_nonnull_prepared_initial_never_silently_becomes_identity(tmp_path, fault):
    initial = np.eye(4)
    if fault == 'scale': initial[0, 0] = 1.1
    elif fault == 'reflection': initial[2, 2] = -1
    elif fault == 'bottom_row': initial[3, 0] = .1
    elif fault == 'wrong_shape': initial = np.eye(3)
    data = prepared()
    data['initial_T_left_right'] = [] if fault == 'empty' else initial.tolist()
    with pytest.raises((CalibrationError, ValueError)):
        make_session(tmp_path, data)
    assert not (tmp_path / 'manual_pairs').exists()


@pytest.mark.parametrize('inconsistent', [False, True])
def test_bootstrap_identifies_matching_manual_result_even_when_unvalidated(tmp_path, inconsistent):
    session = make_session(tmp_path, prepared(inconsistent_matches=inconsistent))
    saved = complete_manual(session)
    assert saved['result']['status'] == ('UNVALIDATED' if inconsistent else 'CANDIDATE')
    shown = alignment.AlignmentWorkspace(session).bootstrap()
    assert shown['initial_source'] == {'kind': 'saved_manual_initial',
        'path': saved['export_dir'], 'status': saved['result']['status']}
    assert shown['initial_T_left_right'] == saved['result']['initial_T_left_right']
    assert shown['saved_candidate'] is None and shown['scene']['live_eligible'] is False


@pytest.mark.parametrize('operation', ['save_manual', 'refine'])
def test_export_preserves_inputs_existing_candidates_and_default_and_formal_files(tmp_path, operation):
    session = make_session(tmp_path)
    original_hash = session.input_hash
    protected = [tmp_path / 'prepared.json', tmp_path / 'point_picker_input.json',
                 tmp_path / 'formal_calibration.json']
    for path in protected[1:]:
        path.write_text('{"protected": "SYNTHETIC-SENTINEL"}', encoding='utf8')
    before = {path: path.read_bytes() for path in protected}
    workspace = alignment.AlignmentWorkspace(session)
    transform = np.eye(4); transform[:3, 3] = [.2, -.3, .4]
    body = request(session, operation, transform.tolist())
    body_before = copy.deepcopy(body)
    first = workspace.export(body)
    first_files = {p: p.read_bytes() for p in Path(first['export_dir']).iterdir()}
    second = workspace.export(body)
    assert first['export_dir'] != second['export_dir']
    assert body == body_before and session.input_hash == original_hash
    assert {path: path.read_bytes() for path in protected} == before
    assert {path: path.read_bytes() for path in first_files} == first_files
    manifest, result, pairs = assert_complete(second, session)
    assert manifest['operation'] == operation
    assert result['refined_T_left_right'] == transform.tolist()
    if operation == 'save_manual':
        assert result['status'] == 'MANUAL_UNVALIDATED' and pairs['pairs'] == []


def test_refine_uses_frozen_clouds_and_exports_original_ids_across_null_rows(tmp_path, monkeypatch):
    data = prepared(holes=True)
    session = make_session(tmp_path, data)
    original = session.scene()
    (tmp_path / 'prepared.json').write_text('{}', encoding='utf8')
    calls = []
    def observe(left, right, transform):
        calls.append(copy.deepcopy((left, right)))
        assert np.isfinite(left).all() and np.isfinite(right).all()
        return fake_result(left, right, transform)
    monkeypatch.setattr(alignment, 'refine_preview', observe)
    response = alignment.AlignmentWorkspace(session).export(request(session, 'refine'))
    _, _, exported = assert_complete(response, session)
    assert calls == [([point['xyz'] for point in original['clouds']['left']],
                     [point['xyz'] for point in original['clouds']['right']])]
    assert exported['pairs'] == [{'left_id': 2, 'right_id': 3,
                                  'distance_m': .01, 'verified': False}]
    assert session.scene() == original and session.input_hash == _hash(data)
    assert (tmp_path / 'prepared.json').read_text(encoding='utf8') == '{}'


def test_manual_save_evaluates_only_and_does_not_run_icp(tmp_path, monkeypatch):
    session = make_session(tmp_path)
    calls = []
    def forbidden(*args):
        pytest.fail('manual save must not invoke ICP')
    def evaluate(left, right, transform):
        calls.append(np.asarray(transform).tolist())
        return {'nn_rmse_m': .123, 'source_points': len(right), 'target_points': len(left)}
    monkeypatch.setattr(alignment, 'refine_preview', forbidden)
    monkeypatch.setattr(alignment, 'evaluate_preview', evaluate)
    transform = np.eye(4); transform[1, 3] = -.42
    result = alignment.AlignmentWorkspace(session).export(request(session, transform=transform.tolist()))['result']
    assert calls == [transform.tolist()]
    assert result['initial_metrics'] == result['final_metrics']
    assert result['final_metrics']['nn_rmse_m'] == .123
    assert result['refined_T_left_right'] == transform.tolist()


@pytest.mark.parametrize('fault', ['nan', 'inf', 'scale', 'reflection', 'bad_bottom_row',
                                  'wrong_shape', 'excessive_translation'])
def test_invalid_rigid_transform_rejected_before_solver_or_output(tmp_path, monkeypatch, fault):
    session = make_session(tmp_path)
    transform = np.eye(4)
    if fault == 'nan': transform[0, 3] = float('nan')
    elif fault == 'inf': transform[0, 3] = float('inf')
    elif fault == 'scale': transform[0, 0] = 1.01
    elif fault == 'reflection': transform[2, 2] = -1
    elif fault == 'bad_bottom_row': transform[3, 0] = .1
    elif fault == 'wrong_shape': transform = np.eye(3)
    else: transform[0, 3] = 10.001
    def forbidden(*args): pytest.fail('invalid input reached numerical processing')
    monkeypatch.setattr(alignment, 'refine_preview', forbidden)
    monkeypatch.setattr(alignment, 'evaluate_preview', forbidden)
    with pytest.raises((CalibrationError, ValueError)):
        alignment.AlignmentWorkspace(session).export(request(session, transform=transform.tolist()))
    assert not (session.output_root.parent / 'alignment_candidates').exists()


@pytest.mark.parametrize('fault', ['hash', 'scene', 'path', 'coordinates', 'token', 'unknown_operation', 'missing'])
def test_stale_or_arbitrary_request_never_selects_files_or_exports(tmp_path, monkeypatch, fault):
    session = make_session(tmp_path)
    body = request(session)
    if fault == 'hash': body['input_hash'] = 'f' * 64
    elif fault == 'scene': body['scene_id'] = 'OTHER-SCENE'
    elif fault == 'path': body['output_root'] = str(tmp_path / 'outside')
    elif fault == 'coordinates': body['left_xyz'] = [[9, 8, 7]]
    elif fault == 'token': body['token'] = session.token
    elif fault == 'unknown_operation': body['operation'] = 'promote'
    else: body.pop('scene_id')
    monkeypatch.setattr(alignment, 'evaluate_preview', lambda *args: pytest.fail('invalid request was processed'))
    with pytest.raises((CalibrationError, ValueError)):
        alignment.AlignmentWorkspace(session).export(body)
    assert not (tmp_path / 'alignment_candidates').exists()
    assert not (tmp_path / 'outside').exists()


def test_bootstrap_reloads_latest_matching_candidate(tmp_path):
    session = make_session(tmp_path)
    workspace = alignment.AlignmentWorkspace(session)
    first = workspace.export(request(session))
    transform = np.eye(4); transform[0, 3] = .321
    second = workspace.export(request(session, transform=transform.tolist()))
    assert first['export_dir'] != second['export_dir']
    loaded = alignment.AlignmentWorkspace(session).bootstrap()
    assert loaded['saved_candidate'] == second
    assert loaded['saved_candidate']['result']['live_eligible'] is False
    assert loaded['initial_source']['kind'] == 'identity_for_manual_adjustment_only'


@pytest.mark.parametrize('fault', ['missing_manifest', 'broken_manifest', 'wrong_hash',
    'wrong_bytes', 'stale_input', 'stale_scene', 'extra_path', 'promoted', 'changed_result'])
def test_incomplete_corrupt_and_foreign_candidates_are_skipped(tmp_path, fault):
    session = make_session(tmp_path)
    workspace = alignment.AlignmentWorkspace(session)
    valid = workspace.export(request(session))
    bad = workspace.export(request(session))
    directory = Path(bad['export_dir'])
    manifest_path = directory / 'manifest.json'
    manifest = read_json(manifest_path)
    if fault == 'missing_manifest': manifest_path.unlink()
    elif fault == 'broken_manifest': manifest_path.write_text('{', encoding='utf8')
    elif fault == 'changed_result':
        with (directory / 'result.json').open('ab') as stream: stream.write(b' ')
    else:
        if fault == 'wrong_hash': manifest['files']['result.json']['sha256'] = '0' * 64
        elif fault == 'wrong_bytes': manifest['files']['result.json']['bytes'] += 1
        elif fault == 'stale_input': manifest['prepared_input_hash'] = 'f' * 64
        elif fault == 'stale_scene': manifest['scene_id'] = 'OTHER-SCENE'
        elif fault == 'extra_path': manifest['files']['../outside.json'] = {'sha256': '0' * 64, 'bytes': 1}
        else: manifest['live_eligible'] = True
        manifest_path.write_text(json.dumps(manifest), encoding='utf8')
    bad_before = {p.name: p.read_bytes() for p in directory.iterdir()}
    assert alignment.AlignmentWorkspace(session).bootstrap()['saved_candidate'] == valid
    assert {p.name: p.read_bytes() for p in directory.iterdir()} == bad_before


@pytest.mark.parametrize('fault', ['file_list', 'file_metadata_list'])
def test_malformed_manifest_shapes_do_not_crash_candidate_reload(tmp_path, fault):
    session = make_session(tmp_path)
    workspace = alignment.AlignmentWorkspace(session)
    valid = workspace.export(request(session))
    bad = workspace.export(request(session))
    manifest_path = Path(bad['export_dir']) / 'manifest.json'
    manifest = read_json(manifest_path)
    if fault == 'file_list':
        manifest['files'] = ['result.json', 'correspondences.json']
    else:
        manifest['files']['result.json'] = []
    manifest_path.write_text(json.dumps(manifest), encoding='utf8')
    before = manifest_path.read_bytes()
    assert workspace.bootstrap()['saved_candidate'] == valid
    assert manifest_path.read_bytes() == before


def test_malformed_manual_provenance_does_not_crash_or_replace_valid_initial(tmp_path):
    session = make_session(tmp_path)
    valid = complete_manual(session)
    bad = complete_manual(session)
    directory = Path(bad['export_dir'])
    selection = read_json(directory / 'selected_pairs.json')
    result = read_json(directory / 'result.json')
    manifest = read_json(directory / 'manifest.json')
    selection['provenance'] = []
    result['input_hash'] = _hash(selection)
    manifest['result_hash'] = _hash(result)
    # Correct digests deliberately isolate malformed object shape from corruption.
    for name, value in (('selected_pairs.json', selection), ('result.json', result)):
        raw = json.dumps(value, allow_nan=False).encode('utf8')
        (directory / name).write_bytes(raw)
        manifest['files'][name] = {'sha256': hashlib.sha256(raw).hexdigest(), 'bytes': len(raw)}
    (directory / 'manifest.json').write_text(json.dumps(manifest), encoding='utf8')
    before = {p.name: p.read_bytes() for p in directory.iterdir()}
    shown = alignment.AlignmentWorkspace(session).bootstrap()
    assert shown['initial_source']['path'] == valid['export_dir']
    assert shown['initial_T_left_right'] == valid['result']['initial_T_left_right']
    assert {p.name: p.read_bytes() for p in directory.iterdir()} == before


def test_failed_partial_export_is_not_reloaded_and_next_export_recovers(tmp_path, monkeypatch):
    session = make_session(tmp_path)
    workspace = alignment.AlignmentWorkspace(session)
    valid = workspace.export(request(session))
    real_write = workspace._write
    def fail_manifest(directory, name, raw):
        if name == 'manifest.json': raise OSError('synthetic storage failure')
        return real_write(directory, name, raw)
    with monkeypatch.context() as patch:
        patch.setattr(workspace, '_write', fail_manifest)
        with pytest.raises(OSError, match='synthetic storage failure'):
            workspace.export(request(session))
    versions = list((tmp_path / 'alignment_candidates').iterdir())
    assert len(versions) == 2
    incomplete = next(path for path in versions if str(path) != valid['export_dir'])
    assert not (incomplete / 'manifest.json').exists()
    assert alignment.AlignmentWorkspace(session).bootstrap()['saved_candidate'] == valid
    recovered = workspace.export(request(session))
    assert_complete(recovered, session)
    assert workspace.bootstrap()['saved_candidate'] == recovered


def test_concurrent_export_is_rejected_without_extra_version(tmp_path, monkeypatch):
    session = make_session(tmp_path)
    workspace = alignment.AlignmentWorkspace(session)
    entered, release = threading.Event(), threading.Event()
    result, errors = [], []
    def wait_refine(left, right, transform):
        entered.set()
        if not release.wait(timeout=5): raise RuntimeError('test release timeout')
        return fake_result(left, right, transform)
    def run():
        try: result.append(workspace.export(request(session, 'refine')))
        except BaseException as exc: errors.append(exc)
    monkeypatch.setattr(alignment, 'refine_preview', wait_refine)
    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    try:
        assert entered.wait(timeout=3)
        with pytest.raises(alignment.AlignmentBusy):
            workspace.export(request(session))
        assert not (tmp_path / 'alignment_candidates').exists()
    finally:
        release.set()
        worker.join(timeout=5)
    assert not worker.is_alive() and not errors and len(result) == 1
    assert len(list((tmp_path / 'alignment_candidates').iterdir())) == 1
    assert_complete(result[0], session)


@pytest.mark.parametrize('bad_index', [-1, 6, True])
def test_invalid_solver_correspondence_does_not_export(tmp_path, monkeypatch, bad_index):
    session = make_session(tmp_path)
    def bad(left, right, transform):
        result = fake_result(left, right, transform)
        result['correspondences'][0]['left_index'] = bad_index
        return result
    monkeypatch.setattr(alignment, 'refine_preview', bad)
    with pytest.raises((CalibrationError, ValueError)):
        alignment.AlignmentWorkspace(session).export(request(session, 'refine'))
    assert not (tmp_path / 'alignment_candidates').exists()


@pytest.fixture
def server(tmp_path):
    instance = PickerServer(make_session(tmp_path), 0)
    worker = threading.Thread(target=instance.serve_forever,
                              kwargs={'poll_interval': .02}, daemon=True)
    worker.start()
    try:
        yield instance
    finally:
        instance.shutdown()
        instance.server_close()
        worker.join(timeout=3)
        assert not worker.is_alive()


def http_request(server, method, route, body=None, *, token=None):
    connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
    headers = {'Origin': server.origin}
    if body is not None:
        headers.update({'Content-Type': 'application/json',
            'X-Picker-Token': server.session.token if token is None else token})
    try:
        connection.request(method, route,
            body=None if body is None else json.dumps(body), headers=headers)
        response = connection.getresponse()
        return response.status, dict(response.getheaders()), json.loads(response.read())
    finally:
        connection.close()


def test_http_alignment_bootstrap_refine_and_reload_contract(server):
    status, headers, shown = http_request(server, 'GET', '/api/alignment-bootstrap')
    assert status == 200 and headers['Cache-Control'] == 'no-store'
    assert shown['scene']['token'] == server.session.token
    assert shown['saved_candidate'] is None
    status, _, exported = http_request(server, 'POST', '/api/alignment',
                                      request(server.session, 'refine'))
    assert status == 200 and set(exported) == {'result', 'export_dir'}
    assert_complete(exported, server.session)
    status, _, shown = http_request(server, 'GET', '/api/alignment-bootstrap')
    assert status == 200 and shown['saved_candidate'] == exported


def test_http_alignment_rejects_wrong_token_and_extra_path_before_export(server):
    status, _, response = http_request(server, 'POST', '/api/alignment',
        request(server.session), token='wrong-token')
    assert status == 403 and 'export_dir' not in response
    body = request(server.session)
    body['output_root'] = str(server.session.output_root.parent / 'outside')
    status, _, response = http_request(server, 'POST', '/api/alignment', body)
    assert status == 422 and 'export_dir' not in response
    assert not (server.session.output_root.parent / 'alignment_candidates').exists()
    assert not (server.session.output_root.parent / 'outside').exists()


def test_http_alignment_busy_returns_conflict_without_claiming_an_export(server, monkeypatch):
    workspace = server.session.alignment_workspace()
    def busy(body):
        raise alignment.AlignmentBusy('synthetic active alignment')
    monkeypatch.setattr(workspace, 'export', busy)
    status, _, response = http_request(server, 'POST', '/api/alignment', request(server.session))
    assert status == 409 and 'export_dir' not in response
    assert not (server.session.output_root.parent / 'alignment_candidates').exists()
