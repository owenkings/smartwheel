"""Synthetic observations and loopback HTTP only; no ROS or device access."""
import copy
import hashlib
import http.client
import json
from pathlib import Path
import threading
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_calibration import picker
from wc_calibration.core import CalibrationError, _hash


@pytest.fixture
def observations():
    right = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [.3, .6, 1.1], [-.4, .8, .5]])
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', [.07, -.12, .09]).as_matrix()
    transform[:3, 3] = [.18, -.21, .04]
    left = right @ transform[:3, :3].T + transform[:3, 3]
    metadata = {side: {'units': 'm', 'coordinate_convention': 'FLU', 'sensor_id': identity,
        'raw_key': ['synthetic-session', identity, 'epoch', 9], 'source_config_hash': 'a'*64,
        'cloud_data_sha256': 'b'*64} for side, identity in (('left', 'SYN-L'), ('right', 'SYN-R'))}
    scene = {'id': 'SYN-ONE', 'left': left.tolist(), 'right': right.tolist(),
        'selected_raw_frames': metadata, 'time_quality': 'ARRIVAL_ONLY_NOT_SYNCHRONIZED',
        'static_suggestion': {'segments': [], 'static_validated': False}}
    data = {'schema_version': 1, 'status': 'PREPARED_NOT_VALIDATED', 'source_mode': 'synthetic',
        'units': 'm', 'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'},
        'sensor_ids': {'left': 'SYN-L', 'right': 'SYN-R'}, 'training': [scene], 'validation': [],
        'input_preparation': {'kind': 'rosbag2_source_frame', 'bag_uri': 'SYNTHETIC-ONLY'}}
    return data, transform


def make_session(tmp_path, data, **kwargs):
    source = tmp_path/'prepared.json'
    source.write_text(json.dumps(data), encoding='utf-8')
    return picker.PickerSession(source, tmp_path/'exports', **kwargs)


def request_body(session, count=5):
    return {'input_hash': session.input_hash, 'scene_id': session.scene()['scene_id'],
        'pairs': [{'left_id': i, 'right_id': i} for i in range(count)]}


def test_freezes_source_and_original_ids_with_holes(tmp_path, observations):
    data, _ = observations
    data['training'][0]['left'][1] = [None, 0, 0]
    session = make_session(tmp_path, data)
    assert session.input_hash == _hash(data)
    shown = session.scene()
    assert [p['id'] for p in shown['clouds']['left']] == [0, 2, 3, 4]
    assert shown['point_counts']['left']['excluded_nonfinite_rows'] == 1
    shown['clouds']['left'][0]['xyz'][0] = 999
    (tmp_path/'prepared.json').write_text('{}')
    selected = session.selection(request_body(session, 1))
    assert selected['left'][0] == data['training'][0]['left'][0]
    assert selected['provenance']['prepared_input_hash'] == _hash(data)
    assert selected['provenance']['static_suggestion']['segments'] == []
    assert selected['live_eligible'] is False
    bad = request_body(session, 1); bad['pairs'][0]['left_id'] = 1
    with pytest.raises(CalibrationError, match='nonfinite'):
        session.selection(bad)


def test_solve_saves_candidate_and_source_hashes_without_promotion(tmp_path, observations):
    data, transform = observations
    session = make_session(tmp_path, data)
    source_before = (tmp_path/'prepared.json').read_bytes()
    response = session.export(request_body(session), solve=True)
    result, selected = response['result'], response['selection']
    assert result['status'] == 'CANDIDATE' and result['live_eligible'] is False
    assert np.allclose(result['initial_T_left_right'], transform, atol=1e-12)
    assert 'calibration_id' not in result and 'T_left_right' not in result
    assert result['policy']['max_pair_residual_m'] == .05 and result['scale_fitted'] is False
    output = Path(response['export_dir'])
    assert output.parent == session.output_root
    assert set(p.name for p in output.iterdir()) == {'selected_pairs.json', 'result.json', 'manifest.json'}
    manifest = json.loads((output/'manifest.json').read_text())
    assert manifest['prepared_input_hash'] == _hash(data)
    assert manifest['selection_hash'] == result['input_hash'] == _hash(selected)
    assert manifest['result_hash'] == _hash(result)
    for name, meta in manifest['files'].items():
        assert hashlib.sha256((output/name).read_bytes()).hexdigest() == meta['sha256']
    assert all(session.token not in p.read_text() for p in output.iterdir())
    assert (tmp_path/'prepared.json').read_bytes() == source_before
    next_export = session.export(request_body(session), solve=True)
    assert next_export['export_dir'] != response['export_dir']


def test_bad_residual_is_saved_as_unvalidated_not_success(tmp_path, observations):
    data, _ = observations
    data['training'][0]['left'] = (np.asarray(data['training'][0]['left'])*1.4).tolist()
    session = make_session(tmp_path, data)
    response = session.export(request_body(session), solve=True)
    assert response['result']['status'] == 'UNVALIDATED'
    assert response['result']['rejection_reasons'] == ['PAIR_RESIDUAL_LIMIT_EXCEEDED']
    assert response['result']['live_eligible'] is False
    assert json.loads((Path(response['export_dir'])/'manifest.json').read_text())['solver_status'] == 'UNVALIDATED'


def test_export_accepts_one_pair_but_never_solves_or_creates_result(tmp_path, observations):
    session = make_session(tmp_path, observations[0])
    response = session.export(request_body(session, 1))
    assert 'result' not in response
    assert {p.name for p in Path(response['export_dir']).iterdir()} == {'selected_pairs.json'}
    assert response['selection']['status'] == 'SELECTED_NOT_VALIDATED'
    with pytest.raises(CalibrationError, match='3..10000'):
        session.export(request_body(session, 1), solve=True)


@pytest.mark.parametrize('fault', ['hash', 'scene', 'duplicate_left', 'duplicate_right', 'negative',
    'out_of_range', 'bool_id', 'float_id', 'xyz', 'output_path', 'pair_field', 'empty', 'too_many'])
def test_requests_cannot_override_frozen_data(tmp_path, observations, fault):
    session = make_session(tmp_path, observations[0]); body = request_body(session)
    if fault == 'hash': body['input_hash'] = 'wrong'
    elif fault == 'scene': body['scene_id'] = 'wrong'
    elif fault.startswith('duplicate_'): body['pairs'][1][fault.removeprefix('duplicate_')+'_id'] = 0
    elif fault == 'negative': body['pairs'][0]['left_id'] = -1
    elif fault == 'out_of_range': body['pairs'][0]['left_id'] = 20000
    elif fault == 'bool_id': body['pairs'][0]['left_id'] = True
    elif fault == 'float_id': body['pairs'][0]['left_id'] = 0.0
    elif fault == 'xyz': body['left'] = [[0, 0, 0]]
    elif fault == 'output_path': body['output_root'] = str(tmp_path)
    elif fault == 'pair_field': body['pairs'][0]['xyz'] = [0, 0, 0]
    elif fault == 'empty': body['pairs'] = []
    else: body['pairs'] *= 2001
    with pytest.raises(CalibrationError):
        session.export(body, solve=True)
    assert list(session.output_root.iterdir()) == []


@pytest.mark.parametrize('fault', ['collinear', 'near_collinear', 'duplicate_xyz'])
def test_solver_degeneracy_is_not_relaxed(tmp_path, observations, fault):
    data, _ = observations
    points = [[0, 0, 0], [1, 0, 0], [2, .0001 if fault == 'near_collinear' else 0, 0]]
    if fault == 'duplicate_xyz': points[2] = points[0]
    data['training'][0]['left'] = data['training'][0]['right'] = points
    session = make_session(tmp_path, data)
    with pytest.raises(CalibrationError): session.export(request_body(session, 3), solve=True)
    assert list(session.output_root.iterdir()) == []


@pytest.mark.parametrize('fault', ['units', 'axes', 'id', 'raw_id', 'status', 'nan', 'infinity',
    'row_type', 'bool_coordinate', 'point_budget', 'no_finite', 'scene_index'])
def test_prepared_input_guards(tmp_path, observations, fault):
    data, _ = observations
    kwargs = {}
    if fault == 'units': data['units'] = 'mm'
    elif fault == 'axes': data['coordinate_conventions']['right'] = 'RDF'
    elif fault == 'id': data['sensor_ids']['left'] = 'SYN-R'
    elif fault == 'raw_id': data['training'][0]['selected_raw_frames']['left']['raw_key'][1] = 'OTHER'
    elif fault == 'status': data['status'] = 'VALIDATED'
    elif fault in ('nan', 'infinity'): data['training'][0]['left'][0][0] = float('nan' if fault == 'nan' else 'inf')
    elif fault == 'row_type': data['training'][0]['left'][0] = 'not a point'
    elif fault == 'bool_coordinate': data['training'][0]['left'][0][0] = True
    elif fault == 'point_budget': data['training'][0]['left'] *= 4001
    elif fault == 'no_finite': data['training'][0]['left'] = [[None, 0, 0]]
    else: kwargs['scene_index'] = 9
    with pytest.raises((CalibrationError, ValueError)):
        make_session(tmp_path, data, **kwargs)


def test_scene_index_uses_selected_training_only_and_derives_explicit_metadata(tmp_path, observations):
    data, _ = observations
    data.pop('units'); data.pop('coordinate_conventions')
    other = copy.deepcopy(data['training'][0]); other['id'] = 'SYN-TWO'
    other['left'][0] = [.2, .3, .4]; data['training'].append(other)
    session = make_session(tmp_path, data, scene_index=1)
    assert session.scene()['scene_id'] == 'SYN-TWO'
    assert session.selection(request_body(session, 1))['left'][0] == [.2, .3, .4]


@pytest.fixture
def http_server(tmp_path, observations, monkeypatch):
    session = make_session(tmp_path, observations[0])
    assets = tmp_path/'assets'; assets.mkdir()
    for name in ('index.html', 'picker.js', 'picker.css'): (assets/name).write_text('SYNTHETIC FIXTURE '+name)
    monkeypatch.setattr(picker, 'ASSET_ROOT', assets)
    server = picker.PickerServer(session, 0)
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .05}, daemon=True)
    thread.start()
    try: yield server
    finally:
        server.shutdown(); server.server_close(); thread.join(timeout=2)
        assert not thread.is_alive()


def http_request(server, method='GET', route='/api/scene', body=None, headers=None):
    connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
    payload = None if body is None else json.dumps(body)
    try:
        connection.request(method, route, body=payload, headers=headers or {})
        response = connection.getresponse()
        return response.status, dict(response.getheaders()), response.read()
    finally: connection.close()


def post_headers(server):
    return {'X-Picker-Token': server.session.token, 'Origin': server.origin, 'Content-Type': 'application/json'}


def test_http_scene_and_solve_contract(http_server):
    server = http_server
    assert server.server_address[0] == '127.0.0.1'
    status, headers, raw = http_request(server)
    assert status == 200 and json.loads(raw)['token'] == server.session.token
    assert headers['Cache-Control'] == 'no-store' and 'Access-Control-Allow-Origin' not in headers
    status, _, raw = http_request(server, 'POST', '/api/solve', request_body(server.session), post_headers(server))
    response = json.loads(raw)
    assert status == 200 and set(response) == {'selection', 'result', 'export_dir'}
    assert response['result']['status'] == 'CANDIDATE'
    status, _, raw = http_request(server, 'POST', '/api/export', request_body(server.session, 1), post_headers(server))
    assert status == 200 and set(json.loads(raw)) == {'selection', 'export_dir'}


@pytest.mark.parametrize('fault', ['host', 'origin', 'token', 'nonascii_token', 'missing_token', 'cross_site'])
def test_http_post_rejects_foreign_or_unauthenticated_requests(http_server, fault):
    headers = post_headers(http_server)
    if fault == 'host': headers['Host'] = 'attacker.example'
    elif fault == 'origin': headers['Origin'] = 'https://attacker.example'
    elif fault == 'token': headers['X-Picker-Token'] = 'wrong'
    elif fault == 'nonascii_token': headers['X-Picker-Token'] = '\u00e9'
    elif fault == 'missing_token': headers.pop('X-Picker-Token')
    else: headers['Sec-Fetch-Site'] = 'cross-site'
    status, _, _ = http_request(http_server, 'POST', '/api/export', request_body(http_server.session, 1), headers)
    assert status == 403
    assert list(http_server.session.output_root.iterdir()) == []


def test_http_get_rejects_dns_rebinding_and_foreign_origin(http_server):
    for headers in ({'Host': 'attacker.example'}, {'Origin': 'null'}, {'Origin': 'http://127.0.0.1:1'}):
        assert http_request(http_server, headers=headers)[0] == 403


def test_fixed_assets_and_paths_do_not_serve_arbitrary_files(http_server):
    for route in ('/', '/index.html', '/picker.js', '/picker.css'):
        status, headers, raw = http_request(http_server, route=route)
        assert status == 200 and raw.startswith(b'SYNTHETIC FIXTURE')
        assert "frame-ancestors 'none'" in headers['Content-Security-Policy']
    for route in ('/../prepared.json', '/%2e%2e/prepared.json', '/api/scene?file=prepared.json', '/manifest.json'):
        assert http_request(http_server, route=route)[0] == 404


def test_http_errors_are_bounded_and_do_not_export(http_server):
    server = http_server
    status, _, _ = http_request(server, 'POST', '/api/solve', request_body(server.session, 1), post_headers(server))
    assert status == 422
    headers = post_headers(server); headers['Content-Type'] = 'text/plain'
    assert http_request(server, 'POST', '/api/export', request_body(server.session, 1), headers)[0] == 415
    headers = post_headers(server); headers['Content-Length'] = str(picker.MAX_BODY_BYTES+1)
    assert http_request(server, 'POST', '/api/export', {}, headers)[0] == 413
    headers = post_headers(server); headers['Transfer-Encoding'] = 'chunked'
    assert http_request(server, 'POST', '/api/export', {}, headers)[0] == 400
    assert list(server.session.output_root.iterdir()) == []


def test_duplicate_json_keys_and_nonfinite_are_rejected_before_solving(http_server):
    for raw in ('{"pairs": [], "pairs": []}', '{"pairs": NaN}'):
        connection = http.client.HTTPConnection('127.0.0.1', http_server.server_port, timeout=3)
        try:
            connection.request('POST', '/api/solve', body=raw, headers=post_headers(http_server))
            response = connection.getresponse(); response.read()
            assert response.status == 400
        finally: connection.close()


def test_partial_file_failure_has_no_completion_manifest(tmp_path, observations, monkeypatch):
    session = make_session(tmp_path, observations[0])
    original = Path.open
    def reject_result(path, *args, **kwargs):
        if path.name == '.result.json.partial': raise OSError('synthetic disk failure')
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', reject_result)
    with pytest.raises(OSError): session.export(request_body(session), solve=True)
    versions = list(session.output_root.iterdir())
    assert len(versions) == 1 and not (versions[0]/'manifest.json').exists()


def test_output_root_is_rechecked_before_creating_any_version(tmp_path, observations, monkeypatch):
    session = make_session(tmp_path, observations[0])
    def refuse_link(path):
        assert path == session.output_root
        raise CalibrationError('synthetic output link replacement')
    monkeypatch.setattr(picker, '_no_links', refuse_link)
    with pytest.raises(CalibrationError, match='link replacement'):
        session.export(request_body(session), solve=True)
    assert list(session.output_root.iterdir()) == []


def test_incomplete_export_never_exposes_a_truncated_selected_pairs(tmp_path, observations, monkeypatch):
    session = make_session(tmp_path, observations[0])
    monkeypatch.setattr(picker.os, 'fsync', lambda fd: (_ for _ in ()).throw(OSError('synthetic disk failure')))
    with pytest.raises(OSError): session.export(request_body(session, 1))
    version, = session.output_root.iterdir()
    assert not (version/'selected_pairs.json').exists()
    assert not (version/'manifest.json').exists()


def test_http_export_file_failure_returns_failure(http_server, monkeypatch):
    def fail(*args, **kwargs): raise OSError('synthetic output failure')
    monkeypatch.setattr(http_server.session, 'export', fail)
    status, _, raw = http_request(http_server, 'POST', '/api/solve', request_body(http_server.session), post_headers(http_server))
    assert status == 500 and 'export_dir' not in json.loads(raw)


def test_cli_has_bounded_duration_and_no_automatic_browser(tmp_path, observations, monkeypatch, capsys):
    make_session(tmp_path, observations[0])
    browser_calls = []
    monkeypatch.setattr(picker.webbrowser, 'open', lambda *a, **k: browser_calls.append(a))
    started = time.monotonic()
    code = picker.main(['--input', str(tmp_path/'prepared.json'), '--output-root', str(tmp_path/'cli_exports'),
        '--port', '0', '--duration', '1'])
    assert code == 0 and .9 < time.monotonic()-started < 4
    assert browser_calls == []
    output = json.loads(capsys.readouterr().out)
    assert output['url'].startswith('http://127.0.0.1:')
    assert 'token' not in output and output['live_eligible'] is False
    assert picker.main(['--input', str(tmp_path/'prepared.json'), '--output-root', str(tmp_path/'unused'),
        '--duration', '43201']) == 2
