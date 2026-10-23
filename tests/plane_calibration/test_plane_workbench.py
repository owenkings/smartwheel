"""Source isolation, persistence and HTTP boundaries of the plane workbench."""
import copy
import http.client
import json
from pathlib import Path
import threading

import numpy as np
import pytest

from wc_calibration.plane_workbench import PlaneServer, PlaneSession, roi_indices


def fake_capture(dataset):
    y, z = np.meshgrid(np.linspace(-.4, .4, 12), np.linspace(-.3, .3, 10))
    cloud = np.column_stack([np.full(y.size, 1.5), y.ravel(), z.ravel()])
    return {'capture_id': Path(dataset).name, 'source_mode': 'synthetic',
            'sensor_ids': {'left': 'synthetic_left', 'right': 'synthetic_right'},
            'width': 12, 'height': 10,
            'clouds': {s: cloud.tolist() for s in ('left', 'right')},
            'amplitude': {s: [100]*120 for s in ('left', 'right')},
            'frames': {s: np.stack([cloud]*5) for s in ('left', 'right')},
            'provenance': {'dataset': str(dataset), 'synthetic': True}}


@pytest.fixture
def session(tmp_path):
    root = tmp_path/'code'
    root.mkdir()
    item = PlaneSession(root, loader=lambda root, path: fake_capture(path), source_check=lambda c: None)
    yield item
    item.close()


def submit(session, action, **body):
    return session.mutate(action, {'revision': session.revision, **body})


def add(session, name='poseA', role='train'):
    return submit(session, 'import', dataset=str(Path(session.project_root)/'data/experiments'/name),
                  label=name, role=role)['state']['poses'][-1]


def test_auto_save_roi_clear_and_resume(session):
    p = add(session)
    for side in ('left', 'right'):
        submit(session, 'roi', pose_id=p['id'], side=side, roi=[0, 0, 9, 11])
    submit(session, 'fit', pose_id=p['id'], same_face_confirmed=True)
    assert session.poses[0]['fit']['left']['status'] == 'PASS'
    workspace = session.output_root/'workspace.json'
    assert json.loads(workspace.read_text())['poses'][0]['same_face_confirmed'] is True
    session.close()
    restored = PlaneSession(session.project_root, resume=workspace,
                            loader=lambda root, path: fake_capture(path), source_check=lambda c: None)
    try:
        assert restored.poses[0]['rois']['left'] == [0, 0, 9, 11]
        assert restored.poses[0]['fit'] is None
        submit(restored, 'fit', pose_id=p['id'], same_face_confirmed=True)
        assert restored.poses[0]['fit']['left']['status'] == 'PASS'
        submit(restored, 'roi', pose_id=p['id'], side='left', roi=None)
        assert restored.poses[0]['fit'] is None
        assert restored.poses[0]['same_face_confirmed'] is False
    finally:
        restored.close()


def test_duplicate_capture_and_stale_revision_rejected(session):
    add(session)
    with pytest.raises(ValueError, match='同一录制'):
        add(session, role='holdout')
    revision = session.revision
    with pytest.raises(ValueError, match='版本已过期'):
        session.mutate('solve', {'revision': revision-1})
    assert session.revision == revision


def test_changed_source_prevents_fit(session):
    p = add(session)
    for side in ('left', 'right'):
        submit(session, 'roi', pose_id=p['id'], side=side, roi=[0, 0, 9, 11])
    def changed(c):
        raise ValueError('bound source changed')
    session._source_check = changed
    with pytest.raises(ValueError, match='bound source changed'):
        submit(session, 'fit', pose_id=p['id'], same_face_confirmed=True)
    assert session.poses[0]['fit'] is None


def test_save_failure_rolls_back_state(session, monkeypatch):
    add(session)
    before = session.state()
    def fail():
        raise OSError('USB unavailable')
    monkeypatch.setattr(session, '_persist', fail)
    with pytest.raises(OSError, match='USB unavailable'):
        submit(session, 'role', pose_id=session.poses[0]['id'], role='holdout')
    assert session.state() == before


def test_roi_bounds_and_mirrored_raw_indices():
    assert roi_indices([1, 2, 2, 3], 3, 5).tolist() == [7, 8, 12, 13]
    for value in ([0, 0, 60, 159], [-1, 0, 2, 3], [0, 2, 1, 1], [False, 0, 1, 1]):
        with pytest.raises(ValueError):
            roi_indices(value, 60, 160)


def test_resume_rejects_modified_aggregation(session):
    add(session)
    workspace = session.output_root/'workspace.json'
    session.close()
    def altered(root, path):
        capture = fake_capture(path)
        capture['clouds']['right'][0][0] += .1
        return capture
    with pytest.raises(ValueError, match='聚合数据发生改变'):
        PlaneSession(session.project_root, resume=workspace, loader=altered, source_check=lambda c: None)


def test_resume_rejects_changed_source_with_same_aggregation(session):
    add(session)
    workspace = session.output_root/'workspace.json'
    session.close()
    def altered(root, path):
        capture = fake_capture(path)
        capture['provenance']['raw_file_sha256'] = 'changed source bytes'
        return capture
    with pytest.raises(ValueError, match='绑定的原始文件'):
        PlaneSession(session.project_root, resume=workspace, loader=altered, source_check=lambda c: None)


def test_full_multi_pose_flow_uses_raw_cloud_and_keeps_export_offline(session):
    normals = np.array([[-1, 0, 0], [-.8, .6, 0], [-.8, 0, .6]], float)
    transform = np.eye(4)
    transform[:3, 3] = [.03, -.43, .02]
    y, z = np.meshgrid(np.linspace(-.4, .4, 12), np.linspace(-.3, .3, 10))
    def loader(root, path):
        capture = fake_capture(path)
        index = int(Path(path).name[1:])
        normal = normals[index % 3]
        axis1 = np.cross(normal, [0, 0, 1]); axis1 /= np.linalg.norm(axis1)
        axis2 = np.cross(normal, axis1)
        cloud = -1.5*normal + y.ravel()[:, None]*axis1 + z.ravel()[:, None]*axis2
        values = {'left': cloud, 'right': cloud-transform[:3, 3]}
        capture['clouds'] = {s: a.tolist() for s, a in values.items()}
        capture['frames'] = {s: np.stack([a]*5) for s, a in values.items()}
        return capture
    session._loader = loader
    for i in range(6):
        pose = add(session, 'p'+str(i), role='train' if i < 3 else 'holdout')
        for side in ('left', 'right'):
            submit(session, 'roi', pose_id=pose['id'], side=side, roi=[0, 0, 9, 11])
        submit(session, 'fit', pose_id=pose['id'], same_face_confirmed=True)
    submit(session, 'solve')
    assert session.result['status'] == 'CANDIDATE', session.result
    np.testing.assert_allclose(session.result['initial_T_left_right'], transform, atol=1e-10)
    assert session.result['independent_validation_performed'] is True
    submit(session, 'icp')
    assert session.icp['status'] == 'CANDIDATE', session.icp
    assert session.icp['independent_validation_passed'] is True
    saved = submit(session, 'save')['saved_path']
    candidate = json.loads((Path(saved)/'plane_calibration.json').read_text())
    assert candidate['live_eligible'] is False
    assert candidate['formal_configuration_modified'] is False
    assert len(candidate['sources']) == 6
    assert (Path(saved)/'complete.json').is_file()


def test_independent_loopback_http_and_token(session):
    with PlaneServer(session, 0) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        def request(path, body=None, host=None, origin=None, token=None):
            connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=10)
            headers = {'Host': host or '127.0.0.1:'+str(server.server_port)}
            if origin is not None:
                headers['Origin'] = origin
            if token is not None:
                headers['X-Picker-Token'] = token
            if body is not None:
                headers['Content-Type'] = 'application/json'
            connection.request('POST' if body is not None else 'GET', path,
                               json.dumps(body) if body is not None else None, headers)
            response = connection.getresponse()
            status, data = response.status, response.read()
            connection.close()
            return status, data
        try:
            assert server.server_address[0] == '127.0.0.1'
            status, raw = request('/api/state')
            assert status == 200 and json.loads(raw)['offline_only'] is True
            assert request('/api/state', host='evil.example')[0] == 403
            assert request('/api/state', origin='https://evil.example')[0] == 403
            body = {'revision': session.revision, 'token': session.token}
            assert request('/api/solve', body=body)[0] == 403
            assert request('/api/solve', body=body, token=session.token)[0] == 422
            assert request('/api/capture', body=body, token=session.token)[0] == 404
            assert request('/../config/storage.local.json')[0] == 404
        finally:
            server.shutdown()
            thread.join(timeout=5)


@pytest.mark.skipif(__import__('os').name != 'posix', reason='POSIX workspace lock')
def test_same_workspace_cannot_have_two_process_owners(session):
    with pytest.raises(ValueError, match='已由另一个服务打开'):
        PlaneSession(session.project_root, resume=session.output_root/'workspace.json')
