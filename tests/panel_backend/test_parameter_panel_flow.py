"""Profile/capture coordination and protection of recordings being prepared."""
from pathlib import Path
import threading
from unittest import mock

import pytest

from wc_panel.backend import PanelBackend
from wc_panel.storage import DataStore


class Guard:
    def __init__(self):
        self.check = mock.Mock(return_value=dict(status='AVAILABLE'))


@pytest.fixture
def service(tmp_path):
    # Replace process/runtime dependencies only; exercise the real public
    # PanelBackend boundaries and real storage delete/active-path logic.
    backend = PanelBackend.__new__(PanelBackend)
    backend.project_root = tmp_path/'project'
    backend.project_root.mkdir()
    backend._lock = threading.RLock()
    backend._preparing_paths = {}
    backend._manager = mock.Mock()
    backend._manager.jobs.return_value = []
    backend._manager.active_paths.return_value = []
    backend._calibration_tools = mock.Mock()
    backend._calibration_tools.active_paths.return_value = []
    backend._profiles = mock.Mock()
    backend._profiles.apply.return_value = dict(name='fixture', revision='new', effective='next_task')
    backend.policy = Guard()
    backend.policy.resolve = lambda value: tmp_path/str(value)
    backend._store = DataStore(backend.project_root, backend._active_data_paths,
        resolver=lambda project, path: (Path(path), Guard()))
    recordings = tmp_path/'recordings'
    recordings.mkdir()
    source = recordings/'completed'
    source.mkdir()
    (source/'raw-original').write_bytes(b'original data retained')
    backend.settings = mock.Mock(return_value=dict(recording_root=str(recordings), results_root=str(tmp_path/'results')))
    return backend, source


@pytest.mark.parametrize('kind', ('live', 'capture', 'recovery'))
@pytest.mark.parametrize('status', ('QUEUED', 'RUNNING', 'STOPPING'))
def test_configuration_switch_refuses_active_hardware_and_save_tasks(service, kind, status):
    backend, _ = service
    backend._manager.jobs.return_value = [dict(id='owned-task', kind=kind, status=status)]
    with pytest.raises(ValueError, match='等待收尾'):
        backend.apply_configuration_profile('confirmed-preview')
    backend._profiles.apply.assert_not_called()


def test_configuration_switch_allows_frozen_offline_queue_and_finished_capture(service):
    backend, _ = service
    backend._manager.jobs.return_value = [dict(id='offline', kind='offline', status='RUNNING'),
                                         dict(id='recorded', kind='capture', status='COMPLETE')]
    result = backend.apply_configuration_profile('confirmed-preview')
    assert result['revision'] == 'new'
    backend._profiles.apply.assert_called_once_with('confirmed-preview')
    assert backend.policy.check.call_count == 2


@pytest.mark.parametrize('fail_prepare', (False, True))
def test_pairing_preparation_protects_input_and_output_and_releases_them(service, fail_prepare):
    backend, source = service
    guard = Guard()
    # A deletion preview issued before preparation must also fail at execution.
    preview = backend.prepare_delete(str(source.parent), [str(source)])
    calls = []

    def prepare(project, dataset, output):
        assert project == backend.project_root and dataset == source
        assert backend._store.active(source) and backend._store.active(output)
        assert source in backend._active_data_paths() and output in backend._active_data_paths()
        with pytest.raises(ValueError, match='占用'):
            backend.execute_delete(preview['token'])
        assert (source/'raw-original').read_bytes() == b'original data retained'
        calls.append(output)
        if fail_prepare:
            raise OSError('synthetic preparation failure')
        return dict(status='READY_FOR_OFFLINE_PICKING', prepared_input=str(output/'prepared.json'))

    with mock.patch('wc_panel.backend.resolve_user_destination', return_value=(source, guard)), \
            mock.patch('wc_panel.pairing_capture.prepare_recording', side_effect=prepare):
        if fail_prepare:
            with pytest.raises(OSError, match='synthetic preparation'):
                backend.prepare_pairing_capture(str(source))
        else:
            assert backend.prepare_pairing_capture(str(source))['prepared_input'].endswith('prepared.json')
    assert len(calls) == 1 and not backend._preparing_paths
    assert not backend._store.active(source) and not backend._store.active(calls[0])
    assert (source/'raw-original').read_bytes() == b'original data retained'


def test_incomplete_active_capture_cannot_enter_preparation(service):
    backend, source = service
    backend._manager.active_paths.return_value = [source]
    with mock.patch('wc_panel.backend.resolve_user_destination', return_value=(source, Guard())), \
            mock.patch('wc_panel.pairing_capture.prepare_recording') as prepare:
        with pytest.raises(ValueError, match='收尾'):
            backend.prepare_pairing_capture(str(source))
    prepare.assert_not_called()
    assert not backend._preparing_paths


def test_storage_loss_before_preparation_leaves_no_permanent_protection(service):
    backend, source = service
    guard = Guard()
    guard.check.side_effect = ValueError('synthetic USB unavailable')
    with mock.patch('wc_panel.backend.resolve_user_destination', return_value=(source, guard)), \
            mock.patch('wc_panel.pairing_capture.prepare_recording') as prepare:
        with pytest.raises(ValueError, match='USB unavailable'):
            backend.prepare_pairing_capture(str(source))
    prepare.assert_not_called()
    assert not backend._preparing_paths and (source/'raw-original').exists()
