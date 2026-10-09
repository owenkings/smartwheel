"""Production lifecycle + native Qt decision + real bounded file retention.

Only device acquisition and map export are synthetic; no hardware is started.
The dedicated Qt test driver includes the production dialog source unchanged.
"""
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from wc_runtime import mapping_app, mapping_save


@pytest.fixture
def native_driver():
    path = os.environ.get('WC_PANEL_SAVE_CHOICE_DRIVER')
    if not path:
        pytest.skip('Native Qt save driver requires target BUILD_TESTING build')
    assert Path(path).is_file()
    return path


@pytest.fixture
def chain(tmp_path):
    project = tmp_path / 'project'
    (project / 'config').mkdir(parents=True)
    archive = tmp_path / 'archive'
    archive.mkdir()
    (project / 'config/storage.json').write_text(json.dumps(dict(
        schema_version=2, backend='directory', archive_root=str(archive), fallback_allowed=False)))
    request = mapping_app.prepare_request('all', project_root=project,
                                         name='SYNTHETIC_ACCEPTANCE', mapping_enabled=True)
    directory = Path(request['output_dir'])
    directory.mkdir(parents=True)
    runtime = project / '.phase1_runtime/sessions/SYNTHETIC_ACCEPTANCE/mapping_app'
    runtime.mkdir(parents=True)
    handle = dict(session_id=request['session_id'], directory=str(directory), runtime=str(runtime),
                  mapping_enabled=True, retention_profile='experiment', mode='all')
    (directory / 'session.json').write_text(json.dumps(dict(session_id=request['session_id'],
        output_dir=str(directory), mapping_enabled=True, data_retention='TEMPORARY_UNTIL_USER_SAVE_CHOICE')))
    (directory / 'runtime_config.json').write_text('{"mapping_enabled":true}')
    for name in ('bag', 'slam', 'export'):
        (directory / name).mkdir()
    (directory / 'bag/source.db3').write_bytes(b'SYNTHETIC source')
    (directory / 'slam/rtabmap.db').write_bytes(b'SYNTHETIC database, no map accuracy claim')
    (directory / 'export/map.ply').write_bytes(b'SYNTHETIC exported map')
    independent = archive / 'data/experiments/independent'
    independent.mkdir(parents=True)
    (independent / 'source.db3').write_bytes(b'INDEPENDENT recording')

    class Backend:
        stopped = False
        fail_stop = False
        def preflight(self, request): return {}
        def start(self, request): return handle
        def rviz_command(self, handle): return ['synthetic-window-close']
        def inspect(self, handle):
            return dict(state='FAILED' if self.fail_stop and self.stopped else
                        'STOPPED' if self.stopped else 'RUNNING', exit_code=1 if self.fail_stop else 0)
        def stop(self, handle):
            self.stopped = True
            return self.inspect(handle)
        def save(self, handle, destination):
            return mapping_save.save_session(project, handle, destination, inspect=self.inspect,
                export=lambda h: dict(status='EXPORTED_EXPERIMENTAL_MAP', level='SYNTHETIC'))
        def discard(self, handle):
            return mapping_save.discard_session(project, handle, inspect=self.inspect, user_confirmed=True)

    return request, Backend(), directory, independent, tmp_path / 'selected_map'


@pytest.mark.parametrize('action,status,code', [
    ('save', 'SAVED', 0), ('cancel_then_save', 'SAVED', 0),
    ('discard', 'DISCARDED', 0), ('cancel_then_pending', 'SAVE_PENDING', 3),
    ('save_lost_destination', 'FAILED', 2), ('stop_failed', 'FAILED', 2)])
def test_close_stop_native_choice_then_real_retention(chain, native_driver, action, status, code):
    request, backend, directory, independent, destination = chain
    announcements, choices = [], []
    backend.fail_stop = action == 'stop_failed'
    def choose(request, handle):
        assert backend.inspect(handle)['state'] == 'STOPPED'
        assert announcements[-1]['status'] == 'SESSION_STOPPED_AWAITING_SAVE'
        invocation = 'save' if action == 'save_lost_destination' else action
        result = subprocess.run([native_driver, invocation, str(destination)],
            env=dict(os.environ, QT_QPA_PLATFORM='offscreen'), capture_output=True, text=True, timeout=12)
        assert result.returncode == 0, result.stderr
        value = json.loads(result.stdout)
        choices.append(value)
        if action == 'save_lost_destination':
            # A concurrently created target must never be overwritten.
            destination.mkdir()
            (destination / 'other_owner').write_bytes(b'preserve')
        return value
    result, actual_code = mapping_app.run_session(request, backend,
        spawn=lambda *a: SimpleNamespace(poll=lambda: 0), close_gui=lambda g: None,
        choose_save=choose, emit=announcements.append)
    assert (result['status'], actual_code) == (status, code)
    assert (independent / 'source.db3').read_bytes() == b'INDEPENDENT recording'
    assert (directory / 'bag/source.db3').exists() == (action != 'discard')
    if status == 'SAVED':
        assert (destination / 'export/map.ply').read_bytes() == b'SYNTHETIC exported map'
    if action == 'save_lost_destination':
        assert (destination / 'other_owner').read_bytes() == b'preserve'
    if action.startswith('cancel_'):
        assert choices[0]['test_choice_count'] == 2
    if action == 'stop_failed':
        assert choices == []
