"""SYNTHETIC recovery uses metadata fixtures and a backend with no device methods."""
import io
import json
from pathlib import Path

import pytest

from wc_runtime import map_save_app as app
from test_mapping_app import tmp_path


def write(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


@pytest.fixture
def session(tmp_path):
    directory = tmp_path/'reports/maps/pending_map'
    runtime = tmp_path/'.phase1_runtime/sessions/pending_map/mapping_app'
    directory.mkdir(parents=True)
    runtime.mkdir(parents=True)
    record = {'kind': 'MAPPING_APP_EXPERIMENT', 'data_retention': 'TEMPORARY_UNTIL_USER_SAVE_CHOICE',
              'session_id': 'pending_map', 'project_root': str(tmp_path), 'output_dir': str(directory),
              'mode': 'right', 'mapping_enabled': True, 'odometry_source': 'wheel_imu', 'source_mode': 'real'}
    write(directory/'session.json', record)
    write(directory/'runtime_config.json', {**record, 'schema_version': 1, 'status': 'EXPERIMENT'})
    write(runtime/'plan.json', {'session_id': 'pending_map', 'role': 'mapping_app', 'mapping_enabled': True})
    write(runtime/'manifest.json', {'session_id': 'pending_map', 'role': 'mapping_app',
                                  'state': 'STOPPED', 'exit_code': 0, 'cleanup_errors': []})
    (directory/'raw_fixture.cdr').write_bytes(b'original data')
    return tmp_path, directory, runtime


class Backend:
    def __init__(self, state=None):
        self.state = state or {'state': 'STOPPED', 'exit_code': 0, 'cleanup_errors': []}
        self.events = []

    def inspect(self, handle):
        self.events.append(('inspect', dict(handle)))
        return self.state

    def save(self, handle, destination):
        self.events.append(('save', dict(handle), destination))
        return {'status': 'SAVED_EXPERIMENTAL_MAP', 'output': destination, 'raw_recordings_saved': False}

    def discard(self, handle):
        self.events.append(('discard', dict(handle)))
        return {'status': 'SESSION_DATA_DISCARDED'}


def recover(session, backend, destination=None, **options):
    root, directory, _ = session
    return app.recover_session(directory, destination, project_root=root, backend=backend, **options)


def test_explicit_destination_reconstructs_handle_without_prompt_or_device_calls(session):
    root, directory, runtime = session
    backend = Backend()
    destination = root.parent/'user_maps'/'chosen_map'
    def forbidden(*args):
        pytest.fail('Explicit destination must not prompt')
    result, code = recover(session, backend, destination, choose_save=forbidden)
    assert code == 0 and result['status'] == 'SAVED'
    assert [event[0] for event in backend.events] == ['inspect', 'save']
    assert backend.events[-1][1:] == ({'session_id': 'pending_map', 'mode': 'right',
        'directory': str(directory), 'runtime': str(runtime), 'odometry_source': 'wheel_imu',
        'mapping_enabled': True}, str(destination))
    assert result['output'] == str(destination) and result['session_output'] == str(directory)
    assert '地图数据库、点云与二维地图已保存' in result['message']
    assert result['save']['raw_recordings_saved'] is False


def test_interactive_choice_is_only_called_after_fresh_closed_inspection(session):
    backend = Backend()
    def choose(request, handle):
        assert [event[0] for event in backend.events] == ['inspect']
        assert request['session_id'] == handle['session_id'] == 'pending_map'
        return {'decision': 'save', 'destination': str(session[0]/'saved_map')}
    result, code = recover(session, backend, choose_save=choose)
    assert code == 0 and result['status'] == 'SAVED'
    assert backend.events[-1][0] == 'save'


def test_no_discards_only_reconstructed_current_session(session):
    backend = Backend()
    result, code = recover(session, backend, choose_save=lambda *_: {'decision': 'discard'})
    assert code == 0 and result['status'] == 'DISCARDED'
    assert [event[0] for event in backend.events] == ['inspect', 'discard']
    assert backend.events[-1][1]['directory'] == str(session[1])


def test_current_experiment_profile_can_resume_pending_user_choice(session):
    root,directory,runtime=session
    for name in ('session.json','runtime_config.json'):
        value=json.loads((directory/name).read_text())
        value['data_retention']='experiment'
        if name=='runtime_config.json': value['retention_profile']='experiment'
        write(directory/name,value)
    backend=Backend()
    result,code=recover(session,backend,choose_save=lambda *_:{'decision':'discard'})
    assert code==0 and result['status']=='DISCARDED'
    assert backend.events[-1][1]['retention_profile']=='experiment'


def test_saved_archive_cannot_be_recovered_as_pending_discard(session):
    write(session[1]/'retention.json',{'saved_map':'/previously/saved','raw_retention':'RETAINED_REFERENCED'})
    backend=Backend()
    result,code=recover(session,backend,choose_save=lambda *_:{'decision':'discard'})
    assert code==2 and backend.events==[]
    assert (session[1]/'raw_fixture.cdr').read_bytes()==b'original data'


@pytest.mark.parametrize('cause', ['pending', 'eof', 'interrupt'])
def test_pending_keeps_original_data_without_backend_mutation(session, cause):
    backend = Backend()
    def choose(*args):
        if cause == 'eof': raise EOFError
        if cause == 'interrupt': raise KeyboardInterrupt
        return {'decision': 'pending', 'reason': 'NON_INTERACTIVE_TERMINAL'}
    result, code = recover(session, backend, choose_save=choose)
    assert code == 3 and result['status'] == 'SAVE_PENDING' and result['errors'] == []
    assert [event[0] for event in backend.events] == ['inspect']
    assert (session[1]/'raw_fixture.cdr').read_bytes() == b'original data'


def test_default_non_tty_choice_never_reads_or_deletes(session, monkeypatch):
    class NonTerminal(io.StringIO):
        def readline(self): pytest.fail('Non-TTY recovery must not wait for stdin')
    monkeypatch.setattr('sys.stdin', NonTerminal('n\n'))
    backend = Backend()
    result, code = recover(session, backend)
    assert code == 3 and result['save_decision']['reason'] == 'NON_INTERACTIVE_TERMINAL'
    assert [event[0] for event in backend.events] == ['inspect']


@pytest.mark.parametrize('state', [{'state': 'RUNNING'}, {'state': 'FAILED', 'exit_code': 1},
    {'state': 'STOPPED', 'exit_code': 1}, {'state': 'STOPPED', 'exit_code': 0, 'cleanup_errors': ['child alive']}])
def test_live_or_abnormal_backend_never_prompts_or_mutates(session, state):
    backend = Backend(state)
    def forbidden(*args): pytest.fail('Unclosed session must never prompt')
    result, code = recover(session, backend, choose_save=forbidden)
    assert code == 2 and result['status'] == 'FAILED'
    assert [event[0] for event in backend.events] == ['inspect']


@pytest.mark.parametrize('filename,key,value', [
    ('session.json', 'data_retention', None), ('session.json', 'kind', 'SAVED_MAP'),
    ('session.json', 'output_dir', '/elsewhere'), ('session.json', 'project_root', '/elsewhere'),
    ('session.json', 'session_id', '../history'), ('session.json', 'session_id', None),
    ('session.json', 'mapping_enabled', 1), ('session.json', 'mode', 'dual'),
    ('session.json', 'source_mode', 'synthetic'), ('session.json', 'odometry_source', 'guess'),
    ('runtime_config.json', 'session_id', 'another_session'), ('runtime_config.json', 'mode', 'left'),
    ('runtime_config.json', 'mapping_enabled', 1), ('runtime_config.json', 'odometry_source', 'icp'),
    ('runtime_config.json', 'status', 'VALIDATED'), ('runtime_config.json', 'source_mode', 'synthetic'),
    ('plan.json', 'session_id', 'another_session'), ('plan.json', 'role', 'drivers'),
    ('plan.json', 'mapping_enabled', False), ('manifest.json', 'session_id', 'another_session'),
    ('manifest.json', 'role', 'drivers'), ('manifest.json', 'state', 'RUNNING'),
    ('manifest.json', 'exit_code', 1), ('manifest.json', 'cleanup_errors', ['unclosed']),
])
def test_untrusted_or_historical_metadata_is_rejected_before_any_backend_call(session, filename, key, value):
    parent = session[2] if filename in ('plan.json', 'manifest.json') else session[1]
    path = parent/filename
    record = json.loads(path.read_text(encoding='utf-8'))
    if value is None: record.pop(key)
    else: record[key] = value
    write(path, record)
    backend = Backend()
    result, code = recover(session, backend, session[0]/'saved')
    assert code == 2 and result['status'] == 'FAILED' and result['errors']
    assert backend.events == []
    assert (session[1]/'raw_fixture.cdr').read_bytes() == b'original data'


@pytest.mark.parametrize('filename', ['session.json', 'runtime_config.json', 'plan.json', 'manifest.json'])
def test_missing_evidence_does_not_guess_a_handle(session, filename):
    parent = session[2] if filename in ('plan.json', 'manifest.json') else session[1]
    (parent/filename).unlink()
    backend = Backend()
    result, code = recover(session, backend, session[0]/'saved')
    assert code == 2 and result['status'] == 'FAILED' and backend.events == []


@pytest.mark.parametrize('workdir', ['.', '.phase1_runtime', '../outside'])
def test_workdir_must_be_scoped_inside_project_without_traversal(session, workdir):
    backend = Backend()
    result, code = app.recover_session(workdir, project_root=session[0], backend=backend)
    assert code == 2 and result['status'] == 'FAILED' and backend.events == []


def test_existing_destination_is_preserved_without_save_or_discard(session):
    destination = session[0]/'historical_map'
    destination.mkdir()
    (destination/'original.db').write_bytes(b'historical database')
    backend = Backend()
    result, code = recover(session, backend, destination)
    assert code == 2 and result['status'] == 'FAILED'
    assert [event[0] for event in backend.events] == ['inspect']
    assert (destination/'original.db').read_bytes() == b'historical database'


def test_destination_expands_home_and_relative_paths_use_project_root(session, monkeypatch):
    monkeypatch.setenv('USERPROFILE' if __import__('os').name == 'nt' else 'HOME', str(session[0]/'home'))
    for entered, expected in [('~/maps/chosen', session[0]/'home/maps/chosen'),
                               ('chosen_map', session[0]/'chosen_map')]:
        backend = Backend()
        result, code = recover(session, backend, entered)
        assert code == 0 and backend.events[-1][2] == str(expected)


def test_save_failure_can_retry_without_discarding_data(session):
    backend = Backend()
    original = backend.save
    def fail(*args): raise RuntimeError('export failed')
    backend.save = fail
    result, code = recover(session, backend, session[0]/'saved')
    assert code == 2 and result['status'] == 'FAILED' and result['errors'] == ['export failed']
    assert (session[1]/'raw_fixture.cdr').read_bytes() == b'original data'
    backend.save = original
    result, code = recover(session, backend, session[0]/'saved')
    assert code == 0 and result['status'] == 'SAVED'
    assert 'discard' not in [event[0] for event in backend.events]


def test_saved_cleanup_failure_still_reports_saved_map(session):
    backend = Backend()
    original = backend.save
    backend.save = lambda *args: {**original(*args), 'cleanup_errors': ['temporary cleanup failed']}
    result, code = recover(session, backend, session[0]/'saved')
    assert code == 0 and result['status'] == 'SAVED'
    assert result['cleanup_errors'] == ['temporary cleanup failed']
    assert '临时数据清理未完成' in result['message']


def test_preview_can_be_discarded_but_cannot_be_saved_as_a_map(session):
    for parent, filename in [(session[1], 'session.json'), (session[1], 'runtime_config.json'), (session[2], 'plan.json')]:
        value = json.loads((parent/filename).read_text(encoding='utf-8'))
        value['mapping_enabled'] = False
        write(parent/filename, value)
    backend = Backend()
    result, code = recover(session, backend, session[0]/'saved')
    assert code == 2 and '纯预览' in result['errors'][0]
    assert [event[0] for event in backend.events] == ['inspect']
    result, code = recover(session, backend, choose_save=lambda *_: {'decision': 'discard'})
    assert code == 0 and result['status'] == 'DISCARDED'


def test_main_explicit_destination_returns_json_without_loading_hardware(session, capsys):
    backend = Backend()
    code = app.main([str(session[1]), str(session[0]/'saved')], project_root=session[0], backend=backend)
    result = json.loads(capsys.readouterr().out)
    assert code == 0 and result['status'] == 'SAVED'
    assert [event[0] for event in backend.events] == ['inspect', 'save']


def test_main_non_tty_returns_pending_json_and_exit_three(session, monkeypatch, capsys):
    monkeypatch.setattr('sys.stdin', io.StringIO())
    assert app.main([str(session[1])], project_root=session[0], backend=Backend()) == 3
    assert json.loads(capsys.readouterr().out)['status'] == 'SAVE_PENDING'


def test_recovery_launcher_uses_project_entry_and_disables_user_site():
    source = Path(__file__).absolute().parents[2]/'scripts/save_map'
    text = source.read_text(encoding='utf-8')
    compile(text, str(source), 'exec')
    assert "PROJECT = Path('/home/nvidia/wheelchair')" in text
    assert 'from wc_runtime.map_save_app import main' in text
    assert "[sys.executable, '-s', __file__, *sys.argv[1:]]" in text
