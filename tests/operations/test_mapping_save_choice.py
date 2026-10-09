"""SYNTHETIC stop-before-choice flow; no devices and no real data removal."""
import io
import signal
from pathlib import Path

import pytest

from wc_runtime import mapping_app as app
from test_mapping_app import Backend, execute, names, request, tmp_path


class Terminal(io.StringIO):
    def isatty(self):
        return True


@pytest.mark.parametrize('arguments, enabled', [([], False), (['--mapping', 'false'], False),
                                               (['--mapping', 'true'], True)])
def test_cli_maps_only_when_explicitly_enabled(tmp_path, arguments, enabled):
    value = app.parse_request(['right', *arguments], project_root=tmp_path)
    assert value['mapping_enabled'] is enabled
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('arguments', [['--mapping'], ['--mapping', 'yes'], ['--mapping', '1'],
                                     ['--mapping', 'TRUE']])
def test_cli_rejects_ambiguous_mapping_switch(tmp_path, arguments):
    with pytest.raises(SystemExit):
        app.parse_request(['right', *arguments], project_root=tmp_path)


@pytest.mark.parametrize('enabled', [1, 'true', None])
def test_request_requires_boolean_mapping_mode(tmp_path, enabled):
    with pytest.raises(ValueError, match='mapping_enabled'):
        app.prepare_request('right', project_root=tmp_path, mapping_enabled=enabled)


def test_preview_stops_and_discards_current_session_without_save_prompt_or_export(tmp_path):
    value = app.prepare_request('right', project_root=tmp_path)
    backend, announcements = Backend(), []
    def forbidden(*args):
        pytest.fail('A live preview must never ask to save a map')
    result, code = execute(value, backend, choose_save=forbidden, emit=announcements.append)
    assert code == 0 and result['status'] == 'PREVIEW_COMPLETE'
    assert result['mapping_enabled'] is False
    assert names(backend)[-3:] == ['close_gui', 'stop', 'discard']
    assert 'save' not in names(backend) and 'export' not in names(backend)
    assert backend.events[-1] == ('discard', value['session_id'])
    started = next(row for row in announcements if row['status'] == 'LIVE_PREVIEW_STARTED')
    assert '不建图、不录包' in started['message']


@pytest.mark.parametrize('cause', ['window', 'ctrl_c', 'signal'])
def test_mapping_asks_after_devices_stop_and_passes_external_destination(tmp_path, cause):
    backend = Backend()
    value = request(tmp_path)
    chosen = str(tmp_path.parent/'outside_project'/'chosen_map')
    def choose(candidate, handle):
        assert names(backend)[-1] == 'stop'
        assert 'save' not in names(backend) and 'export' not in names(backend)
        backend.events.append(('prompt', handle['session_id']))
        return {'decision': 'save', 'destination': chosen}
    def interrupt(_):
        raise KeyboardInterrupt
    options = {'gui_code': None, 'wait': interrupt} if cause == 'ctrl_c' else {}
    if cause == 'signal':
        options['stop_requested'] = lambda: 'start' in names(backend)
    result, code = execute(value, backend, choose_save=choose, **options)
    assert code == 0 and result['status'] == 'SAVED'
    assert names(backend)[-4:] == ['stop', 'prompt', 'save', 'export']
    assert next(row for row in backend.events if row[0] == 'save')[2] == chosen
    assert result['output'] == chosen and result['session_output'] == value['output_dir']
    assert 'discard' not in names(backend)


def test_declining_save_discards_only_this_session_without_export(tmp_path):
    value, backend = request(tmp_path), Backend()
    def choose(candidate, handle):
        assert names(backend)[-1] == 'stop'
        return {'decision': 'discard'}
    result, code = execute(value, backend, choose_save=choose)
    assert code == 0 and result['status'] == 'DISCARDED'
    assert backend.events[-1] == ('discard', value['session_id'])
    assert 'save' not in names(backend) and 'export' not in names(backend)


@pytest.mark.parametrize('choice', ['pending', 'eof', 'interrupt'])
def test_undecided_mapping_retains_data_after_stop_without_export_or_discard(tmp_path, choice):
    backend = Backend()
    def choose(*args):
        if choice == 'eof':
            raise EOFError
        if choice == 'interrupt':
            raise KeyboardInterrupt
        return {'decision': 'pending', 'reason': 'NON_INTERACTIVE_TERMINAL'}
    result, code = execute(request(tmp_path), backend, choose_save=choose)
    assert code == 3 and result['status'] == 'SAVE_PENDING'
    assert result['closed_session']['state'] == 'STOPPED' and result['errors'] == []
    assert names(backend)[-1] == 'stop'
    assert not {'save', 'export', 'discard'} & set(names(backend))


@pytest.mark.parametrize('enabled', [False, True])
def test_failed_stop_never_prompts_or_deletes_data(tmp_path, enabled):
    backend = Backend(stop_state={'state': 'FAILED', 'exit_code': 1})
    def forbidden(*args):
        pytest.fail('Devices must stop normally before any data decision')
    result, code = execute(request(tmp_path, mapping_enabled=enabled), backend, choose_save=forbidden)
    assert code == 2 and result['status'] == 'FAILED'
    assert not {'save', 'export', 'discard'} & set(names(backend))


def test_saved_map_remains_saved_when_temporary_cleanup_reports_errors(tmp_path):
    backend = Backend()
    original = backend.save
    def saved_with_cleanup_issue(handle, destination):
        return {**original(handle, destination), 'cleanup_errors': ['temporary input removal failed']}
    backend.save = saved_with_cleanup_issue
    result, code = execute(request(tmp_path), backend)
    assert code == 0 and result['status'] == 'SAVED'
    assert result['cleanup_errors'] == ['temporary input removal failed']
    assert '临时数据清理未完成' in result['message']
    assert '临时大数据已清理' not in result['message']


def test_save_failure_keeps_session_location_and_never_discards(tmp_path):
    backend = Backend()
    def fail(*args):
        raise FileExistsError('destination already exists')
    backend.save = fail
    value = request(tmp_path)
    result, code = execute(value, backend)
    assert code == 2 and result['status'] == 'FAILED'
    assert result['output'] == value['output_dir'] and 'discard' not in names(backend)
    assert 'destination already exists' in result['errors'][0]


def test_non_tty_does_not_read_or_apply_default_no(tmp_path):
    class NonTerminal(io.StringIO):
        def readline(self):
            pytest.fail('Non-TTY input must never block or infer consent')
    result = app.terminal_save_choice(request(tmp_path), {}, input_stream=NonTerminal('n\n'))
    assert result == {'decision': 'pending', 'reason': 'NON_INTERACTIVE_TERMINAL'}


def test_closed_terminal_keeps_data_pending(tmp_path):
    incoming = Terminal()
    incoming.close()
    result = app.terminal_save_choice(request(tmp_path), {}, input_stream=incoming, output_stream=io.StringIO())
    assert result['decision'] == 'pending'


@pytest.mark.parametrize('answer', ['n\n', 'NO\n'])
def test_terminal_explicit_no_discards_with_scope_explanation(tmp_path, answer):
    output = io.StringIO()
    result = app.terminal_save_choice(request(tmp_path), {}, input_stream=Terminal(answer), output_stream=output)
    assert result == {'decision': 'discard', 'reason': 'USER_DECLINED_SAVE'}
    assert '废弃本次地图和原始记录' in output.getvalue() and '保存到新的目录' not in output.getvalue()


@pytest.mark.parametrize('answer', ['', '\n', 'y\n'])
def test_terminal_eof_at_either_question_keeps_data_pending(tmp_path, answer):
    result = app.terminal_save_choice(request(tmp_path), {}, input_stream=Terminal(answer), output_stream=io.StringIO())
    assert result['decision'] == 'pending'


def test_terminal_yes_uses_project_storage_maps_directory(tmp_path, monkeypatch):
    monkeypatch.setattr(app.Path, 'home', lambda: tmp_path/'user_home')
    value = request(tmp_path)
    result = app.terminal_save_choice(value, {}, input_stream=Terminal('Y\n\n'), output_stream=io.StringIO())
    assert result == {'decision': 'save', 'destination': str(tmp_path/'maps'/value['session_id'])}
    assert not (tmp_path/'user_home').exists()


def test_terminal_reprompts_existing_destination_without_touching_it(tmp_path):
    existing = tmp_path/'history'; existing.mkdir()
    evidence = existing/'map.db'; evidence.write_bytes(b'existing historical map')
    chosen = tmp_path/'new_map'
    output = io.StringIO()
    result = app.terminal_save_choice(request(tmp_path), {},
        input_stream=Terminal('yes\n%s\n%s\n' % (existing, chosen)), output_stream=output)
    assert result == {'decision': 'save', 'destination': str(chosen)}
    assert '已存在' in output.getvalue()
    assert evidence.read_bytes() == b'existing historical map' and not chosen.exists()


def test_prompt_interrupt_becomes_pending_and_restores_acquisition_signal_handlers(tmp_path, monkeypatch):
    handlers = {signal.SIGINT: object(), signal.SIGTERM: object()}
    originals = dict(handlers)
    def register(signum, handler):
        previous = handlers[signum]
        handlers[signum] = handler
        return previous
    class InterruptedTerminal(Terminal):
        def readline(self):
            handlers[signal.SIGINT](signal.SIGINT, None)
    monkeypatch.setattr(app.signal, 'signal', register)
    result = app.terminal_save_choice(request(tmp_path), {},
        input_stream=InterruptedTerminal(), output_stream=io.StringIO())
    assert result['decision'] == 'pending' and handlers == originals
