"""SYNTHETIC GUI-startup retry checks; no ROS, display, processes or devices."""
import json
from pathlib import Path
import shutil
from types import SimpleNamespace
import uuid

import pytest

from wc_runtime import mapping_app as app
from wc_runtime import mapping_controller as controller
from test_mapping_app import Backend, names


DRAWABLE_ERROR = b'libGL error: failed to create drawable\n'


@pytest.fixture
def workspace(monkeypatch):
    parent = Path(__file__).absolute().parent
    root = parent / ('.mapping_rviz_retry_' + uuid.uuid4().hex)
    root.mkdir()
    request = app.prepare_request('right', name='synthetic_retry', project_root=root, mapping_enabled=True)
    directory = Path(request['output_dir'])
    directory.mkdir(parents=True)
    (directory/'runtime_config.json').write_text(
        json.dumps({'rviz_renderer': 'software', 'session_id': request['session_id']}), encoding='utf-8')
    (directory/'rviz.log').write_bytes(DRAWABLE_ERROR)
    monkeypatch.setattr(controller, 'ROOT', root)
    try:
        yield request, {'directory': str(directory), 'session_id': request['session_id']}
    finally:
        resolved = root.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.mapping_rviz_retry_')
        shutil.rmtree(resolved)


def retry_count(result):
    records = result['rviz_startup_retries']
    return records if type(records) is int else len(records)


@pytest.mark.parametrize('elapsed', [0, 2.5, 10.0])
def test_eligible_gate_is_read_only_and_preserves_original_log(workspace, elapsed):
    _, handle = workspace
    directory = Path(handle['directory'])
    before = {p.name: p.read_bytes() for p in directory.iterdir()}
    evidence = controller.rviz_startup_retry(handle, 245, elapsed)
    assert isinstance(evidence, dict) and evidence
    assert {p.name: p.read_bytes() for p in directory.iterdir()} == before


@pytest.mark.parametrize('elapsed', [-.001, 10.001, float('inf'), float('-inf'), float('nan'), True, None, '2'])
def test_gate_rejects_invalid_or_late_elapsed(workspace, elapsed):
    assert controller.rviz_startup_retry(workspace[1], 245, elapsed) is None


@pytest.mark.parametrize('exit_code', [0, 1, -11, 139, 245.0, True, None, '245'])
def test_gate_only_accepts_exact_wrapped_sigsegv_integer(workspace, exit_code):
    assert controller.rviz_startup_retry(workspace[1], exit_code, 1.) is None


@pytest.mark.parametrize('renderer', ['system', None, 'automatic'])
def test_gate_never_retries_other_renderer(workspace, renderer):
    _, handle = workspace
    (Path(handle['directory'])/'runtime_config.json').write_text(
        json.dumps({'rviz_renderer': renderer}), encoding='utf-8')
    assert controller.rviz_startup_retry(handle, 245, 1.) is None


@pytest.mark.parametrize('log', [b'', b'libGL error: different failure\n',
    DRAWABLE_ERROR+b'Actual Ogre renderer: device=llvmpipe\n',
    DRAWABLE_ERROR+b'Native RViz ready; closing discards temporary view edits.\n',
    DRAWABLE_ERROR+b'x'*65536], ids=['empty', 'different_error', 'renderer_ready', 'native_ready', 'oversized'])
def test_gate_requires_bounded_drawable_failure_before_native_ready(workspace, log):
    _, handle = workspace
    path = Path(handle['directory'])/'rviz.log'
    path.write_bytes(log)
    assert controller.rviz_startup_retry(handle, 245, 1.) is None
    assert path.read_bytes() == log


def test_gate_accepts_exact_size_limit_but_not_one_byte_more(workspace):
    _, handle = workspace
    path = Path(handle['directory'])/'rviz.log'
    log = DRAWABLE_ERROR+b'x'*(65536-len(DRAWABLE_ERROR))
    path.write_bytes(log)
    assert controller.rviz_startup_retry(handle, 245, 1.)
    path.write_bytes(log+b'x')
    assert controller.rviz_startup_retry(handle, 245, 1.) is None


class RetryBackend(Backend):
    def __init__(self, handle, **kwargs):
        super().__init__(**kwargs)
        self.handle = handle
        self.after_retry_check = lambda: None

    def start(self, request):
        super().start(request)
        return self.handle

    def storage_status(self, handle):
        assert handle == self.handle
        self.events.append(('storage', handle['session_id']))
        return {'free_bytes': 3*1024**3, 'stop_free_bytes': 2*1024**3}

    def rviz_startup_retry(self, handle, exit_code, elapsed_s):
        assert handle == self.handle
        self.events.append(('retry_check', exit_code, elapsed_s))
        result = controller.rviz_startup_retry(handle, exit_code, elapsed_s)
        self.after_retry_check()
        return result


def execute_retry(workspace, *, codes=(245, 0), first_elapsed=2., backend=None,
                  stop_requested=lambda: False, on_emit=lambda event: None):
    request, handle = workspace
    backend = backend or RetryBackend(handle)
    now = [100.]
    spawned, emitted = [], []

    def spawn(command, gui_request):
        index = len(spawned)
        assert index < len(codes), 'Unexpected extra GUI attempt'
        spawned.append(dict(gui_request))
        backend.events.append(('spawn', index+1))
        first_poll = [True]
        def poll():
            if first_poll[0]:
                now[0] += first_elapsed if index == 0 else .25
                first_poll[0] = False
            return codes[index]
        return SimpleNamespace(poll=poll)

    def emit(event):
        emitted.append(event)
        on_emit(event)

    result, code = app.run_session(request, backend, spawn=spawn,
        close_gui=lambda gui: backend.events.append(('close_gui', gui)),
        wait=lambda seconds: now.__setitem__(0, now[0]+seconds), clock=lambda: now[0],
        stop_requested=stop_requested, emit=emit,
        choose_save=lambda request, handle: {'decision': 'save',
            'destination': str(Path(request['project_root'])/'saved'/'retry_map')})
    return result, code, backend, spawned, emitted


def test_one_retry_keeps_original_backend_and_records_recovery(workspace):
    original = (Path(workspace[1]['directory'])/'rviz.log').read_bytes()
    result, code, backend, spawned, emitted = execute_retry(workspace)
    assert code == 0 and result['status'] == 'SAVED'
    assert len(spawned) == 2 and spawned[1]['rviz_startup_attempt'] == 2
    assert spawned[0].get('rviz_startup_attempt', 1) == 1
    for name in ('session_id', 'output_dir', 'mode', 'duration_s'):
        assert spawned[0][name] == spawned[1][name] == workspace[0][name]
    events = names(backend)
    assert events.count('start') == events.count('stop') == events.count('export') == 1
    assert events.index('inspect') < events.index('storage') < events.index('retry_check')
    assert events.count('retry_check') == 1 and retry_count(result) == 1
    assert sum(event['status'] == 'RVIZ_STARTUP_RETRY' for event in emitted) == 1
    assert (Path(workspace[1]['directory'])/'rviz.log').read_bytes() == original


def test_second_failure_never_retries_or_exports(workspace):
    result, code, backend, spawned, _ = execute_retry(workspace, codes=(245, 245))
    assert code == 2 and result['status'] == 'FAILED'
    assert len(spawned) == 2 and retry_count(result) == 1
    assert names(backend).count('start') == names(backend).count('stop') == 1
    assert names(backend).count('retry_check') == 1 and 'export' not in names(backend)


@pytest.mark.parametrize('gui_code,elapsed', [(0, 1.), (1, 1.), (139, 1.), (245, 10.1)])
def test_normal_close_other_errors_and_late_crash_do_not_retry(workspace, gui_code, elapsed):
    result, code, backend, spawned, _ = execute_retry(workspace, codes=(gui_code,), first_elapsed=elapsed)
    assert len(spawned) == 1 and retry_count(result) == 0
    assert code == (0 if gui_code == 0 else 2)
    assert ('export' in names(backend)) == (gui_code == 0)


@pytest.mark.parametrize('reason', ['pipeline_failed', 'storage_low'])
def test_pipeline_and_storage_stop_take_precedence_over_gui_retry(workspace, reason):
    backend = RetryBackend(workspace[1], states=[{'state': 'FAILED', 'exit_code': 1}]
                           if reason == 'pipeline_failed' else None)
    if reason == 'storage_low':
        backend.storage_status = lambda _: {'free_bytes': 2*1024**3, 'stop_free_bytes': 2*1024**3}
    result, _, backend, spawned, emitted = execute_retry(workspace, backend=backend)
    assert len(spawned) == 1 and 'retry_check' not in names(backend)
    assert retry_count(result) == 0 and not any(e['status'] == 'RVIZ_STARTUP_RETRY' for e in emitted)


def test_stop_arriving_during_retry_decision_prevents_second_gui(workspace):
    stopped = [False]
    backend = RetryBackend(workspace[1])
    backend.after_retry_check = lambda: stopped.__setitem__(0, True)
    result, _, backend, spawned, _ = execute_retry(workspace, backend=backend,
                                                  stop_requested=lambda: stopped[0])
    assert len(spawned) == 1 and names(backend).count('start') == 1
    assert result['stop_reason'] == 'USER_STOP_REQUEST'


def test_stop_arriving_during_retry_announcement_prevents_second_gui(workspace):
    stopped = [False]
    def emit(event):
        if event['status'] == 'RVIZ_STARTUP_RETRY':
            stopped[0] = True
    result, code, backend, spawned, _ = execute_retry(workspace,
        stop_requested=lambda: stopped[0], on_emit=emit)
    assert len(spawned) == 1 and names(backend).count('start') == 1
    assert result['stop_reason'] == 'USER_STOP_REQUEST'
    assert code == 2 and 'export' not in names(backend)


def test_stop_arriving_during_final_session_recheck_prevents_second_gui(workspace):
    stopped = [False]
    backend = RetryBackend(workspace[1])
    inspect = backend.inspect
    def inspect_with_stop(handle):
        result = inspect(handle)
        if names(backend).count('inspect') == 3:
            stopped[0] = True
        return result
    backend.inspect = inspect_with_stop
    result, code, backend, spawned, _ = execute_retry(workspace, backend=backend,
                                                     stop_requested=lambda: stopped[0])
    assert len(spawned) == 1 and names(backend).count('start') == 1
    assert result['stop_reason'] == 'USER_STOP_REQUEST'
    assert code == 2 and 'export' not in names(backend)


def test_launch_retry_uses_new_exclusive_log_and_keeps_first_attempt(workspace, monkeypatch):
    request, handle = workspace
    directory = Path(handle['directory'])
    observed = []
    def launch(command, **kwargs):
        observed.append((command, Path(kwargs['stdout'].name)))
        kwargs['stdout'].write(b'SYNTHETIC SECOND GUI\n')
        return SimpleNamespace(poll=lambda: None)
    monkeypatch.setattr(app.subprocess, 'Popen', launch)
    app.launch_rviz(['synthetic-rviz-no-execution'], {**request, 'rviz_startup_attempt': 2})
    assert observed[0][1] == directory/'rviz.retry-1.log'
    assert (directory/'rviz.log').read_bytes() == DRAWABLE_ERROR
    assert (directory/'rviz.retry-1.log').read_bytes() == b'SYNTHETIC SECOND GUI\n'
    with pytest.raises(FileExistsError):
        app.launch_rviz(['synthetic-rviz-no-execution'], {**request, 'rviz_startup_attempt': 2})
    assert len(observed) == 1


@pytest.mark.parametrize('attempt', [0, 3, True, 1.0, '2', None])
def test_launch_rejects_invalid_attempt_before_process_creation(workspace, monkeypatch, attempt):
    def forbidden(*args, **kwargs):
        raise AssertionError('Invalid attempt started a process')
    monkeypatch.setattr(app.subprocess, 'Popen', forbidden)
    with pytest.raises(ValueError):
        app.launch_rviz(['synthetic-rviz-no-execution'], {**workspace[0], 'rviz_startup_attempt': attempt})
