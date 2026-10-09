"""Offline X11 metadata/image fixtures; no desktop, ROS or hardware is opened."""

import importlib.util
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import shutil
import uuid
import zlib

import pytest


spec = importlib.util.spec_from_file_location('rviz_capture_fixture', Path(__file__).with_name('check_rviz_display.py'))
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def candidate(**changes):
    properties = '''_NET_WM_PID(CARDINAL) = 42
_NET_WM_WINDOW_TYPE(ATOM) = _NET_WM_WINDOW_TYPE_NORMAL
_NET_WM_STATE(ATOM) =
WM_STATE(WM_STATE):
        window state: Normal
        icon window: 0x0
WM_TRANSIENT_FOR:  not found.
'''
    geometry = '''  Absolute upper-left X:  100
  Absolute upper-left Y:  20
  Width: 1280
  Height: 900
  Map State: IsViewable
'''
    result = checker.window_metadata('0x10', properties, geometry)
    result.update(changes)
    return result


def select(*items):
    return checker.select_main_window(items, {42}, (1920, 1080))


def test_same_pid_save_dialog_never_becomes_main_window():
    normal = candidate()
    dialog = candidate(window_id='0x20', types=['_NET_WM_WINDOW_TYPE_DIALOG'],
                       modal=True, transient_for=16, width=1600, height=1000)
    assert select(dialog, normal)['window_id'] == '0x10'
    with pytest.raises(RuntimeError, match='No mapped owned NORMAL'):
        select(dialog)


@pytest.mark.parametrize('change', [
    {'pid': 900}, {'types': []}, {'types': ['_NET_WM_WINDOW_TYPE_NORMAL', '_NET_WM_WINDOW_TYPE_DIALOG']},
    {'mapped': False}, {'normal_state': False}, {'hidden': True}, {'modal': True},
    {'transient_for': 16}, {'width': 639}, {'height': 359}, {'x': 1800}, {'y': -899}, {'width': None},
])
def test_foreign_unmapped_modal_tiny_offscreen_or_incomplete_window_refused(change):
    with pytest.raises(RuntimeError, match='No mapped owned NORMAL'):
        select(candidate(**change))


def test_selection_uses_visible_area_before_total_window_area():
    mostly_offscreen = candidate(window_id='0x20', x=-1000, width=1800, height=900)
    normal = candidate()
    assert select(mostly_offscreen, normal)['window_id'] == '0x10'


def test_properties_identify_mapped_same_pid_transient_dialog():
    item = checker.window_metadata('0x21', '''_NET_WM_PID(CARDINAL) = 42
_NET_WM_WINDOW_TYPE(ATOM) = _NET_WM_WINDOW_TYPE_DIALOG
_NET_WM_STATE(ATOM) = _NET_WM_STATE_MODAL
WM_STATE(WM_STATE):
    window state: Normal
WM_TRANSIENT_FOR(WINDOW): window id # 0x10
''', 'Map State: IsViewable\n Width: 300\n Height: 180\n Absolute upper-left X: 1\n Absolute upper-left Y: 2')
    assert item['mapped'] and item['normal_state'] and item['modal']
    assert item['pid'] == 42 and item['transient_for'] == 16
    with pytest.raises(RuntimeError):
        select(item)


def png_header(width=1280, height=900):
    ihdr = b'IHDR' + struct.pack('>2I5B', width, height, 8, 2, 0, 0, 0)
    return b'\x89PNG\r\n\x1a\n' + struct.pack('>I', 13) + ihdr + struct.pack('>I', zlib.crc32(ihdr) & 0xffffffff)


def test_native_png_header_reports_dimensions_without_pixel_conversion():
    assert checker.png_dimensions(png_header()) == (1280, 900)
    for data in (png_header()[:32], b'X' + png_header()[1:], png_header()[:-1] + b'X', png_header(0, 900)):
        with pytest.raises(RuntimeError):
            checker.png_dimensions(data)


@pytest.mark.parametrize('active', ['0x20', '0x0', 'not found'])
def test_save_dialog_foreign_window_or_missing_active_window_refused(active):
    with pytest.raises(RuntimeError, match='main window is not active'):
        checker.require_active_main_window('_NET_ACTIVE_WINDOW(WINDOW): window id # ' + active, '0x10')


def test_active_id_comparison_accepts_zero_padded_window_id():
    checker.require_active_main_window('_NET_ACTIVE_WINDOW(WINDOW): window id # 0x00000010', '0x10')


@pytest.mark.parametrize('active_ids,png_size,expected_error', [
    (['0x10', '0x10'], (1280, 928), None),
    (['0x20'], (1280, 928), 'main window is not active'),
    (['0x10', '0x20'], (1280, 928), 'main window is not active'),
    (['0x10', '0x10'], (320, 160), 'dimensions do not match'),
    (['0x10', '0x10'], (1920, 1080), 'dimensions do not match'),
])
def test_capture_active_window_checks_surround_actual_screenshot_call(monkeypatch, active_ids, png_size, expected_error):
    # Only in-memory process/command/file fixtures; no X11 command is executed.
    import sys
    from types import ModuleType, SimpleNamespace
    component = ModuleType('wc_runtime.component')
    owner, rviz = SimpleNamespace(pid=1), SimpleNamespace(pid=42)
    component.process = lambda pid: owner
    component.descendants = lambda record: [rviz]
    component.belongs_to = lambda record, parent: True
    monkeypatch.setitem(sys.modules, 'wc_runtime.component', component)
    active = iter(active_ids)
    calls = []
    properties = '_NET_WM_PID(CARDINAL) = 42\n_NET_WM_WINDOW_TYPE(ATOM) = _NET_WM_WINDOW_TYPE_NORMAL\n_NET_WM_STATE(ATOM) =\nWM_STATE(WM_STATE):\n    window state: Normal\nWM_TRANSIENT_FOR: not found.\n'
    geometry = '  Absolute upper-left X: 100\n  Absolute upper-left Y: 20\n  Width: 1280\n  Height: 900\n  Map State: IsViewable\n'
    def read(command, **kwargs):
        calls.append(command)
        if command == ['xprop', '-root', '_NET_CLIENT_LIST']:
            return '_NET_CLIENT_LIST(WINDOW): window id # 0x10'
        if command == ['xprop', '-root', '_NET_ACTIVE_WINDOW']:
            return '_NET_ACTIVE_WINDOW(WINDOW): window id # ' + next(active)
        if command == ['xwininfo', '-root', '-stats']:
            return '  Width: 1920\n  Height: 1080\n'
        if command[0] == 'xwininfo': return geometry
        if command[0] == 'xprop': return properties
        raise AssertionError('Unexpected command: ' + repr(command))
    class Output:
        data = None
        def exists(self): return self.data is not None
        def read_bytes(self): return self.data
        def unlink(self, **kwargs): self.data = None
        def __str__(self): return '/OWNED_FIXTURE.png'
    output = Output()
    def screenshot(command, **kwargs):
        calls.append(command)
        assert command == ['gnome-screenshot', '--window', '--file', str(output)]
        output.data = png_header(*png_size) + bytes(1000)
    monkeypatch.setattr(checker.subprocess, 'check_output', read)
    monkeypatch.setattr(checker.subprocess, 'run', screenshot)
    child = SimpleNamespace(pid=1, poll=lambda: None)
    if expected_error:
        with pytest.raises(RuntimeError, match=expected_error):
            checker.capture_owned_window(child, output, {})
        assert output.data is None
    else:
        result = checker.capture_owned_window(child, output, {})
        assert result['active_window_verified_before_and_after']
        assert result['window_id'] == '0x10' and result['rviz_pid'] == 42
    screenshot_calls = [cmd for cmd in calls if cmd[0] == 'gnome-screenshot']
    assert len(screenshot_calls) == (0 if active_ids[0] != '0x10' else 1)


@pytest.fixture
def capture_directory():
    # Avoid host pytest temporary-directory ACL differences; clean only this
    # freshly created and verified test directory inside the shared workspace.
    parent = Path(__file__).absolute().parent
    directory = parent / ('.rviz_capture_test_' + uuid.uuid4().hex)
    directory.mkdir()
    try:
        yield directory
    finally:
        resolved = directory.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.rviz_capture_test_')
        shutil.rmtree(resolved)


@pytest.fixture
def independent_capture(monkeypatch, capture_directory):
    """Synthetic process/X11/PNG fixtures only; no GUI subprocess is launched."""
    import sys
    from types import ModuleType, SimpleNamespace
    tmp_path = capture_directory
    state = SimpleNamespace(active='0x10', owned=True, qt_error=None, gnome_error=None,
                            qt_after=None, gnome_after=None, ratio=1.0, size=(1280, 900), calls=[])
    owner = SimpleNamespace(pid=1, start_ticks='123')
    rviz = SimpleNamespace(pid=42, start_ticks='456')
    component = ModuleType('wc_runtime.component')
    component.process = lambda pid: owner
    component.descendants = lambda record: [rviz]
    component.belongs_to = lambda record, parent: state.owned
    monkeypatch.setitem(sys.modules, 'wc_runtime.component', component)
    properties = ('_NET_WM_PID(CARDINAL) = 42\n'
                  '_NET_WM_WINDOW_TYPE(ATOM) = _NET_WM_WINDOW_TYPE_NORMAL\n'
                  '_NET_WM_STATE(ATOM) =\nWM_STATE(WM_STATE):\n'
                  '    window state: Normal\nWM_TRANSIENT_FOR: not found.\n')
    geometry = ('  Absolute upper-left X: 100\n  Absolute upper-left Y: 20\n'
                '  Width: 1280\n  Height: 900\n  Map State: IsViewable\n')
    def read(command, **kwargs):
        if command == ['xprop', '-root', '_NET_CLIENT_LIST']:
            return '_NET_CLIENT_LIST(WINDOW): window id # 0x10'
        if command == ['xprop', '-root', '_NET_ACTIVE_WINDOW']:
            state.calls.append('active_check')
            return '_NET_ACTIVE_WINDOW(WINDOW): window id # ' + state.active
        if command == ['xwininfo', '-root', '-stats']:
            return '  Width: 1920\n  Height: 1080\n'
        if command[0] == 'xwininfo': return geometry
        if command[0] == 'xprop': return properties
        raise AssertionError(command)
    output = tmp_path/'view.png'
    def screenshot(command, **kwargs):
        if command[0] == 'gnome-screenshot':
            state.calls.append('gnome_capture')
            output.write_bytes(png_header(1280, 928) + bytes(1000))
            if state.gnome_after: state.gnome_after()
            if state.gnome_error: raise state.gnome_error
            return SimpleNamespace(returncode=0)
        assert command[:3] == [sys.executable, '-c', checker._INDEPENDENT_QT_CAPTURE]
        assert command[3] == '0x10'
        assert command[4] == str(tmp_path/'view_qt.png')
        assert kwargs['timeout'] == 10 and kwargs['check'] and kwargs['capture_output']
        state.calls.append('qt_capture')
        Path(command[4]).write_bytes(png_header(*state.size) + bytes(1000))
        if state.qt_after: state.qt_after()
        if state.qt_error: raise state.qt_error
        return SimpleNamespace(stdout=json.dumps(dict(pixel_width=state.size[0], pixel_height=state.size[1],
            device_pixel_ratio=state.ratio, qt_platform='xcb', screen_name='SYNTHETIC_FIXTURE', window_id='0x10')))
    monkeypatch.setattr(checker.subprocess, 'check_output', read)
    monkeypatch.setattr(checker.subprocess, 'run', screenshot)
    state.capture = lambda: checker.capture_owned_window(
        SimpleNamespace(pid=1, poll=lambda: None), output, {}, independent_qt=True)
    state.output = output
    state.qt = tmp_path/'view_qt.png'
    state.metadata = tmp_path/'view_qt.json'
    return state


@pytest.mark.parametrize('ratio,size', [(1.0, (1280, 900)), (2.0, (2560, 1800))])
def test_independent_qt_is_first_and_records_native_size_hash_identity(independent_capture, ratio, size):
    state = independent_capture
    state.ratio, state.size = ratio, size
    result = state.capture()
    record = json.loads(state.metadata.read_text())
    assert result['independent_qt'] == record
    assert record['status'] == 'PASS' and record['sha256'] == hashlib.sha256(state.qt.read_bytes()).hexdigest()
    assert record['pixel_width'] == size[0] and record['pixel_height'] == size[1]
    assert record['device_pixel_ratio'] == ratio
    assert record['owner_start_ticks'] == '123' and record['rviz_start_ticks'] == '456'
    assert record['window_verified_before'] and record['window_verified_after']
    assert record['active_window_verified_before_and_after']
    assert state.calls.index('qt_capture') < state.calls.index('gnome_capture')
    assert 'active_check' in state.calls[state.calls.index('qt_capture') + 1:state.calls.index('gnome_capture')]


def test_gnome_timeout_still_fails_and_keeps_verified_qt_sidecar(independent_capture):
    state = independent_capture
    state.gnome_error = subprocess.TimeoutExpired('gnome-screenshot', 15)
    with pytest.raises(subprocess.TimeoutExpired): state.capture()
    assert not state.output.exists()
    assert state.qt.exists() and json.loads(state.metadata.read_text())['status'] == 'PASS'


def test_gnome_wrong_active_window_still_fails_after_valid_qt(independent_capture):
    state = independent_capture
    state.gnome_after = lambda: setattr(state, 'active', '0x20')
    with pytest.raises(RuntimeError, match='main window is not active'): state.capture()
    assert not state.output.exists()
    assert state.qt.exists() and json.loads(state.metadata.read_text())['status'] == 'PASS'


@pytest.mark.parametrize('change', ['active', 'owned'])
def test_qt_capture_identity_change_is_fatal_and_never_runs_gnome(independent_capture, change):
    state = independent_capture
    state.qt_after = lambda: setattr(state, change, '0x20' if change == 'active' else False)
    with pytest.raises(RuntimeError): state.capture()
    record = json.loads(state.metadata.read_text())
    assert record['status'] == 'FAIL' and record['window_verified_before']
    assert not record.get('window_verified_after')
    assert not state.qt.exists() and 'gnome_capture' not in state.calls


@pytest.mark.parametrize('failure', ['timeout', 'dimensions'])
def test_optional_qt_tool_failure_is_recorded_without_replacing_gnome(independent_capture, failure):
    state = independent_capture
    if failure == 'timeout': state.qt_error = subprocess.TimeoutExpired('qt_capture', 10)
    else: state.size = (1920, 1080)
    result = state.capture()
    record = result['independent_qt']
    assert record['status'] == 'FAIL' and record['window_verified_after']
    assert not state.qt.exists() and state.output.exists()
    assert result['capture_method'] == 'gnome_verified_active_owned_normal_client'


@pytest.mark.parametrize('existing', ['qt', 'metadata'])
def test_independent_qt_never_overwrites_existing_evidence(independent_capture, existing):
    state = independent_capture
    path = getattr(state, existing)
    path.write_bytes(b'EXISTING_EVIDENCE')
    with pytest.raises(RuntimeError, match='evidence already exists'): state.capture()
    assert path.read_bytes() == b'EXISTING_EVIDENCE'
    assert 'qt_capture' not in state.calls and 'gnome_capture' not in state.calls
