"""Synthetic native-window fixtures; no GUI, ROS, subprocess or hardware runs."""
import importlib.util
import hashlib
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest


spec = importlib.util.spec_from_file_location(
    'qt_only_window_fixtures', Path(__file__).parents[1]/'integration/test_rviz_window_selection.py')
fixtures = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixtures)
capture_directory = fixtures.capture_directory
independent_capture = fixtures.independent_capture


def capture(state, **options):
    return fixtures.checker.capture_owned_window(
        SimpleNamespace(pid=1, poll=lambda: None), state.output, {}, qt_only=True, **options)


@pytest.mark.parametrize('independent', [False, True])
def test_qt_only_records_real_method_and_close_identity_without_gnome(independent_capture, independent):
    state = independent_capture
    state.gnome_error = AssertionError('GNOME must never run in explicit Qt-only mode')
    result = capture(state, independent_qt=independent)
    assert result['capture_mode'] == 'qt_only' and not result['gnome_capture_requested']
    assert result['capture_method'] == 'independent_pyqt5_qscreen_grabWindow_owned_x11_client'
    assert result['status'] == 'PASS' and result['screenshot'] == str(state.qt)
    assert result['sha256'] == hashlib.sha256(state.qt.read_bytes()).hexdigest()
    assert result['pixel_width'] == 1280 and result['pixel_height'] == 900
    assert result['window_id'] == '0x10' and result['rviz_pid'] == 42
    assert result['owner_start_ticks'] == '123' and result['rviz_start_ticks'] == '456'
    assert result['window_verified_before'] and result['window_verified_after']
    assert state.calls.count('qt_capture') == 1 and 'gnome_capture' not in state.calls
    assert not state.output.exists()


@pytest.mark.parametrize('failure', ['timeout', 'dimensions', 'active', 'owner'])
def test_qt_only_failure_raises_without_fallback_or_unattributed_image(independent_capture, failure):
    state = independent_capture
    if failure == 'timeout': state.qt_error = subprocess.TimeoutExpired('synthetic Qt capture', 10)
    elif failure == 'dimensions': state.size = (1920, 1080)
    elif failure == 'active': state.qt_after = lambda: setattr(state, 'active', '0x20')
    else: state.qt_after = lambda: setattr(state, 'owned', False)
    with pytest.raises(RuntimeError): capture(state)
    assert json.loads(state.metadata.read_text())['status'] == 'FAIL'
    assert not state.qt.exists() and not state.output.exists()
    assert 'gnome_capture' not in state.calls


@pytest.mark.parametrize('existing', ['qt', 'metadata'])
def test_qt_only_refuses_to_overwrite_evidence(independent_capture, existing):
    state = independent_capture
    path = getattr(state, existing)
    path.write_bytes(b'SYNTHETIC_EXISTING_EVIDENCE')
    with pytest.raises(RuntimeError, match='evidence already exists'): capture(state)
    assert path.read_bytes() == b'SYNTHETIC_EXISTING_EVIDENCE'
    assert 'qt_capture' not in state.calls and 'gnome_capture' not in state.calls
