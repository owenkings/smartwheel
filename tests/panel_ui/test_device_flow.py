"""Nonmodal device windows must not relabel an already running pairing server."""
import os
import threading
import time
from unittest import mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtCore, QtTest, QtWidgets
import pytest

from wc_panel.ui_common import apply_theme
from wc_panel.ui_device import PairingDialog


@pytest.fixture(scope='session')
def app():
    instance = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    apply_theme(instance)
    yield instance
    instance.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    instance.processEvents()


def drain(predicate):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        QtWidgets.QApplication.processEvents()
        if predicate():
            return
        QtTest.QTest.qWait(10)
    assert predicate()


@pytest.fixture
def panel(app):
    backend = mock.Mock()
    backend.calibration_defaults.return_value = dict(input='/fixture/original/prepared.json')
    backend.calibration_status.return_value = None
    backend.settings.return_value = dict(recording_root='/fixture/recordings', results_root='/fixture/results')
    dialog = PairingDialog(backend)
    dialog.show()
    drain(lambda: dialog.start_button.isEnabled())
    dialog.timer.stop()
    yield dialog, backend
    dialog._busy = False
    dialog.show_task(dict(id='fixture', status='COMPLETE', input=dialog.input.text(), url=''))
    dialog.close()
    QtWidgets.QApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    QtWidgets.QApplication.processEvents()


def test_visible_capture_child_blocks_starting_parent_pairing_service(panel):
    dialog, backend = panel
    dialog.open_capture()
    child = dialog._capture_dialog
    drain(lambda: '尚未录制' in child.status_label.text())
    assert child.isVisible()
    dialog.start()
    backend.start_calibration.assert_not_called()
    assert '录制窗口' in dialog.status_label.text()
    child.close()


@pytest.mark.parametrize('status,process_alive', (('STARTING', False), ('RUNNING', False),
                                               ('STOPPING', False), ('FAILED', True)))
def test_prepared_child_cannot_replace_running_parent_input(panel, status, process_alive):
    dialog, backend = panel
    dialog.show_task(dict(id='active-tool', status=status, process_alive=process_alive,
        input='/fixture/server-input/prepared.json', mode='points', url=''))
    with mock.patch.object(dialog, 'show_error') as error:
        dialog.capture_prepared('/fixture/new-capture/prepared.json')
    assert dialog.input.text() == '/fixture/server-input/prepared.json'
    assert dialog.task['input'] == '/fixture/server-input/prepared.json'
    assert not dialog.capture_button.isEnabled()
    assert '停止工具' in error.call_args.args[0]


def test_prepared_file_can_be_selected_after_parent_tool_finishes(panel):
    dialog, _ = panel
    dialog.show_task(dict(id='finished', status='COMPLETE', input='/fixture/original/prepared.json', url=''))
    dialog.capture_prepared('/fixture/next/prepared.json')
    assert dialog.input.text() == '/fixture/next/prepared.json'
    assert dialog.start_button.isEnabled() and dialog.capture_button.isEnabled()


def test_pairing_start_disables_capture_entry_before_background_launch_returns(panel):
    dialog, backend = panel
    launched, release = threading.Event(), threading.Event()

    def start(source, mode):
        launched.set()
        assert release.wait(3)
        return dict(id='new-tool', status='FAILED', input=source, mode=mode, error='synthetic offline test', url='')

    backend.start_calibration.side_effect = start
    try:
        dialog.start()
        drain(launched.is_set)
        assert dialog._busy and not dialog.capture_button.isEnabled()
        with mock.patch.object(dialog, 'show_error') as error:
            dialog.capture_prepared('/fixture/late-child/prepared.json')
        assert dialog.input.text() == '/fixture/original/prepared.json' and error.called
    finally:
        release.set()
        drain(lambda: not dialog._busy)
    assert dialog.capture_button.isEnabled()
