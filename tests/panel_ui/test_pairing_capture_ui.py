"""Static capture is explicit; only fully finalized recordings can be prepared."""
import os
import threading
import time
from unittest import mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtCore, QtTest, QtWidgets, sip
import pytest

from wc_panel.ui_common import apply_theme, BackgroundTask
from wc_panel.ui_pairing_capture import PairingCaptureDialog


@pytest.fixture(scope='session')
def app():
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    apply_theme(application)
    yield application
    application.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    application.processEvents()


def drain(predicate=lambda: not BackgroundTask.live_tasks):
    until = time.monotonic() + 4
    while time.monotonic() < until:
        QtWidgets.QApplication.processEvents()
        if predicate():
            QtTest.QTest.qWait(10)
            return
        QtTest.QTest.qWait(10)
    assert predicate()


@pytest.fixture
def setup_dialog(app):
    backend = mock.Mock()
    backend.settings.return_value = dict(recording_root='/fixture/recordings')
    backend.start_capture.return_value = dict(id='ours', status='RUNNING', output='/fixture/recordings/calibration_2026_1008_120000')
    backend.cancel.return_value = dict(id='ours', status='STOPPING', output='/fixture/recordings/calibration_2026_1008_120000')
    backend.prepare_pairing_capture.return_value = dict(status='READY_FOR_OFFLINE_PICKING',
        prepared_input='/fixture/reports/new/prepared.json', organized_amplitude_available=True)
    dialog = PairingCaptureDialog(backend)
    dialog.timer.stop()
    drain()
    yield dialog, backend
    drain()
    if not sip.isdeleted(dialog):
        dialog._busy = False
        dialog.task = None
        dialog._cleanup['identifier'] = None
        dialog.close()
    QtWidgets.QApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    QtWidgets.QApplication.processEvents()


def test_requires_explicit_stationary_choice_before_acquisition(setup_dialog):
    dialog, backend = setup_dialog
    assert dialog.root.path() == '/fixture/recordings'
    assert not dialog.start_button.isEnabled()
    dialog.start()
    backend.start_capture.assert_not_called()
    dialog.static.setChecked(True)
    assert dialog.start_button.isEnabled()
    dialog.name.setText('doorway')
    dialog.start_button.click()
    drain()
    backend.start_capture.assert_called_once_with('/fixture/recordings', 'all', manual_drive=False,
        name_options=dict(prefix='doorway', year=True, date=True, time=True))
    assert dialog.task['id'] == 'ours'
    assert not dialog.prepare_button.isEnabled()


def test_ready_result_remains_visible_after_wrapped_layout_reflows(setup_dialog):
    dialog,backend=setup_dialog
    dialog.show();drain()
    result=dict(backend.prepare_pairing_capture.return_value,
                prepared_input='/media/recordings/'+'long_source_name_'*8+'/prepared.json')
    dialog.ready(result)
    QtTest.QTest.qWait(100)
    view=dialog.body_scroll.viewport()
    bottom=dialog.detail.mapTo(view,QtCore.QPoint(0,dialog.detail.height())).y()
    assert 0<bottom<=view.height()
    assert dialog.use_button.isEnabled()


def test_stop_waits_for_complete_and_uses_exact_new_capture(setup_dialog):
    dialog, backend = setup_dialog
    dialog.show_task(backend.start_capture.return_value)
    dialog.stop_and_prepare()
    drain()
    backend.cancel.assert_called_once_with('ours')
    backend.prepare_pairing_capture.assert_not_called()
    assert dialog.task['status'] == 'STOPPING'
    complete = dict(backend.start_capture.return_value, status='COMPLETE')
    dialog.show_task(complete)
    drain()
    backend.prepare_pairing_capture.assert_called_once_with(complete['output'])
    assert dialog.use_button.isEnabled()
    assert '幅度图' in dialog.detail.text()
    selected = []
    dialog.prepared.connect(selected.append)
    assert selected == []
    dialog.use_button.click()
    assert selected == ['/fixture/reports/new/prepared.json']


@pytest.mark.parametrize('state', ['FAILED', 'INTERRUPTED', 'CANCELLED'])
def test_failed_capture_keeps_data_and_does_not_prepare(setup_dialog, state):
    dialog, backend = setup_dialog
    dialog.show_task(backend.start_capture.return_value)
    dialog._prepare_after_stop = True
    dialog.show_task(dict(backend.start_capture.return_value, status=state, error='partial data retained'))
    drain()
    backend.prepare_pairing_capture.assert_not_called()
    assert '原始数据保留' in dialog.detail.text()
    assert dialog.dataset.path() == backend.start_capture.return_value['output']


def test_existing_recording_without_layout_explains_xyz_only(setup_dialog):
    dialog, backend = setup_dialog
    backend.prepare_pairing_capture.return_value['organized_amplitude_available'] = False
    dialog.dataset.set_path('/fixture/old/complete')
    dialog.prepare_existing()
    drain()
    backend.prepare_pairing_capture.assert_called_once_with('/fixture/old/complete')
    backend.start_capture.assert_not_called()
    assert '仅提供真实 XYZ' in dialog.detail.text()


def test_close_requests_owned_stop_and_waits_for_finalization(setup_dialog):
    dialog, backend = setup_dialog
    dialog.show_task(backend.start_capture.return_value)
    dialog.show()
    dialog.close()
    drain()
    assert not sip.isdeleted(dialog)
    assert dialog._closing
    backend.cancel.assert_called_once_with('ours')
    backend.prepare_pairing_capture.assert_not_called()
    dialog.show_task(dict(backend.start_capture.return_value, status='COMPLETE'))
    drain()
    QtWidgets.QApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    assert sip.isdeleted(dialog)


def test_prepare_failure_leaves_source_and_error_visible(setup_dialog):
    dialog, backend = setup_dialog
    backend.prepare_pairing_capture.side_effect = ValueError('PARTIAL data cannot be prepared')
    dialog.dataset.set_path('/fixture/partial')
    dialog.prepare_existing()
    drain()
    assert 'PARTIAL' in dialog.status_label.text()
    assert not dialog.use_button.isEnabled()
    assert dialog.prepare_button.isEnabled()
    assert dialog.dataset.path() == '/fixture/partial'


def test_stop_clicked_during_poll_is_dispatched_after_poll(setup_dialog):
    dialog, backend = setup_dialog
    row = backend.start_capture.return_value
    dialog.show_task(row)
    release = threading.Event()
    backend.jobs.side_effect = lambda: (release.wait(2), [row])[1]
    dialog.poll()
    assert dialog._busy
    dialog.stop_and_prepare()
    assert dialog._stop_pending
    backend.cancel.assert_not_called()
    release.set()
    drain()
    backend.cancel.assert_called_once_with('ours')
    assert dialog.task['status'] == 'STOPPING'
    assert not dialog._stop_pending
