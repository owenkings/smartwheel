"""Recording name/suffix controls and compact same-row sensor selection."""
from datetime import datetime
import os
import time
from types import SimpleNamespace
from unittest import mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtCore, QtTest, QtWidgets, sip
import pytest

from wc_panel.ui_common import apply_theme, BackgroundTask
from wc_panel.ui_dialogs import CaptureDialog


@pytest.fixture(scope='session')
def app():
    instance = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    apply_theme(instance)
    yield instance
    instance.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    instance.processEvents()


def drain():
    until = time.monotonic() + 4
    while time.monotonic() < until:
        QtWidgets.QApplication.processEvents()
        if not BackgroundTask.live_tasks:
            QtTest.QTest.qWait(10)
            return
        QtTest.QTest.qWait(10)
    assert not BackgroundTask.live_tasks


@pytest.fixture
def capture_dialog(app, monkeypatch):
    monkeypatch.setattr('wc_runtime.capture_names.datetime',
                        SimpleNamespace(now=lambda: datetime(2026, 10, 8, 9, 7, 5)))
    backend = mock.Mock()
    backend.settings.return_value = dict(recording_root='/fixture/recordings')
    backend.start_capture.return_value = dict(id='queued-fixture', status='QUEUED')
    dialog = CaptureDialog(backend)
    dialog.name_timer.stop()
    dialog.show()
    drain()
    yield dialog, backend
    drain()
    if not sip.isdeleted(dialog):
        dialog.close()
    QtWidgets.QApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    QtWidgets.QApplication.processEvents()


def test_name_prefix_on_left_and_short_lidar_selector_on_right(capture_dialog):
    dialog, _ = capture_dialog
    left = dialog.prefix.mapTo(dialog, QtCore.QPoint(0, 0))
    right = dialog.sides.mapTo(dialog, QtCore.QPoint(0, 0))
    assert left.x() < right.x() and abs(left.y() - right.y()) <= 3
    assert dialog.sides.width() <= 240
    assert dialog.prefix.width() > dialog.sides.width()


def test_chinese_preview_uses_selected_suffixes_in_order(capture_dialog):
    dialog, _ = capture_dialog
    dialog.prefix.setText('走廊_往返')
    assert dialog.name_preview.text() == '文件夹预览：走廊_往返_2026_1008_090705'
    dialog.suffixes['year'].setChecked(False)
    assert dialog.name_preview.text() == '文件夹预览：走廊_往返_1008_090705'
    dialog.suffixes['time'].setChecked(False)
    assert dialog.name_preview.text() == '文件夹预览：走廊_往返_1008'


def test_empty_or_unsafe_name_blocks_start(capture_dialog):
    dialog, backend = capture_dialog
    for checkbox in dialog.suffixes.values():
        checkbox.setChecked(False)
    dialog.prefix.clear()
    assert not dialog.start_button.isEnabled()
    assert '前缀' in dialog.name_preview.text()
    dialog.start_button.click()
    backend.start_capture.assert_not_called()
    dialog.prefix.setText('../其他文件夹')
    assert not dialog.start_button.isEnabled()
    dialog.prefix.setText('场景一')
    assert dialog.start_button.isEnabled()


def test_start_passes_source_root_lidar_and_exact_naming_preferences(capture_dialog):
    dialog, backend = capture_dialog
    dialog.prefix.setText('办公室')
    dialog.suffixes['year'].setChecked(False)
    dialog.sides.setCurrentIndex(dialog.sides.findData('right'))
    dialog.start_button.click()
    drain()
    backend.start_capture.assert_called_once_with(output_root='/fixture/recordings', sides='right',
        name_options=dict(prefix='办公室', year=False, date=True, time=True))


def test_changing_defaults_preserves_explicit_temporary_destination(capture_dialog):
    dialog, _ = capture_dialog
    dialog.destination.set_path('/fixture/temporary-recordings')
    dialog.update_default_settings(dict(recording_root='/fixture/new-default'))
    assert dialog.destination.path() == '/fixture/temporary-recordings'
