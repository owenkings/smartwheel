"""The profile dialog requires a fresh preview and explicit confirmation."""
import os
import time
from unittest import mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import pytest
from PyQt5 import QtCore, QtTest, QtWidgets

from wc_panel.ui_common import apply_theme, BackgroundTask
from wc_panel.ui_profiles import ProfilesDialog


@pytest.fixture(scope='session')
def app():
    instance = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    apply_theme(instance)
    yield instance
    instance.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    instance.processEvents()


def drain(predicate):
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        QtWidgets.QApplication.processEvents()
        if predicate():
            QtTest.QTest.qWait(10)
            return
        QtTest.QTest.qWait(10)
    assert predicate()


def backend():
    result = mock.Mock()
    result.list_configuration_profiles.return_value = dict(revision='r1', active_name='日常',
        config_paths=['/home/nvidia/wheelchair/config/hardware_setup.json', '/home/nvidia/wheelchair/config/mapping_live.json'],
        profile_root='/home/nvidia/wheelchair/config/panel_profiles', profiles=[
            dict(id='valid', name='室内', path='/config/indoor', available=True, error=''),
            dict(id='invalid', name='依据缺失', path='/config/old', available=False, error='来源文件缺失')])
    result.preview_configuration_profile.return_value = dict(token='preview-token', name='室内',
        changes=[dict(group='wheels', label='轮参数', before={'wheel_radius_m': .185}, after={'wheel_radius_m': .19})],
        message='对后续任务生效；原配置备份，录包不变。')
    result.apply_configuration_profile.return_value = dict(name='室内', backup='/backups/test')
    return result


def close(widget):
    drain(lambda: not BackgroundTask.live_tasks)
    widget.close()
    QtWidgets.QApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    QtWidgets.QApplication.processEvents()


def test_paths_visible_and_selection_requires_preview(app):
    service = backend()
    widget = ProfilesDialog(service)
    widget.show()
    drain(lambda: not widget._busy)
    assert 'hardware_setup.json' in widget.paths.text()
    assert widget.preview_button.isEnabled()
    assert not widget.apply_button.isEnabled()
    widget.preview_button.click()
    drain(lambda: widget.proposal is not None)
    assert widget.apply_button.isEnabled()
    assert '.185' in widget.preview_text.toPlainText() and '.19' in widget.preview_text.toPlainText()
    assert '185 mm → 190 mm' in widget.change_summary.text()
    assert widget.preview_text.isHidden()
    widget.details_button.click()
    assert not widget.preview_text.isHidden()
    widget.profiles.setCurrentIndex(1)
    assert not widget.apply_button.isEnabled()
    assert not widget.preview_button.isEnabled()
    assert '来源文件缺失' in widget.selection_detail.text()
    service.apply_configuration_profile.assert_not_called()
    close(widget)


def test_preview_summary_scrolls_into_view_after_layout_reflow(app):
    service=backend()
    widget=ProfilesDialog(service);widget.show()
    drain(lambda:not widget._busy)
    widget.preview()
    drain(lambda:widget.proposal is not None)
    QtTest.QTest.qWait(100)
    viewport=widget.body_scroll.viewport()
    point=widget.change_summary.mapTo(viewport,QtCore.QPoint(0,widget.change_summary.height()))
    assert 0<point.y()<=viewport.height()
    close(widget)


def test_cancel_confirmation_does_not_apply_and_success_emits_refresh(app):
    service = backend()
    widget = ProfilesDialog(service)
    drain(lambda: not widget._busy)
    widget.preview()
    drain(lambda: widget.proposal is not None)
    with mock.patch.object(QtWidgets.QMessageBox, 'warning', return_value=QtWidgets.QMessageBox.Cancel):
        widget.apply()
    service.apply_configuration_profile.assert_not_called()
    changed = []
    widget.parametersChanged.connect(lambda: changed.append(True))
    with mock.patch.object(QtWidgets.QMessageBox, 'warning', return_value=QtWidgets.QMessageBox.Ok), \
         mock.patch.object(QtWidgets.QMessageBox, 'information'):
        widget.apply()
        drain(lambda: bool(changed))
    service.apply_configuration_profile.assert_called_once_with('preview-token')
    close(widget)


def test_empty_name_cannot_save_and_save_uses_displayed_revision(app):
    service = backend()
    widget = ProfilesDialog(service)
    drain(lambda: not widget._busy)
    widget.save_current()
    service.save_configuration_profile.assert_not_called()
    widget.name.setText('  轮尺寸复测  ')
    widget.save_current()
    drain(lambda: service.save_configuration_profile.called)
    service.save_configuration_profile.assert_called_once_with('轮尺寸复测', 'r1')
    close(widget)
