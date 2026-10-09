"""Plain-language dimensions, preserved edits and owned-tool UI lifecycle."""
import copy
import os
import time
from unittest import mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtCore, QtTest, QtWidgets
import pytest

from wc_panel.ui_common import apply_theme
from wc_panel.ui_device import DeviceOverview, WheelDimensionsDialog, PairingDialog, overview_values


@pytest.fixture(scope='session')
def app():
    # A process must keep one QApplication alive across the pytest functions
    # and the unittest suites. Recreating it leaves stale native QScreen state.
    instance = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    apply_theme(instance)
    yield instance
    instance.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    instance.processEvents()


def snapshot():
    return dict(revision='original', groups=[
        dict(id='wheels', status='UNVALIDATED', value=dict(wheel_radius_m=.185, track_width_m=.61,
            left_sign=-1, right_sign=1, register_to_wheel_rpm=.02, max_gap_s=.2, max_wheel_speed_m_s=1)),
        dict(id='extrinsics', value=[]), dict(id='display', value={})])


def transform(parent, child, vector, reference='FRAME_ORIGIN', status='CALIBRATED'):
    return dict(parent_frame=parent, child_frame=child, direction='child_to_parent', unit='m',
                translation=dict(value_m=vector, reference_kind=reference, status=status))


def drain(predicate=lambda: True):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        QtWidgets.QApplication.processEvents()
        if predicate():
            QtTest.QTest.qWait(10)
            return
        QtTest.QTest.qWait(10)
    assert predicate()


def test_summary_does_not_present_cad_face_as_lidar_origin():
    data = snapshot()
    data['groups'][1]['value'] = [transform('mounting_M', 'lidar_left', [0, .22875, .1333], 'CAD_OUTWARD_FACE_CENTER'),
                                transform('mounting_M', 'lidar_right', [0, -.22875, .1333], 'CAD_OUTWARD_FACE_CENTER')]
    values = overview_values(data)
    assert values['spacing']['value'] == '未测量'
    assert values['left_height']['value'] == '未测量'
    assert '未核验' in values['radius']['detail']


def test_same_parent_origin_distance_is_not_ground_height():
    data = snapshot()
    data['groups'][1]['value'] = [transform('mounting_M', 'lidar_left', [0, .3, .4]),
                                transform('mounting_M', 'lidar_right', [0, -.3, .4])]
    values = overview_values(data)
    assert values['spacing']['value'] == '600 mm'
    assert values['left_height']['value'] == '未测量'


def test_unknown_with_numeric_placeholder_stays_unknown():
    data = snapshot()
    data['groups'][1]['value'] = [transform('lidar_left', 'lidar_right', [1, 2, 3], status='UNKNOWN')]
    assert overview_values(data)['spacing']['value'] == '未测量'


def test_ground_relative_origin_has_explicit_reference():
    data = snapshot()
    data['groups'][1]['value'] = [transform('ground', 'lidar_left', [0, 0, .8])]
    row = overview_values(data)['left_height']
    assert row['value'] == '800 mm' and 'ground' in row['detail']


def test_overview_emits_named_advanced_group(app):
    backend = mock.Mock()
    widget = DeviceOverview(backend)
    widget.refresh(snapshot())
    selected = []
    widget.advancedRequested.connect(selected.append)
    widget.display_button.click()
    assert selected == ['display']
    assert widget.cards['radius'][1].text() == '185 mm'
    widget.close()


def test_wheel_quick_save_preserves_other_parameters_and_revision(app):
    backend = mock.Mock()
    backend.save_parameters.return_value = dict(backup='/fixture/backup')
    dialog = WheelDimensionsDialog(backend, snapshot())
    dialog.authorized(dict(token='accepted-warning', revision='original'))
    dialog.fields['wheel_radius_m'].setValue(190)
    calls = []
    dialog.saved.connect(lambda: calls.append('saved'))
    with mock.patch.object(QtWidgets.QMessageBox, 'information'):
        dialog.save()
        drain(lambda: bool(calls))
    args, kwargs = backend.save_parameters.call_args
    assert args[0] == 'wheels' and args[1]['wheel_radius_m'] == .19
    assert args[1]['track_width_m'] == .61 and args[1]['left_sign'] == -1 and args[1]['register_to_wheel_rpm'] == .02
    assert kwargs == dict(warning_token='accepted-warning', expected_revision='original')


def test_wheel_edit_rejects_stale_snapshot(app):
    dialog = WheelDimensionsDialog(mock.Mock(), snapshot())
    dialog.authorized(dict(token='new', revision='changed-elsewhere'))
    assert not dialog.save_button.isEnabled() and dialog.permission is None
    assert '已改变' in dialog.state.text()
    dialog.close()


def test_unknown_wheel_field_never_becomes_a_fake_size(app):
    data = snapshot()
    data['groups'][0]['value']['wheel_radius_m'] = None
    backend = mock.Mock()
    dialog = WheelDimensionsDialog(backend, data)
    assert '未知' in dialog.fields['wheel_radius_m'].text()
    dialog.authorized(dict(token='confirmed', revision='original'))
    dialog.save()
    assert not backend.save_parameters.called
    assert '未知项不能按零保存' in dialog.state.text()
    dialog.close()


def test_pairing_close_only_stops_known_owned_task(app):
    backend = mock.Mock()
    backend.calibration_defaults.return_value = dict(input='/fixture/prepared.json')
    row = dict(id='owned', status='RUNNING', mode='points', input='/fixture/prepared.json',
               url='http://127.0.0.1:12345/', log_path='/fixture/log', output_root='/fixture/output')
    backend.calibration_status.return_value = row
    backend.stop_calibration.return_value = dict(row, status='COMPLETE', url='')
    dialog = PairingDialog(backend)
    dialog.show()
    drain(lambda: dialog.task is not None)
    assert '历史默认' in dialog.input_detail.text()
    dialog.close()
    drain(lambda: backend.stop_calibration.called)
    assert backend.stop_calibration.call_args.args == ('owned',)
    drain()


def test_pairing_failure_cannot_open_browser(app):
    backend = mock.Mock()
    backend.calibration_defaults.return_value = dict(error='未找到默认输入')
    backend.calibration_status.return_value = None
    dialog = PairingDialog(backend)
    drain(lambda: dialog.start_button.isEnabled())
    dialog.show_task(dict(id='failed', status='FAILED', error='源文件不可用', url=''))
    assert not dialog.open_button.isEnabled() and '源文件不可用' in dialog.status_label.text()
    dialog.close()


def test_pairing_parent_destruction_stops_its_tool(app):
    backend = mock.Mock()
    backend.calibration_defaults.return_value = dict(input='/fixture/prepared.json')
    backend.calibration_status.return_value = dict(id='owned-child', status='RUNNING', mode='points', url='')
    parent = QtWidgets.QWidget()
    dialog = PairingDialog(backend, parent)
    drain(lambda: dialog.task is not None)
    from PyQt5 import sip
    sip.delete(parent)
    drain(lambda: backend.stop_calibration.called)
    assert backend.stop_calibration.call_args.args == ('owned-child',)
