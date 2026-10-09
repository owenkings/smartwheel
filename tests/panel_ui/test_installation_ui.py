"""Editable principal measurements preserve runtime source/revision safeguards."""
import copy
import os
import time
from unittest import mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtCore, QtTest, QtWidgets
import pytest

from wc_panel.installation_parameters import rpy_matrix
from wc_panel.ui_common import apply_theme
from wc_panel.ui_installation import InstallationDialog, PrincipalComponent


@pytest.fixture(scope='session')
def app():
    instance = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    apply_theme(instance)
    yield instance
    instance.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    instance.processEvents()


def transform(parent, child, position, angles):
    return dict(id=parent + '_from_' + child, parent_frame=parent, child_frame=child,
        direction='child_to_parent', unit='m', note='fixture',
        translation=dict(value_m=position, status='UNKNOWN' if position is None else 'USER_MEASURED_EXPERIMENT',
            reference_kind='FRAME_ORIGIN', evidence=[dict(source_id='measurement')], note='fixture origin'),
        rotation=dict(value_matrix=None if angles is None else rpy_matrix(angles),
            status='UNKNOWN' if angles is None else 'USER_MEASURED_EXPERIMENT', reference_kind='DATA_AXES',
            evidence=[dict(source_id='measurement')], note='fixture axes'))


def snapshot():
    return dict(revision='original', groups=[dict(id='extrinsics', assembly_revision='v1',
        source_documents=[dict(id='measurement', evidence_type='GEOMETRY_MEASUREMENT', assembly_revision='v1')],
        value=[transform('axle', 'mounting_M', [.1, .2, .3], [0, 0, 0]),
               transform('mounting_M', 'lidar_left', None, None),
               transform('mounting_M', 'lidar_right', None, None),
               transform('axle', 'imu_native', None, [1.123456789, 2.456789123, 3.987654321]),
               transform('mounting_M', 'camera_left_front_optical', None, None)])])


def drain(predicate):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        QtWidgets.QApplication.processEvents()
        if predicate():
            return
        QtTest.QTest.qWait(10)
    assert predicate()


def test_unknown_position_and_angles_start_blank_and_stay_explicit(app):
    dialog = InstallationDialog(mock.Mock(), snapshot())
    component = dialog.editors[1]['translation']
    assert all(field.text() == '' and not field.isEnabled() for field in component.fields)
    dialog.authorized(dict(token='warning', revision='original'))
    component.status.setCurrentIndex(component.status.findData('USER_MEASURED_EXPERIMENT'))
    assert all(field.text() == '' and field.isEnabled() for field in component.fields)
    assert '补齐输入' in dialog.preview.text()
    with pytest.raises(ValueError, match='完整填写'):
        dialog.proposed(preview=True)
    component.status.setCurrentIndex(component.status.findData('UNKNOWN'))
    assert dialog.proposed()[1]['translation']['value_m'] is None
    dialog.close()


def test_untouched_rotation_matrix_survives_even_when_only_note_changes(app):
    dialog = InstallationDialog(mock.Mock(), snapshot())
    original = copy.deepcopy(dialog.original)
    assert dialog.proposed() == original
    editor = dialog.editors[3]['rotation']
    editor.note.setText('updated provenance description')
    proposed = dialog.proposed()
    assert proposed[3]['rotation']['value_matrix'] == original[3]['rotation']['value_matrix']
    assert proposed[4] == original[4]
    dialog.close()


def test_named_actual_reference_parent_and_imu_are_visible(app):
    dialog = InstallationDialog(mock.Mock(), snapshot())
    labels = [dialog.selector.itemText(i) for i in range(dialog.selector.count())]
    assert len(labels) == 4
    assert 'IMU 数据原点 · 相对轮轴中心' in labels
    assert 'IMU 数据原点 → 轮轴中心' in dialog.preview.text()
    assert '朝向 1.123 / 2.457 / 3.988' in dialog.preview.text()
    dialog.close()


def test_stale_revision_never_enables_edits(app):
    dialog = InstallationDialog(mock.Mock(), snapshot())
    dialog.authorized(dict(token='warning', revision='stale'))
    assert dialog.permission is None and not dialog.save_button.isEnabled()
    assert '配置已改变' in dialog.state.text()
    dialog.close()


def test_partial_input_cannot_be_saved_as_zero(app):
    backend = mock.Mock()
    dialog = InstallationDialog(backend, snapshot())
    dialog.authorized(dict(token='warning', revision='original'))
    editor = dialog.editors[1]['translation']
    editor.status.setCurrentIndex(editor.status.findData('LEGACY_CANDIDATE'))
    editor.fields[0].setText('125')
    dialog.save()
    assert not backend.save_parameters.called
    assert '完整填写' in dialog.state.text()
    dialog.close()


def test_save_converts_mm_preserves_other_relations_and_bound_tokens(app):
    backend = mock.Mock()
    backend.save_parameters.return_value = dict(backup='/fixture/backup')
    dialog = InstallationDialog(backend, snapshot())
    dialog.authorized(dict(token='warning', revision='original'))
    editor = dialog.editors[1]['translation']
    editor.status.setCurrentIndex(editor.status.findData('USER_MEASURED_EXPERIMENT'))
    for field, text in zip(editor.fields, ('50', '228.75', '133.3')):
        field.setText(text)
    source = dict(id='new', evidence_type='GEOMETRY_MEASUREMENT', assembly_revision='v1')
    dialog.evidence_ready(editor, dict(revision='original', source=source, token='bound-source'), 'new origin measurement')
    calls = []
    dialog.saved.connect(lambda: calls.append('saved'))
    with mock.patch.object(QtWidgets.QMessageBox, 'information'):
        dialog.save()
        drain(lambda: bool(calls))
    args, kwargs = backend.save_parameters.call_args
    assert args[0] == 'extrinsics'
    assert args[1][1]['translation']['value_m'] == [.05, .22875, .1333]
    assert args[1][1]['rotation']['value_matrix'] is None
    assert args[1][3:] == snapshot()['groups'][0]['value'][3:]
    assert kwargs == dict(warning_token='warning', expected_revision='original', evidence_tokens=['bound-source'])


def test_calibrated_claim_rejects_measurement_only_source(app):
    dialog = InstallationDialog(mock.Mock(), snapshot())
    editor = dialog.editors[0]['translation']
    editor.status.setCurrentIndex(editor.status.findData('CALIBRATED'))
    with pytest.raises(ValueError, match='外参标定记录'):
        dialog.proposed()
    dialog.close()


def test_form_keeps_advanced_details_collapsed_until_requested(app):
    dialog = InstallationDialog(mock.Mock(), snapshot())
    editor = dialog.editors[0]['translation']
    assert not editor.evidence_toggle.isChecked()
    editor.evidence_toggle.click()
    assert editor.evidence_toggle.isChecked() and not editor.evidence_box.isHidden()
    selected = []
    dialog.advancedRequested.connect(selected.append)
    dialog.advanced_button.click()
    assert selected == ['extrinsics']
    dialog.close()
