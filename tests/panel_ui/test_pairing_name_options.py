"""Pairing capture exposes naming choices without renaming active recordings."""
from datetime import datetime
from types import SimpleNamespace

import pytest

from test_pairing_capture_ui import app, setup_dialog, drain


def freeze_clock(monkeypatch, instant):
    monkeypatch.setattr('wc_runtime.capture_names.datetime',
                        SimpleNamespace(now=lambda: instant))


def test_selected_suffixes_preview_and_reach_capture_backend(setup_dialog, monkeypatch):
    dialog, backend = setup_dialog
    freeze_clock(monkeypatch, datetime(2026, 10, 8, 9, 7, 5))
    assert all(checkbox.isChecked() for checkbox in dialog.suffixes.values())
    dialog.name.setText('门口_标定')
    assert '门口_标定_2026_1008_090705' in dialog.name_preview.text()
    dialog.suffixes['year'].setChecked(False)
    dialog.suffixes['date'].setChecked(False)
    assert '门口_标定_090705' in dialog.name_preview.text()
    dialog.static.setChecked(True)
    dialog.start_button.click()
    drain()
    backend.start_capture.assert_called_once_with('/fixture/recordings', 'all', manual_drive=False,
        name_options=dict(prefix='门口_标定', year=False, date=False, time=True))


@pytest.mark.parametrize('prefix', ['', '../其他目录'])
def test_invalid_names_block_button_and_direct_start(setup_dialog, prefix):
    dialog, backend = setup_dialog
    for checkbox in dialog.suffixes.values():
        checkbox.setChecked(False)
    dialog.name.setText(prefix)
    dialog.static.setChecked(True)
    assert not dialog.start_button.isEnabled()
    dialog.start()
    drain()
    backend.start_capture.assert_not_called()
    assert not dialog.active()
    dialog.name.setText('仅名称')
    assert dialog.start_button.isEnabled()


def test_active_name_is_actual_collision_resolved_output_and_stays_fixed(setup_dialog, monkeypatch):
    dialog, backend = setup_dialog
    row = dict(backend.start_capture.return_value,
               output='/fixture/recordings/标定_2026_1008_090705_02')
    dialog.show_task(row)
    assert not dialog.name.isEnabled()
    assert all(not checkbox.isEnabled() for checkbox in dialog.suffixes.values())
    assert '标定_2026_1008_090705_02' in dialog.name_preview.text()
    preview = dialog.name_preview.text()
    freeze_clock(monkeypatch, datetime(2026, 10, 9, 0, 0, 1))
    assert dialog.name_timer.interval() == 1000
    dialog.name_timer.timeout.emit()
    assert dialog.name_preview.text() == preview
    assert dialog.task['output'] == row['output']
    assert dialog.dataset.path() == row['output']
    backend.start_capture.assert_not_called()


def test_failed_capture_retains_original_location_when_next_name_changes(setup_dialog, monkeypatch):
    dialog, backend = setup_dialog
    row = dict(backend.start_capture.return_value, status='FAILED', error='partial retained')
    dialog.show_task(row)
    freeze_clock(monkeypatch, datetime(2026, 11, 2, 3, 4, 5))
    dialog.name.setText('重试')
    dialog.suffixes['time'].setChecked(False)
    dialog.update_name()
    assert dialog.task['output'] == row['output']
    assert dialog.dataset.path() == row['output']
    assert row['output'] in dialog.status_label.text()
    assert all(checkbox.isEnabled() for checkbox in dialog.suffixes.values())
    backend.start_capture.assert_not_called()
    backend.prepare_pairing_capture.assert_not_called()
