"""Synthetic GUI interaction tests; no hardware, user data, or config writes."""
import copy
import lzma
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtCore, QtGui, QtTest, QtWidgets, sip
import numpy as np

from wc_panel.catalog import variants
from wc_panel.app import MainWindow
from wc_panel.ui_common import BackgroundTask, apply_theme, result_label
from wc_panel.ui_dialogs import (LiveDialog, CaptureDialog, OfflineDialog, ParametersDialog,
                                 StructuredEditor, ResultsDialog, StorageDialog, LogsDialog,
                                 LiveCalibrationEditor, RecoverMapsDialog, ExtrinsicsEditor)
from wc_panel.results_viewer import CloudCanvas, ResultsViewer, ResultCard, load_cloud, load_trajectory


class FixtureBackend:
    """Explicit test fixture, never used by the production entry point."""
    def __init__(self):
        self.calls = []
        self.roots = {"recording_root": "/fixture/recordings", "results_root": "/fixture/results"}
        self.rows = []
        self.capability_overrides = {}

    def settings(self):
        return dict(self.roots)

    def update_settings(self, recording_root, results_root):
        self.calls.append(("settings", recording_root, results_root))
        self.roots = dict(recording_root=recording_root, results_root=results_root)
        return dict(self.roots)

    def capabilities(self):
        enabled = dict(enabled=True, reason="合成测试条件已满足", by_sides={side:dict(enabled=True, reason="合成测试") for side in ("all", "left", "right")})
        return {"offline_variants": variants(), "official_geometry": {"enabled": False},
                "live_mapping": copy.deepcopy(enabled), "live_geometry": copy.deepcopy(enabled),
                "live_motion_correction": dict(enabled=False, reason="测试未配置设备声明"),
                **copy.deepcopy(self.capability_overrides)}

    def jobs(self):
        return copy.deepcopy(self.rows)

    def list_recordings(self, root=None):
        return [dict(id="fixture_recording", name="界面测试录制（合成）", path=str(root)+"/fixture_recording",
                     status="COMPLETE", complete=True, bytes=1024, sides="all", reason="",
                     available_sides=["left", "right"], source_inventory_error="",
                     available_clouds_by_side={side:["raw", "filtered"] for side in ("left", "right")})]

    def list_results(self, root=None):
        return []

    def start_live(self, options):
        self.calls.append(("live", options))
        return dict(id="live_fixture", label="实时预览（测试）", status="RUNNING")

    def start_capture(self, output_root=None, sides="all", *, name_options=None):
        self.capture_naming = name_options
        self.calls.append(("capture", output_root, sides))
        return dict(id="capture_fixture", label="数据录制（测试）", status="RUNNING")

    def plan_offline(self, dataset, output_root, options):
        self.calls.append(("plan", dataset, output_root, options))
        return dict(id="fixture_plan", dataset=dataset, output_root=output_root,
                    tasks=[dict(id=x, label=x, output=output_root+"/"+x) for x in options["variants"]],
                    task_count=len(options["variants"]))

    def start_offline(self, plan):
        self.calls.append(("offline", plan))
        row = dict(id="offline_fixture", kind="offline", label="离线融合（测试）", status="RUNNING", progress=0,
                   task_states=[dict(task, status="QUEUED") for task in plan["tasks"]])
        self.rows.append(row)
        return row

    def cancel(self, identifier):
        self.calls.append(("cancel", identifier))
        row = next(row for row in self.rows if row["id"] == identifier)
        row["status"] = "STOPPING"
        return row

    def list_logs(self, identifier):
        return [dict(id="runtime", label="任务日志", path="/fixture/runtime.log", bytes=0, available=True, error="")]

    def read_log(self, identifier, offset=0, log_id=None):
        return dict(text="", offset=offset, status="RUNNING")

    def device_parameters(self):
        return dict(revision="fixture_revision", identity={"left": {"serial": "SYNTHETIC-ONLY"}}, groups=[
            dict(id="extrinsics", label="设备外参", value=[{"id": "left", "translation_m": [0.2, 0.3, 0.4]}],
                 warning_required=True, effective="next_task", path="/fixture/config/hardware_setup.json"),
            dict(id="wheels", label="轮参数", value={"wheel_radius_m": 0.2, "track_width_m": 0.5},
                 warning_required=True, effective="next_task", path="/fixture/config/hardware_setup.json"),
            dict(id="live_motion", label="在线运动校正", value=dict(calibration={"wheel_yaw_scale":None},gyro_bias_config="",calibration_evidence_path=""),
                 warning_required=True, effective="next_task", path="/fixture/config/mapping_live.json")])

    def load_live_calibration(self, calibration_path, gyro_bias_config, calibration_evidence_path=None):
        self.calls.append(("load_calibration", calibration_path, gyro_bias_config, calibration_evidence_path))
        if "offline" in calibration_path:
            raise ValueError("测试拒绝会话级离线候选")
        return dict(calibration={"wheel_yaw_scale": 1.05, "confirmed_gyro_bias": {"fixture_only": True}},
                    gyro_bias_config=gyro_bias_config, calibration_evidence_path=calibration_evidence_path or "/fixture/device.evidence.json")

    def list_recoverable_maps(self):
        return [dict(id="stopped_fixture", path="/fixture/maps/stopped", status="SAVE_PENDING", recoverable=True, reason="测试已停止"),
                dict(id="blocked_fixture", path="/fixture/maps/blocked", status="RECOVERY_BLOCKED", recoverable=False, reason="测试未确认闭库")]

    def recover_map(self, path):
        self.calls.append(("recover_map", path))
        return dict(id="recovery_fixture", kind="recovery", label="恢复测试", status="RUNNING")

    def request_parameter_edit(self, group):
        self.calls.append(("edit_permission", group))
        return dict(token="fixture_token", revision="fixture_revision", message="测试警告")

    def prepare_extrinsic_evidence(self, path, evidence_type, assembly_revision, note, expected_revision):
        self.calls.append(("prepare_evidence", path, evidence_type, assembly_revision, note, expected_revision))
        return dict(token="evidence_token", source=dict(id="new_source", sha256="0"*64, original_path=path),
                    revision=expected_revision, physical_accuracy_validated=False)

    def save_parameters(self, group, value, warning_token=None, expected_revision=None, evidence_tokens=None):
        call = ("save_parameters", group, value, warning_token, expected_revision)
        self.calls.append(call + (evidence_tokens,) if evidence_tokens is not None else call)
        return dict(revision="updated", backup="/fixture/backup")

    def scan_storage(self, root):
        return dict(root=root, bytes=1024, allocated_bytes=4096, free_bytes=2048, items=[
            dict(name="fixture_only", path=str(root)+"/fixture_only", bytes=1024, allocated_bytes=4096,kind="recording", active=False)])

    def prepare_delete(self, root, paths):
        self.calls.append(("prepare_delete", root, paths))
        return dict(token="fixture_delete_token", paths=paths, bytes=1024, allocated_bytes=4096,file_count=1)

    def execute_delete(self, token):
        self.calls.append(("execute_delete", token))
        return dict(deleted=["fixture_only"], bytes=1024, free_delta_bytes=2048)


def drain(app, predicate=lambda: not BackgroundTask.live_tasks, timeout=5):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        app.processEvents()
        if predicate():
            app.processEvents()
            return
        QtTest.QTest.qWait(5)
    raise AssertionError("GUI background work did not finish")


class PanelUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        apply_theme(cls.app)

    def setUp(self):
        self.backend = FixtureBackend()
        self.windows = []

    def make(self, cls):
        window = cls(self.backend)
        self.windows.append(window)
        window.show()
        drain(self.app)
        return window

    def tearDown(self):
        drain(self.app)
        for window in self.windows:
            if sip.isdeleted(window):
                continue
            for timer in window.findChildren(QtCore.QTimer):
                timer.stop()
            if hasattr(window, "editing"):
                window.editing = False
            if hasattr(window, "known_jobs"):
                window.known_jobs.clear()
            window.close()
        self.app.processEvents()

    def test_home_has_only_seven_function_buttons(self):
        window = self.make(MainWindow)
        self.assertEqual(set(window.buttons), {"实时建图", "数据录制", "离线融合", "结果对比", "设备参数", "数据与存储", "日志"})
        self.assertEqual(len(window.findChildren(QtWidgets.QPushButton)), 7)

    def test_capture_finalization_notifies_once_with_actual_directory(self):
        window = self.make(MainWindow)
        row = dict(id="capture", kind="capture", status="RUNNING", output="/fixture/recordings/20261008_010203")
        window.task_started(row)
        with mock.patch.object(QtWidgets.QMessageBox, "information") as completed, \
             mock.patch.object(QtWidgets.QMessageBox, "warning") as failed:
            window.jobs_loaded([{**row, "status": "STOPPING", "phase": "VERIFYING", "stage_message": "正在检查录制完整性"}])
            self.assertIn("正在收尾", window.notice.text())
            self.assertIn("正在检查录制完整性", window.notice.text())
            self.assertIn(row["output"], window.notice.text())
            completed.assert_not_called()
            window.jobs_loaded([{**row, "status": "COMPLETE"}])
            window.jobs_loaded([{**row, "status": "COMPLETE"}])
            completed.assert_called_once()
            self.assertIn(row["output"], completed.call_args[0][2])
            self.assertIn("完整性检查", completed.call_args[0][2])
            failed.assert_not_called()

    def test_capture_failure_reports_incomplete_data_without_replaying_history(self):
        window = self.make(MainWindow)
        historical = dict(id="old", kind="capture", status="FAILED", output="/fixture/old", error="旧错误")
        with mock.patch.object(QtWidgets.QMessageBox, "warning") as message:
            window.jobs_loaded([historical])
            message.assert_not_called()
            row = {**historical, "id": "new", "status": "RUNNING", "output": "/fixture/new"}
            window.task_started(row)
            window.jobs_loaded([{**row, "status": "FAILED", "error": "校验不一致"}])
            window.jobs_loaded([{**row, "status": "FAILED", "error": "校验不一致"}])
            message.assert_called_once()
            self.assertIn("校验不一致", message.call_args[0][2])
            self.assertIn("可能不完整", message.call_args[0][2])
            self.assertIn("/fixture/new", message.call_args[0][2])

    def test_official_geometry_visible_disabled_and_never_submitted(self):
        dialog = self.make(LiveDialog)
        self.assertTrue(dialog.geometry.isVisible())
        self.assertFalse(dialog.geometry.isEnabled())
        dialog.mode.setCurrentIndex(1)
        dialog.estimator.setCurrentIndex(1)
        dialog.geometry.setChecked(True)
        self.assertTrue(dialog.geometry.isEnabled())
        dialog.estimator.setCurrentIndex(0)
        self.assertFalse(dialog.geometry.isEnabled())
        self.assertFalse(dialog.geometry.isChecked())
        dialog.start()
        drain(self.app)
        self.assertEqual(self.backend.calls[-1][0], "live")
        self.assertFalse(self.backend.calls[-1][1]["geometry"])

    def test_capture_one_time_path_does_not_change_defaults(self):
        dialog = self.make(CaptureDialog)
        dialog.destination.set_path("/fixture/one-time")
        dialog.sides.setCurrentIndex(2)
        dialog.start()
        drain(self.app)
        self.assertEqual(self.backend.calls[-1], ("capture", "/fixture/one-time", "right"))
        self.assertEqual(self.backend.settings()["recording_root"], "/fixture/recordings")

    def test_offline_all_selects_exact_supported_matrix_and_queue(self):
        dialog = self.make(OfflineDialog)
        dialog.select_variants(True)
        self.assertEqual(len(dialog.selected_variants()), 22)
        self.assertEqual(set(dialog.selected_variants()), {v["id"] for v in variants() if v["enabled"]})
        self.assertFalse(dialog.mechanical_initial.isChecked())
        self.assertFalse(dialog.allow_partial.isChecked())
        with mock.patch.object(QtWidgets.QMessageBox, "exec_", return_value=QtWidgets.QMessageBox.Ok):
            dialog.make_plan()
            drain(self.app)
        self.assertEqual(dialog.stack.currentWidget(), dialog.queue_page)
        self.assertEqual(dialog.queue_table.rowCount(), 22)
        self.assertTrue(all(dialog.queue_table.item(i, 1).text() == "等待中" for i in range(22)))
        self.assertEqual(self.backend.calls[0][3]["variants"], dialog.selected_variants())

    def test_completed_offline_queue_reopens_without_resubmitting(self):
        self.backend.rows = [dict(id="completed", kind="offline", label="历史实际批次", status="COMPLETE", progress=1,
                                 started_at=1791394152.7, output="/fixture/results/completed",
                                 task_states=[dict(id="stage", label="历史阶段", status="COMPLETE", output="/fixture/results/completed/stage")])]
        dialog = self.make(OfflineDialog)
        self.assertEqual(dialog.stack.currentWidget(), dialog.setup_page)
        self.assertTrue(dialog.history_button.isEnabled())
        dialog.history.setCurrentIndex(dialog.history.findData("completed"))
        dialog.history_button.click()
        drain(self.app)
        self.assertEqual(dialog.job_id, "completed")
        self.assertEqual(dialog.stack.currentWidget(), dialog.queue_page)
        self.assertIn("已完成", dialog.queue_label.text())
        self.assertEqual(dialog.queue_table.item(0, 1).text(), "已完成")
        self.assertFalse(dialog.cancel_button.isEnabled())
        self.assertEqual(dialog.progress.value(), 100)
        self.assertEqual(self.backend.calls, [])

    def test_existing_job_screenshots_use_history_entry_and_actual_stage_log(self):
        from render_screenshots import capture_existing_job
        self.backend.rows = [dict(id="history", kind="offline", label="已存在批次", status="COMPLETE", progress=1,
                                 started_at=1791394152.7, task_states=[dict(id="stage", label="阶段", status="COMPLETE")])]
        content = "真实读取协议的隔离测试日志\n"
        self.backend.list_logs = lambda identifier: [dict(id="runtime", label="任务日志", bytes=0, path="/fixture/runtime.log", available=True),
                                                    dict(id="stage", label="实际阶段", bytes=len(content.encode()), path="/fixture/stage.log", available=True)]
        self.backend.read_log = lambda identifier, offset=0, log_id=None: dict(text=content[offset:] if log_id == "stage" else "", offset=len(content) if log_id == "stage" else 0, status="COMPLETE")
        with tempfile.TemporaryDirectory(dir=os.environ.get("PANEL_TEST_TMP_ROOT")) as temp:
            record = capture_existing_job(self.app, self.backend, "history", Path(temp), self.windows)
            self.assertEqual(record["job"]["status"], "COMPLETE")
            self.assertIn("查看已有队列", record["ui_entry"])
            self.assertTrue(record["nonempty_stage_log_loaded"])
            self.assertFalse(record["fusion_started"])
            self.assertEqual(record["selected_stage_log"]["id"], "stage")
            self.assertTrue(all(Path(path).stat().st_size > 0 for path in record["screenshots"]))
            self.assertEqual(self.backend.calls, [])
            with self.assertRaises(ValueError):
                capture_existing_job(self.app, self.backend, "does_not_exist", Path(temp), self.windows)

    def test_warning_cancel_leaves_parameters_readonly(self):
        dialog = self.make(ParametersDialog)
        with mock.patch.object(dialog, "confirm_warning", return_value=False):
            dialog.begin_edit()
        self.assertFalse(dialog.editing)
        self.assertFalse(self.backend.calls)
        for editor, kind, _ in dialog.editor.editors.values():
            if isinstance(editor, QtWidgets.QLineEdit):
                self.assertTrue(editor.isReadOnly())

    def test_warning_confirm_precedes_token_and_structured_save(self):
        dialog = self.make(ParametersDialog)
        dialog.group.setCurrentIndex(1)
        with mock.patch.object(dialog, "confirm_warning", return_value=True):
            dialog.begin_edit()
            drain(self.app)
        self.assertEqual(self.backend.calls[0], ("edit_permission", "wheels"))
        self.assertTrue(dialog.editing)
        editor = dialog.editor.editors[("wheel_radius_m",)][0]
        self.assertFalse(editor.isReadOnly())
        editor.setText("0.21")
        with mock.patch.object(QtWidgets.QMessageBox, "information", return_value=QtWidgets.QMessageBox.Ok):
            dialog.save()
            drain(self.app)
        call = self.backend.calls[1]
        self.assertEqual(call[0], "save_parameters")
        self.assertEqual(call[2]["wheel_radius_m"], .21)
        self.assertEqual(call[3:], ("fixture_token", "fixture_revision"))

    def test_structured_form_preserves_nested_types(self):
        value = {"array": [1, 2.5, True, None], "mode": "known"}
        editor = StructuredEditor(value, True)
        self.assertEqual(editor.value(), value)
        editor.editors[("array", 1)][0].setText("3.125")
        self.assertEqual(editor.value()["array"], [1, 3.125, True, None])
        editor.editors[("array", 1)][0].setText("nan")
        with self.assertRaises(ValueError):
            editor.value()

    def test_unknown_extrinsics_require_explicit_complete_metric_values(self):
        editor = StructuredEditor({"translation": {"value_m": None, "status": "UNKNOWN"},
                                   "rotation": {"value_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}}, True)
        translation = editor.editors[("translation", "value_m")][0]
        self.assertIsNone(editor.value()["translation"]["value_m"])
        translation.unknown.setChecked(False)
        with self.assertRaises(ValueError):
            editor.value()
        for field, value in zip(translation.entries, ("0.2", "0", "-0.3")):
            field.setText(value)
        self.assertEqual(editor.value()["translation"]["value_m"], [.2, 0., -.3])
        rotation = editor.editors[("rotation", "value_matrix")][0]
        rotation.entries[0].setText("0.999")
        self.assertEqual(editor.value()["rotation"]["value_matrix"][0][0], .999)

    def test_cloud_parser_and_view_change_are_real_coordinates(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("PANEL_TEST_TMP_ROOT")) as temp:
            ply = Path(temp)/"fixture.ply"
            ply.write_text("ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 0 0\n0 2 0\n0 0 3\n")
            loaded = load_cloud(ply, maximum=3)
            np.testing.assert_array_equal(loaded["points"], np.array([[0, 0, 0], [0, 2, 0]]))
            self.assertEqual(loaded["total"], 4)
            canvas = CloudCanvas()
            canvas.resize(500, 400)
            canvas.set_cloud(load_cloud(ply)["points"])
            before = canvas.render_points()
            canvas.set_state(dict(yaw=90, pitch=30, zoom=2, pan=[.1, .1]))
            after = canvas.render_points()
            self.assertNotEqual(before, after)
            self.assertEqual(canvas.state()["pan"], [.1, .1])
            canvas.top_view()
            self.assertEqual(canvas.state()["pitch"], 90)
            ply.write_text(ply.read_text().replace("element vertex 4", "element vertex 5"))
            with self.assertRaises(ValueError):
                load_cloud(ply)

    def test_pcd_binary_and_trajectory_preserve_metric_values(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("PANEL_TEST_TMP_ROOT")) as temp:
            pcd = Path(temp)/"points.pcd"
            xyz = np.array([[0, 0, 1.2], [5, -2, 0]], dtype="<f4")
            pcd.write_bytes(b"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n" + xyz.tobytes())
            np.testing.assert_allclose(load_cloud(pcd)["points"], xyz)
            route = Path(temp)/"trajectory.csv"
            route.write_text("stamp_ns,x_m,y_m,z_m\n1,1,2,0\n2,-3,5,0\n")
            np.testing.assert_array_equal(load_trajectory(route), [[1, 2], [-3, 5]])

    def test_result_cards_load_actual_cloud_and_link_camera_without_time_axis(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("PANEL_TEST_TMP_ROOT")) as temp:
            pcd = Path(temp)/"points.pcd"
            pcd.write_bytes(b"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n" +
                            np.array([[0, 0, 0], [1, 2, 3]], dtype="<f4").tobytes())
            results = [dict(name="合成测试 " + str(n), path=temp, clouds=[str(pcd)], images=[], trajectories=[]) for n in (1, 2)]
            viewer = ResultsViewer(results)
            self.windows.append(viewer)
            viewer.show()
            viewer.mode.setCurrentIndex(2)
            drain(self.app)
            for card in viewer.cards:
                self.assertEqual(len(card.cloud_canvas.points), 2)
                self.assertEqual(card.tabs.count(), 3)
            self.assertEqual(viewer.findChildren(QtWidgets.QSlider), [])
            viewer.link.setChecked(True)
            viewer.cards[0].cloud_canvas.top_view()
            self.assertEqual(viewer.cards[1].cloud_canvas.pitch, 90)

    def test_deletion_only_after_specific_manifest_confirmation(self):
        dialog = self.make(StorageDialog)
        dialog.table.selectRow(0)
        with mock.patch.object(QtWidgets.QMessageBox, "exec_", return_value=QtWidgets.QMessageBox.Cancel):
            dialog.prepare_delete()
            drain(self.app)
        self.assertEqual(self.backend.calls, [("prepare_delete", "/fixture/recordings", ["/fixture/recordings/fixture_only"])])
        with mock.patch.object(QtWidgets.QMessageBox, "exec_", return_value=QtWidgets.QMessageBox.Yes), \
             mock.patch.object(QtWidgets.QMessageBox, "information", return_value=QtWidgets.QMessageBox.Ok):
            dialog.prepare_delete()
            drain(self.app)
        self.assertEqual(self.backend.calls[-1], ("execute_delete", "fixture_delete_token"))

    def test_live_defaults_preserve_five_state_then_user_session_choice(self):
        main = self.make(MainWindow)
        dialog = main.open_dialog(LiveDialog)
        drain(self.app)
        self.assertEqual(dialog.estimator.currentData(), "five_state")
        dialog.estimator.setCurrentIndex(dialog.estimator.findData("robot_localization"))
        dialog.close()
        self.app.processEvents()
        reopened = main.open_dialog(LiveDialog)
        drain(self.app)
        self.assertEqual(reopened.estimator.currentData(), "robot_localization")
        reopened.close()

    def test_mapping_capability_blocks_only_selected_side_not_native_preview(self):
        self.backend.capability_overrides["live_mapping"] = dict(enabled=True, reason="", by_sides={
            "all":dict(enabled=False, reason="缺少右侧实测外参"), "left":dict(enabled=True, reason=""),
            "right":dict(enabled=False, reason="缺少右侧实测外参")})
        dialog = self.make(LiveDialog)
        self.assertTrue(dialog.start_button.isEnabled())
        dialog.mode.setCurrentIndex(dialog.mode.findData("mapping"))
        self.assertFalse(dialog.start_button.isEnabled())
        self.assertIn("缺少右侧", dialog.availability.text())
        dialog.start()
        self.assertEqual(self.backend.calls, [])
        dialog.sides.setCurrentIndex(dialog.sides.findData("left"))
        self.assertTrue(dialog.start_button.isEnabled())
        self.assertFalse(dialog.motion.isEnabled())
        dialog.mode.setCurrentIndex(dialog.mode.findData("preview"))
        self.assertFalse(dialog.geometry.isEnabled())

    def test_global_defaults_update_open_windows_but_preserve_temporary_path(self):
        main = self.make(MainWindow)
        capture = main.open_dialog(CaptureDialog)
        offline = main.open_dialog(OfflineDialog)
        results = main.open_dialog(ResultsDialog)
        drain(self.app)
        offline.output_root.set_path("/fixture/temporary-output")
        main.settings_changed(dict(recording_root="/fixture/new-recordings", results_root="/fixture/new-results"))
        drain(self.app)
        self.assertEqual(capture.destination.path(), "/fixture/new-recordings")
        self.assertEqual(offline.recording_root.path(), "/fixture/new-recordings")
        self.assertEqual(offline.output_root.path(), "/fixture/temporary-output")
        self.assertEqual(results.root.path(), "/fixture/new-results")
        for window in (capture, offline, results):
            window.close()

    def test_device_calibration_import_validates_before_save_and_rejects_dirty_paths(self):
        dialog = self.make(ParametersDialog)
        dialog.request_group("live_motion")
        with mock.patch.object(dialog, "confirm_warning", return_value=True):
            dialog.begin_edit()
            drain(self.app)
        self.assertIsInstance(dialog.editor, LiveCalibrationEditor)
        editor = dialog.editor
        self.assertIn("未配置", editor.validation_status.text())
        editor.declaration.set_path("/fixture/device.json")
        editor.gyro_bias.set_path("/fixture/bias.json")
        with self.assertRaises(ValueError):
            editor.value()
        editor.import_files()
        drain(self.app)
        self.assertEqual(editor.value()["calibration"]["wheel_yaw_scale"], 1.05)
        self.assertIn("尚未保存", editor.validation_status.text())
        self.assertEqual(self.backend.calls[-1], ("load_calibration", "/fixture/device.json", "/fixture/bias.json", None))
        self.assertFalse(any(call[0] == "save_parameters" for call in self.backend.calls))
        with mock.patch.object(QtWidgets.QMessageBox, "information", return_value=QtWidgets.QMessageBox.Ok):
            dialog.save()
            drain(self.app)
        self.assertEqual(self.backend.calls[-1][0:2], ("save_parameters", "live_motion"))
        self.assertEqual(self.backend.calls[-1][3:], ("fixture_token", "fixture_revision"))

    def test_log_first_load_reads_nonempty_stage_and_switches_without_mixing(self):
        self.backend.rows = [dict(id="job", kind="offline", label="融合任务", status="COMPLETE", started_at=1791394152.7)]
        contents = {"runtime": "", "stage_a": "阶段 A 实际输出\n", "stage_b": "阶段 B 实际输出\n"}
        self.backend.list_logs = lambda identifier: [dict(id=key, label=key, path="/fixture/"+key+".log", bytes=len(value.encode()), available=True)
                                                    for key, value in contents.items()]
        self.backend.read_log = lambda identifier, offset=0, log_id=None: dict(text=contents[log_id][offset:], offset=len(contents[log_id]), status="COMPLETE")
        dialog = self.make(LogsDialog)
        self.assertEqual(dialog.source.currentData(), "stage_a")
        self.assertEqual(dialog.text.toPlainText(), contents["stage_a"])
        self.assertNotIn("1791394152", dialog.task.currentText())
        self.assertRegex(dialog.task.currentText(), r"2026-\d\d-\d\d \d\d:\d\d:\d\d")
        dialog.source.setCurrentIndex(dialog.source.findData("stage_b"))
        drain(self.app)
        self.assertEqual(dialog.text.toPlainText(), contents["stage_b"])
        dialog.poll()
        drain(self.app)
        self.assertEqual(dialog.text.toPlainText(), contents["stage_b"])
        dialog.source.setCurrentIndex(dialog.source.findData("runtime"))
        drain(self.app)
        self.assertEqual(dialog.text.toPlainText(), "")
        self.assertIn("暂无内容", dialog.text.placeholderText())

    def test_log_unavailable_reason_is_visible(self):
        self.backend.rows = [dict(id="job", status="FAILED")]
        self.backend.list_logs = lambda identifier: [dict(id="stage", label="阶段", path="/fixture/missing.log", available=False, bytes=None, error="日志文件不存在")]
        dialog = self.make(LogsDialog)
        self.assertIn("日志文件不存在", dialog.text.placeholderText())

    def test_extrinsic_evidence_binding_preserves_unknown_and_requires_explicit_save(self):
        group = dict(id="extrinsics", assembly_revision="ASSEMBLY_TEST_ONLY", value=[dict(id="left", parent_frame="axle", child_frame="lidar_left",
                     translation=dict(value_m=None, status="UNKNOWN", evidence=[dict(source_id="old_source", line_start=1, line_end=3)]),
                     rotation=dict(value_matrix=None, status="UNKNOWN", evidence=[]))])
        editor = ExtrinsicsEditor(self.backend, group, "fixture_revision", True)
        self.windows.append(editor)
        editor.show()
        editor.open_import()
        dialog = editor.import_dialog
        dialog.file.set_path("/fixture/measurement.txt")
        dialog.note.setText("隔离测试测量")
        dialog.prepare()
        self.assertIn("确认", dialog.status.text())
        self.assertEqual(self.backend.calls, [])
        dialog.confirm_assembly.setChecked(True)
        dialog.prepare()
        drain(self.app)
        value = editor.value()
        self.assertIsNone(value[0]["translation"]["value_m"])
        self.assertEqual(value[0]["translation"]["status"], "UNKNOWN")
        self.assertEqual(value[0]["translation"]["evidence"], [dict(source_id="new_source")])
        self.assertEqual(value[0]["rotation"], group["value"][0]["rotation"])
        self.assertEqual(editor.evidence_tokens(value), ["evidence_token"])
        self.assertFalse(any(call[0] == "save_parameters" for call in self.backend.calls))
        self.assertEqual(editor.evidence_tokens(group["value"]), [])
        status = editor.editors[(0, "translation", "status")][0]
        self.assertIsInstance(status, QtWidgets.QComboBox)
        status.setCurrentIndex(status.findData("USER_MEASURED_EXPERIMENT"))
        self.assertEqual(editor.value()[0]["translation"]["status"], "USER_MEASURED_EXPERIMENT")

    def test_parameter_save_passes_only_referenced_evidence_tokens(self):
        group = dict(id="extrinsics", assembly_revision="ASSEMBLY_TEST_ONLY", value=[dict(id="left",
                     translation=dict(value_m=None, status="UNKNOWN", evidence=[]),
                     rotation=dict(value_matrix=None, status="UNKNOWN", evidence=[]))])
        self.backend.device_parameters = lambda: dict(revision="fixture_revision", identity={}, groups=[group])
        dialog = self.make(ParametersDialog)
        with mock.patch.object(dialog, "confirm_warning", return_value=True):
            dialog.begin_edit()
            drain(self.app)
        response = self.backend.prepare_extrinsic_evidence("/fixture/measurement", "GEOMETRY_MEASUREMENT", "ASSEMBLY_TEST_ONLY", "fixture", "fixture_revision")
        dialog.editor.bind_evidence(dialog.editor.value(), dict(response, target=(0, "translation", "replace")))
        self.assertFalse(any(call[0] == "save_parameters" for call in self.backend.calls))
        with mock.patch.object(QtWidgets.QMessageBox, "information"):
            dialog.save()
            drain(self.app)
        self.assertEqual(self.backend.calls[-1][0:2], ("save_parameters", "extrinsics"))
        self.assertEqual(self.backend.calls[-1][-1], ["evidence_token"])
        self.assertEqual(self.backend.calls[-1][2][0]["translation"]["status"], "UNKNOWN")

    def test_recovery_requires_confirmed_stopped_session_and_only_starts_save_job(self):
        dialog = self.make(RecoverMapsDialog)
        dialog.table.selectRow(1)
        self.assertFalse(dialog.restore_button.isEnabled())
        dialog.restore()
        self.assertEqual(self.backend.calls, [])
        dialog.table.selectRow(0)
        self.assertTrue(dialog.restore_button.isEnabled())
        dialog.restore()
        drain(self.app)
        self.assertEqual(self.backend.calls, [("recover_map", "/fixture/maps/stopped")])

    def test_storage_displays_allocated_separately_from_logical_size(self):
        dialog = self.make(StorageDialog)
        self.assertEqual(dialog.table.item(0, 1).text(), "1.00 KiB")
        self.assertEqual(dialog.table.item(0, 2).text(), "4.00 KiB")
        with mock.patch.object(QtWidgets.QMessageBox, "information", return_value=QtWidgets.QMessageBox.Ok) as message:
            dialog.deleted(dict(bytes=1024,free_delta_bytes=2048))
            drain(self.app)
        self.assertIn("2.00 KiB", message.call_args[0][2])
        self.assertIn("其他任务", message.call_args[0][2])

    def test_storage_incomplete_totals_and_unavailable_entries_cannot_be_deleted(self):
        dialog = self.make(StorageDialog)
        items = [dict(name="partial", path="/fixture/partial", bytes=None, allocated_bytes=None,
                      known_bytes=512, known_allocated_bytes=1024, entry_type="directory", status="UNAVAILABLE",
                      error="拒绝访问 nested", active=False, deletable=False),
                 dict(name=".hidden", path="/fixture/.hidden", bytes=512, allocated_bytes=4096,
                      entry_type="file", status="AVAILABLE", active=False, deletable=False)]
        dialog.scanned(dialog.scan_generation, dict(root="/fixture", bytes=None, allocated_bytes=None,
                       known_bytes=1024, known_allocated_bytes=5120, total_complete=False, unknown_items=1,
                       free_bytes=None, items=items))
        self.assertIn("统计不完整", dialog.usage.text())
        self.assertIn("已知 1.00 KiB", dialog.usage.text())
        self.assertIn("磁盘可用 未统计", dialog.usage.text())
        self.assertIn("拒绝访问", dialog.table.item(0, 4).text())
        self.assertIn("已知 512 B", dialog.table.item(0, 1).text())
        self.assertFalse(dialog.table.item(0, 0).flags() & QtCore.Qt.ItemIsSelectable)
        self.assertFalse(dialog.table.item(1, 0).flags() & QtCore.Qt.ItemIsSelectable)
        self.assertFalse(dialog.delete_button.isEnabled())
        with mock.patch.object(dialog, "show_error"):
            dialog.prepare_delete()
        self.assertEqual(self.backend.calls, [])

    def test_main_can_exit_detached_offline_and_pending_save_is_terminal(self):
        main = self.make(MainWindow)
        spy = QtTest.QSignalSpy(main.exitRequested)
        main.task_started(dict(id="offline",kind="offline",status="RUNNING",execution_mode="detached_offline"))
        main.task_started(dict(id="pending",kind="live",status="PENDING"))
        self.assertTrue(main.close())
        self.assertEqual(len(spy), 1)

    def test_different_size_trajectories_share_metric_scale_and_viewport(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("PANEL_TEST_TMP_ROOT")) as temp:
            results = []
            for length in (1, 10):
                path = Path(temp)/("trajectory_" + str(length) + ".csv")
                path.write_text("x_m,y_m\n0,0\n{},{}\n".format(length, length))
                results.append(dict(name="{} m 合成路线".format(length), path=temp, clouds=[], images=[], trajectories=[str(path)]))
            viewer = ResultsViewer(results)
            self.windows.append(viewer)
            viewer.show()
            viewer.mode.setCurrentIndex(1)
            drain(self.app)
            viewer.sync_route_views()
            first, second = [card.route_canvas for card in viewer.cards]
            self.assertEqual(first.shared_viewport, second.shared_viewport)
            np.testing.assert_array_equal(first.shared_bounds, [[0, 0], [10, 10]])
            p1, r1, _ = first.projected_points()
            p2, r2, _ = second.projected_points()
            self.assertAlmostEqual(np.linalg.norm(p2[1]-p2[0])/np.linalg.norm(p1[1]-p1[0]), 10.0)
            np.testing.assert_allclose(p1[0]-[r1.x(),r1.y()], p2[0]-[r2.x(),r2.y()])
            viewer.uniform_routes.setChecked(False)
            self.assertIsNone(first.shared_bounds)
            self.assertIsNone(second.shared_bounds)

    def test_compressed_ascii_binary_ply_match_plain_sampling_without_extraction(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("PANEL_TEST_TMP_ROOT")) as temp:
            xyz = np.array([[0, 0, 0], [1.25, -2, 3], [4, 5, -6], [7, 8, 9]], dtype="<f4")
            for form in ("ascii", "binary_little_endian", "binary_big_endian"):
                header = ("ply\nformat " + form + " 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nend_header\n").encode()
                payload = ("\n".join(" ".join(map(str, row)) for row in xyz)+"\n").encode() if form == "ascii" else xyz.astype(">f4" if form == "binary_big_endian" else "<f4").tobytes()
                plain = Path(temp)/(form+".ply")
                archived = Path(temp)/(form+".ply.xz")
                plain.write_bytes(header+payload)
                archived.write_bytes(lzma.compress(header+payload))
                files_before = sorted(p.name for p in Path(temp).iterdir())
                expected, observed = load_cloud(plain, maximum=3), load_cloud(archived, maximum=3)
                np.testing.assert_array_equal(expected["points"], observed["points"])
                self.assertEqual(observed["total"], 4)
                self.assertEqual(observed["shown"], expected["shown"])
                self.assertEqual(files_before, sorted(p.name for p in Path(temp).iterdir()))
                archived.write_bytes(lzma.compress(header + payload[:-12]))
                with self.assertRaises(ValueError):
                    load_cloud(archived)

    def test_compressed_pcd_ascii_binary_preserve_fields_sampling_and_no_extraction(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("PANEL_TEST_TMP_ROOT")) as temp:
            values = np.array([[9, 8, n, n*2, -n] for n in range(9)], dtype="<f4")
            for form in ("ascii", "binary"):
                header = ("FIELDS descriptor x y z\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 2 1 1 1\nWIDTH 9\nHEIGHT 1\nPOINTS 9\nDATA " + form + "\n").encode()
                payload = ("\n".join(" ".join(map(str, row)) for row in values)+"\n").encode() if form == "ascii" else values.tobytes()
                plain = Path(temp)/(form+".pcd")
                archive = Path(temp)/(form+".pcd.xz")
                plain.write_bytes(header+payload)
                archive.write_bytes(lzma.compress(header+payload))
                before = sorted(p.name for p in Path(temp).iterdir())
                actual = load_cloud(archive, maximum=4)
                np.testing.assert_array_equal(actual["points"], values[::3, 2:])
                np.testing.assert_array_equal(actual["points"], load_cloud(plain, maximum=4)["points"])
                self.assertEqual((actual["total"], actual["shown"]), (9, 3))
                self.assertEqual(before, sorted(p.name for p in Path(temp).iterdir()))

    def test_compressed_pcd_rejects_truncation_extra_data_bad_schema_and_decoded_limit(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("PANEL_TEST_TMP_ROOT")) as temp:
            archive = Path(temp)/"malformed.pcd.xz"
            header = b"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n"
            payload = np.zeros((2, 3), dtype="<f4").tobytes()
            cases = [header+payload[:-1], header+payload+b"X", header.replace(b"WIDTH 2", b"WIDTH 3")+payload,
                     header.replace(b"COUNT 1 1 1", b"COUNT 2 1 1")+payload,
                     header.replace(b"TYPE F F F", b"TYPE F V F")+payload,
                     header.replace(b"DATA binary", b"DATA binary_compressed")+payload,
                     header.replace(b"DATA binary", b"DATA ascii")+b"0 0 0\nBAD 1 2\n"]
            for content in cases:
                archive.write_bytes(lzma.compress(content))
                with self.subTest(content=content[:40]), self.assertRaises(ValueError):
                    load_cloud(archive, maximum=1)
            archive.write_bytes(lzma.compress(header+payload)[:-8])
            with self.assertRaises((ValueError, EOFError, lzma.LZMAError)):
                load_cloud(archive)
            header = header.replace(b"DATA binary", b"DATA ascii")
            archive.write_bytes(lzma.compress(header+b"0 0 0\n1 2 3\n"+b" "*2048))
            with mock.patch("wc_panel.results_viewer.MAX_DECOMPRESSED_BYTES", 256), self.assertRaisesRegex(ValueError, "读取上限"):
                load_cloud(archive)

    def test_screenshot_group_selection_is_exact_and_never_substitutes(self):
        from render_screenshots import select_results
        rows = [dict(group_id=group, clouds=[group+".ply.xz"]) for group in ("01", "02", "04", "12")]
        self.assertEqual([r["group_id"] for r in select_results(rows, ["02", "4", "12"])], ["02", "04", "12"])
        with self.assertRaisesRegex(ValueError, "匹配 0 项"):
            select_results(rows, ["03"])
        with self.assertRaisesRegex(ValueError, "匹配 2 项"):
            select_results(rows+[rows[1]], ["02"])

    def test_gallery_group_number_and_metric_assets_take_priority(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("PANEL_TEST_TMP_ROOT")) as temp:
            image_paths = {}
            for role in ("grid", "top", "3d", "trajectory"):
                path = Path(temp)/(role+".png")
                image = QtGui.QImage(32, 32, QtGui.QImage.Format_RGB32)
                image.fill(QtGui.QColor("#ffffff"))
                self.assertTrue(image.save(str(path)))
                image_paths[role] = str(path)
            route = Path(temp)/"trajectory.csv"
            route.write_text("x_m,y_m\n0,0\n1,1\n")
            result = dict(name="a_very_long_batch_name / 04 · 官方 filtered", group_id="04", path=temp,
                          images=list(image_paths.values()), trajectories=[image_paths["trajectory"], str(route)],
                          image_roles=image_paths, clouds=[])
            self.assertEqual(result_label(result), "04 · 官方 filtered")
            card = ResultCard(result)
            self.windows.append(card)
            card.show()
            drain(self.app)
            self.assertEqual(card.map_select.currentData(), image_paths["grid"])
            self.assertEqual(card.route_select.currentData(), str(route))
            self.assertNotIn(image_paths["3d"], card.map_images)


if __name__ == "__main__":
    unittest.main()
