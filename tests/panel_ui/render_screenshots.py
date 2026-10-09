"""Render all passive panel windows, without launching devices or algorithms.

Use --fixture for an explicitly marked synthetic UI-only preview. With no flag,
the real backend reads current directories and parameters, but starts no job.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtWidgets

from wc_panel.app import MainWindow
from wc_panel.ui_common import apply_theme
from wc_panel.ui_dialogs import (LiveDialog, CaptureDialog, OfflineDialog, ResultsDialog,
                                 ParametersDialog, StorageDialog, LogsDialog, RecoverMapsDialog)
from test_panel_ui import FixtureBackend, drain


def select_results(results, groups=None):
    if not groups:
        return [result for result in results if result.get("clouds")][:2]
    selected = []
    for group in groups:
        key = str(group).zfill(2)
        matches = [result for result in results if str(result.get("group_id", "")).zfill(2) == key]
        if len(matches) != 1:
            raise ValueError("结果组 {} 匹配 {} 项；请用 --results-root 选择唯一批次，不能替换为其他组。".format(key, len(matches)))
        if not matches[0].get("clouds"):
            raise ValueError("结果组 {} 没有可验收的三维点云文件".format(key))
        if matches[0] not in selected:
            selected.append(matches[0])
    return selected


def capture_existing_job(app, backend, identifier, output, windows, timeout=120):
    """Render persisted state from an existing offline job; never submit a job."""
    jobs = backend.jobs()
    matches = [job for job in jobs if job.get("id") == identifier]
    if len(matches) != 1 or matches[0].get("kind") != "offline":
        raise ValueError("--job-id 必须唯一匹配已经存在的离线任务，不能创建或替换任务。")
    job = matches[0]
    queue = OfflineDialog(backend)
    windows.append(queue)
    queue.show()
    drain(app, timeout=timeout)
    if queue.stack.currentWidget() == queue.queue_page and queue.job_id == identifier:
        ui_entry = "OfflineDialog 自动恢复当前队列"
    else:
        if queue.stack.currentWidget() != queue.setup_page:
            raise ValueError("当前窗口正在显示另一运行批次，不能绕过界面注入历史队列")
        history_index = queue.history.findData(identifier)
        if history_index < 0:
            raise ValueError("所选任务不在实际历史队列入口中")
        queue.history.setCurrentIndex(history_index)
        queue.history_button.click()
        drain(app, timeout=timeout)
        ui_entry = "OfflineDialog.history -> 查看已有队列"
    if queue.stack.currentWidget() != queue.queue_page or queue.job_id != identifier:
        raise RuntimeError("历史队列入口未成功打开所选任务")
    job = queue.displayed_job
    queue.timer.stop()
    app.processEvents()
    queue_path = output/"11_OfflineExistingQueue.png"
    if not queue.grab().save(str(queue_path)):
        raise RuntimeError("Cannot save existing queue screenshot")
    queue.hide()

    logs = LogsDialog(backend)
    windows.append(logs)
    logs.show()
    drain(app, timeout=timeout)
    task_index = logs.task.findData(identifier)
    if task_index < 0:
        raise ValueError("所选任务在日志窗口中已不可用")
    logs.task.setCurrentIndex(task_index)
    drain(app, timeout=timeout)
    entries = backend.list_logs(identifier)
    selected = next((entry for entry in entries if entry["id"] != "runtime" and entry.get("available") and (entry.get("bytes") or 0) > 0), None)
    if selected:
        source_index = logs.source.findData(selected["id"])
        if source_index < 0:
            raise ValueError("阶段日志已发生变化，请重新截图")
        logs.source.setCurrentIndex(source_index)
        drain(app, timeout=timeout)
    logs.timer.stop()
    app.processEvents()
    log_path = output/"12_ExistingStageLog.png"
    if not logs.grab().save(str(log_path)):
        raise RuntimeError("Cannot save stage-log screenshot")
    content = logs.text.toPlainText()
    successful = bool(selected and content and logs.source.currentData() == selected["id"])
    record = dict(source="PERSISTED_BACKEND_JOB_READ_ONLY", read_only_restore=True, fusion_started=False,
                  ui_entry=ui_entry,
                  captured_at_utc=datetime.now(timezone.utc).isoformat(),
                  job={key: job.get(key) for key in ("id", "kind", "label", "status", "progress", "started_at", "ended_at", "output", "log_path", "task_states")},
                  log_sources=entries, selected_stage_log=selected, displayed_log_characters=len(content),
                  displayed_log_sha256=hashlib.sha256(content.encode("utf-8")).hexdigest(),
                  nonempty_stage_log_loaded=successful,
                  error="" if successful else "未找到并显示非空的已登记阶段日志，不能据此判定日志验收通过",
                  screenshots=[str(queue_path), str(log_path)])
    logs.hide()
    return record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixture", action="store_true")
    parser.add_argument("--with-results", action="store_true", help="Read the first two real results and render their 3D cards")
    parser.add_argument("--result-groups", nargs="+", help="Read exact group IDs, e.g. 02 04 12; implies --with-results")
    parser.add_argument("--results-root", help="Read results under this root instead of the shared default")
    parser.add_argument("--job-id", help="Render an existing offline job's actual queue state and nonempty stage log; never run fusion")
    parser.add_argument("--load-timeout", type=float, default=600, help="Maximum passive file-read wait in seconds")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    app = QtWidgets.QApplication([])
    apply_theme(app)
    if args.fixture:
        backend = FixtureBackend()
    else:
        from wc_panel.backend import PanelBackend
        backend = PanelBackend(args.project_root)
    manifest = {"source": "SYNTHETIC_UI_FIXTURE" if args.fixture else "CURRENT_BACKEND_READ_ONLY", "hardware_started": False, "screenshots": [], "source_sha256": {}}
    for relative in ("src/wc_panel/app.py", "src/wc_panel/ui_dialogs.py", "src/wc_panel/ui_common.py",
                     "src/wc_panel/results_viewer.py", "src/wc_panel/backend.py", "src/wc_panel/catalog.py",
                     "src/wc_panel/jobs.py", "src/wc_panel/offline_worker.py",
                     "tests/panel_ui/render_screenshots.py"):
        source = args.project_root/relative
        if source.is_file():
            manifest["source_sha256"][relative] = hashlib.sha256(source.read_bytes()).hexdigest()
    classes = [MainWindow, LiveDialog, CaptureDialog, OfflineDialog, ResultsDialog,
               ParametersDialog, StorageDialog, LogsDialog, RecoverMapsDialog]
    windows = []
    for index, cls in enumerate(classes):
        window = cls(backend)
        windows.append(window)
        window.show()
        drain(app, timeout=120)
        app.processEvents()
        if args.fixture:
            window.setWindowTitle(window.windowTitle() + " · 合成界面测试")
        path = args.output/("{:02d}_{}.png".format(index+1, cls.__name__))
        if not window.grab().save(str(path)):
            raise RuntimeError("Cannot save screenshot: " + str(path))
        manifest["screenshots"].append(str(path))
        window.hide()
    parameter_window = ParametersDialog(backend)
    windows.append(parameter_window)
    parameter_window.request_group("live_motion")
    parameter_window.show()
    drain(app, timeout=120)
    path = args.output/"10_LiveCalibration.png"
    if not parameter_window.grab().save(str(path)):
        raise RuntimeError("Cannot save calibration screenshot")
    manifest["screenshots"].append(str(path))
    parameter_window.hide()
    if args.job_id:
        manifest["existing_job"] = capture_existing_job(app, backend, args.job_id, args.output, windows, args.load_timeout)
        manifest["screenshots"].extend(manifest["existing_job"]["screenshots"])
    if args.with_results or args.result_groups:
        from wc_panel.results_viewer import ResultsViewer
        results = select_results(backend.list_results(args.results_root), args.result_groups)
        manifest["selected_results"] = [{key: result.get(key) for key in ("id", "group_id", "name", "path", "clouds", "trajectories")}
                                        for result in results]
        if results:
            viewer = ResultsViewer(results)
            windows.append(viewer)
            viewer.show()
            for mode, label in ((0, "2d"), (1, "routes"), (2, "3d")):
                viewer.mode.setCurrentIndex(mode)
                drain(app, timeout=args.load_timeout)
                app.processEvents()
                path = args.output/("09_results_" + label + ".png")
                if not viewer.grab().save(str(path)):
                    raise RuntimeError("Cannot save results screenshot: " + str(path))
                manifest["screenshots"].append(str(path))
                # Capture every selected card, including cards below the viewport.
                # This preserves evidence for e.g. group 12 when 02/04 occupy row one.
                if args.result_groups:
                    for result, card in zip(results, viewer.cards):
                        path = args.output/("result_{}_{}.png".format(result["group_id"], label))
                        if not card.grab().save(str(path)):
                            raise RuntimeError("Cannot save result card: " + str(path))
                        manifest["screenshots"].append(str(path))
            manifest["result_reads"] = [dict(group_id=result.get("group_id"), path=result["path"],
                                                cloud_loaded=card.cloud_canvas.points is not None,
                                                cloud_status=card.cloud_status.text(), route_status=card.route_status.text())
                                         for result, card in zip(results, viewer.cards)]
            viewer.hide()
    (args.output/"screenshots.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    for window in windows:
        for timer in window.findChildren(__import__("PyQt5.QtCore", fromlist=["QTimer"]).QTimer):
            timer.stop()
    drain(app)
    print(json.dumps(manifest, ensure_ascii=False))
    failed = any(not row["cloud_loaded"] for row in manifest.get("result_reads", []))
    failed = failed or bool(args.job_id and not manifest["existing_job"]["nonempty_stage_log_loaded"])
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
