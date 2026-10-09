"""Seven-button desktop entry point for the wheelchair project."""
import argparse
import getpass
from pathlib import Path
import sys
import tempfile

from PyQt5 import QtCore, QtGui, QtWidgets

from .ui_common import apply_theme, BackgroundTask, heading, hint, TERMINAL
from .ui_dialogs import (CaptureDialog, LiveDialog, OfflineDialog, ResultsDialog,
                         ParametersDialog, StorageDialog, LogsDialog)


class ActionButton(QtWidgets.QPushButton):
    """Keep the whole card clickable, with a clear title / description hierarchy."""
    def __init__(self, title, description, parent=None):
        super().__init__(title + "\n" + description, parent)
        self.title, self.description = title, description
        self.setProperty("actionCard", True)
        self.setMinimumSize(214, 126)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setCursor(QtCore.Qt.PointingHandCursor)

    def paintEvent(self, event):
        option = QtWidgets.QStyleOptionButton()
        self.initStyleOption(option)
        option.text = ""
        painter = QtWidgets.QStylePainter(self)
        painter.drawControl(QtWidgets.QStyle.CE_PushButton, option)
        painter.setPen(QtGui.QColor("#183650"))
        font = self.font()
        font.setPixelSize(20)
        font.setWeight(QtGui.QFont.DemiBold)
        painter.setFont(font)
        middle = self.height() // 2
        painter.drawText(QtCore.QRect(12, middle-30, self.width()-24, 32), QtCore.Qt.AlignCenter, self.title)
        font.setPixelSize(13)
        font.setWeight(QtGui.QFont.Normal)
        painter.setFont(font)
        painter.setPen(QtGui.QColor("#6a7e92"))
        painter.drawText(QtCore.QRect(12, middle+7, self.width()-24, 24), QtCore.Qt.AlignCenter, self.description)


class MainWindow(QtWidgets.QMainWindow):
    exitRequested = QtCore.pyqtSignal()
    ACTIONS = (
        ("实时建图", "实时预览 · 在线融合", LiveDialog),
        ("数据录制", "录制 · 雷达预览", CaptureDialog),
        ("离线融合", "算法对照 · 任务队列", OfflineDialog),
        ("结果对比", "地图 · 路线 · 三维点云", ResultsDialog),
        ("设备参数", "身份 · 外参 · 算法", ParametersDialog),
        ("数据与存储", "默认路径 · 占用 · 清理", StorageDialog),
        ("日志", "任务记录 · 运行日志", LogsDialog),
    )

    def __init__(self, backend):
        super().__init__()
        self.backend = backend
        self.setWindowTitle("智能轮椅 · 工作面板")
        self.resize(960, 670)
        self.setMinimumSize(800, 590)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(38, 32, 38, 28)
        layout.setSpacing(18)
        layout.addWidget(heading("智能轮椅工作面板"))
        layout.addWidget(hint("选择一项功能开始操作"))
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(16)
        for column in range(3):
            grid.setColumnStretch(column, 1)
            grid.setRowStretch(column, 1)
        self.buttons = {}
        for index, (title, description, dialog_type) in enumerate(self.ACTIONS):
            widget = ActionButton(title, description)
            widget.clicked.connect(lambda checked=False, cls=dialog_type: self.open_dialog(cls))
            widget.setObjectName("action_" + dialog_type.__name__)
            row, column = (0, 1) if index == 0 else (1 + (index-1)//3, (index-1) % 3)
            grid.addWidget(widget, row, column)
            self.buttons[title] = widget
        layout.addLayout(grid, 1)
        self.notice = hint("就绪")
        layout.addWidget(self.notice)
        self.dialogs = {}
        self.known_jobs = {}
        self.known_job_details = {}
        self.last_live_options = {}
        self.polling = False
        self.first_poll = True
        self.recovery_checked = False
        self.job_timer = QtCore.QTimer(self)
        self.job_timer.timeout.connect(self.poll_jobs)
        self.job_timer.start(2500)
        self.poll_jobs()

    def open_dialog(self, dialog_type):
        dialog = self.dialogs.get(dialog_type)
        from PyQt5 import sip
        if dialog is not None and not sip.isdeleted(dialog):
            dialog.showNormal()
            dialog.raise_()
            dialog.activateWindow()
            return dialog
        dialog = dialog_type(self.backend, self)
        self.dialogs[dialog_type] = dialog
        dialog.show()
        return dialog

    def task_started(self, job):
        self.known_jobs[job["id"]] = job.get("status")
        self.known_job_details[job["id"]] = dict(job)
        if job.get("kind") == "capture":
            self.capture_status(job)
        else:
            self.notice.setText("已提交：{}。运行详情可在对应窗口和日志中查看。".format(job.get("label", job["id"])))

    def capture_status(self, job, notify=False):
        status = job.get("status")
        directory = str(job.get("output") or "尚未提供，请查看任务日志")
        if status == "COMPLETE":
            title = "录制已完成"
            message = "录制已完成并通过完整性检查。\n最终保存目录：{}".format(directory)
        elif status in ("FAILED", "CANCELLED"):
            title = "录制未完成"
            message = "录制未完整完成：{}\n数据目录：{}\n请在日志中查看原因，目录中的数据可能不完整。".format(
                job.get("error") or "任务已停止但未确认完整落盘", directory)
        elif status == "STOPPING":
            title = "录制正在收尾"
            message = "录制正在收尾：{}\n最终保存目录：{}\n请等待完成通知后再移除存储设备。".format(
                job.get("stage_message") or "正在停止采集、写入数据并检查完整性", directory)
        else:
            title = "录制任务"
            message = "录制任务已提交，最终保存目录：{}。请在 RViz 窗口顶部点击“结束录制”后等待收尾。".format(directory)
        self.notice.setText(message)
        if notify:
            show = QtWidgets.QMessageBox.information if status == "COMPLETE" else QtWidgets.QMessageBox.warning
            show(self, title, message)

    def settings_changed(self, settings):
        from PyQt5 import sip
        for dialog in self.dialogs.values():
            if not sip.isdeleted(dialog) and hasattr(dialog, "update_default_settings"):
                dialog.update_default_settings(settings)

    def parameters_changed(self):
        from PyQt5 import sip
        dialog = self.dialogs.get(LiveDialog)
        if dialog is not None and not sip.isdeleted(dialog):
            dialog.refresh_capabilities()

    def open_parameters_group(self, identifier):
        dialog = self.open_dialog(ParametersDialog)
        dialog.request_group(identifier)
        return dialog

    def poll_jobs(self):
        if self.polling:
            return
        self.polling = True
        BackgroundTask(self, self.backend.jobs, self.jobs_loaded,
                       lambda error: self.notice.setText("任务状态暂不可读：" + str(error)),
                       lambda: setattr(self, "polling", False))

    def jobs_loaded(self, jobs):
        newly_failed = []
        capture_notifications = []
        for job in jobs:
            old = self.known_jobs.get(job["id"])
            if job.get("kind") == "capture":
                status = job.get("status")
                if status not in TERMINAL or (old is not None and old != status):
                    self.capture_status(job)
                if old is not None and old != status and status in ("COMPLETE", "FAILED", "CANCELLED"):
                    capture_notifications.append(job)
            elif not self.first_poll and job.get("status") == "FAILED" and old != "FAILED":
                newly_failed.append(job)
            self.known_jobs[job["id"]] = job.get("status")
            self.known_job_details[job["id"]] = dict(job)
        self.first_poll = False
        if not self.recovery_checked and hasattr(self.backend, "list_recoverable_maps"):
            self.recovery_checked = True
            BackgroundTask(self, self.backend.list_recoverable_maps, self.recovery_loaded,
                           lambda error: self.notice.setText("临时地图检查未完成：" + str(error)))
        for job in newly_failed:
            QtWidgets.QMessageBox.warning(self, "任务未完成", "{}\n{}\n请在日志窗口查看具体原因。".format(
                job.get("label", job["id"]), job.get("error") or "任务退出失败"))
        for job in capture_notifications:
            self.capture_status(job, notify=True)

    def recovery_loaded(self, rows):
        if rows:
            self.notice.setText("发现 {} 个临时地图记录，可在“实时建图 → 恢复未完成地图”中查看并处理。".format(len(rows)))

    def closeEvent(self, event):
        if BackgroundTask.live_tasks:
            self.notice.setText("正在完成文件读取或操作，请稍后再关闭面板。")
            event.ignore()
            return
        active = [self.known_job_details.get(identifier, {}) for identifier, status in self.known_jobs.items() if status not in TERMINAL]
        managed_active = [job for job in active if job.get("kind") != "offline" or job.get("execution_mode") != "detached_offline"]
        if managed_active:
            QtWidgets.QMessageBox.information(self, "任务仍在运行",
                "主面板将最小化并继续管理任务。请在 RViz 中结束实时/录制任务，或在离线融合中取消队列，再退出主面板。")
            self.showMinimized()
            event.ignore()
            return
        from PyQt5 import sip
        parameter_window = self.dialogs.get(ParametersDialog)
        if parameter_window is not None and not sip.isdeleted(parameter_window) and parameter_window.editing:
            answer = QtWidgets.QMessageBox.question(self, "设备参数尚未保存", "退出面板并放弃尚未保存的参数修改？",
                                                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
            if answer != QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
        self.job_timer.stop()
        super().closeEvent(event)
        if event.isAccepted():
            self.exitRequested.emit()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Wheelchair desktop panel")
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[2])
    args, qt_args = parser.parse_known_args(argv)
    application = QtWidgets.QApplication([sys.argv[0]] + qt_args)
    application.setApplicationName("Wheelchair Panel")
    application.setOrganizationName("Wheelchair")
    apply_theme(application)
    runtime = QtCore.QStandardPaths.writableLocation(QtCore.QStandardPaths.RuntimeLocation)
    runtime_dir = Path(runtime) if runtime else Path(tempfile.gettempdir())
    runtime_dir.mkdir(parents=True, exist_ok=True)
    instance_lock = QtCore.QLockFile(str(runtime_dir/("wheelchair-panel-" + getpass.getuser() + ".lock")))
    instance_lock.setStaleLockTime(0)
    if not instance_lock.tryLock(0):
        QtWidgets.QMessageBox.information(None, "面板已经运行", "当前用户已有工作面板实例，请使用已经打开的窗口。")
        return 1
    application._instance_lock = instance_lock
    try:
        from .backend import PanelBackend
        backend = PanelBackend(args.project_root)
        window = MainWindow(backend)
        window.exitRequested.connect(application.quit)
        window.show()
        return application.exec_()
    except Exception as error:
        QtWidgets.QMessageBox.critical(None, "面板启动失败", "{}\n\n请检查项目路径、存储配置和依赖。".format(error))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
