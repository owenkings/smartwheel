"""Task-oriented windows for the wheelchair desktop panel."""
import copy
import math
from datetime import datetime
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets, sip

from .ui_common import (PanelDialog, PanelScrollArea, PathPicker, FilePicker, BackgroundTask, ComboBox,
                        button, combo, hint, human_bytes, configure_scroll_view,
                        set_readonly_table, status_text, TERMINAL, local_open)
from .ui_common import result_label


def sides_combo():
    return combo([("双雷达", "all"), ("仅左雷达（调试）", "left"), ("仅右雷达（调试）", "right")])


def _mark_action_row(layout):
    """Leave final actions clear when a nearby selector opens its popup."""
    for index in range(layout.count()):
        widget = layout.itemAt(index).widget()
        if isinstance(widget, QtWidgets.QAbstractButton):
            widget.setProperty("popupFooter", True)


class LiveDialog(PanelDialog):
    def __init__(self, backend, parent=None):
        super().__init__(backend, "实时融合", parent, (800, 690))
        self.form_scroll = PanelScrollArea()
        content = QtWidgets.QWidget()
        body = QtWidgets.QVBoxLayout(content)
        body.setContentsMargins(0, 0, 8, 0)
        body.setSpacing(14)
        self.form_scroll.setWidget(content)
        self.layout.addWidget(self.form_scroll, 1)
        body.addWidget(hint("确认后打开统一 RViz 窗口并先显示当前帧。实时建图模式在窗口内点击“开始建图”；再次点击“停止并保存”后恢复预览，再选择保存位置。切换设备时会短暂显示“正在切换”。"))
        form = QtWidgets.QGridLayout()
        form.setVerticalSpacing(14)
        form.setHorizontalSpacing(12)
        form.setColumnStretch(1, 1)
        self.mode = combo([("预览实时数据", "preview"), ("实时建图", "mapping")])
        self.sides = sides_combo()
        self.cloud = combo([("raw 原始点云", "raw"), ("filtered 过滤点云", "filtered")])
        self.estimator = combo([("官方 EKF", "robot_localization"), ("五状态 EKF", "five_state")])
        self.estimator.setCurrentIndex(self.estimator.findData("five_state"))
        self.motion = QtWidgets.QCheckBox("启用运动修正")
        self.motion_reason = hint("正在核实在线修正条件…")
        self.noise = combo([("当前离散噪声模型", "legacy"), ("连续白加速度 PSD", "white_acceleration")])
        self.geometry = QtWidgets.QCheckBox("启用几何反馈")
        self.geometry.setObjectName("live_geometry")
        self.geometry_reason = hint("")
        self.availability = hint("")
        for label, widget in (("运行模式", self.mode), ("雷达", self.sides), ("点云输入", self.cloud),
                              ("状态估计", self.estimator), ("运动修正", self.motion),
                              ("过程噪声", self.noise), ("几何反馈", self.geometry)):
            row_index = form.rowCount()
            form.addWidget(QtWidgets.QLabel(label), row_index, 0)
            form.addWidget(widget, row_index, 1)
            if widget is self.motion:
                form.addWidget(self.motion_reason, row_index + 1, 1)
        form.addWidget(self.geometry_reason, form.rowCount(), 1)
        calibration_row = QtWidgets.QHBoxLayout()
        calibration_row.addWidget(button("在线校正设置…", self.open_calibration))
        calibration_row.addWidget(button("刷新生效条件", self.refresh_capabilities))
        calibration_row.addWidget(button("恢复未完成地图…", self.open_recovery))
        calibration_row.addStretch()
        form.addLayout(calibration_row, form.rowCount(), 1)
        body.addLayout(form)
        body.addWidget(self.availability)
        body.addStretch()
        row = QtWidgets.QHBoxLayout()
        row.addStretch()
        row.addWidget(button("取消", self.close))
        self.start_button = button("确认并打开 RViz", self.start, primary=True)
        row.addWidget(self.start_button)
        _mark_action_row(row)
        self.layout.addLayout(row)
        self.estimator.currentIndexChanged.connect(self.refresh_options)
        self.mode.currentIndexChanged.connect(self.refresh_options)
        self.sides.currentIndexChanged.connect(self.refresh_options)
        self.capabilities = {}
        self.defaults_loaded = False
        self.starting = False
        self.saved_options = copy.deepcopy(getattr(parent, "last_live_options", {}))
        self.refresh_capabilities()
        self.refresh_options()

    def set_capabilities(self, capabilities):
        self.capabilities = capabilities
        self.refresh_options()

    def refresh_capabilities(self):
        if not self.defaults_loaded:
            self.async_call(lambda: (self.backend.capabilities(), self.backend.device_parameters()), self.initial_conditions)
        else:
            self.async_call(self.backend.capabilities, self.set_capabilities)

    def initial_conditions(self, values):
        capabilities, parameters = values
        if self.defaults_loaded:
            self.set_capabilities(capabilities)
            return
        self.capabilities = capabilities
        self.defaults_loaded = True
        algorithms = next((group.get("value", {}) for group in parameters.get("groups", []) if group["id"] == "algorithms"), {})
        defaults = dict(capabilities.get("live_defaults", {}))
        defaults["estimator"] = algorithms.get("wheel_imu_estimator", "five_state")
        defaults.update(self.saved_options)
        # Restore selections only inside this panel session. Runtime conditions
        # below still clear unavailable combinations before a task is submitted.
        for key, control in (("mode", self.mode), ("sides", self.sides), ("cloud", self.cloud),
                             ("estimator", self.estimator), ("process_noise", self.noise)):
            if key in defaults and control.findData(defaults[key]) >= 0:
                control.setCurrentIndex(control.findData(defaults[key]))
        self.motion.setChecked(bool(defaults.get("motion_correction", False)))
        self.geometry.setChecked(bool(defaults.get("geometry", False)))
        self.refresh_options()

    def open_calibration(self):
        if self.parent() and hasattr(self.parent(), "open_parameters_group"):
            self.parent().open_parameters_group("live_motion")
        else:
            dialog = ParametersDialog(self.backend, self)
            dialog.request_group("live_motion")
            dialog.show()

    def open_recovery(self):
        dialog = RecoverMapsDialog(self.backend, self)
        dialog.show()

    def refresh_options(self):
        official = self.estimator.currentData() == "robot_localization"
        mapping = self.mode.currentData() == "mapping"
        if official or not mapping:
            self.geometry.setChecked(False)
            if self.noise.currentData() == "white_acceleration":
                self.noise.setCurrentIndex(0)
        psd_item = self.noise.model().item(self.noise.findData("white_acceleration"))
        psd_item.setEnabled(not official and mapping)
        psd_item.setToolTip("连续白加速度 PSD 当前仅支持五状态 EKF 实时建图。")
        geometry_capability = self.capabilities.get("live_geometry", {})
        geometry_side = geometry_capability.get("by_sides", {}).get(self.sides.currentData(), geometry_capability)
        geometry_enabled = bool(geometry_side.get("enabled", False))
        self.geometry.setEnabled(not official and mapping and geometry_enabled)
        if not self.geometry.isEnabled():
            self.geometry.setChecked(False)
        reason = "官方 EKF 的几何反馈尚未实现；连续 PSD 当前仅支持五状态 EKF。" if official else "五状态几何纠偏修正发布位姿；实时建图模式生效。"
        if not mapping:
            reason += " 预览模式不启用建图修正。"
        elif not official and not geometry_enabled:
            reason += " " + str(geometry_side.get("reason", "当前雷达缺少可用外参条件。"))
        self.geometry.setToolTip(reason)
        self.geometry_reason.setText(reason)
        live_motion = self.capabilities.get("live_motion_correction", {})
        motion_side = live_motion.get("by_sides", {}).get(self.sides.currentData(), live_motion)
        motion_available = bool(motion_side.get("enabled", False)) and mapping
        self.motion.setEnabled(motion_available)
        if not motion_available:
            self.motion.setChecked(False)
        self.motion_reason.setText(str(motion_side.get("reason", "在线修正需要匹配当前设备的有效校正参数。")) if mapping else "预览模式不启用运动修正；切换实时建图后核实设备校正参数。")
        live = self.capabilities.get("live", {})
        mapping_capability = self.capabilities.get("live_mapping", {})
        mapping_side = mapping_capability.get("by_sides", {}).get(self.sides.currentData(), mapping_capability)
        mapping_available = bool(mapping_side.get("enabled", False))
        if not self.defaults_loaded:
            self.availability.setText("正在读取当前算法默认值与设备生效条件…")
        elif mapping and not mapping_available:
            self.availability.setText("当前不能启动实时建图：" + str(mapping_side.get("reason", "正在核实当前设备外参与建图条件…")))
        else:
            self.availability.setText(str(live.get("note", "预览不生成累计地图；建图显示累计 3D 点云与 2D 栅格地图。")))
        self.start_button.setEnabled(self.defaults_loaded and not self.starting and (not mapping or mapping_available))
        self.start_button.setToolTip("" if self.defaults_loaded and (not mapping or mapping_available) else self.availability.text())

    def options(self):
        return {"mode": self.mode.currentData(), "sides": self.sides.currentData(),
                "cloud": self.cloud.currentData(), "estimator": self.estimator.currentData(),
                "motion_correction": self.motion.isChecked(), "process_noise": self.noise.currentData(),
                "geometry": self.geometry.isChecked()}

    def start(self):
        if not self.start_button.isEnabled():
            return
        options = self.options()
        self.starting = True
        self.start_button.setEnabled(False)
        self.start_button.setText("正在启动…")
        self.async_call(lambda: self.backend.start_live(options), self.started,
                        finished=self.start_finished)

    def start_finished(self):
        self.starting = False
        self.refresh_options()
        self.start_button.setText("确认并打开 RViz")

    def started(self, job):
        if self.parent() is not None:
            self.parent().last_live_options = self.options()
        if self.parent() and hasattr(self.parent(), "task_started"):
            self.parent().task_started(job)
        self.accept()

    def closeEvent(self, event):
        if self.parent() is not None and self.defaults_loaded:
            self.parent().last_live_options = self.options()
        super().closeEvent(event)


class CaptureDialog(PanelDialog):
    def __init__(self, backend, parent=None):
        super().__init__(backend, "数据录制", parent, (830, 510))
        self.setMinimumSize(790, 490)
        self.ready = False
        self.starting = False
        self.layout.addWidget(hint("为这次录制设置名称，确认后打开一个包含所选雷达的三栏预览窗口。"))
        self.destination = PathPicker()
        self.destination.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.sides = sides_combo()
        self.sides.setFixedWidth(215)
        self.prefix = QtWidgets.QLineEdit("capture")
        self.prefix.setObjectName("capture_name_prefix")
        self.prefix.setPlaceholderText("例如：走廊_往返")
        self.prefix.setMaxLength(80)
        name_row = QtWidgets.QHBoxLayout()
        name_row.addWidget(self.prefix, 1)
        name_row.addSpacing(12)
        name_row.addWidget(QtWidgets.QLabel("录制雷达"))
        name_row.addWidget(self.sides)
        suffix_row = QtWidgets.QHBoxLayout()
        self.suffixes = {}
        for key, label in (("year", "年份 YYYY"), ("date", "日期 MMDD"), ("time", "时间 HHMMSS")):
            checkbox = QtWidgets.QCheckBox(label)
            checkbox.setChecked(True)
            checkbox.toggled.connect(self.update_name)
            self.suffixes[key] = checkbox
            suffix_row.addWidget(checkbox)
        suffix_row.addStretch()
        form = QtWidgets.QFormLayout()
        form.setVerticalSpacing(14)
        form.addRow("录制保存路径", self.destination)
        form.addRow("数据名称前缀", name_row)
        form.addRow("名称后缀", suffix_row)
        self.layout.addLayout(form)
        self.name_preview = hint("")
        self.layout.addWidget(self.name_preview)
        self.layout.addWidget(hint("名称以 _ 连接；按开始录制时的本地时间命名。同名时自动追加 _02、_03，保留已有数据。"))
        self.layout.addWidget(hint("此处更改仅作用于本次录制；默认路径在“数据与存储”中管理。"))
        self.layout.addWidget(hint("录制中在统一 RViz 窗口顶部点击“结束录制”，或正常关闭该窗口；主面板会提示收尾状态及最终目录，收到完成通知后再拔出存储设备。"))
        self.layout.addStretch()
        row = QtWidgets.QHBoxLayout()
        row.addStretch()
        row.addWidget(button("取消", self.close))
        self.start_button = button("确认并开始录制", self.start, primary=True)
        self.start_button.setEnabled(False)
        row.addWidget(self.start_button)
        _mark_action_row(row)
        self.layout.addLayout(row)
        self.prefix.textChanged.connect(self.update_name)
        self.name_timer = QtCore.QTimer(self)
        self.name_timer.setInterval(1000)
        self.name_timer.timeout.connect(self.update_name)
        self.name_timer.start()
        self.update_name()
        self.async_call(backend.settings, self.loaded)

    def loaded(self, settings):
        self.destination.set_path(settings["recording_root"])
        self.default_recording_root = settings["recording_root"]
        self.ready = True
        self.update_name()

    def name_options(self):
        return dict(prefix=self.prefix.text(), **{key:box.isChecked() for key,box in self.suffixes.items()})

    def update_name(self, *_):
        from wc_runtime.capture_names import recording_folder_name
        try:
            name = recording_folder_name(self.name_options())
        except ValueError as error:
            self.name_preview.setText(str(error))
            self.start_button.setEnabled(False)
            return
        self.name_preview.setText("文件夹预览：" + name)
        self.start_button.setEnabled(self.ready and not self.starting)

    def update_default_settings(self, settings):
        if self.destination.path() == getattr(self, "default_recording_root", ""):
            self.destination.set_path(settings["recording_root"])
        self.default_recording_root = settings["recording_root"]

    def start(self):
        destination, sides = self.destination.path(), self.sides.currentData()
        if not destination:
            self.show_error("请选择录制保存路径。")
            return
        self.start_button.setEnabled(False)
        self.starting = True
        options = self.name_options()
        self.async_call(lambda: self.backend.start_capture(output_root=destination, sides=sides, name_options=options),
                        self.started, finished=self.start_finished)

    def start_finished(self):
        self.starting = False
        self.update_name()

    def started(self, job):
        if self.parent() and hasattr(self.parent(), "task_started"):
            self.parent().task_started(job)
        self.accept()


class RecoverMapsDialog(PanelDialog):
    def __init__(self, backend, parent=None):
        super().__init__(backend, "恢复未完成地图", parent, (980, 540))
        self.layout.addWidget(hint("只处理本项目明确记录的临时地图。确认会话已停止后，重新打开正常保存选择；不会启动传感器。"))
        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["会话", "状态 / 原因", "临时结果位置"])
        set_readonly_table(self.table)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setColumnWidth(0, 220)
        self.table.setColumnWidth(1, 290)
        self.layout.addWidget(self.table, 1)
        self.info = hint("正在查找保留的临时地图…")
        self.layout.addWidget(self.info)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(button("刷新", self.scan))
        row.addStretch()
        row.addWidget(button("关闭", self.close))
        self.restore_button = button("打开保存选择", self.restore, primary=True)
        self.restore_button.setEnabled(False)
        row.addWidget(self.restore_button)
        _mark_action_row(row)
        self.layout.addLayout(row)
        self.table.itemSelectionChanged.connect(self.selection_changed)
        self.scan()

    def scan(self):
        self.restore_button.setEnabled(False)
        self.async_call(self.backend.list_recoverable_maps, self.loaded,
                        lambda error: self.info.setText("读取失败：" + str(error)))

    def loaded(self, rows):
        self.table.setRowCount(len(rows))
        for index, row in enumerate(rows):
            for column, value in enumerate((row.get("id", Path(row["path"]).name), row.get("reason") or row.get("status", ""), row["path"])):
                item = QtWidgets.QTableWidgetItem(str(value))
                item.setData(QtCore.Qt.UserRole, row)
                item.setToolTip(str(value))
                self.table.setItem(index, column, item)
        self.info.setText("发现 {} 个临时地图记录。只有通过已停止状态校验的会话可处理。".format(len(rows)) if rows else "没有待处理的临时地图。")
        self.selection_changed()

    def selection_changed(self):
        rows = self.table.selectionModel().selectedRows()
        selected = self.table.item(rows[0].row(), 0).data(QtCore.Qt.UserRole) if rows else None
        self.restore_button.setEnabled(bool(selected and selected.get("recoverable", False)))

    def restore(self):
        rows = self.table.selectionModel().selectedRows()
        if not rows:
            return
        selected = self.table.item(rows[0].row(), 0).data(QtCore.Qt.UserRole)
        if not selected.get("recoverable", False):
            return
        self.restore_button.setEnabled(False)
        self.async_call(lambda: self.backend.recover_map(selected["path"]), self.started,
                        finished=self.selection_changed)

    def started(self, job):
        owner = self.parent()
        while owner is not None:
            if hasattr(owner, "task_started"):
                owner.task_started(job)
                break
            owner = owner.parent()
        self.accept()


class SelectionDetail(QtWidgets.QTextBrowser):
    """Selectable names that wrap even when a recording ID has no spaces."""

    def __init__(self, text="", parent=None):
        super().__init__(parent)
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setWordWrapMode(QtGui.QTextOption.WrapAtWordBoundaryOrAnywhere)
        self.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse | QtCore.Qt.TextSelectableByKeyboard)
        self.setStyleSheet("QTextBrowser { background: transparent; border: none; padding: 0; color: #58687b; }")
        self.document().setDocumentMargin(0)
        self.setMinimumWidth(0)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.setPlainText(text)

    def minimumSizeHint(self):
        return QtCore.QSize(0, self.fontMetrics().height())

    def setPlainText(self, text):
        super().setPlainText(str(text))
        self.fit_height()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fit_height()

    def fit_height(self):
        self.document().setTextWidth(max(1, self.viewport().width()))
        height = max(self.fontMetrics().height(), math.ceil(self.document().size().height()) + 2)
        if self.height() != height:
            self.setFixedHeight(height)


class SchemeTable(QtWidgets.QTreeWidget):
    """Keep all flat scheme rows in the surrounding page's scroll flow."""

    def __init__(self, parent=None):
        super().__init__(parent)
        configure_scroll_view(self)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self._fit_timer = QtCore.QTimer(self)
        self._fit_timer.setSingleShot(True)
        self._fit_timer.timeout.connect(self.fit_contents)
        self.model().rowsInserted.connect(self.schedule_fit)
        self.model().rowsRemoved.connect(self.schedule_fit)
        self.model().modelReset.connect(self.schedule_fit)
        self.header().sectionResized.connect(self.schedule_fit)
        self.currentItemChanged.connect(self.reveal_current)
        self.scroll_area = None
        self.sticky_header = None

    def attach_page(self, area):
        self.scroll_area = area
        self.sticky_header = QtWidgets.QHeaderView(QtCore.Qt.Horizontal, area.viewport())
        self.sticky_header.setObjectName("offline_schemes_sticky_header")
        self.sticky_header.setModel(self.model())
        self.sticky_header.setDefaultAlignment(self.header().defaultAlignment())
        self.sticky_header.setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
        self.sticky_header.setSectionsClickable(False)
        self.sticky_header.setSectionsMovable(False)
        self.sticky_header.setFocusPolicy(QtCore.Qt.NoFocus)
        self.sticky_header.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.sticky_header.hide()
        area.verticalScrollBar().valueChanged.connect(self.update_sticky_header)
        self.horizontalScrollBar().valueChanged.connect(self.update_sticky_header)
        area.viewport().installEventFilter(self)

    def eventFilter(self, watched, event):
        if (event.type() in (QtCore.QEvent.Resize, QtCore.QEvent.Show)
                and self.scroll_area is not None and not sip.isdeleted(self.scroll_area)
                and watched is self.scroll_area.viewport()):
            self.schedule_fit()
        return super().eventFilter(watched, event)

    def update_sticky_header(self, *_):
        if (self.sticky_header is None or sip.isdeleted(self.sticky_header)
                or self.scroll_area is None or sip.isdeleted(self.scroll_area)):
            return
        viewport = self.scroll_area.viewport()
        original = self.header()
        position = original.mapTo(viewport, QtCore.QPoint())
        body_bottom = self.viewport().mapTo(viewport, self.viewport().rect().bottomLeft()).y()
        visible = self.isVisible() and position.y() < 0 and body_bottom > original.height()
        self.sticky_header.setVisible(visible)
        if visible:
            self.sticky_header.setGeometry(position.x(), 0, original.width(), original.height())
            for column in range(original.count()):
                self.sticky_header.resizeSection(column, original.sectionSize(column))
            self.sticky_header.setOffset(original.offset())
            self.sticky_header.raise_()

    def schedule_fit(self, *_):
        self._fit_timer.start(0)

    def fit_contents(self):
        self.ensurePolished()
        rows_height = sum(max(28, self.sizeHintForRow(row)) for row in range(self.topLevelItemCount()))
        header_height = self.header().sizeHint().height()
        horizontal_height = (self.horizontalScrollBar().sizeHint().height()
                             if self.header().length() > self.viewport().width() else 0)
        height = header_height + rows_height + horizontal_height + 2 * self.frameWidth() + 4
        if self.height() != height:
            self.setFixedHeight(height)
        self.update_sticky_header()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "_fit_timer"):
            self.schedule_fit()

    def reveal_current(self, item, _previous):
        # Arrow-key navigation must reveal a row through the outer page too.
        area = self.parentWidget()
        while area is not None and not isinstance(area, PanelScrollArea):
            area = area.parentWidget()
        if area is not None and item is not None:
            rect = self.visualItemRect(item)
            point = self.viewport().mapTo(area.widget(), rect.center())
            area.ensureVisible(point.x(), point.y(), 0, 32)
            if self.sticky_header is not None and self.sticky_header.isVisible():
                row_top = self.viewport().mapTo(area.viewport(), rect.topLeft()).y()
                clear_top = self.sticky_header.height() + 4
                if row_top < clear_top:
                    bar = area.verticalScrollBar()
                    bar.setValue(bar.value() - (clear_top - row_top))


class OfflineDialog(PanelDialog):
    def __init__(self, backend, parent=None):
        super().__init__(backend, "离线融合", parent, (1020, 790))
        self.plan = None
        self.job_id = None
        self.variants = []
        self.recordings = {}
        self.scan_generation = 0
        self.polling = False
        self.stack = QtWidgets.QStackedWidget()
        self.layout.addWidget(self.stack)
        self.setup_page = QtWidgets.QWidget()
        page_layout = QtWidgets.QVBoxLayout(self.setup_page)
        page_layout.setContentsMargins(0, 0, 0, 0)
        page_layout.setSpacing(12)
        # Keep the final actions visible on shorter screens. The settings above
        # them scroll instead of squeezing selectors and their selected names.
        self.setup_scroll = PanelScrollArea()
        setup_content = QtWidgets.QWidget()
        setup = QtWidgets.QVBoxLayout(setup_content)
        setup.setContentsMargins(0, 0, 8, 0)
        setup.setSpacing(10)
        self.setup_scroll.setWidget(setup_content)
        self.setup_scroll.viewport().setAutoFillBackground(False)
        setup_content.setAutoFillBackground(False)
        page_layout.addWidget(self.setup_scroll, 1)
        self.recording_root = PathPicker()
        self.recording_root.changed.connect(self.scan)
        self.dataset = ComboBox()
        self.dataset.setObjectName("offline_dataset")
        self.dataset_detail = SelectionDetail("尚未选择录制数据")
        self.dataset_detail.setObjectName("offline_dataset_detail")
        self.dataset.currentIndexChanged.connect(self.dataset_changed)
        self.sides = combo([("双雷达点云", "all"), ("仅左雷达点云", "left"), ("仅右雷达点云", "right")])
        self.sides.setObjectName("offline_sides")
        self.sides.setEnabled(False)
        self.sides.currentIndexChanged.connect(self.sources_changed)
        self.source_note = hint("选择录制数据后显示可用雷达。")
        self.output_root = PathPicker()
        form = QtWidgets.QFormLayout()
        form.setVerticalSpacing(8)
        form.addRow("录制根目录", self.recording_root)
        data_row = QtWidgets.QHBoxLayout()
        data_row.addWidget(self.dataset, 1)
        data_row.addWidget(button("刷新数据", self.scan))
        form.addRow("选择录制数据", data_row)
        form.addRow("", self.dataset_detail)
        form.addRow("使用录制中的雷达", self.sides)
        form.addRow("", self.source_note)
        form.addRow("融合结果路径", self.output_root)
        setup.addLayout(form)
        self.scan_label = hint("正在读取默认路径…")
        setup.addWidget(self.scan_label)
        self.recorded_note = hint("算法开关取本次选择；外参、轮参数和基础算法数值使用录包内冻结配置。修改当前设备参数不改写历史录制。")
        setup.addWidget(self.recorded_note)
        diagnostic = QtWidgets.QHBoxLayout()
        self.mechanical_initial = QtWidgets.QCheckBox("使用机械初值（离线实验）")
        self.mechanical_initial.setToolTip("仅在缺少可用外参时显式使用机械初值；不会修改实时外参配置。结果应视为实验。")
        self.allow_partial = QtWidgets.QCheckBox("允许处理未完整结束的录制（诊断）")
        self.allow_partial.setToolTip("仅用于诊断已有数据，不会将不完整录制标记为完整。")
        diagnostic.addWidget(self.mechanical_initial)
        diagnostic.addWidget(self.allow_partial)
        diagnostic.addStretch()
        setup.addLayout(diagnostic)
        select_row = QtWidgets.QHBoxLayout()
        select_row.addWidget(QtWidgets.QLabel("选择对照方案"))
        select_row.addStretch()
        select_row.addWidget(button("全选有效方案", lambda: self.select_variants(True)))
        select_row.addWidget(button("清空选择", lambda: self.select_variants(False)))
        setup.addLayout(select_row)
        self.variant_table = SchemeTable()
        self.variant_table.setHeaderLabels(["方案", "EKF", "点云来源", "点云筛选", "过程噪声", "修正 / 生效条件"])
        self.variant_table.setRootIsDecorated(False)
        self.variant_table.setAlternatingRowColors(True)
        self.variant_table.itemChanged.connect(self.update_count)
        for index, width in enumerate((135, 90, 95, 90, 105)):
            self.variant_table.setColumnWidth(index, width)
        setup.addWidget(self.variant_table)
        self.variant_table.attach_page(self.setup_scroll)
        setup.addStretch()
        self.count_label = hint("0 个有效方案")
        history_section = QtWidgets.QVBoxLayout()
        history_section.setSpacing(7)
        history_section.addWidget(QtWidgets.QLabel("查看已有融合批次"))
        self.history = ComboBox()
        self.history.setObjectName("offline_history")
        self.history.setProperty("popupAbove", True)
        self.history.setToolTip("选择已有的当前或历史批次，只读取队列状态，不重新执行融合。")
        self.history_detail = SelectionDetail("暂无已有批次")
        self.history_detail.setObjectName("offline_history_detail")
        self.history.currentIndexChanged.connect(self.history_changed)
        history_actions = QtWidgets.QHBoxLayout()
        history_actions.addWidget(self.history, 1)
        self.history_button = button("查看已有队列", self.open_history)
        self.history_button.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.history_button.setEnabled(False)
        history_actions.addWidget(self.history_button)
        history_section.addLayout(history_actions)
        history_section.addWidget(self.history_detail)
        page_layout.addLayout(history_section)
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(button("关闭", self.close))
        controls.addWidget(self.count_label, 1)
        self.start_button = button("确认方案并开始融合", self.make_plan, primary=True)
        self.start_button.setEnabled(False)
        controls.addWidget(self.start_button)
        _mark_action_row(controls)
        page_layout.addLayout(controls)
        self.stack.addWidget(self.setup_page)
        self.queue_page = QtWidgets.QWidget()
        queue = QtWidgets.QVBoxLayout(self.queue_page)
        queue.setContentsMargins(0, 0, 0, 0)
        self.queue_label = QtWidgets.QLabel("等待启动")
        self.queue_label.setWordWrap(True)
        queue.addWidget(self.queue_label)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 100)
        queue.addWidget(self.progress)
        self.queue_table = QtWidgets.QTableWidget(0, 3)
        self.queue_table.setHorizontalHeaderLabels(["任务", "状态", "结果位置 / 失败原因"])
        set_readonly_table(self.queue_table)
        self.queue_table.setColumnWidth(0, 330)
        self.queue_table.setColumnWidth(1, 100)
        queue.addWidget(self.queue_table, 1)
        self.queue_log = QtWidgets.QPlainTextEdit()
        self.queue_log.setReadOnly(True)
        self.queue_log.setMaximumBlockCount(1200)
        self.queue_log.setMaximumHeight(175)
        queue.addWidget(self.queue_log)
        controls = QtWidgets.QHBoxLayout()
        self.cancel_button = button("取消剩余任务", self.cancel, danger=True)
        self.new_button = button("新建融合任务", self.new_task)
        self.new_button.setEnabled(False)
        controls.addWidget(self.cancel_button)
        controls.addWidget(self.new_button)
        controls.addStretch()
        controls.addWidget(button("关闭窗口", self.close))
        _mark_action_row(controls)
        queue.addLayout(controls)
        queue.addWidget(hint("关闭此窗口不会取消任务。可从“离线融合”重新查看队列，从“日志”查看完整日志。"))
        self.stack.addWidget(self.queue_page)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(1200)
        self.timer.timeout.connect(self.poll)
        self.log_offset = 0
        self.async_call(lambda: (backend.settings(), backend.capabilities(), backend.jobs()), self.loaded)

    def loaded(self, values):
        settings, capabilities, jobs = values
        self.default_settings = dict(settings)
        self.recording_root.set_path(settings["recording_root"])
        self.output_root.set_path(settings["results_root"])
        self.variants = capabilities.get("offline_variants", [])
        self.variant_table.blockSignals(True)
        self.variant_table.clear()
        for variant in self.variants:
            estimator = "官方" if variant.get("estimator") in ("official", "robot_localization") else "五状态"
            pieces = []
            if variant.get("motion_correction"):
                pieces.append("运动修正")
            if variant.get("process_noise") == "white_acceleration":
                pieces.append("PSD")
            if variant.get("geometry"):
                pieces.append("几何反馈")
            if variant.get("reason"):
                pieces.append(str(variant["reason"]))
            name = "几何反馈" if variant.get("geometry") else "PSD 候选" if variant.get("process_noise") == "white_acceleration" else "运动修正" if variant.get("motion_correction") else "基线"
            item = QtWidgets.QTreeWidgetItem([name, estimator,
                                             variant.get("cloud", ""), "开" if variant.get("filtering") else "关",
                                             "连续 PSD" if variant.get("process_noise") == "white_acceleration" else "当前模型",
                                             " / ".join(pieces) or "原始运动"])
            item.setToolTip(0, variant.get("label", variant["id"]))
            item.setSizeHint(0, QtCore.QSize(135, 28))
            item.setData(0, QtCore.Qt.UserRole, variant["id"])
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(0, QtCore.Qt.Unchecked)
            enabled = variant.get("enabled", True)
            if not enabled:
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEnabled)
                item.setToolTip(0, variant.get("reason", "此组合尚未实现"))
            self.variant_table.addTopLevelItem(item)
        # Even if a backend omits disabled variants, keep the required visible entry.
        if not any(v.get("estimator") in ("official", "robot_localization") and v.get("geometry") for v in self.variants):
            item = QtWidgets.QTreeWidgetItem(["几何反馈", "官方", "raw / filtered", "开", "当前模型", "尚未实现"])
            item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEnabled)
            item.setToolTip(0, "官方 EKF 的几何反馈尚未实现，此选项暂不可选。")
            self.variant_table.addTopLevelItem(item)
        self.variant_table.blockSignals(False)
        self.variant_table.schedule_fit()
        self.update_count()
        self.scan()
        self.refresh_history(jobs)
        active = [job for job in jobs if job.get("kind") == "offline" and job.get("status") not in TERMINAL]
        if active:
            self.job_id = active[-1]["id"]
            self.stack.setCurrentWidget(self.queue_page)
            self.show_job(active[-1])
            self.timer.start()

    def refresh_history(self, jobs):
        previous = self.history.currentData()
        self.history.blockSignals(True)
        self.history.clear()
        for job in reversed(jobs):
            if job.get("kind") != "offline":
                continue
            text = "{} · {} · {}".format(job.get("label", job["id"]), status_text(job.get("status")), job_time(job.get("started_at")))
            self.history.addItem(text, job["id"])
            self.history.setItemData(self.history.count()-1, text + "\n" + str(job.get("output") or ""), QtCore.Qt.ToolTipRole)
        index = self.history.findData(previous)
        if index >= 0:
            self.history.setCurrentIndex(index)
        self.history_button.setEnabled(self.history.count() > 0)
        if not self.history.count():
            self.history.addItem("暂无已有批次", None)
        self.history.blockSignals(False)
        self.history_changed()

    def history_changed(self, *_):
        self.history_detail.setPlainText(self.history.currentText() or "暂无已有批次")
        self.history_detail.setToolTip(self.history.currentData(QtCore.Qt.ToolTipRole) or "")
        self.history_detail.setVisible(bool(self.history.currentData()))

    def dataset_changed(self, *_):
        self.dataset_detail.setPlainText(self.dataset.currentText() or "尚未选择录制数据")
        self.dataset_detail.setToolTip(str(self.dataset.currentData() or ""))
        record = self.recordings.get(self.dataset.currentData(), {})
        available = record.get("available_sides", [])
        previous = self.sides.currentData()
        self.sides.blockSignals(True)
        valid = []
        for index, side in enumerate(("all", "left", "right")):
            needed = ["left", "right"] if side == "all" else [side]
            enabled = not record.get("source_inventory_error") and all(value in available for value in needed)
            item = self.sides.model().item(index)
            item.setEnabled(enabled)
            item.setToolTip("" if enabled else "该录制中缺少对应雷达的有效点云消息")
            if enabled:
                valid.append(side)
        chosen = previous if previous in valid else (valid[0] if valid else None)
        self.sides.setCurrentIndex(self.sides.findData(chosen))
        self.sides.setEnabled(bool(valid))
        self.sides.blockSignals(False)
        self.sources_changed()

    def sources_changed(self, *_):
        record = self.recordings.get(self.dataset.currentData(), {})
        side = self.sides.currentData()
        if record.get("source_inventory_error"):
            self.source_note.setText("无法核实录包中的点云：" + str(record["source_inventory_error"]))
        elif side:
            self.source_note.setText("{}；只筛选此次融合使用的点云，原录包保持不变。".format(self.sides.currentText()))
        else:
            self.source_note.setText("请选择包含有效点云的录制数据。")
        selected = ["left", "right"] if side == "all" else [side] if side else []
        streams = record.get("available_clouds_by_side", {})
        variants = {variant["id"]: variant for variant in self.variants}
        if hasattr(self, "variant_table"):
            self.variant_table.blockSignals(True)
            for index in range(self.variant_table.topLevelItemCount()):
                item = self.variant_table.topLevelItem(index)
                variant = variants.get(item.data(0, QtCore.Qt.UserRole))
                if not variant:
                    continue
                enabled = variant.get("enabled", True) and bool(selected) and all(variant.get("cloud") in streams.get(sensor, []) for sensor in selected)
                item.setFlags((item.flags() | QtCore.Qt.ItemIsEnabled) if enabled else (item.flags() & ~QtCore.Qt.ItemIsEnabled))
                if not enabled:
                    item.setCheckState(0, QtCore.Qt.Unchecked)
                item.setToolTip(0, variant.get("label", variant["id"]) if enabled else variant.get("reason") or "所选雷达缺少此方案需要的点云流")
            self.variant_table.blockSignals(False)
        if hasattr(self, "start_button"):
            self.update_count()

    def open_history(self):
        identifier = self.history.currentData()
        if not identifier:
            return
        self.history_button.setEnabled(False)
        self.async_call(lambda: (self.backend.jobs(), self.backend.read_log(identifier, 0)),
                        lambda value: self.history_loaded(identifier, value),
                        finished=lambda: self.history_button.setEnabled(self.history.count() > 0))

    def history_loaded(self, identifier, value):
        jobs, _ = value
        self.refresh_history(jobs)
        job = next((job for job in jobs if job.get("id") == identifier and job.get("kind") == "offline"), None)
        if job is None:
            self.show_error("该历史批次已不可用，请刷新后重试。")
            return
        self.job_id, self.plan, self.log_offset = identifier, None, 0
        self.queue_log.clear()
        self.stack.setCurrentWidget(self.queue_page)
        self.polled(value)
        if job.get("status") not in TERMINAL:
            self.timer.start()

    def update_default_settings(self, settings):
        previous = getattr(self, "default_settings", {})
        changed_recording = self.recording_root.path() == previous.get("recording_root")
        if changed_recording:
            self.recording_root.set_path(settings["recording_root"])
        if self.output_root.path() == previous.get("results_root"):
            self.output_root.set_path(settings["results_root"])
        self.default_settings = dict(settings)
        if changed_recording:
            self.scan()

    def scan(self, *_):
        root = self.recording_root.path()
        if not root:
            return
        self.scan_generation += 1
        generation = self.scan_generation
        self.recordings = {}
        self.dataset.clear()
        self.start_button.setEnabled(False)
        self.scan_label.setText("正在识别录制子文件夹…")
        self.async_call(lambda: self.backend.list_recordings(root),
                        lambda items: self.scanned(generation, items),
                        lambda error: self.scan_failed(generation, error))

    def scan_failed(self, generation, error):
        if generation == self.scan_generation:
            self.scan_label.setText(str(error))

    def scanned(self, generation, items):
        if generation != self.scan_generation:
            return
        for item in items:
            detail = "{} · {}{}".format(item.get("name", item["id"]), human_bytes(item.get("bytes", 0)),
                                      " · 录制未完整结束" if not item.get("complete", True) else "")
            self.recordings[item["path"]] = item
            self.dataset.addItem(detail, item["path"])
            self.dataset.setItemData(self.dataset.count()-1, item.get("reason", item["path"]), QtCore.Qt.ToolTipRole)
        self.scan_label.setText("识别到 {} 组有效录制。任务会保存使用的算法和配置。".format(len(items)))
        self.dataset_changed()

    def selected_variants(self):
        selected = []
        for i in range(self.variant_table.topLevelItemCount()):
            item = self.variant_table.topLevelItem(i)
            if item.flags() & QtCore.Qt.ItemIsEnabled and item.checkState(0) == QtCore.Qt.Checked:
                selected.append(item.data(0, QtCore.Qt.UserRole))
        return selected

    def select_variants(self, checked):
        self.variant_table.blockSignals(True)
        for i in range(self.variant_table.topLevelItemCount()):
            item = self.variant_table.topLevelItem(i)
            if item.flags() & QtCore.Qt.ItemIsEnabled:
                item.setCheckState(0, QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked)
        self.variant_table.blockSignals(False)
        self.update_count()

    def update_count(self, *_):
        count = len(self.selected_variants())
        self.count_label.setText("已选择 {} 个有效方案；确认前会显示实际任务数量和输出位置。".format(count))
        self.start_button.setEnabled(count > 0 and self.dataset.currentData() is not None and self.sides.isEnabled())

    def make_plan(self):
        dataset, output = self.dataset.currentData(), self.output_root.path()
        variants = self.selected_variants()
        if not dataset or not output or not variants:
            self.show_error("请选择录制数据、有效方案和结果保存路径。")
            return
        self.start_button.setEnabled(False)
        options = {"variants": variants, "mechanical_initial": self.mechanical_initial.isChecked(),
                   "allow_partial": self.allow_partial.isChecked(), "sides": self.sides.currentData()}
        self.async_call(lambda: self.backend.plan_offline(dataset, output, options),
                        self.confirm_plan, finished=self.update_count)

    def confirm_plan(self, plan):
        note = plan.get("recorded_configuration", {}).get("note")
        if note:
            self.recorded_note.setText(note)
        lines = ["录制：{}".format(plan["dataset"]), "输出：{}".format(plan["output_root"]),
                 "点云来源：{}".format({"all": "双雷达", "left": "仅左雷达", "right": "仅右雷达"}.get(plan.get("sides"), self.sides.currentText())),
                 "共 {} 个任务。".format(plan.get("task_count", len(plan.get("tasks", [])))), note or self.recorded_note.text(), ""]
        lines.extend("• " + task.get("label", task.get("id", "")) for task in plan.get("tasks", []))
        box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Question, "确认离线融合方案", "\n".join(lines[:12]),
                                    QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel, self)
        box.setDetailedText("\n".join(lines))
        box.setDefaultButton(QtWidgets.QMessageBox.Cancel)
        if box.exec_() != QtWidgets.QMessageBox.Ok:
            return
        self.plan = plan
        self.async_call(lambda: self.backend.start_offline(plan), self.started)

    def started(self, job):
        self.job_id = job["id"]
        self.log_offset = 0
        self.queue_log.clear()
        self.stack.setCurrentWidget(self.queue_page)
        self.show_job(job)
        self.timer.start()
        if self.parent() and hasattr(self.parent(), "task_started"):
            self.parent().task_started(job)

    def poll(self):
        if self.polling or not self.job_id:
            return
        self.polling = True
        job_id, offset = self.job_id, self.log_offset
        self.async_call(lambda: (self.backend.jobs(), self.backend.read_log(job_id, offset)),
                        self.polled, self.poll_failed, lambda: setattr(self, "polling", False))

    def poll_failed(self, error):
        self.queue_label.setText("暂时无法读取任务状态：" + str(error))

    def polled(self, values):
        jobs, log = values
        job = next((j for j in jobs if j["id"] == self.job_id), None)
        if job:
            self.show_job(job)
        if log.get("text"):
            self.queue_log.moveCursor(QtGui.QTextCursor.End)
            self.queue_log.insertPlainText(log["text"])
            self.queue_log.moveCursor(QtGui.QTextCursor.End)
        self.log_offset = log.get("offset", self.log_offset)

    def show_job(self, job):
        self.displayed_job = copy.deepcopy(job)
        status = job.get("status", "QUEUED")
        self.queue_label.setText("{} · {}{}".format(job.get("label", "离线融合"), status_text(status),
                                                  "\n" + str(job["error"]) if job.get("error") else ""))
        progress = job.get("progress")
        if progress is None:
            self.progress.setRange(0, 0 if status not in TERMINAL else 100)
        else:
            self.progress.setRange(0, 100)
            self.progress.setValue(round(float(progress) * 100) if float(progress) <= 1 else round(float(progress)))
        rows = job.get("task_states") or []
        if not rows and self.plan:
            rows = [{**task, "status": "计划项"} for task in self.plan.get("tasks", [])]
        self.queue_table.setRowCount(len(rows))
        for row, task in enumerate(rows):
            for col, text in enumerate((task.get("label", task.get("id", "")), status_text(task.get("status")),
                                        task.get("error") or task.get("output", ""))):
                self.queue_table.setItem(row, col, QtWidgets.QTableWidgetItem(str(text)))
        self.cancel_button.setEnabled(status not in TERMINAL and status != "STOPPING")
        self.new_button.setEnabled(status in TERMINAL)
        if status in TERMINAL:
            self.timer.stop()

    def cancel(self):
        answer = QtWidgets.QMessageBox.question(self, "取消离线融合", "停止当前离线任务并取消后续任务？已生成的结果会保留。",
                                                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                                QtWidgets.QMessageBox.No)
        if answer == QtWidgets.QMessageBox.Yes:
            job_id = self.job_id
            self.async_call(lambda: self.backend.cancel(job_id), self.show_job)

    def new_task(self):
        self.job_id = None
        self.timer.stop()
        self.stack.setCurrentWidget(self.setup_page)
        self.scan()
        self.async_call(self.backend.jobs, self.refresh_history)


def job_time(value):
    if value is None:
        return "尚未开始"
    try:
        stamp = datetime.fromtimestamp(float(value)).astimezone()
        return stamp.strftime("%Y-%m-%d %H:%M:%S %z")
    except (ValueError, TypeError, OverflowError, OSError):
        return str(value)


class LogsDialog(PanelDialog):
    def __init__(self, backend, parent=None):
        super().__init__(backend, "日志", parent, (1080, 700))
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("任务"))
        self.task = ComboBox()
        self.task.currentIndexChanged.connect(self.task_changed)
        row.addWidget(self.task, 1)
        self.follow = QtWidgets.QCheckBox("自动滚动")
        self.follow.setChecked(True)
        row.addWidget(self.follow)
        row.addWidget(button("打开日志所在目录", self.open_directory))
        self.layout.addLayout(row)
        source_row = QtWidgets.QHBoxLayout()
        source_row.addWidget(QtWidgets.QLabel("日志来源"))
        self.source = ComboBox()
        self.source.currentIndexChanged.connect(self.changed)
        source_row.addWidget(self.source, 1)
        self.layout.addLayout(source_row)
        self.location = hint("正在读取任务…")
        self.layout.addWidget(self.location)
        self.text = QtWidgets.QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(6000)
        font = QtGui.QFont("monospace")
        font.setStyleHint(QtGui.QFont.TypeWriter)
        self.text.setFont(font)
        self.layout.addWidget(self.text, 1)
        self.jobs_by_id = {}
        self.offsets = {}
        self.generation = 0
        self.refresh_requested = False
        self.log_path = None
        self.user_selected_log = False
        self.busy = False
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(1000)
        self.poll()

    def task_changed(self, *_):
        self.source.blockSignals(True)
        self.source.clear()
        self.source.blockSignals(False)
        self.changed()

    def changed(self, *_):
        self.generation += 1
        self.user_selected_log = self.source.currentIndex() >= 0
        self.text.clear()
        self.offsets[(self.task.currentData(), self.source.currentData())] = 0
        self.log_path = None
        self.text.setPlaceholderText("正在读取所选日志…")
        self.refresh_requested = True
        if not self.busy:
            self.refresh_requested = False
            self.poll()

    def poll(self):
        if self.busy:
            return
        self.busy = True
        generation = self.generation
        requested = self.task.currentData(), self.source.currentData()
        offset = self.offsets.get(requested, 0)
        prefer_nonempty = not self.user_selected_log
        self.async_call(lambda: self.fetch(requested, offset, prefer_nonempty),
                        lambda value: self.polled(generation, value),
                        lambda error: self.location.setText("日志读取失败：" + str(error)),
                        self.poll_finished)

    def fetch(self, requested, offset, prefer_nonempty=False):
        jobs = self.backend.jobs()
        job = next((job for job in jobs if job["id"] == requested[0]), jobs[-1] if jobs else None)
        if job is None:
            return jobs, None, [], None, None
        if hasattr(self.backend, "list_logs"):
            sources = self.backend.list_logs(job["id"])
        else:
            sources = [dict(id="runtime", label="任务日志", path=job.get("log_path", ""), available=True, bytes=None)]
        source = next((item for item in sources if item["id"] == requested[1]), None) if job["id"] == requested[0] else None
        if prefer_nonempty and source and not (source.get("bytes") or 0):
            source = None
        if source is None:
            source = next((item for item in sources if item.get("available") and (item.get("bytes") or 0) > 0),
                          next((item for item in sources if item.get("available")), sources[0] if sources else None))
        log = None
        if source is not None and source.get("available"):
            position = offset if requested == (job["id"], source["id"]) else 0
            log = self.backend.read_log(job["id"], position, log_id=source["id"]) if hasattr(self.backend, "list_logs") else self.backend.read_log(job["id"], position)
        return jobs, job["id"], sources, source, log

    def poll_finished(self):
        self.busy = False
        if self.refresh_requested:
            self.refresh_requested = False
            self.poll()

    def polled(self, generation, value):
        if generation != self.generation:
            return
        jobs, selected, sources, source, log = value
        self.jobs_by_id = {job["id"]: job for job in jobs}
        previous = self.task.currentData(), self.source.currentData()
        self.task.blockSignals(True)
        self.task.clear()
        for job in reversed(jobs):
            self.task.addItem("{} · {} · {}".format(job.get("label", job["id"]), status_text(job.get("status")),
                                                    job_time(job.get("started_at"))), job["id"])
        index = self.task.findData(selected)
        self.task.setCurrentIndex(index if index >= 0 else 0)
        self.task.blockSignals(False)
        self.source.blockSignals(True)
        self.source.clear()
        for item in sources:
            suffix = " · " + human_bytes(item["bytes"]) if item.get("bytes") is not None else ""
            self.source.addItem(item.get("label", item["id"]) + suffix + (" · 不可读取" if not item.get("available") else ""), item["id"])
            self.source.setItemData(self.source.count()-1, item.get("error") or item.get("path", ""), QtCore.Qt.ToolTipRole)
        self.source.setCurrentIndex(self.source.findData(source["id"]) if source else -1)
        self.source.blockSignals(False)
        current = selected, source["id"] if source else None
        if previous != current:
            self.text.clear()
        self.log_path = source.get("path") if source else None
        self.location.setText(str(self.log_path or "没有已登记的任务日志"))
        if log:
            if log.get("text"):
                scroll_value = self.text.verticalScrollBar().value()
                self.text.moveCursor(QtGui.QTextCursor.End)
                self.text.insertPlainText(log["text"])
                if self.follow.isChecked():
                    self.text.moveCursor(QtGui.QTextCursor.End)
                else:
                    self.text.verticalScrollBar().setValue(scroll_value)
            self.offsets[current] = log.get("offset", self.offsets.get(current, 0))
        if log and log.get("available") is False:
            message = "日志不可读取：" + str(log.get("error") or "文件状态已改变")
            self.text.setPlaceholderText(message)
            self.location.setText(str(self.log_path or "") + "\n" + message)
        elif source and not source.get("available"):
            message = "日志不可读取：" + str(source.get("error") or "文件尚未生成")
            self.text.setPlaceholderText(message)
            self.location.setText(str(self.log_path or "") + "\n" + message)
        else:
            self.text.setPlaceholderText("所选日志暂无内容，可切换阶段日志查看实际输出。")
        if not jobs:
            self.location.setText("暂无面板任务。启动实时融合、录制或离线融合后，可在这里查看日志。")

    def open_directory(self):
        if self.log_path:
            local_open(Path(self.log_path).parent)


class StorageDialog(PanelDialog):
    def __init__(self, backend, parent=None):
        super().__init__(backend, "数据与存储", parent, (1030, 720))
        self.recording_root, self.results_root = PathPicker(), PathPicker()
        form = QtWidgets.QFormLayout()
        form.addRow("录制默认路径", self.recording_root)
        form.addRow("融合结果默认路径", self.results_root)
        self.layout.addLayout(form)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(hint("选择两个已存在且相互独立的文件夹。保存后供新任务默认使用；历史数据保留原位置。"), 1)
        self.save_button = button("保存默认路径", self.save_settings, primary=True)
        row.addWidget(self.save_button)
        self.layout.addLayout(row)
        browse_row = QtWidgets.QHBoxLayout()
        self.root_kind = combo([("查看录制数据", "recording"), ("查看融合结果", "results")])
        self.root_kind.currentIndexChanged.connect(self.scan)
        browse_row.addWidget(self.root_kind)
        browse_row.addStretch()
        browse_row.addWidget(button("刷新占用", self.scan))
        self.layout.addLayout(browse_row)
        self.usage = hint("正在读取默认路径…")
        self.layout.addWidget(self.usage)
        self.table = QtWidgets.QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["目录项", "文件大小", "实际占用", "类型", "状态"])
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        set_readonly_table(self.table)
        self.table.setColumnWidth(0, 330)
        self.table.setColumnWidth(1, 110)
        self.table.setColumnWidth(2, 110)
        self.table.setColumnWidth(3, 100)
        self.layout.addWidget(self.table, 1)
        bottom = QtWidgets.QHBoxLayout()
        self.delete_button = button("删除所选子文件夹…", self.prepare_delete, danger=True)
        bottom.addWidget(self.delete_button)
        bottom.addWidget(hint("删除前展示具体路径和占用；仅可删除完整可读、非隐藏且未运行的直接子目录。"), 1)
        bottom.addWidget(button("关闭", self.close))
        _mark_action_row(bottom)
        self.layout.addLayout(bottom)
        self.scan_generation = 0
        self.scanned_root = None
        self.async_call(backend.settings, self.loaded)

    def loaded(self, settings):
        self.recording_root.set_path(settings["recording_root"])
        self.results_root.set_path(settings["results_root"])
        self.scan()

    def root_path(self):
        return self.recording_root.path() if self.root_kind.currentData() == "recording" else self.results_root.path()

    def save_settings(self):
        recording, results = self.recording_root.path(), self.results_root.path()
        self.save_button.setEnabled(False)
        self.async_call(lambda: self.backend.update_settings(recording, results), self.saved,
                        finished=lambda: self.save_button.setEnabled(True))

    def saved(self, settings):
        self.loaded(settings)
        if self.parent() and hasattr(self.parent(), "settings_changed"):
            self.parent().settings_changed(settings)
        QtWidgets.QMessageBox.information(self, "默认路径已保存", "新打开的录制、离线融合和结果对比窗口会读取这些默认路径。")

    def scan(self, *_):
        root = self.root_path()
        if not root:
            return
        self.scan_generation += 1
        generation = self.scan_generation
        self.scanned_root = None
        self.table.setRowCount(0)
        self.delete_button.setEnabled(False)
        self.usage.setText("正在统计占用…")
        self.async_call(lambda: self.backend.scan_storage(root), lambda value: self.scanned(generation, value),
                        lambda error: self.scan_failed(generation, error))

    def scan_failed(self, generation, error):
        if generation == self.scan_generation:
            self.usage.setText(str(error))

    def scanned(self, generation, data):
        if generation != self.scan_generation:
            return
        self.scanned_root = data["root"]
        items = data.get("items", [])
        summary = "文件大小 {} · 实际占用 {} · 磁盘可用 {}".format(
            self.size_text(data, "bytes"), self.size_text(data, "allocated_bytes"), self.size_text(data, "free_bytes"))
        if not data.get("total_complete", True):
            summary += "\n统计不完整：{} 项未能完整统计，已知数值不能作为全部占用。".format(data.get("unknown_items", "部分"))
        self.usage.setText(summary + "\n" + data["root"])
        self.table.setRowCount(len(items))
        for row, item in enumerate(items):
            active = item.get("active", False)
            deletable = item.get("deletable", not active) and not active
            state = "运行中，不可删除" if active else ("可选择" if deletable else "不可删除")
            if item.get("status") == "UNAVAILABLE":
                state = "状态不可确认，不可删除" if item.get("active") is None else "不可完整读取，不可删除"
                deletable = False
            if item.get("error"):
                state += "：" + str(item["error"])
            entry_type = {"file": "文件", "directory": "目录", "symlink": "链接", "mount": "嵌套挂载", "other": "其他"}.get(item.get("entry_type"))
            for col, text in enumerate((item["name"], self.size_text(item, "bytes"), self.size_text(item, "allocated_bytes"),
                                       entry_type or item.get("kind", ""), state)):
                cell = QtWidgets.QTableWidgetItem(str(text))
                cell.setData(QtCore.Qt.UserRole, item)
                cell.setToolTip(item["path"] + ("\n" + str(item["error"]) if item.get("error") else ""))
                if not deletable:
                    cell.setFlags(cell.flags() & ~QtCore.Qt.ItemIsSelectable)
                self.table.setItem(row, col, cell)
        self.delete_button.setEnabled(any(item.get("deletable", not item.get("active")) and not item.get("active")
                                          and item.get("status") != "UNAVAILABLE" for item in items))

    @staticmethod
    def size_text(item, key):
        if item.get(key) is not None:
            return human_bytes(item[key])
        known = item.get("known_" + key)
        return "已知 {}（不完整）".format(human_bytes(known)) if known is not None else "未统计"

    def prepare_delete(self):
        rows = sorted({index.row() for index in self.table.selectedIndexes()})
        items = [self.table.item(row, 0).data(QtCore.Qt.UserRole) for row in rows]
        if not items:
            self.show_error("请先选择需要删除的子文件夹。")
            return
        if any(item.get("active") or not item.get("deletable", True) or item.get("status") == "UNAVAILABLE" for item in items):
            self.show_error("所选目录不可删除，请查看状态列中的原因。")
            return
        root = self.scanned_root
        paths = [item["path"] for item in items]
        self.delete_button.setEnabled(False)
        self.async_call(lambda: self.backend.prepare_delete(root, paths), self.confirm_delete,
                        finished=lambda: self.delete_button.setEnabled(True))

    def confirm_delete(self, plan):
        allocation = human_bytes(plan["allocated_bytes"]) if plan.get("allocated_bytes") is not None else "未统计"
        message = "将永久删除以下 {} 个目录。文件大小 {}，实际占用 {}。\n\n{}\n\n无法通过面板撤销。".format(
            len(plan["paths"]), human_bytes(plan.get("bytes")), allocation, "\n".join(map(str, plan["paths"])))
        box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "确认删除具体数据", message,
                                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel, self)
        box.setDefaultButton(QtWidgets.QMessageBox.Cancel)
        box.button(QtWidgets.QMessageBox.Yes).setText("确认永久删除")
        if box.exec_() != QtWidgets.QMessageBox.Yes:
            return
        self.delete_button.setEnabled(False)
        token = plan["token"]
        self.async_call(lambda: self.backend.execute_delete(token), self.deleted,
                        finished=lambda: self.delete_button.setEnabled(True))

    def deleted(self, result):
        message = "已删除所选数据，文件大小 {}。".format(human_bytes(result.get("bytes")))
        if result.get("free_delta_bytes") is not None:
            delta = int(result["free_delta_bytes"])
            message += "\n磁盘可用空间{} {}（清理前后实测，可能包含其他任务同时写入的影响）。".format("增加" if delta >= 0 else "减少", human_bytes(abs(delta)))
        QtWidgets.QMessageBox.information(self, "删除完成", message)
        self.scan()


def field_label(name):
    labels = {"left": "左侧", "right": "右侧", "translation": "平移 [m]", "rotation": "旋转",
              "translation_m": "平移 [m]", "rotation_rpy_rad": "姿态 RPY [rad]", "rpy": "姿态 RPY",
              "radius_m": "轮半径 [m]", "wheel_radius_m": "轮半径 [m]", "track_width_m": "轮距 [m]",
              "wheel_separation_m": "轮距 [m]", "enabled": "启用", "width": "宽度", "height": "高度",
              "value_m": "XYZ 平移 [m]", "value_matrix": "旋转矩阵 R（3 × 3）", "status": "状态 / 依据等级",
              "evidence": "来源依据", "note": "说明", "parent_frame": "目标坐标系", "child_frame": "源坐标系"}
    return labels.get(str(name), str(name))


class NumericArrayEditor(QtWidgets.QWidget):
    """Nullable metric XYZ / SO(3) entries, never seed unknowns with an identity."""
    def __init__(self, value, matrix, editable):
        super().__init__()
        self.matrix, self.editable = matrix, editable
        self.entries = []
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 2, 0, 2)
        self.unknown = QtWidgets.QCheckBox("未知，保留空值")
        self.unknown.setChecked(value is None)
        self.unknown.setEnabled(editable)
        self.unknown.toggled.connect(self.refresh)
        self.layout.addWidget(self.unknown)
        grid = QtWidgets.QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(4)
        rows = 3 if matrix else 1
        for row in range(rows):
            for col in range(3):
                original = None if value is None else value[row][col] if matrix else value[col]
                entry = QtWidgets.QLineEdit("" if original is None else str(original))
                entry.setPlaceholderText("R{}{}".format(row+1, col+1) if matrix else ("X [m]", "Y [m]", "Z [m]")[col])
                validator = QtGui.QDoubleValidator(entry)
                validator.setLocale(QtCore.QLocale.c())
                validator.setNotation(QtGui.QDoubleValidator.ScientificNotation)
                entry.setValidator(validator)
                self.entries.append(entry)
                grid.addWidget(entry, row, col)
        self.layout.addLayout(grid)
        self.refresh()

    def refresh(self, *_):
        for entry in self.entries:
            entry.setReadOnly(not self.editable or self.unknown.isChecked())
            entry.setEnabled(not self.unknown.isChecked())

    def value(self):
        if self.unknown.isChecked():
            return None
        import math
        try:
            values = [float(entry.text()) for entry in self.entries]
        except ValueError as error:
            raise ValueError("请填写完整 XYZ / 旋转矩阵；未知外参不能自动用零或单位矩阵代替。") from error
        if not all(math.isfinite(value) for value in values):
            raise ValueError("外参必须是有限数值。")
        return [values[i:i+3] for i in (0, 3, 6)] if self.matrix else values


class StructuredEditor(QtWidgets.QTreeWidget):
    """A typed nested property form: keys stay fixed, values have typed editors."""
    def __init__(self, value, editable=False, parent=None, status_options=None, evidence_readonly=False):
        super().__init__(parent)
        configure_scroll_view(self)
        self.original = copy.deepcopy(value)
        self.editors = {}
        self.status_options = status_options
        self.evidence_readonly = evidence_readonly
        self.setHeaderLabels(["参数 / 坐标系", "值"])
        self.setColumnWidth(0, 345)
        self.setAlternatingRowColors(True)
        self.build(None, "参数", value, (), editable)
        self.expandToDepth(2)

    def build(self, parent, key, value, path, editable):
        if self.evidence_readonly and "evidence" in path:
            editable = False
        if parent is None and isinstance(value, dict):
            for name, child in value.items():
                self.build(None, name, child, path + (name,), editable)
            return
        item = QtWidgets.QTreeWidgetItem([field_label(key), ""])
        item.setToolTip(0, ".".join(map(str, path)))
        if parent is None:
            self.addTopLevelItem(item)
        else:
            parent.addChild(item)
        if str(key) in {"value_m", "value_matrix"} and (value is None or isinstance(value, list)):
            matrix = str(key) == "value_matrix"
            editor = NumericArrayEditor(value, matrix, editable)
            item.setSizeHint(1, QtCore.QSize(440, 140 if matrix else 70))
            self.setItemWidget(item, 1, editor)
            self.editors[path] = (editor, "numeric_array", value)
            return
        if isinstance(value, (dict, list)):
            pairs = value.items() if isinstance(value, dict) else enumerate(value)
            for name, child in pairs:
                self.build(item, name, child, path + (name,), editable)
            return
        if str(key) in {"id", "parent_frame", "child_frame", "direction", "unit", "units", "schema_version"}:
            editable = False
        if key == "status" and self.status_options is not None and any(key in path for key in ("translation", "rotation")):
            names = {"UNKNOWN": "未知", "CAD_NOMINAL": "CAD 标称", "LEGACY_CANDIDATE": "历史候选",
                     "USER_MEASURED_EXPERIMENT": "用户实测（实验）", "CALIBRATED": "已标定"}
            options = list(self.status_options)
            if value not in options:
                options.insert(0, value)
            editor = combo([(names.get(option, option) + " · " + option, option) for option in options])
            editor.setCurrentIndex(editor.findData(value))
            editor.setEnabled(editable)
            self.setItemWidget(item, 1, editor)
            self.editors[path] = (editor, "status", value)
            return
        if isinstance(value, bool):
            editor = QtWidgets.QCheckBox()
            editor.setChecked(value)
            editor.setEnabled(editable)
        elif isinstance(value, int):
            editor = QtWidgets.QLineEdit(str(value))
            editor.setReadOnly(not editable)
            editor.setValidator(QtGui.QRegularExpressionValidator(QtCore.QRegularExpression(r"[+-]?\d+")))
        elif isinstance(value, float):
            editor = QtWidgets.QLineEdit(repr(value))
            editor.setReadOnly(not editable)
            validator = QtGui.QDoubleValidator(editor)
            validator.setLocale(QtCore.QLocale.c())
            validator.setNotation(QtGui.QDoubleValidator.ScientificNotation)
            editor.setValidator(validator)
        else:
            editor = QtWidgets.QLineEdit("" if value is None else str(value))
            editor.setReadOnly(not editable or value is None)
        self.setItemWidget(item, 1, editor)
        self.editors[path] = (editor, type(value), value)

    def value(self):
        result = copy.deepcopy(self.original)
        for path, (editor, kind, original) in self.editors.items():
            if kind == "numeric_array":
                new_value = editor.value()
            elif kind == "status":
                new_value = editor.currentData()
            elif kind is bool:
                new_value = editor.isChecked()
            elif kind is type(None):
                new_value = None
            else:
                text = editor.text()
                try:
                    new_value = kind(text)
                except ValueError as exc:
                    raise ValueError("参数 {} 的值无效".format(".".join(map(str, path)))) from exc
                if kind is float:
                    import math
                    if not math.isfinite(new_value):
                        raise ValueError("参数必须为有限数值：" + ".".join(map(str, path)))
            if not path:
                result = new_value
            else:
                target = result
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = new_value
        return result


class EvidenceFilePicker(FilePicker):
    def choose(self):
        selected, _ = QtWidgets.QFileDialog.getOpenFileName(self, "选择原始测量或标定证据", self.path(), "所有文件 (*)")
        if selected:
            self.set_path(selected)
            self.changed.emit(selected)


class ExtrinsicEvidenceDialog(PanelDialog):
    prepared = QtCore.pyqtSignal(object)

    def __init__(self, backend, group, revision, transforms, parent=None):
        super().__init__(backend, "导入并绑定测量证据", parent, (850, 530))
        self.revision = revision
        self.assembly = group.get("assembly_revision") or ""
        self.transforms = transforms
        self.setWindowModality(QtCore.Qt.WindowModal)
        self.layout.addWidget(hint("仅为明确选择的外参分量绑定原始证据，不自动改动数值或依据等级。文件副本和配置在点击设备参数窗口“保存参数”时才写入。"))
        form = QtWidgets.QFormLayout()
        self.target = combo([(str(row["id"]) + "（{} ← {}）".format(row.get("parent_frame", ""), row.get("child_frame", "")), index)
                             for index, row in enumerate(transforms)])
        self.component = combo([("平移", "translation"), ("旋转", "rotation")])
        self.file = EvidenceFilePicker()
        self.evidence_type = combo([("几何测量", "GEOMETRY_MEASUREMENT"), ("外参标定", "EXTRINSIC_CALIBRATION")])
        self.binding_mode = combo([("替换此分量的原有依据", "replace"), ("追加到此分量的依据", "append")])
        self.note = QtWidgets.QLineEdit()
        self.note.setPlaceholderText("说明测量方法、对象及日期")
        form.addRow("目标外参", self.target)
        form.addRow("目标分量", self.component)
        form.addRow("原始证据文件", self.file)
        form.addRow("证据类型", self.evidence_type)
        form.addRow("绑定方式", self.binding_mode)
        form.addRow("说明", self.note)
        self.layout.addLayout(form)
        self.confirm_assembly = QtWidgets.QCheckBox("我确认此证据对应当前装配版本：" + (self.assembly or "未提供"))
        self.layout.addWidget(self.confirm_assembly)
        self.status = hint("证据文件及哈希校验不能代替实际测量精度验证。")
        self.layout.addWidget(self.status)
        row = QtWidgets.QHBoxLayout()
        row.addStretch()
        row.addWidget(button("取消", self.close))
        self.import_button = button("校验并绑定（尚未保存）", self.prepare, primary=True)
        self.import_button.setEnabled(hasattr(backend, "prepare_extrinsic_evidence"))
        row.addWidget(self.import_button)
        _mark_action_row(row)
        self.layout.addLayout(row)
        self.pending_target = None

    def prepare(self):
        index, component = self.target.currentData(), self.component.currentData()
        if index is None or not isinstance(self.transforms[index].get(component), dict):
            self.status.setText("所选外参没有该分量，不能绑定证据。")
            return
        if not self.assembly or not self.confirm_assembly.isChecked():
            self.status.setText("请明确确认测量所对应的当前装配版本。")
            return
        path, note = self.file.path(), self.note.text().strip()
        if not path or not note:
            self.status.setText("请选择原始证据文件并填写说明。")
            return
        self.pending_target = index, component, self.binding_mode.currentData()
        evidence_type = self.evidence_type.currentData()
        self.import_button.setEnabled(False)
        self.async_call(lambda: self.backend.prepare_extrinsic_evidence(path, evidence_type, self.assembly, note, self.revision),
                        self.ready, lambda error: self.status.setText("未导入：" + str(error)),
                        lambda: self.import_button.setEnabled(True))

    def ready(self, result):
        if result.get("revision") != self.revision:
            self.status.setText("配置版本已改变，请重新读取设备参数；本次未绑定。")
            return
        self.prepared.emit(dict(result, target=self.pending_target))
        self.accept()


class ExtrinsicsEditor(QtWidgets.QWidget):
    def __init__(self, backend, group, revision, editable=False):
        super().__init__()
        self.backend, self.group, self.revision, self.editable = backend, group, revision, editable
        self.pending_sources = {}
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        row = QtWidgets.QHBoxLayout()
        self.status = hint("数值与依据等级需分别确认；未知外参不会自动填零或变为有效。")
        row.addWidget(self.status, 1)
        self.import_button = button("导入并绑定测量证据…", self.open_import)
        self.import_button.setEnabled(editable and hasattr(backend, "prepare_extrinsic_evidence"))
        row.addWidget(self.import_button)
        self.layout.addLayout(row)
        self.form = None
        self.set_form(group["value"])
        self.import_dialog = None

    def set_form(self, value):
        if self.form is not None:
            self.layout.removeWidget(self.form)
            self.form.deleteLater()
        self.form = StructuredEditor(value, self.editable, status_options=self.group.get("allowed_statuses", [
            "UNKNOWN", "CAD_NOMINAL", "LEGACY_CANDIDATE", "USER_MEASURED_EXPERIMENT", "CALIBRATED"]), evidence_readonly=True)
        self.editors = self.form.editors
        self.layout.addWidget(self.form, 1)

    def value(self):
        return self.form.value()

    def evidence_tokens(self, value):
        referenced = {entry.get("source_id") for transform in value for component in ("translation", "rotation")
                      for entry in transform.get(component, {}).get("evidence", [])}
        return [self.pending_sources[identifier]["token"] for identifier in referenced if identifier in self.pending_sources]

    def open_import(self):
        try:
            current = self.value()
        except ValueError as error:
            QtWidgets.QMessageBox.warning(self, "外参数值尚未完整", str(error))
            return
        self.import_dialog = ExtrinsicEvidenceDialog(self.backend, self.group, self.revision, current, self)
        self.import_dialog.prepared.connect(lambda result: self.bind_evidence(current, result))
        self.import_dialog.show()

    def bind_evidence(self, current, result):
        index, component, mode = result["target"]
        source = result["source"]
        value = copy.deepcopy(current)
        record = {"source_id": source["id"]}
        evidence = value[index][component].get("evidence", []) if mode == "append" else []
        value[index][component]["evidence"] = evidence + [record]
        self.pending_sources[source["id"]] = result
        self.set_form(value)
        self.status.setText("已绑定 {} 的{}证据，尚未保存。数值和依据等级保持原值。".format(value[index]["id"], "平移" if component == "translation" else "旋转"))
        self.status.setToolTip("来源：{}\nSHA256：{}".format(source.get("original_path", ""), source.get("sha256", "")))


class LiveCalibrationEditor(QtWidgets.QWidget):
    """Import independently evidenced device calibration, never a session candidate."""
    def __init__(self, backend, value, editable=False):
        super().__init__()
        self.backend, self.editable = backend, editable
        self.current_value = copy.deepcopy(value)
        self.loading = False
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(hint("在线校正必须匹配当前 IMU、轮设备及有效证据。请选择设备校正声明及其零偏确认文件；录制会话的离线候选不能直接用于在线。"))
        configured = bool(value.get("gyro_bias_config") and value.get("calibration_evidence_path"))
        self.validation_status = hint("已配置文件引用，当前窗口尚未验证；是否可启用以实时融合的生效条件为准。" if configured else
                                      "未配置在线校正。下方声明字段是待填写模板，不代表已测量、已验证或已生效。")
        self.layout.addWidget(self.validation_status)
        form = QtWidgets.QFormLayout()
        self.declaration = FilePicker()
        self.gyro_bias = FilePicker(value.get("gyro_bias_config", ""))
        self.evidence = FilePicker(value.get("calibration_evidence_path", ""))
        self.validated_files = self.file_selection()
        form.addRow("设备校正声明 JSON", self.declaration)
        form.addRow("零偏确认 JSON", self.gyro_bias)
        form.addRow("声明证据 JSON", self.evidence)
        self.layout.addLayout(form)
        self.import_button = button("读取并校验所选文件", self.import_files)
        self.import_button.setEnabled(editable)
        self.layout.addWidget(self.import_button)
        self.status = hint("共校验四个文件：声明及其证据、零偏确认及其同名 .evidence.json。空白声明证据路径会自动查找同名文件。")
        self.layout.addWidget(self.status)
        self.preview = StructuredEditor(value.get("calibration", {}), editable=False)
        self.layout.addWidget(self.preview, 1)
        for picker in (self.declaration, self.gyro_bias, self.evidence):
            picker.setEnabled(editable)
            picker.changed.connect(self.files_changed)

    def file_selection(self):
        return self.declaration.path(), self.gyro_bias.path(), self.evidence.path()

    def files_changed(self, *_):
        if self.file_selection() != self.validated_files:
            self.validation_status.setText("文件选择已改变，尚未验证；当前配置未因此改变。")
            self.status.setText("文件选择已改变，请先点击“读取并校验所选文件”。")

    def import_files(self):
        calibration_path, bias_path = self.declaration.path(), self.gyro_bias.path()
        evidence_path = self.evidence.path() or None
        if not calibration_path or not bias_path:
            self.status.setText("请选择设备校正声明和零偏确认两个 JSON 文件。")
            return
        self.loading = True
        self.import_button.setEnabled(False)
        for picker in (self.declaration, self.gyro_bias, self.evidence):
            picker.setEnabled(False)
        self.status.setText("正在校验设备身份、声明及证据…")
        self.validation_status.setText("正在验证所选文件；尚未保存参数。")
        BackgroundTask(self, lambda: self.backend.load_live_calibration(calibration_path, bias_path, evidence_path),
                       self.imported, self.import_failed, self.import_finished)

    def import_failed(self, error):
        self.validation_status.setText("所选文件未通过验证，未保存到运行配置。")
        self.status.setText("未导入：" + str(error))

    def imported(self, value):
        self.current_value = copy.deepcopy(value)
        self.gyro_bias.set_path(value.get("gyro_bias_config", ""))
        self.evidence.set_path(value.get("calibration_evidence_path", ""))
        self.layout.removeWidget(self.preview)
        self.preview.deleteLater()
        self.preview = StructuredEditor(value.get("calibration", {}), editable=False)
        self.layout.addWidget(self.preview, 1)
        self.validated_files = self.file_selection()
        self.validation_status.setText("所选文件已通过验证，尚未保存；点击“保存参数”后供下一次任务使用。")
        self.status.setText("校验通过。请核对实际数值与身份，再点击窗口下方“保存参数”；尚未写入运行配置。")

    def import_finished(self):
        self.loading = False
        self.import_button.setEnabled(self.editable)
        for picker in (self.declaration, self.gyro_bias, self.evidence):
            picker.setEnabled(self.editable)

    def value(self):
        if self.loading:
            raise ValueError("在线校正文件仍在校验，请等待校验结束后再保存。")
        if self.file_selection() != self.validated_files:
            raise ValueError("所选文件尚未通过校验，请先点击“读取并校验所选文件”。")
        return copy.deepcopy(self.current_value)


class ParametersDialog(PanelDialog):
    WARNING = "如非必要，请勿随意修改外参和轮参数。错误参数会影响定位、点云对齐与地图结果。请确认单位、坐标系和设备身份后再编辑。修改默认对下一次任务生效。"

    def __init__(self, backend, parent=None):
        super().__init__(backend, "设备参数", parent, (1020, 780))
        from .ui_device import DeviceOverview
        self.pages = QtWidgets.QStackedWidget()
        self.layout.addWidget(self.pages)
        self.overview = DeviceOverview(backend, self)
        self.overview.advancedRequested.connect(self.request_group)
        self.overview.parametersChanged.connect(lambda: self.async_call(backend.device_parameters, self.loaded))
        self.pages.addWidget(self.overview)
        self.advanced_page = QtWidgets.QWidget()
        self.advanced_layout = QtWidgets.QVBoxLayout(self.advanced_page)
        self.advanced_layout.setContentsMargins(0, 0, 0, 0)
        self.back_button = button("‹ 返回设备概览", self.show_overview)
        self.advanced_layout.addWidget(self.back_button, 0, QtCore.Qt.AlignLeft)
        self.pages.addWidget(self.advanced_page)
        self.identity = hint("正在读取设备身份与配置…")
        self.advanced_layout.addWidget(self.identity)
        self.identity_tree = StructuredEditor({})
        self.identity_tree.setMaximumHeight(170)
        self.identity_tree.hide()
        self.identity_toggle = button("展开设备身份（只读）", self.toggle_identity)
        self.advanced_layout.addWidget(self.identity_toggle)
        self.advanced_layout.addWidget(self.identity_tree)
        self.group = ComboBox()
        self.group.currentIndexChanged.connect(self.choose_group)
        self.advanced_layout.addWidget(self.group)
        self.editor_area = QtWidgets.QVBoxLayout()
        self.advanced_layout.addLayout(self.editor_area, 1)
        self.effective = hint("默认只读，点击“编辑此组参数”后才能修改。")
        self.advanced_layout.addWidget(self.effective)
        row = QtWidgets.QHBoxLayout()
        self.edit_button = button("编辑此组参数", self.begin_edit)
        self.save_button = button("保存参数", self.save, primary=True)
        self.save_button.setEnabled(False)
        self.discard_button = button("放弃本次修改", self.discard)
        self.discard_button.setEnabled(False)
        row.addWidget(self.edit_button)
        row.addWidget(self.save_button)
        row.addWidget(self.discard_button)
        row.addStretch()
        row.addWidget(button("关闭", self.close))
        _mark_action_row(row)
        self.advanced_layout.addLayout(row)
        self.data = None
        self.editor = None
        self.editing = False
        self.edit_token = None
        self.edit_button.setEnabled(False)
        self.requested_group = None
        self.async_call(backend.device_parameters, self.loaded)

    def loaded(self, data):
        self.data = data
        self.overview.refresh(data)
        identity = data.get("identity", {})
        if isinstance(identity, dict):
            parts = ["{}：{}".format(field_label(k), v) for k, v in identity.items() if not isinstance(v, (dict, list))]
            self.identity.setText("设备身份（只读）  " + "  |  ".join(parts) if parts else "设备身份详见对应参数组。")
            self.identity.setToolTip(str(identity))
        else:
            self.identity.setText(str(identity))
        visible = not self.identity_tree.isHidden()
        self.advanced_layout.removeWidget(self.identity_tree)
        self.identity_tree.deleteLater()
        self.identity_tree = StructuredEditor(identity)
        self.identity_tree.setMaximumHeight(190)
        self.advanced_layout.insertWidget(3, self.identity_tree)
        self.identity_tree.setVisible(visible)
        previous = self.requested_group or self.group.currentData()
        self.group.blockSignals(True)
        self.group.clear()
        for group in data.get("groups", []):
            self.group.addItem(group.get("label", group["id"]), group["id"])
        index = self.group.findData(previous)
        self.group.setCurrentIndex(index if index >= 0 else 0)
        self.group.blockSignals(False)
        self.editing = False
        self.edit_token = None
        self.show_group()

    def request_group(self, identifier):
        self.pages.setCurrentWidget(self.advanced_page)
        self.requested_group = identifier
        index = self.group.findData(identifier)
        if index >= 0 and not self.editing:
            self.group.setCurrentIndex(index)

    def show_overview(self):
        if self.editing:
            answer = QtWidgets.QMessageBox.question(self, "未保存的参数", "放弃本次未保存的修改并返回设备概览？",
                                                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                                    QtWidgets.QMessageBox.No)
            if answer != QtWidgets.QMessageBox.Yes:
                return
            self.discard()
        self.pages.setCurrentWidget(self.overview)

    def toggle_identity(self):
        show = self.identity_tree.isHidden()
        self.identity_tree.setVisible(show)
        self.identity_toggle.setText("收起设备身份" if show else "展开设备身份（只读）")

    def current_group(self):
        return next((group for group in (self.data or {}).get("groups", []) if group["id"] == self.group.currentData()), None)

    def choose_group(self, *_):
        self.show_group()

    def show_group(self):
        if self.editor is not None:
            self.editor_area.removeWidget(self.editor)
            self.editor.deleteLater()
            self.editor = None
        group = self.current_group()
        if not group:
            return
        if group["id"] == "live_motion":
            self.editor = LiveCalibrationEditor(self.backend, group["value"], self.editing)
            if group.get("error"):
                self.editor.validation_status.setText("当前在线校正不可用：" + str(group["error"]))
        elif group["id"] == "extrinsics":
            self.editor = ExtrinsicsEditor(self.backend, group, self.data["revision"], self.editing)
        else:
            self.editor = StructuredEditor(group["value"], self.editing)
        self.editor_area.addWidget(self.editor)
        self.group.setEnabled(not self.editing)
        editable = not group.get("read_only", False)
        self.edit_button.setEnabled(not self.editing and editable)
        self.save_button.setEnabled(self.editing)
        self.discard_button.setEnabled(self.editing)
        effective = "当前显示立即生效" if group.get("effective") in ("immediate", "display") else "对下一次任务生效；当前任务继续使用启动时的配置"
        self.effective.setText("{}。{}".format("编辑中" if self.editing else "只读", effective))
        if group.get("path"):
            self.effective.setToolTip(str(group["path"]))

    def confirm_warning(self, group):
        message = group.get("warning") or self.WARNING
        answer = QtWidgets.QMessageBox.warning(self, "修改参数前请确认", message,
                                               QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel,
                                               QtWidgets.QMessageBox.Cancel)
        return answer == QtWidgets.QMessageBox.Ok

    def begin_edit(self):
        group = self.current_group()
        if not group or self.editing:
            return
        # Every configuration edit has a warning, including nested calibration fields.
        if not self.confirm_warning(group):
            return
        group_id = group["id"]
        self.edit_button.setEnabled(False)
        self.group.setEnabled(False)
        self.async_call(lambda: self.backend.request_parameter_edit(group_id), self.edit_authorized,
                        finished=self.edit_request_finished)

    def edit_request_finished(self):
        self.edit_button.setEnabled(not self.editing)
        self.group.setEnabled(not self.editing)

    def edit_authorized(self, permission):
        if permission.get("revision") != self.data.get("revision"):
            self.show_error("配置已被其他任务修改，请重新打开设备参数后再编辑。")
            return
        self.edit_token = permission.get("token")
        self.editing = True
        self.show_group()

    def discard(self):
        self.editing = False
        self.edit_token = None
        self.show_group()

    def save(self):
        group = self.current_group()
        try:
            value = self.editor.value()
        except ValueError as error:
            self.show_error(str(error))
            return
        self.save_button.setEnabled(False)
        group_id, token, revision = group["id"], self.edit_token, self.data["revision"]
        extra = {"evidence_tokens": self.editor.evidence_tokens(value)} if isinstance(self.editor, ExtrinsicsEditor) else {}
        self.async_call(lambda: self.backend.save_parameters(group_id, value, warning_token=token,
                                                             expected_revision=revision, **extra), self.saved,
                        finished=lambda: self.save_button.setEnabled(self.editing))

    def saved(self, result):
        self.editing = False
        if self.parent() and hasattr(self.parent(), "parameters_changed"):
            self.parent().parameters_changed()
        QtWidgets.QMessageBox.information(self, "参数已保存", "配置已保存并记录备份。请在下一次任务中确认生效参数。")
        self.async_call(self.backend.device_parameters, self.loaded)

    def closeEvent(self, event):
        if self.editing:
            answer = QtWidgets.QMessageBox.question(self, "未保存的参数", "放弃本次未保存的参数修改并关闭？",
                                                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                                    QtWidgets.QMessageBox.No)
            if answer != QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
        super().closeEvent(event)


class ResultsDialog(PanelDialog):
    def __init__(self, backend, parent=None):
        super().__init__(backend, "结果对比", parent, (1100, 650))
        self.root = PathPicker()
        self.root.changed.connect(self.scan)
        form = QtWidgets.QFormLayout()
        form.addRow("融合结果路径", self.root)
        self.layout.addLayout(form)
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(hint("选择一组或多组结果，比较 2D 地图、路线和三维点云。"), 1)
        controls.addWidget(button("刷新", self.scan))
        self.layout.addLayout(controls)
        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["组 / 方案", "数据来源", "可用内容", "文件大小"])
        set_readonly_table(self.table)
        self.table.setColumnWidth(0, 330)
        self.table.setColumnWidth(1, 250)
        self.table.setColumnWidth(2, 210)
        self.table.setColumnWidth(3, 120)
        self.table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.layout.addWidget(self.table, 1)
        self.info = hint("正在读取默认路径…")
        self.layout.addWidget(self.info)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(button("关闭", self.close))
        row.addStretch()
        self.open_button = button("打开所选结果", self.open_results, primary=True)
        row.addWidget(self.open_button)
        _mark_action_row(row)
        self.layout.addLayout(row)
        self.generation = 0
        self.async_call(backend.settings, self.loaded)
        self.viewers = []

    def loaded(self, settings):
        self.root.set_path(settings["results_root"])
        self.default_results_root = settings["results_root"]
        self.scan()

    def update_default_settings(self, settings):
        if self.root.path() == getattr(self, "default_results_root", ""):
            self.root.set_path(settings["results_root"])
            self.scan()
        self.default_results_root = settings["results_root"]

    def scan(self, *_):
        root = self.root.path()
        if not root:
            return
        self.generation += 1
        generation = self.generation
        self.table.setRowCount(0)
        self.info.setText("正在查找可查看的结果…")
        self.async_call(lambda: self.backend.list_results(root), lambda value: self.scanned(generation, value),
                        lambda error: self.scan_failed(generation, error))

    def scan_failed(self, generation, error):
        if generation == self.generation:
            self.info.setText(str(error))

    def scanned(self, generation, results):
        if generation != self.generation:
            return
        self.table.setRowCount(len(results))
        for row, result in enumerate(results):
            available = []
            if result.get("images"):
                available.append("地图图片")
            if result.get("trajectories"):
                available.append("路线")
            if result.get("clouds"):
                available.append("3D")
            for col, value in enumerate((result_label(result), result.get("dataset") or "未提供",
                                         " / ".join(available), human_bytes(result.get("bytes")))):
                item = QtWidgets.QTableWidgetItem(str(value))
                item.setData(QtCore.Qt.UserRole, result)
                item.setToolTip(str(value) + "\n" + str(result.get("name", "")) + "\n" + result["path"])
                self.table.setItem(row, col, item)
        self.info.setText("找到 {} 组结果。3D 支持旋转、平移、缩放；不需要联网。".format(len(results)))

    def open_results(self):
        rows = sorted({index.row() for index in self.table.selectedIndexes()})
        if not rows:
            self.show_error("请选择至少一组结果。")
            return
        results = [self.table.item(row, 0).data(QtCore.Qt.UserRole) for row in rows]
        from .results_viewer import ResultsViewer
        viewer = ResultsViewer(results, self.parent())
        self.viewers.append(viewer)
        viewer.show()
