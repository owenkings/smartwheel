"""Owned stationary capture and completed-recording preparation for point pairing."""
import threading
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

from .ui_common import PanelDialog, PanelScrollArea, PathPicker, button, hint, local_open, status_text


ACTIVE = {'QUEUED', 'RUNNING', 'STOPPING'}


class PairingCaptureDialog(PanelDialog):
    prepared = QtCore.pyqtSignal(str)

    def __init__(self, backend, parent=None):
        super().__init__(backend, '录制并准备双雷达点云', parent, (840, 780))
        self.task = None
        self._busy = False
        self._closing = False
        self._prepare_after_stop = False
        self._stop_pending = False
        self._prepared_path = ''
        self._settings_loaded = False
        self._name_valid = False
        self._cleanup = dict(identifier=None, destroyed=False)
        cleanup = self._cleanup

        def destroyed():
            cleanup['destroyed'] = True
            if cleanup['identifier']:
                def stop_owned():
                    try:
                        backend.cancel(cleanup['identifier'])
                    except (OSError, ValueError, RuntimeError):
                        pass
                threading.Thread(target=stop_owned, daemon=True).start()
        self.destroyed.connect(destroyed)
        self.body_scroll = PanelScrollArea()
        content = QtWidgets.QWidget()
        body = QtWidgets.QVBoxLayout(content)
        body.setContentsMargins(0, 0, 8, 0); body.setSpacing(14)
        self.body_scroll.setWidget(content)
        self.body_scroll.verticalScrollBar().rangeChanged.connect(self.reveal_result)
        self.layout.addWidget(self.body_scroll, 1)
        body.addWidget(hint('先录制同一静态场景中的双雷达原始点云，再从完整录包中准备选点输入。左右雷达应看见同一物体；建议静止录制约 10～20 秒。'))
        body.addWidget(hint('沿用现有录制与完整性检查，关闭手动驾驶入口。请保持轮椅、传感器支架和场景静止；到达时间接近不代表曝光同步。'))
        form = QtWidgets.QFormLayout()
        self.root = PathPicker('')
        self.name = QtWidgets.QLineEdit('calibration')
        self.name.setPlaceholderText('例如：走廊_标定')
        self.name.setMaxLength(80)
        suffix_row = QtWidgets.QHBoxLayout()
        self.suffixes = {}
        for key,label in (('year','年份 YYYY'),('date','日期 MMDD'),('time','时间 HHMMSS')):
            box = QtWidgets.QCheckBox(label)
            box.setChecked(True)
            self.suffixes[key] = box
            suffix_row.addWidget(box)
        suffix_row.addStretch()
        form.setVerticalSpacing(14)
        form.addRow('原始录制路径', self.root)
        form.addRow('名称前缀', self.name)
        form.addRow('名称后缀', suffix_row)
        body.addLayout(form)
        self.name_preview = hint('')
        self.name_preview.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        body.addWidget(self.name_preview)
        body.addWidget(hint('名称以 _ 连接，按开始录制时的本地时间命名。同名自动追加 _02、_03，保留已有数据。'))
        self.static = QtWidgets.QCheckBox('我会保持轮椅与场景静止，且两侧雷达视野有重叠')
        body.addWidget(self.static)
        start_row = QtWidgets.QHBoxLayout()
        self.start_button = button('开始双雷达录制', self.start, primary=True)
        self.stop_button = button('停止录制并准备点云', self.stop_and_prepare)
        self.start_button.setEnabled(False); self.stop_button.setEnabled(False)
        start_row.addWidget(self.start_button); start_row.addWidget(self.stop_button); start_row.addStretch()
        body.addLayout(start_row)
        existing = QtWidgets.QGroupBox('也可以使用已完成的录制')
        existing_layout = QtWidgets.QVBoxLayout(existing)
        self.dataset = PathPicker('')
        self.dataset.edit.setPlaceholderText('选择含 capture_manifest.json 的具体录制子文件夹')
        existing_layout.addWidget(self.dataset)
        self.prepare_button = button('检查完整性并准备此录包', self.prepare_existing)
        existing_layout.addWidget(self.prepare_button, 0, QtCore.Qt.AlignLeft)
        existing_layout.addWidget(hint('只接受 COMPLETE 的双雷达录包；保留原始数据。每次生成新的 prepared.json，不覆盖其他场景或设备外参。'))
        body.addWidget(existing)
        self.status_label = QtWidgets.QLabel('正在读取默认保存路径…')
        self.status_label.setWordWrap(True)
        self.status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        body.addWidget(self.status_label)
        self.detail = hint('准备的帧对来自同一个场景，不作为独立标定验证。')
        self.detail.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        body.addWidget(self.detail)
        body.addStretch()
        row = QtWidgets.QHBoxLayout()
        self.open_output_button = button('打开原始录制', self.open_recording)
        self.open_output_button.setEnabled(False)
        self.use_button = button('使用这份点云', self.use_prepared, primary=True)
        self.use_button.setEnabled(False)
        row.addWidget(self.open_output_button); row.addStretch()
        row.addWidget(button('关闭', self.close)); row.addWidget(self.use_button)
        for index in range(row.count()):
            widget = row.itemAt(index).widget()
            if widget:
                widget.setProperty('popupFooter', True)
        self.layout.addLayout(row)
        self.static.toggled.connect(self.update_controls)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(1000); self.timer.timeout.connect(self.poll); self.timer.start()
        self.name.textChanged.connect(self.update_name)
        for box in self.suffixes.values():
            box.toggled.connect(self.update_name)
        self.name_timer = QtCore.QTimer(self)
        self.name_timer.setInterval(1000)
        self.name_timer.timeout.connect(self.update_name)
        self.name_timer.start()
        self.update_name()
        self.async_call(backend.settings, self.loaded, failed=self.failed)

    def loaded(self, settings):
        self.root.set_path(settings.get('recording_root', ''))
        self._settings_loaded = True
        self.status_label.setText('尚未录制。选择保存位置，确认静止后开始。')
        self.update_controls()

    def active(self):
        return bool(self.task and self.task.get('status') in ACTIVE)

    def name_options(self):
        return dict(prefix=self.name.text(), **{key:box.isChecked() for key,box in self.suffixes.items()})

    def update_name(self,*_):
        if self.active():
            if self.task.get('output'):
                self.name_preview.setText('本次录制文件夹：'+Path(self.task['output']).name)
            return
        if self._busy:
            return
        from wc_runtime.capture_names import recording_folder_name
        try:
            name = recording_folder_name(self.name_options())
            self._name_valid = True
            self.name_preview.setText('文件夹预览：'+name)
        except ValueError as error:
            self._name_valid = False
            self.name_preview.setText(str(error))
        self.update_controls()

    def update_controls(self):
        active = self.active()
        idle = not active and not self._busy
        self.start_button.setEnabled(idle and self._settings_loaded and self._name_valid and self.static.isChecked())
        self.stop_button.setEnabled(active and not self._busy and self.task.get('status') != 'STOPPING')
        self.prepare_button.setEnabled(idle)
        for control in (self.root, self.name, self.static, self.dataset, *self.suffixes.values()):
            control.setEnabled(idle)
        self.use_button.setEnabled(idle and bool(self._prepared_path))

    def start(self):
        if self._busy or self.active() or not self.static.isChecked() or not self._settings_loaded:
            return
        self.update_name()
        if not self._name_valid:
            return
        options = self.name_options()
        self._busy = True
        self._prepared_path = ''
        self.update_controls()
        self.status_label.setText('正在启动双雷达录制与统一预览…')
        root = self.root.path()
        backend, cleanup = self.backend, self._cleanup

        def launch():
            row = backend.start_capture(root, 'all', manual_drive=False,
                name_options=options)
            cleanup['identifier'] = row['id']
            if cleanup['destroyed']:
                return backend.cancel(row['id'])
            return row
        self.async_call(launch, self.started, failed=self.failed)

    def started(self, row):
        self._busy = False
        self.show_task(row)
        pending, self._stop_pending = self._stop_pending, False
        if (pending or self._closing) and self.active() and row.get('status') != 'STOPPING':
            self.stop()

    def show_task(self, row):
        self.task = row
        self._cleanup['identifier'] = row['id'] if row.get('status') in ACTIVE else None
        output = row.get('output', '')
        if output:
            self.dataset.set_path(output)
        text = row.get('stage_message') or status_text(row.get('status'))
        if output:
            text += '\n原始录制：' + output
        if row.get('error'):
            text += '\n' + str(row['error'])
        self.status_label.setText(text)
        self.open_output_button.setEnabled(bool(output))
        self.update_name()
        self.update_controls()
        if row.get('status') not in ACTIVE:
            if self._closing:
                self.close()
            elif self._prepare_after_stop:
                self._prepare_after_stop = False
                if row.get('status') == 'COMPLETE':
                    self.prepare_existing()
                else:
                    self.detail.setText('录制未完整结束，原始数据保留；未生成选点输入，请查看日志或重新录制。')

    def poll(self):
        if self._busy or not self.active():
            return
        self._busy = True
        identifier = self.task['id']

        def status():
            row = next((row for row in self.backend.jobs() if row['id'] == identifier), None)
            if row is None:
                raise ValueError('未找到本次录制任务，请在日志窗口核查收尾状态')
            return row
        self.async_call(status, self.started, failed=self.failed)

    def stop_and_prepare(self):
        self._prepare_after_stop = True
        self.stop()

    def stop(self):
        if self._busy:
            self._stop_pending = True
            self.status_label.setText('已请求停止；当前状态检查返回后立即开始收尾。')
            return
        if not self.active():
            if self._closing:
                self.close()
            return
        if self.task.get('status') == 'STOPPING':
            return
        self._busy = True
        self.status_label.setText('正在停止并收尾；等待完整性检查和最终落盘后才会准备点云。')
        self.update_controls()
        identifier = self.task['id']
        self.async_call(lambda: self.backend.cancel(identifier), self.started, failed=self.failed)

    def prepare_existing(self):
        if self._busy or self.active():
            return
        dataset = self.dataset.path()
        if not dataset:
            self.status_label.setText('请先选择具体的录制子文件夹。')
            return
        self._prepared_path = ''
        self._busy = True
        self.update_controls()
        self.status_label.setText('正在扫描原始录包、核对采集来源和文件哈希，再提取帧对。\n较长录包需要更多时间；原数据保持不变，完成后显示新文件位置。')
        self.async_call(lambda: self.backend.prepare_pairing_capture(dataset), self.ready, failed=self.failed)

    def ready(self, result):
        self._busy = False
        if result.get('status') != 'READY_FOR_OFFLINE_PICKING' or not result.get('prepared_input'):
            self.failed('点云准备未提供有效结果，不能打开配对工具。')
            return
        self._prepared_path = result['prepared_input']
        self.status_label.setText('点云准备完成：\n' + self._prepared_path)
        detail = ('包含原始幅度图，可使用幅度图与点云联动选点。' if result.get('organized_amplitude_available')
                  else '原录包缺少完整的像素顺序或幅度信息；仅提供真实 XYZ 点云，不生成幅度图。')
        self.detail.setText(detail + '\n原始录制保持不变；场景静止仍是人工条件，未作独立验证。')
        self.update_controls()
        QtCore.QTimer.singleShot(0,self.reveal_result)
        if self._closing:
            self.close()

    def reveal_result(self,*_range):
        if self._prepared_path:
            self.body_scroll.ensureWidgetVisible(self.detail,0,12)

    def use_prepared(self):
        if self._prepared_path and not self._busy and not self.active():
            self.prepared.emit(self._prepared_path)
            self.close()

    def failed(self, message):
        self._busy = False
        self.status_label.setText(str(message))
        self._closing = False
        self.update_controls()
        pending, self._stop_pending = self._stop_pending, False
        if pending and self.active():
            self.stop()

    def open_recording(self):
        if self.dataset.path():
            local_open(self.dataset.path())

    def closeEvent(self, event):
        if self._busy or self.active():
            event.ignore()
            self._closing = True
            self._prepare_after_stop = False
            self.stop()
            return
        self.timer.stop()
        self.name_timer.stop()
        super().closeEvent(event)
