"""Plain-language device overview and the existing offline cloud-pairing tools."""
import copy
import math
from pathlib import Path
import threading

from PyQt5 import QtCore, QtGui, QtWidgets, sip

from .ui_common import BackgroundTask, PanelDialog, button, combo, hint, local_open
from .ui_scroll import PanelScrollArea


STATUS_NAMES = {'UNKNOWN': '未测量', 'UNVALIDATED': '未核验', 'CAD_NOMINAL': '机械标称',
                'USER_MEASURED_EXPERIMENT': '人工实测（实验）', 'CALIBRATED': '已标定',
                'LEGACY_CANDIDATE': '历史候选'}


def _dimension(value):
    if type(value) not in (float, int) or not math.isfinite(value):
        return '未知'
    return '{:g} mm'.format(value * 1000)


def overview_values(snapshot):
    """Summarize frame origins only; mechanical faces are not lidar origins."""
    groups = {row['id']: row for row in snapshot.get('groups', [])}
    wheels = groups.get('wheels', {})
    transforms = groups.get('extrinsics', {}).get('value', [])
    readings = {}
    for side in ('left', 'right'):
        readings[side + '_height'] = dict(
            label=('左' if side == 'left' else '右') + '雷达离地高度', value='未测量',
            detail='从地面量到雷达数据原点；外壳表面高度不能直接代替。')
    readings['spacing'] = dict(label='两雷达原点间距', value='未测量',
                              detail='指左右雷达数据原点的直线距离，不是两侧外壳的跨度。')

    def translation(row):
        part = row.get('translation', {})
        value = part.get('value_m')
        if (row.get('unit') != 'm' or row.get('direction') != 'child_to_parent'
                or part.get('reference_kind') != 'FRAME_ORIGIN' or part.get('status') not in STATUS_NAMES
                or part.get('status') == 'UNKNOWN' or not isinstance(value, list) or len(value) != 3
                or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
            return None
        return value

    if isinstance(transforms, list):
        by_side = {}
        for row in transforms:
            if not isinstance(row, dict):
                continue
            vector = translation(row)
            if vector is None:
                continue
            child, parent = row.get('child_frame'), row.get('parent_frame')
            status = STATUS_NAMES[row['translation']['status']]
            if child in ('lidar_left', 'lidar_right'):
                by_side[child] = row
                if parent == 'ground' and vector[2] >= 0:
                    readings[child.removeprefix('lidar_') + '_height'].update(
                        value=_dimension(vector[2]), detail='数据原点相对 ground 地面坐标系的高度 · ' + status)
            if {child, parent} == {'lidar_left', 'lidar_right'}:
                readings['spacing'].update(value=_dimension(math.sqrt(sum(x*x for x in vector))),
                    detail='直接左右雷达原点变换的平移长度 · ' + status)
        left, right = by_side.get('lidar_left'), by_side.get('lidar_right')
        if (readings['spacing']['value'] == '未测量' and left and right
                and left['parent_frame'] == right['parent_frame']):
            a, b = translation(left), translation(right)
            readings['spacing'].update(value=_dimension(math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))),
                detail='同一参考系 {} 中的原点距离 · 左{} / 右{}'.format(left['parent_frame'],
                    STATUS_NAMES[left['translation']['status']], STATUS_NAMES[right['translation']['status']]))
    values = wheels.get('value', {})
    wheel_status = STATUS_NAMES.get(wheels.get('status', 'UNVALIDATED'), '未核验')
    readings['radius'] = dict(label='轮半径', value=_dimension(values.get('wheel_radius_m')),
                             detail='车轮中心到轮缘 · 当前配置，' + wheel_status)
    readings['track'] = dict(label='左右轮中心距', value=_dimension(values.get('track_width_m')),
                            detail='两侧驱动轮中心之间的距离 · 当前配置，' + wheel_status)
    from .installation_parameters import principal_values,frame_name
    readings['mounting'] = dict(label='安装基准相对轮轴', value='位置未确定',
        detail='安装基准 M 的原点相对轮轴中心；不代表雷达数据原点。')
    readings['imu'] = dict(label='IMU 朝向', value='朝向未确定',
        detail='IMU 的位置与朝向是两项独立参数，均可在安装参数中查看。')
    for row in transforms if isinstance(transforms,list) else []:
        if not isinstance(row,dict): continue
        try: principal=principal_values(row)
        except (TypeError,ValueError,KeyError): continue
        if row.get('child_frame')=='mounting_M' and row.get('parent_frame')=='axle' and principal['position_mm'] is not None:
            x,y,z=principal['position_mm']
            readings['mounting'].update(value='前后 {:g} · 左右 {:g} · 上下 {:g} mm'.format(x,y,z),
                detail='相对轮轴中心，前／左／上为正；'+STATUS_NAMES.get(row['translation'].get('status'),'未知'))
        if row.get('child_frame')=='imu_native' and principal['orientation_deg'] is not None:
            r,p,y=(0. if abs(angle)<1e-9 else angle for angle in principal['orientation_deg'])
            readings['imu'].update(value='横滚 {:g}° · 俯仰 {:g}° · 偏航 {:g}°'.format(r,p,y),
                detail='相对 '+frame_name(row.get('parent_frame','未知'))+'；位置'+('未确定' if principal['position_mm'] is None else '已提供')+'。角度顺序及含义见安装参数。')
    return readings


class DeviceOverview(QtWidgets.QWidget):
    advancedRequested = QtCore.pyqtSignal(str)
    parametersChanged = QtCore.pyqtSignal()

    def __init__(self, backend, parent=None):
        super().__init__(parent)
        self.backend = backend
        self.snapshot = {}
        self._pairing = self._wheel_dialog = self._installation_dialog = self._profiles_dialog = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        scroll = PanelScrollArea(self)
        body = QtWidgets.QWidget()
        scroll.setWidget(body)
        layout.addWidget(scroll)
        content = QtWidgets.QVBoxLayout(body)
        content.setContentsMargins(2, 2, 12, 8)
        content.setSpacing(16)
        content.addWidget(hint('当前设备参数用于后续录制与在线任务；已有录包的离线融合沿用录包内冻结配置。'))
        self.config_paths=hint('')
        primary=QtWidgets.QHBoxLayout()
        self.installation_button=button('安装参数与 IMU',self.edit_installation,primary=True)
        self.profiles_button=button('选择／保存配置集',self.open_profiles)
        primary.addWidget(self.installation_button);primary.addWidget(self.profiles_button);primary.addStretch()
        content.addLayout(primary)
        content.addWidget(self.config_paths)
        self.offline_note=hint('')
        content.addWidget(self.offline_note)
        self.cards = {}
        grid = QtWidgets.QGridLayout()
        wheels_grid = QtWidgets.QGridLayout()
        grid.setSpacing(12)
        wheels_grid.setSpacing(12)
        for index, key in enumerate(('mounting', 'imu', 'spacing', 'radius', 'track')):
            card = QtWidgets.QFrame()
            card.setObjectName('deviceMetricCard')
            card.setStyleSheet('QFrame#deviceMetricCard { background:white; border:1px solid #d9e0e8; border-radius:14px; }')
            card_layout = QtWidgets.QVBoxLayout(card)
            card_layout.setContentsMargins(17, 14, 17, 14)
            title, value, detail = QtWidgets.QLabel(), QtWidgets.QLabel(), hint('')
            value.setStyleSheet('font-size:19px; font-weight:600; color:#183b61;')
            for label in (title, value, detail):
                label.setWordWrap(True)
                card_layout.addWidget(label)
            self.cards[key] = (title, value, detail)
            if index < 3:
                grid.addWidget(card, 0, index)
                grid.setColumnStretch(index, 1)
            else:
                wheels_grid.addWidget(card, 0, index-3)
                wheels_grid.setColumnStretch(index-3, 1)
        content.addLayout(grid)
        content.addLayout(wheels_grid)
        self.notice = hint('未知项保留未知。高度、间距只是尺寸，不能单独确定两个雷达之间的完整平移和旋转。')
        content.addWidget(self.notice)
        row = QtWidgets.QHBoxLayout()
        self.wheel_button = button('修改轮尺寸', self.edit_wheels)
        self.display_button = button('调整画面与显示', lambda: self.advancedRequested.emit('display'))
        row.addWidget(self.wheel_button); row.addWidget(self.display_button); row.addStretch()
        content.addLayout(row)
        pairing = QtWidgets.QGroupBox('双雷达点云配对')
        pairing_layout = QtWidgets.QVBoxLayout(pairing)
        pairing_layout.addWidget(hint('可录制新的静态双雷达数据，或选择已录制数据准备点云，再选取对应点／拖动右云进行 ICP。用于计算右雷达到左雷达的坐标转换。'))
        self.pairing_button = button('打开点云配对工具', self.open_pairing, primary=True)
        pairing_layout.addWidget(self.pairing_button, 0, QtCore.Qt.AlignLeft)
        pairing_layout.addWidget(hint('打开此入口不启动硬件；新录制由用户单独确认。配对结果另存为候选，不会自动覆盖设备参数。'))
        content.addWidget(pairing)
        advanced = QtWidgets.QHBoxLayout()
        for label, group in (('完整外参与测量依据', 'extrinsics'), ('更多轮参数', 'wheels'), ('高级算法设置', 'algorithms')):
            advanced.addWidget(button(label, lambda checked=False, selected=group: self.advancedRequested.emit(selected)))
        content.addLayout(advanced)
        content.addStretch()
        self.refresh({})

    def refresh(self, snapshot):
        self.snapshot = copy.deepcopy(snapshot or {})
        for key, row in overview_values(self.snapshot).items():
            if key not in self.cards: continue
            for widget, value in zip(self.cards[key], (row['label'], row['value'], row['detail'])):
                widget.setText(value)
        groups = {row['id'] for row in self.snapshot.get('groups', [])}
        self.wheel_button.setEnabled('wheels' in groups)
        self.display_button.setEnabled('display' in groups)
        self.pairing_button.setEnabled(hasattr(self.backend, 'start_calibration'))
        self.installation_button.setEnabled('extrinsics' in groups)
        self.profiles_button.setEnabled(hasattr(self.backend,'list_configuration_profiles'))
        paths=list(dict.fromkeys(row['path'] for row in self.snapshot.get('groups',[]) if row.get('path')))
        self.config_paths.setText('当前配置目录：'+str(Path(paths[0]).parent)+' · 完整文件列表见“选择／保存配置集”' if paths else '')
        self.config_paths.setToolTip('\n'.join(paths))
        from .installation_parameters import installation_summary
        summary=installation_summary(self.snapshot)
        self.offline_note.setText(('离线机械初值：当前配置可生成显式的离线比较初值，但它不是已标定外参，也不能用于解除实时建图限制。\n'
            if summary.get('offline_initial') else '离线初值：'+summary.get('offline_error','')+'\n')+
            '历史结果的实际外参请以各自 runtime_config.json／frozen_runtime_config.json 为准。这里的“未测量”指当前正式数据原点外参未补齐，不表示旧离线结果没有使用坐标转换。')

    def edit_installation(self):
        if self._installation_dialog is not None and not sip.isdeleted(self._installation_dialog):
            self._installation_dialog.raise_();self._installation_dialog.activateWindow();return
        from .ui_installation import InstallationDialog
        self._installation_dialog=InstallationDialog(self.backend,self.snapshot,self)
        self._installation_dialog.saved.connect(self._refresh_from_backend)
        self._installation_dialog.advancedRequested.connect(self.open_advanced)
        self._installation_dialog.show()

    def open_advanced(self,group):
        self.advancedRequested.emit(group)
        self.window().raise_();self.window().activateWindow()

    def open_profiles(self):
        if self._profiles_dialog is not None and not sip.isdeleted(self._profiles_dialog):
            self._profiles_dialog.raise_();self._profiles_dialog.activateWindow();return
        from .ui_profiles import ProfilesDialog
        self._profiles_dialog=ProfilesDialog(self.backend,self)
        self._profiles_dialog.parametersChanged.connect(self._refresh_from_backend)
        self._profiles_dialog.show()

    def edit_wheels(self):
        if self._wheel_dialog is not None and not sip.isdeleted(self._wheel_dialog):
            self._wheel_dialog.raise_(); self._wheel_dialog.activateWindow(); return
        self._wheel_dialog = WheelDimensionsDialog(self.backend, self.snapshot, self)
        self._wheel_dialog.saved.connect(self._refresh_from_backend)
        self._wheel_dialog.show()

    def _refresh_from_backend(self):
        self.parametersChanged.emit()
        BackgroundTask(self, self.backend.device_parameters, self.refresh)

    def open_pairing(self):
        if self._pairing is not None and not sip.isdeleted(self._pairing):
            self._pairing.raise_(); self._pairing.activateWindow(); return
        self._pairing = PairingDialog(self.backend, self)
        self._pairing.show()


class WheelDimensionsDialog(PanelDialog):
    saved = QtCore.pyqtSignal()

    def __init__(self, backend, snapshot, parent=None):
        super().__init__(backend, '轮尺寸', parent, (570, 390))
        self.snapshot = copy.deepcopy(snapshot)
        self.original = next(row['value'] for row in snapshot['groups'] if row['id'] == 'wheels')
        self.permission = None
        self.layout.addWidget(hint('单位为毫米。轮半径影响行驶距离，左右轮中心距影响转弯估计。保存后对下一次任务生效。'))
        form = QtWidgets.QFormLayout()
        self.fields = {}
        for label, key in (('轮半径（轮中心到轮缘）', 'wheel_radius_m'), ('左右轮中心距', 'track_width_m')):
            edit = QtWidgets.QDoubleSpinBox()
            edit.setRange(0, 10000); edit.setDecimals(3); edit.setSuffix(' mm'); edit.setSpecialValueText('未知，请先测量')
            value = self.original.get(key)
            if type(value) in (int, float) and math.isfinite(value):
                edit.setValue(value * 1000)
            edit.setEnabled(False)
            self.fields[key] = edit
            form.addRow(label, edit)
        self.layout.addLayout(form)
        self.state = hint('当前只读。其他轮参数保持原值；如需调整轮方向等参数，请打开“更多轮参数”。')
        self.layout.addWidget(self.state)
        self.layout.addStretch()
        row = QtWidgets.QHBoxLayout()
        self.edit_button = button('开始修改', self.authorize)
        self.save_button = button('保存轮尺寸', self.save, primary=True)
        self.save_button.setEnabled(False)
        row.addWidget(self.edit_button); row.addStretch(); row.addWidget(button('关闭', self.close)); row.addWidget(self.save_button)
        self.layout.addLayout(row)

    def authorize(self):
        answer = QtWidgets.QMessageBox.warning(self, '修改尺寸前请确认',
            '请使用实际测量的轮半径和轮中心距。错误尺寸会影响路线尺度和转弯估计；保存前将备份原配置。',
            QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel, QtWidgets.QMessageBox.Cancel)
        if answer != QtWidgets.QMessageBox.Ok:
            return
        self.edit_button.setEnabled(False)
        self.async_call(lambda: self.backend.request_parameter_edit('wheels'), self.authorized,
                        failed=self.failed)

    def authorized(self, permission):
        if permission.get('revision') != self.snapshot.get('revision'):
            self.failed('设备配置已改变，请重新打开设备参数后再修改。'); return
        self.permission = permission
        for field in self.fields.values():
            field.setEnabled(True)
        self.save_button.setEnabled(True)
        self.state.setText('修改完成后点击“保存轮尺寸”。当前运行中的任务保持原配置。')

    def failed(self, message):
        self.state.setText(str(message))
        self.edit_button.setEnabled(self.permission is None)
        self.save_button.setEnabled(self.permission is not None)

    def save(self):
        if not self.permission:
            return
        value = copy.deepcopy(self.original)
        for key, field in self.fields.items():
            if field.value() <= 0:
                self.failed('请为两项尺寸填写大于零的实测值；未知项不能按零保存。')
                return
            value[key] = field.value() / 1000
        self.save_button.setEnabled(False)
        self.async_call(lambda: self.backend.save_parameters('wheels', value,
            warning_token=self.permission['token'], expected_revision=self.snapshot['revision']), self.saved_value, failed=self.failed)

    def saved_value(self, response):
        self.saved.emit()
        QtWidgets.QMessageBox.information(self, '轮尺寸已保存', '对下一次任务生效。\n原配置备份：' + str(response.get('backup', '')))
        self.close()


class PairingDialog(PanelDialog):
    def __init__(self, backend, parent=None):
        super().__init__(backend, '双雷达点云配对', parent, (820, 640))
        self.task = None
        self._busy = False
        self._closing = False
        self._capture_dialog = None
        # Qt can destroy a child with its parent without sending closeEvent.
        # Keep cleanup state independent of QObject lifetime, including a start
        # that completes after the containing device window was destroyed.
        self._cleanup = dict(identifier=None, destroyed=False)
        cleanup = self._cleanup

        def stop_destroyed_owner():
            cleanup['destroyed'] = True
            identifier = cleanup['identifier']
            if identifier:
                def stop_owned():
                    try:
                        backend.stop_calibration(identifier)
                    except (OSError, ValueError, RuntimeError):
                        pass
                threading.Thread(target=stop_owned, daemon=True).start()
        self.destroyed.connect(stop_destroyed_owner)
        self.layout.addWidget(hint('使用已准备的双雷达点云。左云作为参考，求右雷达 → 左雷达的变换；结果保持为离线候选。'))
        self.capture_button=button('录制新数据／从录包准备点云…',self.open_capture)
        self.layout.addWidget(self.capture_button,0,QtCore.Qt.AlignLeft)
        form = QtWidgets.QFormLayout()
        input_row = QtWidgets.QWidget()
        input_layout = QtWidgets.QHBoxLayout(input_row); input_layout.setContentsMargins(0, 0, 0, 0)
        self.input = QtWidgets.QLineEdit()
        self.input.setPlaceholderText('prepared.json · 可沿用既有工具的默认点云')
        self.choose_button = button('选择文件…', self.choose_input)
        input_layout.addWidget(self.input, 1); input_layout.addWidget(self.choose_button)
        form.addRow('点云文件', input_row)
        self.mode = combo([('对应点配对', 'points'), ('拖动点云 / ICP', 'alignment')])
        form.addRow('操作方式', self.mode)
        self.layout.addLayout(form)
        self.input_detail = hint('正在读取默认点云…')
        self.layout.addWidget(self.input_detail)
        self.layout.addWidget(hint('对应点配对：分别点选同一个位置，计算坐标转换。\n拖动 / ICP：先移动、转动右侧点云，再细化对齐。两种方式均复用已有工具，在浏览器中打开。'))
        self.status_label = QtWidgets.QLabel('尚未启动')
        self.status_label.setWordWrap(True)
        self.status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.layout.addWidget(self.status_label)
        self.output = hint('')
        self.output.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.layout.addWidget(self.output)
        self.layout.addStretch()
        links = QtWidgets.QHBoxLayout()
        self.open_button = button('重新打开工具', self.open_browser)
        self.log_button = button('查看运行日志', self.open_log)
        self.output_button = button('打开候选目录', self.open_output)
        for widget in (self.open_button, self.log_button, self.output_button):
            widget.setEnabled(False); links.addWidget(widget)
        self.layout.addLayout(links)
        row = QtWidgets.QHBoxLayout()
        self.stop_button = button('停止本次工具', self.stop)
        self.start_button = button('启动并打开工具', self.start, primary=True)
        self.start_button.setEnabled(False); self.stop_button.setEnabled(False)
        row.addWidget(self.stop_button); row.addStretch(); row.addWidget(button('关闭', self.close)); row.addWidget(self.start_button)
        self.layout.addLayout(row)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(1200); self.timer.timeout.connect(self.poll); self.timer.start()
        self.async_call(lambda: (backend.calibration_defaults(), backend.calibration_status()), self.loaded, failed=self.failed)

    def open_capture(self):
        if self._capture_dialog is not None and not sip.isdeleted(self._capture_dialog):
            self._capture_dialog.raise_();self._capture_dialog.activateWindow();return
        from .ui_pairing_capture import PairingCaptureDialog
        self._capture_dialog=PairingCaptureDialog(self.backend,self)
        self._capture_dialog.prepared.connect(self.capture_prepared)
        self._capture_dialog.show()

    def capture_prepared(self,path):
        if self._busy or (self.task and (self.task.get('status') in ('STARTING','RUNNING','STOPPING') or self.task.get('process_alive'))):
            self.show_error('配对工具仍在使用原点云。请停止工具后选择本次准备文件：\n'+path)
            return
        self.input.setText(path)
        self.input_detail.setText('使用本次准备文件；请保持场景静止。主机到达时间接近不代表硬件同步。')

    def loaded(self, response):
        defaults, row = response
        self.input.setText(defaults.get('input', ''))
        self.input_detail.setText(defaults.get('error') or '沿用既有工具的历史默认点云，不保证是最新场景。请核对文件路径，或选择其他 prepared.json。')
        self.output.setText('对应点保存：{}\n配准候选保存：{}'.format(defaults.get('output_root', ''), defaults.get('alignment_output_root', '')))
        self.start_button.setEnabled(True)
        if row:
            self.show_task(row)

    def choose_input(self):
        selected, _ = QtWidgets.QFileDialog.getOpenFileName(self, '选择已准备的双雷达点云', self.input.text(), '点云准备文件 (*.json)')
        if selected:
            self.input.setText(selected)
            self.input_detail.setText('启动时将检查双雷达来源、场景和点云格式。')

    def start(self):
        if self._busy:
            return
        if self._capture_dialog is not None and not sip.isdeleted(self._capture_dialog) and self._capture_dialog.isVisible():
            self.status_label.setText('请先在录制窗口完成点云准备并选择使用，或关闭录制窗口，再启动配对工具。')
            return
        self._busy = True
        self.start_button.setEnabled(False)
        self.input.setEnabled(False); self.mode.setEnabled(False); self.choose_button.setEnabled(False);self.capture_button.setEnabled(False)
        self.status_label.setText('正在启动并检查本机服务…')
        source, mode = self.input.text().strip(), self.mode.currentData()
        backend, cleanup = self.backend, self._cleanup

        def launch():
            row = backend.start_calibration(source, mode)
            cleanup['identifier'] = row.get('id')
            if cleanup['destroyed'] and row.get('id'):
                return backend.stop_calibration(row['id'])
            return row
        self.async_call(launch, self.started, failed=self.failed)

    def started(self, row):
        self._busy = False
        self.show_task(row)
        if self._closing:
            self.stop()
        elif row.get('status') == 'RUNNING':
            self.open_browser()

    def show_task(self, row):
        self.task = row
        self._cleanup['identifier'] = row.get('id')
        active = row.get('status') in ('STARTING', 'RUNNING', 'STOPPING') or row.get('process_alive', False)
        states = {'STARTING': '正在启动', 'RUNNING': '工具已就绪', 'STOPPING': '正在停止', 'COMPLETE': '工具已关闭', 'FAILED': '工具未完成'}
        text = states.get(row.get('status'), str(row.get('status')))
        if row.get('error'):
            text += '\n' + row['error']
        if row.get('url'):
            text += '\n' + row['url']
        self.status_label.setText(text)
        self.start_button.setEnabled(not active and not self._busy)
        self.stop_button.setEnabled(active and not self._busy)
        self.open_button.setEnabled(row.get('status') == 'RUNNING' and bool(row.get('url')))
        self.log_button.setEnabled(bool(row.get('log_path')))
        self.output_button.setEnabled(bool(row.get('output_root')))
        for control in (self.input, self.mode, self.choose_button, self.capture_button):
            control.setEnabled(not active and not self._busy)
        if active and row.get('input'):
            self.input.setText(row['input'])
            self.mode.setCurrentIndex(max(0, self.mode.findData(row.get('mode'))))

    def poll(self):
        if self._busy or not self.task or (self.task.get('status') not in ('STARTING', 'RUNNING', 'STOPPING') and not self.task.get('process_alive')):
            return
        self._busy = True
        identifier = self.task['id']
        self.async_call(lambda: self.backend.calibration_status(identifier), self.polled, failed=self.failed)

    def polled(self, row):
        self._busy = False
        self.show_task(row)
        if self._closing:
            self.stop() if row.get('status') in ('STARTING', 'RUNNING', 'STOPPING') or row.get('process_alive') else self.close()

    def failed(self, message):
        self._busy = False
        self.status_label.setText(str(message))
        self.start_button.setEnabled(True)
        for control in (self.input, self.mode, self.choose_button, self.capture_button):
            control.setEnabled(True)
        if self._closing:
            self._closing = False

    def stop(self):
        if self._busy:
            return
        if not self.task or (self.task.get('status') not in ('STARTING', 'RUNNING', 'STOPPING') and not self.task.get('process_alive')):
            if self._closing:
                self.close()
            return
        self._busy = True
        self.stop_button.setEnabled(False)
        self.status_label.setText('正在停止本次工具；已导出的候选会保留。')
        identifier = self.task['id']
        self.async_call(lambda: self.backend.stop_calibration(identifier), self.stopped, failed=self.failed)

    def stopped(self, row):
        self._busy = False
        self.show_task(row)
        if self._closing and row.get('status') == 'COMPLETE':
            self.close()
        else:
            self._closing = False

    def open_browser(self):
        if self.task and self.task.get('status') == 'RUNNING' and self.task.get('url'):
            if not QtGui.QDesktopServices.openUrl(QtCore.QUrl(self.task['url'])):
                self.status_label.setText('浏览器未能自动打开，可复制本机地址：\n' + self.task['url'])

    def open_log(self):
        if self.task and self.task.get('log_path'):
            local_open(self.task['log_path'])

    def open_output(self):
        if self.task and self.task.get('output_root'):
            local_open(self.task['output_root'])

    def closeEvent(self, event):
        if self._busy or (self.task and (self.task.get('status') in ('STARTING', 'RUNNING', 'STOPPING') or self.task.get('process_alive'))):
            event.ignore()
            self._closing = True
            self.stop()
            return
        self.timer.stop()
        super().closeEvent(event)
