"""Inspect, name and switch validated local device parameter sets."""
import json
import math

from PyQt5 import QtCore, QtWidgets

from .ui_common import PanelDialog, button, combo, hint, local_open
from .ui_scroll import PanelScrollArea


def _dimension(value):
    return '{:g} mm'.format(value * 1000) if type(value) in (int, float) else '未知'


def _position(value):
    return '未知' if value is None else 'X {}、Y {}、Z {}'.format(*(_dimension(item) for item in value))


def _orientation(matrix):
    if matrix is None: return '未知'
    pitch = math.asin(max(-1., min(1., -matrix[2][0])))
    if abs(math.cos(pitch)) < 1e-6: return '俯仰约 ±90°，完整方向见技术明细'
    roll, yaw = math.atan2(matrix[2][1], matrix[2][2]), math.atan2(matrix[1][0], matrix[0][0])
    return '偏航 {:.2f}°、俯仰 {:.2f}°、横滚 {:.2f}°'.format(*(math.degrees(x) for x in (yaw, pitch, roll)))


def change_summary(changes):
    """A short units-aware description; complete JSON remains an opt-in detail."""
    lines = []
    names = {'wheel_radius_m': '轮半径', 'track_width_m': '左右轮中心距', 'register_to_wheel_rpm': '轮速换算系数',
             'left_sign': '左轮正负方向', 'right_sign': '右轮正负方向', 'max_gap_s': '轮数据最大间隔（秒）',
             'max_wheel_speed_m_s': '轮速上限（米/秒）'}
    frames = {'lidar_left': '左雷达', 'lidar_right': '右雷达', 'imu': 'IMU', 'imu_native': 'IMU原始坐标',
              'mounting_M': '安装参考系', 'axle': '轮轴参考系', 'wheel_axle': '轮轴参考系'}
    for change in changes:
        before, after = change['before'], change['after']
        lines.append(change['label'])
        if change['group'] == 'wheels':
            for key in before:
                if before[key] != after[key]:
                    a, b = (map(_dimension, (before[key], after[key])) if key in ('wheel_radius_m', 'track_width_m')
                            else (str(before[key]), str(after[key])))
                    lines.append('  {}：{} → {}'.format(names.get(key, key), a, b))
        elif change['group'] == 'extrinsics':
            for original, proposed in zip(before, after):
                if original == proposed: continue
                child, parent = proposed['child_frame'], proposed['parent_frame']
                lines.append('  {}相对{}：'.format(frames.get(child, child), frames.get(parent, parent)))
                for component, field, formatter, label in (('translation', 'value_m', _position, '位置'),
                        ('rotation', 'value_matrix', _orientation, '朝向')):
                    a, b = original[component], proposed[component]
                    if a.get(field) != b.get(field):
                        lines.append('    {}：{} → {}'.format(label, formatter(a.get(field)), formatter(b.get(field))))
                    elif a != b:
                        lines.append('    {}数值不变，测量状态或依据更新'.format(label))
        elif change['group'] == 'display':
            for role, value in after['camera_rotations'].items():
                old = before['camera_rotations'][role]
                if old != value: lines.append('  {}旋转：{}° → {}°'.format(role, old, value))
            for key, label in (('camera_profile', '相机显示方案'), ('rviz_renderer', '渲染方式')):
                if before[key] != after[key]: lines.append('  {}：{} → {}'.format(label, before[key], after[key]))
            if before['visualization'] != after['visualization']: lines.append('  车体显示参数更新，详细数值见技术明细')
        elif change['group'] == 'algorithms':
            titles = {'cloud_filter': '点云过滤', 'planar_ekf': '融合权重', 'map_profile': '地图参数',
                      'input_rate_hz': '默认输入频率（Hz）', 'max_pair_delta_ns': '帧配对时间阈值（ns）',
                      'wheel_imu_covariance': '轮速 / IMU 噪声', 'wheel_imu_estimator': '默认估计器'}
            estimators = {'five_state': '五状态 EKF', 'robot_localization': '官方 EKF'}
            for key in before:
                if before[key] == after[key]: continue
                if isinstance(after[key], (list, dict)):
                    lines.append('  {}已更新，详细数值见技术明细'.format(titles.get(key, key)))
                else:
                    a, b = before[key], after[key]
                    if key == 'wheel_imu_estimator': a, b = estimators.get(a, a), estimators.get(b, b)
                    lines.append('  {}：{} → {}'.format(titles.get(key, key), a, b))
        else:
            lines.append('  在线运动校正文件引用更新；仅在选择对应校正的任务中生效')
    return '\n'.join(lines) or '该配置集与当前支持的参数值相同。'


class ProfilesDialog(PanelDialog):
    parametersChanged = QtCore.pyqtSignal()

    def __init__(self, backend, parent=None):
        super().__init__(backend, '设备配置集', parent, (850, 700))
        self.snapshot = {}
        self.proposal = None
        self._busy = False
        self.layout.addWidget(hint('为当前设备保存多套参数，并在停止实时／录制任务后切换。设备身份、装配及坐标系绑定保持不变；已录制数据继续使用录包内冻结的参数。'))
        scroll = PanelScrollArea(self)
        self.body_scroll=scroll
        content = QtWidgets.QWidget()
        scroll.setWidget(content)
        scroll.verticalScrollBar().rangeChanged.connect(self.reveal_changes)
        body = QtWidgets.QVBoxLayout(content)
        body.setContentsMargins(2, 0, 12, 8)
        self.active = QtWidgets.QLabel('正在读取当前配置…')
        self.active.setWordWrap(True)
        body.addWidget(self.active)
        self.paths = hint('')
        body.addWidget(self.paths)
        self.directory_button = button('打开配置文件夹', self.open_directory)
        body.addWidget(self.directory_button, 0, QtCore.Qt.AlignLeft)
        save_box = QtWidgets.QGroupBox('保存当前参数')
        save_layout = QtWidgets.QVBoxLayout(save_box)
        save_layout.addWidget(hint('只新增命名配置集，不改变当前数值。例：室内日常、轮尺寸复测后。'))
        save_row = QtWidgets.QHBoxLayout()
        self.name = QtWidgets.QLineEdit()
        self.name.setMaxLength(64)
        self.name.setPlaceholderText('输入容易区分的配置名称')
        self.save_button = button('另存配置集', self.save_current)
        save_row.addWidget(self.name, 1); save_row.addWidget(self.save_button)
        save_layout.addLayout(save_row)
        body.addWidget(save_box)
        selection_box = QtWidgets.QGroupBox('选择其他参数')
        selection_layout = QtWidgets.QVBoxLayout(selection_box)
        selection_row = QtWidgets.QHBoxLayout()
        self.profiles = combo([])
        self.profiles.currentIndexChanged.connect(self.selection_changed)
        self.preview_button = button('预览变更', self.preview)
        selection_row.addWidget(self.profiles, 1); selection_row.addWidget(self.preview_button)
        selection_layout.addLayout(selection_row)
        self.selection_detail = hint('')
        selection_layout.addWidget(self.selection_detail)
        self.change_summary = hint('先选择配置集，再预览变更。')
        selection_layout.addWidget(self.change_summary)
        self.details_button = button('查看技术明细', self.toggle_details)
        selection_layout.addWidget(self.details_button, 0, QtCore.Qt.AlignLeft)
        self.preview_text = QtWidgets.QPlainTextEdit()
        self.preview_text.setReadOnly(True)
        self.preview_text.setMinimumHeight(150)
        self.preview_text.setPlaceholderText('先选择一个配置集，再预览将修改的参数。')
        self.preview_text.hide()
        selection_layout.addWidget(self.preview_text)
        body.addWidget(selection_box)
        body.addStretch()
        self.layout.addWidget(scroll, 1)
        self.status = hint('')
        self.layout.addWidget(self.status)
        footer = QtWidgets.QHBoxLayout()
        self.refresh_button = button('刷新', self.refresh)
        self.apply_button = button('确认切换', self.apply, primary=True)
        self.apply_button.setEnabled(False)
        footer.addWidget(self.refresh_button); footer.addStretch()
        footer.addWidget(button('关闭', self.close)); footer.addWidget(self.apply_button)
        for index in range(footer.count()):
            item = footer.itemAt(index)
            if item.widget(): item.widget().setProperty('popupFooter', True)
        self.layout.addLayout(footer)
        self.refresh()

    def set_busy(self, value):
        self._busy = value
        for widget in (self.save_button, self.refresh_button, self.name, self.profiles):
            widget.setEnabled(not value)
        self.preview_button.setEnabled(not value and bool(self.profiles.currentData())
                                       and self.profiles.currentData().get('available', False))
        self.apply_button.setEnabled(not value and bool(self.proposal))

    def refresh(self):
        if self._busy: return
        self.proposal = None
        self.set_busy(True)
        self.async_call(self.backend.list_configuration_profiles, self.loaded, failed=self.failed)

    def loaded(self, snapshot):
        self.snapshot = snapshot
        self.active.setText('当前使用：' + snapshot['active_name'])
        self.paths.setText('正在生效的文件：\n' + '\n'.join(snapshot['config_paths']) +
                           '\n\n配置集目录：' + snapshot['profile_root'])
        previous = self.profiles.currentData()
        selected = previous.get('id') if previous else None
        self.profiles.blockSignals(True)
        self.profiles.clear()
        for row in snapshot['profiles']:
            self.profiles.addItem(row['name'] + (' · 依据不可用' if not row['available'] else ''), row)
            if row['id'] == selected: self.profiles.setCurrentIndex(self.profiles.count()-1)
        self.profiles.blockSignals(False)
        self.set_busy(False)
        self.selection_changed()
        self.status.setText('保存与切换均会检查当前配置版本和测量依据。' if snapshot['profiles'] else '尚无命名配置集，可先保存当前参数。')

    def selection_changed(self):
        self.proposal = None
        self.preview_text.clear()
        self.change_summary.setText('先选择配置集，再预览变更。')
        self.apply_button.setEnabled(False)
        row = self.profiles.currentData()
        self.preview_button.setEnabled(bool(row and row.get('available')) and not self._busy)
        self.selection_detail.setText((row.get('error') or row.get('path', '')) if row else '尚无命名配置集。')

    def failed(self, message):
        self.proposal = None
        self.set_busy(False)
        self.status.setText(str(message))

    def open_directory(self):
        if self.snapshot.get('config_paths'):
            from pathlib import Path
            local_open(Path(self.snapshot['config_paths'][0]).parent)

    def save_current(self):
        if self._busy: return
        name = self.name.text().strip()
        if not name:
            self.status.setText('请先填写配置名称。'); self.name.setFocus(); return
        revision = self.snapshot.get('revision')
        self.set_busy(True)
        self.async_call(lambda: self.backend.save_configuration_profile(name, revision), self.saved, failed=self.failed)

    def saved(self, row):
        self.set_busy(False)
        self.name.clear()
        self.refresh()

    def preview(self):
        row = self.profiles.currentData()
        if self._busy or not row or not row.get('available'): return
        identifier = row['id']
        self.set_busy(True)
        self.async_call(lambda: self.backend.preview_configuration_profile(identifier), self.previewed, failed=self.failed)

    def previewed(self, row):
        self.proposal = row
        blocks = []
        for change in row['changes']:
            before = json.dumps(change['before'], ensure_ascii=False, indent=2)
            after = json.dumps(change['after'], ensure_ascii=False, indent=2)
            blocks.append(change['label'] + '\n当前：\n' + before + '\n切换后：\n' + after)
        self.preview_text.setPlainText('\n\n'.join(blocks) if blocks else '该配置集与当前支持的参数值相同。')
        self.change_summary.setText(change_summary(row['changes']))
        self.status.setText('将切换到“{}”：{}。确认前请核对上方变更。'.format(row['name'],
            '、'.join(change['label'] for change in row['changes']) or '数值不变'))
        self.set_busy(False)

        QtCore.QTimer.singleShot(0,self.reveal_changes)

    def reveal_changes(self,*_range):
        if self.proposal:
            self.body_scroll.ensureWidgetVisible(self.change_summary,0,12)

    def toggle_details(self):
        visible = not self.preview_text.isHidden()
        self.preview_text.setVisible(not visible)
        self.details_button.setText('查看技术明细' if visible else '收起技术明细')

    def apply(self):
        if self._busy or not self.proposal: return
        row = self.proposal
        answer = QtWidgets.QMessageBox.warning(self, '确认切换设备配置',
            '切换到“{}”？\n\n{}'.format(row['name'], row['message']),
            QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel, QtWidgets.QMessageBox.Cancel)
        if answer != QtWidgets.QMessageBox.Ok: return
        token = row['token']
        self.set_busy(True)
        self.async_call(lambda: self.backend.apply_configuration_profile(token), self.applied, failed=self.failed)

    def applied(self, row):
        self.proposal = None
        self.set_busy(False)
        self.parametersChanged.emit()
        QtWidgets.QMessageBox.information(self, '配置已切换',
            '当前配置：{}\n对后续任务生效；录包冻结配置不变。\n原配置备份：{}'.format(row['name'], row['backup']) +
            ('\n' + row['cleanup_warning'] if row.get('cleanup_warning') else ''))
        self.refresh()
