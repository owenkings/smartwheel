"""Principal installation measurements instead of editable rotation matrices."""
import copy
import math

from PyQt5 import QtCore, QtGui, QtWidgets

from .installation_parameters import (CORE_FRAMES, STATUS_NAMES, USABLE_STATUSES,
    derived_transforms, frame_name, installation_summary, principal_values, update_component)
from .ui_common import PanelDialog, button, combo, hint
from .ui_scroll import PanelScrollArea


def _numbers(values, unit=''):
    if values is None:
        return '未确定'
    return ' / '.join('{:.3f}'.format(0 if abs(x) < .0005 else x).rstrip('0').rstrip('.') or '0' for x in values) + unit


class PrincipalComponent(QtWidgets.QGroupBox):
    changed = QtCore.pyqtSignal()
    importRequested = QtCore.pyqtSignal(object)

    def __init__(self, row, component, sources, assembly, parent=None):
        self.component = component
        self.original = copy.deepcopy(row[component])
        self.assembly = assembly
        self.sources = copy.deepcopy(sources)
        self._editable = False
        self._loading = True
        title = '位置 · 毫米' if component == 'translation' else '朝向 · 角度'
        super().__init__(title, parent)
        layout = QtWidgets.QVBoxLayout(self)
        labels = ('前后 X（前为正）', '左右 Y（左为正）', '上下 Z（上为正）') if component == 'translation' else (
            '横滚 Roll（绕 X）', '俯仰 Pitch（绕 Y）', '偏航 Yaw（绕 Z）')
        self.status = combo([(STATUS_NAMES[value], value) for value in STATUS_NAMES])
        self.status.setCurrentIndex(self.status.findData(self.original.get('status', 'UNKNOWN')))
        self.status.setEnabled(False)
        layout.addWidget(self.status)
        grid = QtWidgets.QGridLayout()
        self.fields = []
        value = principal_values(row)['position_mm' if component == 'translation' else 'orientation_deg']
        self.initial_text = []
        for index, label in enumerate(labels):
            grid.addWidget(QtWidgets.QLabel(label), 0, index)
            edit = QtWidgets.QLineEdit()
            validator = QtGui.QDoubleValidator(-3000 if component == 'translation' else -360,
                                              3000 if component == 'translation' else 360, 6, edit)
            validator.setLocale(QtCore.QLocale.c())
            validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
            edit.setValidator(validator)
            edit.setPlaceholderText('未知，留空')
            text = '' if value is None else '{:.6f}'.format(
                0 if abs(value[index]) < .0000005 else value[index]).rstrip('0').rstrip('.')
            edit.setText(text)
            edit.setEnabled(False)
            self.initial_text.append(text)
            self.fields.append(edit)
            grid.addWidget(edit, 1, index)
            edit.textChanged.connect(self._changed)
        layout.addLayout(grid)
        self.evidence_toggle = QtWidgets.QToolButton()
        self.evidence_toggle.setText('测量依据与说明')
        self.evidence_toggle.setCheckable(True)
        self.evidence_toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.evidence_toggle.setArrowType(QtCore.Qt.RightArrow)
        self.evidence_toggle.setStyleSheet('QToolButton { border:none; padding:4px 0; color:#2167a7; }')
        layout.addWidget(self.evidence_toggle)
        self.evidence_box = QtWidgets.QWidget()
        evidence_layout = QtWidgets.QFormLayout(self.evidence_box)
        evidence_layout.setContentsMargins(0, 0, 0, 0)
        self.source = combo([('未选择测量依据', None)])
        self.source.setEnabled(False)
        evidence_layout.addRow('依据文件', self.source)
        self.note = QtWidgets.QLineEdit(self.original.get('note', ''))
        self.note.setPlaceholderText('说明测量方法、所量原点与适用条件')
        self.note.setEnabled(False)
        evidence_layout.addRow('文字说明', self.note)
        self.import_button = button('导入测量记录…', lambda: self.importRequested.emit(self))
        self.import_button.setEnabled(False)
        evidence_layout.addRow('', self.import_button)
        self.evidence_box.hide()
        layout.addWidget(self.evidence_box)
        self.source_note = hint('')
        layout.addWidget(self.source_note)
        self.evidence_toggle.toggled.connect(self.toggle_evidence)
        self.note.textChanged.connect(self._changed)
        self.source.currentIndexChanged.connect(self._changed)
        self.status.currentIndexChanged.connect(self._status_changed)
        self.refresh_sources()
        self._loading = False

    def toggle_evidence(self, shown):
        self.evidence_box.setVisible(shown)
        self.evidence_toggle.setArrowType(QtCore.Qt.DownArrow if shown else QtCore.Qt.RightArrow)

    def refresh_sources(self, selected=None):
        if selected is None:
            selected = self.source.currentData()
            if selected is None:
                selected = next((row.get('source_id') for row in self.original.get('evidence', [])), None)
        self.source.blockSignals(True)
        self.source.clear()
        self.source.addItem('未选择测量依据', None)
        for row in self.sources:
            if (row.get('assembly_revision') == self.assembly and
                    row.get('evidence_type') in ('GEOMETRY_MEASUREMENT', 'EXTRINSIC_CALIBRATION')):
                self.source.addItem(row.get('id', '') + ' · ' + ('标定' if row['evidence_type'] == 'EXTRINSIC_CALIBRATION' else '实测'), row['id'])
        index = self.source.findData(selected)
        self.source.setCurrentIndex(max(0, index))
        self.source.blockSignals(False)
        existing = ', '.join(str(item.get('source_id', '')) for item in self.original.get('evidence', []))
        self.source_note.setText('当前状态：{}；原始依据：{}'.format(
            STATUS_NAMES.get(self.original.get('status'), '未知'), existing or '无'))

    def set_editable(self, editable):
        self._editable = editable
        self.status.setEnabled(editable)
        self._apply_enabled()

    def _apply_enabled(self):
        known = self.status.currentData() != 'UNKNOWN'
        for field in self.fields:
            field.setEnabled(self._editable and known)
        self.source.setEnabled(self._editable and known)
        self.note.setEnabled(self._editable)
        self.import_button.setEnabled(self._editable and known)

    def _status_changed(self):
        # Toggling known never fabricates zero or identity; values stay blank.
        self._apply_enabled()
        self._changed()

    def _changed(self):
        if not self._loading:
            self.changed.emit()

    def is_changed(self):
        selected = self.source.currentData()
        original_source = next((row.get('source_id') for row in self.original.get('evidence', [])), None)
        source_changed = selected is not None and selected != original_source
        return (self.status.currentData() != self.original.get('status')
                or [field.text() for field in self.fields] != self.initial_text
                or self.note.text() != self.original.get('note', '') or source_changed)

    def value(self, preview=False):
        if not self.is_changed():
            # Exact original matrix is preserved, even at a singular Euler angle.
            return copy.deepcopy(self.original)
        status = self.status.currentData()
        values = None
        if status != 'UNKNOWN':
            texts = [field.text().strip() for field in self.fields]
            if any(not text for text in texts):
                raise ValueError('请完整填写三个位置或角度；未知项保持“未确定”')
            try:
                values = [float(text) for text in texts]
            except ValueError:
                raise ValueError('位置和角度必须是有限数值')
            if not all(math.isfinite(value) for value in values):
                raise ValueError('位置和角度必须是有限数值')
        source_id = self.source.currentData()
        if not preview and status == 'CALIBRATED':
            source = next((item for item in self.sources if item.get('id') == source_id), {})
            if source.get('evidence_type') != 'EXTRINSIC_CALIBRATION':
                raise ValueError('“已标定”需要选择外参标定记录')
        result = update_component(self.original, self.component, values,
            'LEGACY_CANDIDATE' if preview and status in USABLE_STATUSES else status,
            source_id=source_id, note=self.note.text())
        # Updating a note/source must not re-encode an untouched rotation from
        # the rounded display angles (especially near Euler singularities).
        if status != 'UNKNOWN' and [field.text() for field in self.fields] == self.initial_text:
            key = 'value_m' if self.component == 'translation' else 'value_matrix'
            if self.original.get(key) is not None:
                result[key] = copy.deepcopy(self.original[key])
        return result


class InstallationDialog(PanelDialog):
    saved = QtCore.pyqtSignal()
    advancedRequested = QtCore.pyqtSignal(str)

    def __init__(self, backend, snapshot, parent=None):
        super().__init__(backend, '安装参数与 IMU', parent, (940, 840))
        self.snapshot = copy.deepcopy(snapshot)
        self.group = next((row for row in snapshot.get('groups', []) if row['id'] == 'extrinsics'), {})
        self.original = copy.deepcopy(self.group.get('value', []))
        self.permission = None
        self.pending_sources = {}
        self.editors = {}
        self.sources = copy.deepcopy(self.group.get('source_documents', []))
        self.layout.addWidget(hint('位置描述“原点在哪里”，朝向描述“坐标轴朝哪里”。每个设备只填 3 个位置和 3 个角度，其他坐标转换自动计算。'))
        self.layout.addWidget(hint('正 X 向前、正 Y 向左、正 Z 向上；角度按右手规则，R = Rz(偏航) × Ry(俯仰) × Rx(横滚)。上下位置以所选参考点为零，不是离地高度。'))
        scroll = PanelScrollArea(self)
        body = QtWidgets.QWidget()
        scroll.setWidget(body)
        content = QtWidgets.QVBoxLayout(body)
        content.setContentsMargins(2, 2, 12, 10)
        self.selector = combo([])
        self.stack = QtWidgets.QStackedWidget()
        for index, row in enumerate(self.original):
            if row.get('child_frame') not in CORE_FRAMES:
                continue
            self.selector.addItem(frame_name(row['child_frame']) + ' · 相对' + frame_name(row['parent_frame']), index)
            page = QtWidgets.QWidget()
            page_layout = QtWidgets.QVBoxLayout(page)
            page_layout.setContentsMargins(0, 0, 0, 0)
            page_layout.addWidget(hint('将 {} 的点变换到 {}；参考坐标关系不在此处重绑定。'.format(
                frame_name(row['child_frame']), frame_name(row['parent_frame']))))
            components = {}
            for component in ('translation', 'rotation'):
                editor = PrincipalComponent(row, component, self.sources, self.group.get('assembly_revision'))
                editor.changed.connect(self.update_preview)
                editor.importRequested.connect(self.import_evidence)
                components[component] = editor
                page_layout.addWidget(editor)
            self.editors[index] = components
            self.stack.addWidget(page)
        self.selector.currentIndexChanged.connect(self.stack.setCurrentIndex)
        self.selector.currentIndexChanged.connect(self._resize_stack)
        content.addWidget(self.selector)
        content.addWidget(self.stack)
        preview = QtWidgets.QGroupBox('自动计算 · 当前输入的坐标关系')
        preview_layout = QtWidgets.QVBoxLayout(preview)
        self.preview = QtWidgets.QLabel()
        self.preview.setWordWrap(True)
        self.preview.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        preview_layout.addWidget(self.preview)
        preview_layout.addWidget(hint('位置顺序 X / Y / Z（毫米），朝向顺序横滚 / 俯仰 / 偏航（度）。有输入即可预览；只有保存后通过来源校验的配置才可能用于正式任务。'))
        content.addWidget(preview)
        self.offline_note = hint(self._offline_description())
        content.addWidget(self.offline_note)
        content.addStretch()
        self.layout.addWidget(scroll, 1)
        self.state = hint('当前只读。保存前校验并备份；已录数据沿用录制时冻结的参数。')
        self.layout.addWidget(self.state)
        row = QtWidgets.QHBoxLayout()
        self.edit_button = button('开始修改', self.authorize)
        self.advanced_button = button('完整外参与依据', lambda: self.advancedRequested.emit('extrinsics'))
        self.save_button = button('保存安装参数', self.save, primary=True)
        self.save_button.setEnabled(False)
        row.addWidget(self.edit_button); row.addWidget(self.advanced_button); row.addStretch()
        row.addWidget(button('关闭', self.close)); row.addWidget(self.save_button)
        for index in range(row.count()):
            widget = row.itemAt(index).widget()
            if widget:
                widget.setProperty('popupFooter', True)
        self.layout.addLayout(row)
        self.edit_button.setEnabled(bool(self.editors))
        self.update_preview()
        self._resize_stack()

    def _resize_stack(self):
        for index in range(self.stack.count()):
            page = self.stack.widget(index)
            page.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                QtWidgets.QSizePolicy.Preferred if index == self.stack.currentIndex() else QtWidgets.QSizePolicy.Ignored)
        self.stack.adjustSize()

    def _offline_description(self):
        summary = installation_summary(self.snapshot)
        initial = summary.get('offline_initial')
        if initial:
            transforms = initial.get('initial_transforms', {})
            details = []
            for key, label in (('T_axle_lidar_left', '左雷达'), ('T_axle_lidar_right', '右雷达')):
                matrix = transforms.get(key)
                if matrix:
                    details.append(label + '相对轮轴位置 ' + _numbers([matrix[i][3]*1000 for i in range(3)], ' mm'))
            return ('离线机械初值（单独显示）：' + '；'.join(details) + '。该方案使用外壳前面中心近似数据原点、朝向采用明确的平行安装假设。'
                    '它可用于已选定的离线对照，不会把上方未知数据原点改成“已测量”。旧录包与结果仍以其冻结配置和初始化记录为准。')
        return '离线初值状态：' + summary.get('offline_error', '未提供') + '。已有录包与结果以各自冻结配置为准。'

    def proposed(self, preview=False):
        result = copy.deepcopy(self.original)
        for index, components in self.editors.items():
            for component, editor in components.items():
                result[index][component] = editor.value(preview=preview)
        return result

    def update_preview(self):
        try:
            rows = derived_transforms(self.proposed(preview=True))
            texts = []
            for row in rows:
                status = {'UNKNOWN': '缺少关系', 'DECLARED': '配置声明', 'CANDIDATE': '候选预览'}[row['status']]
                texts.append('{}：位置 {} mm · 朝向 {}° · {}'.format(row['label'],
                    _numbers(row['position_mm']), _numbers(row['orientation_deg']), status))
            self.preview.setText('\n'.join(texts))
        except (ValueError, TypeError, KeyError) as error:
            self.preview.setText('补齐输入后自动计算：' + str(error))

    def authorize(self):
        if QtWidgets.QMessageBox.warning(self, '修改安装参数',
                '错误位置或朝向会改变融合坐标和路线。请按显示的参考原点填写，并为实测/标定值选择真实依据。'
                '保存前备份原文件；已有录包和运行中的任务保留原配置。',
                QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel,
                QtWidgets.QMessageBox.Cancel) != QtWidgets.QMessageBox.Ok:
            return
        self.edit_button.setEnabled(False)
        self.async_call(lambda: self.backend.request_parameter_edit('extrinsics'), self.authorized, failed=self.failed)

    def authorized(self, permission):
        if permission.get('revision') != self.snapshot.get('revision'):
            self.failed('配置已改变，请重新打开设备参数。')
            return
        self.permission = permission
        for components in self.editors.values():
            for editor in components.values():
                editor.set_editable(True)
        self.edit_button.setEnabled(False)
        self.save_button.setEnabled(True)
        self.state.setText('填写位置和朝向，自动预览其他关系。实测/标定值需在“测量依据与说明”中绑定记录；未知项可保持未确定。')

    def failed(self, message):
        self.state.setText(str(message))
        self.edit_button.setEnabled(self.permission is None and bool(self.editors))
        self.save_button.setEnabled(self.permission is not None)

    def import_evidence(self, editor):
        if not self.permission:
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, '选择测量或标定记录')
        if not path:
            return
        note, accepted = QtWidgets.QInputDialog.getText(self, '记录说明',
            '说明原点/朝向的测量方法与适用条件：', text=editor.note.text())
        if not accepted or not note.strip():
            return
        assembly = self.group.get('assembly_revision')
        if QtWidgets.QMessageBox.question(self, '确认装配版本',
                '该记录是否对应当前装配版本 {}？'.format(assembly),
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
            return
        kind = 'EXTRINSIC_CALIBRATION' if editor.status.currentData() == 'CALIBRATED' else 'GEOMETRY_MEASUREMENT'
        editor.import_button.setEnabled(False)
        self.async_call(lambda: self.backend.prepare_extrinsic_evidence(path, kind, assembly, note, self.snapshot['revision']),
            lambda result: self.evidence_ready(editor, result, note), failed=self.failed,
            finished=lambda: editor.import_button.setEnabled(bool(self.permission)))

    def evidence_ready(self, editor, result, note):
        if result.get('revision') != self.snapshot.get('revision'):
            self.failed('配置已改变，请重新打开设备参数。'); return
        source = result['source']
        self.pending_sources[source['id']] = result['token']
        self.sources.append(copy.deepcopy(source))
        for components in self.editors.values():
            for target in components.values():
                target.sources = copy.deepcopy(self.sources)
                target.refresh_sources(source['id'] if target is editor else None)
        editor.note.setText(note)
        editor.evidence_toggle.setChecked(True)
        self.state.setText('测量记录已绑定，点击保存后才写入配置。')

    def save(self):
        if not self.permission:
            return
        try:
            value = self.proposed()
            derived_transforms(value)
        except (ValueError, TypeError, KeyError) as error:
            self.failed(str(error)); return
        if value == self.original:
            self.state.setText('没有修改任何参数。'); return
        referenced = {entry.get('source_id') for row in value for component in ('translation', 'rotation')
                      for entry in row[component].get('evidence', [])}
        tokens = [token for identifier, token in self.pending_sources.items() if identifier in referenced]
        self.save_button.setEnabled(False)
        self.async_call(lambda: self.backend.save_parameters('extrinsics', value,
            warning_token=self.permission['token'], expected_revision=self.snapshot['revision'], evidence_tokens=tokens),
            self.saved_value, failed=self.failed)

    def saved_value(self, response):
        self.saved.emit()
        QtWidgets.QMessageBox.information(self, '安装参数已保存',
            '对后续任务生效；旧录包仍使用冻结配置。\n原配置备份：' + str(response.get('backup', '')))
        self.close()
