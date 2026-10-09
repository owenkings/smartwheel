"""Small Qt building blocks. Slow filesystem work never runs in the GUI thread."""
from pathlib import Path
import weakref

from PyQt5 import QtCore, QtGui, QtWidgets, sip

from .ui_popup import ComboBox
from .ui_scroll import PanelScrollArea, WheelRouter, configure_scroll_view


STYLE = """
QWidget { font-family: 'Noto Sans CJK SC', 'Microsoft YaHei', sans-serif; font-size: 14px; color: #263444; }
QDialog, QMainWindow { background: #f4f6f8; }
QLabel#title { font-size: 25px; font-weight: 600; color: #172d45; }
QLabel#hint { color: #657487; font-size: 13px; }
QPushButton { background: #ffffff; border: 1px solid #ccd5df; border-radius: 10px; padding: 9px 17px; min-height: 20px; }
QPushButton:hover { background: #eaf3fc; border-color: #729dcc; }
QPushButton:pressed { background: #d8e8f8; }
QPushButton:disabled { color: #9aa5b0; background: #eceff3; }
QPushButton[primary=true] { background: #2167a7; color: white; border-color: #2167a7; }
QPushButton[primary=true]:disabled { background: #e3e9f0; color: #93a0ad; border-color: #d3dce5; }
QPushButton[danger=true] { color: #a53932; border-color: #d8aaa6; }
QPushButton:focus { border: 1px solid #337fb8; }
QPushButton[actionCard=true] { border-radius: 16px; padding: 0; }
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { background: white; border: 1px solid #ccd5df; border-radius: 9px; padding: 8px 11px; min-height: 22px; selection-background-color: #dceaf8; selection-color: #18334d; }
QLineEdit:hover, QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover { border-color: #91abc5; }
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus { border-color: #337fb8; }
QComboBox { padding-right: 38px; }
QComboBox::drop-down { subcontrol-origin: border; subcontrol-position: top right; width: 35px; border: none; background: transparent; }
QComboBox::down-arrow { image: none; width: 0px; height: 0px; }
QComboBox QAbstractItemView { background: white; border: 1px solid #ccd5df; border-radius: 9px; padding: 4px; outline: none; selection-background-color: #e6f0fa; selection-color: #183b61; }
QComboBox QAbstractItemView::item { padding: 8px 10px; border-radius: 6px; min-height: 24px; }
QLineEdit:disabled, QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled { background: #edf1f5; color: #929eab; border-color: #dce2e9; }
QSpinBox::up-button, QDoubleSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 28px; border: none; background: transparent; }
QSpinBox::down-button, QDoubleSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 28px; border: none; background: transparent; }
QLineEdit:read-only { background: #eff2f6; }
QGroupBox { background: white; border: 1px solid #d9e0e8; border-radius: 10px; margin-top: 13px; padding: 14px; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
QTreeWidget, QTableWidget, QListWidget, QPlainTextEdit { background: white; border: 1px solid #d9e0e8; border-radius: 9px; selection-background-color: #dceaf8; selection-color: #18334d; }
QHeaderView::section { background: #eaf0f6; padding: 7px; border: 0; border-bottom: 1px solid #d8e0e8; }
QTabBar::tab { padding: 9px 15px; }
QProgressBar { border: 1px solid #cbd5df; border-radius: 4px; text-align: center; min-height: 20px; }
QProgressBar::chunk { background: #337fb8; }
QCheckBox, QRadioButton { padding: 4px 0; }
QCheckBox::indicator, QRadioButton::indicator { width: 18px; height: 18px; }
QCheckBox:disabled, QRadioButton:disabled { color: #949da9; }
QTreeWidget::item:disabled { color: #929eab; }
QScrollBar:vertical { background: transparent; width: 10px; margin: 3px 1px; }
QScrollBar::handle:vertical { background: #bdcad7; border-radius: 4px; min-height: 28px; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 1px 3px; }
QScrollBar::handle:horizontal { background: #bdcad7; border-radius: 4px; min-width: 28px; }
QScrollBar::handle:hover { background: #8ea6bd; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; border: none; }
QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }
"""


class PanelStyle(QtWidgets.QProxyStyle):
    """Use crisp vector indicators at the current display scale."""
    def drawPrimitive(self, element, option, painter, widget=None):
        arrows = {self.PE_IndicatorArrowDown: 0, self.PE_IndicatorArrowUp: 180,
                  self.PE_IndicatorArrowRight: -90, self.PE_IndicatorArrowLeft: 90}
        indicators = (self.PE_IndicatorCheckBox, self.PE_IndicatorItemViewItemCheck, self.PE_IndicatorRadioButton)
        if element not in arrows and element not in indicators:
            return super().drawPrimitive(element, option, painter, widget)
        painter.save()
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        enabled = bool(option.state & self.State_Enabled)
        if element in arrows:
            painter.translate(QtCore.QRectF(option.rect).center())
            painter.rotate(arrows[element])
            painter.setPen(QtGui.QPen(QtGui.QColor("#657e96" if enabled else "#afbac5"), 1.7,
                                      QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
            painter.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(-4, -2), QtCore.QPointF(0, 2), QtCore.QPointF(4, -2)]))
        else:
            checked = bool(option.state & (self.State_On | self.State_NoChange))
            rect = QtCore.QRectF(option.rect).adjusted(1, 1, -1, -1)
            color = "#2167a7" if enabled and checked else "#bcc7d2" if checked else "#b3c1cf"
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 1.3))
            painter.setBrush(QtGui.QColor(color if checked else "#ffffff" if enabled else "#edf1f5"))
            if element == self.PE_IndicatorRadioButton:
                painter.drawEllipse(rect)
            else:
                painter.drawRoundedRect(rect, 4, 4)
            if checked:
                painter.setPen(QtGui.QPen(QtGui.QColor("white"), 1.8, QtCore.Qt.SolidLine,
                                          QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
                cx, cy = rect.center().x(), rect.center().y()
                if element == self.PE_IndicatorRadioButton:
                    painter.setBrush(QtGui.QColor("white"))
                    painter.drawEllipse(QtCore.QPointF(cx, cy), 2.5, 2.5)
                elif option.state & self.State_NoChange:
                    painter.drawLine(QtCore.QPointF(cx-4, cy), QtCore.QPointF(cx+4, cy))
                else:
                    painter.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(cx-4, cy), QtCore.QPointF(cx-1, cy+3), QtCore.QPointF(cx+4, cy-3)]))
        painter.restore()


def apply_theme(application):
    """Include a readable font when an offscreen platform has no font database."""
    database = QtGui.QFontDatabase()
    families = database.families()
    if not families:
        for path in ("C:/Windows/Fonts/msyh.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                     "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
            if Path(path).is_file():
                font_id = database.addApplicationFont(path)
                names = database.applicationFontFamilies(font_id)
                if names:
                    application.setFont(QtGui.QFont(names[0], 11))
                    break
    application.setStyle(PanelStyle("Fusion"))
    application.setStyleSheet(STYLE)
    if not hasattr(application, "_panel_wheel_router"):
        application._panel_wheel_router = WheelRouter(application)
        application.installEventFilter(application._panel_wheel_router)


def human_bytes(value):
    value = int(value or 0)
    for suffix, divisor in (("TiB", 1024**4), ("GiB", 1024**3), ("MiB", 1024**2), ("KiB", 1024)):
        if value >= divisor:
            return "{:.2f} {}".format(value / divisor, suffix)
    return "{} B".format(value)


def result_label(result):
    name = str(result.get("name") or Path(result["path"]).name)
    identifier = result.get("group_id")
    if identifier:
        marker = " / {} · ".format(identifier)
        title = result.get("title") or (name.split(marker, 1)[1] if marker in name else result.get("cell") or name)
        return "{} · {}".format(identifier, title)
    return name


def hint(text):
    widget = QtWidgets.QLabel(text)
    widget.setObjectName("hint")
    widget.setWordWrap(True)
    widget.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
    return widget


def heading(text):
    widget = QtWidgets.QLabel(text)
    widget.setObjectName("title")
    return widget


def button(text, callback=None, primary=False, danger=False):
    widget = QtWidgets.QPushButton(text)
    widget.setProperty("primary", primary)
    widget.setProperty("danger", danger)
    if callback is not None:
        widget.clicked.connect(callback)
    return widget


def combo(items):
    widget = ComboBox()
    for label, value in items:
        widget.addItem(label, value)
    return widget


class PathPicker(QtWidgets.QWidget):
    changed = QtCore.pyqtSignal(str)

    def __init__(self, value="", parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.edit = QtWidgets.QLineEdit(str(value))
        self.edit.setClearButtonEnabled(True)
        self.browse = button("选择…", self.choose)
        layout.addWidget(self.edit, 1)
        layout.addWidget(self.browse)
        self.edit.editingFinished.connect(lambda: self.changed.emit(self.path()))

    def path(self):
        return self.edit.text().strip()

    def set_path(self, value):
        self.edit.setText(str(value))

    def choose(self):
        selected = QtWidgets.QFileDialog.getExistingDirectory(self, "选择文件夹", self.path())
        if selected:
            self.set_path(selected)
            self.changed.emit(selected)


class FilePicker(PathPicker):
    def choose(self):
        selected, _ = QtWidgets.QFileDialog.getOpenFileName(self, "选择校正文件", self.path(), "JSON 文件 (*.json);;所有文件 (*)")
        if selected:
            self.set_path(selected)
            self.changed.emit(selected)


class _Worker(QtCore.QObject):
    result = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, function):
        super().__init__()
        self.function = function

    @QtCore.pyqtSlot()
    def run(self):
        try:
            self.result.emit(self.function())
        except Exception as exc:
            self.failed.emit("{}: {}".format(type(exc).__name__, exc))
        finally:
            self.finished.emit()


class BackgroundTask(QtCore.QObject):
    """Application-owned thread, guarded callbacks if a window closes mid-read."""
    live_tasks = set()

    def __init__(self, owner, function, done, failed=None, finished=None):
        super().__init__(QtWidgets.QApplication.instance())
        self.owner = weakref.ref(owner)
        self.on_done, self.on_failed, self.on_finished = done, failed, finished
        self.thread = QtCore.QThread()
        self.worker = _Worker(function)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.result.connect(self.deliver)
        self.worker.failed.connect(self.failure)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.cleanup)
        self.live_tasks.add(self)
        self.thread.start()

    def alive(self):
        owner = self.owner()
        return owner is not None and not sip.isdeleted(owner)

    @QtCore.pyqtSlot(object)
    def deliver(self, value):
        if self.alive() and self.on_done is not None:
            self.on_done(value)

    @QtCore.pyqtSlot(str)
    def failure(self, message):
        if not self.alive():
            return
        if self.on_failed:
            self.on_failed(message)
        else:
            QtWidgets.QMessageBox.warning(self.owner(), "操作未完成", message)

    @QtCore.pyqtSlot()
    def cleanup(self):
        if self.alive() and self.on_finished:
            self.on_finished()
        self.thread.deleteLater()
        self.live_tasks.discard(self)
        self.on_done = self.on_failed = self.on_finished = None
        self.deleteLater()


class PanelDialog(QtWidgets.QDialog):
    def __init__(self, backend, title, parent=None, size=(780, 620)):
        super().__init__(parent)
        self.backend = backend
        self.setWindowTitle(title)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.resize(*size)
        self.setWindowModality(QtCore.Qt.NonModal)
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(25, 22, 25, 22)
        self.layout.setSpacing(14)
        self.layout.addWidget(heading(title))

    def async_call(self, function, done, failed=None, finished=None):
        return BackgroundTask(self, function, done, failed, finished)

    def show_error(self, message):
        QtWidgets.QMessageBox.warning(self, "操作未完成", str(message))


STATUS_LABELS = {
    "QUEUED": "等待中", "RUNNING": "运行中", "STOPPING": "正在收尾",
    "COMPLETE": "已完成", "FAILED": "失败", "CANCELLED": "已取消", "PENDING": "待保存处理",
    "INTERRUPTED": "已中断，待核查", "SKIPPED": "未执行",
}
TERMINAL = {"COMPLETE", "FAILED", "CANCELLED", "INTERRUPTED", "SKIPPED", "PENDING"}


def status_text(value):
    return STATUS_LABELS.get(value, str(value or "未知"))


def set_readonly_table(table):
    configure_scroll_view(table)
    table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
    table.setAlternatingRowColors(True)
    table.verticalHeader().hide()
    table.horizontalHeader().setStretchLastSection(True)


def local_open(path):
    QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(Path(path))))
