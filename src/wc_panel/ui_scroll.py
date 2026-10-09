"""Consistent scroll surfaces and wheel routing for panel forms."""
from PyQt5 import QtCore, QtGui, QtWidgets


class SlimScrollBar(QtWidgets.QScrollBar):
    """A quiet thumb with a larger native drag target and no arrow buttons."""
    def __init__(self, orientation=QtCore.Qt.Vertical, parent=None):
        super().__init__(orientation, parent)
        self.setMouseTracking(True)
        self._hover = False
        self.setStyleSheet("""
            QScrollBar:vertical { width: 14px; margin: 3px 0; background: transparent; }
            QScrollBar:horizontal { height: 14px; margin: 0 3px; background: transparent; }
            QScrollBar::handle:vertical { min-height: 32px; }
            QScrollBar::handle:horizontal { min-width: 32px; }
            QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
            QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }
        """)
        self.sliderPressed.connect(self.update)
        self.sliderReleased.connect(self.update)

    def enterEvent(self, event):
        self._hover = True
        self.update()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self._hover = False
        self.update()
        super().leaveEvent(event)

    def paintEvent(self, event):
        if self.maximum() <= self.minimum():
            return
        option = QtWidgets.QStyleOptionSlider()
        self.initStyleOption(option)
        thumb = QtCore.QRectF(self.style().subControlRect(
            QtWidgets.QStyle.CC_ScrollBar, option, QtWidgets.QStyle.SC_ScrollBarSlider, self))
        active = self._hover or self.isSliderDown()
        thickness = 8 if active else 5
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtCore.Qt.NoPen)
        if self.orientation() == QtCore.Qt.Vertical:
            thumb.setLeft((self.width()-thickness)/2)
            thumb.setWidth(thickness)
            track = QtCore.QRectF((self.width()-8)/2, 3, 8, self.height()-6)
        else:
            thumb.setTop((self.height()-thickness)/2)
            thumb.setHeight(thickness)
            track = QtCore.QRectF(3, (self.height()-8)/2, self.width()-6, 8)
        if active:
            painter.setBrush(QtGui.QColor("#e9eef4"))
            painter.drawRoundedRect(track, 4, 4)
        painter.setBrush(QtGui.QColor("#6286a8" if self.isSliderDown() else "#90a9c0" if active else "#c0cdd9"))
        painter.drawRoundedRect(thumb, thickness/2, thickness/2)


def configure_scroll_view(view):
    """Keep standard keyboard/drag behavior, and scroll data views by pixels."""
    for orientation, getter, setter in (
            (QtCore.Qt.Vertical, view.verticalScrollBar, view.setVerticalScrollBar),
            (QtCore.Qt.Horizontal, view.horizontalScrollBar, view.setHorizontalScrollBar)):
        if not isinstance(getter(), SlimScrollBar):
            setter(SlimScrollBar(orientation, view))
    if isinstance(view, QtWidgets.QAbstractItemView):
        view.setVerticalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        view.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
    view.verticalScrollBar().setSingleStep(24)
    view.horizontalScrollBar().setSingleStep(24)
    return view


def _outer_scroll(widget):
    """Never route a popup's wheel into the form underneath that popup."""
    if widget.isWindow():
        return None
    parent = widget.parentWidget()
    while parent is not None:
        if isinstance(parent, QtWidgets.QAbstractScrollArea):
            return parent
        if parent.isWindow():
            break
        parent = parent.parentWidget()
    return None


def forward_wheel(widget, event):
    outer = _outer_scroll(widget)
    if outer is None:
        return False
    viewport = outer.viewport()
    local = viewport.mapFromGlobal(event.globalPos())
    forwarded = QtGui.QWheelEvent(QtCore.QPointF(local), event.globalPosF(),
                                  event.pixelDelta(), event.angleDelta(), event.buttons(),
                                  event.modifiers(), event.phase(), event.inverted(), event.source())
    forwarded.setAccepted(False)
    QtWidgets.QApplication.sendEvent(viewport, forwarded)
    return forwarded.isAccepted()


class WheelRouter(QtCore.QObject):
    """At a nested view's edge, let its parent page consume the wheel."""
    def eventFilter(self, widget, event):
        if event.type() == QtCore.QEvent.Show and isinstance(widget, QtWidgets.QAbstractScrollArea):
            configure_scroll_view(widget)
        if event.type() != QtCore.QEvent.Wheel or not isinstance(widget, QtWidgets.QWidget):
            return False
        # Image and 3D canvases own the wheel even when their scrollbars are at
        # an edge: they use it to zoom, not to scroll their containing page.
        if widget.property("consumesWheel") or (widget.parentWidget() is not None and
                                                widget.parentWidget().property("consumesWheel")):
            return False
        field = widget
        while field is not None:
            if isinstance(field, (QtWidgets.QComboBox, QtWidgets.QAbstractSpinBox)):
                forward_wheel(field, event)
                event.accept()
                return True
            if field.isWindow() or isinstance(field, QtWidgets.QAbstractScrollArea):
                break
            field = field.parentWidget()
        parent = widget.parentWidget()
        if not isinstance(parent, QtWidgets.QAbstractScrollArea) or widget is not parent.viewport():
            return False
        horizontal = bool(event.angleDelta().x() or event.pixelDelta().x())
        delta = (event.pixelDelta().x() or event.angleDelta().x()) if horizontal else (event.pixelDelta().y() or event.angleDelta().y())
        bar = parent.horizontalScrollBar() if horizontal else parent.verticalScrollBar()
        policy = parent.horizontalScrollBarPolicy() if horizontal else parent.verticalScrollBarPolicy()
        at_edge = (bar.maximum() <= bar.minimum() or policy == QtCore.Qt.ScrollBarAlwaysOff or
                   (delta > 0 and bar.value() <= bar.minimum()) or
                   (delta < 0 and bar.value() >= bar.maximum()))
        if delta and at_edge and forward_wheel(parent, event):
            event.accept()
            return True
        return False


class PanelScrollArea(QtWidgets.QScrollArea):
    """One primary, pixel-based scroll surface for a long settings page."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setStyleSheet("QScrollArea { background: transparent; border: none; }")
        self.viewport().setAutoFillBackground(False)
        configure_scroll_view(self)

    def setWidget(self, widget):
        super().setWidget(widget)
        widget.setAutoFillBackground(False)

    def wheelEvent(self, event):
        pixel, angle = event.pixelDelta(), event.angleDelta()
        horizontal = bool(pixel.x() or angle.x())
        bar = self.horizontalScrollBar() if horizontal else self.verticalScrollBar()
        delta = pixel.x() if horizontal else pixel.y()
        if not delta:
            delta = (angle.x() if horizontal else angle.y()) * 0.6
        old = bar.value()
        bar.setValue(old-round(delta))
        if bar.value() != old:
            event.accept()
        else:
            event.ignore()
