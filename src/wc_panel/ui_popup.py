"""A field-aligned choice menu with no platform-private combo popup chrome."""
from PyQt5 import QtCore, QtGui, QtWidgets

from .ui_scroll import SlimScrollBar, forward_wheel


_ROW_HEIGHT = 44
_SHADOW = 6
_INSET = 12
_CHROME = _INSET * 2


class _ChoiceDelegate(QtWidgets.QStyledItemDelegate):
    def __init__(self, combo, parent):
        super().__init__(parent)
        self.combo = combo

    def sizeHint(self, option, index):
        return QtCore.QSize(0, _ROW_HEIGHT)

    def paint(self, painter, option, index):
        painter.save()
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        enabled = bool(index.flags() & QtCore.Qt.ItemIsEnabled)
        selected = index.row() == self.combo.currentIndex()
        hovered = bool(option.state & QtWidgets.QStyle.State_Selected)
        rect = QtCore.QRectF(option.rect).adjusted(0, 2, 0, -2)
        painter.setPen(QtCore.Qt.NoPen)
        if enabled and (selected or hovered):
            painter.setBrush(QtGui.QColor("#e9f2fb" if selected else "#f0f5fa"))
            painter.drawRoundedRect(rect, 7, 7)
        text_rect = option.rect.adjusted(12, 0, -34, 0)
        decoration = index.data(QtCore.Qt.DecorationRole)
        if isinstance(decoration, QtGui.QIcon) and not decoration.isNull():
            decoration.paint(painter, QtCore.QRect(text_rect.left(), text_rect.center().y()-8, 16, 16))
            text_rect.adjust(24, 0, 0, 0)
        painter.setFont(option.font)
        painter.setPen(QtGui.QColor("#1f4e7a" if enabled and selected else "#31475b" if enabled else "#9aa7b4"))
        text = str(index.data(QtCore.Qt.DisplayRole) or "")
        text = option.fontMetrics.elidedText(text, QtCore.Qt.ElideMiddle, max(0, text_rect.width()))
        painter.drawText(text_rect, QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, text)
        if selected:
            x, y = option.rect.right()-17, option.rect.center().y()
            painter.setPen(QtGui.QPen(QtGui.QColor("#2874b8" if enabled else "#a9b5c1"), 1.8,
                                      QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
            painter.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(x-4, y), QtCore.QPointF(x-1, y+3),
                                                 QtCore.QPointF(x+5, y-4)]))
        painter.restore()

    def helpEvent(self, event, view, option, index):
        if event.type() == QtCore.QEvent.ToolTip and index.isValid():
            text = index.data(QtCore.Qt.ToolTipRole) or index.data(QtCore.Qt.DisplayRole)
            if text:
                QtWidgets.QToolTip.showText(event.globalPos(), str(text), view)
                return True
        return super().helpEvent(event, view, option, index)


class _ChoiceList(QtWidgets.QListView):
    def __init__(self, combo, parent):
        super().__init__(parent)
        self.combo = combo
        self.setObjectName("panelChoiceList")
        self.setAccessibleName("选择一项")
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setUniformItemSizes(True)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        self.setVerticalScrollBar(SlimScrollBar(QtCore.Qt.Vertical, self))
        self.verticalScrollBar().setSingleStep(24)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.setMouseTracking(True)
        self.setTextElideMode(QtCore.Qt.ElideMiddle)
        self.setItemDelegate(_ChoiceDelegate(combo, self))
        self.setStyleSheet("""
            QListView#panelChoiceList { background: transparent; border: none; border-radius: 0;
                padding: 0; margin: 0; outline: none; }
            QListView#panelChoiceList::item { padding: 0; margin: 0; border: none; }
        """)
        self.viewport().setAutoFillBackground(False)
        self.clicked.connect(combo._commit)
        self.entered.connect(combo._highlight)

    def keyPressEvent(self, event):
        if not self.combo._popup_key(event):
            super().keyPressEvent(event)


class _ChoicePopup(QtWidgets.QFrame):
    def __init__(self, combo):
        super().__init__(combo, QtCore.Qt.Popup | QtCore.Qt.FramelessWindowHint | QtCore.Qt.NoDropShadowWindowHint)
        self.combo = combo
        self.setObjectName("panelChoicePopup")
        self.setProperty("panelChoicePopup", True)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground)
        self.setAttribute(QtCore.Qt.WA_NoMouseReplay)
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setAutoFillBackground(False)
        self.setStyleSheet("QFrame#panelChoicePopup { background: transparent; border: none; }")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(_INSET, _INSET, _INSET, _INSET)
        self.list = _ChoiceList(combo, self)
        layout.addWidget(self.list)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        surface = QtCore.QRectF(self.rect()).adjusted(_SHADOW, _SHADOW-2, -_SHADOW, -_SHADOW-2)
        painter.setPen(QtCore.Qt.NoPen)
        # Paint one surface; the transparent outer pixels remain truly rounded
        # even on Linux styles which give native combo menus a square frame.
        for spread in range(5, 0, -1):
            painter.setBrush(QtGui.QColor(24, 47, 70, 3 + (5-spread)))
            painter.drawRoundedRect(surface.adjusted(-spread, -spread+2, spread, spread+2),
                                    10+spread, 10+spread)
        painter.setBrush(QtGui.QColor("#ffffff"))
        painter.setPen(QtGui.QPen(QtGui.QColor("#d3dde7"), 1))
        painter.drawRoundedRect(surface, 10, 10)

    def hideEvent(self, event):
        self.combo._popup_closed()
        super().hideEvent(event)

    def mousePressEvent(self, event):
        if not self.rect().contains(event.pos()):
            self.combo.hidePopup()
            event.accept()
            return
        super().mousePressEvent(event)

    def keyPressEvent(self, event):
        if not self.combo._popup_key(event):
            super().keyPressEvent(event)


class ComboBox(QtWidgets.QComboBox):
    """QComboBox data/signals with a bounded, independently painted menu.

    Selection is staged until click or Enter; dismissing the menu never changes
    the configured value. Mark a dialog footer ``popupFooter=True`` to reserve
    its action area. ``popupAbove=True`` prefers opening above the field.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizeAdjustPolicy(self.AdjustToMinimumContentsLengthWithIcon)
        self.setMinimumContentsLength(12)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.setMinimumHeight(42)
        self.setMaxVisibleItems(6)
        self.popup_widget = _ChoicePopup(self)
        self._watched_ancestors = []
        self._search_text = ""
        self._search_clock = QtCore.QElapsedTimer()
        self.currentTextChanged.connect(self.setToolTip)

    def view(self):
        return self.popup_widget.list

    def setModel(self, model):
        # Async catalog refreshes may replace and delete the former model.
        # Dismiss before QComboBox releases it so no staged row can survive.
        if hasattr(self, "popup_widget"):
            self.hidePopup()
        super().setModel(model)

    def clear(self):
        if hasattr(self, "popup_widget"):
            self.hidePopup()
        super().clear()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtGui.QPen(QtGui.QColor("#55738e" if self.isEnabled() else "#aebbc7"), 1.7,
                                  QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
        x = 18 if self.layoutDirection() == QtCore.Qt.RightToLeft else self.width()-18
        y = self.height()/2
        direction = -1 if self.popup_widget.isVisible() else 1
        painter.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(x-4, y-2*direction),
                                             QtCore.QPointF(x, y+2*direction),
                                             QtCore.QPointF(x+4, y-2*direction)]))

    def wheelEvent(self, event):
        # A wheel over a form field scrolls the form, never changes a parameter.
        event.accept() if forward_wheel(self, event) else event.ignore()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self.isEnabled():
            self.setFocus(QtCore.Qt.MouseFocusReason)
            self.hidePopup() if self.popup_widget.isVisible() else self.showPopup()
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def keyPressEvent(self, event):
        if self.popup_widget.isVisible() and self._popup_key(event):
            return
        if event.key() in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter, QtCore.Qt.Key_F4) or (
                event.key() == QtCore.Qt.Key_Down and event.modifiers() & QtCore.Qt.AltModifier):
            self.showPopup()
            event.accept()
            return
        super().keyPressEvent(event)

    def _bounds(self):
        screen = self.screen().availableGeometry()
        window = self.window()
        content = QtCore.QRect(window.mapToGlobal(QtCore.QPoint(12, 12)), window.size()-QtCore.QSize(24, 24))
        bounds = content.intersected(screen)
        for widget in window.findChildren(QtWidgets.QWidget):
            if widget.property("popupFooter") and widget.isVisibleTo(window):
                bounds.setBottom(min(bounds.bottom(), widget.mapToGlobal(QtCore.QPoint()).y()-5))
        explicit_bottom = self.property("popupBottomBoundary")
        if isinstance(explicit_bottom, int):
            bounds.setBottom(min(bounds.bottom(), explicit_bottom))
        return bounds

    def showPopup(self):
        if not self.count() or not self.isEnabled() or self.popup_widget.isVisible():
            return
        view = self.view()
        view.setModel(self.model())
        view.setModelColumn(self.modelColumn())
        view.setRootIndex(self.rootModelIndex())
        bounds = self._bounds()
        top = self.mapToGlobal(QtCore.QPoint(0, -2)).y()
        bottom = self.mapToGlobal(QtCore.QPoint(0, self.height()+2)).y()
        above, below = max(0, top-bounds.top()), max(0, bounds.bottom()-bottom+1)
        wanted = _ROW_HEIGHT * min(self.count(), max(1, min(6, self.maxVisibleItems()))) + _CHROME
        prefer_above = bool(self.property("popupAbove"))
        use_above = above >= wanted if prefer_above else below < wanted and above > below
        if prefer_above and above < wanted:
            use_above = above >= below
        available = above if use_above else below
        if available < _CHROME + 24:
            # A field without room for even one row must first be scrolled into
            # view; do not cover a protected action footer to force a popup.
            return
        rows = max(1, min(6, self.maxVisibleItems(), self.count(), (available-_CHROME)//_ROW_HEIGHT))
        height = min(available, rows*_ROW_HEIGHT+_CHROME)
        width = min(self.width()+2*_SHADOW, bounds.width())
        x = self.mapToGlobal(QtCore.QPoint(-_SHADOW, 0)).x()
        x = max(bounds.left(), min(x, bounds.right()-width+1))
        y = top-height if use_above else bottom
        self.popup_widget.setGeometry(x, y, width, height)
        self.popup_widget.setAccessibleName(self.accessibleName() or "选项")
        index = self.model().index(self.currentIndex(), self.modelColumn(), self.rootModelIndex())
        if not self._enabled(index):
            index = self._next_index(-1, 1)
        view.setCurrentIndex(index)
        self._search_text = ""
        self.popup_widget.show()
        view.doItemsLayout()
        view.scrollTo(index, QtWidgets.QAbstractItemView.PositionAtCenter)
        view.setFocus(QtCore.Qt.PopupFocusReason)
        self._watched_ancestors = []
        parent = self.parentWidget()
        while parent is not None:
            self._watched_ancestors.append(parent)
            parent = parent.parentWidget()
        QtWidgets.QApplication.instance().installEventFilter(self)
        self.update()

    def hidePopup(self):
        self.popup_widget.hide()

    def _popup_closed(self):
        QtWidgets.QApplication.instance().removeEventFilter(self)
        self._watched_ancestors = []
        self.update()

    def eventFilter(self, watched, event):
        if self.popup_widget.isVisible():
            kind = event.type()
            if (watched in self._watched_ancestors or watched is self) and kind in (
                    QtCore.QEvent.Move, QtCore.QEvent.Resize, QtCore.QEvent.Hide, QtCore.QEvent.Close):
                self.hidePopup()
            elif kind == QtCore.QEvent.MouseButtonPress and isinstance(watched, QtWidgets.QWidget):
                if watched is not self and watched is not self.popup_widget and not self.popup_widget.isAncestorOf(watched):
                    self.hidePopup()
                    event.accept()
                    return True
        return super().eventFilter(watched, event)

    @staticmethod
    def _enabled(index):
        return index.isValid() and bool(index.flags() & QtCore.Qt.ItemIsEnabled) and bool(index.flags() & QtCore.Qt.ItemIsSelectable)

    def _next_index(self, row, step):
        for candidate in range(row+step, self.count() if step > 0 else -1, step):
            index = self.model().index(candidate, self.modelColumn(), self.rootModelIndex())
            if self._enabled(index):
                return index
        return QtCore.QModelIndex()

    def _highlight(self, index):
        if self._enabled(index):
            self.view().setCurrentIndex(index)
            self.highlighted[int].emit(index.row())
            self.highlighted[str].emit(str(index.data(QtCore.Qt.DisplayRole) or ""))

    def _commit(self, index):
        if not self._enabled(index):
            return
        row = index.row()
        self.setCurrentIndex(row)
        text = self.currentText()
        self.hidePopup()
        self.setFocus(QtCore.Qt.PopupFocusReason)
        self.activated[int].emit(row)
        self.activated[str].emit(text)
        if hasattr(self, "textActivated"):
            self.textActivated.emit(text)

    def _popup_key(self, event):
        if not self.count():
            self.hidePopup()
            event.accept()
            return True
        key = event.key()
        view = self.view()
        if key in (QtCore.Qt.Key_Escape, QtCore.Qt.Key_F4):
            self.hidePopup()
            self.setFocus(QtCore.Qt.PopupFocusReason)
        elif key in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter, QtCore.Qt.Key_Space):
            self._commit(view.currentIndex())
        elif key in (QtCore.Qt.Key_Tab, QtCore.Qt.Key_Backtab):
            self.hidePopup()
            self.setFocus(QtCore.Qt.TabFocusReason)
            self.focusNextPrevChild(key == QtCore.Qt.Key_Tab)
        elif key in (QtCore.Qt.Key_Down, QtCore.Qt.Key_Up, QtCore.Qt.Key_Home, QtCore.Qt.Key_End,
                     QtCore.Qt.Key_PageDown, QtCore.Qt.Key_PageUp):
            row = view.currentIndex().row()
            if key == QtCore.Qt.Key_Home:
                row = -1
            elif key == QtCore.Qt.Key_End:
                row = self.count()
            step = -1 if key in (QtCore.Qt.Key_Up, QtCore.Qt.Key_End, QtCore.Qt.Key_PageUp) else 1
            count = max(1, view.viewport().height()//_ROW_HEIGHT) if key in (QtCore.Qt.Key_PageDown, QtCore.Qt.Key_PageUp) else 1
            for _ in range(count):
                index = self._next_index(row, step)
                if not index.isValid():
                    break
                row = index.row()
                self._highlight(index)
            view.scrollTo(view.currentIndex(), QtWidgets.QAbstractItemView.EnsureVisible)
        elif event.text() and not event.modifiers() & (QtCore.Qt.ControlModifier | QtCore.Qt.AltModifier | QtCore.Qt.MetaModifier):
            text = event.text().casefold()
            if not self._search_clock.isValid() or self._search_clock.elapsed() > 900:
                self._search_text = ""
            self._search_text += text
            self._search_clock.start()
            start = view.currentIndex().row()
            # Repeated letters cycle through matching entries as in a native
            # choice field; a typed prefix keeps refining the current match.
            prefix = self._search_text
            cycling = len(set(prefix)) == 1
            if cycling:
                prefix = text
            for offset in range(1 if cycling else 0, self.count()+(1 if cycling else 0)):
                row = (start+offset) % self.count()
                index = self.model().index(row, self.modelColumn(), self.rootModelIndex())
                if self._enabled(index) and str(index.data(QtCore.Qt.DisplayRole) or "").casefold().startswith(prefix):
                    self._highlight(index)
                    view.scrollTo(index, QtWidgets.QAbstractItemView.EnsureVisible)
                    break
        else:
            return False
        event.accept()
        return True
