"""User scrolling behavior in small panel windows; no device or data writes."""
import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtCore, QtGui, QtTest, QtWidgets, sip

from test_panel_ui import FixtureBackend, drain
from wc_panel.ui_common import PanelScrollArea, apply_theme
from wc_panel.ui_dialogs import LiveDialog, OfflineDialog


class ScrollControlsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        apply_theme(cls.app)

    def setUp(self):
        self.windows = []

    def tearDown(self):
        drain(self.app)
        for window in self.windows:
            if sip.isdeleted(window):
                continue
            for timer in window.findChildren(QtCore.QTimer):
                timer.stop()
            window.close()
        self.app.processEvents()

    def offline(self, backend=None):
        dialog = OfflineDialog(backend or FixtureBackend())
        self.windows.append(dialog)
        dialog.show()
        drain(self.app, lambda: dialog.variant_table.topLevelItemCount() > 0)
        dialog.resize(960, 680)
        QtTest.QTest.qWait(30)
        return dialog

    def wheel(self, widget, delta=-120, point=None):
        point = point or widget.rect().center()
        event = QtGui.QWheelEvent(QtCore.QPointF(point), QtCore.QPointF(widget.mapToGlobal(point)),
                                  QtCore.QPoint(), QtCore.QPoint(0, delta),
                                  QtCore.Qt.NoButton, QtCore.Qt.NoModifier,
                                  QtCore.Qt.NoScrollPhase, False)
        self.app.sendEvent(widget, event)
        self.app.processEvents()

    def test_wheel_over_focused_closed_selector_scrolls_page_without_changing_value(self):
        dialog = self.offline()
        dialog.dataset.addItem("另一份录制", "/fixture/another")
        dialog.dataset.setFocus()
        selected = dialog.dataset.currentData()
        bar = dialog.setup_scroll.verticalScrollBar()
        bar.setValue(0)
        self.wheel(dialog.dataset)
        self.assertGreater(bar.value(), 0)
        self.assertEqual(dialog.dataset.currentData(), selected)

    def test_wheel_over_focused_spin_editor_scrolls_page_without_changing_value(self):
        area = PanelScrollArea()
        self.windows.append(area)
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        spin = QtWidgets.QDoubleSpinBox()
        spin.setValue(4.5)
        layout.addWidget(spin)
        filler = QtWidgets.QWidget()
        filler.setMinimumHeight(1200)
        layout.addWidget(filler)
        area.setWidget(content)
        area.resize(400, 260)
        area.show()
        spin.setFocus()
        QtTest.QTest.qWait(20)
        self.wheel(spin.lineEdit())
        self.assertEqual(spin.value(), 4.5)
        self.assertGreater(area.verticalScrollBar().value(), 0)

    def test_wheel_over_wrapped_recording_name_does_not_trap_page_scroll(self):
        dialog = self.offline()
        bar = dialog.setup_scroll.verticalScrollBar()
        bar.setValue(0)
        self.wheel(dialog.dataset_detail.viewport())
        self.assertGreater(bar.value(), 0)

    def test_scheme_rows_share_the_page_scroll_and_last_row_is_reachable(self):
        dialog = self.offline()
        area, table = dialog.setup_scroll, dialog.variant_table
        bar = area.verticalScrollBar()
        bar.setValue(table.mapTo(area.widget(), QtCore.QPoint()).y())
        before = bar.value()
        for _ in range(30):
            visible_point = table.viewport().mapFromGlobal(area.viewport().mapToGlobal(area.viewport().rect().center()))
            self.wheel(table.viewport(), point=visible_point)
        self.assertGreater(bar.value(), before)
        self.assertEqual(bar.value(), bar.maximum())
        self.assertEqual(table.verticalScrollBar().value(), 0)
        last = table.visualItemRect(table.topLevelItem(table.topLevelItemCount()-1))
        last_center = area.viewport().mapFromGlobal(table.viewport().mapToGlobal(last.center()))
        self.assertTrue(area.viewport().rect().contains(last_center))
        self.assertFalse(table.verticalScrollBar().isVisible())

    def test_dragging_scroll_thumb_moves_the_page(self):
        dialog = self.offline()
        area = dialog.setup_scroll
        bar = area.verticalScrollBar()
        bar.setValue(0)
        initial_y = area.widget().pos().y()
        option = QtWidgets.QStyleOptionSlider()
        bar.initStyleOption(option)
        thumb = bar.style().subControlRect(QtWidgets.QStyle.CC_ScrollBar, option,
                                           QtWidgets.QStyle.SC_ScrollBarSlider, bar)
        start = thumb.center()
        end = QtCore.QPoint(start.x(), min(bar.height()-12, start.y()+100))
        QtTest.QTest.mousePress(bar, QtCore.Qt.LeftButton, pos=start)
        move = QtGui.QMouseEvent(QtCore.QEvent.MouseMove, QtCore.QPointF(end),
                                 QtCore.QPointF(bar.mapToGlobal(end)), QtCore.Qt.NoButton,
                                 QtCore.Qt.LeftButton, QtCore.Qt.NoModifier)
        self.app.sendEvent(bar, move)
        QtTest.QTest.mouseRelease(bar, QtCore.Qt.LeftButton, pos=end)
        self.app.processEvents()
        self.assertGreater(bar.value(), 0)
        self.assertLess(area.widget().pos().y(), initial_y)

    def test_scheme_column_labels_stay_visible_at_bottom_and_hide_at_form_top(self):
        dialog = self.offline()
        area, table = dialog.setup_scroll, dialog.variant_table
        area.verticalScrollBar().setValue(area.verticalScrollBar().maximum())
        self.app.processEvents()
        header = table.sticky_header
        self.assertTrue(header.isVisible())
        self.assertTrue(area.viewport().rect().contains(header.geometry()))
        self.assertEqual(header.geometry().top(), 0)
        self.assertEqual(header.model().headerData(1, QtCore.Qt.Horizontal), "EKF")
        self.assertEqual(header.model().headerData(4, QtCore.Qt.Horizontal), "过程噪声")
        table.setColumnWidth(0, 190)
        QtTest.QTest.qWait(20)
        table.horizontalScrollBar().setValue(table.horizontalScrollBar().maximum())
        self.app.processEvents()
        for column in range(table.columnCount()):
            self.assertEqual(header.sectionViewportPosition(column), table.header().sectionViewportPosition(column))
        area.verticalScrollBar().setValue(0)
        self.app.processEvents()
        self.assertFalse(header.isVisible())

    def test_keyboard_navigation_reveals_scheme_in_outer_page(self):
        dialog = self.offline()
        area, table = dialog.setup_scroll, dialog.variant_table
        table.setFocus()
        table.setCurrentItem(table.topLevelItem(0))
        QtTest.QTest.keyClick(table, QtCore.Qt.Key_End)
        self.app.processEvents()
        selected = table.visualItemRect(table.currentItem()).center()
        visible = area.viewport().mapFromGlobal(table.viewport().mapToGlobal(selected))
        self.assertTrue(area.viewport().rect().contains(visible))
        self.assertGreater(area.verticalScrollBar().value(), 0)
        for _ in range(18):
            QtTest.QTest.keyClick(table, QtCore.Qt.Key_Up)
        selected_top = table.visualItemRect(table.currentItem()).topLeft()
        visible_top = area.viewport().mapFromGlobal(table.viewport().mapToGlobal(selected_top))
        header_bottom = table.sticky_header.geometry().bottom() if table.sticky_header.isVisible() else 0
        self.assertGreater(visible_top.y(), header_bottom)

    def test_small_offline_window_keeps_actions_visible_while_content_scrolls(self):
        backend = FixtureBackend()
        backend.rows = [dict(id="existing", kind="offline", status="COMPLETE", started_at="2026-10-08T01:00:00Z",
                             label="离线融合_" + "recording_20261008_010203_"*7)]
        dialog = self.offline(backend)
        for value in (0, dialog.setup_scroll.verticalScrollBar().maximum()):
            dialog.setup_scroll.verticalScrollBar().setValue(value)
            self.app.processEvents()
            for action in (dialog.start_button, dialog.history_button):
                rect = QtCore.QRect(action.mapTo(dialog, QtCore.QPoint()), action.size())
                self.assertTrue(dialog.rect().contains(rect))
                self.assertTrue(action.isVisible())
        self.assertGreater(dialog.setup_scroll.viewport().height(), 200)
        self.assertLessEqual(dialog.history.geometry().right(), dialog.history_button.geometry().left())

    def test_small_live_window_scrolls_form_without_moving_confirmation(self):
        dialog = LiveDialog(FixtureBackend())
        self.windows.append(dialog)
        dialog.show()
        drain(self.app, lambda: dialog.defaults_loaded)
        dialog.resize(800, 520)
        QtTest.QTest.qWait(20)
        bar = dialog.form_scroll.verticalScrollBar()
        button_position = dialog.start_button.mapTo(dialog, QtCore.QPoint())
        self.assertGreater(bar.maximum(), 0)
        self.wheel(dialog.mode)
        self.assertGreater(bar.value(), 0)
        self.assertEqual(dialog.start_button.mapTo(dialog, QtCore.QPoint()), button_position)
        self.assertTrue(dialog.rect().contains(QtCore.QRect(button_position, dialog.start_button.size())))


if __name__ == "__main__":
    unittest.main()
