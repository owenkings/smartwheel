"""Choice-menu interaction checks; fixture widgets only, no runtime tasks."""
import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtCore, QtGui, QtTest, QtWidgets

from wc_panel.ui_common import apply_theme
from wc_panel.ui_popup import ComboBox


class PopupControlsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        apply_theme(cls.app)

    def setUp(self):
        self.dialog = QtWidgets.QDialog()
        self.dialog.resize(640, 520)
        self.combo = ComboBox(self.dialog)
        self.combo.setGeometry(32, 70, 560, 44)
        self.combo.addItem("双雷达", "both")
        self.combo.addItem("仅左雷达（调试）", "left")
        self.combo.addItem("仅右雷达（调试）", "right")
        self.footer = QtWidgets.QPushButton("确认并开始录制", self.dialog)
        self.footer.setGeometry(420, 448, 172, 44)
        self.footer.setProperty("popupFooter", True)
        self.dialog.show()
        self.app.processEvents()

    def tearDown(self):
        self.combo.hidePopup()
        self.dialog.close()
        self.dialog.deleteLater()
        self.app.processEvents()

    def open_menu(self):
        self.combo.showPopup()
        self.app.processEvents()
        self.assertTrue(self.combo.popup_widget.isVisible())

    def test_keyboard_navigation_is_staged_and_escape_keeps_configuration(self):
        self.open_menu()
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_Down)
        self.assertEqual(self.combo.view().currentIndex().row(), 1)
        self.assertEqual(self.combo.currentData(), "both")
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_Escape)
        self.assertFalse(self.combo.popup_widget.isVisible())
        self.assertEqual(self.combo.currentData(), "both")

    def test_enter_commits_once_and_emits_combo_signals(self):
        activated = QtTest.QSignalSpy(self.combo.activated[int])
        changed = QtTest.QSignalSpy(self.combo.currentIndexChanged[int])
        self.open_menu()
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_End)
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_Return)
        self.assertEqual(self.combo.currentData(), "right")
        self.assertEqual(list(activated), [[2]])
        self.assertEqual(list(changed), [[2]])
        self.assertFalse(self.combo.popup_widget.isVisible())

    def test_disabled_choice_cannot_be_clicked_or_selected_with_keyboard(self):
        self.combo.model().item(1).setEnabled(False)
        self.open_menu()
        index = self.combo.model().index(1, 0)
        QtTest.QTest.mouseClick(self.combo.view().viewport(), QtCore.Qt.LeftButton,
                               pos=self.combo.view().visualRect(index).center())
        self.assertEqual(self.combo.currentData(), "both")
        self.assertTrue(self.combo.popup_widget.isVisible())
        self.combo.view().setCurrentIndex(self.combo.model().index(0, 0))
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_Down)
        self.assertEqual(self.combo.view().currentIndex().row(), 2)
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_Return)
        self.assertEqual(self.combo.currentData(), "right")

    def test_mouse_click_commits_only_clicked_row(self):
        self.open_menu()
        index = self.combo.model().index(1, 0)
        activated = QtTest.QSignalSpy(self.combo.activated[int])
        QtTest.QTest.mouseClick(self.combo.view().viewport(), QtCore.Qt.LeftButton,
                               pos=self.combo.view().visualRect(index).center())
        self.assertEqual(self.combo.currentData(), "left")
        self.assertEqual(list(activated), [[1]])

    def test_outside_click_dismisses_without_committing_hover(self):
        self.open_menu()
        started = QtTest.QSignalSpy(self.footer.clicked)
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_Down)
        QtTest.QTest.mouseClick(self.footer, QtCore.Qt.LeftButton)
        self.assertFalse(self.combo.popup_widget.isVisible())
        self.assertEqual(self.combo.currentData(), "both")
        self.assertEqual(len(started), 0, "dismissing a menu must not start a task underneath")

    def test_menu_never_covers_footer_and_long_list_scrolls_to_current(self):
        for number in range(40):
            self.combo.addItem("录制 {} / 很长的目录名称与完整配置说明".format(number), number)
        self.combo.setCurrentIndex(39)
        self.open_menu()
        popup = self.combo.popup_widget
        self.assertLess(popup.geometry().bottom(), self.footer.mapToGlobal(QtCore.QPoint()).y())
        self.assertLessEqual(self.combo.view().viewport().height(), 6*44)
        self.assertGreater(self.combo.view().verticalScrollBar().maximum(), 0)
        current_rect = self.combo.view().visualRect(self.combo.view().currentIndex())
        self.assertTrue(self.combo.view().viewport().rect().intersects(current_rect))
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_Home)
        self.assertEqual(self.combo.view().currentIndex().row(), 0)

    def test_history_preferred_above_and_resize_cancel_preserves_value(self):
        self.combo.move(32, 360)
        self.combo.setProperty("popupAbove", True)
        self.open_menu()
        self.assertLess(self.combo.popup_widget.geometry().bottom(), self.combo.mapToGlobal(QtCore.QPoint()).y())
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_Down)
        self.dialog.resize(650, 530)
        self.app.processEvents()
        self.assertFalse(self.combo.popup_widget.isVisible())
        self.assertEqual(self.combo.currentData(), "both")

    def test_type_search_respects_disabled_items_and_escape_cancels(self):
        self.combo.clear()
        self.combo.addItems(["Alpha", "Beta disabled", "Beta enabled", "Bravo", "Charlie"])
        self.combo.model().item(1).setEnabled(False)
        self.open_menu()
        QtTest.QTest.keyClicks(self.combo.view(), "be")
        self.assertEqual(self.combo.view().currentIndex().row(), 2)
        self.assertEqual(self.combo.currentIndex(), 0)
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_Return)
        self.assertEqual(self.combo.currentText(), "Beta enabled")

    def test_closed_wheel_never_changes_choice(self):
        self.combo.setFocus()
        point = self.combo.rect().center()
        event = QtGui.QWheelEvent(QtCore.QPointF(point), QtCore.QPointF(self.combo.mapToGlobal(point)),
                                 QtCore.QPoint(), QtCore.QPoint(0, -120), QtCore.Qt.NoButton,
                                 QtCore.Qt.NoModifier, QtCore.Qt.NoScrollPhase, False)
        self.app.sendEvent(self.combo, event)
        self.assertEqual(self.combo.currentData(), "both")

    def test_model_replacement_is_visible_and_popup_corners_are_transparent(self):
        model = QtGui.QStandardItemModel(self.combo)
        for name in ("Official", "Five state"):
            model.appendRow(QtGui.QStandardItem(name))
        self.combo.setModel(model)
        self.open_menu()
        self.assertIs(self.combo.view().model(), model)
        image = self.combo.popup_widget.grab().toImage()
        self.assertEqual(image.pixelColor(0, 0).alpha(), 0)
        self.assertEqual(image.pixelColor(image.width()//2, image.height()//2).alpha(), 255)

    def test_refresh_dismisses_staged_selection_before_replacing_or_clearing_model(self):
        self.open_menu()
        QtTest.QTest.keyClick(self.combo.view(), QtCore.Qt.Key_Down)
        model = QtGui.QStandardItemModel(self.combo)
        model.appendRow(QtGui.QStandardItem("Replacement"))
        self.combo.setModel(model)
        self.assertFalse(self.combo.popup_widget.isVisible())
        self.open_menu()
        self.assertIs(self.combo.view().model(), model)
        self.assertEqual(self.combo.currentText(), "Replacement")
        self.combo.clear()
        self.assertFalse(self.combo.popup_widget.isVisible())
        self.assertEqual(self.combo.count(), 0)


if __name__ == "__main__":
    unittest.main()
