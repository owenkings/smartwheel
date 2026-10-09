"""Result inspection interactions in a nested page; synthetic local files only."""
import os
from pathlib import Path
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtCore, QtGui, QtTest, QtWidgets, sip
import numpy as np

from wc_panel.results_viewer import ImageView, ImageTools, ResultCard, ResultsViewer
from wc_panel.ui_common import PanelScrollArea, apply_theme


class ResultsInteractionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        apply_theme(cls.app)

    def setUp(self):
        self.windows = []
        self.scratch = tempfile.TemporaryDirectory(dir=os.environ.get("PANEL_TEST_TMP_ROOT"))
        self.path = Path(self.scratch.name) / "grid.png"
        image = QtGui.QImage(1200, 900, QtGui.QImage.Format_RGB32)
        image.fill(QtGui.QColor("white"))
        painter = QtGui.QPainter(image)
        painter.fillRect(160, 130, 700, 440, QtGui.QColor("#687b8f"))
        painter.fillRect(210, 180, 600, 340, QtGui.QColor("#f0f3f6"))
        painter.end()
        self.assertTrue(image.save(str(self.path)))

    def tearDown(self):
        for window in self.windows:
            if not sip.isdeleted(window):
                window.close()
        self.app.processEvents()
        self.app.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        self.windows.clear()
        self.scratch.cleanup()

    def wheel(self, widget, delta=-120, point=None, pixels=0):
        point = widget.rect().center() if point is None else point
        event = QtGui.QWheelEvent(QtCore.QPointF(point), QtCore.QPointF(widget.mapToGlobal(point)),
                                  QtCore.QPoint(0, pixels), QtCore.QPoint(0, delta),
                                  QtCore.Qt.NoButton, QtCore.Qt.NoModifier,
                                  QtCore.Qt.NoScrollPhase, False)
        self.app.sendEvent(widget, event)
        self.app.processEvents()

    def image_view(self):
        view = ImageView()
        self.windows.append(view)
        view.resize(700, 500)
        view.show()
        view.set_image(self.path)
        self.app.processEvents()
        return view

    def nested_image(self):
        area = PanelScrollArea()
        self.windows.append(area)
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        image = ImageView()
        image.setFixedHeight(300)
        layout.addWidget(image)
        filler = QtWidgets.QWidget()
        filler.setMinimumHeight(1000)
        layout.addWidget(filler)
        area.setWidget(content)
        area.resize(700, 430)
        area.show()
        image.set_image(self.path)
        self.app.processEvents()
        return area, image

    def test_nested_map_wheel_changes_zoom_and_never_scrolls_outer_page(self):
        area, image = self.nested_image()
        bar = area.verticalScrollBar()
        self.assertGreater(bar.maximum(), 0)
        before = image.transform().m11()
        self.wheel(image.viewport())
        self.assertLess(image.transform().m11(), before)
        self.assertEqual(bar.value(), 0)
        self.assertFalse(image.fit_mode)
        self.assertFalse(image.verticalScrollBar().isVisible())

    def test_wheel_keeps_point_under_mouse_including_fit_image_edges(self):
        view = self.image_view()
        point = QtCore.QPoint(180, 190)
        before = view.mapToScene(point)
        self.wheel(view.viewport(), delta=240, point=point)
        after = view.mapToScene(point)
        self.assertLess((after-before).manhattanLength()*view.transform().m11(), 3)

    def test_touchpad_pixel_delta_also_zooms(self):
        view = self.image_view()
        before = view.transform().m11()
        self.wheel(view.viewport(), delta=0, pixels=40)
        self.assertGreater(view.transform().m11(), before)

    def test_zoom_controls_fit_and_empty_controls_are_consistent(self):
        view = self.image_view()
        tools = ImageTools(view)
        self.windows.append(tools)
        tools.show()
        fitted = view.transform().m11()
        QtTest.QTest.mouseClick(tools.plus, QtCore.Qt.LeftButton)
        self.assertAlmostEqual(view.transform().m11(), fitted*1.25)
        self.assertNotIn("适应", tools.percent.text())
        QtTest.QTest.mouseClick(tools.minus, QtCore.Qt.LeftButton)
        self.assertAlmostEqual(view.transform().m11(), fitted)
        QtTest.QTest.mouseClick(tools.fit, QtCore.Qt.LeftButton)
        self.assertTrue(view.fit_mode)
        self.assertIn("适应", tools.percent.text())
        view.clear_image()
        self.assertFalse(tools.plus.isEnabled())
        self.assertFalse(tools.fit.isEnabled())
        self.assertEqual(tools.percent.text(), "暂无图片")

    def test_resize_preserves_manual_zoom_and_scene_center(self):
        view = self.image_view()
        view.zoom_by(2)
        view.centerOn(700, 420)
        before = view.mapToScene(view.viewport().rect().center())
        scale = view.transform().m11()
        view.resize(960, 620)
        self.app.processEvents()
        after = view.mapToScene(view.viewport().rect().center())
        self.assertAlmostEqual(view.transform().m11(), scale)
        self.assertLess((after-before).manhattanLength()*scale, 4)

    def test_fit_mode_continues_fitting_when_resized_and_double_click_resets(self):
        view = self.image_view()
        before = view.transform().m11()
        view.resize(1000, 730)
        self.app.processEvents()
        self.assertTrue(view.fit_mode)
        self.assertGreater(view.transform().m11(), before)
        fitted = view.transform().m11()
        view.zoom_by(2)
        QtTest.QTest.mouseDClick(view.viewport(), QtCore.Qt.LeftButton)
        self.assertTrue(view.fit_mode)
        self.assertAlmostEqual(view.transform().m11(), fitted)

    def test_dragging_map_pans_image_without_changing_scale(self):
        view = self.image_view()
        view.zoom_by(2)
        scale = view.transform().m11()
        start = view.viewport().rect().center()
        before = view.mapToScene(start)
        end = start+QtCore.QPoint(65, 35)
        QtTest.QTest.mousePress(view.viewport(), QtCore.Qt.LeftButton, pos=start)
        event = QtGui.QMouseEvent(QtCore.QEvent.MouseMove, QtCore.QPointF(end),
                                  QtCore.QPointF(view.viewport().mapToGlobal(end)),
                                  QtCore.Qt.NoButton, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier)
        self.app.sendEvent(view.viewport(), event)
        QtTest.QTest.mouseRelease(view.viewport(), QtCore.Qt.LeftButton, pos=end)
        after = view.mapToScene(start)
        self.assertLess(after.x(), before.x())
        self.assertLess(after.y(), before.y())
        self.assertAlmostEqual(view.transform().m11(), scale)

    def test_route_image_uses_same_zoom_controls_and_tab_api(self):
        card = ResultCard(dict(name="合成路线图片", path=self.scratch.name, images=[],
                               trajectories=[str(self.path)], clouds=[]))
        self.windows.append(card)
        card.resize(860, 740)
        card.show()
        card.tabs.setCurrentIndex(1)
        self.app.processEvents()
        self.assertEqual(card.tabs.widget(1), card.route_page)
        self.assertTrue(card.route_tools.isVisible())
        self.assertEqual(card.route_stack.currentWidget(), card.route_image)
        before = card.route_image.transform().m11()
        card.route_tools.plus.click()
        self.assertGreater(card.route_image.transform().m11(), before)
        self.assertEqual(card.tabs.count(), 3)

    def test_failed_image_read_clears_previously_displayed_map(self):
        view = self.image_view()
        with self.assertRaises(ValueError):
            view.set_image(Path(self.scratch.name)/"missing.png")
        self.assertFalse(view.has_image)
        self.assertEqual(len(view.scene().items()), 0)
        self.assertEqual(view.message, "图片读取失败")

    def test_cloud_wheel_remains_interactive_inside_comparison_page(self):
        results = [dict(name="合成结果 "+str(i), path=self.scratch.name, images=[str(self.path)],
                        trajectories=[], clouds=[]) for i in range(3)]
        viewer = ResultsViewer(results)
        self.windows.append(viewer)
        viewer.resize(1050, 700)
        viewer.show()
        viewer.mode.setCurrentIndex(2)
        cloud = viewer.cards[0].cloud_canvas
        cloud.set_cloud(np.array([[0, 0, 0], [1, 2, 3]], dtype=float))
        self.app.processEvents()
        before = cloud.zoom
        scroll = viewer.scroll.verticalScrollBar()
        scroll.setValue(0)
        self.wheel(cloud, delta=-120)
        self.assertLess(cloud.zoom, before)
        self.assertEqual(scroll.value(), 0)

    def test_large_portrait_export_fits_visible_card_and_bottom_controls(self):
        # Export pixels must not become the scroll page's preferred height.
        large = QtGui.QImage(2400, 3600, QtGui.QImage.Format_RGB32)
        large.fill(QtGui.QColor("white"))
        self.assertTrue(large.save(str(self.path)))
        viewer = ResultsViewer([dict(name="大幅地图适应窗口", path=self.scratch.name,
                                     images=[str(self.path)], trajectories=[], clouds=[])])
        self.windows.append(viewer)
        viewer.show()
        card = viewer.cards[0]
        for size in ((1480, 940), (1180, 860), (1020, 780)):
            viewer.resize(*size)
            card.tabs.setCurrentIndex(0)
            self.app.processEvents()
            self.app.processEvents()
            card.map_tools.fit.click()
            self.assertEqual(viewer.scroll.verticalScrollBar().maximum(), 0, size)
            image_on_screen = card.map_view.mapFromScene(card.map_view.image_rect).boundingRect()
            self.assertTrue(card.map_view.viewport().rect().adjusted(-2, -2, 2, 2).contains(image_on_screen), size)
            bottom = card.map_status.mapTo(viewer.scroll.viewport(), card.map_status.rect().bottomRight())
            self.assertLess(bottom.y(), viewer.scroll.viewport().height(), size)
            self.assertIn("适应", card.map_tools.percent.text())
            card.tabs.setCurrentIndex(2)
            self.app.processEvents()
            reset = next(b for b in card.cloud_page.findChildren(QtWidgets.QPushButton) if b.text() == "复位视角")
            button_bottom = reset.mapTo(viewer.scroll.viewport(), reset.rect().bottomRight())
            self.assertLess(button_bottom.y(), viewer.scroll.viewport().height(), size)
            self.assertTrue(reset.isVisible())

    def test_multiple_cards_scroll_without_each_card_exceeding_viewport_height(self):
        results = [dict(name="对比 "+str(i), path=self.scratch.name, images=[str(self.path)],
                        trajectories=[], clouds=[]) for i in range(4)]
        viewer = ResultsViewer(results)
        self.windows.append(viewer)
        viewer.resize(1280, 940)
        viewer.show()
        self.app.processEvents()
        self.app.processEvents()
        self.assertGreater(viewer.scroll.verticalScrollBar().maximum(), 0)
        for card in viewer.cards:
            self.assertLessEqual(card.height(), viewer.scroll.viewport().height())


if __name__ == "__main__":
    unittest.main()
