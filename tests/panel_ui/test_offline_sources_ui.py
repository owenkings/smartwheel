"""Offline source choices reflect actual recorded streams and reach the plan."""
import os
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtCore, QtWidgets
from wc_panel.ui_common import apply_theme
from wc_panel.ui_dialogs import OfflineDialog
from test_panel_ui import FixtureBackend, drain


class OfflineSourceUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        apply_theme(cls.app)

    def setUp(self):
        self.backend = FixtureBackend()
        self.dialog = OfflineDialog(self.backend)
        self.dialog.show()
        drain(self.app)

    def tearDown(self):
        drain(self.app)
        self.dialog.close()
        self.app.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        self.app.processEvents()

    def test_selected_right_cloud_reaches_offline_plan(self):
        dialog = self.dialog
        dialog.sides.setCurrentIndex(dialog.sides.findData("right"))
        dialog.select_variants(True)
        with mock.patch.object(dialog, "confirm_plan"):
            dialog.make_plan()
            drain(self.app)
        plan_call = next(call for call in self.backend.calls if call[0] == "plan")
        self.assertEqual(plan_call[-1]["sides"], "right")

    def test_single_sensor_recording_disables_missing_choices_and_cloud_variants(self):
        dialog = self.dialog
        record = self.backend.list_recordings("/fixture")[0]
        record.update(available_sides=["left"], available_clouds_by_side={"left":["raw"], "right":[]})
        dialog.dataset.clear()
        dialog.scanned(dialog.scan_generation, [record])
        self.assertEqual(dialog.sides.currentData(), "left")
        self.assertFalse(dialog.sides.model().item(0).isEnabled())
        self.assertFalse(dialog.sides.model().item(2).isEnabled())
        dialog.select_variants(True)
        chosen = dialog.selected_variants()
        self.assertTrue(chosen)
        self.assertTrue(all(v["cloud"] == "raw" for v in dialog.variants if v["id"] in chosen))

    def test_inventory_failure_blocks_execution_and_shows_reason(self):
        dialog = self.dialog
        record = self.backend.list_recordings("/fixture")[0]
        record.update(source_inventory_error="SQLite unavailable", available_sides=[])
        dialog.dataset.clear()
        dialog.scanned(dialog.scan_generation, [record])
        dialog.select_variants(True)
        self.assertFalse(dialog.start_button.isEnabled())
        self.assertFalse(dialog.sides.isEnabled())
        self.assertIn("SQLite unavailable", dialog.source_note.text())


if __name__ == "__main__":
    unittest.main()
