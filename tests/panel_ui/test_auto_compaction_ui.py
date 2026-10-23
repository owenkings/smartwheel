import unittest
from unittest import mock
from PyQt5 import QtCore, QtWidgets
from wc_panel.ui_dialogs import OfflineDialog
from test_panel_ui import FixtureBackend, drain


class AutoCompactionUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.backend=FixtureBackend();self.dialog=OfflineDialog(self.backend);drain(self.app)

    def tearDown(self):
        self.dialog.close();self.app.sendPostedEvents(None,QtCore.QEvent.DeferredDelete);drain(self.app)

    def test_default_on_and_opt_out_reaches_plan(self):
        self.assertTrue(self.dialog.auto_compact.isChecked())
        self.dialog.auto_compact.setChecked(False);self.dialog.select_variants(True)
        with mock.patch.object(self.dialog,'confirm_plan'):
            self.dialog.make_plan();drain(self.app)
        call=next(c for c in self.backend.calls if c[0]=='plan')
        self.assertFalse(call[-1]['auto_compact'])

    def test_remembered_false_is_loaded(self):
        self.backend.roots['auto_compact']=False
        self.dialog.loaded((self.backend.settings(),self.backend.capabilities(),[]));drain(self.app)
        self.assertFalse(self.dialog.auto_compact.isChecked())

    def test_retry_only_enabled_for_interrupted_storage_stage(self):
        row=dict(id='old',status='FAILED',task_states=[dict(id='compact__a',status='FAILED')])
        self.dialog.show_job(row);self.dialog.job_id='old'
        self.assertTrue(self.dialog.retry_compact_button.isEnabled())
        retry=mock.Mock(return_value=dict(id='retry',status='RUNNING',task_states=[]))
        self.backend.retry_offline_compaction=retry
        self.dialog.retry_compact_button.click();drain(self.app)
        retry.assert_called_once_with('old')
        self.assertEqual(self.dialog.job_id,'retry')
        self.assertFalse(self.dialog.retry_compact_button.isEnabled())
