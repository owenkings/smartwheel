"""Explicit live filter selection is passed to the owned mapping command."""
from unittest.mock import patch

from test_backend import Fixture
from wc_panel.backend import PanelBackend


class LiveFilterOptionsTests(Fixture):
    def test_filter_choice_overrides_default_without_rewriting_configuration(self):
        backend = PanelBackend(self.project)
        path = self.project/'config/mapping_live.json'
        original = path.read_bytes()
        for enabled in (False, True):
            with patch.object(backend, 'capabilities', return_value={
                    'live_mapping': {'by_sides': {'left': {'enabled': True}}}}), \
                 patch.object(backend._manager, 'submit', return_value={}) as submit:
                backend.start_live(dict(mode='mapping', sides='left', cloud_filter_enabled=enabled))
            command = submit.call_args.args[2][0]['commands'][0]
            self.assertEqual(command[command.index('--cloud-filter')+1], 'on' if enabled else 'off')
            self.assertIn('--interactive', command)
            self.assertEqual(path.read_bytes(), original)

    def test_filter_default_matches_config_and_invalid_types_are_rejected(self):
        import json
        backend = PanelBackend(self.project)
        expected = json.loads((self.project/'config/mapping_live.json').read_text()).get('cloud_filter', {}).get('enabled', False)
        self.assertIs(backend.capabilities()['live_defaults']['cloud_filter_enabled'], expected)
        for invalid in (1, 'on', None):
            with self.assertRaisesRegex(ValueError, '点云筛选'):
                backend.start_live(dict(cloud_filter_enabled=invalid))
