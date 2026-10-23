"""Explicit experimental selection cannot weaken validated live capabilities."""
import copy
import json
from unittest.mock import patch

from test_backend import Fixture
from wc_panel.backend import PanelBackend
from wc_panel.live_calibration import capabilities


def live_rows(enabled=True, reason='synthetic experiment'):
    return {key:dict(enabled=enabled,reason=reason,
                     by_sides={side:dict(enabled=enabled,reason=reason) for side in ('all','left','right')})
            for key in ('live_mapping','live_motion_correction','live_geometry')}


class LiveExperimentTests(Fixture):
    def test_experimental_capability_does_not_promote_unknown_validated_geometry(self):
        path=self.project/'config/mapping_live.json'
        original=path.read_bytes()
        with patch('wc_runtime.live_experiment.experiment_capability',return_value={
                'available':True,'reason':'','provenance':{'status':'EXPERIMENTAL_REUSE_NOT_CALIBRATED',
                                                         'physical_accuracy_validated':False}}):
            actual=capabilities(self.project)
        self.assertFalse(actual['live_mapping']['enabled'])
        self.assertFalse(actual['live_motion_correction']['enabled'])
        self.assertEqual(actual['live_schemes']['validated']['live_mapping'],actual['live_mapping'])
        experiment=actual['live_schemes']['experiment']
        self.assertTrue(experiment['live_mapping']['enabled'])
        self.assertTrue(experiment['live_motion_correction']['enabled'])
        self.assertTrue(experiment['live_geometry']['enabled'])
        self.assertEqual(experiment['parameter_status'],'UNCALIBRATED_EXPERIMENT')
        self.assertFalse(experiment['provenance']['physical_accuracy_validated'])
        self.assertEqual(path.read_bytes(),original)

    def test_bad_experiment_evidence_is_not_a_fallback_to_validated_rows(self):
        with patch('wc_runtime.live_experiment.experiment_capability',return_value={
                'available':False,'reason':'candidate SHA-256 mismatch','provenance':None}):
            actual=capabilities(self.project)
        for row in actual['live_schemes']['experiment'].values():
            if isinstance(row,dict) and 'enabled' in row:
                self.assertFalse(row['enabled'])
                self.assertIn('SHA-256',row['reason'])

    def test_custom_profile_uses_the_same_configured_source_as_runtime(self):
        path=self.project/'config/mapping_live.json'
        mapping=json.loads(path.read_text())
        mapping['live_experiment_profile_config']='config/calibration/custom_experiment.json'
        path.write_text(json.dumps(mapping))
        for available in (True,False):
            with patch('wc_runtime.live_experiment.experiment_capability',return_value={
                    'available':available,'reason':'custom profile invalid' if not available else '',
                    'provenance':None}) as verify:
                actual=capabilities(self.project)
            self.assertEqual(verify.call_args.kwargs['profile_path'],mapping['live_experiment_profile_config'])
            self.assertIs(actual['live_schemes']['experiment']['live_mapping']['enabled'],available)

    def test_explicit_experiment_uses_selected_scheme_and_owned_cli_argument(self):
        backend=PanelBackend(self.project)
        path=self.project/'config/mapping_live.json';original=path.read_bytes()
        current=live_rows(False,'missing measured extrinsics')
        capability={**current,'live_schemes':{'validated':copy.deepcopy(current),'experiment':live_rows()}}
        with patch.object(backend,'capabilities',return_value=capability), \
             patch.object(backend._manager,'submit',return_value={}) as submit:
            with self.assertRaisesRegex(ValueError,'missing measured'):
                backend.start_live({'mode':'mapping'})
            self.assertFalse(submit.called)
            backend.start_live(dict(mode='mapping',live_scheme='experiment',motion_correction=True,geometry=True))
        command=submit.call_args.args[2][0]['commands'][0]
        self.assertEqual(command[command.index('--scheme')+1],'experiment')
        self.assertEqual(command[command.index('--motion-correction')+1],'on')
        self.assertIn('未标定实验',submit.call_args.args[1])
        self.assertEqual(path.read_bytes(),original)

    def test_experiment_failure_blocks_preview_and_official_geometry_stays_unsupported(self):
        backend=PanelBackend(self.project)
        capability={**live_rows(),'live_schemes':{'validated':live_rows(),
                                                'experiment':live_rows(False,'source hash changed')}}
        with patch.object(backend,'capabilities',return_value=capability), \
             patch.object(backend._manager,'submit',return_value={}) as submit:
            with self.assertRaisesRegex(ValueError,'source hash changed'):
                backend.start_live(dict(mode='preview',live_scheme='experiment'))
            with self.assertRaisesRegex(ValueError,'尚未实现'):
                backend.start_live(dict(mode='mapping',live_scheme='experiment',estimator='robot_localization',geometry=True))
            self.assertFalse(submit.called)

    def test_validated_default_is_explicit_and_unknown_schemes_fail(self):
        backend=PanelBackend(self.project)
        self.assertEqual(backend.capabilities()['live_defaults']['live_scheme'],'validated')
        with patch.object(backend._manager,'submit',return_value={}) as submit:
            backend.start_live({})
        command=submit.call_args.args[2][0]['commands'][0]
        self.assertEqual(command[command.index('--scheme')+1],'validated')
        for value in ('offline',None,True):
            with self.assertRaisesRegex(ValueError,'参数方案'):
                backend.start_live({'live_scheme':value})
