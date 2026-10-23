import json
from pathlib import Path
from unittest.mock import patch

from test_backend import Fixture, fake_resolve
from wc_panel.backend import PanelBackend
from wc_panel.storage import atomic_json


class AutoCompactionTests(Fixture):
    def plan(self,backend,options=None):
        with patch('wc_panel.backend.resolve_user_destination',fake_resolve):
            return backend.plan_offline(self.dataset,self.results,options or {})

    def test_all_variants_interleave_map_and_compaction_with_separate_candidates(self):
        backend=PanelBackend(self.project);plan=self.plan(backend,{'all':True})
        self.assertTrue(plan['options']['auto_compact'])
        self.assertEqual((plan['task_count'],plan['comparison_count']),(46,22))
        candidates=plan['tasks'][:2]
        self.assertTrue(all(t['id'].startswith('motion_candidate') for t in candidates))
        for map_task,storage_task in zip(plan['tasks'][2::2],plan['tasks'][3::2]):
            self.assertTrue(map_task['variant'])
            self.assertEqual(storage_task['stage'],'compaction')
            self.assertEqual(storage_task['output'],map_task['output'])
            self.assertEqual(storage_task['id'],'compact__'+map_task['id'])
            preflight=map_task['preflight']
            self.assertIn('preflight',preflight)
            self.assertEqual('--geometry' in preflight,map_task['variant']['geometry'])
            command=storage_task['commands'][0]
            self.assertEqual(command[command.index('--dataset')+1],str(self.dataset))
            self.assertEqual(command[command.index('--run')+1],map_task['output'])
        self.assertFalse(Path(plan['output']).exists())

    def test_opt_out_keeps_original_algorithms_and_native_frame_rates(self):
        backend=PanelBackend(self.project);plan=self.plan(backend,{'all':True,'auto_compact':False})
        self.assertEqual(plan['task_count'],24)
        self.assertFalse(any(t.get('stage')=='compaction' or 'preflight' in t for t in plan['tasks']))
        enabled=self.plan(backend,{'all':True,'auto_compact':True})
        tasks=[{k:v for k,v in t.items() if k!='preflight'} for t in enabled['tasks'] if t.get('stage')!='compaction']
        for a,b in zip(tasks,plan['tasks']):
            self.assertEqual(a['variant'],b['variant'])
            # Unique output IDs differ; all algorithm/rate flags must remain equal.
            ca,cb=a['commands'][0],b['commands'][0]
            normalize=lambda command:[v.replace(enabled['output'],'BATCH').replace(plan['output'],'BATCH') for v in command]
            self.assertEqual(normalize(ca),normalize(cb))

    def test_new_setting_defaults_on_and_preserves_last_successful_selection(self):
        backend=PanelBackend(self.project)
        self.assertTrue(backend.settings()['auto_compact'])
        plan=self.plan(backend,{'auto_compact':False})
        with patch('wc_panel.backend.resolve_user_destination',fake_resolve),patch.object(backend._manager,'submit',return_value={'id':'mock'}):
            backend.start_offline(plan)
        self.assertFalse(PanelBackend(self.project).settings()['auto_compact'])
        with patch('wc_panel.backend.resolve_user_destination',fake_resolve):
            backend.update_settings(self.recordings,self.results)
        self.assertFalse(backend.settings()['auto_compact'])
        self.assertFalse(self.plan(backend)['options']['auto_compact'])

    def test_retry_rebuilds_only_storage_commands_and_preserves_failed_history(self):
        backend=PanelBackend(self.project);plan=self.plan(backend)
        batch=Path(plan['output']);batch.mkdir();target=Path(plan['tasks'][-1]['output']);target.mkdir()
        atomic_json(target/'result.json',{'status':'OFFLINE_COMPARISON_COMPLETE'})
        old=dict(id='previous',kind='offline',status='FAILED',output=str(batch),task_states=[dict(
                 id=plan['tasks'][-1]['id'],status='FAILED',output=str(target))])
        plan['tasks'][-1]['commands']=[['NEVER_EXECUTE_UNTRUSTED_SAVED_COMMAND']]
        metadata=dict(job_id='previous',status='FAILED',plan=plan);atomic_json(batch/'panel_result.json',metadata)
        with patch('wc_panel.backend.resolve_user_destination',fake_resolve),patch.object(backend._manager,'get',return_value=old),\
             patch.object(backend._store,'active',return_value=False),patch.object(backend._manager,'submit',return_value={'id':'retry'}) as submit:
            backend.retry_offline_compaction('previous')
        task,=submit.call_args.args[2]
        self.assertEqual(task['stage'],'compaction')
        self.assertIn('wc_runtime.offline_compaction',task['commands'][0])
        self.assertNotIn('NEVER_EXECUTE_UNTRUSTED_SAVED_COMMAND',str(task))
        self.assertEqual(json.loads((batch/'_compaction/previous_panel_result.json').read_text()),metadata)

    def test_reject_non_boolean_and_incomplete_diagnostic_compaction(self):
        backend=PanelBackend(self.project)
        for value in (1,'true',None):
            with self.assertRaisesRegex(ValueError,'布尔'):self.plan(backend,{'auto_compact':value})
        atomic_json(self.dataset/'capture_manifest.json',{'status':'PARTIAL','recording_complete':False})
        with self.assertRaisesRegex(ValueError,'诊断结果'):self.plan(backend,{'allow_partial':True,'auto_compact':True})
        self.assertFalse(self.plan(backend,{'allow_partial':True})['options']['auto_compact'])



def test_capacity_preflight_failure_prevents_mapping_and_later_tasks(tmp_path):
    from wc_panel.jobs import JobManager
    project=tmp_path/'project';project.mkdir()
    manager=JobManager(project,tmp_path/'jobs',detached_offline=False)
    tasks=[dict(id='map',label='map',preflight=['CAPACITY'],commands=[['MAP']]),
           dict(id='compact__map',label='compact',commands=[['COMPACT']])]
    with patch('wc_panel.jobs.threading.Thread.start'):
        row=manager.submit('offline','capacity test',tasks)
    class Exited:
        pid=987654321
        returncode=1
        def poll(self):return self.returncode
    with patch('wc_panel.jobs.subprocess.Popen',return_value=Exited()) as spawn:
        manager._run(row['id'],tasks,())
    assert spawn.call_count==1
    assert spawn.call_args.args[0]==['CAPACITY']
    done=manager.get(row['id'])
    assert done['status']=='FAILED'
    assert [s['status'] for s in done['task_states']]==['FAILED','SKIPPED']
