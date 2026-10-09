import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from wc_panel.backend import PanelBackend
from wc_panel import catalog
from wc_panel.jobs import JobManager
from wc_panel.parameters import ParameterStore
from wc_panel.storage import DataStore, atomic_json

PROJECT=Path(__file__).resolve().parents[2]


class Guard:
    required_uuid=None
    def check(self): return {'status':'AVAILABLE'}


def fake_resolve(project,path,**kwargs): return Path(path),Guard()


class Fixture(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='wc_panel_test_')
        self.root=Path(self.temp.name)
        self.project=self.root/'project';(self.project/'config').mkdir(parents=True)
        # Synthetic projects use one coherent public template, never host bindings.
        for name in ('hardware_setup.json','mapping_live.json','cameras.json','device_bindings.json','wheel_feedback_current.json'):
            shutil.copyfile(PROJECT/'config'/name,self.project/'config'/name)
        atomic_json(self.project/'config/storage.json',{'schema_version':1,'enabled':False})
        self.recordings=self.project/'data/experiments';self.recordings.mkdir(parents=True)
        self.results=self.project/'data/analysis';self.results.mkdir()
        self.dataset=self.recordings/'capture_20261007_120000';(self.dataset/'bag').mkdir(parents=True)
        with sqlite3.connect(self.dataset/'bag/data.db3') as db:
            db.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT)')
            db.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)')
            for index,(side,suffix) in enumerate(((side,suffix) for side in ('left','right')
                                                 for suffix in ('','_filtered')),1):
                db.execute('INSERT INTO topics VALUES(?,?,?)',(index,'/wc_mapping/lidar_'+side+'/source_frame'+suffix,
                                                               'wc_interfaces/msg/SourceFrame'))
                db.execute('INSERT INTO messages VALUES(?,?,?,?)',(index,index,index,b'synthetic-test-payload'))
        db.close()
        atomic_json(self.dataset/'runtime_config.json',{'mode':'all'})
        atomic_json(self.dataset/'capture_manifest.json',{'status':'COMPLETE','recording_complete':True,'bag_path':'bag','runtime_config_path':'runtime_config.json'})
    def tearDown(self): self.temp.cleanup()


class CatalogTests(Fixture):
    def test_result_cells_keep_separate_clouds_and_matching_trajectory(self):
        batch=self.results/'batch';batch.mkdir()
        for name in ('02_official','04_five_state'):
            cell=batch/name;native=batch/'native'/name
            cell.mkdir();native.mkdir(parents=True)
            (cell/'trajectory.csv').write_text('x,y\n1,2\n')
            (native/'map.ply').write_text('ply\nend_header\n')
        atomic_json(batch/'result.json',{'status':'COMPLETE','cells':{'02_official':{},'04_five_state':{}}})
        rows=catalog.results(self.results)
        self.assertEqual(len(rows),2)
        self.assertTrue(all(len(row['clouds'])==len(row['trajectories'])==1 for row in rows))
        self.assertNotEqual(rows[0]['clouds'],rows[1]['clouds'])

    def test_valid_and_incomplete_recordings_are_distinct(self):
        (self.recordings/'empty').mkdir()
        rows=catalog.recordings(self.recordings)
        self.assertEqual(len(rows),1);self.assertTrue(rows[0]['complete'])
        atomic_json(self.dataset/'capture_manifest.json',{'status':'PARTIAL','recording_complete':False})
        self.assertFalse(catalog.recordings(self.recordings)[0]['complete'])

    def test_matrix_is_legal_and_geometry_stays_visible_disabled(self):
        all_rows=catalog.variants()
        enabled=catalog.choose_variants({'all':True})
        self.assertEqual(len(enabled),22)
        disabled=[row for row in all_rows if not row['enabled']]
        self.assertEqual(len(disabled),2)
        self.assertTrue(all(row['estimator']=='robot_localization' and row['geometry'] for row in disabled))
        with self.assertRaisesRegex(ValueError,'尚未实现'):
            catalog.choose_variants({'variants':[disabled[0]['id']]})
        self.assertFalse(any(row['estimator']=='robot_localization' and row['geometry'] for row in enabled))


class DeletionTests(Fixture):
    def test_confirmed_exact_scope_keeps_sibling(self):
        kept=self.recordings/'keep';kept.mkdir();(kept/'original').write_bytes(b'keep')
        store=DataStore(self.project,resolver=fake_resolve)
        plan=store.prepare_delete(self.recordings,[self.dataset])
        self.assertGreater(plan['bytes'],0)
        receipt=store.execute_delete(plan['token'])
        self.assertIn('allocated_bytes',plan)
        self.assertEqual(receipt['free_delta_bytes'],receipt['free_after_bytes']-receipt['free_before_bytes'])
        self.assertFalse(self.dataset.exists());self.assertEqual((kept/'original').read_bytes(),b'keep')
        with self.assertRaises(ValueError):store.execute_delete(plan['token'])

    def test_changed_tree_requires_new_confirmation(self):
        store=DataStore(self.project,resolver=fake_resolve)
        plan=store.prepare_delete(self.recordings,[self.dataset])
        (self.dataset/'new_data').write_bytes(b'new')
        with self.assertRaisesRegex(ValueError,'变化'): store.execute_delete(plan['token'])
        self.assertTrue((self.dataset/'new_data').exists())

    def test_active_task_and_root_deletion_rejected(self):
        active=[]
        store=DataStore(self.project,lambda:active,resolver=fake_resolve)
        with self.assertRaises(ValueError):store.prepare_delete(self.recordings,[self.recordings])
        plan=store.prepare_delete(self.recordings,[self.dataset]);active.append(self.dataset)
        with self.assertRaisesRegex(ValueError,'占用'):store.execute_delete(plan['token'])

    @unittest.skipIf(os.name=='nt','POSIX symlink fixture')
    def test_link_never_followed(self):
        (self.dataset/'linked').symlink_to(self.results,target_is_directory=True)
        store=DataStore(self.project,resolver=fake_resolve)
        with self.assertRaisesRegex(ValueError,'链接'):store.prepare_delete(self.recordings,[self.dataset])


class ParameterTests(Fixture):
    def calibration_fixture(self):
        from wc_panel.live_calibration import device_context
        context=device_context(self.project)
        directory=self.root/'declaration';directory.mkdir()
        bias_evidence=directory/'bias.evidence.json';atomic_json(bias_evidence,{'synthetic':True,'kind':'operator_confirmation'})
        bias=dict(status='INDEPENDENTLY_CONFIRMED',sensor_id=context['imu_sensor_id'],bias_native_rad_s=[.001,0.,0.],
                  evidence_id='synthetic_bias',evidence_sha256=hashlib.sha256(bias_evidence.read_bytes()).hexdigest(),
                  source='operator_visual_confirmation')
        bias_path=directory/'bias.json';atomic_json(bias_path,bias)
        evidence=directory/'device.evidence.json';atomic_json(evidence,{'synthetic':True,'kind':'device_calibration'})
        declaration=dict(schema_version=1,status='EXPLICIT_DEVICE_CALIBRATION_DECLARATION',scope='device',
            imu_sensor_id=context['imu_sensor_id'],wheel_device_id=context['wheel_device_id'],R_reference_imu=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
            wheel_yaw_scale=1.01,wheel_yaw_speed_coefficient=.01,confirmed_gyro_bias=bias,evidence_id='synthetic_device',
            evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
        declaration_path=directory/'device.json';atomic_json(declaration_path,declaration)
        return declaration_path,bias_path,evidence

    def test_live_import_saves_four_frozen_files_and_two_pointers(self):
        from wc_panel.live_calibration import load_declaration,verify_value
        declaration,bias,evidence=self.calibration_fixture()
        value=load_declaration(self.project,declaration,bias,evidence)
        store=ParameterStore(self.project,self.results/'backups');before=store.read()
        with self.assertRaisesRegex(ValueError,'警告'):store.save('live_motion',value,expected_revision=before['revision'])
        proof=store.request_edit('live_motion')
        result=store.save('live_motion',value,proof['token'],before['revision'])
        config=json.loads((self.project/'config/mapping_live.json').read_text())
        self.assertNotIn('live_motion_calibration',config)
        current=self.project/config['live_motion_calibration_config']
        self.assertEqual(current.with_name(current.stem+'.evidence.json').read_bytes(),evidence.read_bytes())
        frozen_bias=self.project/config['gyro_bias_config']
        self.assertEqual(frozen_bias.read_bytes(),bias.read_bytes())
        self.assertTrue(frozen_bias.with_name(frozen_bias.stem+'.evidence.json').is_file())
        self.assertFalse(result['calibration_provenance']['physical_accuracy_validated'])
        from wc_panel.live_calibration import capabilities
        geometry={'geometry_capabilities':{'wheel_imu_motion_'+side:{'status':'AVAILABLE','reasons':[]} for side in ('all','left','right')}}
        with patch('wc_runtime.hardware_setup.resolve_hardware_setup',return_value=geometry), \
             patch('wc_runtime.calibration_geometry.verify_geometry_sources',return_value={'status':'PASS'}):
            enabled=capabilities(self.project)
        self.assertTrue(enabled['live_mapping']['enabled'])
        self.assertTrue(enabled['live_motion_correction']['enabled'])
        old_revision=result['revision']
        current.with_name(current.stem+'.evidence.json').write_text('{"tampered":true}')
        self.assertNotEqual(store.read()['revision'],old_revision)
        new_value=next(g['value'] for g in store.read()['groups'] if g['id']=='live_motion')
        with self.assertRaisesRegex(ValueError,'SHA-256'):verify_value(self.project,new_value)

    def test_live_import_rejects_session_candidate_bias_mismatch_and_bad_evidence(self):
        from wc_panel.live_calibration import load_declaration
        declaration,bias,evidence=self.calibration_fixture()
        original=json.loads(declaration.read_text())
        for mutation in ('scope','bias','hash'):
            value=copy.deepcopy(original)
            if mutation=='scope':value['scope']='SESSION_OFFLINE_ONLY'
            if mutation=='bias':value['confirmed_gyro_bias']['bias_native_rad_s']=[.002,0.,0.]
            if mutation=='hash':value['evidence_sha256']='0'*64
            atomic_json(declaration,value)
            with self.assertRaises(ValueError):load_declaration(self.project,declaration,bias,evidence)

    def test_missing_live_declaration_is_explicitly_disabled(self):
        backend=PanelBackend(self.project)
        row=backend.capabilities()['live_motion_correction']
        self.assertFalse(row['enabled']);self.assertTrue(row['reason'])
        self.assertEqual(set(row['by_sides']),{'all','left','right'})

    def test_warning_revision_backup_and_immutable_identity(self):
        store=ParameterStore(self.project,self.results/'backups')
        before=store.read();group=next(g for g in before['groups'] if g['id']=='wheels')
        changed=copy.deepcopy(group['value']);changed['wheel_radius_m']=.19
        with self.assertRaisesRegex(ValueError,'警告'):
            store.save('wheels',changed,expected_revision=before['revision'])
        proof=store.request_edit('wheels')
        original=(self.project/'config/hardware_setup.json').read_bytes()
        result=store.save('wheels',changed,proof['token'],before['revision'])
        self.assertEqual((Path(result['backup'])/'hardware_setup.json').read_bytes(),original)
        self.assertNotEqual(result['revision'],before['revision'])
        with self.assertRaisesRegex(ValueError,'配置已改变'):
            store.save('wheels',changed,proof['token'],before['revision'])
        with self.assertRaisesRegex(ValueError,'未知参数组'): store.request_edit('identity')

    def test_invalid_wheel_and_nan_fail_before_write(self):
        store=ParameterStore(self.project,self.results/'backups');before=store.read()
        group=next(g for g in before['groups'] if g['id']=='wheels')
        for invalid in (-1,float('nan')):
            changed=copy.deepcopy(group['value']);changed['wheel_radius_m']=invalid
            proof=store.request_edit('wheels')
            with self.assertRaises(ValueError):store.save('wheels',changed,proof['token'],before['revision'])
            self.assertEqual(store.read()['revision'],before['revision'])


class PlanTests(Fixture):
    def test_capture_keeps_manual_operation_and_explicit_read_only_test_mode(self):
        backend=PanelBackend(self.project)
        with patch('wc_panel.backend.resolve_user_destination',fake_resolve),patch.object(backend._manager,'submit',return_value={}) as submit:
            backend.start_capture(self.recordings,sides='left')
            task=submit.call_args.args[2][0]
            self.assertIn('--manual-drive',task['commands'][0])
            self.assertTrue(task['manual_drive'])
            self.assertEqual(len(task['progress_paths']),2)
            self.assertEqual(task['commands'][0][task['commands'][0].index('--sides')+1],'left')
            backend.start_capture(self.recordings,manual_drive=False)
            task=submit.call_args.args[2][0]
            self.assertNotIn('--manual-drive',task['commands'][0])
            self.assertFalse(task['manual_drive'])

    def test_offline_submission_exception_cannot_leave_queued_result(self):
        backend=PanelBackend(self.project)
        with patch('wc_panel.backend.resolve_user_destination',fake_resolve):
            plan=backend.plan_offline(self.dataset,self.results,{})
            with patch.object(backend._manager,'submit',side_effect=OSError('fixture log destination unavailable')):
                with self.assertRaises(OSError):backend.start_offline(plan)
        result=json.loads((Path(plan['output'])/'panel_result.json').read_text(encoding='utf-8'))
        self.assertEqual(result['status'],'FAILED')
        self.assertIn('fixture log destination',result['error'])

    def test_waiting_task_configuration_is_bound_to_recording_not_current_parameters(self):
        backend=PanelBackend(self.project)
        with patch('wc_panel.backend.resolve_user_destination',fake_resolve):
            plan=backend.plan_offline(self.dataset,self.results,{})
            config=json.loads((self.project/'config/hardware_setup.json').read_text(encoding='utf-8'))
            config['wheel_odometry']['wheel_radius_m']=.195
            atomic_json(self.project/'config/hardware_setup.json',config)
            self.assertEqual(backend._recorded_configuration(self.dataset),plan['recorded_configuration'])
            atomic_json(self.dataset/'runtime_config.json',{'mode':'left'})
            with self.assertRaisesRegex(ValueError,'录包配置'):backend.start_offline(plan)

    def test_full_plan_shares_candidates_per_cloud_and_never_changes_raw(self):
        backend=PanelBackend(self.project)
        original=(self.dataset/'capture_manifest.json').read_bytes()
        with patch('wc_panel.backend.resolve_user_destination',fake_resolve):
            plan=backend.plan_offline(self.dataset,self.results,{'all':True})
        self.assertEqual(plan['comparison_count'],22);self.assertEqual(plan['task_count'],24)
        candidates=[]
        for task in plan['tasks'][1:]:
            command=task['commands'][0]
            if '--motion-candidate' in command: candidates.append(command[command.index('--motion-candidate')+1])
        self.assertEqual(len(set(candidates)),2)
        self.assertFalse(Path(plan['output']).exists())
        self.assertEqual((self.dataset/'capture_manifest.json').read_bytes(),original)

    def test_modified_plan_cannot_execute_arbitrary_commands(self):
        backend=PanelBackend(self.project)
        with patch('wc_panel.backend.resolve_user_destination',fake_resolve):
            plan=backend.plan_offline(self.dataset,self.results,{})
            plan['tasks'][0]['commands']=[[sys.executable,'-c','raise Exception()']]
            with self.assertRaisesRegex(ValueError,'被更改'):backend.start_offline(plan)

    def test_all_variants_use_the_same_native_frame_cap_and_wall_interval(self):
        backend=PanelBackend(self.project)
        with patch('wc_panel.backend.resolve_user_destination',fake_resolve):
            plan=backend.plan_offline(self.dataset,self.results,{'all':True,'input_rate_hz':3.0})
        compare_count=refine_count=0
        for task in plan['tasks']:
            if task['variant'] is None:continue
            command=task['commands'][0]
            value=lambda flag:command[command.index(flag)+1]
            self.assertEqual(float(value('--input-rate-hz')),3.0)
            self.assertEqual(float(value('--native-wall-interval-s')),0.2)
            self.assertEqual(task['native_rate_hz'],3.0)
            self.assertEqual(task['native_wall_interval_s'],0.2)
            if task['variant']['geometry'] or task['variant']['process_noise']=='white_acceleration':
                refine_count+=1  # refine forwards its input_rate_hz as native_rate_hz.
            else:
                compare_count+=1
                self.assertEqual(float(value('--native-rate-hz')),3.0)
        self.assertEqual((compare_count,refine_count),(16,6))


class ProcessTests(Fixture):
    @unittest.skipIf(os.name=='nt','Detached worker and kernel queue lock require target Linux')
    def test_detached_queue_survives_gui_parent_exit_and_is_reattached(self):
        launcher=self.root/'launch_test.py'; receipt=self.root/'submitted.json'
        log=self.root/'execution_order.txt'
        # A separate short-lived process represents closing the GUI. Its two
        # detached workers must continue in submission order after it exits.
        program='from wc_panel.jobs import JobManager\nimport json,sys\nfrom pathlib import Path\n'
        program+='m=JobManager(Path(sys.argv[1]),Path(sys.argv[2]))\nids=[]\n'
        program+='for name in ("first","second"):\n'
        program+=' code="import time;from pathlib import Path;p=Path("+repr(sys.argv[4])+");f=p.open(\\"a\\");f.write("+repr(name+"\\n")+");f.close();time.sleep(.6)"\n'
        program+=' row=m.submit("offline",name,[dict(id=name,label=name,commands=[[sys.executable,"-c",code]])]);ids.append(row["id"])\n'
        program+='Path(sys.argv[3]).write_text(json.dumps(ids))\n'
        # Avoid shell interpolation: all generated paths are positional argv.
        launcher.write_text(program,encoding='utf-8')
        launched=subprocess.run([sys.executable,str(launcher),str(self.project),str(self.results/'detached_jobs'),str(receipt),str(log)],
                                capture_output=True,text=True,timeout=10)
        self.assertEqual(launched.returncode,0,launched.stderr)
        identifiers=json.loads(receipt.read_text())
        manager=JobManager(self.project,self.results/'detached_jobs')
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            rows=[manager.get(identifier) for identifier in identifiers]
            if all(row['status']=='COMPLETE' for row in rows):break
            if any(row['status']=='FAILED' for row in rows):
                self.fail(str(dict(jobs=rows,logs={identifier:manager.read_log(identifier)['text'] for identifier in identifiers})))
            time.sleep(.1)
        self.assertTrue(all(row['status']=='COMPLETE' for row in rows),
                        str(dict(jobs=rows,logs={identifier:manager.read_log(identifier)['text'] for identifier in identifiers})))
        self.assertEqual(log.read_text().splitlines(),['first','second'])

    def wait(self,manager,identifier):
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            row=manager.get(identifier)
            if row['status'] not in ('QUEUED','RUNNING','STOPPING'): return row
            time.sleep(.05)
        self.fail('owned process did not terminate')

    def test_process_failure_marks_remaining_tasks_skipped_and_preserves_log(self):
        manager=JobManager(self.project,self.results/'jobs')
        tasks=[dict(id='fail',label='失败命令',commands=[[sys.executable,'-c','print("evidence");raise SystemExit(7)']]),
               dict(id='next',label='不得运行',commands=[[sys.executable,'-c','print("NOT_RUN")']])]
        row=manager.submit('offline','fixture',tasks)
        done=self.wait(manager,row['id'])
        self.assertEqual(done['status'],'FAILED')
        self.assertEqual([s['status'] for s in done['task_states']],['FAILED','SKIPPED'])
        log=manager.read_log(row['id']);self.assertIn('evidence',log['text']);self.assertNotIn('NOT_RUN',log['text'])

    def test_successful_queue_has_terminal_task_states(self):
        manager=JobManager(self.project,self.results/'jobs')
        task=dict(id='ok',label='成功命令',commands=[[sys.executable,'-c','print("完成")']])
        first=manager.submit('offline','one',[task]);second=manager.submit('offline','two',[task])
        a=self.wait(manager,first['id']);b=self.wait(manager,second['id'])
        self.assertEqual((a['status'],b['status']),('COMPLETE','COMPLETE'))
        self.assertGreaterEqual(b['started_at'],a['started_at'])


if __name__=='__main__':unittest.main()
