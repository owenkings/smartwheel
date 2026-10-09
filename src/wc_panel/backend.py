"""GUI-independent panel service. All public boundaries use dictionaries."""
import copy
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import secrets
import sys
import threading
import time
from . import catalog
from .jobs import JobManager, ACTIVE
from .parameters import ParameterStore
from .storage import DataStore, atomic_json, ordinary, resolve_user_destination


class PanelBackend:
    def __init__(self, project_root):
        from wc_runtime.storage_policy import StoragePolicy
        self.project_root=ordinary(Path(project_root).absolute())
        self.policy=StoragePolicy(self.project_root)
        self.settings_file=self.project_root/'config/panel.local.json'
        self._lock=threading.RLock()
        self._plans={}
        report_root=self.policy.resolve('reports/panel')
        self._manager=JobManager(self.project_root,report_root/'jobs',log_guard=self.policy)
        self._parameters=ParameterStore(self.project_root,report_root/'parameter_revisions')
        from .config_profiles import ConfigProfiles
        self._profiles=ConfigProfiles(self._parameters)
        self._preparing_paths={}
        self._recover_profiles()
        from .calibration_tools import CalibrationTools
        self._calibration_tools=CalibrationTools(self.project_root)
        self._store=DataStore(self.project_root,self._active_data_paths)

    def _active_data_paths(self):
        with self._lock:
            return [*self._manager.active_paths(), *self._calibration_tools.active_paths(),
                    *(path for paths in self._preparing_paths.values() for path in paths)]

    def _recover_profiles(self):
        if (self._profiles.root/'.transaction.json').exists():
            self.policy.check()
            self._profiles.list()
            self.policy.check()

    def list_configuration_profiles(self):
        with self._lock:
            self.policy.check()
            result=self._profiles.list()
            self.policy.check()
            return result

    def save_configuration_profile(self,name,expected_revision):
        with self._lock:
            self.policy.check()
            result=self._profiles.save_current(name,expected_revision)
            self.policy.check()
            return result

    def preview_configuration_profile(self,identifier):
        with self._lock:
            self.policy.check()
            result=self._profiles.preview(identifier)
            self.policy.check()
            return result

    def apply_configuration_profile(self,token):
        with self._lock:
            active=[row for row in self.jobs() if row['kind'] in ('live','capture','recovery')
                    and row['status'] in ACTIVE]
            if active: raise ValueError('请先停止实时／录制任务并等待收尾完成，再切换设备配置。')
            self.policy.check()
            result=self._profiles.apply(token)
            self.policy.check()
            return result

    def prepare_pairing_capture(self,dataset):
        from .pairing_capture import prepare_recording
        with self._lock:
            source,guard=resolve_user_destination(self.project_root,dataset)
            if self._store.active(source): raise ValueError('录制仍在运行或收尾，请等待完成后准备配对点云。')
            identifier=self._identity('pairing')
            output=self.policy.resolve('reports/calibration')/identifier
            self._preparing_paths[identifier]=(source,output)
        try:
            guard.check();self.policy.check()
            result=prepare_recording(self.project_root,source,output)
            guard.check();self.policy.check()
            return result
        finally:
            with self._lock: self._preparing_paths.pop(identifier,None)

    def calibration_defaults(self): return self._calibration_tools.defaults()
    def start_calibration(self,input_path='',mode='points'): return self._calibration_tools.start(input_path,mode)
    def calibration_status(self,identifier=None): return self._calibration_tools.status(identifier)
    def stop_calibration(self,identifier): return self._calibration_tools.stop(identifier)

    def settings(self):
        if self.settings_file.exists():
            value=catalog.read_json(self.settings_file)
            if value.get('schema_version')!=1: raise ValueError('面板路径配置版本错误')
            return {key:value[key] for key in ('recording_root','results_root')}
        return dict(recording_root=str(self.policy.resolve('data/experiments')),
                    results_root=str(self.policy.resolve('data/analysis')))

    def update_settings(self,recording_root,results_root):
        roots=[]
        for path in (recording_root,results_root):
            root,guard=resolve_user_destination(self.project_root,path)
            guard.check();roots.append(root)
        if roots[0]==roots[1] or roots[0].is_relative_to(roots[1]) or roots[1].is_relative_to(roots[0]):
            raise ValueError('录制和结果根目录必须相互独立')
        value=dict(schema_version=1,recording_root=str(roots[0]),results_root=str(roots[1]),updated_at=time.time())
        atomic_json(self.settings_file,value)
        return self.settings()

    def capabilities(self):
        from .live_calibration import capabilities as live_capabilities
        mapping=catalog.read_json(self.project_root/'config/mapping_live.json')
        return dict(estimators=[dict(value='robot_localization',label='官方 EKF'),dict(value='five_state',label='五状态 EKF')],
            live_defaults=dict(estimator=mapping.get('wheel_imu_estimator','five_state'),cloud=mapping.get('cloud_source','filtered'),
                               process_noise=mapping.get('process_noise','legacy')),
            clouds=['raw','filtered'],sides=['all','left','right'],offline_variants=catalog.variants(),
            official_geometry=dict(enabled=False,reason='官方 EKF 的几何反馈尚未实现'),
            process_noise=['legacy','white_acceleration'],
            conditions=['运动修正候选必须来自同一次录制并通过校验',
                        'PSD/几何离线方案当前需要五状态、运动修正及点云筛选',
                        '未知外参仍会被运行时阻止；机械初值仅在显式选择后用于离线实验'],
            **live_capabilities(self.project_root))

    def list_recordings(self,root=None):
        path,guard=resolve_user_destination(self.project_root,root or self.settings()['recording_root'])
        rows=catalog.recordings(path);guard.check();return rows

    def list_results(self,root=None):
        path,guard=resolve_user_destination(self.project_root,root or self.settings()['results_root'])
        rows=catalog.results(path);guard.check();return rows

    def _ros_command(self,command):
        setup=os.environ.get('WHEELCHAIR_ROS_SETUP','/opt/ros/humble/setup.bash')
        return ['bash','--noprofile','--norc','-c',
            'source "$1" && source "$2/install/main/setup.bash" && export WHEELCHAIR_PROJECT_ROOT="$2" && '
            'export PYTHONPATH="$2/src${PYTHONPATH:+:$PYTHONPATH}" && shift 2 && exec "$@"',
            'wc_panel',setup,str(self.project_root),*map(str,command)]

    def _cli(self,operation,*arguments):
        return self._ros_command([sys.executable,'-s',str(self.project_root/'scripts/wc_phase1'),operation,*arguments])

    @staticmethod
    def _identity(prefix):
        return prefix+'_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(3)

    @staticmethod
    def _recorded_configuration(source):
        source=Path(source)
        names={'capture_manifest.json'}
        manifest=catalog.read_json(source/'capture_manifest.json') if (source/'capture_manifest.json').is_file() else {}
        names.update([manifest.get('runtime_config_path','runtime_config.json'),manifest.get('hardware_setup_path','hardware_setup.json')])
        names.update(name for name in manifest.get('input_hashes',{}) if name.startswith('configuration/'))
        files={}
        for name in sorted(names):
            relative=Path(name)
            if relative.is_absolute() or '..' in relative.parts:raise ValueError('录包配置路径越界')
            path=ordinary(source/relative)
            if path.is_file():files[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        return dict(basis='RECORDED_CONFIGURATION_ONLY',files=files,
                    note='算法开关取本次选择；硬件、轮参数和基础算法数值取录包内冻结配置，修改当前设备参数不会改变已排队的离线任务')

    def plan_offline(self,dataset,output_root=None,options=None):
        options=copy.deepcopy(options or {})
        source=ordinary(dataset)
        _,source_guard=resolve_user_destination(self.project_root,source)
        info=catalog.recording(source)
        if not info['complete'] and not options.get('allow_partial'):
            raise ValueError(info['reason'])
        destination,destination_guard=resolve_user_destination(self.project_root,output_root or self.settings()['results_root'])
        if source==destination or destination.is_relative_to(source) or source.is_relative_to(destination):
            raise ValueError('融合输出目录不能与原始录制目录重叠')
        selected=catalog.choose_variants(options)
        from wc_runtime.recording_sources import select_sources
        if info['source_inventory_error']: raise ValueError(info['source_inventory_error'])
        sides,selected_sides=select_sources(info['available_clouds_by_side'],options.get('sides'))
        if options.get('all'):
            selected=[variant for variant in selected if all(variant['cloud'] in info['available_clouds_by_side'][side]
                                                             for side in selected_sides)]
            if not selected:raise ValueError('所选雷达之间没有共同录制的点云类型，无法生成有效融合方案')
        for cloud in {variant['cloud'] for variant in selected}:
            select_sources(info['available_clouds_by_side'],sides,cloud=cloud)
        options['sides']=sides
        rate=float(options.get('input_rate_hz',5.0))
        if not 0<rate<=10: raise ValueError('离线输入频率必须在 (0,10] Hz')
        identifier=self._identity('fusion')
        output=destination/(source.name+'_'+identifier)
        tasks=[]
        common=['--dataset',str(source),'--project-root',str(self.project_root),'--sides',sides]
        if options.get('allow_partial'): common.append('--allow-partial')
        if options.get('mechanical_initial'): common.append('--mechanical-initial')
        # Raw and filtered packets can establish different first clock epochs.
        # Bind each candidate to its exact stream; EKFs using that stream share
        # the candidate without weakening verify_candidate's clock equality.
        motion_clouds=[cloud for cloud in ('raw','filtered')
                       if any(v['motion_correction'] and v['cloud']==cloud for v in selected)]
        candidates={cloud:output/('_motion_candidate' if len(motion_clouds)==1 else '_motion_candidate_'+cloud)/
                    'motion_candidate.json' for cloud in motion_clouds}
        for cloud,candidate in candidates.items():
            args=[sys.executable,'-s','-m','wc_panel.offline_worker',*common,'--cloud',cloud,
                  '--output',str(candidate.parent)]
            tasks.append(dict(id='motion_candidate' if len(motion_clouds)==1 else 'motion_candidate_'+cloud,
                              label='估计并校验本次录制的运动修正候选 · '+cloud,output=str(candidate.parent),
                              commands=[self._ros_command(args)],variant=None))
        for index,variant in enumerate(selected,1):
            target=output/('{:02d}_'.format(index)+variant['id'])
            args=[*common,'--output',str(target),'--cloud',variant['cloud'],'--input-rate-hz',str(rate),'--native-map','--user-data-paths']
            if variant['motion_correction']: args+=['--motion-candidate',str(candidates[variant['cloud']])]
            if variant['geometry'] or variant['process_noise']=='white_acceleration':
                args+=['--process-noise',variant['process_noise'],'--geometry','on' if variant['geometry'] else 'off',
                       '--native-cells','geometric_motion' if variant['geometry'] else 'calibrated_motion',
                       '--native-wall-interval-s','0.2']
                command=self._cli('refine',*args)
            else:
                args+=['--estimators',variant['estimator'],'--filter','on' if variant['filtering'] else 'off',
                       '--native-rate-hz',str(rate),'--native-wall-interval-s','0.2']
                command=self._cli('compare',*args)
            tasks.append(dict(id=variant['id'],label=variant['label'],variant=variant,output=str(target),commands=[command],
                              native_rate_hz=rate,native_wall_interval_s=0.2))
        plan=dict(id=identifier,dataset=str(source),output_root=str(destination),output=str(output),tasks=tasks,
                  task_count=len(tasks),comparison_count=len(selected),options=options,created_at=time.time(),
                  sides=sides,selected_sides=selected_sides,available_sides=info['available_sides'])
        source_guard.check();destination_guard.check()
        plan['recorded_configuration']=self._recorded_configuration(source)
        for task in tasks:task['input_configuration_hashes']=plan['recorded_configuration']['files']
        self._plans[identifier]=copy.deepcopy(plan)
        return plan

    def start_offline(self,plan):
        with self._lock:
            saved=self._plans.get(plan.get('id'))
            if saved is None or saved!=plan: raise ValueError('融合计划未知或被更改，请重新生成计划')
            source,sg=resolve_user_destination(self.project_root,plan['dataset'])
            if self._recorded_configuration(source)!=plan['recorded_configuration']:
                raise ValueError('录包配置在生成计划后改变，请重新检查并生成计划')
            destination,dg=resolve_user_destination(self.project_root,plan['output_root'])
            output=ordinary(plan['output'])
            if output.parent!=destination or output.exists(): raise ValueError('结果目录已存在，请重新生成计划')
            snapshot=self.device_parameters()
            sg.check();dg.check();self.policy.check()
            output.mkdir()
            atomic_json(output/'panel_result.json',dict(schema_version=1,status='QUEUED',dataset=str(source),plan=plan))
            try:
                label='离线融合 '+source.name+' · '+{'all':'双雷达','left':'仅左雷达','right':'仅右雷达'}[plan['sides']]
                row=self._manager.submit('offline',label,plan['tasks'],output=output,
                                         protected_paths=[source,output],guards=[sg,dg]+([self.policy] if self.policy.enabled else []),snapshot=snapshot)
            except Exception as error:
                try:
                    metadata=catalog.read_json(output/'panel_result.json')
                    metadata.update(status='FAILED',error=str(error))
                    atomic_json(output/'panel_result.json',metadata)
                except (OSError,ValueError):pass
                raise
            self._plans.pop(plan['id'])
            return row

    def start_capture(self,output_root=None,sides='all',*,manual_drive=True,name_options=None):
        with self._lock:
            self._recover_profiles()
            return self._start_capture(output_root,sides,manual_drive=manual_drive,name_options=name_options)

    def _start_capture(self,output_root=None,sides='all',*,manual_drive=True,name_options=None):
        if sides not in ('all','left','right'): raise ValueError('雷达选择必须是 all/left/right')
        if type(manual_drive) is not bool: raise ValueError('手动驾驶选项必须为布尔值')
        destination,guard=resolve_user_destination(self.project_root,output_root or self.settings()['recording_root'])
        session=self._identity('capture')
        if name_options is None:
            output=destination/session
        else:
            from wc_runtime.capture_names import recording_folder_name,available_folder
            output=available_folder(destination,recording_folder_name(name_options),self._manager.active_paths())
        arguments=['--profile','mapping_core','--session',session,'--duration','0','--staging','memory',
                   '--output-root',str(destination),'--preview','--sides',sides,'--preview-layout','unified']
        if name_options is not None: arguments+=['--folder-name',output.name]
        if manual_drive: arguments.append('--manual-drive')
        if guard.required_uuid: arguments+=['--required-output-uuid',guard.required_uuid]
        from wc_runtime.project_paths import capture_staging_root
        staging=capture_staging_root()/session
        task=dict(id=session,label='数据录制',output=str(output),commands=[self._cli('capture',*arguments)],
                  progress_paths=[str(staging/'progress.json'),str(output/'progress.json')],manual_drive=manual_drive,
                  folder_name=output.name,session_id=session)
        return self._manager.submit('capture','数据录制 '+output.name,[task],output=output,
            protected_paths=[output],guards=[guard],snapshot=self.device_parameters())

    def start_live(self,options):
        with self._lock:
            self._recover_profiles()
            return self._start_live(options)

    def _start_live(self,options):
        options=copy.deepcopy(options)
        mapping=catalog.read_json(self.project_root/'config/mapping_live.json')
        mode=options.get('mode','preview');sides=options.get('sides','all');cloud=options.get('cloud','filtered')
        estimator={'official':'robot_localization'}.get(options.get('estimator'),options.get('estimator',mapping.get('wheel_imu_estimator','five_state')))
        noise={'discrete':'legacy','continuous_psd':'white_acceleration'}.get(options.get('process_noise'),options.get('process_noise','legacy'))
        geometry=options.get('geometry',False);motion=options.get('motion_correction',False)
        if mode not in ('preview','mapping') or sides not in ('all','left','right') or cloud not in ('raw','filtered'):
            raise ValueError('实时模式、雷达或点云来源参数错误')
        if estimator not in ('robot_localization','five_state') or noise not in ('legacy','white_acceleration'):
            raise ValueError('未知估计器或过程噪声')
        if type(geometry) is not bool or type(motion) is not bool: raise ValueError('修正开关必须为布尔值')
        if estimator=='robot_localization' and geometry: raise ValueError('官方 EKF 的几何反馈尚未实现')
        if estimator=='robot_localization' and noise!='legacy': raise ValueError('连续噪声 PSD 当前仅适用于五状态 EKF')
        if mode=='preview' and (motion or geometry or noise!='legacy'):
            raise ValueError('预览模式不执行运动修正、连续噪声或几何纠偏，请选择实时建图')
        if mode=='mapping':
            mapping_capability=self.capabilities()['live_mapping']['by_sides'][sides]
            if not mapping_capability['enabled']:raise ValueError(mapping_capability['reason'])
        if motion or geometry:
            capability=self.capabilities()
            for enabled,key in ((motion,'live_motion_correction'),(geometry,'live_geometry')):
                if enabled:
                    row=capability[key]['by_sides'][sides]
                    if not row['enabled']:raise ValueError(row['reason'])
        session=self._identity('live')
        command=self._ros_command([sys.executable,'-s',self.project_root/'scripts/map',sides,
            '--mapping','true' if mode=='mapping' else 'false','--name',session,'--cloud',cloud,
            '--estimator',estimator,'--motion-correction','on' if motion else 'off','--process-noise',noise,
            '--geometry','on' if geometry else 'off','--panel-layout','unified','--save-dialog'])
        self.policy.check()
        task=dict(id=session,label='实时建图' if mode=='mapping' else '实时预览',commands=[command],output=None,accepted_exit_codes=[0,3])
        return self._manager.submit('live',task['label'],[task],guards=[self.policy],snapshot=self.device_parameters())

    def jobs(self): return self._manager.jobs()
    def list_recoverable_maps(self):
        from wc_runtime.map_save_app import session_handle
        root=self.policy.resolve('reports/maps')
        if not root.is_dir():return []
        rows=[]
        for directory in sorted(root.iterdir(),reverse=True):
            if not directory.is_dir() or directory.is_symlink() or not (directory/'session.json').is_file():continue
            try:
                session=catalog.read_json(directory/'session.json')
                if (session.get('kind')!='MAPPING_APP_EXPERIMENT' or session.get('project_root')!=str(self.project_root)
                        or session.get('mapping_enabled') is not True
                        or session.get('data_retention') not in ('TEMPORARY_UNTIL_USER_SAVE_CHOICE','experiment','map_only')):continue
                if (directory/'retention.json').is_file():
                    disposition=catalog.read_json(directory/'retention.json')
                    if disposition.get('saved_map') or disposition.get('raw_retention')=='DISCARDED':continue
                if self._store.active(directory):continue
                try:
                    request,handle=session_handle(directory,project_root=self.project_root)
                    rows.append(dict(id=session['session_id'],path=str(directory),status='SAVE_PENDING',
                                     recoverable=True,reason='设备已正常停止；可重新选择保存路径或废弃本次临时地图'))
                except (OSError,ValueError,RuntimeError,KeyError) as error:
                    rows.append(dict(id=session.get('session_id',directory.name),path=str(directory),status='RECOVERY_BLOCKED',
                                     recoverable=False,reason=str(error)))
            except (OSError,ValueError,KeyError):continue
        self.policy.check()
        return rows
    def recover_map(self,path):
        selected=ordinary(path)
        valid={row['path']:row for row in self.list_recoverable_maps()}
        if str(selected) not in valid:raise ValueError('仅允许处理本项目已停止、尚未保存的临时地图')
        if not valid[str(selected)]['recoverable']:raise ValueError(valid[str(selected)]['reason'])
        if self._store.active(selected):raise ValueError('该临时地图仍被活动进程使用')
        from wc_runtime.map_save_app import session_handle
        session_handle(selected,project_root=self.project_root)
        identifier=self._identity('recover')
        command=self._ros_command([sys.executable,'-s',self.project_root/'scripts/save_map',selected])
        task=dict(id=identifier,label='处理临时地图 '+selected.name,commands=[command],output=str(selected),accepted_exit_codes=[0,3])
        return self._manager.submit('recovery',task['label'],[task],output=selected,protected_paths=[selected],guards=[self.policy])
    def cancel(self,job_id): return self._manager.cancel(job_id)
    def list_logs(self,job_id): return self._manager.list_logs(job_id)
    def read_log(self,job_id,offset=0,log_id=None): return self._manager.read_log(job_id,offset,log_id)
    def device_parameters(self):
        with self._lock:
            self._recover_profiles()
            return self._parameters.read()
    def load_live_calibration(self,calibration_path,gyro_bias_config,calibration_evidence_path=None):
        from .live_calibration import load_declaration
        return load_declaration(self.project_root,calibration_path,gyro_bias_config,calibration_evidence_path)
    def request_parameter_edit(self,group):
        with self._lock: return self._parameters.request_edit(group)
    def prepare_extrinsic_evidence(self,path,evidence_type,assembly_revision,note,expected_revision):
        return self._parameters.prepare_extrinsic_evidence(path,evidence_type,assembly_revision,note,expected_revision)
    def save_parameters(self,group,value,warning_token=None,expected_revision=None,evidence_tokens=None):
        with self._lock:
            self._recover_profiles()
            self.policy.check()
            return self._parameters.save(group,value,warning_token,expected_revision,evidence_tokens)
    def scan_storage(self,root): return self._store.scan(root)
    def prepare_delete(self,root,paths):
        if str(ordinary(root)) not in self.settings().values(): raise ValueError('只能清理当前设置的录制或融合结果根目录')
        return self._store.prepare_delete(root,paths)
    def execute_delete(self,token):
        with self._lock:
            plan=self._store.tokens.get(token)
            if plan is not None and plan[0]['root'] not in self.settings().values(): raise ValueError('默认路径已改变，请重新查看删除清单')
            return self._store.execute_delete(token)
