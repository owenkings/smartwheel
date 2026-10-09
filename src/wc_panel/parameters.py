"""Validated, revision-checked configuration edits with original-byte backups."""
import copy
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import stat
import threading
import time
from .storage import atomic_json, ordinary

FILES = ('hardware_setup.json', 'mapping_live.json', 'cameras.json')
ALGORITHM_FIELDS = ('cloud_filter', 'planar_ekf', 'map_profile', 'input_rate_hz', 'max_pair_delta_ns', 'wheel_imu_covariance','wheel_imu_estimator')
WHEEL_NUMBERS = ('wheel_radius_m', 'track_width_m', 'register_to_wheel_rpm', 'left_sign', 'right_sign', 'max_gap_s', 'max_wheel_speed_m_s')
EXTRINSIC_STATUSES = ('UNKNOWN', 'CAD_NOMINAL', 'LEGACY_CANDIDATE', 'USER_MEASURED_EXPERIMENT', 'CALIBRATED')
MAX_EVIDENCE_BYTES = 16 * 1024 * 1024


def algorithm_values(mapping):
    return dict({key:mapping[key] for key in ALGORITHM_FIELDS if key in mapping},
                wheel_imu_estimator=mapping.get('wheel_imu_estimator','five_state'))


class ParameterStore:
    def __init__(self, project_root, backup_root):
        self.project = ordinary(Path(project_root).absolute())
        self.backup_root = Path(backup_root)
        self.tokens = {}
        self.evidence_tokens = {}
        self.lock = threading.RLock()

    def selected_paths(self):
        from wc_runtime.project_config import selected_config_path
        return {name: ordinary(selected_config_path(self.project, name)) for name in FILES}

    def identity_paths(self):
        from wc_runtime.project_config import selected_config_path
        paths = {}
        for name in ('device_bindings.json', 'device_bindings.local.json', 'wheel_feedback_current.json'):
            path = ordinary(self.project / 'config' / name)
            if name == 'wheel_feedback_current.json':
                local = ordinary(self.project / 'config/local/project' / name)
                if local.exists() or path.exists():
                    path = selected_config_path(self.project, name)
            paths[name] = path
        return paths

    def path_identity(self, paths=None):
        paths = self.selected_paths() if paths is None else paths
        return {name: paths[name].relative_to(self.project).as_posix() for name in FILES}

    def require_paths(self, paths):
        if self.selected_paths() != paths:
            raise ValueError('配置来源路径已改变，请刷新；保留备份和未完成事务')

    def require_bytes(self, paths, expected):
        self.require_paths(paths)
        if any(ordinary(paths[name]).read_bytes() != expected[name] for name in FILES):
            raise ValueError('配置在写入期间改变，保留备份并停止修改')

    @staticmethod
    def _write_config(path, raw):
        path = ordinary(path)
        mode = stat.S_IMODE(path.stat().st_mode)
        temporary = path.with_name('.panel_write_' + secrets.token_hex(8))
        try:
            with temporary.open('xb') as stream:
                os.chmod(temporary, mode)
                stream.write(raw); stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if temporary.exists(): temporary.unlink()

    def _snapshot(self):
        paths = self.selected_paths()
        raw = {name: ordinary(paths[name]).read_bytes() for name in FILES}
        configs={name: json.loads(data.decode('utf-8')) for name,data in raw.items()}
        digest=hashlib.sha256(json.dumps(self.path_identity(paths), sort_keys=True).encode())
        digest.update(b''.join(name.encode()+raw[name] for name in FILES))
        for name, path in self.identity_paths().items():
            digest.update(path.relative_to(self.project).as_posix().encode())
            digest.update(path.read_bytes() if path.is_file() else b':ABSENT')
        # An edited/missing evidence snapshot invalidates an open edit session,
        # just as changing hardware_setup.json does.
        for source in configs['hardware_setup.json'].get('source_documents', []):
            name = source.get('snapshot_path', '')
            digest.update(name.encode())
            try:
                path = ordinary(self.project/name)
                if path.stat().st_size > MAX_EVIDENCE_BYTES: raise ValueError('证据文件过大')
                digest.update(path.read_bytes())
            except (OSError, ValueError): digest.update(b':UNAVAILABLE')
        from .live_calibration import config_path
        for field in ('gyro_bias_config','live_motion_calibration_config'):
            value=configs['mapping_live.json'].get(field)
            if value:
                try:
                    path=config_path(self.project,value)
                    for selected in (path,path.with_name(path.stem+'.evidence.json')):
                        digest.update(str(selected).encode());digest.update(selected.read_bytes())
                except (OSError,ValueError):digest.update((field+':UNAVAILABLE').encode())
        revision=digest.hexdigest()
        self.require_paths(paths)
        return raw, configs, revision, paths

    def _read(self):
        return self._snapshot()[:3]

    def read(self):
        raw, configs, revision, paths = self._snapshot()
        setup, mapping, cameras = (configs[name] for name in FILES)
        identity = {}
        for name, path in self.identity_paths().items():
            if path.is_file(): identity[name] = json.loads(path.read_text(encoding='utf-8'))
        from .live_calibration import current_value
        try:live_value=current_value(self.project,mapping);live_error=''
        except (OSError,ValueError,KeyError) as error:
            live_value=dict(calibration=None,gyro_bias_config=mapping.get('gyro_bias_config') or '',calibration_evidence_path='')
            live_error=str(error)
        groups = [
            dict(id='extrinsics',label='设备外参',value=setup.get('data_transforms', []),warning_required=True,path=str(paths['hardware_setup.json']),
                 assembly_revision=setup.get('assembly', {}).get('revision'),
                 source_documents=copy.deepcopy(setup.get('source_documents', [])),allowed_statuses=list(EXTRINSIC_STATUSES)),
            dict(id='wheels',label='轮参数',value={key:setup['wheel_odometry'][key] for key in WHEEL_NUMBERS},warning_required=True,path=str(paths['hardware_setup.json']),
                 status=setup['wheel_odometry'].get('status','UNVALIDATED'),
                 provenance=copy.deepcopy(setup['wheel_odometry'].get('provenance', {})),
                 formal_odometry_eligible=setup['wheel_odometry'].get('formal_odometry_eligible', False)),
            dict(id='display',label='显示设置',value=dict(visualization=setup.get('visualization',{}),
                 camera_rotations={row['role']:row['rotate_deg'] for row in cameras['cameras']},
                 camera_profile=cameras['default_profile'],rviz_renderer=mapping.get('rviz_renderer','software')),warning_required=False,path=str(paths['cameras.json'])),
            dict(id='algorithms',label='算法配置',value=algorithm_values(mapping),warning_required=False,path=str(paths['mapping_live.json'])),
            dict(id='live_motion',label='在线运动校正',value=live_value,warning_required=True,
                 path=str(paths['mapping_live.json']),error=live_error,
                 calibration_scope='EXPLICIT_DEVICE_DECLARATION',physical_accuracy_validated=False)]
        for group in groups:
            group['effective'] = 'next_task'
            group['conditions'] = '保存后下次任务生效；当前任务继续使用启动时的冻结配置。外参可用性仍由现有校验器判断。'
        return dict(revision=revision, identity=identity, groups=groups,
                    installation=copy.deepcopy(setup))

    def request_edit(self, group):
        if group not in ('extrinsics','wheels','display','algorithms','live_motion'): raise ValueError('未知参数组')
        revision = self._read()[2]
        token = secrets.token_urlsafe(32)
        self.tokens[token] = dict(group=group,revision=revision,expires=time.time()+900)
        message=('在线校正必须来自已确认的当前设备声明和独立陀螺零偏依据。请勿将某次录包的离线候选直接声明为设备校正；文件校验不等于物理精度验证。'
                 if group=='live_motion' else '如非必要，请不要随意更改外参或轮参数。错误设置会影响融合、地图尺度和方向；请确认已有测量依据。')
        return dict(token=token,revision=revision,message=message)

    def prepare_extrinsic_evidence(self, path, evidence_type, assembly_revision, note, expected_revision):
        """Preview a user-declared measurement; do not promote any transform.

        The opaque token is revision-bound and expires with the editing session.
        Its original bytes are frozen only when the user saves the parameters.
        """
        with self.lock:
            _, configs, revision = self._read()
            if expected_revision != revision: raise ValueError('配置已改变，请重新打开参数窗口')
            setup = configs['hardware_setup.json']
            if assembly_revision != setup.get('assembly', {}).get('revision'):
                raise ValueError('测量依据必须明确绑定当前装配版本')
            if evidence_type not in ('GEOMETRY_MEASUREMENT', 'EXTRINSIC_CALIBRATION'):
                raise ValueError('请选择实测记录或外参标定记录类型')
            if not isinstance(note, str) or not note.strip() or len(note) > 4096:
                raise ValueError('请填写测量方法、所支持的数据坐标关系及适用条件（最多4096字符）')
            selected = ordinary(path)
            if not selected.is_file() or not 0 < selected.stat().st_size <= MAX_EVIDENCE_BYTES:
                raise ValueError('依据必须是非空普通文件，大小不超过16 MiB')
            with selected.open('rb') as stream: raw = stream.read(MAX_EVIDENCE_BYTES + 1)
            if not raw or len(raw) > MAX_EVIDENCE_BYTES: raise ValueError('依据文件大小改变或超过16 MiB')
            # JSON measurements are parsed by the runtime verifier as JSON too.
            if selected.suffix.lower() == '.json': json.loads(raw.decode('utf-8-sig'))
            digest = hashlib.sha256(raw).hexdigest()
            identifier = 'panel_' + secrets.token_hex(12)
            suffix = selected.suffix.lower()
            if len(suffix) > 16 or not all(c.isalnum() or c == '.' for c in suffix): suffix = '.bin'
            source = dict(id=identifier, snapshot_path='config/calibration/panel_extrinsics/' + identifier + suffix,
                          sha256=digest, evidence_type=evidence_type, assembly_revision=assembly_revision,
                          note=note.strip(), original_path=str(selected))
            token = secrets.token_urlsafe(32)
            self.evidence_tokens = {key: item for key, item in self.evidence_tokens.items() if item['expires'] >= time.time()}
            if len(self.evidence_tokens) >= 32: raise ValueError('本次待导入依据过多，请先保存或重新打开面板')
            self.evidence_tokens[token] = dict(source=source, revision=revision, expires=time.time()+900)
            return dict(token=token, source=copy.deepcopy(source), revision=revision,
                        physical_accuracy_validated=False, status='PENDING_PARAMETER_SAVE')

    @staticmethod
    def _verify_sources(setup, project):
        if setup.get('schema_version') != 2: return
        from wc_runtime.calibration_geometry import verify_geometry_sources
        proof = verify_geometry_sources(setup, project)
        if proof.get('status') != 'PASS':
            failures = [str(row.get('id')) + ': ' + str(row.get('reason')) for row in proof.get('sources', []) if row.get('status') != 'PASS']
            raise ValueError('外参来源文件校验失败（缺失、SHA-256改变或记录不一致）' + ('：' + '；'.join(failures) if failures else ''))

    def save(self, group, value, warning_token=None, expected_revision=None, evidence_tokens=None):
        with self.lock:
            raw, configs, revision, paths = self._snapshot()
            if expected_revision is None or expected_revision != revision:
                raise ValueError('配置已改变或缺少版本信息，请重新打开参数窗口')
            if group in ('extrinsics','wheels','live_motion'):
                proof = self.tokens.get(warning_token)
                if not proof or proof['group']!=group or proof['revision']!=revision or proof['expires']<time.time():
                    raise ValueError('请先阅读并确认外参/轮参数修改警告')
            setup, mapping, cameras = (configs[name] for name in FILES)
            changed = []
            live_verification=None
            pending_evidence=[]
            if evidence_tokens and group != 'extrinsics': raise ValueError('测量依据只能用于外参参数组')
            if evidence_tokens is not None and (not isinstance(evidence_tokens, list) or
                    any(not isinstance(token, str) for token in evidence_tokens) or len(set(evidence_tokens)) != len(evidence_tokens)):
                raise ValueError('测量依据令牌列表无效')
            if group == 'extrinsics':
                before = setup.get('data_transforms', [])
                if not isinstance(value,list) or len(value)!=len(before): raise ValueError('不能添加、删除或重绑定设备外参身份')
                for original, proposed in zip(before,value):
                    for key in ('id','parent_frame','child_frame','direction','unit'):
                        if proposed.get(key)!=original.get(key): raise ValueError('外参身份、方向和单位只读: '+key)
                    # Structural/evidence validation is performed by the runtime
                    # resolver; the UI must retain source evidence and unknowns.
                self._verify_sources(configs['hardware_setup.json'], self.project)
                setup['data_transforms']=copy.deepcopy(value)
                for token in evidence_tokens or []:
                    evidence = self.evidence_tokens.get(token)
                    if not evidence or evidence['revision'] != revision or evidence['expires'] < time.time():
                        raise ValueError('测量依据导入已过期或配置改变，请重新导入')
                    source = evidence['source']
                    references = [item.get('source_id') for transform in value for component in ('translation', 'rotation')
                                  for item in transform.get(component, {}).get('evidence', [])]
                    if source['id'] not in references: raise ValueError('导入的测量依据尚未绑定具体外参分量')
                    path = ordinary(source['original_path'])
                    with path.open('rb') as stream: data = stream.read(MAX_EVIDENCE_BYTES+1)
                    if len(data) > MAX_EVIDENCE_BYTES or hashlib.sha256(data).hexdigest() != source['sha256']:
                        raise ValueError('待导入的测量依据文件已改变，请重新导入')
                    setup['source_documents'].append(copy.deepcopy(source))
                    pending_evidence.append((source, data))
                changed=['hardware_setup.json']
            elif group == 'wheels':
                if not isinstance(value,dict) or set(value)!=set(WHEEL_NUMBERS): raise ValueError('轮参数字段不匹配')
                for key,number in value.items():
                    if type(number) not in (int,float) or not math.isfinite(number): raise ValueError('轮参数必须为有限数值')
                    if key.endswith('_sign') and number not in (-1,1): raise ValueError('左右轮符号必须为 -1 或 1')
                    if not key.endswith('_sign') and number<=0: raise ValueError('轮参数必须大于零')
                setup['wheel_odometry'].update(value)
                changed=['hardware_setup.json']
            elif group == 'display':
                if not isinstance(value,dict) or set(value)!= {'visualization','camera_rotations','camera_profile','rviz_renderer'}:
                    raise ValueError('显示配置字段不匹配')
                if value['camera_profile'] not in cameras['profiles']: raise ValueError('未知相机显示配置')
                if value['rviz_renderer'] not in ('software','system'): raise ValueError('渲染器必须是 software 或 system')
                rotations=value['camera_rotations']
                if set(rotations)!={row['role'] for row in cameras['cameras']}: raise ValueError('相机身份不可修改')
                for row in cameras['cameras']:
                    rotation=rotations[row['role']]
                    if type(rotation) is not int or rotation not in (0,90,180,270): raise ValueError('相机旋转必须为 0/90/180/270 度')
                    row['rotate_deg']=rotation
                cameras['default_profile']=value['camera_profile']
                setup['visualization']=copy.deepcopy(value['visualization'])
                mapping['rviz_renderer']=value['rviz_renderer']
                changed=list(FILES)
            elif group == 'algorithms':
                before=algorithm_values(mapping)
                if set(value)!=set(before): raise ValueError('算法配置字段不匹配')
                if value['wheel_imu_estimator'] not in ('five_state','robot_localization'):raise ValueError('估计器必须为 five_state 或 robot_localization')
                self._validate_numeric_structure({key:before[key] for key in value if key!='wheel_imu_estimator'},
                                                 {key:value[key] for key in value if key!='wheel_imu_estimator'})
                if not 0<float(value.get('input_rate_hz',5))<=5: raise ValueError('在线输入频率必须在 (0,5] Hz；离线频率请在离线融合窗口选择')
                pair_delta=value.get('max_pair_delta_ns',1)
                if type(pair_delta) is not int or not 0<=pair_delta<=50_000_000: raise ValueError('配对时间阈值必须为 0..50000000 ns 的整数')
                for key in ('planar_ekf','wheel_imu_covariance'):
                    self._positive_weights(value.get(key,{}))
                mapping.update(copy.deepcopy(value))
                from wc_runtime.mapping_filter import resolve_cloud_filter
                from wc_runtime.mapping_planar import validate_planar_config
                from wc_runtime.mapping_profile import validate_profile
                from wc_runtime.mapping_odometry import validate_covariance
                resolve_cloud_filter(mapping)
                validate_planar_config(mapping.get('planar_ekf'))
                validate_profile(mapping.get('map_profile'))
                validate_covariance(mapping.get('wheel_imu_covariance'))
                changed=['mapping_live.json']
            elif group=='live_motion':
                from .live_calibration import verify_value
                live_verification=verify_value(self.project,value)
                changed=['mapping_live.json']
            else: raise ValueError('未知参数组')
            from wc_runtime.hardware_setup import resolve_hardware_setup
            resolve_hardware_setup(setup)
            # JSON allow_nan=False also rejects hidden NaNs in nested structures.
            for name in changed: json.dumps(configs[name],allow_nan=False)
            if self._read()[2] != revision:
                raise ValueError('配置在校验过程中改变，请刷新后重试')
            self.require_paths(paths)
            backup=self.backup_root/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(4))
            backup.mkdir(parents=True,exist_ok=False)
            for name in changed:
                with (backup/name).open('xb') as stream: stream.write(raw[name])
            atomic_json(backup/'revision.json',dict(schema_version=2,source_paths=self.path_identity(paths),group=group,before_revision=revision,changed_files=changed,time=time.time()))
            written_evidence=[]
            try:
                for source, data in pending_evidence:
                    destination=ordinary(self.project/source['snapshot_path'])
                    destination.parent.mkdir(parents=True,exist_ok=True)
                    with destination.open('xb') as stream:
                        written_evidence.append(destination)
                        stream.write(data);stream.flush();os.fsync(stream.fileno())
                if group == 'extrinsics': self._verify_sources(setup, self.project)
            except Exception:
                for path in written_evidence: path.unlink()
                raise
            if live_verification is not None:
                destination=self.project/'config/calibration/panel_live'/backup.name
                destination.mkdir(parents=True,exist_ok=False)
                for name,data in live_verification['raw_files'].items():
                    with (destination/name).open('xb') as stream:
                        stream.write(data);stream.flush()
                        os.fsync(stream.fileno())
                atomic_json(destination/'live_motion_calibration.json',live_verification['value']['calibration'])
                atomic_json(destination/'provenance.json',live_verification['metadata'])
                mapping['live_motion_calibration_config']=(destination/'live_motion_calibration.json').relative_to(self.project).as_posix()
                mapping['gyro_bias_config']=(destination/'confirmed_gyro_bias.json').relative_to(self.project).as_posix()
                mapping.pop('live_motion_calibration',None)
                mapping.pop('confirmed_gyro_bias',None)
            encoded = {name: (json.dumps(configs[name], ensure_ascii=False, indent=2, allow_nan=False) + chr(10)).encode('utf-8')
                       for name in changed}
            expected = dict(raw)
            try:
                for name in changed:
                    self.require_bytes(paths, expected)
                    self._write_config(paths[name], encoded[name])
                    expected[name] = encoded[name]
                self.require_bytes(paths, expected)
            except Exception:
                # Never follow a newly selected local/base path during rollback.
                self.require_paths(paths)
                for name in changed:
                    if ordinary(paths[name]).read_bytes() not in (raw[name], encoded[name]):
                        raise RuntimeError('配置存在其他修改，未覆盖；原始备份：' + str(backup))
                for name in changed:
                    self.require_paths(paths)
                    self._write_config(paths[name], raw[name])
                for path in written_evidence: path.unlink()
                raise
            if warning_token in self.tokens: self.tokens.pop(warning_token)
            for token in evidence_tokens or []: self.evidence_tokens.pop(token, None)
            new_revision=self._read()[2]
            atomic_json(backup/'revision.json',dict(schema_version=2,source_paths=self.path_identity(paths),group=group,before_revision=revision,after_revision=new_revision,
                changed_files=changed,time=time.time(),effective='next_task'))
            return dict(revision=new_revision,backup=str(backup),effective='next_task',
                        **({'calibration_provenance':live_verification['metadata']} if live_verification else {}))

    @staticmethod
    def _validate_numeric_structure(before,after):
        if isinstance(before,dict):
            if not isinstance(after,dict) or set(before)!=set(after): raise ValueError('算法配置结构不能改变')
            for key in before: ParameterStore._validate_numeric_structure(before[key],after[key])
        elif isinstance(before,list):
            if not isinstance(after,list) or len(before)!=len(after): raise ValueError('算法数组尺寸不能改变')
            for a,b in zip(before,after): ParameterStore._validate_numeric_structure(a,b)
        elif type(before) is bool:
            if type(after) is not bool: raise ValueError('开关必须为布尔值')
        elif type(before) in (int,float):
            if type(after) not in (int,float) or not math.isfinite(after): raise ValueError('算法参数必须为有限数值')
        elif before!=after: raise ValueError('算法状态、模式和单位字段只读')

    @staticmethod
    def _positive_weights(value):
        if isinstance(value,dict):
            for item in value.values(): ParameterStore._positive_weights(item)
        elif isinstance(value,list):
            for item in value: ParameterStore._positive_weights(item)
        elif type(value) in (int,float) and value<=0: raise ValueError('噪声和协方差参数必须大于零')
