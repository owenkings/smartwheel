"""Evidence-checked device declarations shared by import, save and capability UI.

The device scope is an explicit operator declaration. A matching hash establishes
provenance, not the physical accuracy of the correction or stationarity.
"""
import copy
import hashlib
import json
from pathlib import Path
from .storage import ordinary


def read_document(path,limit=262144):
    path=ordinary(Path(path).absolute())
    if not path.is_file() or path.stat().st_size>limit:
        raise ValueError('校正文件缺失或过大: '+str(path))
    raw=path.read_bytes()
    if len(raw)>limit:raise ValueError('校正文件读取时超出大小上限')
    value=json.loads(raw.decode('utf-8'),parse_constant=lambda value:(_ for _ in ()).throw(ValueError('不允许非有限JSON数值')))
    if not isinstance(value,dict):raise ValueError('校正声明和依据必须为 JSON 对象')
    return raw,value


def config_path(project,value):
    if isinstance(value,Path):value=str(value)
    if not isinstance(value,str) or not value:raise ValueError('尚未选择校正配置文件')
    path=Path(value)
    if not path.is_absolute():
        from wc_runtime.storage_policy import StoragePolicy
        path=StoragePolicy(project).resolve(path)
    path = ordinary(path)
    from wc_runtime.project_config import PROJECT_CONFIG_NAMES, selected_config_path
    for name in PROJECT_CONFIG_NAMES:
        if path == Path(project).absolute() / 'config' / name:
            return selected_config_path(project, name)
    return path


def device_context(project,mapping=None):
    from wc_runtime.device_bindings import load_device_bindings
    project=Path(project)
    if mapping is None:
        from wc_runtime.project_config import selected_config_path
        _,mapping=read_document(selected_config_path(project, 'mapping_live.json'))
    bindings=load_device_bindings(project)
    hardware_path=config_path(project,mapping.get('wheel_hardware_config','config/wheel_feedback_current.json'))
    _,hardware=read_document(hardware_path)
    from wc_motion.feedback_transport import validate_config
    validate_config(hardware)
    imu=bindings['imu']['sensor_id']
    if mapping.get('imu_sensor_id') not in (None,imu):raise ValueError('算法配置的 IMU 身份与当前设备绑定不一致')
    return dict(imu_sensor_id=imu,wheel_device_id=hardware['device_id'],motion_model=mapping.get('motion_model'),
                continuous_mapping=mapping.get('continuous_mapping'),mapping=mapping)


def verify_value(project,value,*,context=None):
    """Read both evidence files again; no writes or automatic scope conversion."""
    if not isinstance(value,dict) or set(value)!={'calibration','gyro_bias_config','calibration_evidence_path'}:
        raise ValueError('在线校正需要声明、独立零偏确认文件和声明依据文件')
    context=context or device_context(project)
    bias_path=config_path(project,value['gyro_bias_config'])
    bias_raw,bias=read_document(bias_path,32768)
    bias_evidence_path=bias_path.with_name(bias_path.stem+'.evidence.json')
    bias_evidence_raw,bias_evidence=read_document(bias_evidence_path)
    from wc_runtime.mapping_planar import validate_confirmed_bias
    normalized_bias=validate_confirmed_bias(bias,expected_sensor_id=context['imu_sensor_id'])
    if hashlib.sha256(bias_evidence_raw).hexdigest()!=normalized_bias['evidence_sha256']:
        raise ValueError('独立陀螺零偏依据 SHA-256 与确认文件不一致')
    from wc_runtime.live_motion_options import validate_live_calibration
    calibration=validate_live_calibration(value['calibration'],context)
    if calibration['confirmed_gyro_bias']!=normalized_bias:
        raise ValueError('设备声明中的陀螺零偏与所选独立确认文件不一致；不能重复或替换校正')
    evidence_path=config_path(project,value['calibration_evidence_path'])
    evidence_raw,evidence=read_document(evidence_path)
    if hashlib.sha256(evidence_raw).hexdigest()!=calibration['evidence_sha256'].lower():
        raise ValueError('设备校正依据 SHA-256 与设备声明不一致')
    calibration['evidence_sha256']=calibration['evidence_sha256'].lower()
    return dict(value=dict(calibration=calibration,gyro_bias_config=str(bias_path),calibration_evidence_path=str(evidence_path)),
        raw_files={'confirmed_gyro_bias.json':bias_raw,'confirmed_gyro_bias.evidence.json':bias_evidence_raw,
                   'live_motion_calibration.evidence.json':evidence_raw},
        metadata=dict(device_scope='EXPLICIT_OPERATOR_DECLARATION',physical_accuracy_validated=False,
            stationarity_automatically_inferred=False,calibration_evidence_sha256=hashlib.sha256(evidence_raw).hexdigest(),
            bias_evidence_sha256=hashlib.sha256(bias_evidence_raw).hexdigest(),
            source_gyro_bias_config=str(bias_path),source_calibration_evidence_path=str(evidence_path)))


def load_declaration(project,calibration_path,gyro_bias_config,calibration_evidence_path=None):
    declaration_path=config_path(project,calibration_path)
    _,calibration=read_document(declaration_path,32768)
    evidence_path=calibration_evidence_path or str(declaration_path.with_name(declaration_path.stem+'.evidence.json'))
    value=dict(calibration=calibration,gyro_bias_config=str(gyro_bias_config),calibration_evidence_path=str(evidence_path))
    return verify_value(project,value)['value']


def current_value(project,mapping):
    context=device_context(project,mapping)
    declared=mapping.get('live_motion_calibration_config')
    if declared:
        path=config_path(project,declared)
        _,calibration=read_document(path,32768)
        return dict(calibration=calibration,gyro_bias_config=mapping.get('gyro_bias_config') or '',
                    calibration_evidence_path=str(path.with_name(path.stem+'.evidence.json')))
    # These unknowns are intentionally not numerical defaults. Runtime import
    # requires a full declaration plus independent bias and evidence files.
    return dict(calibration=dict(schema_version=1,status='EXPLICIT_DEVICE_CALIBRATION_DECLARATION',scope='device',
        imu_sensor_id=context['imu_sensor_id'],wheel_device_id=context['wheel_device_id'],R_reference_imu=None,
        wheel_yaw_scale=None,wheel_yaw_speed_coefficient=None,confirmed_gyro_bias=None,evidence_id='',evidence_sha256=''),
        gyro_bias_config=mapping.get('gyro_bias_config') or '',calibration_evidence_path='')


def capabilities(project):
    by_sides={side:dict(enabled=False,reason='当前外参与算法条件尚未满足') for side in ('all','left','right')}
    geometry_checked=False
    motion=dict(enabled=False,reason='尚未配置独立设备校正声明',by_sides=copy.deepcopy(by_sides))
    geometry=dict(enabled=False,reason='几何纠偏仅支持五状态实时建图',by_sides=copy.deepcopy(by_sides),estimator='five_state')
    try:
        context=device_context(project);mapping=context['mapping']
        from wc_runtime.hardware_setup import resolve_hardware_setup
        from wc_runtime.calibration_geometry import verify_geometry_sources
        _,setup=read_document(config_path(project,mapping.get('hardware_setup_config','config/hardware_setup.json')))
        resolved=resolve_hardware_setup(setup)
        if setup.get('schema_version')==2:
            proof=verify_geometry_sources(setup,project)
            if proof.get('status')!='PASS':raise ValueError('外参来源文件尚未通过验证')
        base_reason='' if mapping.get('continuous_mapping') is True and mapping.get('motion_model')=='planar_ekf' else '需要连续 planar_ekf 建图配置'
        for side in by_sides:
            row=resolved['geometry_capabilities']['wheel_imu_motion_'+side]
            reason=base_reason or ('；'.join(row.get('reasons',[])) if row['status']!='AVAILABLE' else '')
            by_sides[side]=dict(enabled=not reason,reason=reason)
        geometry_checked=True
        geometry.update(by_sides=copy.deepcopy(by_sides),enabled=any(row['enabled'] for row in by_sides.values()))
        geometry['reason']='仅在五状态实时建图生效；不反馈更新官方 EKF' if geometry['enabled'] else by_sides['all']['reason']
        if mapping.get('live_motion_calibration') is not None:
            raise ValueError('在线设备声明必须来自 live_motion_calibration_config 文件及同名依据，不能手填 inline 声明')
        if not mapping.get('live_motion_calibration_config'):
            raise ValueError('请在“设备参数 → 在线运动校正”导入独立设备声明及零偏确认文件')
        verified=verify_value(project,current_value(project,mapping),context=context)
        from wc_runtime.live_calibration_files import load_live_calibration_files
        load_live_calibration_files(project,dict(mapping,motion_correction=True,
            imu_sensor_id=context['imu_sensor_id'],wheel_device_id=context['wheel_device_id'],
            confirmed_gyro_bias=verified['value']['calibration']['confirmed_gyro_bias'],
            gyro_bias_provenance=verified['metadata']))
        motion.update(by_sides=copy.deepcopy(by_sides),enabled=any(row['enabled'] for row in by_sides.values()),
                      declaration_verified=True,physical_accuracy_validated=False)
        motion['reason']='设备声明和独立零偏依据已校验；仅适用于建图' if motion['enabled'] else by_sides['all']['reason']
    except (OSError,ValueError,KeyError,TypeError) as error:
        motion.update(enabled=False,reason=str(error),by_sides={s:dict(enabled=False,reason=str(error)) for s in by_sides})
        if not geometry_checked:
            by_sides={s:dict(enabled=False,reason=str(error)) for s in by_sides}
            geometry['by_sides']=copy.deepcopy(by_sides)
            geometry['reason']=str(error)
    return dict(live_motion_correction=motion,live_geometry=geometry,
                live_mapping=dict(enabled=any(row['enabled'] for row in by_sides.values()),
                    reason=by_sides['all']['reason'],by_sides=copy.deepcopy(by_sides)))
