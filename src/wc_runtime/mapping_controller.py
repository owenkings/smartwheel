"""One-command experimental mapping backend; actual hardware and native maps.

RViz is owned by mapping_app. This backend owns a bounded supervisor and an
owner-loss watcher, and requires measured mounting candidates before startup.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import shutil
import sqlite3
import subprocess
import sys
import time

import numpy as np
import yaml

from .cli import ROOT,RUN,target,device_preflight,imu_preflight,ros_command,environment,H30_DEVICE,H30_BY_ID,H30_SERIAL
from .prepare_picker_input import project_path,read_json,write_new,json_bytes,_json_loads
from .storage_policy import logical_path, StoragePolicy
from .hardware_setup import resolve_hardware_setup
from .supervisor import stop_registered,shutdown_policy,ticks
from .sensor_viewer import desktop_environment,confirm_display
from .single_mapping import hash_file
from .mapping_shutdown import PERSISTENCE_CLOSE_S, MAPPING_COMPONENT_GRACE_S, policy as persistence_policy
from .mapping_cloud import selected_cloud_source, cloud_description, source_frame_topic, preview_cloud_topic


STORAGE_STOP_FREE_BYTES = 2 * 1024**3
EXPORT_EXTRA_FREE_BYTES = 512 * 1024**2
# Run11 single-left bag + input: about 706 decimal MB / 107 s, before lite.
# This is an advisory estimate, neither an upper bound nor a capacity guarantee.
ESTIMATED_BYTES_PER_SECOND_PER_LIDAR = 6_600_000


def storage_status(handle):
    """Read the actual session filesystem; never substitute ROOT on failure."""
    directory = project_path(ROOT, handle['directory'])
    try:
        if not directory.is_dir():
            raise OSError('会话目录不存在或不是目录: ' + str(directory))
        free = shutil.disk_usage(directory).free
        if type(free) is not int or free < 0:
            raise OSError('可用空间返回值无效')
    except OSError as error:
        raise RuntimeError('STORAGE_CHECK_FAILED: 无法读取会话文件系统可用空间: ' + str(error)) from error
    return {'path': str(directory), 'free_bytes': free,
            'stop_free_bytes': STORAGE_STOP_FREE_BYTES,
            'policy': '2 GiB closing reserve estimate; successful persistence is not guaranteed'}


def storage_estimate(request):
    # The new session does not exist yet. Its nearest existing parent identifies
    # the destination filesystem, including an explicit project-local mount.
    directory = project_path(ROOT, request['output_dir'])
    while not directory.exists() and directory != ROOT:
        directory = directory.parent
    status = storage_status({'directory': directory})
    if request.get('mapping_enabled') is False:
        return {**status, 'mode': request['mode'], 'duration_s': 0,
                'estimated_recording_bytes': 0, 'estimate_only': True,
                'warning': status['free_bytes'] <= STORAGE_STOP_FREE_BYTES,
                'message': '容量提示：实时预览不录包、不生成累计地图；仅保留少量运行日志。当前可用 %.2f GB。'
                           '仍保留 2 GiB 空间阈值，空间不足会停止预览。'
                           % (status['free_bytes'] / 1_000_000_000)}
    sides = 2 if request['mode'] == 'all' else 1
    if request['duration_s'] == 0:
        return {**status, 'mode': request['mode'], 'duration_s': 0,
                'estimated_recording_bytes': None, 'estimate_only': True,
                'warning': status['free_bytes'] <= STORAGE_STOP_FREE_BYTES,
                'message': '容量提示：无预设运行时限，总记录量未知；当前可用 %.2f GB。'
                           '仍保留 2 GiB 收尾阈值，空间不足会提前停止；不保证无限容量或保存成功。'
                           % (status['free_bytes'] / 1_000_000_000)}
    expected = ESTIMATED_BYTES_PER_SECOND_PER_LIDAR * sides * request['duration_s']
    return {**status, 'mode': request['mode'], 'duration_s': request['duration_s'],
            'estimated_recording_bytes': expected, 'estimate_only': True,
            'warning': status['free_bytes'] < expected + STORAGE_STOP_FREE_BYTES,
            'message': '容量提示：按单路旧记录约 6.6 MB/s、双路两倍估算，本次记录约 %.2f GB，当前可用 %.2f GB；'
                       '另留 2 GiB 收尾余量。估算不是硬件上限，也不保证能运行至指定时长或完成导出；低空间将提前停止。'
                       % (expected / 1_000_000_000, status['free_bytes'] / 1_000_000_000)}


def rotation(value,label):
    matrix=np.asarray(value,dtype=float)
    if matrix.shape!=(3,3) or not np.isfinite(matrix).all() or not np.allclose(matrix.T@matrix,np.eye(3),atol=1e-6) or not math.isclose(np.linalg.det(matrix),1,abs_tol=1e-6):
        raise ValueError(label+' 必须是有测量依据的三维旋转矩阵')
    return matrix


def configuration(request):
    config=read_json(ROOT,project_path(ROOT,request['config_path']))
    if config.get('schema_version')!=1 or config.get('status')!='EXPERIMENT' or config.get('source_mode')!='real':
        raise ValueError('建图配置须明确标记真实硬件试验')
    if request['mode'] not in ('left','right','all'):raise ValueError('无效雷达模式')
    odometry_source=config.get('odometry_source','wheel_imu')
    if odometry_source not in ('wheel_imu','icp'):
        raise ValueError('odometry_source 只支持 wheel_imu 或 icp，不进行自动切换')
    config['odometry_source']=odometry_source
    mapping_enabled=request.get('mapping_enabled',config.get('mapping_enabled',False))
    if type(mapping_enabled) is not bool:
        raise ValueError('mapping_enabled 必须为 true 或 false')
    config['mapping_enabled']=mapping_enabled
    # An explicit CLI selection overrides the file; old files keep filtered.
    selected_cloud_source(config)
    config['cloud_source']=selected_cloud_source(request if 'cloud_source' in request else config)
    # This entry is user-operated continuous mapping. Strict diagnostic tools
    # keep their own defaults; old mapping configuration cannot restore timers.
    config['continuous_mapping']=True
    motion_model=config.get('motion_model','se3_gyro')
    if motion_model not in ('planar_ekf','se3_gyro'):
        raise ValueError('motion_model 只支持 planar_ekf 或 se3_gyro')
    config['motion_model']=motion_model
    estimator=config.get('wheel_imu_estimator','five_state')
    if estimator not in ('five_state','robot_localization'):
        raise ValueError('wheel_imu_estimator 只支持 five_state 或 robot_localization，不自动切换')
    if estimator=='robot_localization' and motion_model!='planar_ekf':
        raise ValueError('robot_localization 对照只支持明确的 planar_ekf 运动模型')
    config['wheel_imu_estimator']=estimator
    offline=config.get('offline_experiment',False)
    if type(offline) is not bool:
        raise ValueError('offline_experiment 必须为显式布尔值')
    config['offline_experiment']=offline
    from .mapping_input import CHECKPOINT_POLICIES
    checkpoint_policy=config.get('archive_checkpoint_policy','periodic')
    if checkpoint_policy not in CHECKPOINT_POLICIES:
        raise ValueError('archive_checkpoint_policy 只支持 periodic 或 on_close')
    config['archive_checkpoint_policy']=checkpoint_policy
    if odometry_source=='wheel_imu':
        from .mapping_odometry import validate_covariance
        config['wheel_imu_covariance']=validate_covariance(config.get('wheel_imu_covariance'))
    renderer=config.get('rviz_renderer','system')
    if renderer not in ('system','software'):
        raise ValueError('rviz_renderer 只支持 system 或 software')
    config['rviz_renderer']=renderer
    rate=config.get('input_rate_hz')
    rate_limit=10 if offline else 5
    if type(rate) not in (int,float) or not math.isfinite(rate) or not 0<rate<=rate_limit:
        raise ValueError('建图输入频率必须在 0 至 %s Hz 之间；10 Hz 仅限 offline_experiment=true' % rate_limit)
    if type(config.get('max_pair_delta_ns')) is not int or not 0<=config['max_pair_delta_ns']<=50_000_000:
        raise ValueError('双源接收时间差上限须为 0 至 50000000 ns')
    if any(key in config for key in ('mounts','imu_mount','wheel_candidate','wheel_candidate_config')):
        raise ValueError('安装与轮参数只能来自 hardware_setup_config；请移除建图配置中的旧重复参数')
    setup_path=project_path(ROOT,config['hardware_setup_config'])
    if not setup_path.is_file() or setup_path.stat().st_size>131072:
        raise ValueError('硬件安装文件不存在或超过 128 KiB')
    raw=setup_path.read_bytes()
    if len(raw)>131072:raise ValueError('硬件安装文件超过 128 KiB')
    setup=_json_loads(raw.decode('utf-8'))
    config.update(resolve_hardware_setup(setup))
    from .calibration_geometry import require_geometry_capability, motion_capability, verify_geometry_sources
    if setup['schema_version']==2:
        source_verification=verify_geometry_sources(setup,ROOT)
        config['geometry_report']['source_verification']=source_verification
        if source_verification['status']!='PASS':
            raise ValueError('外参原始资料快照缺失、越界或SHA-256不一致；不能用原路径或历史值替代')
    capability=(motion_capability(request['mode']) if mapping_enabled else 'native_preview')
    if offline and setup['schema_version']==1:
        capability='legacy_replay'
    config['required_geometry_capability']=capability
    require_geometry_capability(config,capability)
    config['native_preview_only']=(not mapping_enabled and
        config['geometry_capabilities'][motion_capability(request['mode'])]['status']!='AVAILABLE'
        and not (offline and setup['schema_version']==1))
    if config['native_preview_only'] and (config.get('manual_controls') or {}).get('interaction_policy') != 'hybrid_manual':
        config['manual_controls']=None
        config['manual_controls_disabled_reason']='V7外参未齐：原生预览不启动轮反馈/控制，不继承旧运动配置。'
    elif config['native_preview_only']:
        config['manual_authorization_scope']='USER_SELECTED_HYBRID_MANUAL_INDEPENDENT_OF_LIDAR_GEOMETRY'
    from .map_quality import validate_map_quality_policy
    config['map_quality_policy']=validate_map_quality_policy(config.get('map_quality_policy'))
    from .mapping_profile import validate_profile, resolve_native_profile
    config['map_profile']=validate_profile(config.get('map_profile'))
    # Resolved mode is required for the selected sensor origin and ray policy.
    config['mode']=request['mode']
    if config['native_preview_only']:
        config['native_mapping_profile']={'motion_model':motion_model,'parameters':{},'grid_profile':None,
            'status':'UNAVAILABLE_GEOMETRY',
            'reasons':list(config['geometry_capabilities']['ground_height_'+request['mode']]['reasons'])}
    else:
        config['native_mapping_profile']=resolve_native_profile(config)
    config['_hardware_setup_raw_text']=raw.decode('utf-8')
    config['hardware_setup_provenance']={'source_path':logical_path(ROOT, setup_path),
        'sha256':hashlib.sha256(raw).hexdigest(),'snapshot_file':'hardware_setup.json',
        'mounting_validated':False,'imu_translation_used':False}
    if type(config.get('cameras_enabled',False)) is not bool:
        raise ValueError('cameras_enabled must be boolean')
    if config.get('cameras_enabled'):
        from wc_cameras.config import validate_config as validate_cameras
        camera_path=project_path(ROOT,config['camera_config'])
        camera_raw=camera_path.read_bytes()
        if len(camera_raw)>131072:raise ValueError('摄像头配置超过 128 KiB')
        validate_cameras(_json_loads(camera_raw.decode('utf-8')))
        config['_cameras_raw_text']=camera_raw.decode('utf-8')
        config['cameras_provenance']={'source_path':logical_path(ROOT, camera_path),
            'sha256':hashlib.sha256(camera_raw).hexdigest(),'snapshot_file':'cameras_config.json',
            'images_recorded':False}
    hardware=read_json(ROOT,project_path(ROOT,config['wheel_hardware_config']))
    from wc_motion.feedback_transport import validate_config
    validate_config(hardware)
    wheel=config['wheel_candidate']
    from wc_motion.history_preview import HistoryPreview
    HistoryPreview(wheel)
    config.update(mode=request['mode'],session_id=request['session_id'],
                  sensor_ids=read_json(ROOT,ROOT/'config/live_unvalidated.json')['sensor_ids'],
                  wheel_candidate=wheel,wheel_device_id=hardware['device_id'])
    config['prior_template']={'schema_version':1,'status':'EXPERIMENT','source_mode':'real',
        'reference_frame':'mapping_reference','imu_sensor_id':config['imu_sensor_id'],
        'wheel_device_id':hardware['device_id'],'wheel_candidate':wheel,
        'input_cloud_topic':'/wc_mapping/app/input_cloud','output_cloud_topic':'/wc_mapping/app/scan_cloud',
        'private_tf_topic':'/wc_mapping/app/tf','guess_frame_id':'prior_odom',
        'prior_odom_topic':'/wc_mapping/app/prior_odom',
        'require_dual_time_offsets':request['mode']=='all',
        'imu_topic':'/wc_mapping/imu/source_frame','wheel_topic':'/wc_mapping/wheel/feedback_raw'}
    config['prior_template']['odometry_source']=odometry_source
    config['prior_template']['continuous_mapping']=True
    config['prior_template']['motion_model']=motion_model
    config['prior_template']['estimator']=estimator
    config['prior_template']['offline_experiment']=offline
    if motion_model=='planar_ekf':
        from .mapping_planar import validate_planar_config
        config['planar_ekf']=validate_planar_config(config.get('planar_ekf'))
        config['prior_template']['planar_ekf']=config['planar_ekf']
    # A confirmed calibration is optional. Its provenance is archived; loading
    # a declaration does not independently prove the chair was stationary.
    if config.get('gyro_bias_config') is not None:
        from .mapping_planar import validate_confirmed_bias
        bias_path=project_path(ROOT,config['gyro_bias_config'])
        if not bias_path.is_file() or bias_path.stat().st_size>32768:
            raise ValueError('陀螺零偏文件不存在或超过32 KiB')
        bias_raw=bias_path.read_bytes()
        config['confirmed_gyro_bias']=validate_confirmed_bias(_json_loads(bias_raw.decode('utf-8')))
        evidence_path=bias_path.with_name(bias_path.stem+'.evidence.json')
        if not evidence_path.is_file() or evidence_path.is_symlink() or evidence_path.stat().st_size>262144:
            raise ValueError('零偏文件必须附带同名 .evidence.json 依据文件')
        evidence_raw=evidence_path.read_bytes()
        if hashlib.sha256(evidence_raw).hexdigest()!=config['confirmed_gyro_bias']['evidence_sha256']:
            raise ValueError('零偏依据文件SHA-256与声明不一致')
        _json_loads(evidence_raw.decode('utf-8'))
        config['_gyro_bias_raw_text']=bias_raw.decode('utf-8')
        config['_gyro_bias_evidence_raw_text']=evidence_raw.decode('utf-8')
        config['gyro_bias_provenance']={'source_path':logical_path(ROOT, bias_path),
            'sha256':hashlib.sha256(bias_raw).hexdigest(),'snapshot_file':'confirmed_gyro_bias.json',
            'evidence_sha256':hashlib.sha256(evidence_raw).hexdigest(),
            'evidence_snapshot_file':'confirmed_gyro_bias.evidence.json'}
    elif config.get('confirmed_gyro_bias') is not None:
        raise ValueError('零偏只能来自gyro_bias_config指向的独立确认文件')
    else:
        config['confirmed_gyro_bias']=None
        config['gyro_bias_provenance']=None
    config['prior_template']['confirmed_gyro_bias']=config['confirmed_gyro_bias']
    if odometry_source=='wheel_imu':
        config['prior_template']['wheel_imu_covariance']=config['wheel_imu_covariance']
    config['shutdown_persistence'] = persistence_policy()
    return config


def check_configuration(request):
    """Parse and compose without target, serial, ROS, GUI or file mutations."""
    config=configuration(request)
    from .mapping_input import validate_config
    validate_config(config)
    setup=_json_loads(config['_hardware_setup_raw_text'])
    return {'status':'CONFIG_VALID_EXPERIMENT','hardware_started':False,
        'mode':request['mode'],'hardware_setup':config['hardware_setup_provenance'],
        'mapping_enabled':config['mapping_enabled'],
        'cloud_source':config['cloud_source'],'cloud_description':cloud_description(config['cloud_source']),
        'source_topics':{side:source_frame_topic(side,config['cloud_source']) for side in
                         (('left','right') if config['mode']=='all' else (config['mode'],))},
        'operation':'mapping' if config['mapping_enabled'] else 'live_preview',
        'rviz_renderer':config['rviz_renderer'],
        'odometry_source':config['odometry_source'],
        'motion_model':config['motion_model'],
        'wheel_imu_estimator':config['wheel_imu_estimator'],
        'offline_experiment':config['offline_experiment'],
        'input_rate_hz':config['input_rate_hz'],
        'geometry_capabilities':config['geometry_capabilities'],
        'geometry_report':config['geometry_report'],
        'required_geometry_capability':config['required_geometry_capability'],
        'native_preview_only':config['native_preview_only'],
        'native_mapping_profile':config['native_mapping_profile'],
        'cloud_filter':config.get('cloud_filter'),
        'cloud_filter_effective':bool(config['mapping_enabled'] and config.get('cloud_filter',{}).get('enabled')),
        'gyro_bias_provenance':config.get('gyro_bias_provenance'),
        'icp_odometry_required':config['mapping_enabled'] and config['odometry_source']=='icp',
        'continuous_mapping':True,
        'runtime_limits':{'duration_s':None,'archive_outputs':None,
                          'stationary_startup_required':False,'data_freshness_shutdown':False},
        'archive_checkpoint_policy':config['archive_checkpoint_policy'],
        'manual_controls':config.get('manual_controls'),
        'manual_controls_disabled_reason':config.get('manual_controls_disabled_reason'),
        'shutdown_persistence':config['shutdown_persistence'],
        'calibration_reference':setup.get('calibration_reference'),
        'lidars':setup.get('lidars'),'imu':setup.get('imu'),'wheel_odometry':config['wheel_candidate'],
        'resolved_mounts':config['mounts'],'resolved_imu':config['imu_mount'],
        'message':'配置语法和变换检查通过；不会启动硬件，也不代表现场安装或地图精度通过。'}


def preflight(request):
    target()
    if Path(request['project_root'])!=ROOT or Path.cwd().resolve()!=ROOT:
        raise RuntimeError('请在 /home/nvidia/wheelchair 工程目录运行')
    if configuration(request).get('offline_experiment'):
        raise ValueError('offline_experiment cannot start live sources; use compare')
    output=project_path(ROOT,request['output_dir'])
    if output.exists():raise RuntimeError('地图输出目录已存在，请使用新名称')
    if (RUN/'sessions'/request['session_id']).exists():raise RuntimeError('会话名称已使用')
    env=desktop_environment();confirm_display(env)
    device_preflight()
    from wc_motion.feedback_transport import validate_config
    c=configuration(request)
    if c['mapping_enabled']:imu_preflight()
    hardware=read_json(ROOT,project_path(ROOT,c['wheel_hardware_config']))
    # The transport validates USB/by-id/alias and ownership again under its lease.
    validate_config(hardware)
    from wc_motion.feedback_transport import verify_identity,require_unoccupied
    if c['mapping_enabled'] or c.get('manual_controls') is not None:
        actual,_=verify_identity(hardware);require_unoccupied(actual)
    native=[('rviz2','rviz2')]
    if c['mapping_enabled']:
        native.append(('rtabmap_slam','rtabmap'))
        if c['odometry_source']=='icp':native.append(('rtabmap_odom','icp_odometry'))
    for package,executable in native:
        if not (Path('/opt/ros/humble/lib')/package/executable).is_file():
            raise RuntimeError('目标缺少 '+package+'/'+executable)
    for path in ['src/wc_runtime/mapping_input.py','src/wc_runtime/mapping_prior.py',
                 'src/wc_runtime/mapping_cloud.py','src/wc_runtime/mapping_bias.py',
                 'src/wc_runtime/mapping_monitor.py','src/wc_bringup/launch/mapping_app.launch.py',
                 'config/rviz/mapping_live.rviz','install/main/wc_bringup/lib/wc_bringup/mapping_rviz']:
        if not (ROOT/path).is_file():raise RuntimeError('建图组件尚未部署: '+path)
    optional=[]
    if c.get('cameras_enabled'):
        optional+=['src/wc_runtime/mapping_cameras.py','install/main/wc_camera_panel/lib/libwc_camera_panel.so']
    if c.get('visualization'):optional+=['src/wc_runtime/mapping_visuals.py']
    if c.get('manual_controls') is not None:optional+=['src/wc_runtime/mapping_wheel.py']
    if not c['mapping_enabled']:optional+=['src/wc_runtime/mapping_preview.py']
    for path in optional:
        if not (ROOT/path).is_file():raise RuntimeError('建图界面组件尚未部署: '+path)
    return {'storage': storage_estimate(request)}


def plan(request,config):
    directory=Path(request['output_dir']);duration=0;sensor_mode='dual' if request['mode']=='all' else 'single_'+request['mode']
    # The supervisor owns the user's total runtime and the existing shutdown
    # grace. Sensor children remain live until its signal, so an independent
    # diagnostic tool's finite duration range cannot truncate this session.
    source_duration = 0
    sides=('left','right') if request['mode']=='all' else (request['mode'],)
    close_args = ['--close-timeout-s', str(PERSISTENCE_CLOSE_S)]
    topics=['/wc_mapping/imu/source_frame','/wc_mapping/imu/data_raw','/wc_mapping/imu/diagnostics',
        '/wc_mapping/wheel/feedback_raw','/wc_mapping/wheel/odom_preview','/wc_mapping/wheel/preview_diagnostics']
    topics += ['/wc_mapping/lidar_'+side+'/'+topic for side in sides for topic in ('source_frame','source_frame_filtered','diagnostics')]
    odometry_source=config.get('odometry_source','icp')
    topics += ['/wc_mapping/app/'+topic for topic in ('input_cloud','scan_cloud','prior_odom','odom','info',
               'mapData','mapGraph','mapPath','cloud_map','grid_map','tf','tf_static','status_markers')]
    if odometry_source=='icp':topics.append('/wc_mapping/app/odom_info')
    commands=[ros_command(['ros2','bag','record','-s','sqlite3','--max-bag-size','536870912','-o',directory/'bag',*topics]),
        ros_command(['ros2','launch',ROOT/'src/wc_bringup/launch/mapping_app.launch.py','session_root:='+str(directory),
                     'odometry_source:='+odometry_source]),
        ros_command([sys.executable,'-s','-m','wc_runtime.mapping_input','--config',directory/'runtime_config.json','--output-root',directory/'input',*close_args]),
        ros_command([sys.executable,'-s','-m','wc_runtime.mapping_prior','--config',directory/'prior_config.json',
                     '--session-id',request['session_id'],'--output-root',directory/'prior','--wait-config-seconds','0',*close_args]),
        ros_command([sys.executable,'-s','-m','wc_runtime.mapping_monitor','--session-root',directory,*close_args]),
        ros_command([sys.executable,'-s','-m','wc_imu.ros_node','--device',H30_DEVICE,'--expected-by-id',H30_BY_ID,
                     '--hardware-serial',H30_SERIAL,'--sensor-id',config['imu_sensor_id'],'--session-id',request['session_id'],
                     '--run-root',RUN,'--duration',str(source_duration),'--stale-timeout','0']),
        ros_command([sys.executable,'-s','-m','wc_motion.feedback_transport','--config',ROOT/config['wheel_hardware_config'],
            '--output',directory/'wheel_feedback.jsonl','--run-root',RUN,'--samples',str(source_duration*10),
            '--rate-hz','10','--max-duration',str(source_duration),'--allow-read-queries','--publish-ros','--async-journal',
            '--preview-history-calibration','--continuous-preview','--preview-config',directory/'wheel_candidate.json',
            '--journal-close-timeout-s',str(PERSISTENCE_CLOSE_S)]),
        ros_command(['ros2','launch','wc_xt_driver','dual_sources.launch.py','source_mode:='+sensor_mode,
            'session_id:='+request['session_id'],'run_root:='+str(RUN),'allow_hardware:=true','read_only_probe:=false',
            'device_config_policy:=preserve_current','publish_cloud_mirror:=true','max_runtime_seconds:='+str(source_duration),
            'source_stale_seconds:=0'])]
    if config.get('visualization'):
        commands.append(ros_command([sys.executable,'-s','-m','wc_runtime.mapping_visuals','--session-root',directory]))
    if config.get('manual_controls') is not None:
        # A single serial owner keeps real feedback available before activation.
        # No separate teleop process may race the feedback transport for the port.
        commands[6]=ros_command([sys.executable,'-s','-m','wc_runtime.mapping_wheel','--session-root',directory,*close_args])
    if config.get('cameras_enabled'):
        commands.append(ros_command([sys.executable,'-s','-m','wc_runtime.mapping_cameras',
            '--config',directory/'cameras_config.json','--output-root',directory/'cameras','--duration',str(source_duration),*close_args]))
    if not config['mapping_enabled']:
        # Raw single-frame preview does not instantiate SLAM, motion integration,
        # archive workers, a rosbag recorder or an IMU acquisition process.
        native_preview = any(side not in config.get('mounts', {}) for side in sides)
        commands=([commands[6]] if config.get('manual_controls') is not None else [])+[commands[7],ros_command([sys.executable,'-s','-m',
            'wc_runtime.mapping_preview','--session-root',directory])]+(
                [commands[-1]] if config.get('cameras_enabled') else [])
        topics=[]
    commands.append(ros_command([sys.executable,'-s','-m','wc_runtime.runtime_health',
                                 '--session-root',directory,'--watch']))
    return {'session_id':request['session_id'],'role':'mapping_app','commands':commands,'environment':environment(),
        'offline_experiment':config.get('offline_experiment',False),
        'launch_allowed':not config.get('offline_experiment',False),
        'mapping_enabled':config['mapping_enabled'],
        'duration_s':duration,'sigint_grace_s':MAPPING_COMPONENT_GRACE_S,'allow_component_exit':False,
        'locks':['sensor_owner.lock','domain-83-source.lock','domain-83-processing.lock','mapping-app.lock'],
        'bag_topics':topics,'owner_pid':request['owner_pid'],'owner_start_ticks':ticks(request['owner_pid'])}


def inspect(handle):
    path=Path(handle['runtime'])/'manifest.json'
    if not path.exists():return {'state':'FAILED','exit_code':None,'reason':'Manifest missing'}
    value=read_json(ROOT,path)
    result={k:value.get(k) for k in ('state','exit_code','stop_reason','cleanup_errors','supervisor_pid','supervisor_start_ticks')}
    if result['state']=='FAILED' and handle.get('directory'):
        causes={}
        for component,relative,field in [('input','input/status.json','failure_reason'),
                ('prior','prior/status.json','failure'),('monitor','health/status.json','failure'),
                ('wheel','wheel_status.json','failure')]:
            try:
                status=read_json(ROOT,project_path(ROOT,Path(handle['directory'])/relative),limit=2_000_000)
                if status.get(field):causes[component]=str(status[field])[:2000]
            except (OSError,ValueError):
                continue
        if causes:result['component_failures']=causes
    return result


def stop(handle):
    path=project_path(ROOT,Path(handle['runtime'])/'manifest.json')
    value=read_json(ROOT,path);stop_registered(path)
    deadline=time.monotonic()+shutdown_policy(value)['cli_stop_wait_s']
    while time.monotonic()<deadline:
        value=inspect(handle)
        if value['state']!='RUNNING':return value
        time.sleep(.2)
    raise RuntimeError('建图会话未按时关闭，请保留日志: '+str(path))


def archive_configuration(directory,config):
    """Freeze exactly the configuration already resolved for this session."""
    config=dict(config)
    setup_bytes=config.pop('_hardware_setup_raw_text').encode('utf-8')
    if hashlib.sha256(setup_bytes).hexdigest()!=config['hardware_setup_provenance']['sha256']:
        raise ValueError('硬件安装快照与解析来源哈希不一致')
    write_new(directory/'hardware_setup.json',setup_bytes)
    if config.get('cameras_enabled'):
        camera_bytes=config.pop('_cameras_raw_text').encode('utf-8')
        if hashlib.sha256(camera_bytes).hexdigest()!=config['cameras_provenance']['sha256']:
            raise ValueError('摄像头配置快照哈希不一致')
        write_new(directory/'cameras_config.json',camera_bytes)
    write_new(directory/'wheel_candidate.json',json_bytes(config['wheel_candidate']))
    if config.get('_gyro_bias_raw_text') is not None:
        bias_bytes=config.pop('_gyro_bias_raw_text').encode('utf-8')
        if hashlib.sha256(bias_bytes).hexdigest()!=config['gyro_bias_provenance']['sha256']:
            raise ValueError('陀螺零偏快照哈希不一致')
        write_new(directory/'confirmed_gyro_bias.json',bias_bytes)
        evidence_bytes=config.pop('_gyro_bias_evidence_raw_text').encode('utf-8')
        if hashlib.sha256(evidence_bytes).hexdigest()!=config['gyro_bias_provenance']['evidence_sha256']:
            raise ValueError('陀螺零偏依据快照哈希不一致')
        write_new(directory/'confirmed_gyro_bias.evidence.json',evidence_bytes)
    write_new(directory/'runtime_config.json',json_bytes(config))
    if config.get('geometry_report'):
        write_new(directory/'capability_assessment.json', json_bytes(config['geometry_report']))
    return config


def render_view(config):
    """Bind camera labels to the same immutable bytes used by acquisition."""
    view=yaml.safe_load((ROOT/'config/rviz/mapping_live.rviz').read_text(encoding='utf-8'))
    cloud_source=selected_cloud_source(config)
    for display in view['Visualization Manager']['Displays']:
        topic=display.get('Topic',{}).get('Value')
        if topic=='/wc_mapping/app/odom':
            display['Name']=('平面 EKF：轮速 + IMU' if config.get('motion_model')=='planar_ekf' else '轮速 + IMU 里程计') if config.get('odometry_source')=='wheel_imu' else 'ICP 里程计'
        elif topic=='/wc_mapping/app/prior_odom' and config.get('odometry_source')=='wheel_imu':
            display['Name']='轮速 + IMU 原始诊断（与主里程计相同轨迹）'
            display['Enabled']=display['Value']=False
        elif topic=='/wc_mapping/app/scan_cloud':
            display['Name']='实时建图输入（'+('主机滤波前 SDK XYZ' if cloud_source=='raw' else '主机滤波后 SDK XYZ')+'）'
            if config.get('mapping_enabled') and config.get('cloud_filter',{}).get('enabled'):
                display['Name']+='＋建图几何筛选'
    if not config.get('visualization',{}).get('display_ground_projection'):
        for display in view['Visualization Manager']['Displays']:
            if display.get('Topic',{}).get('Value')=='/wc_mapping/app/view_grid':
                display['Topic']['Value']='/wc_mapping/app/grid_map'
                display['Name']='2D map at native projection height'
    if not config['mapping_enabled']:
        manager=view['Visualization Manager']
        native_preview = any(side not in config.get('mounts', {}) for side in
                             (('left','right') if config['mode']=='all' else (config['mode'],)))
        manager['Global Options']['Fixed Frame']=('lidar_right' if config['mode']=='right' else 'lidar_left') if native_preview else 'mapping_reference'
        manager['Displays']=[d for d in manager['Displays'] if d.get('Class')=='rviz_default_plugins/TF'
            or d.get('Topic',{}).get('Value')=='/wc_mapping/app/view_markers']
        for side in (('left','right') if config['mode']=='all' else (config['mode'],)):
            for filtered in (True,False):
                source='filtered' if filtered else 'raw'
                enabled=source==cloud_source
                manager['Displays'].insert(0,{
                    'Class':'rviz_default_plugins/PointCloud2',
                    'Name':('左' if side=='left' else '右')+'雷达当前帧（'+('主机滤波后' if filtered else '主机滤波前 SDK XYZ')+'）',
                    'Enabled':enabled,'Value':enabled,'Alpha':1.,'Color Transformer':'AxisColor','Axis':'Z',
                    'Autocompute Value Bounds':{'Value':True},'Position Transformer':'XYZ',
                    'Style':'Points','Size (Pixels)':2,'Size (m)':.03,'Decay Time':0.,'Use Fixed Frame':True,
                    'Topic':{'Value':preview_cloud_topic(side,source),
                        'Depth':1,'History Policy':'Keep Last','Reliability Policy':'Best Effort','Durability Policy':'Volatile'}})
                if native_preview:
                    manager['Displays'][0]['Name']+=' [原生坐标：切换 Fixed Frame 为 lidar_'+side+']'
    if config.get('cameras_enabled'):
        cameras=_json_loads(config['_cameras_raw_text'])
        panel={'Class':'wc_camera_panel/CameraPanel','Name':'四路实时摄像头',
               'Compact Layout':True,'Mapping Status':cameras['mapping_status']}
        panel.update({item['role']+' USB Port':item['port'] for item in cameras['cameras']})
        view['Panels'].append(panel)
    return yaml.safe_dump(view,allow_unicode=True,sort_keys=False).encode('utf-8')


def start(request):
    config=configuration(request);directory=project_path(ROOT,request['output_dir'])
    if config.get('offline_experiment'):
        raise ValueError('offline_experiment cannot start live sources; use compare')
    runtime=project_path(ROOT,RUN/'sessions'/request['session_id']/'mapping_app')
    handle={'session_id':request['session_id'],'mode':request['mode'],'directory':str(directory),'runtime':str(runtime),
            'odometry_source':config['odometry_source'],'mapping_enabled':config['mapping_enabled'],
            'motion_model':config['motion_model'],
            'manual_controls':config.get('manual_controls'),
            'confirmed_gyro_bias_applied':config['confirmed_gyro_bias'] is not None,
            'cloud_source':config['cloud_source']}
    handle['retention_profile']=request.get('retention_profile','experiment')
    config['retention_profile']=handle['retention_profile']
    directory.mkdir(parents=True,exist_ok=False)
    if config['mapping_enabled']:(directory/'slam').mkdir()
    runtime.mkdir(parents=True,exist_ok=False)
    write_new(directory/'view.rviz',render_view(config))
    config['duration_s']=0
    config=archive_configuration(directory,config)
    from .mapping_control_paths import manual_socket_path
    socket_directory = manual_socket_path(ROOT, directory, request['session_id']).parent
    task=plan(request,config);write_new(runtime/'plan.json',json_bytes(task))
    write_new(directory/'session.json',json_bytes({**request,'kind':'MAPPING_APP_EXPERIMENT',
        'mapping_enabled':config['mapping_enabled'],'data_retention':config['retention_profile'],
        'manual_socket_directory':str(socket_directory),
        'cloud_source':config['cloud_source'],'cloud_description':cloud_description(config['cloud_source']),
        'odometry_source':config['odometry_source'],'icp_odometry_required':config['mapping_enabled'] and config['odometry_source']=='icp',
        'motion_model':config['motion_model'],'native_mapping_profile':config['native_mapping_profile'],
        'gyro_bias_provenance':config['gyro_bias_provenance'],
        'source_mode':'real','time_source':'arrival_only','wheel_scale_validated':False,'mounting_validated':False,
        'navigation_validated':False,'formal_acceptance':False,'raw_data_preserved':None if config['mapping_enabled'] else False,
        'raw_data_preservation_note':'Experiment retains input archive and hashes after save; map_only is an explicit irreversible retention choice.' if config['mapping_enabled'] else 'Live preview; no point-cloud archive or rosbag recording.',
        'hardware_setup':config['hardware_setup_provenance'],
        'files':{p:hash_file(ROOT/p) for p in ('src/wc_runtime/hardware_setup.py','src/wc_runtime/mapping_input.py','src/wc_runtime/mapping_prior.py','src/wc_runtime/mapping_odometry.py',
            'src/wc_runtime/mapping_monitor.py','src/wc_runtime/mapping_controller.py','src/wc_bringup/launch/mapping_app.launch.py',
            'src/wc_runtime/mapping_visuals.py','src/wc_runtime/mapping_cameras.py','src/wc_runtime/mapping_wheel.py',
            'src/wc_bringup/src/mapping_rviz.cpp','src/wc_bringup/src/mapping_teleop.cpp',
            'src/wc_bringup/include/wc_bringup/mapping_render_diagnostics.hpp',
             'src/wc_runtime/mapping_shutdown.py','src/wc_runtime/mapping_app.py',
             'src/wc_runtime/mapping_cloud.py',
             'src/wc_runtime/mapping_bias.py',
             'src/wc_runtime/mapping_planar.py','src/wc_runtime/mapping_filter.py',
             'src/wc_runtime/mapping_profile.py','src/wc_runtime/mapping_bias_confirm.py',
            'src/wc_runtime/supervisor.py','src/wc_runtime/component.py','src/wc_motion/feedback_transport.py')}}))
    env=dict(os.environ);env.update(environment());env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    child=None
    try:
        with (runtime/'supervisor.log').open('xb') as log:
            child=subprocess.Popen([sys.executable,'-m','wc_runtime.supervisor','--plan',str(runtime/'plan.json')],
                cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if (runtime/'manifest.json').exists():
                value=inspect(handle)
                if value['state']=='RUNNING':break
                if value['state']=='FAILED':raise RuntimeError('启动失败: '+str(value))
            if child.poll() is not None:raise RuntimeError('建图主管提前退出')
            time.sleep(.1)
        else:raise RuntimeError('建图主管未确认启动')
        with (runtime/'owner-watch.log').open('xb') as log:
            subprocess.Popen([sys.executable,'-m','wc_runtime.mapping_controller','--watch-owner','--runtime',str(runtime)],
                cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        return handle
    except BaseException:
        if (runtime/'manifest.json').exists():stop(handle)
        elif child is not None and child.poll() is None:
            child.send_signal(signal.SIGTERM);child.wait(timeout=shutdown_policy(task)['cli_stop_wait_s'])
        raise


def rviz_environment(handle):
    env=desktop_environment();env.update(environment())
    config=read_json(ROOT,project_path(ROOT,Path(handle['directory'])/'runtime_config.json'))
    renderer=config.get('rviz_renderer','system')
    if renderer not in ('system','software'):
        raise ValueError('会话 rviz_renderer 配置无效')
    if renderer=='software':
        # Only the owned GUI inherits these overrides. Sensor capture, wheel
        # feedback, ICP and RTAB-Map keep their original process environments.
        # Mesa software rendering avoids the observed native GL display fault;
        # no system driver, desktop or shell environment is modified.
        env.update(LIBGL_ALWAYS_SOFTWARE='1',__GLX_VENDOR_LIBRARY_NAME='mesa')
    return env


def rviz_command(handle):
    return ros_command([ROOT/'install/main/wc_bringup/lib/wc_bringup/mapping_rviz','-d',Path(handle['directory'])/'view.rviz','--ros-args',
        '-r','__node:=mapping_app_rviz','-r','/tf:=/wc_mapping/app/tf','-r','/tf_static:=/wc_mapping/app/tf_static'])


def rviz_startup_retry(handle, exit_code, elapsed_s):
    """Recognize only the observed software GL initialization failure.

    The caller must have reaped the owned component and must recheck session
    health and cancellation. This is a once-only compatibility retry, not a
    diagnosis of the GL fault or permission to restart an initialized viewer.
    """
    if (type(exit_code) is not int or exit_code != 245 or
            type(elapsed_s) not in (int, float) or not math.isfinite(elapsed_s) or
            not 0 <= elapsed_s <= 10):
        return None
    directory = project_path(ROOT, handle['directory'])
    config = read_json(ROOT, directory/'runtime_config.json')
    if config.get('rviz_renderer') != 'software':
        return None
    log = project_path(ROOT, directory/'rviz.log')
    if not log.is_file() or log.stat().st_size > 65536:
        return None
    with log.open('rb') as stream:
        data = stream.read(65537)
    signature = b'libGL error: failed to create drawable'
    if (len(data) > 65536 or signature not in data or
            b'Actual Ogre renderer:' in data or b'Native RViz ready;' in data):
        return None
    return {'reason': 'SOFTWARE_GL_DRAWABLE_INITIALIZATION_EXIT',
            'exit_code': exit_code, 'elapsed_s': elapsed_s,
            'original_log': str(log), 'original_log_sha256': hashlib.sha256(data).hexdigest(),
            'renderer': 'software', 'root_cause_confirmed': False}


def save_grid(path,output):
    import yaml
    with np.load(path,allow_pickle=False) as saved:
        data=saved['data'];metadata=json.loads(str(saved['metadata'].item()))
    width,height=metadata.get('width'),metadata.get('height')
    if type(width) is not int or type(height) is not int or not 0<width*height<=10_000_000 or min(width,height)<=0:
        raise RuntimeError('2D map dimensions must be positive bounded integers')
    if data.shape!=(height,width):raise RuntimeError('2D map shape mismatch')
    if data.dtype.kind not in 'iu' or not ((data>=-1)&(data<=100)).all():
        raise RuntimeError('2D map values must be integer occupancy in [-1,100]')
    resolution=metadata.get('resolution');origin=metadata.get('origin')
    if type(resolution) not in (int,float) or not math.isfinite(resolution) or resolution<=0:
        raise RuntimeError('2D map resolution must be finite positive metres per cell')
    if not isinstance(origin,list) or len(origin)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in origin):
        raise RuntimeError('2D map origin must have three finite coordinates in metres')
    if metadata.get('frame_id')!='mapping_map':raise RuntimeError('2D map frame must be mapping_map')
    # ROS map-server trinary encoding: unknown 205, occupied 0, free 254.
    pixels=np.where(data<0,205,np.where(data>=65,0,np.where(data<=19,254,205))).astype(np.uint8)
    image=output/'map_2d.pgm'
    write_new(image,('P5\n%d %d\n255\n'%(data.shape[1],data.shape[0])).encode()+np.flipud(pixels).tobytes())
    specification={'image':image.name,'mode':'trinary','resolution':metadata['resolution'],
        'origin':[metadata['origin'][0],metadata['origin'][1],0.0],'negate':0,'occupied_thresh':.65,'free_thresh':.196}
    write_new(output/'map_2d.yaml',yaml.safe_dump(specification,sort_keys=False).encode())
    return {'width':data.shape[1],'height':data.shape[0],'resolution_m':metadata['resolution'],
        'known_cells':int(((pixels==0)|(pixels==254)).sum()),'occupied_cells':int((data>=65).sum()),
        'note':'Last received experimental grid preview; may precede final database optimization; not a navigation acceptance map'}


def input_archive_summary(handle):
    """Read-only final-close audit of the unified input's checkpoint contract.

    Scan bounded JSONL indexes and CDR directory metadata, not CDR contents.
    A closed checkpoint certifies physical sync; matching file lengths alone
    are not a substitute for the checkpoint or a byte-integrity rehash.
    """
    from .single_mapping_input import MAX_CDR_BYTES
    def require(condition, reason):
        if not condition:
            raise RuntimeError('输入归档最终同步验收失败: '+reason)
    def count(value, name, maximum=None):
        require(type(value) is int and value >= 0 and (maximum is None or value <= maximum), name+' invalid count')
        return value
    directory=project_path(ROOT,handle['directory']);root=project_path(ROOT,directory/'input')
    mode=handle['mode'];require(mode in ('left','right','all'),'invalid mode')
    sides=('left','right') if mode=='all' else (mode,)
    status=read_json(ROOT,root/'status.json');checkpoint=read_json(ROOT,root/'checkpoint.json')
    checkpoint_policy=status.get('archive_checkpoint_policy','periodic')
    require(checkpoint_policy in ('periodic','on_close') and
            checkpoint.get('checkpoint_policy','periodic')==checkpoint_policy,
            'checkpoint policy differs from input status')
    require(status.get('schema_version')==1 and status.get('kind')=='MAPPING_INPUT_EXPERIMENT' and
            status.get('session_id')==handle['session_id'] and status.get('mode')==mode and
            status.get('source_mode')=='real','status identity mismatch')
    require(status.get('state')=='STOPPED' and status.get('failure_reason') is None,'input did not close normally')
    require(status.get('bootstrap_complete') is True,'bootstrap incomplete')
    for name in ('pending_outputs','pending_reserved_bytes'):
        require(type(status.get(name)) is int and status[name]==0,name+' not zero')
    accepted=count(status.get('enqueued_outputs'),'enqueued_outputs')
    archived=count(status.get('archived_outputs'),'archived_outputs')
    published=count(status.get('published_frames'),'published_frames')
    discarded=count(status.get('outputs_discarded_on_close'),'outputs_discarded_on_close')
    require(accepted>0 and accepted==archived and published<=archived and archived-published==discarded,
            'accepted/archived/published/close-discarded counts disagree')
    require(checkpoint.get('schema_version')==1 and checkpoint.get('kind')=='FSYNCED_MAPPING_ARCHIVE_PREFIX' and
            checkpoint.get('final') is True,'missing final checkpoint')
    require(type(checkpoint.get('generation')) is int and checkpoint['generation']>0,'invalid checkpoint generation')
    require(count(checkpoint.get('archived_outputs'),'checkpoint outputs')==accepted,'checkpoint does not cover all accepted outputs')
    storage=status.get('storage')
    require(isinstance(storage,dict) and storage.get('error') is None,'storage failure or missing storage evidence')
    require(storage.get('checkpoint_policy','periodic')==checkpoint_policy,'storage checkpoint policy mismatch')
    require(storage.get('checkpoint')==checkpoint,'status checkpoint differs from committed manifest')
    require(type(storage.get('accepted_outputs_not_checkpointed')) is int and
            storage['accepted_outputs_not_checkpointed']==0,'accepted outputs remain uncheckpointed')
    prefix={key:checkpoint.get(key) for key in ('archived_outputs','pairs_index_bytes','sources','artifacts')}
    require(storage.get('written_prefix')==prefix,'written prefix exceeds or differs from final checkpoint')
    sources=checkpoint.get('sources');source_status=status.get('sources')
    require(isinstance(sources,dict) and set(sources)==set(sides) and
            isinstance(source_status,dict) and set(source_status)==set(sides),'checkpoint source set mismatch')
    artifacts=checkpoint.get('artifacts')
    expected_artifacts={directory/'prior_config.json',root/'bootstrap.json'}
    require(isinstance(artifacts,list) and len(artifacts)==2 and all(isinstance(path,str) for path in artifacts),
            'bootstrap artifact checkpoint missing')
    require({project_path(ROOT,path) for path in artifacts}==expected_artifacts and
            all(path.is_file() for path in expected_artifacts),'bootstrap artifact files missing or mismatched')

    def read_index(path, expected_bytes, expected_rows, inspect_row):
        path=project_path(ROOT,path)
        require(type(expected_bytes) is int and 0<expected_bytes<=expected_rows*65536,'invalid index byte prefix')
        require(path.is_file() and path.stat().st_size==expected_bytes,path.name+' length differs from checkpoint')
        rows=consumed=0
        with path.open('rb') as stream:
            while True:
                line=stream.readline(65537)
                if not line:break
                require(len(line)<=65536 and line.endswith(b'\n'),'incomplete or oversized JSONL record')
                rows+=1;consumed+=len(line)
                require(rows<=expected_rows,'index contains extra rows')
                row=json.loads(line)
                require(isinstance(row,dict),'invalid JSONL row')
                inspect_row(row,rows)
        require(rows==expected_rows and consumed==expected_bytes and path.stat().st_size==expected_bytes,
                'index row count or final length differs from checkpoint')
        return {'rows':rows,'bytes':consumed}

    summary_sources={}
    for side in sides:
        record=sources[side];gate=source_status[side]
        require(isinstance(record,dict) and isinstance(gate,dict),'invalid source checkpoint/status')
        frames=count(record.get('frames'),side+' checkpoint frames')
        require(frames==accepted and count(gate.get('archived_frames'),side+' archived frames')==frames,
                side+' archive count mismatch')
        require(gate.get('state')=='STOPPED' and gate.get('failure_reason') is None and
                count(gate.get('published_frames'),side+' published frames')==published,side+' did not close consistently')
        total_bytes=0
        def source_row(row, number):
            nonlocal total_bytes
            require(type(row.get('archive_index')) is int and row['archive_index']==number and row.get('side')==side,
                    side+' source index order/identity mismatch')
            key=row.get('raw_key')
            require(isinstance(key,list) and len(key)==4 and key[0]==handle['session_id'],side+' source session mismatch')
            filename='frames/%08d.cdr'%number
            require(row.get('file')==filename,side+' CDR filename mismatch')
            size=count(row.get('cdr_bytes'),side+' CDR bytes',MAX_CDR_BYTES)
            require(size>0,'empty CDR envelope')
            envelope=project_path(ROOT,root/side/filename)
            require(envelope.is_file() and envelope.stat().st_size==size,side+' CDR size differs from index or envelope missing')
            total_bytes+=size
        source_index=read_index(root/side/'frames.jsonl',record.get('index_bytes'),frames,source_row)
        frames_dir=project_path(ROOT,root/side/'frames');seen=0
        with os.scandir(frames_dir) as entries:
            for entry in entries:
                match=re.fullmatch(r'([0-9]{8,})\.cdr',entry.name)
                require(match is not None and 1<=int(match[1])<=frames and entry.name=='%08d.cdr'%int(match[1]) and
                        not entry.is_symlink() and entry.is_file(follow_symlinks=False),
                        side+' unexpected or redirected CDR entry')
                seen+=1
        require(seen==frames,side+' missing CDR envelopes')
        summary_sources[side]={'frames':frames,'source_index':source_index,'cdr_bytes':total_bytes}
    require(count(status.get('archived_frames'),'total archived frames')==accepted*len(sides),
            'total source archive count mismatch')
    def pair_row(row,number):
        require(type(row.get('archive_output_index')) is int and row['archive_output_index']==number and
                row.get('session_id')==handle['session_id'] and row.get('mode')==mode,'pair index order/identity mismatch')
        members=row.get('sources')
        require(isinstance(members,list) and len(members)==len(sides) and all(isinstance(item,dict) for item in members),
                'pair source membership mismatch')
        require([item.get('side') for item in members]==list(sides),'pair source order mismatch')
        require(all(item.get('cdr_file')==side+'/frames/%08d.cdr'%number for item,side in zip(members,sides)),
                'pair does not refer to its archived source envelopes')
    pairs=read_index(root/'pairs.jsonl',checkpoint.get('pairs_index_bytes'),accepted,pair_row)
    return {'status':'FINAL_CHECKPOINT_VERIFIED','accepted_outputs':accepted,'archived_outputs':archived,
            'archive_publication_mode':status.get('archive_publication_mode','written_before_publish'),
            'publication_contract':storage.get('publication_contract'),
            'published_outputs':published,'archived_but_not_published_on_close':discarded,'pending_outputs':0,
            'checkpoint_generation':checkpoint['generation'],'checkpoint_final':True,
            'status_file':str(root/'status.json'),'checkpoint_file':str(root/'checkpoint.json'),
            'checkpoint_sha256':hash_file(root/'checkpoint.json'),'pairs_index':pairs,'sources':summary_sources,
            'cdr_contents_rehashed':False,'cdr_file_sizes_verified':True,
            'verification_scope':'Final sync manifest, closed status, complete source/pair indexes and CDR file sizes; '
                                 'does not repeat CDR content hashes or validate mapping accuracy.'}


def export(handle):
    from contextlib import closing
    import uuid
    from .mapping_app import normal_close
    def bag_counts(directory, odometry_source):
        metadata=directory/'bag/metadata.yaml'
        if not metadata.is_file():raise RuntimeError('原始录包尚未完成关闭')
        bag=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
        counts={row['topic_metadata']['name']:row['message_count'] for row in bag['topics_with_message_count']}
        required=['/wc_mapping/imu/source_frame','/wc_mapping/wheel/feedback_raw']
        required+=['/wc_mapping/app/'+name for name in required_outputs(odometry_source)]
        for side in (('left','right') if handle['mode']=='all' else (handle['mode'],)):
            required+=['/wc_mapping/lidar_'+side+'/source_frame','/wc_mapping/lidar_'+side+'/source_frame_filtered']
        if any(counts.get(topic,0)<=0 for topic in required):raise RuntimeError('必需硬件或建图录包话题为空')
        for relative in bag['relative_file_paths']:
            path=project_path(ROOT,directory/'bag'/relative)
            with closing(sqlite3.connect(path.as_uri()+'?mode=ro',uri=True)) as connection:
                connection.execute('PRAGMA query_only=ON')
                if connection.execute('PRAGMA integrity_check').fetchall()!=[('ok',)]:
                    raise RuntimeError('原始录包完整性失败')
        return counts
    if not normal_close(inspect(handle)):raise RuntimeError('异常退出的会话保留原件，不标记保存成功')
    input_archive=input_archive_summary(handle)
    directory=Path(handle['directory']);health=read_json(ROOT,directory/'health/status.json')
    runtime_config=read_json(ROOT,directory/'runtime_config.json')
    odometry_source=runtime_config.get('odometry_source','icp')
    from .mapping_monitor import required_outputs
    if health.get('odometry_source','icp')!=odometry_source:
        raise RuntimeError('建图里程计来源与会话快照不一致')
    if health.get('failure') or any(health.get('counts',{}).get(k,0)==0 for k in required_outputs(odometry_source)):
        raise RuntimeError('缺真实IMU/轮预测或3D/2D输出证据，原始数据已保留')
    db=project_path(ROOT,directory/'slam/rtabmap.db')
    log=project_path(ROOT,Path(handle['runtime'])/'process-1.log').read_text(errors='replace')
    if not any('Saving database/long-term memory...done!' in line and str(db) in line for line in log.splitlines()):
        raise RuntimeError('未确认原生地图数据库正常关闭')
    for suffix in ('-wal','-journal'):
        if Path(str(db)+suffix).exists():raise RuntimeError('地图数据库尚未关闭完成')
    before=hash_file(db)
    with closing(sqlite3.connect(db.as_uri()+'?mode=ro',uri=True)) as source:
        source.execute('PRAGMA query_only=ON')
        if source.execute('PRAGMA integrity_check').fetchall()!=[('ok',)]:raise RuntimeError('地图数据库完整性失败')
        nodes=source.execute('SELECT COUNT(*) FROM Node').fetchone()[0]
        if nodes==0:raise RuntimeError('没有地图节点')
    output=project_path(ROOT,directory/'export')
    if output.exists():
        if not output.is_dir():raise RuntimeError('既有 export 路径不是目录，保留原件')
        existing=None
        if (output/'result.json').is_file():
            try:existing=read_json(ROOT,output/'result.json')
            except (ValueError,UnicodeError):pass  # Interrupted JSON remains with its failed attempt.
        if isinstance(existing,dict) and existing.get('status')=='EXPORTED_EXPERIMENTAL_MAP':
            identity=(existing.get('session_id')==handle['session_id'] and existing.get('mode')==handle['mode']
                and existing.get('odometry_source', 'icp')==odometry_source and existing.get('database')==str(db)
                and existing.get('output')==str(output) and existing.get('database_sha256')==before
                and existing.get('retained_database_nodes')==nodes)
            if not identity:raise RuntimeError('既有成功导出身份或原数据库哈希不匹配；保留原件')
            files=existing.get('files')
            if not isinstance(files,dict) or not files or 'native_export_copy.db' not in files or 'export.log' not in files:
                raise RuntimeError('既有成功导出缺少完整文件清单；保留原件')
            expected=set(files)|{'result.json'}
            actual=set()
            for entry in output.iterdir():
                entry=project_path(output,entry)
                if not entry.is_file():raise RuntimeError('既有成功导出包含未登记目录；保留原件')
                actual.add(entry.name)
            if actual!=expected:raise RuntimeError('既有成功导出文件清单不一致；保留原件')
            if not any(name.endswith('.ply') for name in files) or not any(name.endswith('.yaml') for name in files):
                raise RuntimeError('既有成功导出缺少三维或二维地图；保留原件')
            for name,expected_file in files.items():
                if not isinstance(name,str) or Path(name).name!=name:
                    raise RuntimeError('既有成功导出文件名无效；保留原件')
                path=project_path(output,output/name)
                if (not isinstance(expected_file,dict) or type(expected_file.get('bytes')) is not int
                        or path.stat().st_size!=expected_file['bytes']
                        or hash_file(path)!=expected_file.get('sha256')):
                    raise RuntimeError('既有成功导出文件哈希不匹配；保留原件: '+name)
            bag_counts(directory,odometry_source)
            if hash_file(db)!=before:raise RuntimeError('验证既有导出时原数据库发生变化')
            from .map_quality import inspect_map_quality
            quality=inspect_map_quality(db,policy=runtime_config.get('map_quality_policy'))
            if quality.get('file_integrity',{}).get('status')!='PASS':
                raise RuntimeError('既有地图数据库重新检查未通过；保留原件')
            return {**existing,'export_reused':True,'map_quality':quality,
                    'quality_evidence_origin':'CURRENT_READONLY_INSPECTION_OF_REUSED_DATABASE'}
    export_storage=storage_status({'directory':directory})
    export_storage.update(database_bytes=db.stat().st_size, export_extra_free_bytes=EXPORT_EXTRA_FREE_BYTES,
                          policy='Database copy size plus 512 MiB estimated export reserve; not a capacity guarantee')
    export_storage['required_free_bytes']=export_storage['database_bytes']+EXPORT_EXTRA_FREE_BYTES
    if export_storage['free_bytes'] < export_storage['required_free_bytes']:
        raise RuntimeError('EXPORT_INSUFFICIENT_FREE_SPACE: 导出前可用空间不足以容纳数据库副本及 512 MiB 估计余量；'
                           '原始数据库保留。可用 %d B，需要至少 %d B；该余量不是导出容量保证。'
                           % (export_storage['free_bytes'],export_storage['required_free_bytes']))
    previous_partial=None
    if output.exists():
        from wc_maps.store import _rename_no_replace, _sync_directory
        retained=project_path(ROOT,directory/('export_failed_'+uuid.uuid4().hex))
        # Both resolved paths were checked inside this same owned session.
        _rename_no_replace(output,retained)
        _sync_directory(directory)
        previous_partial=str(retained)
    output.mkdir(exist_ok=False)
    with closing(sqlite3.connect(db.as_uri()+'?mode=ro',uri=True)) as source:
        source.execute('PRAGMA query_only=ON')
        with closing(sqlite3.connect(output/'native_export_copy.db')) as dest:source.backup(dest)
    command=['/opt/ros/humble/bin/rtabmap-export','--scan','--cloud','--map','--poses','--poses_format','11','--opt','2',
             '--voxel','.03','--output','map_3d','--output_dir',str(output),str(output/'native_export_copy.db')]
    with (output/'export.log').open('xb') as stream:
        result=subprocess.run(ros_command(command),cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=180)
    if result.returncode or hash_file(db)!=before:raise RuntimeError('3D地图导出失败，请查看export.log')
    vertices=0
    for path in output.glob('*.ply'):
        with path.open('rb') as f:
            for _ in range(200):
                line=f.readline(4096).strip()
                if line.startswith(b'element vertex '):vertices+=int(line.split()[-1])
                if line==b'end_header':break
    if vertices<=0:raise RuntimeError('3D点云导出为空')
    maps=[]
    for specification in output.glob('*.yaml'):
        description=yaml.safe_load(specification.read_text())
        if not isinstance(description,dict) or 'image' not in description:continue
        resolution=description.get('resolution');origin=description.get('origin')
        if not isinstance(resolution,(int,float)) or not math.isfinite(resolution) or resolution<=0:
            raise RuntimeError('原生二维地图分辨率无效')
        if not isinstance(origin,list) or len(origin)!=3 or not all(math.isfinite(v) for v in origin):
            raise RuntimeError('原生二维地图原点无效')
        image=project_path(output,output/description['image'])
        if not image.is_file() or image.stat().st_size<20:raise RuntimeError('原生二维地图图像缺失或为空')
        maps.append({'yaml':specification.name,'image':image.name,'resolution_m':resolution,'origin':origin})
    if not maps:raise RuntimeError('原生数据库没有导出可重载的二维地图 YAML/图像')
    grid={'source':'native_closed_database_export','optimization':'requested_stored_optimized_poses_with_native_fallback','maps':maps,
          'same_closed_database_as_3d':True,'navigation_validated':False}
    counts=bag_counts(directory,odometry_source)
    from .map_quality import inspect_map_quality
    quality=inspect_map_quality(db,policy=runtime_config.get('map_quality_policy'))
    write_new(output/'map_quality.json',json_bytes(quality))
    if quality.get('file_integrity',{}).get('status')!='PASS':
        raise RuntimeError('地图数据库文件检查未通过；诊断已保留，不标记导出成功')
    report={'status':'EXPORTED_EXPERIMENTAL_MAP','session_id':handle['session_id'],'mode':handle['mode'],
        'odometry_source':odometry_source,'icp_odometry_required':odometry_source=='icp',
        'output':str(output),'database':str(db),'database_sha256':before,'retained_database_nodes':nodes,
        'ply_vertices':vertices,'map_2d':grid,'recorded_topic_counts':counts,'health':health,'input_archive':input_archive,
        'export_storage_check':export_storage,'previous_partial_export':previous_partial,
        'formal_acceptance':False,'navigation_validated':False,'motion_route_verified':False,
        'map_quality':quality,
        'files':{p.name:{'bytes':p.stat().st_size,'sha256':hash_file(p)} for p in output.iterdir() if p.is_file()}}
    write_new(output/'result.json',json_bytes(report));return report


def save(handle,destination):
    from .mapping_save import save_session
    return save_session(ROOT,handle,destination,inspect=inspect,export=export)


def discard(handle):
    from .mapping_save import discard_session
    return discard_session(ROOT,handle,inspect=inspect,user_confirmed=True)


def watch(runtime):
    runtime=project_path(ROOT,runtime);p=read_json(ROOT,runtime/'plan.json')
    deadline=time.monotonic()+p['duration_s']+60 if p['duration_s'] else None
    while deadline is None or time.monotonic()<deadline:
        state=read_json(ROOT,runtime/'manifest.json')
        if state['state']!='RUNNING':return 0
        try:alive=ticks(p['owner_pid'])==p['owner_start_ticks']
        except (OSError,ValueError):alive=False
        if not alive:
            stop_registered(runtime/'manifest.json');return 0
        time.sleep(.5)
    stop_registered(runtime/'manifest.json');return 0


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--watch-owner',action='store_true',required=True)
    parser.add_argument('--runtime',type=Path,required=True);args=parser.parse_args()
    target();raise SystemExit(watch(args.runtime))
