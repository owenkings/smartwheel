"""Evidence-based session health. Reading reports never opens or commands devices.

Lifecycle, observed data, calibration and retention are independent dimensions.
The optional ROS watch process publishes diagnostics, never control commands.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

SCHEMA = 'wc_runtime_health_v1'
MAX_DOCUMENT_BYTES = 2_000_000
MAX_ISSUES = 64
TROUBLESHOOTING = {
    'CAMERA_NOT_ENUMERATED': ('/cameras',
        ('预期 USB 连接或供电未建立。', '相机所接物理 USB 端口与 by-path 配置不一致。'),
        ('只读核对预期 by-path 和当前枚举；由操作者检查线缆与供电后复测，不自动换绑其他 video 节点。',)),
    'ULTRASONIC_RESPONSE_TIMEOUT': ('/ultrasonic',
        ('串口、波特率、地址或接线与实际设备不一致。', '设备供电或收发方向未建立。'),
        ('先核对该事务的地址、请求、收到的字节与超时；只读确认串口归属，获授权后复测对应地址。',)),
    'ULTRASONIC_CRC_ERROR': ('/ultrasonic', ('串口参数或链路干扰导致收到的帧不完整。',),
        ('核对原始请求/响应字节与 CRC 计算，保留失败事务；不要把错误响应作为距离。',)),
    'RECORDING_GAP': ('/recording_profile', ('归档写入失败、队列丢弃或尾部未完成。',),
        ('比较源成功捕获、落盘和留存计数；检查源 summary、recorder_summary 与离线审计。',)),
}


def _read(root, relative, errors):
    path = root / relative
    if not path.exists():
        return {}
    try:
        if path.is_symlink() or not path.is_file() or path.stat().st_size > MAX_DOCUMENT_BYTES:
            raise ValueError('report is not a bounded regular file')
        def reject_constant(value): raise ValueError('nonfinite JSON constant: '+value)
        value = json.loads(path.read_text(encoding='utf-8'), parse_constant=reject_constant)
        if not isinstance(value, dict):
            raise ValueError('report must be a JSON object')
        return value
    except (OSError, ValueError, UnicodeError) as error:
        errors.append((relative, str(error)))
        return {}


def issue(code, component, observed, *, impact=(), hypotheses=(), parameter_pointer=None,
          evidence=(), retest=(), severity='WARN', phase='runtime'):
    """Observed facts never become a confirmed cause just because an issue exists."""
    guide = TROUBLESHOOTING.get(code)
    if guide:
        parameter_pointer = parameter_pointer or guide[0]
        hypotheses = hypotheses or guide[1]
        retest = retest or guide[2]
    return {'code': code, 'component': component, 'severity': severity, 'phase': phase,
            'observed': observed, 'capability_impact': list(impact),
            'possible_causes': [{'status': 'HYPOTHESIS', 'description': h} for h in hypotheses],
            'parameter_pointer': parameter_pointer, 'evidence_refs': list(evidence),
            'retest': list(retest), 'automatic_device_action': False}


def _count(value, *names):
    for name in names:
        n = value.get(name)
        if type(n) is int and n >= 0:
            return n
    return None


def _cap(status='UNKNOWN', reasons=()):
    return {'status': status, 'reasons': list(reasons)}


def _mapping(value, pointer, issues):
    if isinstance(value, dict): return value
    issues.append(issue('REPORT_SCHEMA_INVALID', pointer, '报告字段应为对象。', evidence=(pointer,), severity='ERROR'))
    return {}


def collect_session_health(session_root, capability_assessment=None, *, capture_manifest=None):
    """Read an existing session; no directory creation, imports of drivers or scans.

    capability_assessment may be the geometry gate's result, or a snapshot with
    a capabilities mapping. Missing evidence stays UNKNOWN, not VALIDATED.
    """
    root = Path(session_root).absolute()
    errors = []
    documents = {name: _read(root, name, errors) for name in (
        'session.json', 'runtime_config.json', 'health/status.json', 'prior/status.json',
        'wheel_status.json', 'input/status.json', 'cameras/status.json',
        'hardware_setup.json', 'dataset/capture_manifest.json',
        'capture_manifest.json', 'configuration/runtime_config.json', 'configuration/hardware_setup.json',
        'recorder_summary.json', 'capability_assessment.json', 'retention.json', 'dataset/audit.json',
        'export/map_quality.json')}
    if capture_manifest is not None:
        existing = documents['capture_manifest.json']
        if (not isinstance(capture_manifest, dict) or not existing.get('session_id')
                or capture_manifest.get('session_id') != existing['session_id']):
            raise ValueError('prospective capture manifest must match the existing root session')
        # Finalization generates health before committing COMPLETE to disk.
        # The prospective view never modifies the durable source manifest.
        documents['capture_manifest.json'] = dict(capture_manifest)
    session, config = documents['session.json'], documents['runtime_config.json']
    capture_ref = 'capture_manifest.json' if documents['capture_manifest.json'] else 'dataset/capture_manifest.json'
    capture = documents[capture_ref]
    profile = capture.get('profile', config.get('recording_profile'))
    selection = {}
    if capture and profile in ('mapping_core', 'mapping_cameras', 'all_sensors'):
        from .capture import source_selection
        selection = source_selection(profile)
    excluded_sources = set(selection.get('excluded_sources', []))
    recording_capability = {'mapping_cameras': 'mapping_cameras_recording',
                            'mapping_core': 'mapping_core_recording'}.get(profile, 'all_sensors_recording')
    session_id = session.get('session_id') or config.get('session_id') or capture.get('session_id')
    issues = []
    if not session and not capture:
        issues.append(issue('SESSION_REPORT_MISSING', 'session', '缺少可读取的 session.json；会话身份未确认。',
                            evidence=('session.json',), severity='ERROR'))
    for name, error in errors:
        issues.append(issue('REPORT_UNREADABLE', name, error, evidence=(name,),
                            retest=('核对该会话报告是否完整；保留原件，不覆盖失败记录。',)))
    for name, value in list(documents.items()):
        if value.get('session_id') and session_id and value['session_id'] != session_id:
            issues.append(issue('SESSION_ID_MISMATCH', name, '报告属于另一会话。',
                                evidence=(name,), severity='ERROR'))
            documents[name] = {}
    monitor, prior, wheel, cameras = (documents[n] for n in
        ('health/status.json', 'prior/status.json', 'wheel_status.json', 'cameras/status.json'))
    inputs = documents['input/status.json']
    session = documents['session.json']
    config = documents['runtime_config.json'] or documents['configuration/runtime_config.json']
    capture = documents[capture_ref]
    assessment = capability_assessment or documents['capability_assessment.json']
    capabilities = dict(_mapping(assessment.get('capabilities', {}), 'capability_assessment.json#/capabilities', issues)) if isinstance(assessment, dict) else {}
    capabilities = {name: _mapping(value, 'capability_assessment.json#/capabilities/'+name, issues)
                    for name, value in capabilities.items()}
    capabilities.setdefault('source_preview', _cap('UNKNOWN'))
    capabilities.setdefault('source_recording', _cap('UNKNOWN'))
    capabilities.setdefault('motion_fusion', _cap('UNVALIDATED', ('安装与轮参数须有当前硬件的标定依据。',)))
    capabilities.setdefault('camera_preview', _cap('UNKNOWN'))
    capabilities.setdefault('all_sensors_recording', _cap('UNKNOWN'))
    if capture and profile != 'all_sensors':
        capabilities['all_sensors_recording'] = _cap('NOT_REQUESTED', ('本次采集范围不包含全部传感器。',))
    if 'ultrasonic' in excluded_sources:
        capabilities['ultrasonic_recording'] = _cap('NOT_REQUESTED', ('超声波明确排除；未预检、启动、订阅或校验。',))
    components = {}
    for name, report in [('motion', monitor), ('prior', prior), ('wheel', wheel), ('cameras', cameras)]:
        components[name] = {'lifecycle': report.get('status', report.get('state', 'UNKNOWN')),
                            'evidence_ref': {'motion': 'health/status.json', 'prior': 'prior/status.json',
                                'wheel': 'wheel_status.json', 'cameras': 'cameras/status.json'}[name]}
        failure = report.get('failure') or report.get('problem')
        if failure:
            issues.append(issue('COMPONENT_FAILURE', name, str(failure), severity='ERROR',
                                evidence=(components[name]['evidence_ref'],)))
    data = {'counts': _mapping(monitor.get('counts', {}), 'health/status.json#/counts', issues),
            'age_s': _mapping(monitor.get('age_s', {}), 'health/status.json#/age_s', issues),
            'waiting_outputs': monitor.get('waiting_outputs', []),
            'motion_coverage_complete': prior.get('motion_coverage_complete'),
            'archived_outputs': inputs.get('archived_outputs'),
            'published_outputs_not_confirmed_archived': inputs.get('published_outputs_not_confirmed_archived')}
    waiting = data['waiting_outputs']
    stale = [name for name, age in data['age_s'].items()
             if type(age) in (int, float) and math.isfinite(age) and age > 3]
    if waiting:
        issues.append(issue('WAITING_OUTPUTS', 'mapping', '尚未观察到：' + ', '.join(waiting),
                            impact=('motion_fusion',), evidence=('health/status.json#/waiting_outputs',),
                            retest=('先分别核对来源计数和订阅端，不能仅凭进程存在认定有数据。',)))
    if stale and monitor.get('status') == 'RUNNING':
        issues.append(issue('STALE_OUTPUTS', 'mapping', '输出超过 3 秒未更新：' + ', '.join(stale),
                            evidence=('health/status.json#/age_s',),
                            retest=('核对源端、发布端和消费端计数；旧画面不可作为当前观测。',)))
    slots = _mapping(cameras.get('slots', {}), 'cameras/status.json#/slots', issues)
    camera_degraded = False
    for role, row in slots.items():
        row = _mapping(row, 'cameras/status.json#/slots/'+role, issues)
        components['camera/' + role] = {'lifecycle': row.get('state', 'UNKNOWN'),
                                       'evidence_ref': 'cameras/status.json#/slots/' + role}
        if row.get('error') or row.get('state') in ('UNAVAILABLE', 'EXITED'):
            camera_degraded = True
            code = row.get('error_code', 'CAMERA_UNAVAILABLE')
            text = str(row.get('error') or row.get('restart_reason') or '未获得有效图像。')
            capture_stop = row.get('capture_stop', {})
            if isinstance(capture_stop, dict) and capture_stop.get('error'):
                text += '; ' + str(capture_stop['error'])
            logs = row.get('log_tail', [])
            words = ('FileNotFoundError', 'No such file', 'by-path disappeared')
            if isinstance(logs, list):
                for line in logs:
                    if any(word in str(line) for word in words):
                        text += '; ' + str(line); break
            if any(word in text for word in ('No such file', 'FileNotFoundError', 'by-path disappeared')):
                code = 'CAMERA_NOT_ENUMERATED'
            issues.append(issue(code, 'camera/' + role, text,
                impact=('camera_preview', recording_capability),
                hypotheses=('设备未接入或供电异常。', '预期 USB 端口与实际连接不一致。'),
                parameter_pointer='/cameras/' + role,
                evidence=('cameras/status.json#/slots/' + role,),
                retest=('只读比较预期 by-path 与当前枚举；不可自动替换为其他 /dev/video 节点。',)))
    if slots:
        capabilities['camera_preview'] = _cap('DEGRADED' if camera_degraded else 'PROCESS_READY',
                                             ('进程就绪不等于持续图像帧率已验证。',))
    sources = capture.get('source_accounting', capture.get('sources', {}))
    if isinstance(sources, list):
        sources = {str(i): row for i, row in enumerate(sources)}
    sources = _mapping(sources, 'dataset/capture_manifest.json#/source_accounting', issues)
    recording_sources = {}
    for name, row in sources.items():
        if name in excluded_sources: continue
        if not isinstance(row, dict):
            continue
        observed = _count(row, 'successfully_received', 'captured_frames', 'successful_reads', 'observed_count', 'received')
        written = _count(row, 'persisted', 'persisted_frames', 'written_frames', 'archived_frames', 'written_count', 'written')
        missing = _count(row, 'missing_frames', 'dropped_frames', 'not_archived')
        recording_sources[name] = {'state': row.get('status', row.get('state', 'UNKNOWN')),
            'observed': observed, 'written': written, 'missing': missing,
            'retained': row.get('retained'), 'evidence_ref': capture_ref+'#/source_accounting/' + name}
        state = str(row.get('status', row.get('state', '')))
        failed = state in ('FAILED', 'UNAVAILABLE', 'NOT_ENUMERATED', 'RESPONSE_TIMEOUT', 'NOT_IMPLEMENTED')
        if failed or (missing is not None and missing > 0) or (observed is not None and written is not None and written < observed):
            code = row.get('error_code') or ('RECORDING_GAP' if not failed else state)
            issues.append(issue(code, name, str(row.get('error') or row.get('reason') or state or '源端与落盘计数不一致。'),
                impact=(recording_capability,), evidence=(recording_sources[name]['evidence_ref'],),
                retest=('分别对账来源成功读取、发布、落盘与留存，勿将录包缺失推断成硬件断流。',)))
    capture_issues = capture.get('issues', [])
    if not isinstance(capture_issues, list):
        issues.append(issue('REPORT_SCHEMA_INVALID', 'recording', '采集审计 issues 字段不是列表。', severity='ERROR'))
        capture_issues = [{}]
    for row in capture_issues:
        if isinstance(row, dict):
            component = 'recording'
            reference = str(row.get('evidence', 'dataset/capture_manifest.json'))
            if str(row.get('code', '')).startswith('CAMERA_'):
                for role in ('left_front', 'right_front', 'left_side', 'right_side'):
                    if role in reference: component = 'camera/' + role
            issues.append(issue(str(row.get('code', 'CAPTURE_AUDIT_FAILED')), component, str(row.get('detail', '采集审计未通过。')),
                impact=(recording_capability,), evidence=(str(row.get('evidence', 'dataset/capture_manifest.json')),),
                severity='ERROR', retest=('按源端、落盘、留存三个环节核对计数和哈希，再执行离线审计。',)))
    for source, check in _mapping(capture.get('preflight', {}), capture_ref+'#/preflight', issues).items():
        if source in excluded_sources: continue
        if isinstance(check, dict) and check.get('status') == 'BLOCKED':
            text = str(check.get('reason', '采集预检未通过。'))
            camera = source.startswith('camera_')
            code = ('CAMERA_NOT_ENUMERATED' if camera and any(word in text for word in
                ('FileNotFoundError', 'No such file', 'by-path disappeared', 'does not exist', 'not found')) else 'CAPTURE_PREFLIGHT_BLOCKED')
            component = 'camera/'+source[len('camera_'):] if camera else source
            issues.append(issue(code, component, text, impact=(recording_capability,),
                evidence=(capture_ref+'#/preflight/'+source,), parameter_pointer='/cameras' if camera else '/'+source))
    declared_complete = capture.get('recording_complete')
    complete = declared_complete
    accounting_failed = any((row['missing'] or 0) > 0 or (row['observed'] is not None and row['written'] is not None
                        and row['observed'] != row['written']) for row in recording_sources.values())
    if complete is True and (accounting_failed or capture_issues):
        complete = False
        issues.append(issue('RECORDING_COMPLETENESS_CONTRADICTION', 'recording', '完整标记与源计数或审计问题不一致。',
                            evidence=('dataset/capture_manifest.json',), severity='ERROR'))
    if capture:
        capabilities['source_recording'] = _cap('OBSERVED' if complete is True else 'INCOMPLETE')
        capabilities[recording_capability] = _cap('COMPLETE' if complete is True else 'INCOMPLETE')
    retention = documents['retention.json']
    recording = {'profile': profile, 'recording_complete': complete, 'declared_recording_complete': declared_complete,
        **selection,
        'sources': recording_sources, 'status': capture.get('status', 'NOT_REQUESTED' if not config.get('mapping_enabled') else 'UNKNOWN'),
        'camera_images_recorded': cameras.get('images_recorded_to_bag', False),
        'camera_archive_observed': any(name.startswith('camera_') and (row['written'] or 0) > 0 for name,row in recording_sources.items()),
        'camera_archive_scope': '实际成功捕获的 BGR8 帧；不宣称设备曝光或 USB 链路从未丢帧。',
        'preview_rate_is_archive_rate': False,
        'retention': retention.get('raw_retention', capture.get('raw_retention', 'UNKNOWN')),
        'capture_manifest_ref': capture_ref if capture else None,
        'recorder_summary': documents['recorder_summary.json'],
        'retention_profile': config.get('retention_profile', 'experiment'),
        'audit': documents['dataset/audit.json']}
    if capture and complete is not True:
        issues.append(issue('RECORDING_INCOMPLETE', 'recording', '全部请求来源的完整记录尚未确认。',
                            impact=(recording_capability,), evidence=(capture_ref,)))
    quality = {'status': 'UNVALIDATED', 'formal_acceptance': False, 'navigation_validated': False,
               'output_state': 'WAITING' if waiting else ('STALE_OUTPUT' if stale else
                   ('OBSERVED' if monitor.get('map_points', 0) else 'UNKNOWN')),
               'geometry_reference_verified': False}
    if documents['export/map_quality.json']:
        quality['closed_database_checks']=documents['export/map_quality.json']
        qualification=quality['closed_database_checks'].get('movement_map_qualification',{})
        if qualification.get('status')!='QUALIFIED_EXPERIMENTAL_CANDIDATE':
            issues.append(issue('MOVING_MAP_NOT_QUALIFIED','map_quality',
                '地图文件可以保存，但移动量、连通图或有效扫描尚未满足实验地图判据。',
                impact=('moving_map_review',), evidence=('export/map_quality.json',),
                parameter_pointer='/map_quality_policy',
                retest=('按报告逐项检查移动量、图连接及每节点扫描；独立几何精度仍需现场测量。',)))
    blocked = [name for name, value in capabilities.items() if isinstance(value, dict) and value.get('status') == 'BLOCKED']
    if blocked:
        issues.append(issue('CALIBRATION_REQUIRED', 'motion_fusion', '当前几何 gate 阻止运动融合。',
                            impact=tuple(blocked), parameter_pointer='/calibration',
                            evidence=('capability_assessment.json',),
                            retest=('先采集和标定缺失项目；源坐标预览与源录包不依赖完整外参。',)))
    return {'schema': SCHEMA, 'schema_version': 1, 'session_id': session_id,
        'session_root': str(root), 'updated_unix_ns': time.time_ns(),
        'lifecycle': monitor.get('status', capture.get('status', session.get('status', 'UNKNOWN'))),
        'session_profile_status': session.get('status', config.get('status', 'UNKNOWN')),
        'capabilities': capabilities, 'components': components, 'data': data,
        'parameter_provenance': {'parameters': assessment.get('parameters', config.get('hardware_setup_provenance', {})),
            'validation_level': assessment.get('validation_level', 'CONFIG_ONLY'),
            'source_documents': assessment.get('source_documents', []),
            'unresolved_components': assessment.get('unresolved_components', []),
            'limitations': assessment.get('limitations', []),
            'calibration_evidence_is_independent': True} if isinstance(assessment, dict) else {},
        'recording': recording, 'map_quality': quality, 'issues': issues[:MAX_ISSUES],
        'issues_truncated': len(issues) > MAX_ISSUES,
        'summary_zh': '地图几何未验证；' + ('存在需检查项（%d）。' % len(issues) if issues else '当前报告未发现异常，不等于已完成实机验收。'),
        'hardware_actions_performed': False}


def diagnosis_zh(report):
    lines = ['# 会话诊断', '', '会话：' + str(report.get('session_id') or '未知'),
             '', report['summary_zh'], '', '运行生命周期：' + str(report['lifecycle']),
             '地图质量：UNVALIDATED；保存成功与几何/运动验收分别记录。', '',
             'OBSERVED 表示已有运行观测；CONFIG_ONLY 表示只检查了配置，尚未独立标定；原因假设不能当作事实。', '',
             '| 能力 | 状态 | 证据层级 | 原因 |', '|---|---|---|---|']
    for name, value in report['capabilities'].items():
        lines.append('| %s | %s | %s | %s |' % (name, value.get('status', 'UNKNOWN'), value.get('validation_level', 'OBSERVATION_ONLY'),
                                           '；'.join(map(str, value.get('reasons', []))).replace('|', '/')))
    lines += ['', '参数来源与未确认关系：', json.dumps(report.get('parameter_provenance', {}), ensure_ascii=False, indent=2),
              '', '录制和留存：', json.dumps(report.get('recording', {}), ensure_ascii=False, indent=2)]
    for item in report['issues']:
        lines += ['', '## ' + item['code'] + ' / ' + item['component'],
                  '观察：' + item['observed'], '影响：' + '、'.join(item['capability_impact']),
                  '参数：' + str(item['parameter_pointer'] or '无'),
                  '证据：' + '、'.join(item['evidence_refs'])]
        lines += ['待查假设：' + cause['description'] for cause in item['possible_causes']]
        lines += ['复测：' + instruction for instruction in item['retest']]
    lines += ['', '本报告仅解释已有证据，不自动修改设备或标定参数。', '']
    return '\n'.join(lines)


def write_health_report(session_root, report=None, *, capture_manifest=None):
    """Write only reports into an already existing owned session directory."""
    root = Path(session_root).absolute()
    if root.is_symlink() or not root.is_dir():
        raise ValueError('existing regular session directory required')
    if report is not None and capture_manifest is not None:
        raise ValueError('provide either a report or a prospective capture manifest')
    value = report or collect_session_health(root, capture_manifest=capture_manifest)
    for name, text in [('health.json', json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)),
                       ('diagnosis_zh.md', diagnosis_zh(value))]:
        temporary = root / ('.' + name + '.tmp-' + uuid.uuid4().hex)
        try:
            with temporary.open('x', encoding='utf-8') as stream:
                stream.write(text); stream.flush()
            os.replace(temporary, root / name)
        finally:
            if temporary.exists(): temporary.unlink()
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', required=True, type=Path)
    parser.add_argument('--check', action='store_true', help='read only; no ROS or file writes')
    parser.add_argument('--watch', action='store_true', help='publish independent health; never controls')
    args = parser.parse_args(argv)
    if args.check:
        print(json.dumps(collect_session_health(args.session_root), ensure_ascii=False, allow_nan=False))
        return 0
    if not args.watch:
        write_health_report(args.session_root)
        return 0
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('wc_runtime_health')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, '/wc_mapping/health', qos)
    stopped = [False]
    def stop(*_): stopped[0] = True
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    try:
        while not stopped[0] and rclpy.ok():
            report = write_health_report(args.session_root)
            publisher.publish(String(data=json.dumps(report, ensure_ascii=False, allow_nan=False)))
            rclpy.spin_once(node, timeout_sec=1.)
    finally:
        write_health_report(args.session_root)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
