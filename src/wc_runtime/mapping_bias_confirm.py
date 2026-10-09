"""Turn a screened candidate into an explicitly operator-confirmed calibration.

This command is offline and never opens a sensor or modifies a running prior.
Screening is not stationary truth; only the operator can confirm the displayed
physical interval. The resulting optional file is used by a subsequent session.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

from .mapping_planar import validate_confirmed_bias
from .device_bindings import identity_token


def _read(path, maximum=262144):
    if not path.is_file() or path.stat().st_size > maximum:
        raise ValueError('Missing or oversized calibration evidence: '+str(path))
    raw = path.read_bytes()
    value = json.loads(raw.decode('utf-8'), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
    return raw, value


def proposal(session):
    session = Path(session).resolve(strict=True)
    status_raw, status = _read(session/'prior/status.json')
    _, config = _read(session/'runtime_config.json')
    identity_token(config.get('imu_sensor_id'), prefix='H30-')
    if config.get('source_mode') != 'real':
        raise ValueError('Expected an identified real H30 session')
    candidate = status.get('gyro_bias', {}).get('latest_candidate')
    if not isinstance(candidate, dict) or candidate.get('screening_passed') is not True:
        raise ValueError('没有完整合格零偏候选；需在 --mapping true 会话中观察静止片段。纯预览不启动IMU/prior。')
    for key in ('start_stamp_ns', 'end_stamp_ns', 'sample_count'):
        if type(candidate.get(key)) is not int or candidate[key] <= 0:
            raise ValueError('Invalid candidate interval/count')
    if candidate['end_stamp_ns']-candidate['start_stamp_ns'] < 5_000_000_000 or candidate['sample_count'] < 200:
        raise ValueError('Candidate has insufficient recorded duration/samples')
    raw_sha = hashlib.sha256(status_raw).hexdigest()
    bias = validate_confirmed_bias({
        'status': 'INDEPENDENTLY_CONFIRMED', 'sensor_id': config['imu_sensor_id'],
        'bias_native_rad_s': candidate.get('bias_native_rad_s'),
        'evidence_id': str(config['session_id'])+':'+str(candidate['start_stamp_ns'])+':'+str(candidate['end_stamp_ns']),
        'evidence_sha256': raw_sha, 'source': 'operator_visual_confirmation',
        'evidence_note': 'Operator attests this entire displayed interval was physically stationary. Software screening alone does not prove stationarity or calibration accuracy.',
    })
    evidence = {'schema_version': 1, 'source_session': str(session), 'candidate': candidate,
                'prior_status_sha256': raw_sha, 'prior_status': status,
                'physical_stationarity_confirmed_by_operator': False,
                'automatic_stationarity_proven': False, 'holdout_accuracy_validated': False}
    evidence['candidate_token']=hashlib.sha256(json.dumps(candidate,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return bias, evidence


def save_confirmation(session, output, *, confirmed=False, candidate_token=None):
    bias, evidence = proposal(session)
    if not confirmed:
        return {'status': 'CONFIRMATION_REQUIRED', 'candidate': evidence['candidate'],
                'candidate_token': evidence['candidate_token'],
                'interval_local': [datetime.fromtimestamp(evidence['candidate'][k]/1e9,timezone.utc).astimezone().isoformat()
                                   for k in ('start_stamp_ns','end_stamp_ns')],
                'session': evidence['source_session'], 'hardware_started': False}
    if candidate_token != evidence['candidate_token']:
        raise ValueError('候选已变化或未指定核对凭据；请重新查看并确认完整静止区间，不会保存另一个窗口')
    output = Path(output).absolute()
    if output.suffix != '.json' or not output.parent.is_dir():
        raise ValueError('Choose a new .json in an existing configuration directory')
    if any(p.is_symlink() or getattr(p, 'is_junction', lambda: False)() for p in (output, *output.parents)):
        raise ValueError('Linked calibration destinations are not allowed')
    evidence_path = output.with_name(output.stem+'.evidence.json')
    if output.exists() or evidence_path.exists():
        raise ValueError('Calibration/evidence destination already exists; originals are preserved')
    evidence['physical_stationarity_confirmed_by_operator'] = True
    evidence['confirmed_at_utc'] = datetime.now(timezone.utc).isoformat()
    evidence_bytes = (json.dumps(evidence, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    bias['evidence_sha256'] = hashlib.sha256(evidence_bytes).hexdigest()
    bias['evidence_id'] += ':'+evidence_path.name
    bias = validate_confirmed_bias(bias)
    with evidence_path.open('xb') as stream:
        stream.write(evidence_bytes)
    with output.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(bias, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    return {'status': 'OPERATOR_CONFIRMED_CALIBRATION_SAVED', 'file': str(output),
            'evidence': str(evidence_path), 'bias_native_rad_s': bias['bias_native_rad_s'],
            'applied_to_running_session': False, 'holdout_accuracy_validated': False,
            'message': '将建图配置gyro_bias_config设为此文件的工程内路径，下次会话生效；未修改运行中位姿。'}


def main(argv=None):
    parser = argparse.ArgumentParser(description='查看陀螺零偏候选；只在你确认对应完整时段轮椅静止后保存。不会启动设备。')
    parser.add_argument('session', type=Path, nargs='?')
    parser.add_argument('--calibration-jsonl', type=Path, help='Original timestamped native IMU/wheel rows; 10+10+10 protocol')
    parser.add_argument('--capture-dataset', type=Path, help='Extract native IMU and CRC-checked zero wheel registers directly from capture bag; no extrinsics needed')
    parser.add_argument('--stationary-evidence', type=Path, help='Independent physical stationarity declaration covering all 30 s')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--confirm-stationary', action='store_true',
                        help='明确声明候选显示的完整时段内轮椅物理静止；零轮速本身不足以确认')
    parser.add_argument('--candidate-token', help='先查看候选得到的凭据；与--confirm-stationary配合，防止确认了不同窗口')
    args = parser.parse_args(argv)
    if args.calibration_jsonl or args.capture_dataset:
        if args.session or not args.stationary_evidence or not args.output:
            parser.error('--calibration-jsonl requires --stationary-evidence and --output; no session argument')
        if args.calibration_jsonl and args.capture_dataset:
            parser.error('choose --calibration-jsonl or --capture-dataset')
        try:
            result = save_holdout_calibration(args.calibration_jsonl, args.stationary_evidence, args.output,
                                             capture_dataset=args.capture_dataset)
            print(json.dumps(result, ensure_ascii=False, allow_nan=False))
            return 0
        except (ValueError, OSError, KeyError, TypeError) as error:
            print(json.dumps({'status':'CALIBRATION_NOT_SAVED','error':str(error)},ensure_ascii=False))
            return 1
    if args.session is None:
        parser.error('session or --calibration-jsonl required')
    if args.confirm_stationary and args.output is None:
        parser.error('--confirm-stationary requires a new --output .json')
    if args.confirm_stationary and not args.candidate_token:
        parser.error('--confirm-stationary requires the reviewed --candidate-token')
    try:
        token=args.candidate_token
        confirmed=args.confirm_stationary
        if args.output is not None and not confirmed and sys.stdin.isatty():
            reviewed=save_confirmation(args.session,args.output)
            print(json.dumps(reviewed,ensure_ascii=False,indent=2))
            print('请核对上方完整时间段。只有你亲自确认该时段轮椅保持静止，才输入 CONFIRM 保存；其它输入取消。')
            confirmed=input().strip()=='CONFIRM'
            token=reviewed['candidate_token']
        result = save_confirmation(args.session, args.output or Path('unused.json'), confirmed=confirmed,candidate_token=token)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 0
    except (ValueError, OSError, KeyError, TypeError) as error:
        print(json.dumps({'status': 'CALIBRATION_NOT_SAVED', 'error': str(error)}, ensure_ascii=False))
        return 1


def save_holdout_calibration(rows_path, evidence_path, output, *, capture_dataset=None):
    from .mapping_bias import calibrate_stationary
    evidence_path, output = Path(evidence_path), Path(output).absolute()
    if capture_dataset:
        rows, source_provenance = capture_calibration_rows(capture_dataset)
        rows_raw=('\n'.join(json.dumps(r,sort_keys=True,allow_nan=False) for r in rows)+'\n').encode('utf-8')
    else:
        rows_path=Path(rows_path)
        if not rows_path.is_file() or rows_path.stat().st_size > 128*1024*1024:
            raise ValueError('missing or oversized calibration JSONL')
        rows_raw = rows_path.read_bytes()
        rows = [json.loads(line) for line in rows_raw.decode('utf-8').splitlines() if line.strip()]
        source_provenance={'path':str(rows_path),'sha256':hashlib.sha256(rows_raw).hexdigest()}
    evidence_raw, declaration = _read(evidence_path)
    # Only identities attached to this input/evidence are eligible. Never read
    # this machine's live binding to label an archived calibration.
    identities = {identity_token(r['sensor_id'], prefix='H30-') for r in rows if 'sensor_id' in r}
    for value in (source_provenance.get('sensor_id'), declaration.get('sensor_id')):
        if value is not None:
            identities.add(identity_token(value, prefix='H30-'))
    if len(identities) != 1:
        raise ValueError('calibration requires one explicit, consistent input sensor_id')
    sensor_id = identities.pop()
    if any('sensor_id' not in row for row in rows) and not declaration.get('sensor_id') and not source_provenance.get('sensor_id'):
        raise ValueError('unidentified rows require a sensor_id in the independent declaration')
    source_provenance['sensor_id'] = sensor_id
    declaration['evidence_sha256'] = hashlib.sha256(evidence_raw).hexdigest()
    result = calibrate_stationary(rows,declaration)
    if result['status'] != 'HOLDOUT_PASSED':
        raise ValueError('10+10+10 calibration rejected: '+','.join(result['rejection_reasons']))
    result.update(input_rows_sha256=hashlib.sha256(rows_raw).hexdigest(),
                  source_provenance=source_provenance,
                  independent_declaration_sha256=hashlib.sha256(evidence_raw).hexdigest(),
                  confirmed_at_utc=datetime.now(timezone.utc).isoformat())
    destination = output.with_name(output.stem+'.evidence.json')
    if output.suffix != '.json' or not output.parent.is_dir() or output.exists() or destination.exists():
        raise ValueError('choose new calibration/evidence files in an existing directory')
    if any(p.is_symlink() or getattr(p,'is_junction',lambda:False)() for p in (output,*output.parents)):
        raise ValueError('linked calibration destinations not allowed')
    result_raw=(json.dumps(result,ensure_ascii=False,allow_nan=False,indent=2)+'\n').encode('utf-8')
    bias = validate_confirmed_bias({'status':'INDEPENDENTLY_CONFIRMED','sensor_id':sensor_id,
        'bias_native_rad_s':result['bias_native_rad_s'],'evidence_id':declaration['evidence_id'],
        'evidence_sha256':hashlib.sha256(result_raw).hexdigest(),'source':declaration['source'],
        'evidence_note':'10 s warmup + 10 s estimate + independent 10 s held-out residual screening; thresholds experimental.'})
    with destination.open('xb') as stream: stream.write(result_raw)
    with output.open('x',encoding='utf-8') as stream:
        json.dump(bias,stream,ensure_ascii=False,allow_nan=False,indent=2); stream.write('\n')
    return {'status':'HOLDOUT_CALIBRATION_SAVED','file':str(output),'evidence':str(destination),
            'protocol':'SOURCE_TIME_10_10_10','bias_native_rad_s':bias['bias_native_rad_s'],
            'applied_to_running_session':False,'hardware_started':False,
            'holdout_accuracy_validated':False,'holdout_residual_screening_passed':True}


def capture_calibration_rows(dataset):
    """Native bias calibration remains possible while motion geometry is blocked."""
    import sqlite3
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from wc_motion.protocol import parse_exchange
    from wc_motion.feedback_transport import QUERY
    from .source_time import SourceClock,event_key
    from .compare_replay import digest
    root=Path(dataset).resolve(strict=True); files=sorted((root/'bag').glob('*.db3'))
    if not files: raise ValueError('capture has no original SQLite bag')
    identity_path = root/'configuration/capture_contract.json'
    if identity_path.exists():
        identity_raw, contract = _read(identity_path)
        imu_spec = contract['sources']['imu']
        sensor_id = identity_token(imu_spec['source_id'], prefix='H30-')
        if imu_spec['expected_identity']['sensor_id'] != sensor_id:
            raise ValueError('frozen capture IMU identity is inconsistent')
    else:
        # Legacy archives had an explicit identity in their manifest.
        identity_path = root/'capture_manifest.json'
        identity_raw, manifest = _read(identity_path, maximum=16*1024*1024)
        sensor_id = identity_token(manifest.get('imu_sensor_id'), prefix='H30-')
    events=[]; hashes={}
    for path in files:
        hashes[str(path.relative_to(root))]=digest(path)
        with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
            db.execute('PRAGMA query_only=ON')
            for topic_id,topic,kind in db.execute('SELECT id,name,type FROM topics'):
                category={'/wc_mapping/imu/source_frame':'imu','/wc_mapping/wheel/feedback_raw':'wheel'}.get(topic)
                if category is None: continue
                expected='wc_interfaces/msg/H30Frame' if category=='imu' else 'std_msgs/msg/String'
                if kind!=expected: raise ValueError('invalid calibration source message type')
                cls=get_message(kind)
                for message_id,payload in db.execute('SELECT id,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topic_id,)):
                    message=deserialize_message(payload,cls)
                    if category=='wheel':
                        record=json.loads(message.data); source_stamp=record['stamp_ns']; mono=record['receive_monotonic_ns']; sequence=record['sequence']
                    else:
                        if message.sensor_id != sensor_id:
                            raise ValueError('IMU message identity differs from frozen capture identity')
                        source_stamp=message.host_receive_time.sec*10**9+message.host_receive_time.nanosec
                        mono=int(message.host_monotonic_ns); sequence=int(message.frame_sequence)
                    events.append(dict(category=category,stamp_ns=source_stamp,monotonic_ns=mono,sequence=sequence,
                        message=message,bag=path.name,message_id=message_id,cdr_sha256=hashlib.sha256(payload).hexdigest()))
                    if len(events)>1_000_000: raise ValueError('calibration event limit exceeded')
    clock=SourceClock(events)
    events=[clock.canonicalize(event) for event in events]; events.sort(key=event_key)
    rows=[]; latest_wheel=None
    for event in events:
        message=event['message']
        if event['category']=='wheel':
            record=json.loads(message.data); request=bytes.fromhex(record['request_hex']); response=bytes.fromhex(record['response_hex'])
            if request!=QUERY or record.get('status')!='RESPONSE_VALID': raise ValueError('invalid wheel stationarity-screening transaction')
            _,_,words=parse_exchange(request,response)
            if hashlib.sha256(response).hexdigest()!=record['response_sha256']: raise ValueError('wheel response hash mismatch')
            latest_wheel=(event['stamp_ns'],all(word==0 for word in words))
        elif latest_wheel is not None:
            if message.sensor_id!=sensor_id or not message.angular_velocity_valid or not message.linear_acceleration_valid:
                raise ValueError('identified valid native H30 gyro/acceleration required')
            a,g=message.imu.linear_acceleration,message.imu.angular_velocity
            # Conservative zero-register screen needs no guessed radius/scale.
            # A nonzero register gives a finite failing screen marker (1 m/s).
            rows.append(dict(sensor_id=sensor_id,stamp_ns=event['stamp_ns'],sequence=event['sequence'],stream_epoch=message.stream_epoch,
                original_source_stamp_ns=event['original_source_stamp_ns'],monotonic_ns=event['monotonic_ns'],
                gyro_native=[g.x,g.y,g.z],acceleration_native=[a.x,a.y,a.z],
                wheel_velocity_m_s=0. if latest_wheel[1] else 1.,
                wheel_age_s=(event['stamp_ns']-latest_wheel[0])*1e-9,cdr_sha256=event['cdr_sha256']))
    return rows,{'dataset':str(root),'bag_sha256':hashes,'clock':clock.report(),
        'sensor_id':sensor_id,'identity_evidence':str(identity_path.relative_to(root)),
        'identity_evidence_sha256':hashlib.sha256(identity_raw).hexdigest(),
        'wheel_screen':'CRC-checked exactly zero raw registers; no radius/track/scale used',
        'independent_physical_stationarity_required':True,'geometry_required':False}


if __name__ == '__main__':
    raise SystemExit(main())
