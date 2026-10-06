"""Reconcile independent source evidence against index AND immutable bag bytes."""
from collections import Counter
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3

from .source_archive import digest, atomic_json, read_camera_frame


def rows(path):
    with Path(path).open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def key(row):
    return (str(row['source_id']), str(row['stream_epoch']), int(row['sequence']), str(row['representation']))


def audit_wheel_control(root, events, summary):
    """Manual capture may contain reviewed writes; ordinary capture may not."""
    root=Path(root)
    manifest=json.loads((root/'capture_manifest.json').read_text(encoding='utf-8')) if (root/'capture_manifest.json').exists() else {}
    manual=manifest.get('manual_drive') is True
    writes=[row for row in events if row.get('event') in ('wheel_io_complete','wheel_io_failed','best_effort_zero')
            and row.get('request_hex') != '010320ab0002be2b' and row.get('transmitted_bytes',0)>0]
    reported=summary.get('control_transmissions',0 if not manual else None)
    if type(reported) is not int or reported<0 or len(writes)!=reported:
        raise ValueError('wheel control transmissions do not match source transactions')
    if not manual:
        if reported: raise ValueError('read-only capture contains control transmissions')
        return {'control_transmissions':0,'manual_drive':False}
    configuration=root/'configuration/manual_runtime.json'
    runtime=json.loads(configuration.read_text(encoding='utf-8'))
    expected=manifest.get('input_hashes',{}).get('configuration/manual_runtime.json')
    if not expected or digest(configuration)!=expected or runtime.get('session_id')!=manifest.get('session_id'):
        raise ValueError('manual capture authorization snapshot identity/hash mismatch')
    if (runtime.get('manual_controls',{}).get('interaction_policy')!='hybrid_manual' or
        runtime.get('manual_controls',{}).get('arm_allowed') is not True or
        runtime.get('manual_authorization',{}).get('source')!='EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT' or
        summary.get('interaction_policy')!='hybrid_manual'):
        raise ValueError('manual capture missing explicit current-session authorization')
    from .mapping_wheel import validate_response
    for row in writes:
        if row.get('event')=='wheel_io_complete':
            validate_response(bytes.fromhex(row['request_hex']),bytes.fromhex(row['response_hex']))
    return {'control_transmissions':reported,'manual_drive':True,'authorization_snapshot':'configuration/manual_runtime.json'}


def audit_capture(root, profile, process_results, *, durability_complete=None):
    from .capture import CAMERA_PROFILES, source_selection
    selection = source_selection(profile)
    root = Path(root)
    issues, accounting = [], {}
    expected, actual = {}, {}
    manifest_path = root/'capture_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8')) if manifest_path.exists() else {}
    contract_path = root/'configuration/capture_contract.json'
    contract = None
    contract_invalid = False
    component_ids, source_stamps = {}, {}
    index_storage, retained_identities = {}, set()
    def issue(code, evidence, detail):
        issues.append(dict(code=code, evidence=str(evidence), detail=str(detail)))
    if contract_path.exists():
        try:
            from .capture_contract import validate_contract
            contract = validate_contract(json.loads(contract_path.read_text(encoding='utf-8')))
            if contract['session_id']!=manifest.get('session_id') or contract.get('profile')!=profile:
                raise ValueError('contract session/profile differs from manifest')
            required_logical=set(selection['selected_sources'])-{'wheel','ultrasonic'}
            required_logical|={'wheel_left','wheel_right'}
            if profile=='all_sensors':required_logical|={'ultrasonic_address_'+str(address) for address in (1,2,3,4)}
            if set(contract['sources'])!=required_logical:
                raise ValueError('contract does not cover exactly the selected logical sources')
            frozen_hash = manifest.get('input_hashes',{}).get('configuration/capture_contract.json')
            if not frozen_hash or digest(contract_path)!=frozen_hash:
                raise ValueError('capture contract hash differs from frozen manifest')
        except (OSError,ValueError,TypeError,KeyError) as error:
            contract_invalid = True
            contract = None
            issue('CAPTURE_CONTRACT_INVALID',contract_path,error)
    elif manifest.get('capture_contract_required') is True or manifest.get('capture_contract_path'):
        contract_invalid = True
        issue('CAPTURE_CONTRACT_MISSING',contract_path,'new capture contract absent')
    def assert_source_identity(logical, value, *, side=None, role=None):
        if contract is None: return
        spec = contract['sources'].get(logical)
        if (not spec or value.get('sensor_id',value.get('device_id',value.get('source_id')))!=spec['source_id']
                or side is not None and spec.get('side')!=side
                or side is not None and value.get('side',side)!=side
                or role is not None and spec.get('role')!=role
                or value.get('session_id',contract['session_id'])!=contract['session_id']):
            issue('SOURCE_IDENTITY_MISMATCH',logical,value)
    def check_lidar_pipeline(side, summary):
        if contract is None: return
        pipeline,sdk = summary.get('pipeline'),summary.get('sdk_queues')
        if not isinstance(pipeline,dict) or not isinstance(sdk,dict):
            issue('SDK_DROP_EVIDENCE_MISSING','lidar_'+side,'pipeline/sdk_queues missing');return
        fields = [(pipeline,k) for k in ('rejected_frames','rejected_bytes','pending_frames','pending_bytes')]
        fields += [(sdk,k) for k in ('raw_dropped','image_dropped','raw_pending_frames','raw_pending_bytes',
                                     'image_pending_frames','image_pending_bytes')]
        if (any(type(value.get(key)) is not int or value[key]!=0 for value,key in fields)
                or pipeline.get('failure_code') or pipeline.get('shutdown_deadline_exceeded') is not False
                or sdk.get('strict_delivery') is not True):
            issue('SOURCE_PIPELINE_DROPPED','lidar_'+side,dict(pipeline=pipeline,sdk_queues=sdk))
        stages=[pipeline.get(key) for key in ('accepted_frames','worker_processed','worker_journaled','worker_published')]
        stages.extend([summary.get('journal_frames'),summary.get('published_frames')])
        if (any(type(value) is not int or value<0 for value in stages) or len(set(stages))!=1
                or type(pipeline.get('callback_received')) is not int
                or pipeline['callback_received']!=pipeline.get('accepted_frames',-1)+pipeline.get('rejected_frames',-1)):
            issue('SOURCE_PIPELINE_COUNT_MISMATCH','lidar_'+side,pipeline)
        if any(pipeline.get(key) is not True for key in ('publication_acknowledged',
            'raw_publication_acknowledged','filtered_publication_acknowledged')):
            issue('SOURCE_PUBLICATION_ACK_UNCONFIRMED','lidar_'+side,pipeline)
    def add(row):
        identity = key(row)
        if identity in expected:
            issue('SOURCE_IDENTITY_DUPLICATE', 'sources', identity)
        expected[identity] = row['payload_sha256']
    def load(path):
        return json.loads(path.read_text(encoding='utf-8'))
    def checked_summary(path, count_field=None):
        value = load(path)
        if value.get('synchronized') is not True or value.get('closed_normally') is not True:
            issue('SOURCE_CLOSE_UNCONFIRMED', path, value)
        if count_field and value.get(count_field, 0) <= 0:
            issue('SOURCE_EMPTY', path, count_field)
        return value
    selection_path = root/'configuration/source_selection.json'
    if selection_path.exists():
        try:
            frozen = load(selection_path)
            if frozen != dict(profile=profile, **selection):
                raise ValueError('requested profile differs from frozen selected/excluded source scope')
        except (OSError, ValueError, TypeError) as exc:
            issue('CAPTURE_SOURCE_SELECTION_MISMATCH', 'configuration/source_selection.json', exc)
    required_processes = {'lidar','imu','wheel','recorder'}
    if profile in CAMERA_PROFILES:
        required_processes |= {name for name in selection['selected_sources'] if name.startswith('camera_')}
    if profile == 'all_sensors': required_processes.add('ultrasonic')
    for name in required_processes-set(process_results):
        issue('PROCESS_CLOSE_EVIDENCE_MISSING','capture_manifest.json',name)
    selected_processes = required_processes | {'manual_ui'}
    for name, rc in process_results.items():
        if name not in selected_processes: continue
        if rc != 0:
            issue('PROCESS_INCOMPLETE', 'logs/'+name+'.log', rc)
    for side in ('left', 'right'):
        try:
            dirs = list((root/'sources/lidar'/side).glob('*'))
            dirs = [p for p in dirs if p.is_dir()]
            if len(dirs) != 1:
                raise ValueError('expected one source epoch; got '+str(len(dirs)))
            directory = dirs[0]
            summary = checked_summary(directory/'source_summary.json', 'published_frames')
            assert_source_identity('lidar_'+side,summary,side=side)
            if summary.get('side',side)!=side:
                issue('SOURCE_SIDE_MISMATCH',directory,summary)
            check_lidar_pipeline(side,summary)
            count = 0
            stamps = []
            for row in rows(directory/'source_frames.jsonl'):
                count += 1
                component_ids['lidar_'+side] = row['sensor_id']
                assert_source_identity('lidar_'+side,row,side=side)
                if row.get('host_monotonic_ns') is not None:
                    stamps.append(row['host_monotonic_ns'])
                for representation in ('raw', 'filtered'):
                    add(dict(source_id=row['sensor_id'], stream_epoch=row['stream_epoch'], sequence=row['sequence'],
                             representation=representation, payload_sha256=row[representation+'_sha256']))
            if count != summary['journal_frames'] or count != summary['published_frames']:
                issue('SOURCE_TAIL_MISSING', directory, [count, summary])
            accounting['lidar_'+side] = dict(successfully_received=count, published=summary['published_frames'])
            source_stamps[side] = stamps
            accounting['lidar_'+side]['device_sdk_output_hz'] = ((count-1)*1e9/(stamps[-1]-stamps[0])
                if len(stamps)>1 and stamps[-1]>stamps[0] else None)
            accounting['lidar_'+side]['rate_basis'] = 'source callback host monotonic interval; not display Hz'
        except (OSError, ValueError, KeyError) as exc:
            issue('LIDAR_EVIDENCE_MISSING', 'sources/lidar/'+side, exc)
    try:
        directory = root/'sources/imu'
        summary = checked_summary(directory/'summary.json')
        if digest(directory/'events.jsonl') != summary['events_sha256']:
            raise ValueError('source journal hash differs')
        count, batches = 0, set()
        for row in rows(directory/'events.jsonl'):
            if row['event'] == 'byte_batch':
                if hashlib.sha256(bytes.fromhex(row['bytes_hex'])).hexdigest() != row['sha256']:
                    raise ValueError('IMU byte batch corrupt')
                batches.add(row['batch_id'])
            if row['event'] == 'frame':
                count += 1
                component_ids['imu'] = row['sensor_id']
                assert_source_identity('imu',row)
                if row['batch_id'] not in batches:
                    raise ValueError('IMU packet missing source read batch')
                add(dict(source_id=row['sensor_id'], stream_epoch=row['stream_epoch'], sequence=row['sequence'],
                         representation='packet', payload_sha256=row['packet_sha256']))
        if not count or count != summary['event_counts'].get('frame'):
            raise ValueError('missing IMU frame/tail evidence')
        accounting['imu'] = dict(successfully_received=count, published=count, byte_batches=len(batches))
    except (OSError, ValueError, KeyError) as exc:
        issue('IMU_EVIDENCE_INVALID', 'sources/imu', exc)
    try:
        events = list(rows(root/'sources/wheel_feedback.jsonl'))
        summary = checked_summary(root/'sources/wheel_summary.json')
        if summary['events_sha256'] != digest(root/'sources/wheel_feedback.jsonl'):
            raise ValueError('wheel final source hash mismatch')
        completed = [r for r in events if r.get('event') == 'capture_complete']
        transactions = [r for r in events if r.get('event') == 'transaction_complete']
        if len(completed) != 1 or not transactions or completed[0]['completed'] != len(transactions):
            raise ValueError('wheel capture_complete count mismatch/absent')
        owner_events = [row for row in events if row.get('event') not in ('journal_finalization','publication_ack')]
        if not owner_events or owner_events[-1].get('event') != 'lease_closed':
            raise ValueError('wheel lease closure missing')
        acknowledgements = [(index,row) for index,row in enumerate(events) if row.get('event')=='publication_ack']
        if contract is not None or acknowledgements:
            closures = [index for index,row in enumerate(events) if row.get('event')=='lease_closed']
            if len(acknowledgements)!=1 or len(closures)!=1:
                raise ValueError('exactly one wheel publication ACK and lease close required')
            ack_index,ack = acknowledgements[0]
            semantic_events=[row for row in events if row.get('event')!='journal_finalization']
            if (ack_index<=closures[0] or semantic_events[-1].get('event')!='publication_ack' or
                    ack.get('acknowledged') is not True or type(ack.get('timeout_s')) not in (int,float) or
                    not .05<=ack['timeout_s']<=10):
                raise ValueError('wheel publication ACK failed/missing or precedes lease close')
        if summary['completed'] != len(transactions) or summary['journal'].get('final_fsync_complete') is not True:
            raise ValueError('wheel post-fsync summary count mismatch or absent')
        for row in transactions:
            if row.get('status') != 'RESPONSE_VALID':
                raise ValueError('wheel transaction invalid')
            from wc_motion.protocol import parse_exchange
            response = bytes.fromhex(row['response_hex'])
            if hashlib.sha256(response).hexdigest() != row['response_sha256'] or row['request_hex'] != '010320ab0002be2b':
                raise ValueError('wheel request/response bytes differ from declared evidence')
            _, _, registers = parse_exchange(bytes.fromhex(row['request_hex']), response)
            if list(registers) != row['register_words_u16']:
                raise ValueError('wheel decoded registers differ from source response')
            add(dict(source_id=row['device_id'], stream_epoch=row['stream_epoch'], sequence=row['sequence'],
                     representation='transaction', payload_sha256=row['response_sha256']))
            component_ids['wheel'] = row['device_id']
            assert_source_identity('wheel_left',row,side='left')
            assert_source_identity('wheel_right',row,side='right')
        accounting['wheel'] = dict(successfully_received=len(transactions), published=completed[0]['completed'],
                                  **audit_wheel_control(root,events,summary))
    except (OSError, ValueError, KeyError) as exc:
        issue('WHEEL_EVIDENCE_INVALID', 'sources/wheel_feedback.jsonl', exc)
    if profile in CAMERA_PROFILES:
        from wc_cameras.config import ROLES
        for role in ROLES:
            try:
                directory = root/'sources/cameras'/role
                summary = checked_summary(directory/'summary.json', 'captured_frames')
                assert_source_identity('camera_'+role,summary,role=role)
                if contract is not None:
                    from .capture_contract import identity_matches
                    camera_configuration = load(directory/'capture_configuration.json')
                    spec = contract['sources'].get('camera_'+role,{})
                    if (camera_configuration.get('camera',{}).get('role')!=role
                            or not identity_matches(spec.get('expected_identity',{}),camera_configuration.get('identity'))
                            or spec.get('observed_preflight_identity') is not None
                            and not identity_matches(spec['observed_preflight_identity'],camera_configuration.get('identity'))):
                        raise ValueError('camera physical USB identity/role differs from frozen contract')
                if digest(directory/'frames.jsonl') != summary['index_sha256']:
                    raise ValueError('camera index hash mismatch')
                for chunk in summary['chunks']:
                    if digest(directory/chunk['path']) != chunk['sha256']:
                        raise ValueError('camera chunk hash mismatch')
                frames = list(rows(directory/'frames.jsonl'))
                identities = {(r['stream_epoch'], r['sequence']) for r in frames}
                if len(frames) != summary['captured_frames'] or len(identities) != len(frames):
                    raise ValueError('camera missing/duplicate frame or tail')
                for row in frames:
                    assert_source_identity('camera_'+role,row,role=role)
                    if row.get('stream_epoch') != summary['stream_epoch']:
                        raise ValueError('camera stream epoch differs from final summary')
                    read_camera_frame(directory,row)
                accounting['camera_'+role] = dict(successfully_received=len(frames), persisted=len(frames),
                    retained=len(frames), exposure_completeness='NOT_VERIFIED', representation=summary['representation'])
            except (OSError, ValueError, KeyError) as exc:
                issue('CAMERA_ARCHIVE_INVALID', 'sources/cameras/'+role, exc)
    if profile == 'all_sensors':
        try:
            directory = root/'sources/ultrasonic'
            summary = checked_summary(directory/'summary.json')
            if contract is not None:
                from .capture_contract import identity_matches
                observed = load(directory/'identity.json')
                if (observed.get('session_id')!=contract['session_id'] or observed.get('source_id')!='ultrasonic'
                        or observed.get('evidence_basis')!='CURRENT_UDEV_AND_OPEN_DESCRIPTOR_IDENTITY'
                        or not isinstance(observed.get('opened_identity'),dict)
                        or type(observed.get('verified_monotonic_ns')) is not int
                        or observed['verified_monotonic_ns']<=0):
                    raise ValueError('actual ultrasonic opened/udev identity evidence absent')
                for logical,spec in contract['sources'].items():
                    if spec['kind']=='ultrasonic' and not identity_matches(spec['expected_identity'],observed):
                        raise ValueError('ultrasonic bus transport differs from frozen contract: '+logical)
            if digest(directory/'events.jsonl') != summary['events_sha256']:
                raise ValueError('ultrasonic journal hash mismatch')
            journal_events = list(rows(directory/'events.jsonl'))
            transactions = [row for row in journal_events if row.get('event')=='transaction'
                            or contract is None and row.get('event') is None and 'address' in row]
            finalizations = [row for row in journal_events if row.get('event')=='source_finalization']
            if len(transactions)+len(finalizations)!=len(journal_events):
                raise ValueError('unexpected ultrasonic source journal event')
            if contract is not None:
                if len(finalizations)!=1 or journal_events[-1].get('event')!='source_finalization':
                    raise ValueError('exactly one final ultrasonic source finalization required at journal tail')
                finalization = finalizations[0]
                count=len(transactions)
                counters=[finalization.get('completed_transactions'),finalization.get('published_transactions')]
                if any(type(value) is not int or value!=count for value in counters):
                    raise ValueError('ultrasonic final completed/published transaction count mismatch')
                if (finalization.get('session_id')!=contract['session_id'] or
                        {row.get('stream_epoch') for row in transactions}!={finalization.get('stream_epoch')}):
                    raise ValueError('ultrasonic final session/epoch differs from source transactions')
                ack=finalization.get('publication_ack')
                if (finalization.get('lease_closed') is not True or finalization.get('cleanup_errors')!=[] or
                        not isinstance(ack,dict) or ack.get('status')!='PASS' or ack.get('acknowledged') is not True):
                    raise ValueError('ultrasonic final lease/publication ACK/cleanup unconfirmed')
                expected_counts={'transaction':count,'source_finalization':1}
                if (summary.get('event_counts')!=expected_counts or
                        any(type(summary.get(key)) is not int or summary[key]!=len(journal_events)
                            for key in ('accepted_records','persisted_records')) or
                        type(summary.get('rejected_records')) is not int or summary['rejected_records']!=0):
                    raise ValueError('ultrasonic final source journal counters differ from retained records')
            if {r['address'] for r in transactions} != {1, 2, 3, 4}:
                raise ValueError('not all four addresses sampled')
            for row in transactions:
                from .ultrasonic_capture import request, decode
                response = bytes.fromhex(row['response_hex'])
                if hashlib.sha256(response).hexdigest() != row['response_sha256'] or bytes.fromhex(row['request_hex']) != request(row['address']):
                    raise ValueError('ultrasonic request/response bytes differ from source evidence')
                if decode(response, row['address'])['status'] != row['status']:
                    raise ValueError('ultrasonic protocol status differs from actual response')
                add(dict(source_id='ultrasonic', stream_epoch=row['stream_epoch'], sequence=row['sequence'],
                         representation='transaction', payload_sha256=row['response_sha256']))
                if row['status'] != 'VALID_RESPONSE':
                    issue('ULTRASONIC_'+row['status'], directory, 'address='+str(row['address']))
            accounting['ultrasonic'] = dict(successfully_received=sum(r['status']=='VALID_RESPONSE' for r in transactions),
                                          published=len(transactions))
            component_ids['ultrasonic'] = 'ultrasonic'
        except (OSError, ValueError, KeyError) as exc:
            issue('ULTRASONIC_EVIDENCE_INVALID', 'sources/ultrasonic', exc)
    try:
        summary = checked_summary(root/'recorder_summary.json', 'persisted')
        if digest(root/'records.jsonl') != summary['index_sha256']:
            raise ValueError('recording index hash mismatch')
        index_cdr = Counter()
        for row in rows(root/'records.jsonl'):
            identity = key(row)
            if identity in actual:
                issue('RECORDER_DUPLICATE', 'records.jsonl', identity)
            actual[identity] = row['payload_sha256']
            index_storage[identity] = (row['topic'], row['storage_stamp_ns'], row['cdr_sha256'])
            index_cdr[(row['topic'], row['storage_stamp_ns'], row['cdr_sha256'])] += 1
        bag_cdr = Counter()
        for database in (root/'bag').glob('*.db3'):
            with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro', uri=True)) as conn:
                for topic, stamp, data in conn.execute('SELECT topics.name,messages.timestamp,messages.data FROM messages JOIN topics ON topics.id=messages.topic_id'):
                    bag_cdr[(topic, stamp, hashlib.sha256(data).hexdigest())] += 1
        retained_identities = {identity for identity, record in index_storage.items() if bag_cdr[record] > 0}
        if not (root/'bag/metadata.yaml').is_file() or not bag_cdr or bag_cdr != index_cdr:
            issue('BAG_INDEX_MISMATCH', 'bag', 'stored CDR differs or bag closing metadata missing')
        if sum(index_cdr.values()) != summary['persisted'] or summary['received'] != summary['persisted']:
            issue('RECORDER_TAIL_MISSING', 'recorder_summary.json', summary)
    except (OSError, ValueError, KeyError, sqlite3.Error) as exc:
        issue('RECORDER_EVIDENCE_INVALID', 'recorder_summary.json', exc)
    missing = sorted(set(expected)-set(actual))
    extra = sorted(set(actual)-set(expected))
    corrupted = [identity for identity in expected.keys() & actual.keys() if expected[identity] != actual[identity]]
    if missing or extra or corrupted:
        issue('SOURCE_RECORDING_MISMATCH', 'records.jsonl', dict(missing=missing[:100], extra=extra[:100], corrupted=corrupted[:100],
              missing_count=len(missing), extra_count=len(extra), corrupted_count=len(corrupted)))
    for component, source_id in component_ids.items():
        if component not in accounting:
            continue
        keys = [identity for identity in actual if identity[0] == source_id]
        retained = [identity for identity in retained_identities if identity[0] == source_id]
        if component.startswith('lidar_'):
            raw = {(k[1],k[2]) for k in keys if k[3]=='raw'}
            filtered = {(k[1],k[2]) for k in keys if k[3]=='filtered'}
            retained_raw = {(k[1],k[2]) for k in retained if k[3]=='raw'}
            retained_filtered = {(k[1],k[2]) for k in retained if k[3]=='filtered'}
            accounting[component].update(raw_persisted=len(raw),filtered_persisted=len(filtered),
                                         persisted=len(raw & filtered),retained=len(retained_raw & retained_filtered))
        elif component in accounting:
            accounting[component].update(persisted=len(keys),retained=len(retained))
    left, right = source_stamps.get('left',[]), source_stamps.get('right',[])
    i = j = pairs = 0
    while i < len(left) and j < len(right):
        delta = left[i]-right[j]
        if abs(delta)<=50_000_000:
            pairs += 1; i += 1; j += 1
        elif delta < 0:
            i += 1
        else:
            j += 1
    rates = dict(pair_candidate_count=pairs,pair_window_ns=50_000_000,
                 pairing_basis='arrival-only diagnostic; no geometric or physical synchronization claim',
                 map_consumption_hz=None,display_hz=None,
                 inactive_reason='independent capture starts no SLAM; optional preview state is independent',
                 display_rate_basis='NOT_MEASURED',preview_status_path='capture_preview_status.json')
    transport_complete = not issues
    window_complete = None
    window_report = None
    if contract is not None:
        acquisition = manifest.get('acquisition_window')
        if acquisition is None or acquisition == {}:
            window_complete = False
            detail = 'common capture window was not established; window verification was not run'
            window_report = dict(status='NOT_ESTABLISHED', evaluated=False,
                window_complete=False, full_window_verified=False, reason=detail,
                requested_source_duration_s=manifest.get('requested_source_duration_s'),
                stop_reason=manifest.get('stop_reason'))
            issue('CAPTURE_WINDOW_NOT_ESTABLISHED','acquisition_window',detail)
        else:
            try:
                from .capture_contract import collect_source_evidence, evaluate_capture_window
                if not isinstance(acquisition,dict):
                    raise ValueError('acquisition window must be an object')
                if any(type(acquisition.get(field)) is not int for field in (
                        'start_monotonic_ns','end_monotonic_ns','requested_duration_ns')):
                    raise ValueError('acquisition window endpoints and duration must be present integers')
                if acquisition['requested_duration_ns']!=acquisition['end_monotonic_ns']-acquisition['start_monotonic_ns']:
                    raise ValueError('requested window duration differs from frozen endpoints')
                requested_seconds = manifest.get('requested_source_duration_s')
                if requested_seconds is not None and (type(requested_seconds) is not int
                        or acquisition['requested_duration_ns']!=requested_seconds*1_000_000_000):
                    raise ValueError('acquisition window differs from original requested duration')
                evidence = collect_source_evidence(root,contract)
                window_report = evaluate_capture_window(evidence,contract,acquisition['start_monotonic_ns'],
                                                         acquisition['end_monotonic_ns'])
                window_complete = window_report['window_complete']
                for failure in window_report['issues']:
                    issue(failure['code'],failure['source'],'requested common host-arrival window')
            except (OSError,ValueError,KeyError,TypeError) as error:
                window_complete = False
                issue('CAPTURE_WINDOW_EVIDENCE_INVALID','acquisition_window',error)
    elif contract_invalid:
        window_complete = False
    legacy = contract is None and not contract_invalid
    complete = transport_complete and not issues and (legacy and durability_complete is not False
        or window_complete is True and durability_complete is True)
    return dict(recording_complete=complete, status='COMPLETE' if complete else 'PARTIAL',
                transport_complete=transport_complete,window_complete=window_complete,
                durability_complete=durability_complete,full_window_verified=window_complete is True,
                completion_pending=['durability'] if not legacy and transport_complete and window_complete is True and durability_complete is None else [],
                completeness_contract='LEGACY_CAPTURED_BYTES_ONLY' if legacy else 'IDENTITY_TRANSPORT_COMMON_WINDOW_DURABILITY',
                window_diagnostics=window_report,
                **selection,
                issues=issues, source_accounting=accounting, expected_ros_records=len(expected),
                frequency_diagnostics=rates,
                retained_ros_records=len(retained_identities), absolute_accuracy='待评估',
                scope='SDK output onward; camera decoded BGR8; no network-packet or exposure completeness claim')


def verify_archive(root):
    root = Path(root).resolve(strict=True)
    manifest_path = root/'capture_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    durability = manifest.get('durability_evidence',{}).get('complete')
    if durability is None:
        durability = manifest.get('durability_complete')
    report = audit_capture(root, manifest['profile'], manifest.get('process_exit_codes', {'capture':'NO_FINAL_PROCESS_EVIDENCE'}),
                           durability_complete=durability)
    for field in ('selected_sources', 'excluded_sources', 'completeness_scope'):
        if field in manifest and manifest[field] != report[field]:
            report['issues'].append(dict(code='CAPTURE_SOURCE_SELECTION_MISMATCH', evidence='capture_manifest.json#/'+field,
                                         detail='manifest source selection differs from the explicit capture profile'))
    report['original_manifest_sha256'] = digest(manifest_path)
    for relative, expected in manifest.get('input_hashes', {}).items():
        path = root/relative
        if Path(relative).is_absolute() or '..' in Path(relative).parts or not path.resolve().is_relative_to(root):
            raise ValueError('archive reference escapes dataset')
        if not path.is_file() or path.is_symlink() or digest(path) != expected:
            report['issues'].append(dict(code='RETAINED_FILE_HASH_MISMATCH', evidence=relative, detail='missing or changed'))
    if manifest.get('recording_complete') is not True:
        report['issues'].append(dict(code='ORIGINAL_CAPTURE_INCOMPLETE',evidence='capture_manifest.json',
                                     detail='re-audit cannot erase original capture failures'))
    if report['issues']:
        report.update(status='PARTIAL',recording_complete=False)
    return report


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, help='Optional new audit report outside original archive')
    args = parser.parse_args(argv)
    report = verify_archive(args.dataset)
    if args.output:
        if args.output.exists() or args.output.resolve().is_relative_to(args.dataset.resolve()):
            raise ValueError('audit output must be new and outside immutable capture')
        atomic_json(args.output, report)
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['recording_complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
