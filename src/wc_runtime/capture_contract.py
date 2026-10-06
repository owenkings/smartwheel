"""Hardware-free capture identity, readiness and common arrival-window contracts.

Readiness is a small sink/source observation, never proof of persistence. Window
checks use original host monotonic samples; no exposure/device synchronization
or measurement accuracy is inferred. These helpers never write configuration.
"""
import json
from pathlib import Path
import re

GAP_NS = {'lidar':500_000_000,'imu':100_000_000,'wheel':500_000_000,
          'camera':200_000_000,'ultrasonic':3_000_000_000}
ROLES = ('left_front','right_front','left_side','right_side')


def _json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def _identifier(value, label='identifier'):
    if not isinstance(value,str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,119}',value):
        raise ValueError('invalid '+label)
    return value


def _ns(value, label, *, positive=False):
    if type(value) is not int or value < (1 if positive else 0):
        raise ValueError(label+' must be integer host monotonic nanoseconds')
    return value


def build_capture_contract(configuration_dir, profile, session_id, *, imu_sensor_id,
                           expected_identities=None):
    """Build from frozen configuration; caller supplies the reviewed IMU identity."""
    from .capture import CAMERA_PROFILES, source_selection
    configuration_dir = Path(configuration_dir)
    source_selection(profile)
    _identifier(session_id,'session_id')
    sources = {}
    def add(logical,kind,source_id,identity,**fields):
        sources[logical] = dict(kind=kind,source_id=source_id,max_gap_ns=GAP_NS[kind],
                               expected_identity=identity,**fields)
    for side in ('left','right'):
        text = (configuration_dir/'lidar'/(side+'.yaml')).read_text(encoding='utf-8')
        values = {}
        for key in ('side','expected_serial','device_ip','receive_ip'):
            found = re.search(r'^\s*'+key+r':\s*([^\s#]+)\s*(?:#.*)?$',text,re.MULTILINE)
            if not found: raise ValueError('missing frozen lidar '+key)
            values[key] = found.group(1).strip('\"\'')
        if values['side'] != side: raise ValueError('frozen lidar side mismatch')
        add('lidar_'+side,'lidar',values['expected_serial'],values,side=side,
            physical_role_status='CONFIGURATION_BOUND_SERIAL')
    add('imu','imu',_identifier(imu_sensor_id,'imu_sensor_id'),
        {'sensor_id':imu_sensor_id},physical_role_status='IDENTIFIED_NATIVE_FRAME')
    wheel = _json(configuration_dir/'wheel_feedback.json')
    for index,side in enumerate(('left','right')):
        add('wheel_'+side,'wheel',wheel['device_id'],
            {k:wheel[k] for k in ('device','expected_by_id','hardware_serial','usb_vid','usb_pid')},
            side=side,register_index=index,register_address=0x20AB+index,
            physical_role_status='LEGACY_MAPPING_UNVALIDATED',units_status='UNVALIDATED')
    if profile in CAMERA_PROFILES:
        cameras = _json(configuration_dir/'cameras.json')
        for camera in cameras['cameras']:
            role = camera['role']
            port = camera['port']
            identity = dict(device=f'/dev/v4l/by-path/platform-3610000.usb-usb-0:3.{port}:1.0-video-index0',
                identity={'ID_VENDOR_ID':camera['vid'],'ID_MODEL_ID':camera['pid'],
                          'ID_SERIAL_SHORT':camera['serial'],
                          'ID_PATH':f'platform-3610000.usb-usb-0:3.{port}:1.0'})
            add('camera_'+role,'camera',role,identity,role=role,port=port,
                physical_role_status=cameras['mapping_status'])
    if profile == 'all_sensors':
        ultra = _json(configuration_dir/'ultrasonic.json')
        for address in ultra['addresses']:
            add('ultrasonic_address_'+str(address),'ultrasonic','ultrasonic',
                {k:ultra[k] for k in ('device','expected_by_id','expected_usb_path','usb_vid','usb_pid')},
                address=address,physical_role_status='UNKNOWN',units_status='UNVERIFIED')
    if expected_identities:
        for logical,identity in expected_identities.items():
            if logical not in sources or not isinstance(identity,dict):
                raise ValueError('unknown/nonobject expected transport identity')
            # Actual preflight supplements configured transport identity, rather
            # than silently replacing the reviewed serial or USB port binding.
            sources[logical]['observed_preflight_identity'] = identity
    return validate_contract(dict(schema_version=1,session_id=session_id,profile=profile,sources=sources,
        time_source='host_monotonic_ns',scope='HOST_ARRIVAL_SUCCESSFULLY_CAPTURED',
        physical_synchronization_verified=False))


def validate_contract(contract):
    if not isinstance(contract,dict) or contract.get('schema_version') != 1:
        raise ValueError('capture contract schema_version 1 required')
    _identifier(contract.get('session_id'),'session_id')
    sources = contract.get('sources')
    if not isinstance(sources,dict) or not sources:
        raise ValueError('nonempty contract sources required')
    for logical,spec in sources.items():
        _identifier(logical,'logical source')
        if not isinstance(spec,dict) or spec.get('kind') not in GAP_NS:
            raise ValueError('unknown capture source kind')
        _identifier(spec.get('source_id'),'source_id')
        if (type(spec.get('max_gap_ns')) is not int or not 0 < spec['max_gap_ns'] <= GAP_NS[spec['kind']]
                or not isinstance(spec.get('expected_identity'),dict)):
            raise ValueError('explicit identity and reviewed or stricter gap bound required')
        if spec['kind']=='wheel' and spec.get('register_index') not in (0,1):
            raise ValueError('wheel register index required')
        if spec['kind']=='ultrasonic' and type(spec.get('address')) is not int:
            raise ValueError('ultrasonic address required')
    return contract


def identity_matches(expected, actual):
    """Expected identity is a recursive subset, retaining actual driver metadata."""
    if isinstance(expected,dict):
        return isinstance(actual,dict) and all(key in actual and identity_matches(value,actual[key])
                                             for key,value in expected.items())
    return type(expected) is type(actual) and expected == actual


def read_source_readiness(dataset_root, contract):
    """Bounded small JSON reads only: safe to poll without rereading frame logs."""
    validate_contract(contract)
    root = Path(dataset_root).resolve()
    valid,missing,invalid = {},[],{}
    for logical,spec in contract['sources'].items():
        path = root/'ready'/(logical+'.json')
        try:
            if not path.is_file():
                missing.append(logical); continue
            if path.is_symlink() or not path.resolve().is_relative_to(root):
                raise ValueError('linked readiness path')
            data = path.read_bytes()
            if len(data)>65536: raise ValueError('readiness marker too large')
            value = json.loads(data)
            first = _ns(value.get('first_valid_monotonic_ns'),'first',positive=True)
            last = _ns(value.get('last_valid_monotonic_ns'),'last',positive=True)
            if (value.get('schema_version')!=1 or value.get('session_id')!=contract['session_id']
                    or value.get('logical_source')!=logical or value.get('source_id')!=spec['source_id']
                    or not isinstance(value.get('stream_epoch'),str) or not value['stream_epoch']
                    or value.get('ready') is not True or value.get('error')
                    or type(value.get('valid_count')) is not int or value['valid_count']<1 or last<first):
                raise ValueError('readiness identity/count/state mismatch')
            valid[logical]=value
        except (OSError,ValueError,TypeError,KeyError) as error:
            invalid[logical]=str(error)
    all_ready = not missing and not invalid and len(valid)==len(contract['sources'])
    return dict(ready=all_ready,all_ready=all_ready,missing=missing,invalid=invalid,sources=valid,
                observed_common_ready_ns=max((v['first_valid_monotonic_ns'] for v in valid.values()),default=None))


def _rows(path):
    with Path(path).open(encoding='utf-8') as stream:
        for line_number,line in enumerate(stream,1):
            yield json.loads(line),str(path),line_number


def iter_source_samples(dataset_root, contract):
    """Finalization scans original journals into logical motor/address rows."""
    validate_contract(contract)
    root = Path(dataset_root)
    for logical,spec in contract['sources'].items():
        kind = spec['kind']
        if kind=='lidar':
            epochs = [path for path in (root/'sources/lidar'/spec['side']).glob('*') if path.is_dir()]
            if len(epochs)!=1: raise ValueError('expected one lidar epoch for '+logical)
            iterator = _rows(epochs[0]/'source_frames.jsonl')
        elif kind=='imu': iterator=_rows(root/'sources/imu/events.jsonl')
        elif kind=='wheel': iterator=_rows(root/'sources/wheel_feedback.jsonl')
        elif kind=='camera': iterator=_rows(root/'sources/cameras'/spec['role']/'frames.jsonl')
        else: iterator=_rows(root/'sources/ultrasonic/events.jsonl')
        for row,path,line_number in iterator:
            if kind=='imu' and row.get('event')!='frame': continue
            if kind=='wheel' and row.get('event')!='transaction_complete': continue
            if kind=='ultrasonic' and row.get('address')!=spec['address']: continue
            sequence = row['sequence']
            monotonic = row.get('receive_monotonic_ns') if kind=='wheel' else row.get('host_monotonic_ns')
            value = dict(logical_source=logical,source_id=row.get('sensor_id',row.get('device_id',row.get('source_id'))),
                stream_epoch=row['stream_epoch'],sequence=sequence,host_monotonic_ns=monotonic,
                evidence=str(Path(path).relative_to(root))+':'+str(line_number),
                session_id=row.get('session_id'),payload_sha256=row.get('payload_sha256',row.get('response_sha256',row.get('packet_sha256'))))
            if kind=='wheel':
                words=row['register_words_u16']
                if len(words)!=2: raise ValueError('wheel pair registers absent')
                value.update(register_address=spec['register_address'],register_value_u16=words[spec['register_index']])
            if kind=='ultrasonic': value.update(address=row['address'],response_value_u16=row.get('response_value_u16'))
            if kind=='lidar':
                value.update(side=row.get('side',spec['side']),raw_valid_count=row.get('raw_valid_count'),
                             filtered_valid_count=row.get('filtered_valid_count'))
            yield value


def collect_source_evidence(dataset_root, contract):
    """Finalization-only sample scan, preserving original monotonic order."""
    validate_contract(contract)
    evidence = {logical:dict(count=0,observed_count=0,unavailable_count=0,samples_monotonic_ns=[],first_valid_monotonic_ns=None,
                            last_valid_monotonic_ns=None,stream_epoch=None,sequence_first=None,
                            sequence_last=None,last_observed_monotonic_ns=None,max_gap_ns=0,issues=[])
                for logical in contract['sources']}
    for row in iter_source_samples(dataset_root,contract):
        logical=row['logical_source']; spec=contract['sources'][logical]; source=evidence[logical]
        stamp=_ns(row['host_monotonic_ns'],logical+' stamp',positive=True)
        sequence=_ns(row['sequence'],logical+' sequence')
        if row['source_id']!=spec['source_id']:
            source['issues'].append('SOURCE_IDENTITY_MISMATCH')
        if row.get('session_id') is not None and row['session_id']!=contract['session_id']:
            source['issues'].append('SOURCE_SESSION_MISMATCH')
        epoch=row['stream_epoch']
        if not isinstance(epoch,str) or not epoch:
            source['issues'].append('SOURCE_EPOCH_MISSING')
        if source['observed_count']==0:
            source.update(stream_epoch=epoch,sequence_first=sequence)
        else:
            if epoch!=source['stream_epoch']: source['issues'].append('SOURCE_EPOCH_CHANGED')
            if sequence!=source['sequence_last']+1:
                # A bus-wide ultrasonic sequence alternates the four addresses.
                # Its address index must advance, but is not consecutive by one.
                if spec['kind']!='ultrasonic' or sequence<=source['sequence_last']:
                    source['issues'].append('SOURCE_SEQUENCE_GAP_OR_REORDER')
            if stamp<source['last_observed_monotonic_ns']: source['issues'].append('SOURCE_MONOTONIC_REORDER')
        source.update(observed_count=source['observed_count']+1,sequence_last=sequence,last_observed_monotonic_ns=stamp)
        if spec['kind']=='lidar':
            counts=[row.get('raw_valid_count'),row.get('filtered_valid_count')]
            if any(type(value) is not int or value<0 for value in counts):
                source['issues'].append('SOURCE_VALID_COUNT_MISSING_OR_INVALID')
                source['unavailable_count']+=1
                continue
            if any(value==0 for value in counts):
                # Keep and reconcile every raw frame, but all-NaN/filter warmup
                # frames cannot establish an available geometric time window.
                source['unavailable_count']+=1
                continue
        if source['count']==0: source['first_valid_monotonic_ns']=stamp
        else: source['max_gap_ns']=max(source['max_gap_ns'],stamp-source['last_valid_monotonic_ns'])
        source['count']+=1
        source['samples_monotonic_ns'].append(stamp)
        source['last_valid_monotonic_ns']=stamp
    for value in evidence.values(): value['issues']=sorted(set(value['issues']))
    return dict(schema_version=1,session_id=contract['session_id'],sources=evidence,
                time_basis='original host arrival monotonic; physical synchronization unverified')


def evaluate_capture_window(evidence, contract, start_monotonic_ns, end_monotonic_ns):
    validate_contract(contract)
    start=_ns(start_monotonic_ns,'window start',positive=True)
    end=_ns(end_monotonic_ns,'window end',positive=True)
    if end<=start: raise ValueError('capture window must have positive duration')
    reports,issues={},[]
    for logical,spec in contract['sources'].items():
        source=evidence.get('sources',{}).get(logical,{})
        stamps=source.get('samples_monotonic_ns',[])
        failures=list(source.get('issues',[]))
        if not stamps or stamps[0]>start or stamps[-1]<end:
            failures.append('REQUESTED_WINDOW_NOT_COVERED')
        # Only gaps intersecting the requested interval count; retain pre-roll
        # diagnostics separately without silently enlarging the target window.
        gaps=[right-left for left,right in zip(stamps,stamps[1:]) if left<end and right>start]
        maximum=max(gaps,default=0)
        if maximum>spec['max_gap_ns']: failures.append('MAX_SOURCE_GAP_EXCEEDED')
        failures=sorted(set(failures))
        reports[logical]=dict(window_complete=not failures,count=source.get('count',0),
            first_valid_monotonic_ns=stamps[0] if stamps else None,last_valid_monotonic_ns=stamps[-1] if stamps else None,
            maximum_window_gap_ns=maximum,max_allowed_gap_ns=spec['max_gap_ns'],issues=failures)
        issues.extend(dict(code=code,source=logical) for code in failures)
    firsts=[source.get('first_valid_monotonic_ns') for source in evidence.get('sources',{}).values()]
    lasts=[source.get('last_valid_monotonic_ns') for source in evidence.get('sources',{}).values()]
    return dict(window_complete=not issues,full_window_verified=not issues,sources=reports,issues=issues,
        start_monotonic_ns=start,end_monotonic_ns=end,requested_duration_ns=end-start,
        observed_common_start_ns=max(firsts) if firsts and all(x is not None for x in firsts) else None,
        observed_common_end_ns=min(lasts) if lasts and all(x is not None for x in lasts) else None,
        physical_synchronization_verified=False)
