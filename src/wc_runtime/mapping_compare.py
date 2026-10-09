"""One immutable capture/bag, independent same-input estimator experiments.

SDK output is the boundary. Old SDK equivalence, physical sensor completeness
and absolute accuracy are never inferred from a successful offline run.
"""
import argparse
from collections import Counter
import copy
import hashlib
import itertools
import json
import math
import os
from pathlib import Path, PurePosixPath
from .project_paths import compare_staging_root
import sqlite3
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .compare_replay import RecordedPackets, replay_cell, write_json, digest, stats

def _json(path):
    path=Path(path)
    if path.stat().st_size>2*1024*1024: raise ValueError('oversized experiment configuration: '+str(path))
    return json.loads(path.read_text(encoding='utf-8'),parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))

def _inside(root,name):
    name=Path(name)
    if name.is_absolute() or '..' in name.parts: raise ValueError('archive path must be relative: '+str(name))
    path=root/name
    if any(p.is_symlink() for p in (path,*path.parents)): raise ValueError('archive path must not follow symlinks')
    if not path.resolve().is_relative_to(root.resolve()): raise ValueError('archive path escapes dataset')
    return path

def archived_geometry_source_map(setup):
    """Locate frozen calibration sources without changing their bytes/identity."""
    from .calibration_geometry import validate_sources
    result = {}
    for row in validate_sources(setup['source_documents']).values():
        original = PurePosixPath(row['snapshot_path'])
        if original.parts[:2] != ('config', 'calibration'):
            raise ValueError('capture geometry evidence was not frozen in configuration/calibration: '+str(original))
        result[str(original)] = str(PurePosixPath('configuration/calibration', *original.parts[2:]))
    return result

def identify_sources(bag):
    """Read recorded identities; never guess current sensor IDs or open devices."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    identities={'sensor_ids':{},'session_ids':set()}
    for path in sorted(bag.glob('*.db3')):
        with sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True) as db:
            db.execute('PRAGMA query_only=ON')
            for topic_id,topic,kind in db.execute('SELECT id,name,type FROM topics'):
                if kind not in ('wc_interfaces/msg/SourceFrame','wc_interfaces/msg/H30Frame','std_msgs/msg/String'): continue
                if kind=='std_msgs/msg/String' and topic!='/wc_mapping/wheel/feedback_raw': continue
                result=db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id LIMIT 1',(topic_id,)).fetchone()
                if result is None: continue
                message=deserialize_message(result[0],get_message(kind))
                if kind=='wc_interfaces/msg/SourceFrame':
                    side=message.side
                    old=identities['sensor_ids'].setdefault(side,message.sensor_id)
                    if old!=message.sensor_id: raise ValueError('recording has multiple sensor identities on '+side)
                    identities['session_ids'].add(message.session_id)
                elif kind=='wc_interfaces/msg/H30Frame':
                    old=identities.setdefault('imu_sensor_id',message.sensor_id)
                    if old!=message.sensor_id: raise ValueError('recording has multiple IMU identities')
                    identities['session_ids'].add(message.session_id)
                else:
                    row=json.loads(message.data)
                    old=identities.setdefault('wheel_device_id',row['device_id'])
                    if old!=row['device_id']: raise ValueError('recording has multiple wheel devices')
    if len(identities['session_ids'])!=1 or not identities['sensor_ids'] or \
            'imu_sensor_id' not in identities or 'wheel_device_id' not in identities:
        raise ValueError('one identified source session with lidar/IMU/wheel is required')
    identities['session_id']=identities.pop('session_ids').pop()
    return identities

def select_lidar(recorded_mode, sensor_ids, lidar=None):
    """Select recorded point clouds; keep the original acquisition untouched."""
    modes = ('left', 'right', 'all')
    if recorded_mode not in modes or (lidar is not None and lidar not in modes):
        raise ValueError('lidar mode must be left, right or all')
    selected_mode = recorded_mode if lidar is None else lidar
    selected = ['left', 'right'] if selected_mode == 'all' else [selected_mode]
    available = [side for side in ('left', 'right') if sensor_ids.get(side)]
    missing = sorted(set(selected) - set(available))
    if missing:
        raise ValueError('requested lidar not present in recording: ' + ', '.join(missing))
    original = ['left', 'right'] if recorded_mode == 'all' else [recorded_mode]
    # Use the original recording's epoch even when its earliest lidar is excluded.
    # This keeps session-bound motion calibration and timestamps comparable.
    clock_sides = sorted((set(original) | set(selected)) & set(available))
    return {'recorded_mode': recorded_mode, 'selected_mode': selected_mode,
            'selected_sides': selected, 'excluded_sides': sorted(set(available) - set(selected)),
            'clock_reference_sides': clock_sides,
            'selected_sensor_ids': {side: sensor_ids[side] for side in selected},
            'selection_basis': 'RECORDED_MODE' if lidar is None else 'EXPLICIT_OFFLINE_LIDAR_SELECTION',
            'original_recording_modified': False}


def load_dataset(source,*,allow_partial=False,hardware_setup=None,gyro_bias=None,project_root=None,mechanical_initial=False,
                 lidar=None,sides=None,cloud=None):
    from .recording_sources import resolve_lidar_option,source_inventory,select_sources
    selection=resolve_lidar_option(lidar,sides)
    source=Path(source).resolve(strict=True)
    manifest_path=source/'capture_manifest.json'
    manifest=_json(manifest_path) if manifest_path.is_file() else None
    if manifest and (manifest.get('status')!='COMPLETE' or manifest.get('recording_complete') is not True) and not allow_partial:
        raise ValueError('capture incomplete; --allow-partial only permits an explicitly incomplete diagnostic')
    runtime_path=_inside(source,manifest['runtime_config_path']) if manifest else source/'runtime_config.json'
    runtime=_json(runtime_path)
    bag=_inside(source,manifest.get('bag_path','bag')) if manifest else source/'bag'
    hashes={str(runtime_path):digest(runtime_path)}
    if manifest: hashes[str(manifest_path)]=digest(manifest_path)
    for path in sorted(bag.glob('*.db3')): hashes[str(path)]=digest(path)
    if not any(path.endswith('.db3') for path in hashes): raise ValueError('no immutable SQLite source bag parts')
    if (bag/'metadata.yaml').is_file(): hashes[str(bag/'metadata.yaml')]=digest(bag/'metadata.yaml')
    if manifest:
        for name,value in manifest.get('input_hashes',{}).items():
            expected=value.get('sha256') if isinstance(value,dict) else value
            if not isinstance(expected,str): raise ValueError('capture input hash has invalid format: '+name)
            path=_inside(source,name)
            if not path.is_file() or digest(path)!=expected: raise ValueError('capture input hash mismatch: '+name)
            hashes[str(path)]=expected
    identity=identify_sources(bag)
    runtime.update(identity)
    recorded_mode=manifest.get('mode',runtime.get('mode','all')) if manifest else runtime.get('mode','all')
    inventory=source_inventory(bag)
    selected_mode,selected_sides=select_sources(inventory,recorded_mode if selection is None else selection,cloud=cloud)
    runtime['offline_lidar_selection']=select_lidar(recorded_mode,runtime['sensor_ids'],selection)
    runtime['mode']=runtime['offline_lidar_selection']['selected_mode']
    if cloud is not None:runtime['cloud_source']=cloud
    runtime['offline_source_selection']={'recorded_mode':recorded_mode,'mode':selected_mode,
        'selected_sides':selected_sides,'available_sides':[side for side in ('left','right') if inventory[side]],
        'cloud':cloud or runtime.get('cloud_source','filtered'),
        'clock_reference_sides':runtime['offline_lidar_selection']['clock_reference_sides'],
        'original_recording_modified':False,'single_lidar_diagnostic':selected_mode!='all'}
    runtime.update(continuous_mapping=True,odometry_source='wheel_imu',motion_model='planar_ekf',offline_experiment=True)
    setup_path=Path(hardware_setup).resolve(strict=True) if hardware_setup else \
        (_inside(source,manifest['hardware_setup_path']) if manifest else source/'hardware_setup.json')
    if setup_path.is_file():
        from .hardware_setup import resolve_hardware_setup
        from .calibration_geometry import motion_capability, require_geometry_capability,verify_geometry_sources
        setup=_json(setup_path)
        resolved=resolve_hardware_setup(setup)
        # schema1 is allowed only as a historical source-session replay. It must
        # not be selected as an override for the present V7 assembly.
        if setup.get('schema_version')==1 and (manifest or hardware_setup):
            raise ValueError('V7/current capture cannot adopt legacy schema1 installation candidates')
        if setup.get('schema_version')!=1:
            verification_root=Path(project_root or Path(__file__).resolve().parents[2])
            source_path_map=None
            if manifest and not hardware_setup:
                verification_root=source
                source_path_map=archived_geometry_source_map(setup)
            verification=verify_geometry_sources(setup,verification_root,source_path_map=source_path_map)
            if verification['status']!='PASS': raise ValueError('geometry source evidence hash/catalogue check failed: '+json.dumps(verification))
            runtime['geometry_source_verification']=verification
            for row in verification['sources']:
                if source_path_map is not None:
                    evidence_path=_inside(verification_root,row['snapshot_path'])
                else:
                    from .storage_policy import resolve_storage_path
                    evidence_path=resolve_storage_path(verification_root,row['snapshot_path'])
                hashes[str(evidence_path)]=digest(evidence_path)
            if mechanical_initial:
                from .compare_geometry import mechanical_initial_geometry
                resolved=mechanical_initial_geometry(setup,resolved)
            require_geometry_capability(resolved,motion_capability(runtime['mode']))
        runtime.update(resolved)
        hashes[str(setup_path)]=digest(setup_path)
        runtime['hardware_setup_provenance']={'sha256':hashes[str(setup_path)],'source_path':str(setup_path),
                                              'snapshot_file':'hardware_setup.json','mounting_validated':False}
    elif manifest or not {'mounts','imu_mount','wheel_candidate','mount_model'}<=set(runtime):
        raise ValueError('explicit hardware setup with complete required transforms is required for motion replay')
    confirmed=runtime.get('confirmed_gyro_bias') or runtime.get('prior_template',{}).get('confirmed_gyro_bias')
    if gyro_bias:
        from .mapping_planar import validate_confirmed_bias
        bias_path=Path(gyro_bias).resolve(strict=True); evidence=bias_path.with_name(bias_path.stem+'.evidence.json')
        confirmed=validate_confirmed_bias(_json(bias_path))
        if digest(evidence)!=confirmed['evidence_sha256']: raise ValueError('bias evidence SHA-256 mismatch')
        hashes[str(bias_path)]=digest(bias_path); hashes[str(evidence)]=digest(evidence)
    elif confirmed:
        evidence=source/'confirmed_gyro_bias.evidence.json'
        if not evidence.is_file() or digest(evidence)!=confirmed['evidence_sha256']:
            raise ValueError('embedded confirmed bias requires its archived evidence; use explicit --gyro-bias otherwise')
        hashes[str(evidence)]=digest(evidence)
    elif runtime.get('gyro_bias_config') is not None:
        raise ValueError('gyro_bias_config names an unsnapshotted file; provide explicit --gyro-bias with evidence')
    runtime['confirmed_gyro_bias']=confirmed
    prior=copy.deepcopy(runtime.get('prior_template',{}))
    prior.update(schema_version=1,status='EXPERIMENT',source_mode='real',reference_frame='mapping_reference',
        imu_sensor_id=runtime['imu_sensor_id'],wheel_device_id=runtime['wheel_device_id'],wheel_candidate=runtime['wheel_candidate'],
        input_cloud_topic='/wc_mapping/app/input_cloud',output_cloud_topic='/wc_mapping/app/scan_cloud',
        private_tf_topic='/wc_mapping/app/tf',guess_frame_id='prior_odom',prior_odom_topic='/wc_mapping/app/prior_odom',
        require_dual_time_offsets=runtime['mode']=='all',imu_topic='/wc_mapping/imu/source_frame',
        wheel_topic='/wc_mapping/wheel/feedback_raw',odometry_source='wheel_imu',continuous_mapping=True,
        motion_model='planar_ekf',confirmed_gyro_bias=confirmed,wheel_imu_covariance=runtime['wheel_imu_covariance'])
    runtime['prior_template']=prior; runtime['_offline_bag_path']=str(bag)
    return runtime,manifest,hashes

def source_accounting(packets):
    streams={}
    for event in packets.events:
        stream=event['topic']; values=streams.setdefault(stream,[]); values.append(event)
    result={}
    for name,rows in streams.items():
        seq=[r['sequence'] for r in rows]; stamps=[r['stamp_ns'] for r in rows]
        result[name]={'recorded_events':len(rows),'first_sequence':seq[0],'last_sequence':seq[-1],
            'sequence_gaps':[{'previous':a,'next':b,'missing':b-a-1} for a,b in zip(seq,seq[1:]) if b>a+1],
            'nonincreasing_sequences':sum(b<=a for a,b in zip(seq,seq[1:])),
            'largest_source_gap_s':max((b-a)*1e-9 for a,b in zip(stamps,stamps[1:])) if len(stamps)>1 else None,
            'duplicate_source_stamps':len(stamps)-len(set(stamps))}
    return result

def compare_cells(cells):
    common=sorted(set.intersection(*(set(c['frames']) for c in cells.values())))
    if not common: raise ValueError('no common forwarded original cloud stamps across comparison cells')
    for key in common:
        bindings=[]
        for cell in cells.values():
            row=cell['pairs'][key]
            bindings.append([(r['side'],r['raw_key'],r['source_stamp_ns']) for r in row['sources']])
        if any(row!=bindings[0] for row in bindings[1:]): raise ValueError('common stamp contains different actual source pairs')
    comparisons=[]
    for (left,a),(right,b) in itertools.combinations(cells.items(),2):
        factors=('estimator','input_rate_hz','filter_enabled')
        changed=[key for key in factors if a['summary'][key]!=b['summary'][key]]
        if len(changed)!=1: continue
        shared=sorted(set(a['frames'])&set(b['frames'])); positions=[]; rotations=[]
        for stamp in shared:
            pa,pb=(np.asarray(c['frames'][stamp]['T_prior_reference']) for c in (a,b))
            positions.append(float(np.linalg.norm(pa[:3,3]-pb[:3,3])))
            rotations.append(float(np.rad2deg(Rotation.from_matrix(pa[:3,:3].T@pb[:3,:3]).magnitude())))
        comparisons.append({'left':left,'right':right,'changed_factor':changed[0],'common_frames':len(shared),
            'position_difference_m':stats(positions),'rotation_difference_deg':stats(rotations),
            'interpretation':'same-source algorithm difference; not independent pose error'})
    return {'common_frame_stamps_ns':common,'common_frames':len(common),'single_factor_comparisons':comparisons}

def write_trajectory(cell):
    path=cell['directory']/'trajectory.csv'
    with path.open('x',encoding='utf-8',newline='') as stream:
        stream.write('stamp_ns,x_m,y_m,z_m,qx,qy,qz,qw\n')
        for stamp,row in sorted(cell['frames'].items()):
            pose=np.asarray(row['T_prior_reference']); q=Rotation.from_matrix(pose[:3,:3]).as_quat()
            stream.write(','.join([str(stamp)]+[format(float(v),'.17g') for v in [*pose[:3,3],*q]])+'\n')
    return digest(path)

def run_shadow(cell):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from wc_sensors.pointcloud import decode_pointcloud2
    from .icp_shadow import point_to_plane
    previous=None; counts=Counter()
    with (cell['directory']/'icp_shadow.jsonl').open('x',encoding='utf-8') as stream:
        for stamp,row in sorted(cell['frames'].items()):
            path=cell['directory']/'frontend'/row['cloud_cdr_file']
            if digest(path)!=row['cloud_cdr_sha256']: raise ValueError('derived scan hash changed before ICP shadow')
            points=decode_pointcloud2(deserialize_message(path.read_bytes(),PointCloud2))
            pose=np.asarray(row['T_prior_reference'])
            if previous is not None:
                old_stamp,old_points,old_pose=previous
                report=point_to_plane(points,old_points,np.linalg.inv(old_pose)@pose)
                report.update(stamp_ns=stamp,target_stamp_ns=old_stamp)
                counts['accepted' if report['accepted'] else 'rejected']+=1
                stream.write(json.dumps(report,allow_nan=False,separators=(',',':'))+'\n')
            previous=stamp,points,pose
    return {'mode':'SHADOW_ONLY','trajectory_changed':False,**dict(counts)}

def truth_errors(cells,path,*,min_common_samples=1):
    """All cells use one exact truth-stamp intersection, with no silent pruning."""
    if type(min_common_samples) is not int or min_common_samples<1:
        raise ValueError('truth minimum common sample count must be a positive integer')
    result={'status':'FAILED_TRUTH_INPUT','input_path':str(path),
        'alignment':'no fitted alignment, no interpolation, exact source stamps',
        'independence':'CALLER_DECLARED','formal_accuracy_acceptance':False,
        'coverage_policy':{'min_common_samples':min_common_samples,'declared_before_evaluation':True},
        'rotation_definition':'SO(3) geodesic magnitude in degrees; includes roll/pitch/yaw',
        'yaw_definition':'absolute wrapped difference of atan2(R[1,0],R[0,0]); shared defined-heading subset',
        'invalid_truth_rows':[],'invalid_estimates':[],'cells':{}}
    try:
        result['input_sha256']=digest(path)
        truth=_json(path)
        if truth.get('independent') is not True or truth.get('time_correspondence_status')!='CONFIRMED' or \
                truth.get('frame_id')!='mapping_odom' or truth.get('child_frame_id')!='mapping_reference' or \
                not isinstance(truth.get('evidence_id'),str) or not truth['evidence_id'].strip():
            raise ValueError('truth needs independent evidence, confirmed source-time correspondence and exact mapping frames')
        if not isinstance(truth.get('poses'),list): raise ValueError('truth poses must be an explicit array')
        if not cells: raise ValueError('truth evaluation requires experiment cells')
        poses={}; from .calibration_geometry import rigid_transform
        for index,row in enumerate(truth['poses']):
            try:
                stamp=row['stamp_ns']
                if type(stamp) is not int or stamp<0 or stamp in poses: raise ValueError('duplicate/invalid truth stamp')
                poses[stamp]=rigid_transform(row['T_mapping_odom_reference'],'truth pose',translation_limit_m=1e6)
            except (ValueError,KeyError,TypeError) as error:
                result['invalid_truth_rows'].append({'row_index':index,'error':str(error)})
        estimates={}; available={}
        for name,cell in cells.items():
            available[name]=set(cell['frames']); estimates[name]={}
            for stamp,row in cell['frames'].items():
                try:
                    if type(stamp) is not int or stamp<0: raise ValueError('invalid estimate stamp')
                    estimates[name][stamp]=rigid_transform(row['T_prior_reference'],'estimated pose',translation_limit_m=1e6)
                except (ValueError,KeyError,TypeError) as error:
                    result['invalid_estimates'].append({'cell':name,'stamp_ns':stamp,'error':str(error)})
        truth_stamps=set(poses)
        common=sorted(truth_stamps.intersection(*(available.values())))
        result.update(input_truth_rows=len(truth['poses']),valid_truth_rows=len(poses),
            common_truth_stamp_ns=common,common_truth_samples=len(common),
            truth_stamps_excluded_from_all_cell_comparison=sorted(truth_stamps-set(common)),
            truth_exclusion_count=len(truth_stamps-set(common)))
        for name in cells:
            individual=truth_stamps&available[name]
            result['cells'][name]={'matched_truth_samples':len(common),'individual_exact_matches':len(individual),
                'missing_truth_stamp_ns':sorted(truth_stamps-available[name]),
                'excluded_due_to_other_cells_stamp_ns':sorted(individual-set(common)),
                'output_without_truth_count':len(available[name]-truth_stamps)}
        if result['invalid_truth_rows'] or result['invalid_estimates']:
            result['status']='FAILED_INVALID_TRUTH_OR_ESTIMATE'
            return result  # Never hide corrupt outputs by removing them from a metric.
        if len(common)<min_common_samples:
            result['status']='FAILED_INSUFFICIENT_COMMON_TRUTH'
            return result
        def heading(pose):
            if np.linalg.norm(pose[:2,0])<1e-8: return None
            return math.atan2(pose[1,0],pose[0,0])
        yaw_stamps=[stamp for stamp in common if heading(poses[stamp]) is not None and
                    all(heading(estimates[name][stamp]) is not None for name in cells)]
        result.update(common_yaw_stamp_ns=yaw_stamps,
            yaw_excluded_undefined_heading_stamp_ns=sorted(set(common)-set(yaw_stamps)))
        for name in cells:
            distances=[]; angles=[]; yaw=[]
            for stamp in common:
                estimate=estimates[name][stamp]; reference=poses[stamp]
                distances.append(float(np.linalg.norm(estimate[:3,3]-reference[:3,3])))
                angles.append(float(np.rad2deg(Rotation.from_matrix(reference[:3,:3].T@estimate[:3,:3]).magnitude())))
            for stamp in yaw_stamps:
                delta=heading(estimates[name][stamp])-heading(poses[stamp])
                yaw.append(abs(math.degrees(math.atan2(math.sin(delta),math.cos(delta)))))
            result['cells'][name].update(position_error_m=stats(distances),
                so3_rotation_error_deg=stats(angles),rotation_error_deg=stats(angles),
                rotation_error_deg_alias='SO(3) geodesic error; not yaw error',
                yaw_error_deg=stats(yaw),matched_yaw_samples=len(yaw_stamps),
                position_rmse_m=float(np.sqrt(np.mean(np.square(distances)))))
        result['status']='COMMON_EXACT_TRUTH_DIAGNOSTICS_COMPLETE'
    except (ValueError,KeyError,TypeError,OSError) as error:
        result['error']=type(error).__name__+': '+str(error)
    return result

def comparison_storage_root(root,output,memory_work=False):
    """Only this explicit offline job may use an owned tmpfs scratch subtree."""
    if not memory_work:
        return root
    base=compare_staging_root()
    if output.parent!=base or '..' in output.parts:
        raise ValueError('--memory-work requires a new child of '+str(base))
    if any(p.is_symlink() for p in (output,*output.parents)):
        raise ValueError('memory comparison must not follow symlinks')
    mounts=Path('/proc/mounts').read_text().splitlines()
    if not any(len(row.split())>2 and row.split()[1:3]==['/dev/shm','tmpfs'] for row in mounts):
        raise ValueError('/dev/shm must be a mounted tmpfs')
    base.mkdir(mode=0o700,exist_ok=True)
    metadata=base.stat()
    if metadata.st_dev!=Path('/dev/shm').stat().st_dev:
        raise ValueError('memory comparison root must remain on the /dev/shm filesystem')
    if metadata.st_uid!=os.getuid() or metadata.st_mode & 0o022:
        raise ValueError('memory comparison root must be owned and not group/world writable')
    return base


def validate_comparison_rates(input_rates, native_rate):
    """Zero removes only the offline frontend throttle, never source checks."""
    if not input_rates or any(not math.isfinite(v) or v < 0 or v > 10 for v in input_rates):
        raise ValueError('experimental cloud consumption rate must be 0 (all valid pairs) or (0,10] Hz')
    # Unlimited frontend cells have no artificial cap to compare against. Keep
    # the existing native replay ceiling for an all-unlimited experiment.
    native_limit = min((v for v in input_rates if v > 0), default=10.)
    if not math.isfinite(native_rate) or not 0 < native_rate <= native_limit:
        raise ValueError('native replay rate must be positive and not exceed any capped frontend cell or 10 Hz')


def validate_native_wall_interval(interval):
    """Bound wall-clock pacing independently of recorded-time sampling."""
    if not math.isfinite(interval) or not .05 <= interval <= 10.:
        raise ValueError('native wall interval must be finite and in [0.05,10] seconds')


def freeze_motion_candidate(path, output, source, input_hashes, runtime, packets):
    """Freeze original candidate bytes and verify the same-session raw inputs."""
    from .offline_refinement_inputs import verify_candidate
    from .offline_motion_adapter import prior_factory
    path = Path(path).absolute()
    if '..' in path.parts or any(p.is_symlink() for p in (path,*path.parents)):
        raise ValueError('motion candidate must be an unlinked file without traversal')
    if runtime.get('confirmed_gyro_bias') is not None or \
            runtime.get('prior_template',{}).get('confirmed_gyro_bias') is not None or \
            runtime.get('offline_motion_correction') is not None:
        raise ValueError('motion candidate requires uncorrected input; do not subtract bias twice')
    path = path.resolve(strict=True)
    if path.stat().st_size > 2*1024*1024:
        raise ValueError('oversized motion candidate')
    raw = path.read_bytes()
    sha = hashlib.sha256(raw).hexdigest()
    snapshot = output/'motion_candidate.json'
    with snapshot.open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    candidate = _json(snapshot)
    proof = verify_candidate(candidate,snapshot,source,input_hashes,runtime=runtime,packets=packets)
    if digest(snapshot) != sha or digest(path) != sha:
        raise ValueError('motion candidate changed while freezing evidence')
    input_hashes[str(path)] = sha
    proof.update(original_candidate_path=str(path),snapshot_file='motion_candidate.json')
    write_json(output/'candidate_verification.json',proof)
    evidence = {'status':'OFFLINE_CANDIDATE_APPLIED','candidate_sha256':sha,
        'candidate_id':candidate['candidate_id'],'snapshot_file':'motion_candidate.json',
        'verification_file':'candidate_verification.json','automatic_live_application':False,
        'process_noise':'UNCHANGED_PER_ESTIMATOR','geometry_feedback':False}
    return prior_factory(candidate,sha), evidence


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--user-data-paths',action='store_true',help='Explicit user-selected data directories, with per-operation mount guards')
    parser.add_argument('--project-root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--estimators',nargs='+',choices=('five_state','robot_localization'),default=['five_state','robot_localization'])
    parser.add_argument('--input-rate-hz',nargs='+',type=float,default=[5.],help='Offline cloud consumption: 0 = all valid paired frames without throttling; otherwise (0,10] Hz. Every real wheel/IMU event retained')
    parser.add_argument('--filter',nargs='+',choices=('off','on'),default=['on'])
    parser.add_argument('--cloud',choices=('raw','filtered'),default='raw')
    parser.add_argument('--imu-time-mode',choices=('auto','arrival','device_relative'),default='auto',
        help='auto uses a frozen verified IMU timing profile when present; legacy bags retain arrival timing')
    parser.add_argument('--lidar',choices=('left','right','all'),
        help='Use left, right or both recorded lidars with the original IMU/wheel inputs; omitted = recorded mode; never opens hardware')
    parser.add_argument('--sides',choices=('all','left','right'),
        help='Alias of --lidar for panel jobs; conflicting simultaneous values are rejected.')
    parser.add_argument('--hardware-setup',type=Path,help='Explicit newly confirmed geometry override; snapshot/hash retained')
    parser.add_argument('--gyro-bias',type=Path,help='Optional explicit bias file with same-name evidence')
    parser.add_argument('--motion-candidate',type=Path,
        help='Verified same-session offline IMU bias and wheel yaw correction, shared by selected EKFs; does not change process noise or live calibration')
    parser.add_argument('--allow-partial',action='store_true')
    parser.add_argument('--mechanical-initial',action='store_true',help='Explicit offline V7 mechanical-face initial geometry; does not change live calibration')
    parser.add_argument('--memory-work',action='store_true',help='Derived scratch output under /dev/shm/wc_compare_<UID>; export results to persistent storage afterwards')
    parser.add_argument('--source-limit',type=int,default=0,help='Diagnostic prefix only; 0 consumes all source events')
    parser.add_argument('--truth-json',type=Path)
    parser.add_argument('--truth-min-common-samples',type=int,default=1,
                        help='Predeclared positive minimum for the all-cell common exact truth stamps; not an accuracy threshold')
    parser.add_argument('--map-quality-policy',type=Path,help='Explicit experimental moving-map qualification limits; frozen before evaluation')
    parser.add_argument('--icp-shadow',choices=('off','point_to_plane'),default='off')
    parser.add_argument('--native-map',action='store_true',help='Run isolated hardware-free RTABMap and close/export each map')
    parser.add_argument('--domain',type=int,default=89)
    parser.add_argument('--native-rate-hz',type=float,default=1.)
    parser.add_argument('--native-limit',type=int,default=0)
    parser.add_argument('--native-wall-interval-s',type=float,default=1.,
        help='Minimum wall-clock interval between native frames, [0.05,10] seconds; '
             'each original-stamp Info acknowledgement is still required. '
             'Does not change recorded timestamps or native sampling rate')
    args=parser.parse_args(argv)
    from .recording_sources import resolve_lidar_option
    try:resolve_lidar_option(args.lidar,args.sides)
    except ValueError as error:parser.error(str(error))
    output=args.output.absolute(); source=args.dataset.absolute(); root=args.project_root.resolve(strict=True)
    # Keep the original syntax checks before aliases are resolved, so a
    # parent-traversal argument can never be normalized into a valid output.
    if '..' in output.parts or '..' in source.parts:
        raise ValueError('output must be new, unlinked and separate from original dataset')
    from .storage_policy import StoragePolicy
    storage_policy = StoragePolicy(root)
    if args.user_data_paths:
        if args.memory_work:
            raise ValueError('--user-data-paths cannot be combined with volatile --memory-work')
        from .user_data_paths import UserDataPaths
        storage_policy = UserDataPaths(root,args.dataset,args.output,
            [getattr(args,key) for key in ('hardware_setup','gyro_bias','motion_candidate','truth_json','map_quality_policy')])
    if storage_policy.enabled:
        output = args.output.absolute() if args.memory_work else storage_policy.resolve(args.output)
        source = storage_policy.resolve(args.dataset)
        for field in ('hardware_setup', 'gyro_bias', 'motion_candidate', 'truth_json', 'map_quality_policy'):
            value = getattr(args, field)
            if value is not None: setattr(args, field, storage_policy.resolve(value))
    source=source.resolve(strict=True)
    if output.exists() or '..' in output.parts or any(p.is_symlink() for p in (output,*output.parents)) or \
            output.is_relative_to(source) or source.is_relative_to(output): raise ValueError('output must be new, unlinked and separate from original dataset')
    if args.source_limit<0 or args.native_limit<0:
        raise ValueError('invalid source limits')
    if args.motion_candidate is not None and args.gyro_bias is not None:
        raise ValueError('--motion-candidate and --gyro-bias cannot be combined (double bias)')
    validate_comparison_rates(args.input_rate_hz,args.native_rate_hz)
    validate_native_wall_interval(args.native_wall_interval_s)
    if args.truth_min_common_samples<1: raise ValueError('truth minimum common sample count must be positive')
    if any(len(values)!=len(set(values)) for values in (args.estimators,args.input_rate_hz,args.filter)):
        raise ValueError('duplicate experiment factor values')
    if not 1<=args.domain<=232 or args.domain==83: raise ValueError('isolated domain required, never production domain 83')
    storage_root=storage_policy.archive_root if args.user_data_paths else comparison_storage_root(root,output,args.memory_work)
    if storage_policy.enabled and not args.memory_work:
        if not output.is_relative_to(storage_policy.archive_root):
            raise ValueError('configured comparison outputs must use USB data/reports/maps storage')
        storage_root=storage_policy.archive_root
    runtime,manifest,input_hashes=load_dataset(source,allow_partial=args.allow_partial,
        hardware_setup=args.hardware_setup,gyro_bias=args.gyro_bias,project_root=root,mechanical_initial=args.mechanical_initial,
        lidar=args.lidar,sides=args.sides,cloud=args.cloud)
    from .map_quality import validate_map_quality_policy
    quality_policy=validate_map_quality_policy(_json(args.map_quality_policy) if args.map_quality_policy else runtime.get('map_quality_policy'))
    if args.map_quality_policy: input_hashes[str(args.map_quality_policy.resolve(strict=True))]=digest(args.map_quality_policy)
    runtime['map_quality_policy']=quality_policy
    runtime['cloud_source']=args.cloud
    runtime['offline_imu_time_mode']=args.imu_time_mode
    storage_policy.check()
    output.mkdir(parents=True)
    report={'schema_version':1,'status':'FAILED','dataset':str(source),'hardware_started':False,
        'input_hashes':input_hashes,'recording_complete':manifest.get('recording_complete') if manifest else None,
        'capture_status':manifest.get('status') if manifest else 'LEGACY_COMPLETENESS_UNKNOWN',
        'source_accounting':manifest.get('source_accounting') if manifest else None,
        'source_selection':runtime.get('offline_source_selection'),
        'input_scope':'same SDK decoded XYZ/raw SourceFrame and original H30/wheel messages; not old SDK/full UDP equivalence',
        'time_policy':'single host-monotonic epoch mapped to controlled ROS time; raw wall/device stamps retained in immutable input evidence',
        'clock_accuracy':'COMMON_MEASUREMENT_TIME_UNVALIDATED',
        'absolute_accuracy':'TRUTH_EVALUATION_NOT_RUN' if args.truth_json else 'NO_INDEPENDENT_TRUTH',
        'map_quality_policy':quality_policy,'lidar_selection':runtime.get('offline_lidar_selection'),
        'storage':{'root':str(storage_root),'volatile':args.memory_work,'source_project':str(root),
                   'destination':storage_policy.check()},
        'offline_geometry_initialization':runtime.get('offline_geometry_initialization'),
        'truth_coverage_policy':{'min_common_samples':args.truth_min_common_samples,'declared_before_evaluation':True},
        'prefix_only':bool(args.source_limit),'cells':{},'native_map':{'status':'NOT_RUN'},
        'native_replay_pacing':{'enabled':args.native_map,
            'minimum_wall_interval_s':args.native_wall_interval_s,
            'wait_for_each_original_stamp_info_ack':True,'frame_ack_timeout_s':30.,
            'source_timestamps_changed':False,'source_sampling_changed':False,
            'policy':'Publish the next selected frame only after the current original-stamp Info ACK and the wall interval'}}
    report['algorithm_hashes']={str(path.relative_to(root)):digest(path) for path in
        (Path(__file__),Path(__file__).with_name('mapping_planar.py'),Path(__file__).with_name('mapping_prior.py'),
         Path(__file__).with_name('mapping_input.py'),Path(__file__).with_name('single_mapping_input.py'),
         Path(__file__).with_name('measurement.py'),Path(__file__).with_name('source_time.py'),
         Path(__file__).with_name('offline_imu_time.py'),Path(__file__).parents[1]/'wc_imu'/'device_time.py',
         Path(__file__).with_name('robot_localization_provider.py'),Path(__file__).with_name('compare_replay.py'),
         Path(__file__).with_name('compare_native.py'),Path(__file__).with_name('icp_shadow.py'),
          Path(__file__).with_name('resource_audit.py'),Path(__file__).with_name('map_quality.py'),
          Path(__file__).with_name('compare_geometry.py'),Path(__file__).with_name('offline_motion_adapter.py'),
          Path(__file__).with_name('offline_motion_calibration.py'),Path(__file__).with_name('offline_refinement_inputs.py'),
          Path(__file__).with_name('recording_sources.py'))}
    if args.truth_json:
        report['truth']={'status':'TRUTH_EVALUATION_NOT_RUN','input_path':str(args.truth_json.absolute()),
            'reason':'EXPERIMENT_NOT_REACHED_TRUTH_EVALUATION','formal_accuracy_acceptance':False,
            'coverage_policy':dict(report['truth_coverage_policy'])}
    packets=None; active_cell=None; motion_factory=None
    try:
        # Associate supplied truth before any estimator/replay can fail. This
        # freezes input evidence; it does not validate truth or compute metrics.
        if args.truth_json:
            truth_path=args.truth_json.resolve(strict=True)
            truth_hash=digest(truth_path)
            report['truth'].update(input_path=str(truth_path),input_sha256=truth_hash)
            input_hashes[str(truth_path)]=truth_hash
            if truth_path.stat().st_size<=2*1024*1024:
                with (output/'truth_input.json').open('xb') as stream: stream.write(truth_path.read_bytes())
                snapshot_hash=digest(output/'truth_input.json')
                report['truth']['input_snapshot_sha256']=snapshot_hash
                if snapshot_hash!=truth_hash: raise ValueError('truth input changed while freezing evidence')
        if 'robot_localization' in args.estimators:
            from .robot_localization_provider import worker_path
            from ament_index_python.packages import get_package_prefix
            import xml.etree.ElementTree as ET
            worker=worker_path(); report['actual_rl_worker']={'path':str(worker),'sha256':digest(worker)}
            prefix=Path(get_package_prefix('robot_localization'))
            library=prefix/'lib/librl_lib.so'
            xml=prefix/'share/robot_localization/package.xml'
            report['actual_rl_library']={'path':str(library),'sha256':digest(library),
                'version':ET.fromstring(xml.read_text(encoding='utf-8')).findtext('version')}
        write_json(output/'map_quality_policy.json',quality_policy)
        if args.hardware_setup:
            write_json(output/'hardware_setup_override.json',_json(args.hardware_setup))
        packets=RecordedPackets(source,runtime)
        from .offline_imu_time import configure_runtime
        configure_runtime(runtime,packets.clock)
        if args.motion_candidate is not None:
            motion_factory,correction = freeze_motion_candidate(args.motion_candidate,output,source,
                input_hashes,runtime,packets)
            runtime['offline_refinement'] = copy.deepcopy(correction)
            report['offline_motion_correction'] = correction
        write_json(output/'frozen_runtime_config.json',runtime)
        report['controlled_clock']=packets.clock.report()
        report['recorded_packet_counts']=dict(packets.counts); report['recorded_sequence_accounting']=source_accounting(packets)
        event_hash=hashlib.sha256()
        with (output/'original_event_index.jsonl').open('x',encoding='utf-8') as index:
            for event in packets.events:
                raw=(json.dumps(event,sort_keys=True,allow_nan=False,separators=(',',':'))+'\n').encode()
                event_hash.update(raw); index.write(raw.decode())
        report['original_event_index_sha256']=event_hash.hexdigest()
        cells={}
        for estimator,rate,filtering in itertools.product(args.estimators,args.input_rate_hz,args.filter):
            storage_policy.check()
            rate_name='allframes' if rate == 0 else 'hz'+format(rate,'g').replace('.','p')
            name=estimator+'_'+rate_name+'_filter_'+filtering
            active_cell=name
            print(json.dumps({'stage':'REPLAY_CELL','cell':name}),flush=True)
            storage_options={'storage_root':storage_root} if storage_root != root else {}
            if motion_factory is not None: storage_options['prior_factory'] = motion_factory
            cell=replay_cell(root,output,runtime,runtime,packets,name,estimator,filtering=='on',args.source_limit,rate,**storage_options)
            cell['summary']['trajectory_sha256']=write_trajectory(cell)
            if args.icp_shadow!='off': cell['summary']['icp_shadow']=run_shadow(cell)
            cells[name]=cell; report['cells'][name]=cell['summary']
            active_cell=None
        comparison_error=None
        try:
            report['comparisons']=compare_cells(cells)
        except ValueError as error:
            comparison_error=error
            report['comparisons']={'status':'FAILED_COMMON_OUTPUT_SUPPORT','error':str(error),
                'available_outputs_by_cell':{name:len(cell['frames']) for name,cell in cells.items()}}
        if args.truth_json:
            frozen_truth=report['truth']
            report['truth']=truth_errors(cells,args.truth_json,min_common_samples=args.truth_min_common_samples)
            report['truth']['input_sha256_before_replay']=frozen_truth['input_sha256']
            if frozen_truth.get('input_snapshot_sha256'):
                report['truth']['input_snapshot_sha256']=frozen_truth['input_snapshot_sha256']
            if report['truth'].get('input_sha256')!=frozen_truth['input_sha256']:
                report['truth']['status']='FAILED_TRUTH_INPUT_CHANGED'
            write_json(output/'truth_evaluation.json',report['truth'])
            if report['truth']['status']!='COMMON_EXACT_TRUTH_DIAGNOSTICS_COMPLETE':
                report['absolute_accuracy']='TRUTH_EVALUATION_FAILED'
                raise ValueError('truth evaluation failed: '+report['truth']['status'])
            report['absolute_accuracy']='INDEPENDENT_DECLARED_TRUTH_COMMON_EXACT_STAMP_DIAGNOSTICS'
        if comparison_error is not None: raise comparison_error
        if args.native_map:
            storage_policy.check()
            report['native_map']=run_native_maps(root,output,cells,report['comparisons']['common_frame_stamps_ns'],args,storage_root=storage_root)
        for path,expected in input_hashes.items():
            if digest(Path(path))!=expected: raise ValueError('original input changed during comparison: '+path)
        report['status']='OFFLINE_COMPARISON_COMPLETE'
        if args.source_limit or args.allow_partial and manifest and manifest.get('recording_complete') is not True:
            report['status']='INCOMPLETE_INPUT_DIAGNOSTIC_COMPLETE'
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        if active_cell is not None: report['failed_cell']=active_cell
        if report.get('truth',{}).get('status')=='TRUTH_EVALUATION_NOT_RUN':
            report['truth'].update(reason='EXPERIMENT_FAILED_BEFORE_TRUTH_EVALUATION',error=report['error'])
            if active_cell is not None: report['truth']['failed_cell']=active_cell
    finally:
        if packets is not None: packets.close()
        storage_policy.check()  # Never recreate a removed USB mount to save a report.
        if report.get('truth',{}).get('status')=='TRUTH_EVALUATION_NOT_RUN':
            write_json(output/'truth_evaluation.json',report['truth'])
        write_json(output/'result.json',report)
        write_chinese_summary(output,report)
        write_artifact_manifest(output,report)
    print(json.dumps({'status':report['status'],'output':str(output),'native_map_status':report['native_map']['status']},allow_nan=False))
    return 1 if report['status']=='FAILED' else 0

def write_chinese_summary(output,report):
    """Human-readable evidence boundaries and artifact links for every run."""
    absolute=('未提供独立真值，不将轨迹差异称为定位误差。' if report['absolute_accuracy']=='NO_INDEPENDENT_TRUTH' else
        ('已提供真值，但实验在评估前失败；未计算精度统计，不能比较精度。' if report['absolute_accuracy']=='TRUTH_EVALUATION_NOT_RUN' else
        ('真值评估失败；保留已完成轨迹和剔除/失效证据，不能比较精度。' if report['absolute_accuracy']=='TRUTH_EVALUATION_FAILED' else
         '使用调用者声明独立、时间对应确认、同一参考坐标的真值；所有单元共用精确时刻，不拟合对齐；几何精度验收仍须独立目标。')))
    lines=['# 离线对照实验摘要','',
        '- 状态：'+report['status'],
        '- 输入数据集：'+report['dataset'],
        '- 采集完整性：'+str(report['capture_status'])+'；recording_complete='+str(report['recording_complete']),
        '- 比较范围：同一厂商SDK输出之后的原始点云、IMU消息、轮反馈；不代表旧SDK或完整UDP解码全栈。',
        '- 时间：各源端host-monotonic统一映射到同一受控ROS时轴，原始wall/device时间与CDR保留；物理测量同步未验证。',
        '- 绝对精度：'+absolute,
        '- 原生地图：'+report['native_map']['status']+'；只有显式--native-map才启动隔离的RTAB-Map并闭库导出。','',
        '| 实验单元 | 估计器 | 点云消费Hz | 后级筛选 | 实际输出帧 | 输入末尾待位姿帧 | 零偏状态 |',
        '|---|---|---:|---|---:|---:|---|']
    for name,cell in report['cells'].items():
        bias=cell.get('bias',{}).get('calibration_status','UNKNOWN')
        rate_text='ALL_VALID_PAIRS (0 = no throttle)' if cell['input_rate_hz'] == 0 else str(cell['input_rate_hz'])
        lines.append('| '+name+' | '+cell['estimator']+' | '+rate_text+' | '+
            str(cell['filter_enabled'])+' | '+str(cell['output_frames'])+' | '+str(cell['pending_clouds_at_input_end'])+' | '+bias+' |')
    lines.extend(['', 'input_rate_hz=0: offline frontend uses all valid pairs without rate throttling. '
        'Identity, ordering, pair time bounds, bounded queues and pose coverage checks still apply. '
        'This changes neither sensor output frequency nor the independent native-map consumption rate.'])
    if report.get('truth'):
        truth=report['truth']
        if truth['status']=='TRUTH_EVALUATION_NOT_RUN':
            lines.extend(['','真值尚未评估：输入 '+truth['input_path']+'；状态 '+truth['status']+
                '；原因 '+truth['reason']+'。truth_evaluation.json保留输入关联和失败原因；共同样本、剔除和误差统计均未计算。'])
        else:
            lines.extend(['','真值覆盖：状态 '+truth['status']+'；所有单元共同样本 '+str(truth.get('common_truth_samples',0))+
                '，预声明最低 '+str(truth['coverage_policy']['min_common_samples'])+'；剔除 '+str(truth.get('truth_exclusion_count','未知'))+'。'
                'truth_evaluation.json保留各单元缺失、因其它单元剔除及失效记录；SO(3)全姿态角与水平yaw误差分别列出，不能互换。'])
    lines.extend(['','5/10Hz只控制点云消费，不重采样轮反馈或IMU。只把恰好一个因素不同的组合列为单因素对照。',
        '时钟诊断：input_status关闭快照中的last_publication_source_age_ns用当前主机时钟减历史录包monotonic，'
        '不能作为回放延迟、掉线或轨迹有效性的指标；实际事件处理与位姿查询使用受控的同一源时轴。',
        '资源范围：CPU按Python父进程、仍运行的RL worker及已回收子进程分别列出；父进程包含回放与归档。'
        '峰值RSS为该进程启动后的累计值，不是单组独立峰值；一次短前缀受首次导入、缓存和执行顺序影响，不能据此判定估计器速度优劣。',
        '参数来源：frozen_runtime_config.json来自采集/历史运行配置；每单元runtime_config.json记录显式实验覆盖，'
        'resolved_estimator_config.json与prior_initialization.json记录实际默认展开后的估计器、外参和零偏声明。'
        '5.0σ/NIS门、过程噪声及新增holdout阈值均为实验候选；UNKNOWN零偏不等于已验证的零值。',
        '输入关联：original_event_index.jsonl保留源时序、原CDR哈希和原消息索引；每单元input/pairs.jsonl把点云配对绑定到原源；'
        'frontend/index.jsonl绑定配对、位姿、协方差及派生CDR；trajectory.csv只从该索引导出。',
        '地图与ICP：native/每单元/result.json记录原生确认、闭库和导出结果；icp_shadow.jsonl记录匹配、退化、残差与拒绝，'
        '影子ICP始终不修改权威轨迹。原生选择的共同子集不代表全帧10Hz吞吐验证。',
        '归档：artifact_manifest.json列出原始输入引用及全部输出SHA-256；result.json含代码、实际RL库/worker、源计数、缺段和比较统计。'])
    if report.get('error'): lines.extend(['','失败原因：'+report['error']])
    if report.get('prefix_only'): lines.extend(['','本次仅为受限前缀诊断，未重估整段记录。'])
    with (output/'summary_zh.md').open('x',encoding='utf-8') as stream: stream.write('\n'.join(lines)+'\n')

def write_artifact_manifest(output,report):
    files={}
    for path in sorted(output.rglob('*')):
        if path.is_symlink(): raise ValueError('derived artifact symlink forbidden: '+str(path))
        if path.is_file() and path.name!='artifact_manifest.json':
            files[str(path.relative_to(output)).replace('\\','/')]=dict(sha256=digest(path),bytes=path.stat().st_size)
    write_json(output/'artifact_manifest.json',{'schema_version':1,'status':report['status'],
        'original_inputs':report['input_hashes'],'outputs':files,'original_inputs_retained':True,
        'association':'original_event_index -> input/pairs -> frontend/index -> trajectory/native-map/ICP shadow',
        'self_hash_policy':'manifest excludes itself; all other produced artifacts listed',
        'absolute_accuracy':report['absolute_accuracy']})

def run_native_maps(root,output,cells,common,args,*,storage_root=None):
    from .compare_native import load_index,subsample,run_cell
    import fcntl
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    import rclpy
    native=output/'native'; native.mkdir()
    selected=subsample(common,args.native_rate_hz,args.native_limit)
    selected_args=argparse.Namespace(input_hz=args.native_rate_hz,
        wall_interval=args.native_wall_interval_s,frame_timeout=30.)
    result={'status':'FAILED','selected_original_stamp_ns':selected,'domain':args.domain,'cells':{}}
    lock_dir=root/'.phase1_runtime/locks'; lock_dir.mkdir(parents=True,exist_ok=True)
    with (lock_dir/('domain-'+str(args.domain)+'-geometry-review.lock')).open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        rclpy.init(args=[])
        try:
            for name,cell in cells.items():
                rows=load_index(cell['directory'])
                print(json.dumps({'stage':'NATIVE_MAP_CELL','cell':name,'frames':len(selected)}),flush=True)
                result['cells'][name]=run_cell(root,output,native,name,rows,selected,selected_args,rclpy,storage_root=storage_root)
                time.sleep(2.)
            result['status']='NATIVE_MAPS_CLOSED_AND_EXPORTED_REQUIRES_REVIEW'
        finally: rclpy.shutdown()
    return result

if __name__=='__main__': raise SystemExit(main())
