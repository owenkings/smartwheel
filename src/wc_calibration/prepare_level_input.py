#!/usr/bin/env python3
"""Prepare untouched cloud rows plus a hash-bound IMU leveling display reference."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from wc_calibration.alignment import _iter_h30_bag, h30_bag_samples, imu_gravity
from wc_calibration.core import CalibrationError, _hash
from wc_calibration.gravity_level import estimate_gravity_level
from wc_calibration.level_reference import build_level_reference
from wc_calibration.prepare_stability_input import prepare_study
from wc_runtime.project_paths import project_root
from wc_runtime.project_config import selected_config_path
from wc_runtime.device_bindings import load_device_bindings
from wc_runtime.prepare_picker_input import completed_study, project_path, read_json


def digest(path):
    hasher=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):hasher.update(block)
    return hasher.hexdigest()


def prepare_level(root,study_value,output):
    root=Path(root).resolve();study,stages=completed_study(root,study_value)
    output=project_path(root,output)
    if output.exists():raise CalibrationError('New prepared output required')
    output.parent.mkdir(parents=True,exist_ok=True)
    config_path=selected_config_path(root,'live_unvalidated.json');config=read_json(root,config_path)
    imu_sensor_id=load_device_bindings(root)['imu']['sensor_id']
    mounting=config['mounting_user_observations']['imu_axis_clarification']
    if mounting['rotation_status']!='USER_REPORTED_AXES_PENDING_GEOMETRY_CHECK':
        raise CalibrationError('Explicit user-reported mounting rotation required')
    manifest=read_json(root,study/'imu_bags.json')
    if manifest.get('status')!='LIDAR_AND_H30_CAPTURE_AUDITED':
        raise CalibrationError('All four new lidar and H30 bags must pass audit')
    data=prepare_study(study,config['sensor_ids'],identity_reference={
        'path':str(config_path),'sha256':digest(config_path)})
    scene=data['training'][0];metadata=scene['input_files']
    evidence={};gravities={};native={}
    for label,entry in stages.items():
        session=entry['session'];stage=study/label
        prov=read_json(root,stage/'bag_provenance.json')
        audit=read_json(root,stage/'bag_audit.json',20_000_000)
        if prov['source_session']!=session or audit.get('status')!='DATA_REVIEW_OK':
            raise CalibrationError('IMU bag session/audit mismatch')
        if manifest['stages'][label]['source_session']!=session or audit.get('h30_recorded') is not True:
            raise CalibrationError('Audited stage manifest does not match this H30 capture')
        bag=project_path(root,root/'data/bags'/session)
        if prov['bag_uri']!=str(bag):raise CalibrationError('Unexpected IMU bag path')
        capture=read_json(root,stage/'capture.json')
        if label in ('left_single','right_single'):
            side='left' if label=='left_single' else 'right'
            center=metadata[side]['raw_frame']['host_ns']
        else:
            row=capture['sides']['left'];center=(row['first']['host_ns']+row['last']['host_ns'])//2
        selection={'schema_version':1,'source_mode':'real','sensor_id':imu_sensor_id,
            'coordinate_convention':'H30_NATIVE_UNVALIDATED','acceleration_units':'m/s^2',
            'angular_velocity_units':'rad/s','acceleration_includes_gravity':True,
            'acceleration_sign':'specific_force','units_evidence':'Reviewed H30 TLV native XYZ: acceleration 1e-6 m/s^2; angular velocity 1e-6 deg/s converted to rad/s. User-reported same-axis mounting remains a candidate.',
            'bag_uri':str(bag),'topic':'/wc_mapping/imu/source_frame',
            'start_ns':center-2_500_000_000,'end_ns':center+2_500_000_000}
        host_times=[];arrival_delays=[]
        def records():
            for stamp,msg in _iter_h30_bag(selection):
                if msg.session_id!=session:raise CalibrationError('H30 recording session does not match selected lidar stage')
                host_ns=int(msg.host_receive_time.sec)*10**9+int(msg.host_receive_time.nanosec)
                host_times.append(host_ns);arrival_delays.append(stamp-host_ns)
                yield stamp,msg
        samples=h30_bag_samples(selection,records=records())
        if not host_times or not min(host_times)<=center<=max(host_times):
            raise CalibrationError('Selected H30 host receive interval does not cover the selected lidar frame')
        # Existing native estimator also enforces explicit minimum window duration.
        native[label]=imu_gravity(samples)
        acceleration=[s['acceleration'] for s in samples['samples']]
        gyro=[s['angular_velocity'] for s in samples['samples']]
        gravity=estimate_gravity_level(acceleration,mounting['R_left_imu_candidate'],angular_velocity=gyro)
        gravities[label]=gravity
        payload=_hash(samples)
        sample_path=output.parent/(label+'_imu_samples_'+payload[:12]+'.json')
        if sample_path.exists():
            if _hash(json.loads(sample_path.read_text()))!=payload:raise CalibrationError('IMU evidence path collision')
        else:
            with sample_path.open('x') as stream:json.dump(samples,stream,indent=2,allow_nan=False)
        files=[bag/'metadata.yaml',*sorted(bag.glob('*.db3'))]
        evidence[label]={'source_session':session,'bag_uri':str(bag),'topic':selection['topic'],
            'bag_files':{p.name:{'sha256':digest(p),'bytes':p.stat().st_size} for p in files},
            'bag_audit':{'path':str(stage/'bag_audit.json'),'sha256':digest(stage/'bag_audit.json')},
            'sample_window_ns':[selection['start_ns'],selection['end_ns']],
            'selected_lidar_host_ns':center,'sample_count':len(samples['samples']),
            'imu_host_receive_interval_ns':[min(host_times),max(host_times)],
            'bag_minus_host_receive_interval_ns':[min(arrival_delays),max(arrival_delays)],
            'first_sample_ns':samples['samples'][0]['time_ns'],'last_sample_ns':samples['samples'][-1]['time_ns'],
            'timestamp_basis':'bag record time window around lidar host receive time; arrival-only, not measurement synchronization',
            'samples_path':str(sample_path),'samples_sha256':payload,
            'gravity':gravity,'native_tilt':native[label]}
    up=np.array(gravities['left_single']['up_unit_in_left'])
    angles={k:float(np.degrees(np.arccos(np.clip(up@np.array(v['up_unit_in_left']),-1,1)))) for k,v in gravities.items()}
    if max(angles.values())>0.5:
        raise CalibrationError('Gravity changed over 0.5 degrees between capture stages; preserve evidence and recapture static scene')
    provenance={'reference_stage':'left_single','stages':evidence,'interstage_angle_deg':angles,
        'max_interstage_angle_deg_candidate_gate':0.5,'scene_static_source':'onsite operator confirmation',
        'physical_static_independently_verified':False,'measurement_time_synchronized':False}
    scene['level_reference']=build_level_reference(scene['id'],metadata,gravities['left_single'],provenance,mounting)
    data['input_preparation']['leveling']='Display-only gravity reference; original cloud rows and original-frame extrinsic contract preserved'
    with output.open('x',encoding='utf-8') as stream:json.dump(data,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({'status':'PREPARED_WITH_LEVEL_REFERENCE','output':str(output),
        'scene_id':scene['id'],'level_reference_sha256':scene['level_reference']['payload_sha256'],
        'left_gravity':gravities['left_single'],'interstage_angle_deg':angles}))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=project_root(start=__file__)
    prepare_level(root,args.study,args.output)

if __name__=='__main__':main()
