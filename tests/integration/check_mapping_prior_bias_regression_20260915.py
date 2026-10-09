"""Old/new production prior on the same original packets, without ROS nodes."""
import bisect
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np
from wc_runtime.cli import ROOT, target
from wc_runtime import mapping_prior as current
from analyze_mapping_geometry_20260915 import bag_topics, messages, stamp


def main():
    target()
    source=ROOT/'reports/maps/map_right_20260914T153949Z_680ffa'
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result-name',default='prior_runtime_regression.json')
    name=parser.parse_args().result_name
    if Path(name).name!=name or not name.endswith('.json'):
        raise ValueError('result name must be a JSON filename inside the diagnostic directory')
    output=ROOT/'reports/map_geometry_fix_20260915'/name
    assert not output.exists()
    previous=ROOT/'reports/map_geometry_fix_20260915/runtime_before/src/wc_runtime/mapping_prior.py'
    spec=importlib.util.spec_from_file_location('wc_runtime.mapping_prior_baseline',previous)
    before=importlib.util.module_from_spec(spec);sys.modules[spec.name]=before;spec.loader.exec_module(before)
    config=json.loads((source/'prior_config.json').read_text())
    priors=[before.MotionPrior(config,source.name),current.MotionPrior(config,source.name)]
    events=[]
    for _,msg,_ in messages(bag_topics(source),config['imu_topic']):
        g,a=msg.imu.angular_velocity,msg.imu.linear_acceleration
        data=dict(stamp_ns=stamp(msg.host_receive_time),monotonic_ns=int(msg.host_monotonic_ns),
            acceleration=[a.x,a.y,a.z],angular_velocity=[g.x,g.y,g.z],sensor_id=msg.sensor_id,
            session_id=msg.session_id,stream_epoch=msg.stream_epoch,sequence=int(msg.frame_sequence),
            coordinate_convention=msg.coordinate_convention,time_source=msg.time_source,common_time_valid=msg.common_time_valid)
        events.append((data['stamp_ns'],1,data['sequence'],data))
    for _,msg,_ in messages(bag_topics(source),config['wheel_topic']):
        data=json.loads(msg.data);events.append((data['stamp_ns'],0,data['sequence'],data))
    events.sort(key=lambda row:row[:3])
    scans=sorted(stamp(msg.header.stamp) for _,msg,_ in messages(bag_topics(source),'/wc_mapping/app/scan_cloud'))
    index=0;compared=0;dropped=0;elapsed=[0.,0.];max_pose_difference=0.
    for event_stamp,kind,_,data in events:
        for j,prior in enumerate(priors):
            started=time.perf_counter()
            if kind==0:prior.add_wheel(data)
            else:prior.add_imu(**data)
            elapsed[j]+=time.perf_counter()-started
        if priors[0].initialization is None:continue
        watermark=min(priors[0].imu[-1]['stamp'],priors[0].wheel[-1]['stamp'])
        while index<len(scans) and scans[index]<=watermark:
            query=scans[index];index+=1;poses=[];errors=[]
            for prior in priors:
                try:
                    prior.require_observed_cloud_time(query)
                    pose=prior.pose_at(query)
                    prior.record_forwarded(query,pose);poses.append(pose);errors.append(None)
                except (before.DroppedCloud,current.DroppedCloud) as error:
                    poses.append(None);errors.append(str(error))
            assert errors[0]==errors[1],errors
            if errors[0] is not None:dropped+=1;continue
            difference=float(np.abs(poses[0]-poses[1]).max());max_pose_difference=max(max_pose_difference,difference)
            assert np.array_equal(poses[0],poses[1]),(query,difference)
            compared+=1
    assert compared>=500 and priors[1].failure is None
    assert priors[1].bias_report()['candidate_windows']>0
    assert not priors[1].bias_report()['bias_estimated']
    report={'status':'PASS','verification_level':'REAL_BAG','hardware_accessed':False,'ros_nodes_started':False,
        'source':source.name,'packet_order':'deterministic source time, wheel first at ties; identical for both variants, not original callback timing',
        'packets':len(events),'compared_cloud_poses':compared,'equally_dropped_clouds':dropped,
        'poses_exactly_equal':True,'maximum_pose_element_difference':max_pose_difference,
        'source_callback_elapsed_s':{'before':elapsed[0],'after':elapsed[1]},
        'background_bias':priors[1].bias_report(), 'coverage':priors[1].coverage_report()}
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({key:report[key] for key in ['status','packets','compared_cloud_poses','poses_exactly_equal','source_callback_elapsed_s']}))


if __name__=='__main__':main()
