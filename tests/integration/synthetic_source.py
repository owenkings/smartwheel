#!/usr/bin/env python3
"""Deterministic dual forward-looking scene for software tests only.

Plane samples are field-of-view filtered; this simplified model does not model
optical interference, multipath, HDR timing, beam occlusion or device accuracy.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation


def prepare(session, period_s=0.5, with_wheel_filter_fixture=False):
    import re
    if not re.fullmatch('[A-Za-z0-9][A-Za-z0-9_.-]{0,63}',session):
        raise ValueError('invalid session identifier')
    # Permit a separately labelled 1.25 Hz functional check after the 2 Hz
    # capacity failure. Production age/queue/ICP limits are unchanged.
    if not 0.1 <= period_s <= 0.8:
        raise ValueError('explicit synthetic period must be 0.1..0.8 seconds')
    root=Path('/home/nvidia/wheelchair/data/synthetic')/session
    root.mkdir(parents=True,exist_ok=False)
    extrinsic=np.eye(4);extrinsic[1,3]=-0.6
    calibration=dict(calibration_id='SYNTHETIC_cal_v1',status='CANDIDATE',source_mode='synthetic',
                     sensor_ids={'left':'SYNTHETIC_LEFT','right':'SYNTHETIC_RIGHT'},T_left_right=extrinsic.tolist())
    profile=dict(resolution=0.1,ground_z=-1.0,ground_tolerance=0.025,
                 collision_min_height=0.1,collision_max_height=1.6,ground_valid=True,
                 sensor_mode='dual',max_voxels=500000,max_grid_cells=1000000)
    config=dict(schema_version=1,session_id=session,source_mode='synthetic',sensor_mode='dual',
                execution_mode='synthetic',session_root=str(root),run_root='/home/nvidia/wheelchair/.phase1_runtime',
                synthetic_source_reliability='reliable',
                graph_epoch='synthetic_graph_1',sensor_ids=calibration['sensor_ids'],calibration=calibration,
                clock_model_ids={'left':'synthetic_clock','right':'synthetic_clock'},
                time_model_id='synthetic_clock',graph_noise_model='diagonal_assumption',
                graph_noise_diagonal=[0.0001,0.0001,0.0001,0.0001,0.0001,0.0001],
                graph_noise_note='Explicit synthetic graph optimization assumption, not measured sensor covariance',
                graph={'stm_size':10},
                icp_timeout_ns=3_000_000_000,min_icp_correspondences=20,
                fusion_policy={'max_age_ns':1_000_000_000,'max_pair_ns':20_000_000,
                               'max_motion_speed_mps':0.2,'max_motion_angular_radps':0.2},
                synthetic={'seed':40911,'frame_count':60,'period_s':period_s,'startup_delay_s':6,'post_publish_hold_s':120,
                           'rate_note':'Explicit diagnostic integration rate; prior incomplete attempts are preserved and are not throughput passes',
                           'transport_note':'Reliable synthetic source for deterministic integration; hardware SourceFrame acquisition uses best effort and needs separate validation',
                           'scene_model':'Independent samples of shared planes with per-sensor FOV filtering; no occlusion model'})
    if with_wheel_filter_fixture:
        config['cloud_input']='xtcfg_filtered'
        config['motion_prior']={
            'enabled':True,'coordinate_status':'SYNTHETIC','time_status':'SYNTHETIC',
            'T_rig_axle':np.eye(4).tolist(),'max_time_uncertainty_ns':1_000_000,
            'note':'Explicit synthetic coincident axle/rig reference, never a measured physical mounting transform'}
        config['synthetic'].update(with_wheel_filter_fixture=True,
            wheel_model='Synthetic axle pose moving straight forward and backward; no real encoder feedback',
            filter_model='Deterministic every-second-point fixture; not the XT vendor filter or filter-quality evidence',
            source_stamp_lag_ns=50_000_000,wheel_offsets_ns=[-20_000_000,-10_000_000])
    for filename,value in (('session.json',config),('calibration.json',calibration),('map_config.json',profile)):
        (root/filename).write_text(json.dumps(value,indent=2),encoding='utf-8')
    print(root/'session.json')
    return root/'session.json'


def geometry(seed,side):
    rng=np.random.default_rng(seed+(0 if side=='left' else 1))
    # Each side has independently sampled observations and a separate FOV.
    forward=np.column_stack((np.full(1000,4.5),rng.uniform(-2.5,2.5,1000),rng.uniform(-1,1.5,1000)))
    left=np.column_stack((rng.uniform(.7,4.5,800),np.full(800,2.0),rng.uniform(-1,1.5,800)))
    right=np.column_stack((rng.uniform(.7,4.5,800),np.full(800,-2.0),rng.uniform(-1,1.5,800)))
    floor=np.column_stack((rng.uniform(.7,4.5,1000),rng.uniform(-2,2,1000),np.full(1000,-1.0)))
    low=np.column_stack((rng.uniform(2,2.3,100),rng.uniform(-.2,.2,100),np.full(100,-.8)))
    return np.concatenate((forward,left,right,floor,low))


def truth(index,count):
    # Smooth, slow forward and return motion, ending at the same rig pose.
    phase=2*np.pi*index/(count-1)
    pose=np.eye(4)
    pose[0,3]=0.12*(1-np.cos(phase))
    pose[1,3]=0.03*np.sin(phase)
    pose[:3,:3]=Rotation.from_euler('z',0.05*np.sin(phase)).as_matrix()
    return pose


def wheel_fixture_truth(index,count):
    """Continuous straight-line trajectory compatible with differential drive."""
    phase=2*np.pi*np.clip(index,0,count-1)/(count-1)
    pose=np.eye(4)
    pose[0,3]=0.12*(1-np.cos(phase))
    return pose


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--prepare',metavar='SESSION_ID')
    group.add_argument('--session-config',type=Path)
    parser.add_argument('--period-s',type=float,default=0.5,help='Preparation only: explicit test period in seconds')
    parser.add_argument('--with-wheel-filter-fixture',action='store_true',
                        help='Prepare explicit synthetic wheel prediction + filtered/raw branch integration')
    args,ros_args=parser.parse_known_args(argv)
    if args.prepare:
        prepare(args.prepare,args.period_s,args.with_wheel_filter_fixture);return 0
    config=json.loads(args.session_config.read_text())
    if config['source_mode']!='synthetic':raise ValueError('synthetic publisher refuses real session')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from wc_interfaces.msg import SourceFrame
    from wc_fusion.ros_node import cloud_message
    from wc_fusion.core import transform
    options=config['synthetic'];count=options['frame_count'];index=0
    with_wheel=options.get('with_wheel_filter_fixture',False)
    if args.with_wheel_filter_fixture and not with_wheel:
        raise ValueError('wheel/filter fixture must be enabled when preparing the session')
    if with_wheel and (config.get('cloud_input')!='xtcfg_filtered' or not config.get('motion_prior',{}).get('enabled')):
        raise ValueError('wheel/filter fixture requires its matching configured processing path')
    base=None
    truth_rows=[]
    wheel_rows=[]
    world={side:geometry(options['seed'],side) for side in ('left','right')}
    extrinsics={'left':np.eye(4),'right':np.asarray(config['calibration']['T_left_right'])}
    rclpy.init(args=ros_args)
    node=Node('wc_synthetic_dual_source')
    pubs={side:node.create_publisher(SourceFrame,'/wc_mapping/lidar_'+side+'/source_frame',QoSProfile(depth=8,reliability=ReliabilityPolicy.RELIABLE)) for side in ('left','right')}
    if with_wheel:
        import copy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        wheel_qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
        wheel_pub=node.create_publisher(Odometry,'/wc_mapping/wheel/odom',wheel_qos)
        wheel_diag_pub=node.create_publisher(String,'/wc_mapping/wheel/diagnostics',wheel_qos)
        filtered_pubs={side:node.create_publisher(SourceFrame,'/wc_mapping/lidar_'+side+'/source_frame_filtered',
            QoSProfile(depth=8,reliability=ReliabilityPolicy.RELIABLE)) for side in ('left','right')}
    start=time.monotonic()+options['startup_delay_s']
    try:
        while rclpy.ok() and time.monotonic()<start:
            rclpy.spin_once(node,timeout_sec=.05)
        base=node.get_clock().now().nanoseconds
        if with_wheel:base-=options['source_stamp_lag_ns']
        next_frame=time.monotonic()
        for index in range(count):
            if not rclpy.ok():break
            pose=wheel_fixture_truth(index,count) if with_wheel else truth(index,count)
            stamp=base+round(index*options['period_s']*1e9)
            if with_wheel:
                if stamp>node.get_clock().now().nanoseconds:
                    raise ValueError('synthetic wheel fixture refuses future source timestamps')
                for offset_index,offset in enumerate(options['wheel_offsets_ns']):
                    wheel_stamp=stamp+offset
                    wheel_index=(wheel_stamp-base)/(options['period_s']*1e9)
                    wheel_pose=wheel_fixture_truth(wheel_index,count)
                    wheel_msg=Odometry()
                    wheel_msg.header.stamp.sec,wheel_msg.header.stamp.nanosec=divmod(wheel_stamp,1_000_000_000)
                    wheel_msg.header.frame_id='wheel_odom';wheel_msg.child_frame_id='axle_link'
                    wheel_msg.pose.pose.position.x=float(wheel_pose[0,3]);wheel_msg.pose.pose.orientation.w=1.
                    phase=2*np.pi*np.clip(wheel_index,0,count-1)/(count-1)
                    wheel_msg.twist.twist.linear.x=float(.12*np.sin(phase)*2*np.pi/((count-1)*options['period_s'])) if wheel_index>=0 else 0.
                    for diagonal in (0,7,14,21,28,35):
                        wheel_msg.pose.covariance[diagonal]=1e6;wheel_msg.twist.covariance[diagonal]=1e6
                    wheel_msg.twist.covariance[0]=.0001;wheel_msg.twist.covariance[35]=.0001
                    wheel_sequence=2*index+offset_index+1
                    diag={'schema':'wc_wheel_diagnostics_v1','state':'FEEDBACK','time_valid':True,
                        'time_source':'external_common_time','stamp_ns':wheel_stamp,'uncertainty_ns':10_000,
                        'stream_epoch':'SYNTHETIC_WHEEL_BOOT','sequence':wheel_sequence,'publishes_tf':False,
                        'source_mode':'synthetic','fixture':'not_real_encoder_feedback'}
                    wheel_pub.publish(wheel_msg);wheel_diag_pub.publish(String(data=json.dumps(diag)))
                    wheel_rows.append(dict(sequence=wheel_sequence,stamp_ns=wheel_stamp,t_ref_ns=stamp,
                        actual_publish_unix_ns=time.time_ns(),T_wheel_axle=wheel_pose.tolist()))
                    # Separate DDS topics have no cross-topic ordering guarantee.
                    # Give the already running fusion callback time to join each
                    # explicit wheel/diagnostic pair before delivering lidar.
                    until=time.monotonic()+.02
                    while rclpy.ok() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
            for side in ('left','right'):
                points=transform(np.linalg.inv(pose@extrinsics[side]),world[side])
                valid=(points[:,0]>.1)&(np.abs(np.arctan2(points[:,1],points[:,0]))<np.deg2rad(62))&(
                    np.abs(np.arctan2(points[:,2],np.linalg.norm(points[:,:2],axis=1)))<np.deg2rad(48))
                points=points[valid]
                msg=SourceFrame()
                msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,1_000_000_000)
                msg.header.frame_id='lidar_'+side
                msg.session_id=config['session_id'];msg.side=side;msg.sensor_id=config['sensor_ids'][side]
                msg.stream_epoch='synthetic_boot';msg.frame_sequence=index+1
                msg.cloud=cloud_message(points,msg.header.stamp,msg.header.frame_id)
                msg.source_config_hash=hashlib.sha256(json.dumps(options,sort_keys=True).encode()).hexdigest()
                msg.coordinate_convention='FLU';msg.units='m'
                msg.device_timestamp_valid=True;msg.device_timestamp_raw=stamp;msg.device_timestamp_unit='ns'
                msg.device_time_type='synthetic_truth';msg.device_sync_state='SYNTHETIC_KNOWN'
                msg.host_receive_time=node.get_clock().now().to_msg();msg.host_monotonic_ns=time.monotonic_ns()
                msg.common_time_valid=True;msg.common_time_ns=stamp;msg.clock_model_id='synthetic_clock'
                msg.time_source='synthetic';msg.uncertainty_valid=True;msg.uncertainty_ns=10_000
                msg.raw_count=msg.valid_count=len(points)
                publish_start=time.monotonic_ns()
                pubs[side].publish(msg)
                if with_wheel:
                    filtered=copy.deepcopy(msg)
                    filtered.cloud=cloud_message(points[::2],msg.header.stamp,msg.header.frame_id)
                    filtered.raw_count=filtered.valid_count=len(points[::2])
                    filtered.source_config_hash=hashlib.sha256((msg.source_config_hash+':SYNTHETIC_EVERY_SECOND_POINT').encode()).hexdigest()
                    filtered.diagnostic_flags=[
                        'raw_source_config_hash='+msg.source_config_hash,
                        'before_host_filter_cloud_sha256='+hashlib.sha256(bytes(msg.cloud.data)).hexdigest(),
                        'SYNTHETIC_FILTER_SUBSAMPLE_ONLY_NOT_VENDOR_FILTER']
                    filtered_pubs[side].publish(filtered)
                truth_rows.append(dict(kind='source_publish',side=side,frame_sequence=index+1,
                    host_monotonic_ns=msg.host_monotonic_ns,publish_duration_ns=time.monotonic_ns()-publish_start,
                    actual_publish_unix_ns=time.time_ns(),stamp_ns=stamp))
            truth_rows.append(dict(frame_sequence=index+1,stamp_ns=stamp,T_map_rig=pose.tolist()))
            next_frame+=options['period_s']
            while rclpy.ok() and time.monotonic()<next_frame:
                rclpy.spin_once(node,timeout_sec=min(.02,max(0,next_frame-time.monotonic())))
        truth_result=dict(source_mode='synthetic',
            observations_model=options['scene_model'],seed=options['seed'],poses=[r for r in truth_rows if r.get('kind')!='source_publish'],
            source_publish_timings=[r for r in truth_rows if r.get('kind')=='source_publish'])
        if with_wheel:truth_result.update(wheel_model=options['wheel_model'],filter_model=options['filter_model'],wheel_publish_timings=wheel_rows)
        (Path(config['session_root'])/'synthetic_truth.json').write_text(json.dumps(truth_result,indent=2))
        # Allow final ICP/graph callbacks to complete before supervisor stops the
        # session. The finite source is a test fixture, never a physical driver.
        until=time.monotonic()+options.get('post_publish_hold_s',120)
        while rclpy.ok() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0


if __name__=='__main__':raise SystemExit(main())
