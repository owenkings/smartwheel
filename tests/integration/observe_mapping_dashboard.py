#!/usr/bin/env python3
"""Metadata-only live dashboard check: never saves images or sends controls."""
import argparse
from collections import OrderedDict
import hashlib
import json
from pathlib import Path
import signal
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=180)
    parser.add_argument('--session-root',type=Path,required=True)
    args=parser.parse_args()
    from wc_runtime.cli import ROOT,target
    from wc_runtime.prepare_picker_input import project_path
    target();output=project_path(ROOT,args.output)
    session=project_path(ROOT,args.session_root)
    if output.exists():raise RuntimeError('Observer output already exists')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from sensor_msgs.msg import Image
    from nav_msgs.msg import OccupancyGrid
    from visualization_msgs.msg import MarkerArray
    from std_msgs.msg import String
    from rtabmap_msgs.msg import OdomInfo
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('mapping_dashboard_evidence_observer')
    stop=[False]
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.__setitem__(0,True))
    cameras={role:{'frames':0,'unique_stamps':0,'first_monotonic':None,'last_monotonic':None,'last_stamp':None,
                   'max_gap_s':0.,'gaps_over_1_5_s':0}
             for role in ('left_front','right_front','left_side','right_side')}
    def receive_camera(role,msg):
        row=cameras[role];stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        row['frames']+=1
        if row['last_stamp']==stamp:return
        now=time.monotonic()
        row['unique_stamps']+=1
        if row['last_monotonic'] is not None:
            gap=now-row['last_monotonic']
            row['max_gap_s']=max(row['max_gap_s'],gap)
            row['gaps_over_1_5_s']+=int(gap>1.5)
        row['last_stamp']=stamp
        row['first_monotonic']=row['first_monotonic'] or now
        row['last_monotonic']=now
        row.update(width=msg.width,height=msg.height,encoding=msg.encoding,frame_id=msg.header.frame_id)
    best=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions=[node.create_subscription(Image,'/wc_mapping/cameras/'+role+'/image_raw',
        lambda msg,role=role:receive_camera(role,msg),best) for role in cameras]
    odom_info={'messages':0,'lost_messages':0,'max_local_scan_bytes':0,'max_full_subscribers':0}
    def receive_info(msg):
        odom_info.update(messages=odom_info['messages']+1,
            lost_messages=odom_info['lost_messages']+int(msg.lost),
            max_local_scan_bytes=max(odom_info['max_local_scan_bytes'],len(msg.local_scan_map.data)),
            last_icp_inliers_ratio=msg.icp_inliers_ratio,
            last_stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
            last_guess_translation=[getattr(msg.guess.translation,k) for k in ('x','y','z')],
            last_guess_rotation=[getattr(msg.guess.rotation,k) for k in ('x','y','z','w')])
        if odom_info['messages']%10==0:
            odom_info['max_full_subscribers']=max(odom_info['max_full_subscribers'],
                node.count_subscribers('/wc_mapping/app/unused_odom_info_full'))
    subscriptions.append(node.create_subscription(OdomInfo,'/wc_mapping/app/odom_info',receive_info,10))
    grids={};grid_pending={'native':OrderedDict(),'display':OrderedDict()}
    grid_pairs={'matched':0,'mismatches':0}
    marker={'messages':0};manual={'messages':0,'max_control_transmissions':0,
        'max_control_write_attempts':0,'max_control_bytes_observed':0,'ever_peer_connected':False}
    def receive_manual(msg):
        value=json.loads(msg.data)
        if value['session_id']!=session.name:raise RuntimeError('Unexpected manual-status session')
        manual.update(messages=manual['messages']+1,state=value['state'],arm_allowed=value['arm_allowed'],
            max_control_transmissions=max(manual['max_control_transmissions'],value['control_transmissions']),
            max_control_write_attempts=max(manual['max_control_write_attempts'],value['control_write_attempts']),
            max_control_bytes_observed=max(manual['max_control_bytes_observed'],value['control_bytes_observed']),
            ever_peer_connected=manual['ever_peer_connected'] or value['peer_connected'])
    subscriptions.append(node.create_subscription(String,'/wc_mapping/wheel/manual_status',receive_manual,10))
    def receive_grid(kind,msg):
        row={'stamp':msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
            'frame_id':msg.header.frame_id,'width':msg.info.width,'height':msg.info.height,
            'resolution':msg.info.resolution,
            'origin':[msg.info.origin.position.x,msg.info.origin.position.y,msg.info.origin.position.z],
            'orientation':[getattr(msg.info.origin.orientation,k) for k in ('x','y','z','w')],
            'cells_sha256':hashlib.sha256(msg.data.tobytes()).hexdigest()}
        grids[kind]=row
        pending=grid_pending[kind];pending[row['stamp']]=row
        while len(pending)>8:pending.popitem(last=False)
        other='display' if kind=='native' else 'native'
        if row['stamp'] in grid_pending[other]:
            pair={kind:pending.pop(row['stamp']),other:grid_pending[other].pop(row['stamp'])}
            native,display=pair['native'],pair['display']
            same=all(native[k]==display[k] for k in ('stamp','frame_id','width','height','resolution','cells_sha256','orientation')) and native['origin'][:2]==display['origin'][:2]
            grid_pairs['matched']+=1
            grid_pairs['mismatches']+=int(not same)
            grid_pairs['latest_pair']=pair
    for kind,topic in [('native','grid_map'),('display','view_grid')]:
        subscriptions.append(node.create_subscription(OccupancyGrid,'/wc_mapping/app/'+topic,
            lambda msg,kind=kind:receive_grid(kind,msg),latched))
    def receive_markers(msg):
        marker.update(messages=marker['messages']+1,count=len(msg.markers),
            frames=sorted(set(m.header.frame_id for m in msg.markers)),
            frame_locked=all(m.frame_locked for m in msg.markers))
    subscriptions.append(node.create_subscription(MarkerArray,'/wc_mapping/app/view_markers',receive_markers,latched))
    start=time.monotonic()
    try:
        while not stop[0] and time.monotonic()-start<args.duration:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.destroy_node();rclpy.shutdown()
    last_camera=max((row['last_monotonic'] or 0) for row in cameras.values())
    for row in cameras.values():
        elapsed=(row['last_monotonic'] or 0)-(row['first_monotonic'] or 0)
        row['observed_hz']=(row['unique_stamps']-1)/elapsed if elapsed>0 else 0
        row['end_lag_from_latest_camera_s']=last_camera-(row['last_monotonic'] or 0)
    cameras_fresh_at_end=all(row['unique_stamps']>=10 and row['end_lag_from_latest_camera_s']<=1.5 for row in cameras.values())
    same=grid_pairs['matched']>0 and grid_pairs['mismatches']==0
    ground_matches=False;expected_ground_z=None
    initialization=session/'prior/initialization.json'
    if initialization.exists():
        initial=json.loads(initialization.read_text());runtime=json.loads((session/'runtime_config.json').read_text())
        if initial['session_id']!=session.name or runtime['session_id']!=session.name:raise RuntimeError('Unexpected initialization session')
        t=initial['T_reference_axle'];radius=runtime['wheel_candidate']['wheel_radius_m']
        expected_ground_z=t[2][3]-t[2][2]*radius
        display=grid_pairs.get('latest_pair',{}).get('display',{})
        ground_matches=bool(display) and abs(display['origin'][2]-expected_ground_z)<1e-8
    passed=cameras_fresh_at_end and same and marker.get('count',0)>=9 and \
        marker.get('frames')==['mapping_reference'] and marker.get('frame_locked') is True and ground_matches and \
        manual['messages']>=10 and manual['max_control_transmissions']==manual['max_control_write_attempts']==manual['max_control_bytes_observed']==0 and manual.get('arm_allowed') is False and manual['ever_peer_connected']
    result={'validation_level':'LIVE_STATIC','status':'PASS' if passed else 'FAIL',
        'cameras':cameras,'images_saved':False,'grids':grids,'grid_content_and_xy_preserved':same,'markers':marker,
        'all_cameras_fresh_relative_to_final_camera':cameras_fresh_at_end,
        'camera_interruption_observed':any(row['gaps_over_1_5_s'] for row in cameras.values()),
        'scope':'ROS metadata only; native screenshot requires separate visual inspection',
        'grid_pairs':grid_pairs,'unmatched_grid_stamps':{k:list(v) for k,v in grid_pending.items()},
        'expected_ground_z_m':expected_ground_z,'ground_display_matches_initial_mount':ground_matches,
        'manual':manual}
    result['odom_info_transport']=odom_info
    output.write_text(json.dumps(result,indent=2))
    print(json.dumps(result))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
