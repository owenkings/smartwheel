#!/usr/bin/env python3
"""Rebuild a native map from recorded wheel/IMU poses and matching scans.

REAL_BAG_REBUILD: no device nodes, control commands, original ICP odometry,
original maps, or recorded ICP TF are published. The unchanged wheel/IMU
integration was used during original acquisition; this checks the new authority
adapter and native mapping, not a new raw-sensor integration or motion trial.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def file_hash(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()


def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


def main():
    import fcntl
    import yaml
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry,OccupancyGrid
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from rtabmap_msgs.msg import MapGraph
    from wc_runtime.cli import ROOT,target
    from wc_runtime.mapping_odometry import adapt_prior_odometry,authority_identity_transform
    from wc_runtime.mapping_monitor import Health
    from wc_runtime.mapping_visuals import build_scene,display_grid
    target()
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-name',default='native_rebuild_01')
    args=parser.parse_args()
    if not args.run_name.replace('_','').isalnum():raise ValueError('Simple new run name required')
    if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Requires isolated localhost domain 87')
    source=ROOT/'reports/maps/map_right_20260914T113532Z_3d234e'
    out=ROOT/'reports/map_wheel_authority_20260914'/args.run_name
    out.mkdir(exist_ok=False);(out/'slam').mkdir()
    lock=(ROOT/'.phase1_runtime/locks/domain-87-rebuild.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    bag=next((source/'bag').glob('*.db3'));before=file_hash(bag)
    original_config=json.loads((source/'runtime_config.json').read_text())
    weights=json.loads((ROOT/'config/mapping_live.json').read_text())['wheel_imu_covariance']
    types={'/wc_mapping/app/scan_cloud':PointCloud2,'/wc_mapping/app/prior_odom':Odometry}
    rows={name:{} for name in types}
    with sqlite3.connect(bag.as_uri()+'?mode=ro',uri=True) as db:
        ids={i:name for i,name in db.execute('select id,name from topics') if name in types}
        for tid,name in ids.items():
            for raw, in db.execute('select data from messages where topic_id=? order by timestamp',(tid,)):
                msg=deserialize_message(raw,types[name]);key=stamp(msg)
                if key in rows[name]:raise RuntimeError('Duplicate recorded source stamp')
                rows[name][key]=(msg,hashlib.sha256(raw).hexdigest())
    clouds=rows['/wc_mapping/app/scan_cloud'];priors=rows['/wc_mapping/app/prior_odom']
    if set(clouds)!=set(priors) or len(clouds)<100:raise RuntimeError('Recorded scan/pose pairs incomplete')
    stamps=sorted(clouds);first=stamps[0]
    scene=build_scene(original_config,json.loads((source/'prior/initialization.json').read_text()))
    spec=module(ROOT/'src/wc_bringup/launch/mapping_app.launch.py','native_wheel_spec').build_spec(out,odometry_source='wheel_imu')
    if len(spec['nodes'])!=1 or spec['nodes'][0]['executable']!='rtabmap':raise RuntimeError('Unexpected native graph')
    native=copy.deepcopy(spec['nodes'][0]);parameters=native['parameters'][0];parameters['use_sim_time']=True
    params=out/'native_params.yaml'
    params.write_text(yaml.safe_dump({'/**':{'ros__parameters':parameters}},sort_keys=False))
    (out/'native_spec.json').write_text(json.dumps(spec,indent=2))
    native_command=[sys.executable,'-m','wc_runtime.component','--parent',str(os.getpid()),'--sigint-grace-s','120','--',
        '/opt/ros/humble/lib/rtabmap_slam/rtabmap','--ros-args','--params-file',str(params),
        '-r','__node:=rtabmap','-r','__ns:=/wc_mapping/app','--log-level','warn']
    for src,dst in native['remappings']:native_command+=['-r',src+':='+dst]
    capture=module(ROOT/'tests/integration/check_rviz_display.py','wheel_rebuild_capture')
    env=capture.desktop_environment();env.update(LIBGL_ALWAYS_SOFTWARE='1',__GLX_VENDOR_LIBRARY_NAME='mesa')
    view=yaml.safe_load((ROOT/'config/rviz/mapping_live.rviz').read_text())
    for d in view['Visualization Manager']['Displays']:
        topic=d.get('Topic',{}).get('Value')
        if topic=='/wc_mapping/app/odom':d['Name']='轮速 + IMU 里程计（录包重建）'
        if topic=='/wc_mapping/app/prior_odom':d['Enabled']=d['Value']=False
    view_path=out/'display_only.rviz';view_path.write_text(yaml.safe_dump(view,allow_unicode=True,sort_keys=False))
    gui_command=[sys.executable,'-m','wc_runtime.component','--parent',str(os.getpid()),'--',
        str(ROOT/'install/main/wc_bringup/lib/wc_bringup/mapping_rviz'),'-d',str(view_path),'--ros-args',
        '-r','__node:=wheel_recorded_rebuild_rviz','-r','/tf:=/wc_mapping/app/tf',
        '-r','/tf_static:=/wc_mapping/app/tf_static','-p','use_sim_time:=true']
    report={'status':'FAIL','validation_level':'REAL_BAG_REBUILD','source_bag':str(bag),'source_bag_sha256':before,
        'source_session':source.name,'hardware_started':False,'control_commands_sent':False,
        'recorded_input_topics':list(types),'recorded_icp_odom_tf_maps_published':False,
        'wheel_imu_covariance':weights,'source_pairs':len(stamps),'native_spec':str(out/'native_spec.json'),
        'effective_native_params':str(params),'effective_native_command':native_command,
        'offline_override':{'use_sim_time':True,'clock_source':'original paired scan/pose timestamps'},
        'limitation':'Reuses recorded wheel/IMU integration output, with original scans/stamps; no raw-sensor reintegration, new motion or accuracy acceptance.',
        'published_pairs':0,'graphs':[],'input_pair_hashes':[]}
    rclpy.init(args=[]);node=rclpy.create_node('wheel_recorded_rebuild')
    for _ in range(20):rclpy.spin_once(node,timeout_sec=.05)
    foreign=[n for n,ns in node.get_node_names_and_namespaces() if n!=node.get_name()]
    if foreign:raise RuntimeError('Domain occupied: '+repr(foreign))
    qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cloud_pub=node.create_publisher(PointCloud2,'/wc_mapping/app/scan_cloud',qos)
    prior_pub=node.create_publisher(Odometry,'/wc_mapping/app/prior_odom',qos)
    odom_pub=node.create_publisher(Odometry,'/wc_mapping/app/odom',qos)
    tf_pub=node.create_publisher(TFMessage,'/wc_mapping/app/tf',qos)
    clock_pub=node.create_publisher(Clock,'/clock',10)
    grid_pub=node.create_publisher(OccupancyGrid,'/wc_mapping/app/view_grid',latched)
    health=Health(out/'health',args.run_name,'right',odometry_source='wheel_imu')
    def receive_grid(m):health.receive_grid(m);grid_pub.publish(display_grid(m,scene['ground_z_m']))
    def receive_graph(m):
        report['graphs'].append({'stamp_ns':stamp(m),'nodes':len(m.poses_id),
            'ids':list(m.poses_id),'positions':[[p.position.x,p.position.y,p.position.z] for p in m.poses]})
    subscriptions=[node.create_subscription(PointCloud2,'/wc_mapping/app/cloud_map',health.receive_cloud,latched),
        node.create_subscription(OccupancyGrid,'/wc_mapping/app/grid_map',receive_grid,latched),
        node.create_subscription(MapGraph,'/wc_mapping/app/mapGraph',receive_graph,latched),
        node.create_subscription(Odometry,'/wc_mapping/app/odom',lambda m:health.receive('odom'),qos),
        node.create_subscription(Odometry,'/wc_mapping/app/prior_odom',lambda m:health.receive('prior_odom'),qos)]
    slam=gui=None
    logs=[]
    try:
        for name,command,child_env in [('native',native_command,os.environ.copy()),('rviz',gui_command,env)]:
            log=(out/(name+'.log')).open('x');logs.append(log)
            child=subprocess.Popen(command,env=child_env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            if name=='native':slam=child
            else:gui=child
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            if slam.poll() is not None or gui.poll() is not None:raise RuntimeError('Owned component exited during startup')
            clock_pub.publish(Clock(clock=priors[first][0].header.stamp))
            rclpy.spin_once(node,timeout_sec=.1)
            if cloud_pub.get_subscription_count()>=2 and odom_pub.get_subscription_count()>=3:break
        else:raise RuntimeError('Expected native/GUI subscriptions not ready')
        health.started=time.monotonic()  # begin this offline test's output deadline at playback
        started=time.monotonic()
        for key in stamps:
            deadline=started+(key-first)/1e9
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=min(.02,max(0,deadline-time.monotonic())))
                if slam.poll() is not None or gui.poll() is not None:raise RuntimeError('Owned component exited during replay')
            cloud,cloud_hash=clouds[key];prior,prior_hash=priors[key]
            authority=adapt_prior_odometry(prior,weights)
            transform=TransformStamped();transform.header=copy.deepcopy(prior.header);transform.child_frame_id=prior.child_frame_id
            transform.transform.translation.x=prior.pose.pose.position.x
            transform.transform.translation.y=prior.pose.pose.position.y
            transform.transform.translation.z=prior.pose.pose.position.z
            transform.transform.rotation=copy.deepcopy(prior.pose.pose.orientation)
            clock_pub.publish(Clock(clock=prior.header.stamp))
            tf_pub.publish(TFMessage(transforms=[authority_identity_transform(prior),transform]))
            prior_pub.publish(prior);odom_pub.publish(authority);cloud_pub.publish(cloud)
            report['published_pairs']+=1
            report['input_pair_hashes'].append({'stamp_ns':key,'cloud_sha256':cloud_hash,'prior_sha256':prior_hash})
            rclpy.spin_once(node,timeout_sec=.001)
            if not health.tick():raise RuntimeError(health.failure)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if health.failure or health.counts.get('cloud_map',0)<3 or health.counts.get('grid_map',0)<3:
            raise RuntimeError('Native maps did not repeatedly update')
        positions=[p for g in report['graphs'] for p in g['positions']]
        if not positions or max(p[0] for p in positions)-min(p[0] for p in positions)<1.0:
            raise RuntimeError('Native graph did not follow the recorded moving wheel/IMU trajectory')
        report['graph_x_span_m']=max(p[0] for p in positions)-min(p[0] for p in positions)
        report['window']=capture.capture_owned_window(gui,out/'final_map.png',env,activate_owned=True,qt_only=True)
        report['status']='REBUILT_REQUIRES_EXPORT_AND_VISUAL_REVIEW'
    except Exception as error:report['error']=repr(error)
    finally:
        for name,child in [('rviz',gui),('native',slam)]:
            if child is None:continue
            if child.poll() is None:child.send_signal(signal.SIGINT)
            try:child.wait(timeout=140 if name=='native' else 20)
            except subprocess.TimeoutExpired:
                report.setdefault('cleanup_errors',[]).append(name+' SIGINT timeout')
                # Keep the descendant-owning wrapper alive; do not orphan its tree.
                report.setdefault('unreaped_owners',[]).append({'component':name,'pid':child.pid})
                report['status']='FAIL'
            report[name+'_exit_code']=child.returncode
            if child.returncode!=0:report['status']='FAIL'
        try:health.close(timeout_s=30)
        except Exception as error:report['status']='FAIL';report['health_close_error']=repr(error)
        report['health']=health.snapshot(persist=False)
        node.destroy_node();rclpy.shutdown();lock.close()
        for log in logs:log.close()
    try:
        if report['status']=='REBUILT_REQUIRES_EXPORT_AND_VISUAL_REVIEW':
            db=out/'slam/rtabmap.db'
            if 'Saving database/long-term memory...done!' not in (out/'native.log').read_text(errors='replace'):
                raise RuntimeError('Native database close was not confirmed')
            with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as c:
                if c.execute('pragma integrity_check').fetchall()!=[('ok',)]:raise RuntimeError('Native DB integrity failed')
                report['database_nodes']=c.execute('select count(*) from Node').fetchone()[0]
            export=out/'export';export.mkdir()
            export_db=export/'native_export_copy.db'
            with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as source_db:
                with sqlite3.connect(export_db) as copy_db:source_db.backup(copy_db)
            command=['/opt/ros/humble/bin/rtabmap-export','--scan','--cloud','--map','--poses','--poses_format','11',
                '--opt','2','--voxel','.03','--output','wheel_imu_map','--output_dir',str(export),str(export_db)]
            before_db=file_hash(db)
            with (export/'export.log').open('x') as log:
                completed=subprocess.run(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,timeout=120)
            if completed.returncode or before_db!=file_hash(db):raise RuntimeError('Native export failed or modified DB')
            report['export_files']={p.name:{'bytes':p.stat().st_size,'sha256':file_hash(p)} for p in export.iterdir() if p.is_file()}
            if not any(x.endswith('.ply') for x in report['export_files']) or not any(x.endswith('.yaml') for x in report['export_files']):
                raise RuntimeError('Missing native 3D/2D export')
            report['status']='PASS_REQUIRES_VISUAL_REVIEW'
    except Exception as error:report['status']='FAIL';report['export_or_integrity_error']=repr(error)
    try:
        after=file_hash(bag)
        report.update(source_bag_after_sha256=after,source_bag_unchanged=before==after)
        if before!=after:raise RuntimeError('Original user bag changed')
    except Exception as error:report['status']='FAIL';report['source_integrity_error']=repr(error)
    (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('input_pair_hashes','graphs','health')},ensure_ascii=False))
    return 0 if report['status']=='PASS_REQUIRES_VISUAL_REVIEW' else 1


if __name__=='__main__':raise SystemExit(main())
