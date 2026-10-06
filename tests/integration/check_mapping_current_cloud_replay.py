#!/usr/bin/env python3
"""Display two real recorded frames in an isolated ROS domain; no device/control nodes.

This is a frame display check, not a new SLAM run or a live motion acceptance.
Only original scan/map/TF messages are published, with their original stamps.
Recorded TF is preloaded around each chosen stamp solely for offline rendering.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time


def main():
    import fcntl
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import OccupancyGrid
    from wc_runtime.cli import ROOT, target
    from wc_runtime.mapping_visuals import build_scene, display_grid
    target()
    if os.environ.get('ROS_DOMAIN_ID') != '86' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Display-only test requires isolated localhost domain 86')
    root=ROOT; sid='map_right_20260914T113532Z_3d234e';source=root/'reports/maps'/sid
    out=root/'reports/map_user_motion_20260914/current_cloud_replay'
    out.mkdir(exist_ok=False)
    lock=(root/'.phase1_runtime/locks/domain-86-display.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    view=out/'display_only.rviz'  # Deliberately not a controllable session view.rviz.
    view.write_bytes((root/'config/rviz/mapping_live.rviz').read_bytes())
    spec=importlib.util.spec_from_file_location('owned_native_capture',root/'tests/integration/check_rviz_display.py')
    capture=importlib.util.module_from_spec(spec);spec.loader.exec_module(capture)
    env=capture.desktop_environment();env.update(LIBGL_ALWAYS_SOFTWARE='1',__GLX_VENDOR_LIBRARY_NAME='mesa')
    rclpy.init(args=[]);node=rclpy.create_node('mapping_recorded_frame_display')
    for _ in range(20):rclpy.spin_once(node,timeout_sec=.1)
    foreign=[n for n,ns in node.get_node_names_and_namespaces() if n!=node.get_name()]
    if foreign:raise RuntimeError('Display domain already occupied: '+repr(foreign))
    allowed={'/wc_mapping/app/scan_cloud','/wc_mapping/app/cloud_map','/wc_mapping/app/grid_map','/wc_mapping/app/tf'}
    bag=next((source/'bag').glob('*.db3'));db=sqlite3.connect('file:'+str(bag)+'?mode=ro',uri=True)
    start=db.execute('select min(timestamp) from messages').fetchone()[0]
    topics={i:(name,get_message(typ)) for i,name,typ in db.execute('select id,name,type from topics') if name in allowed}
    rows={name:[] for name in allowed};publishers={}
    for tid,(name,typ) in topics.items():
        for ts,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
            rows[name].append(((ts-start)/1e9,deserialize_message(data,typ),hashlib.sha256(data).hexdigest()))
        qos=QoSProfile(depth=100 if name.endswith('/tf') else 2,reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if name.endswith(('/cloud_map','/grid_map')) else DurabilityPolicy.VOLATILE)
        publishers[name]=node.create_publisher(typ,name,qos)
    db.close()
    clock_pub=node.create_publisher(Clock,'/clock',10)
    grid_pub=node.create_publisher(OccupancyGrid,'/wc_mapping/app/view_grid',QoSProfile(depth=1,
        reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    scene=build_scene(json.loads((source/'runtime_config.json').read_text()),json.loads((source/'prior/initialization.json').read_text()))
    command=[sys.executable,'-m','wc_runtime.component','--parent',str(os.getpid()),'--',
        str(root/'install/main/wc_bringup/lib/wc_bringup/mapping_rviz'),'-d',str(view),'--ros-args',
        '-r','__node:=mapping_recorded_frame_rviz','-r','/tf:=/wc_mapping/app/tf','-r','/tf_static:=/wc_mapping/app/tf_static',
        '-p','use_sim_time:=true']
    report={'status':'FAIL','validation_level':'REAL_BAG_DISPLAY_ONLY','source_session':sid,'source_bag':str(bag),
        'allowed_recorded_topics':sorted(allowed),'hardware_nodes_started':False,'control_topics_published':False,
        'freshness_or_mapping_accuracy_test':False,'view_sha256':hashlib.sha256(view.read_bytes()).hexdigest(),'frames':[]}
    with (out/'rviz.log').open('x') as log:
        child=subprocess.Popen(command,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                if child.poll() is not None:raise RuntimeError('Owned native RViz exited at startup')
                rclpy.spin_once(node,timeout_sec=.1)
                if all(publishers[n].get_subscription_count() for n in ('/wc_mapping/app/scan_cloud','/wc_mapping/app/cloud_map')):break
            else:raise RuntimeError('RViz did not subscribe to both current cloud and cumulative map')
            for target_s in (30.,40.):
                ts,cloud,cloud_hash=min(rows['/wc_mapping/app/scan_cloud'],key=lambda item:abs(item[0]-target_s))
                cm=max((x for x in rows['/wc_mapping/app/cloud_map'] if x[0]<=target_s),key=lambda x:x[0])
                gm=max((x for x in rows['/wc_mapping/app/grid_map'] if x[0]<=target_s),key=lambda x:x[0])
                clock_msg=Clock(clock=cloud.header.stamp)
                for t,transform,h in rows['/wc_mapping/app/tf']:
                    if target_s-6<=t<=target_s+1:
                        publishers['/wc_mapping/app/tf'].publish(transform)
                        rclpy.spin_once(node,timeout_sec=0)
                for _ in range(25):
                    clock_pub.publish(clock_msg)
                    publishers['/wc_mapping/app/cloud_map'].publish(cm[1])
                    publishers['/wc_mapping/app/grid_map'].publish(gm[1])
                    grid_pub.publish(display_grid(gm[1],scene['ground_z_m']))
                    publishers['/wc_mapping/app/scan_cloud'].publish(cloud)
                    rclpy.spin_once(node,timeout_sec=.1)
                shot=capture.capture_owned_window(child,out/('frame_'+str(int(target_s))+'.png'),env,activate_owned=True,qt_only=True)
                report['frames'].append({'source_bag_time_s':ts,'source_cloud_sha256':cloud_hash,'source_map_sha256':cm[2],
                    'stamp_ns':cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec,'window':shot})
            assert report['frames'][0]['source_cloud_sha256']!=report['frames'][1]['source_cloud_sha256']
            assert report['frames'][0]['source_map_sha256']==report['frames'][1]['source_map_sha256']
            report['status']='PASS_REQUIRES_VISUAL_REVIEW'
        except Exception as error:report['error']=str(error)
        finally:
            if child.poll() is None:child.send_signal(signal.SIGINT)
            child.wait(timeout=20)
            report['gui_exit_code']=child.returncode
            if child.returncode:report['status']='FAIL'
            node.destroy_node();rclpy.shutdown();lock.close()
    (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False))
    return 0 if report['status']=='PASS_REQUIRES_VISUAL_REVIEW' else 1


if __name__=='__main__':raise SystemExit(main())
