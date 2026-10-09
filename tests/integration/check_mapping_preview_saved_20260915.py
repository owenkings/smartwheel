#!/usr/bin/env python3
"""Offline preview TF/current-cloud and saved-map GUI checks. No hardware drivers."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from check_mapping_app_live import close_owned_window
from check_rviz_display import capture_owned_window


def main():
    from wc_runtime.cli import ROOT, target
    from wc_runtime.mapping_app import prepare_request
    from wc_runtime.mapping_controller import configuration, render_view
    from wc_runtime.map_viewer import stop_owned
    from wc_runtime.sensor_viewer import desktop_environment
    target(); os.chdir(ROOT)
    assert os.environ.get('ROS_DOMAIN_ID') == '84'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    out = ROOT/'reports/map_preview_save_20260915/offline_gui_final'
    out.mkdir(exist_ok=False)
    env = desktop_environment()
    env.update(ROS_DOMAIN_ID='84', ROS_LOCALHOST_ONLY='1', LIBGL_ALWAYS_SOFTWARE='1',
               GALLIUM_DRIVER='llvmpipe', __GLX_VENDOR_LIBRARY_NAME='mesa', PYTHONNOUSERSITE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import MarkerArray
    rclpy.init(args=[]); node = rclpy.create_node('preview_saved_offline_probe')
    report = {'status':'FAIL', 'validation_level':'SYNTHETIC_PREVIEW_AND_REAL_SAVED_MAP_GUI',
              'hardware_started':False, 'control_messages_published':False, 'checks':[]}
    children = []; streams = []; lock = None
    def spawn(command, name):
        stream = (out/(name+'.log')).open('xb'); streams.append(stream)
        child = subprocess.Popen(command, env=env, cwd=ROOT, stdin=subprocess.DEVNULL,
                                 stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child); return child
    try:
        lock = (ROOT/'.phase1_runtime/locks/domain-84-offline-review.lock').open('a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for _ in range(10): rclpy.spin_once(node, timeout_sec=.1)
        foreign = [name for name, _ in node.get_node_names_and_namespaces() if name != node.get_name()]
        assert not foreign, foreign
        request = prepare_request('right', project_root=ROOT)
        config = configuration(request)
        config['cameras_enabled'] = False; config['manual_controls'] = None
        (out/'runtime_config.json').write_text(json.dumps(config))
        view = out/'preview_only.rviz'; view.write_bytes(render_view(config))
        received = {'tf': [], 'markers': 0}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [node.create_subscription(TFMessage, '/wc_mapping/app/tf_static',
            lambda message: received['tf'].extend((t.header.frame_id,t.child_frame_id) for t in message.transforms), latched),
            node.create_subscription(MarkerArray, '/wc_mapping/app/view_markers',
                lambda message: received.__setitem__('markers',len(message.markers)), 1)]
        publisher = node.create_publisher(PointCloud2, '/wc_mapping/lidar_right/points_filtered',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        preview = spawn([sys.executable,'-s','-m','wc_runtime.mapping_preview','--session-root',str(out)], 'preview')
        component = [sys.executable,'-s','-m','wc_runtime.component','--parent',str(os.getpid()),'--']
        gui = spawn(component+[str(ROOT/'install/main/wc_bringup/lib/wc_bringup/mapping_rviz'),
            '-d',str(view),'--ros-args','-r','__node:=preview_validation',
            '-r','/tf:=/wc_mapping/app/tf','-r','/tf_static:=/wc_mapping/app/tf_static'], 'preview_rviz')
        deadline = time.monotonic()+40; ready = None
        while time.monotonic()<deadline:
            assert preview.poll() is None and gui.poll() is None, 'Preview child exited'
            cloud = create_cloud_xyz32(Header(stamp=node.get_clock().now().to_msg(),frame_id='lidar_right'),
                [(i*.06,j*.06,-.7) for i in range(-30,31) for j in range(-30,31)]+
                [(2.,j*.06,k*.06-.7) for j in range(-30,31) for k in range(35)])
            publisher.publish(cloud); rclpy.spin_once(node, timeout_sec=.1)
            if received['tf'] and received['markers'] and publisher.get_subscription_count():
                ready = ready or time.monotonic()
                if time.monotonic()-ready>5: break
        else: raise RuntimeError('Preview TF, markers or current-cloud RViz subscription missing')
        assert ('mapping_reference','lidar_right') in received['tf']
        cloud_subscribers = publisher.get_subscription_count()
        assert cloud_subscribers > 0
        shot = capture_owned_window(gui, out/'preview.png', env, activate_owned=True, qt_only=True)
        close_owned_window(gui,shot,env); assert gui.wait(timeout=20)==0
        stop_owned(preview)
        report['checks'].append({'name':'default_preview','static_tf':received['tf'],
            'markers':received['markers'],'current_cloud_subscribers_before_close':cloud_subscribers,'window':shot})
        for sub in subscriptions: node.destroy_subscription(sub)
        node.destroy_publisher(publisher)
        lock.close(); lock = None
        source = ROOT/'reports/maps/map_right_20260914T153949Z_680ffa'
        sources = [('session_directory',source),('database_copy_export',source/'slam/rtabmap.db'),
                   ('existing_export',ROOT/'reports/maps/map_right_20260914T133623Z_1ca313/export')]
        for name, path in sources:
            viewer = spawn([sys.executable,'-s',str(ROOT/'scripts/view_map'),str(path)],name)
            deadline = time.monotonic()+220; ready = None
            while time.monotonic()<deadline:
                assert viewer.poll() is None, 'Saved viewer exited: '+name
                rclpy.spin_once(node, timeout_sec=.1)
                topics = {suffix:[s.node_name for s in node.get_subscriptions_info_by_topic('/wc_mapping/offline/'+suffix)
                    if s.node_name=='saved_map_view'] for suffix in ('cloud_map','grid_map')}
                if all(topics.values()):
                    ready=ready or time.monotonic()
                    if time.monotonic()-ready>5: break
            else: raise RuntimeError('Saved map RViz subscriptions missing: '+name)
            shot = capture_owned_window(viewer,out/(name+'.png'),env,activate_owned=True,qt_only=True)
            close_owned_window(viewer,shot,env); assert viewer.wait(timeout=30)==0
            report['checks'].append({'name':name,'window':shot,'subscriptions':topics,'exit_code':viewer.returncode})
            for _ in range(15): rclpy.spin_once(node,timeout_sec=.1)
        report['status']='PASS_REQUIRES_VISUAL_REVIEW'
    except Exception as error:
        report['error']=str(error)
    finally:
        for child in reversed(children):
            try: stop_owned(child)
            except Exception as error: report.setdefault('cleanup_errors',[]).append(str(error)); report['status']='FAIL'
        for stream in streams: stream.close()
        if lock: lock.close()
        node.destroy_node(); rclpy.shutdown()
    (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False))
    return 0 if report['status']=='PASS_REQUIRES_VISUAL_REVIEW' else 1


if __name__=='__main__': raise SystemExit(main())
