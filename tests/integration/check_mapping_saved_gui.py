#!/usr/bin/env python3
"""Reload saved real PLY/PGM into native RViz and test its owned window close."""
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
    from wc_runtime.cli import ROOT,target
    from wc_runtime.sensor_viewer import desktop_environment
    target();os.chdir(ROOT)
    out=ROOT/'reports/mapping_app_deploy_20260914/saved_gui_review';out.mkdir(exist_ok=False)
    lock=(ROOT/'.phase1_runtime/locks/domain-84-offline-review.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    env=desktop_environment();env.update(ROS_DOMAIN_ID='84',ROS_LOCALHOST_ONLY='1',PYTHONNOUSERSITE='1')
    os.environ.update(ROS_DOMAIN_ID='84',ROS_LOCALHOST_ONLY='1')
    import rclpy
    rclpy.init(args=[]);node=rclpy.create_node('saved_map_gui_probe')
    result={'test':'NATIVE_RVIZ_RELOAD_REAL_SAVED_MAP_AND_CLOSE','status':'FAIL','geometry_accuracy_validated':False}
    gui=publisher=None;logs=[]
    try:
        log=(out/'publisher.log').open('xb');logs.append(log)
        publisher=subprocess.Popen([sys.executable,'-s','tests/integration/check_mapping_app_export.py',
            '--session-root','reports/maps/map_app_right_20260914_01','--output',str(out/'audit.json'),
            '--publish','--duration','90'],env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+15
        while not (out/'audit.rviz').exists():
            if publisher.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Saved map publisher failed')
            time.sleep(.1)
        log=(out/'rviz.log').open('xb');logs.append(log)
        gui=subprocess.Popen([sys.executable,'-s','-m','wc_runtime.component','--parent',str(os.getpid()),'--',
            str(ROOT/'install/main/wc_bringup/lib/wc_bringup/mapping_rviz'),'-d',str(out/'audit.rviz'),
            '--ros-args','-r','__node:=mapping_saved_validation'],env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+25;ready=None
        while time.monotonic()<deadline:
            if gui.poll() is not None:raise RuntimeError('Native RViz exited early')
            rclpy.spin_once(node,timeout_sec=.1)
            subscribers={topic:[v.node_name for v in node.get_subscriptions_info_by_topic('/wc_mapping/offline/'+topic)
                if v.node_name=='mapping_saved_validation'] for topic in ('cloud_map','grid_map')}
            if all(subscribers.values()):
                ready=ready or time.monotonic()
                if time.monotonic()-ready>=5:break
        else:raise RuntimeError('Native RViz map subscriptions not confirmed')
        result['subscriptions']=subscribers
        result['window']=capture_owned_window(gui,out/'saved_maps.png',env)
        close_owned_window(gui,result['window'],env)
        result['close_exit_code']=gui.wait(timeout=15)
        if gui.returncode:raise RuntimeError('Native RViz window did not close normally')
        result['status']='PASS';result['close_method']='WM_DELETE_WINDOW_ONLY'
    except Exception as error:result['error']=str(error)
    finally:
        for child in (gui,publisher):
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=18)
                except subprocess.TimeoutExpired:result['cleanup_error']='Owned child did not stop';result['status']='FAIL'
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown();lock.close()
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
