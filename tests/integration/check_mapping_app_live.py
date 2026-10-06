#!/usr/bin/env python3
"""Root-scheduled bounded LIVE_STATIC check; no vehicle-control commands."""
import argparse
import ctypes as C
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def close_owned_window(child, captured, env):
    from wc_runtime.component import process, descendants, belongs_to
    owner=process(child.pid)
    if owner is None or child.poll() is not None:raise RuntimeError('App owner gone')
    owned={p.pid:p for p in descendants(owner)}
    pid=captured['rviz_pid'];wid=captured['window_id']
    props=subprocess.check_output(['xprop','-id',wid,'_NET_WM_PID','WM_PROTOCOLS'],env=env,text=True,timeout=5)
    if pid not in owned or not belongs_to(owned[pid],owner) or ('= '+str(pid)) not in props or 'WM_DELETE_WINDOW' not in props:
        raise RuntimeError('Window close ownership/protocol check failed')
    class Data(C.Union):_fields_=[('b',C.c_char*20),('s',C.c_short*10),('l',C.c_long*5)]
    class Client(C.Structure):
        _fields_=[('type',C.c_int),('serial',C.c_ulong),('send_event',C.c_int),('display',C.c_void_p),
                  ('window',C.c_ulong),('message_type',C.c_ulong),('format',C.c_int),('data',Data)]
    class Event(C.Union):_fields_=[('client',Client),('pad',C.c_long*24)]
    lib=C.CDLL('libX11.so.6');lib.XOpenDisplay.argtypes=[C.c_char_p];lib.XOpenDisplay.restype=C.c_void_p
    lib.XInternAtom.argtypes=[C.c_void_p,C.c_char_p,C.c_int];lib.XInternAtom.restype=C.c_ulong
    lib.XSendEvent.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_long,C.POINTER(Event)]
    lib.XFlush.argtypes=[C.c_void_p];lib.XCloseDisplay.argtypes=[C.c_void_p]
    os.environ.update({k:env[k] for k in ('DISPLAY','XAUTHORITY')})
    display=lib.XOpenDisplay(env['DISPLAY'].encode())
    if not display:raise RuntimeError('Cannot open authorized desktop')
    try:
        event=Event();event.client.type=33;event.client.send_event=1;event.client.display=display
        event.client.window=int(wid,16);event.client.message_type=lib.XInternAtom(display,b'WM_PROTOCOLS',False)
        event.client.format=32;event.client.data.l[0]=lib.XInternAtom(display,b'WM_DELETE_WINDOW',False)
        if not lib.XSendEvent(display,int(wid,16),False,0,C.byref(event)):raise RuntimeError('WM_DELETE send failed')
        lib.XFlush(display)
    finally:lib.XCloseDisplay(display)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=['left','right','all'],required=True)
    parser.add_argument('--name',required=True);parser.add_argument('--stop',choices=['window','interrupt'],default='window')
    parser.add_argument('--stable-seconds',type=float,default=8.)
    parser.add_argument('--probe-native-resize',action='store_true',help='Diagnostic only: resize the owned captured RViz client by 16 px')
    parser.add_argument('--independent-qt-capture',action='store_true',help='Capture the verified RViz client independently before the GNOME screenshot')
    parser.add_argument('--qt-only-capture',action='store_true',help='Explicitly use verified native QScreen capture; never invoke GNOME or fall back')
    args=parser.parse_args()
    if not 0<=args.stable_seconds<=90:parser.error('stable seconds must be between 0 and 90')
    from wc_runtime.cli import ROOT,target
    from wc_runtime.sensor_viewer import desktop_environment
    target();os.chdir(ROOT)
    out=ROOT/'reports/mapping_app_live_review'/args.name;out.mkdir(parents=True,exist_ok=False)
    session=ROOT/'reports/maps'/args.name
    spec=importlib.util.spec_from_file_location('owned_capture',ROOT/'tests/integration/check_rviz_display.py')
    capture=importlib.util.module_from_spec(spec);spec.loader.exec_module(capture)
    env=desktop_environment();env.update(PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    # Exercise the unsourced public command. Every ROS child sources its own environment.
    for key in list(env):
        if key.startswith(('AMENT_','COLCON_','ROS_')) or key in ('PYTHONPATH','LD_LIBRARY_PATH','CMAKE_PREFIX_PATH'):
            env.pop(key,None)
    env['PATH']='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    duration=120+int(args.stable_seconds)
    command=['/usr/bin/python3',str(ROOT/'scripts/map'),args.mode,'--name',args.name,'--duration',str(duration)]
    report={'validation_level':'LIVE_STATIC','mode':args.mode,'command':command,'status':'FAIL','stop_method':args.stop,
            'capture_mode':'qt_only' if args.qt_only_capture else ('gnome_with_independent_qt' if args.independent_qt_capture else 'gnome')}
    with (out/'app.log').open('xb') as stream:
        child=subprocess.Popen(command,cwd=ROOT.parent,env=env,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            ready=None;deadline=time.monotonic()+85+args.stable_seconds
            while time.monotonic()<deadline:
                if child.poll() is not None:raise RuntimeError('App exited before live map readiness')
                health=session/'health/status.json'
                if health.exists():
                    status=json.loads(health.read_text())
                    if status.get('failure'):raise RuntimeError('Live health: '+str(status['failure']))
                    counts=status.get('counts',{})
                    if all(counts.get(k,0)>=1 for k in ('cloud_map','grid_map','prior_odom','odom','odom_info')) and counts.get('odom',0)>=30:
                        if ready is None:
                            ready=time.monotonic()
                            report['ready_capture_monotonic']=ready
                            try:
                                report['ready_window']=capture.capture_owned_window(child,out/'rviz_ready.png',env,activate_owned=True,independent_qt=args.independent_qt_capture,qt_only=args.qt_only_capture)
                                if args.probe_native_resize:
                                    from diagnose_owned_rviz_resize import probe
                                    report['resize_diagnostic']=probe(child,report['ready_window'],env)
                                    report['resized_window']=capture.capture_owned_window(child,out/'rviz_after_resize.png',env,activate_owned=True,qt_only=args.qt_only_capture)
                            except Exception as early_capture_error:report['ready_capture_error']=str(early_capture_error)
                        if time.monotonic()-ready>=args.stable_seconds:break
                time.sleep(.5)
            else:raise RuntimeError('No complete live map within readiness budget')
            report['health_before_close']=status
            report['capture_requested_monotonic']=time.monotonic()
            report['window']=capture.capture_owned_window(child,out/'rviz.png',env,activate_owned=True,independent_qt=args.independent_qt_capture,qt_only=args.qt_only_capture)
            if args.stop=='window':close_owned_window(child,report['window'],env)
            else:child.send_signal(signal.SIGINT)
            report['returncode']=child.wait(timeout=420)
            if child.returncode:raise RuntimeError('App failed; inspect app.log')
            app_result=json.loads((out/'app.log').read_text().splitlines()[-1])
            report['app_stop_reason']=app_result.get('stop_reason')
            expected='RVIZ_WINDOW_CLOSED' if args.stop=='window' else 'USER_STOP_REQUEST'
            if report['app_stop_reason']!=expected:raise RuntimeError('App stopped by '+str(report['app_stop_reason'])+' rather than requested '+expected)
            exported=json.loads((session/'export/result.json').read_text())
            report['export_status']=exported['status'];report['ply_vertices']=exported['ply_vertices'];report['map_2d']=exported['map_2d']
            report['status']='PASS'
        except Exception as error:
            report['error']=str(error)
            if child.poll() is None:
                try:report['failure_window']=capture.capture_owned_window(child,out/'rviz_failure.png',env,activate_owned=True,independent_qt=args.independent_qt_capture,qt_only=args.qt_only_capture)
                except Exception as capture_error:report['failure_capture_error']=str(capture_error)
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=420)
                except subprocess.TimeoutExpired:report['cleanup_error']='Owned app has not finished; retain owner watcher and bounded supervisor'
    report['app_tail']=(out/'app.log').read_text(errors='replace')[-6000:]
    (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False))
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
