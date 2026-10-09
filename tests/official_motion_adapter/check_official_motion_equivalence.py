#!/usr/bin/env python3
"""Corrected wheel full covariance: official node vs native worker, hardware-free.

Uses explicit zero TF lever arm, identical covariance, source stamps, 2D
constraints and a controlled /clock. Each source correction is acknowledged
before the next; startup, turning, conflicts and rejection are all compared.
Does not test missing-data freeze (an explicit separate provider policy).
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain',type=int,default=90)
    p.add_argument('--steps',type=int,default=40)
    p.add_argument('--tolerance',type=float,default=2e-7)
    args=p.parse_args(argv)
    if not 1<=args.domain<=232 or args.domain==83 or args.steps<5 or args.steps>1000 or args.output.exists():
        raise ValueError('new output, isolated domain and bounded steps required')
    root=Path(__file__).resolve().parents[2]; sys.path.insert(0,str(root/'src'))
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    import fcntl
    import yaml
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from rcl_interfaces.srv import GetParameters
    from wc_runtime.robot_localization_provider import RobotLocalizationEKF,native_parameters
    from wc_runtime.mapping_planar import validate_planar_config
    from wc_runtime.offline_motion_adapter import corrected_wheel_observation
    args.output.mkdir(parents=True)
    config=validate_planar_config({}); x,pdiag,qdiag=native_parameters(config)
    mask=[False]*15; mask[6]=mask[11]=True
    imu_mask=[False]*15; imu_mask[11]=True
    params={'use_sim_time':True,'frequency':100.,'sensor_timeout':1000000.,
        'two_d_mode':True,'publish_tf':False,'print_diagnostics':False,
        'predict_to_current_time':False,'permit_corrected_publication':True,
        'smooth_lagged_data':False,'dynamic_process_noise_covariance':False,
        'map_frame':'audit_map','odom_frame':'audit_odom','world_frame':'audit_odom',
        'base_link_frame':'audit_axle','base_link_output_frame':'audit_axle',
        'initial_state':x.tolist(),'initial_estimate_covariance':np.diag(pdiag).reshape(-1).tolist(),
        'process_noise_covariance':np.diag(qdiag).reshape(-1).tolist(),
        'twist0':'/audit/wheel','twist0_config':mask,'twist0_queue_size':100,
        'twist0_rejection_threshold':config['innovation_gate_sigma'],
        'imu0':'/audit/imu','imu0_config':imu_mask,'imu0_queue_size':100,
        'imu0_differential':False,'imu0_relative':False,
        'imu0_twist_rejection_threshold':config['innovation_gate_sigma']}
    parameters=args.output/'official_params.yaml'
    parameters.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=False),encoding='utf-8')
    executable=Path(get_package_prefix('robot_localization'))/'lib/robot_localization/ekf_node'
    command=[str(executable),'--ros-args','--params-file',str(parameters),'-r','__node:=wc_rl_official_audit',
             '-r','odometry/filtered:=/audit/filtered']
    report={'status':'FAIL','hardware_started':False,'domain':args.domain,'official_command':command,
        'parameters':params,'steps':args.steps,'tolerance':args.tolerance,'comparisons':[],
        'scope':'initialized continuous valid input and explicit 2D wrapper; not physical accuracy or missing-data freeze',
        'wheel_correction':{'scale':.9012,'speed_coefficient':-.013,'covariance':'full J R J^T'},
        'wrapper_reference':'https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/src/ros_filter.cpp'}
    lock_dir=root/'.phase1_runtime/locks'; lock_dir.mkdir(parents=True,exist_ok=True)
    with (lock_dir/('domain-'+str(args.domain)+'-geometry-review.lock')).open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        rclpy.init(args=[]); node=rclpy.create_node('wc_rl_parity_controller')
        child=None; log=None; received=[]; clock_ns=100_000_000_000
        try:
            until=time.monotonic()+2.
            while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.05)
            foreign=[(name,ns) for name,ns in node.get_node_names_and_namespaces() if name!=node.get_name()]
            if foreign: raise RuntimeError('isolated domain occupied: '+repr(foreign))
            wheel_pub=node.create_publisher(TwistWithCovarianceStamped,'/audit/wheel',100)
            imu_pub=node.create_publisher(Imu,'/audit/imu',100)
            clock_pub=node.create_publisher(Clock,'/clock',10)
            sub=node.create_subscription(Odometry,'/audit/filtered',received.append,100)
            log=(args.output/'official.log').open('xb')
            child=subprocess.Popen(command,cwd=root,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            def pump(predicate,timeout,reason):
                nonlocal clock_ns
                deadline=time.monotonic()+timeout
                while not predicate():
                    if child.poll() is not None: raise RuntimeError('official node exited: '+str(child.poll()))
                    if time.monotonic()>deadline: raise TimeoutError(reason)
                    clock_ns+=10_000_000
                    clock_pub.publish(Clock(clock=Time(sec=clock_ns//10**9,nanosec=clock_ns%10**9)))
                    rclpy.spin_once(node,timeout_sec=.02)
            pump(lambda:wheel_pub.get_subscription_count() and imu_pub.get_subscription_count() and clock_pub.get_subscription_count(),
                 30.,'official subscriptions/clock unavailable')
            client=node.create_client(GetParameters,'/wc_rl_official_audit/get_parameters')
            pump(client.service_is_ready,10.,'official parameter service not ready')
            request=GetParameters.Request(names=['imu0_twist_rejection_threshold','twist0_rejection_threshold','frequency','two_d_mode'])
            future=client.call_async(request)
            pump(future.done,10.,'official parameter handshake failed')
            response=future.result()
            if response is None or len(response.values)!=4 or response.values[0].type!=3 or \
                    response.values[0].double_value!=config['innovation_gate_sigma'] or \
                    response.values[1].double_value!=config['innovation_gate_sigma']:
                raise RuntimeError('official candidate gates not loaded')
            report['official_parameter_handshake']=[{'type':v.type,'double_value':v.double_value,'bool_value':v.bool_value} for v in response.values]
            # Discovery alone can precede constructor completion. Service ack
            # confirms the executor is live; allow the controlled clock to settle.
            until=time.monotonic()+.5
            while time.monotonic()<until:
                clock_ns+=10_000_000
                clock_pub.publish(Clock(clock=Time(sec=clock_ns//10**9,nanosec=clock_ns%10**9)))
                rclpy.spin_once(node,timeout_sec=.02)
            model=RobotLocalizationEKF(config)
            def compare(source_stamp,source,sequence):
                nonlocal clock_ns
                expected,pcov=model.state(); before=len(received)
                clock_ns=max(clock_ns+20_000_000,source_stamp+20_000_000)
                pump(lambda:any(m.header.stamp.sec*10**9+m.header.stamp.nanosec==source_stamp for m in received[before:]),
                     5.,'official original-stamp output unavailable after '+source)
                # Allow queued source callback and corrected same-time publication
                # to complete. Wait for the newest covariance to stop changing.
                until=time.monotonic()+.10
                while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.01)
                message=[m for m in received if m.header.stamp.sec*10**9+m.header.stamp.nanosec==source_stamp][-1]
                pp,qq=message.pose.pose.position,message.pose.pose.orientation
                ll,aa=message.twist.twist.linear,message.twist.twist.angular
                observed=np.array([pp.x,pp.y,pp.z,*Rotation.from_quat([qq.x,qq.y,qq.z,qq.w]).as_euler('xyz'),
                                   ll.x,ll.y,ll.z,aa.x,aa.y,aa.z])
                state_error=float(np.max(np.abs(observed-expected[:12])))
                pose_error=float(np.max(np.abs(np.asarray(message.pose.covariance).reshape(6,6)-pcov[:6,:6])))
                twist_error=float(np.max(np.abs(np.asarray(message.twist.covariance).reshape(6,6)-pcov[6:12,6:12])))
                row={'source':source,'sequence':sequence,'source_stamp_ns':source_stamp,'max_state_error':state_error,
                     'max_pose_covariance_error':pose_error,'max_twist_covariance_error':twist_error,
                     'native_last_update':model.last_update.get('wheel' if source=='wheel' else 'gyro')}
                report['comparisons'].append(row)
                if max(state_error,pose_error,twist_error)>args.tolerance:
                    raise AssertionError('official/library mismatch: '+json.dumps(row))
            for i in range(args.steps):
                source_stamp=100_000_000_000+i*100_000_000
                if i: model.predict(.1)
                velocity=.2+.03*np.sin(i*.2); yaw=.03*np.cos(i*.1)
                if i==args.steps//2: velocity,yaw=20.,4. # Actual rejection challenge.
                corrected,covariance=corrected_wheel_observation(velocity,yaw,config,.9012,-.013)
                velocity,yaw=corrected
                wheel=TwistWithCovarianceStamped(); wheel.header.stamp=Time(sec=source_stamp//10**9,nanosec=source_stamp%10**9)
                wheel.header.frame_id='audit_axle'; wheel.twist.twist.linear.x=float(velocity); wheel.twist.twist.angular.z=float(yaw)
                wheel.twist.covariance[0]=float(covariance[0,0]); wheel.twist.covariance[35]=float(covariance[1,1])
                wheel.twist.covariance[5]=wheel.twist.covariance[30]=float(covariance[0,1])
                for axis in (7,14,21,28): wheel.twist.covariance[axis]=1e6
                model.wheel_covariance(i,float(velocity),float(yaw),covariance)
                wheel_pub.publish(wheel); compare(source_stamp,'wheel',i)
                gyro=float(.03*np.cos(i*.1)+.003)
                if i==args.steps//2+1: gyro=5.
                imu=Imu(); imu.header.stamp=wheel.header.stamp; imu.header.frame_id='audit_axle'
                imu.orientation_covariance[0]=-1.; imu.linear_acceleration_covariance[0]=-1.
                imu.angular_velocity.z=gyro; imu.angular_velocity_covariance[8]=config['gyro_w_variance']
                imu.angular_velocity_covariance[0]=imu.angular_velocity_covariance[4]=1e6
                model.gyro(i,gyro); imu_pub.publish(imu); compare(source_stamp,'gyro',i)
            report['final_native_report']=model.report(); report['status']='OFFICIAL_EKF_NODE_EQUIVALENCE_PASS'
        except Exception as error:
            report['error']=type(error).__name__+': '+str(error)
            report['received_output_count']=len(received)
            report['last_controlled_clock_ns']=clock_ns
            report['received_stamp_ns']=[m.header.stamp.sec*10**9+m.header.stamp.nanosec for m in received[-20:]]
        finally:
            if child is not None:
                if child.poll() is None: child.send_signal(signal.SIGINT)
                try: child.wait(timeout=10.)
                except subprocess.TimeoutExpired: child.kill(); child.wait(timeout=5.)
                report['official_exit_code']=child.returncode
                if child.returncode!=0: report['status']='FAIL'
            if log: log.close()
            node.destroy_node(); rclpy.shutdown()
    (args.output/'result.json').write_text(json.dumps(report,allow_nan=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'status':report['status'],'output':str(args.output),'comparisons':len(report['comparisons'])}))
    return 0 if report['status']=='OFFICIAL_EKF_NODE_EQUIVALENCE_PASS' else 1

if __name__=='__main__': raise SystemExit(main())
