"""Offline planar constraints and coherent two-arrival cloud/pose re-export."""
import copy
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def write_json(path, data):
    with Path(path).open('x',encoding='utf-8') as f:
        json.dump(data,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')


def correction_at(stamps, corrections, stamp):
    """Piecewise SE(2) correction field, shortest-yaw interpolation, end hold."""
    i=int(np.searchsorted(stamps,stamp,side='right'))
    if i==0:return corrections[0].copy()
    if i==len(stamps):return corrections[-1].copy()
    a,b=corrections[i-1],corrections[i]
    u=float(stamp-int(stamps[i-1]))/float(int(stamps[i])-int(stamps[i-1]))
    ya,yb=(math.atan2(c[1,0],c[0,0]) for c in (a,b))
    yaw=ya+u*math.atan2(math.sin(yb-ya),math.cos(yb-ya))
    out=np.eye(4);out[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()
    out[:3,3]=(1-u)*a[:3,3]+u*b[:3,3]
    return out


def redeskew_points(points, offsets, stamp, old_pose, new_pose, stamps, corrections):
    """D = inv(new reference pose) C(source time) old reference pose.

    Input is already compensated by inv(T_old_ref) T_old_source. This is
    equivalent to reprojecting each raw side through the final pose provider.
    Rows/fields/source identity are never reordered or dropped here.
    """
    result=np.array(points,copy=True);finite=np.isfinite(points).all(axis=1)
    maximum=0.
    for offset in np.unique(offsets):
        mask=finite & (offsets==offset)
        if not np.any(mask):continue
        D=np.linalg.inv(new_pose)@correction_at(stamps,corrections,int(stamp)+int(offset))@old_pose
        result[mask]=points[mask]@D[:3,:3].T+D[:3,3]
        maximum=max(maximum,float(np.linalg.norm(result[mask]-points[mask],axis=1).max()))
    return result,maximum


def geometry_cell(output, source_cell, *, name='geometric_motion', policy=None):
    from rclpy.serialization import deserialize_message,serialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from wc_sensors.pointcloud import decode_pointcloud2,validate_pointcloud2
    from .mapping_prior import _field_values
    from .compare_native import load_index
    from .offline_planar_registration import TrajectoryRefiner
    from .mapping_compare import write_trajectory
    source=Path(source_cell);directory=Path(output)/name;directory.mkdir()
    frontend=directory/'frontend';frontend.mkdir()
    rows=load_index(source);stamps=np.array(sorted(rows),dtype=np.int64)
    engine=TrajectoryRefiner(policy=policy);poses=[];corrections=[];results=[]
    def cloud(row):
        path=source/'frontend'/row['cloud_cdr_file']
        if digest(path)!=row['cloud_cdr_sha256']:raise ValueError('motion cloud hash mismatch')
        return deserialize_message(path.read_bytes(),PointCloud2)
    with (directory/'registration.jsonl').open('x',encoding='utf-8') as f:
        for i,stamp in enumerate(stamps):
            row=rows[int(stamp)];old=np.asarray(row['T_prior_reference'])
            result=engine.update(int(stamp),decode_pointcloud2(cloud(row)),old)
            pose=np.asarray(result['T_world_scan']);poses.append(pose);corrections.append(pose@np.linalg.inv(old));results.append(result)
            f.write(json.dumps(result,ensure_ascii=False,allow_nan=False)+'\n')
            if i%100==0:print(json.dumps({'stage':'GEOMETRY','frames':i+1,'total':len(stamps)}),flush=True)
    config=json.loads((source/'runtime_config.json').read_text())
    config['offline_geometry']={'policy':engine.summary().get('policy'),
        'method':'planar point-to-plane with bounded submap and quality rejection',
        'registration_code_sha256':digest(Path(__file__).with_name('offline_planar_registration.py')),
        'pose_provider':'T_final(t)=C(t)*T_calibrated_ekf(t)',
        'compensation':'same final pose provider for both lidar arrivals; piecewise linear XY/shortest-yaw correction',
        'outside_trajectory':'hold first/last correction; retain calibrated EKF relative motion',
        'geometric_covariance':'unknown; rotated EKF model plus explicit uncertainty budget, not posterior confidence',
        'pose_uncertainty_budget_diagonal':[.25,.25,0.,0.,0.,math.radians(15.)**2],
        'twist_uncertainty_budget_diagonal':[1.,1.,1.,1.,1.,1.],
        'loop_closure':False,'forced_rectangle':False}
    write_json(directory/'runtime_config.json',config)
    frames={};maxdeskew=0.;lastpose=None;laststamp=None
    with (frontend/'index.jsonl').open('x',encoding='utf-8') as index:
        for i,stamp in enumerate(stamps):
            stamp=int(stamp);row=copy.deepcopy(rows[stamp]);old=np.asarray(row['T_prior_reference']);pose=poses[i]
            msg=cloud(row);layout=validate_pointcloud2(msg);xyz=decode_pointcloud2(msg)
            offsets=np.rint(_field_values(msg,layout,'time_offset_s').reshape(-1)*1e9).astype(np.int64)
            corrected,delta=redeskew_points(xyz,offsets,stamp,old,pose,stamps,corrections);maxdeskew=max(maxdeskew,delta)
            storage=bytearray(msg.data);finite=np.isfinite(xyz).all(axis=1)
            for k,field in enumerate(('x','y','z')):
                view=_field_values(msg,layout,field,storage)
                values=view.copy().reshape(-1)
                if not np.isfinite(corrected[finite,k]).all() or np.any(np.abs(corrected[finite,k])>np.finfo(view.dtype).max):
                    raise ValueError('recompensated coordinate exceeds finite output range')
                values[finite]=corrected[finite,k];view[:]=values.reshape(view.shape)
            msg.data=bytes(storage)
            payload=bytes(serialize_message(msg));(frontend/row['cloud_cdr_file']).write_bytes(payload)
            row['cloud_cdr_sha256']=hashlib.sha256(payload).hexdigest();row['cloud_data_sha256']=hashlib.sha256(msg.data).hexdigest()
            op=source/'frontend'/row['odom_cdr_file']
            if digest(op)!=row['odom_cdr_sha256']:raise ValueError('motion odometry hash mismatch')
            odom=deserialize_message(op.read_bytes(),Odometry)
            p=odom.pose.pose.position;p.x,p.y,p.z=map(float,pose[:3,3])
            q=odom.pose.pose.orientation;q.x,q.y,q.z,q.w=map(float,Rotation.from_matrix(pose[:3,:3]).as_quat())
            J=np.eye(6);J[:3,:3]=corrections[i][:3,:3];J[3:,3:]=corrections[i][:3,:3]
            cov=J@np.array(odom.pose.covariance).reshape(6,6)@J.T+np.diag(config['offline_geometry']['pose_uncertainty_budget_diagonal'])
            odom.pose.covariance=cov.reshape(-1).tolist()
            v=np.zeros(3);w=np.zeros(3)
            if lastpose is not None:
                dt=(stamp-laststamp)*1e-9
                v=pose[:3,:3].T@(pose[:3,3]-lastpose[:3,3])/dt
                w[2]=math.atan2((lastpose[:3,:3].T@pose[:3,:3])[1,0],(lastpose[:3,:3].T@pose[:3,:3])[0,0])/dt
            odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=map(float,v)
            odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z=map(float,w)
            odom.twist.covariance=(np.array(odom.twist.covariance).reshape(6,6)+np.eye(6)).reshape(-1).tolist()
            payload=bytes(serialize_message(odom));(frontend/row['odom_cdr_file']).write_bytes(payload)
            row.update(odom_cdr_sha256=hashlib.sha256(payload).hexdigest(),T_prior_reference=pose.tolist(),
                pose_covariance=list(odom.pose.covariance),twist_covariance=list(odom.twist.covariance),
                linear_velocity_reference_m_s=v.tolist(),angular_velocity_reference_rad_s=w.tolist(),
                input_pair_index_file='../../'+source.name+'/input/pairs.jsonl',
                calibrated_motion_cloud_sha256=rows[stamp]['cloud_cdr_sha256'],
                calibrated_motion_odom_sha256=rows[stamp]['odom_cdr_sha256'])
            prior_report=row.pop('sample_report');row['calibrated_ekf_report']=prior_report
            row['sample_report']={'T_prior_reference':pose.tolist(),'pose_provider':config['offline_geometry']['pose_provider'],
                'geometry_status':results[i]['status'],'segment_id':results[i]['segment_id'],
                'source_offsets_ns':sorted(map(int,np.unique(offsets))),
                'redeskew_max_point_displacement_m':delta,
                'covariance_status':'UNKNOWN_GEOMETRY_UNCERTAINTY_WITH_EXPLICIT_UNVALIDATED_BUDGET',
                'compensation_basis':'arrival-time approximation, no per-point exposure timing',
                'source_pose_samples':{str(offset):(correction_at(stamps,corrections,stamp+int(offset))@np.array(prior_report['source_pose_samples'][str(offset)])).tolist() for offset in np.unique(offsets)}}
            index.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');frames[stamp]=row
            lastpose,laststamp=pose,stamp
    summary=engine.summary();summary.update(status='GEOMETRY_REPLAY_COMPLETE_REQUIRES_REVIEW',frames=len(frames),
        redeskew_max_point_displacement_m=maxdeskew,registration_input='calibrated EKF arrival-compensated clouds',
        final_clouds='recompensated with final geometric pose correction field',
        uncertainty='No independent ground truth; geometric matching can be wrong despite gates',
        no_forced_rectangle=True,loop_closure=False)
    write_json(directory/'summary.json',summary)
    cell={'directory':directory,'frames':frames,'summary':summary};write_trajectory(cell)
    return cell
