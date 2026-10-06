#!/usr/bin/env python3
"""Recorded-pose time-offset sensitivity, without ICP fitting or hardware access.

The same recorded scan pairs and points are compared at every offset. This is
an effective-delay sensitivity experiment, not clock calibration or truth.
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

from analyze_mapping_geometry_20260915 import bag_topics, messages, stamp


def downsample(points, voxel=.06, maximum=1800):
    points=np.asarray(points,dtype=float)
    points=points[np.isfinite(points).all(axis=1)]
    distance=np.linalg.norm(points,axis=1)
    points=points[(distance>.8)&(distance<6.)]
    _,indices=np.unique(np.floor(points/voxel).astype(np.int64),axis=0,return_index=True)
    points=points[np.sort(indices)]
    if len(points)>maximum: points=points[np.linspace(0,len(points)-1,maximum,dtype=int)]
    return points


def normal_model(points):
    tree=cKDTree(points)
    _,neighbors=tree.query(points,k=min(20,len(points)),workers=1)
    neighborhoods=points[neighbors]
    centered=neighborhoods-neighborhoods.mean(axis=1,keepdims=True)
    covariance=np.einsum('nki,nkj->nij',centered,centered)/neighbors.shape[1]
    values,vectors=np.linalg.eigh(covariance)
    planar=(values[:,0]/np.maximum(values.sum(axis=1),1e-15)<.03)&(values[:,1]>.0001)
    return tree,vectors[:,:,0],planar


def score_pair(source,target,tree,normals,planar,relative,maximum=.25):
    transformed=source@relative[:3,:3].T+relative[:3,3]
    distances,nearest=tree.query(transformed,k=1,workers=1)
    good=(distances<maximum)&planar[nearest]
    errors=np.abs(np.einsum('ij,ij->i',transformed-target[nearest],normals[nearest]))
    # A rejected point keeps a penalty; dropping more points cannot improve cost.
    penalized=np.where(good,np.minimum(errors,maximum),maximum)
    accepted=errors[good]
    return {'penalized_rmse_m':float(np.sqrt(np.mean(penalized**2))),
            'accepted_ratio':float(good.mean()),'accepted_count':int(good.sum()),
            'inlier_p50_m':float(np.median(accepted)) if len(accepted) else None,
            'inlier_p95_m':float(np.quantile(accepted,.95)) if len(accepted) else None}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root',type=Path,required=True)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--max-pairs',type=int,default=100)
    args=parser.parse_args()
    from wc_runtime.cli import ROOT,target
    from wc_runtime.prepare_picker_input import project_path
    from wc_sensors.pointcloud import decode_pointcloud2
    target();source=project_path(ROOT,args.session_root);out=project_path(ROOT,args.output_root)
    out.mkdir(parents=True,exist_ok=False)
    started=time.monotonic()
    prior_file=source/'prior/guesses.jsonl'
    prior_bytes=prior_file.read_bytes()
    rows=[json.loads(line) for line in prior_bytes.splitlines() if line]
    origin=int(rows[0]['stamp_ns'])
    pose_times=np.array([(int(row['stamp_ns'])-origin)*1e-9 for row in rows])
    poses=np.array([row['T_prior_reference'] for row in rows])
    interpolation=Slerp(pose_times,Rotation.from_matrix(poses[:,:3,:3]))
    def pose_at(t):
        matrix=np.eye(4);matrix[:3,:3]=interpolation(float(t)).as_matrix()
        matrix[:3,3]=[np.interp(t,pose_times,poses[:,axis,3]) for axis in range(3)]
        return matrix
    scans=[]
    for _,message,digest in messages(bag_topics(source),'/wc_mapping/app/scan_cloud'):
        t=(stamp(message.header.stamp)-origin)*1e-9
        if pose_times[0]+.31<t<pose_times[-1]-.31:
            points=downsample(decode_pointcloud2(message))
            if len(points)>=300: scans.append({'time_s':t,'points':points,'cdr_sha256':digest})
    candidates=[]
    for index in range(len(scans)-1):
        a,b=scans[index:index+2];dt=b['time_s']-a['time_s']
        if not .1<dt<.6:continue
        covered=pose_times[(pose_times>=a['time_s']-.4)&(pose_times<=b['time_s']+.4)]
        if len(covered)<3 or np.diff(covered).max()>.65:continue
        relative=np.linalg.inv(pose_at(a['time_s']))@pose_at(b['time_s'])
        angular=Rotation.from_matrix(relative[:3,:3]).magnitude()/dt
        linear=np.linalg.norm(relative[:3,3])/dt
        category='turn' if angular>.05 else 'translation' if linear>.04 else 'stationary_candidate'
        candidates.append((index,category,float(angular),float(linear)))
    # Fixed temporal sample includes controls; all offsets use these exact pairs.
    selection=np.unique(np.linspace(0,len(candidates)-1,min(args.max_pairs,len(candidates)),dtype=int))
    chosen=[candidates[index] for index in selection]
    offsets=np.round(np.arange(-.3,.3001,.01),6)
    results=[]
    for number,(index,category,angular,linear) in enumerate(chosen):
        a,b=scans[index:index+2];tree,normals,planar=normal_model(a['points'])
        measurements=[]
        for offset in offsets:
            relative=np.linalg.inv(pose_at(a['time_s']+offset))@pose_at(b['time_s']+offset)
            measurements.append(score_pair(b['points'],a['points'],tree,normals,planar,relative))
        results.append({'time_s':a['time_s'],'next_time_s':b['time_s'],'category':category,
            'angular_rate_rad_s':angular,'linear_rate_m_s':linear,'source_points':len(b['points']),
            'target_points':len(a['points']),'cdr_sha256':[a['cdr_sha256'],b['cdr_sha256']],
            'measurements':measurements})
        if number%20==0:print(json.dumps({'status':'TIME_OFFSET_PAIRS','done':number+1,'total':len(chosen)}),flush=True)
    split=float(np.median([row['time_s'] for row in results]))
    summaries={}
    for category in ['all','turn','translation','stationary_candidate']:
        for partition in ['all','first_half','second_half']:
            selected=[row for row in results if (category=='all' or row['category']==category) and
                (partition=='all' or (row['time_s']<=split)==(partition=='first_half'))]
            if not selected: continue
            costs=np.array([[m['penalized_rmse_m'] for m in row['measurements']] for row in selected])
            curve=np.median(costs,axis=0);best=int(np.argmin(curve));zero=int(np.argmin(abs(offsets)))
            summaries[category+'/'+partition]={'pairs':len(selected),'median_cost_curve_m':curve.tolist(),
                'best_offset_s':float(offsets[best]),'baseline_cost_m':float(curve[zero]),
                'best_cost_m':float(curve[best]),'relative_improvement':float(1-curve[best]/curve[zero])}
    held_out={}
    for category in ['all','turn','translation']:
        train=summaries.get(category+'/first_half');test=summaries.get(category+'/second_half')
        if train and test:
            index=int(np.argmin(abs(offsets-train['best_offset_s'])))
            corrected=test['median_cost_curve_m'][index]
            held_out[category]={'trained_offset_s':train['best_offset_s'],'validation_pairs':test['pairs'],
                'baseline_cost_m':test['baseline_cost_m'],'trained_offset_cost_m':corrected,
                'relative_improvement':1-corrected/test['baseline_cost_m']}
    report={'status':'COMPLETED_SENSITIVITY_ONLY','hardware_accessed':False,'ros_nodes_started':False,
        'source_session':source.name,'prior_sha256':hashlib.sha256(prior_bytes).hexdigest(),
        'cloud_topic':'/wc_mapping/app/scan_cloud','cloud_representation':'host_filtered_fixed_reference',
        'offset_definition':'query recorded pose at scan_stamp + offset for both ends; positive queries later pose',
        'pose_model':'recorded poses, piecewise translation interpolation and rotational Slerp; no raw IMU re-integration',
        'parameters':{'voxel_m':.06,'max_points':1800,'normal_neighbors':20,'normal_curvature_max':.03,
            'correspondence_distance_m':.25,'max_pair_interval_s':.6},
        'offsets_s':offsets.tolist(),'split_time_s':split,'summaries':summaries,'held_out':held_out,
        'pairs':results,'elapsed_s':time.monotonic()-started,
        'limitations':['No external time or pose ground truth; this does not identify sensor clock offset.',
            'Constant velocity intervals weakly constrain a common time shift; scene variation and interpolation confound the score.',
            'Same points and rejection penalty at all offsets; no ICP transform is fitted to hide pose error.',
            'Time offsets do not correct raw geometry, gyro bias or motion within temporal HDR stages.']}
    (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    np.savez_compressed(out/'curves.npz',offsets_s=offsets,**{key.replace('/','_'):np.array(value['median_cost_curve_m']) for key,value in summaries.items()})
    assert hashlib.sha256(prior_file.read_bytes()).hexdigest()==report['prior_sha256']
    print(json.dumps({'status':report['status'],'pairs':len(results),'held_out':held_out,'elapsed_s':report['elapsed_s']}),flush=True)


if __name__=='__main__': main()
