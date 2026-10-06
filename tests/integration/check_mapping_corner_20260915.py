#!/usr/bin/env python3
"""Offline same-pixel corner inspection; never starts ROS or opens devices."""
import argparse
from contextlib import closing
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import sys

for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'
import numpy as np


def sha(data):
    return hashlib.sha256(data).hexdigest()


def extract(root, output):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from wc_sensors.pointcloud import decode_pointcloud2
    from check_mapping_filter_ab_20260915 import verify_pair
    assert not output.exists(), 'OUTPUT_MUST_BE_NEW'
    output.mkdir(parents=True)
    prior_report = json.loads((root/'reports/map_geometry_fix_20260915/filter_ab_v2.json').read_text())
    source = Path(prior_report['source_root'])
    initial = json.loads((source/'prior/initialization.json').read_text())
    rotation = np.asarray(initial['R_reference_base'])
    entries = []
    for index in (1, 2, 3):
        row = prior_report['frames'][index]
        messages, points = {}, {}
        for rep in ('raw', 'filtered'):
            record = row['cdr_records'][rep]
            bag = source/'bag'/record['bag']
            before = bag.stat()
            with closing(sqlite3.connect(bag.as_uri()+'?mode=ro', uri=True)) as conn:
                conn.execute('PRAGMA query_only=ON')
                data = bytes(conn.execute('SELECT data FROM messages WHERE id=?', (record['message_id'],)).fetchone()[0])
            assert sha(data) == record['cdr_sha256']
            messages[rep] = deserialize_message(data, get_message('wc_interfaces/msg/SourceFrame'))
            points[rep] = decode_pointcloud2(messages[rep].cloud) @ rotation.T
            after = bag.stat()
            assert (before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns)
        binding = verify_pair(messages['raw'], messages['filtered'], prior_report['session_id'])
        name = 'right_20260914_seq'+str(messages['raw'].frame_sequence)
        np.savez_compressed(output/(name+'.npz'), **points)
        entries.append({'name': name, 'side': 'right', 'date': '2026-09-14 23:39 CST',
            'time_s': row['time_s'], 'motion': row['motion'], 'source': str(source),
            'cdr_records': row['cdr_records'], 'binding': binding,
            'rotation': rotation.tolist(), 'coordinate_system': 'original fixed mapping reference; common rigid rotation only'})
    study = root/'reports/lidar_stability/compare02_20260913T095313Z'
    npz = study/'left_single/left.npz'
    expected = '10ebb9eafcb193526742067b3f91102c18e7663657528f01751a7e573f2887d7'
    assert sha(npz.read_bytes()) == expected
    metadata_path = study/'left_single/left_frames.json'
    metadata = json.loads(metadata_path.read_text())
    (output/'left_original_frame_metadata.json').write_text(json.dumps(metadata, indent=2))
    with np.load(npz, allow_pickle=False) as arrays:
        for index in (124, 125, 126):
            name = 'left_20260913_index'+str(index)
            points = {rep: arrays[rep+'_xyz'][index].astype(float) for rep in ('raw', 'filtered')}
            np.savez_compressed(output/(name+'.npz'), **points)
            entries.append({'name': name, 'side': 'left', 'date': '2026-09-13',
                'source': str(npz), 'npz_sha256': expected, 'frame_index': index,
                'coordinate_system': 'original sensor coordinates; no IMU or 9/14 mount transform applied',
                'pairing': 'same capture NPZ index; original frame metadata retained; weaker than SourceFrame CDR hash binding'})
        keys = list(arrays.keys())
    assert sha(npz.read_bytes()) == expected
    manifest = {'validation_level': 'REAL_BAG_EXISTING_NPZ', 'hardware_accessed': False,
        'ros_nodes_started': False, 'entries': entries, 'left_npz_keys': keys,
        'limitations': ['Historical frames, not the user current live view.',
            'Left and right were recorded on different dates and cannot be physically paired.',
            'No surveyed corner angle, photo annotation or direct optical depth is available.']}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2))


def explore(cache, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((cache/'manifest.json').read_text())
    for entry in manifest['entries']:
        if entry['name'] not in ('right_20260914_seq113', 'left_20260913_index125'):
            continue
        with np.load(cache/(entry['name']+'.npz')) as npz:
            a, b = npz['raw'], npz['filtered']
        finite = np.isfinite(a).all(axis=1) & (np.linalg.norm(a, axis=1) < 8)
        fig, axes = plt.subplots(3, 3, figsize=(15, 13), constrained_layout=True)
        for ax, (i, j, other) in zip(axes[0], ((0, 1, 2), (0, 2, 1), (1, 2, 0))):
            p=ax.scatter(a[finite, i], a[finite, j], c=a[finite, other], s=2, cmap='turbo')
            fig.colorbar(p, ax=ax, label='XYZ'[other]+' (m)')
            ax.set(xlabel='XYZ'[i]+' (m)', ylabel='XYZ'[j]+' (m)', title='Full raw context')
            ax.set_aspect('equal', adjustable='box')
        centers = (-.6, -.4, -.2, 0., .2, .4) if entry['side']=='right' else (-.5, 0., .5, 1., 1.5, 2.)
        for ax, y in zip(axes[1:].ravel(), centers):
            mask = finite & (abs(a[:, 1]-y)<.10)
            ax.scatter(a[mask, 0], a[mask, 2], s=10, c='#d55e00', label='raw')
            ax.scatter(b[mask, 0], b[mask, 2], s=8, marker='x', c='#0072b2', label='filtered same pixel')
            ax.set(xlabel='X (m)', ylabel='Z (m)', title=f'Raw Y = {y:g} +/- 0.10 m; N={mask.sum()}')
            ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.2); ax.legend(fontsize=7)
        fig.suptitle(entry['name']+' | exploratory fixed-coordinate slices; no surface rejection')
        fig.savefig(output/(entry['name']+'_explore.png'), dpi=145)
        plt.close(fig)
        if entry['side']=='left':
            fig, axes=plt.subplots(2, 3, figsize=(15, 9), constrained_layout=True)
            for ax, x in zip(axes.ravel(), (1.5, 1.8, 2.1, 2.4, 2.7, 3.0)):
                mask=finite & (abs(a[:, 0]-x)<.10)
                ax.scatter(a[mask, 1], a[mask, 2], s=10, c='#d55e00', label='raw')
                ax.scatter(b[mask, 1], b[mask, 2], s=8, marker='x', c='#0072b2', label='filtered same pixel')
                ax.set(xlabel='Y (m)', ylabel='Z (m)', title=f'Raw X = {x:g} +/- 0.10 m; N={mask.sum()}')
                ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.2); ax.legend(fontsize=7)
            fig.suptitle(entry['name']+' | exploratory X slices; no semantic wall annotation')
            fig.savefig(output/(entry['name']+'_x_slices.png'), dpi=145); plt.close(fig)


RIGHT_ROIS = {
    'floor': [[1.45, 1.95], [-1.0, -.2], [-.90, -.60]],
    'wall': [[2.45, 2.85], [-1.0, -.2], [-.10, .55]],
}


def box_mask(points, bounds):
    bounds=np.asarray(bounds)
    return np.isfinite(points).all(axis=1) & ((points>=bounds[:, 0]) & (points<=bounds[:, 1])).all(axis=1)


def quantiles(values):
    return {'count': len(values), 'p50': float(np.median(values)),
        'p95': float(np.quantile(values, .95)), 'max': float(np.max(values))} if len(values) else {'count': 0}


def fit_plane(points, kind):
    assert len(points)>=30
    center=points.mean(axis=0)
    _, singular, axes=np.linalg.svd(points-center, full_matrices=False)
    normal=axes[-1]
    if (kind=='floor' and normal[2]<0) or (kind=='wall' and normal@center>0):
        normal=-normal
    d=float(-normal@center)
    return {'normal': normal.tolist(), 'd_m': d, 'count': len(points),
        'center_m': center.tolist(), 'singular_values': singular.tolist(),
        'absolute_residual_m': quantiles(abs(points@normal+d)),
        'extent_m': np.array([points.min(axis=0),points.max(axis=0)]).tolist(),
        'in_plane_p05_p95_span_m': np.ptp(np.quantile((points-center)@axes[:2].T,[.05,.95],axis=0),axis=0).tolist()}


def analyze(cache, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    assert not output.exists(), 'OUTPUT_MUST_BE_NEW'
    output.mkdir(parents=True)
    manifest=json.loads((cache/'manifest.json').read_text())
    results=[]
    for entry in manifest['entries']:
        if entry['side']!='right': continue
        with np.load(cache/(entry['name']+'.npz')) as data:
            a,b=data['raw'],data['filtered']
        finite=np.isfinite(a).all(axis=1)
        common=finite & np.isfinite(b).all(axis=1)
        fits={}; masks={}; support={}
        for kind,bounds in RIGHT_ROIS.items():
            rawmask=box_mask(a,bounds)
            mask=rawmask & common
            masks[kind]=mask
            ids=np.flatnonzero(mask)
            fits[kind]={rep:fit_plane(points[mask],kind) for rep,points in (('raw',a),('filtered',b))}
            support[kind]={'raw_selected_count':int(rawmask.sum()), 'filtered_missing_count':int((rawmask & ~common).sum()),
                'shared_indices': ids.tolist(), 'shared_indices_sha256':sha(ids.astype('<u4').tobytes()),
                'raw_only_fit':fit_plane(a[rawmask],kind)}
        nf=np.array(fits['floor']['raw']['normal']); nw=np.array(fits['wall']['raw']['normal'])
        df=fits['floor']['raw']['d_m']; dw=fits['wall']['raw']['d_m']
        along=np.cross(nw,nf); along/=np.linalg.norm(along)
        if along[1]<0: along=-along
        inward=np.cross(along,nf)
        if inward@nw<0: inward=-inward
        # The slice is perpendicular to the measured plane intersection, centered
        # where its Y coordinate is -0.60 m. No 90-degree constraint is applied.
        origin=np.linalg.solve(np.array([nw,nf,[0,1,0]]),[-dw,-df,-.60])
        basis=np.array([inward,nf,along])
        qa=(a-origin)@basis.T; qb=(b-origin)@basis.T
        raw_roi=finite & (abs(qa[:,2])<=.10) & (qa[:,0]>=-.15) & (qa[:,0]<=1.35) & (qa[:,1]>=-.15) & (qa[:,1]<=1.4)
        slice_mask=raw_roi & common
        close_roi=raw_roi & (qa[:,0]<=.65) & (qa[:,1]<=.65)
        close=close_roi & common
        ids=np.flatnonzero(close)
        metrics={}
        for rep,points in (('raw',a),('filtered',b)):
            floor_dist=points[close]@nf+df; wall_dist=points[close]@nw+dw
            closest=np.minimum(floor_dist,wall_dist)
            metrics[rep]={'count':len(ids), 'positive_distance_from_both_raw_planes_m':quantiles(np.maximum(closest,0)),
                'distance_to_floor_raw_plane_m':floor_dist.tolist(),'distance_to_wall_raw_plane_m':wall_dist.tolist(),
                'points_inside_both_planes_beyond_m':{str(t):int(((floor_dist>t)&(wall_dist>t)).sum()) for t in (.03,.05,.08,.10)},
                'own_fit_positive_distance_from_both_planes_m':quantiles(np.maximum(np.minimum(
                    points[close]@np.asarray(fits['floor'][rep]['normal'])+fits['floor'][rep]['d_m'],
                    points[close]@np.asarray(fits['wall'][rep]['normal'])+fits['wall'][rep]['d_m']),0))}
            rays=a[close]/np.linalg.norm(a[close],axis=1)[:,None]
            denominators=np.column_stack((rays@nf,rays@nw))
            predicted=-np.array([df,dw])/denominators
            predicted[predicted<=0]=np.inf
            predicted=predicted.min(axis=1)
            assert np.isfinite(predicted).all()
            delta=np.linalg.norm(points[close],axis=1)-predicted
            raw_bridge=((a[close]@nf+df)>.05)&((a[close]@nw+dw)>.05)
            metrics[rep]['range_minus_first_positive_raw_plane_intersection_m']={
                'all_corner_p05_p50_p95':np.quantile(delta,[.05,.50,.95]).tolist(),
                'negative_count':int((delta<0).sum()),
                'raw_bridge_subset_count':int(raw_bridge.sum()),
                'raw_bridge_subset_p05_p50_p95':np.quantile(delta[raw_bridge],[.05,.50,.95]).tolist(),
                'assumption':'Extend the two raw support planes; raw rays originate at unchanged sensor origin. This is not physical range ground truth.'}
        angles={rep:float(np.degrees(np.arccos(np.clip(abs(np.dot(fits['floor'][rep]['normal'],fits['wall'][rep]['normal'])),0,1)))) for rep in ('raw','filtered')}
        row={'source':entry,'support_rois_m':RIGHT_ROIS,'support':support,'fits':fits,
            'acute_plane_angle_deg':angles,'basis_rows_inward_floor_normal_intersection':basis.tolist(),
            'intersection_origin_m':origin.tolist(),'slice_half_width_m':.10,
            'raw_slice_count':int(raw_roi.sum()),'shared_slice_count':int(slice_mask.sum()),
            'raw_corner_count':int(close_roi.sum()),'shared_corner_count':int(close.sum()),
            'raw_corner_filtered_missing':int((close_roi & ~common).sum()),
            'corner_same_pixel_indices':ids.tolist(),'corner_same_pixel_indices_sha256':sha(ids.astype('<u4').tobytes()),
            'corner_coordinates_inward_up_along_m':{'raw':qa[close].tolist(),'filtered':qb[close].tolist()},
            'corner_metrics_relative_to_raw_support_planes':metrics,
            'same_pixel_displacement_m':quantiles(np.linalg.norm(b[close]-a[close],axis=1)),
            'raw_corner_unshared_positive_distance_m':quantiles(np.maximum(np.minimum(a[close_roi & ~common]@nf+df,a[close_roi & ~common]@nw+dw),0))}
        results.append(row)
        if entry['name']!='right_20260914_seq113': continue
        fig,axes=plt.subplots(1,3,figsize=(17,7.2),gridspec_kw={'width_ratios':[1.0,1.15,1.15]})
        fig.subplots_adjust(top=.80,bottom=.18,left=.05,right=.985,wspace=.27)
        context=finite & (np.linalg.norm(a,axis=1)<5)
        axes[0].scatter(a[context,0],a[context,1],c='#dddddd',s=2,rasterized=True)
        for kind,color in (('floor','#009e73'),('wall','#8b56ab')):
            axes[0].scatter(a[masks[kind],0],a[masks[kind],1],s=7,c=color,label=kind+' support (raw ROI)')
        axes[0].scatter(a[raw_roi,0],a[raw_roi,1],s=4,c='#d55e00',label='20 cm slice')
        axes[0].set(xlabel='Reference X (m)',ylabel='Reference Y (m)',title='Raw scene context / top view')
        axes[0].legend(fontsize=8,loc='lower left')
        for ax in axes[1:]:
            ax.scatter(qa[slice_mask,0],qa[slice_mask,1],s=18,c='#d55e00',label='raw: before host filter',zorder=3)
            ax.scatter(qb[slice_mask,0],qb[slice_mask,1],s=16,c='#0072b2',marker='x',label='filtered: identical pixels',zorder=4)
            missing=raw_roi & ~common
            ax.scatter(qa[missing,0],qa[missing,1],s=22,facecolors='none',edgecolors='#777777',label='raw pixel missing in filtered',zorder=2)
            x=np.array([-.15,1.35]); y=np.array([-.15,1.4])
            ax.plot(x,[0,0],c='#009e73',lw=1.5,label='remote floor support extrapolated')
            ax.plot(-(nw@nf)*y/(nw@inward),y,c='#8b56ab',lw=1.5,label='remote wall support extrapolated')
            ax.set(xlabel='Distance inward along fitted floor (m)',ylabel='Height along fitted floor normal (m)')
        axes[1].set(title=f'Raw-selected 20 cm slice: {slice_mask.sum()} paired pixels',xlim=(-.10,1.3),ylim=(-.10,1.35))
        axes[1].legend(fontsize=7,loc='upper right')
        axes[2].add_collection(LineCollection(np.stack((qa[close,:2],qb[close,:2]),axis=1),colors='#999999',linewidths=.5,alpha=.6))
        axes[2].set(title=f'Corner detail: {close.sum()} identical pixel pairs',xlim=(-.07,.66),ylim=(-.07,.66))
        for ax in axes:
            ax.set_aspect('equal',adjustable='box'); ax.grid(alpha=.17)
        fig.suptitle('Recorded RIGHT sensor | 2026-09-14 23:39 CST | source seq 113 (t=11.30 s)\n'
            'Both branches already contain a curved transition; actual physical corner identity/angle is unmeasured.',fontsize=13,y=.965)
        fig.text(.5,.025,f"Planes fitted once to fixed remote ROIs; no RANSAC / no 90-degree constraint. Raw fitted acute angle: {angles['raw']:.2f} deg.\n"
            f"Corner pixels >5 cm inside BOTH extrapolated raw planes: raw {metrics['raw']['points_inside_both_planes_beyond_m']['0.05']}/{len(ids)}, "
            f"filtered {metrics['filtered']['points_inside_both_planes_beyond_m']['0.05']}/{len(ids)}. Offsets are not surveyed errors.",ha='center',fontsize=10)
        fig.savefig(output/'corner_right_same_frame.png',dpi=170); fig.savefig(output/'corner_right_same_frame.pdf'); plt.close(fig)
    report={'method':'Manual raw coordinate boxes away from transition, all shared support points in one TLS fit; no rejection, no forced right angle.',
        'selection_basis':'ROIs chosen from retained exploratory full context and raw slices of seq113; same boxes reused in seq4/57.',
        'corner_window':'Raw basis u -0.15..0.65 m, v -0.15..0.65 m, along +/-0.10 m; filtered uses exactly same pixel IDs.',
        'left_status':'NO_RELIABLY_ANNOTATED_WALL_FLOOR_PAIR: multiple foreground surfaces and occlusion in 9/13 data; exploratory slices retained, no angle/error claim.',
        'frames':results}
    (output/'corner_metrics.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    for row in results:
        print(json.dumps({'frame':row['source']['name'],'paired_corner_count':row['shared_corner_count'],
            'fits':{k:{rep:fit['absolute_residual_m'] for rep,fit in pairs.items()} for k,pairs in row['fits'].items()},
            'corner':{rep:{k:v for k,v in values.items() if k not in ('distance_to_floor_raw_plane_m','distance_to_wall_raw_plane_m')}
                for rep,values in row['corner_metrics_relative_to_raw_support_planes'].items()}}))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('extract', 'explore','analyze'))
    parser.add_argument('--root', type=Path, default=Path('/home/nvidia/wheelchair'))
    parser.add_argument('--cache', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args()
    if args.mode=='extract': extract(args.root, args.output)
    elif args.mode=='explore': explore(args.cache, args.output)
    else: analyze(args.cache,args.output)


if __name__=='__main__':
    main()
