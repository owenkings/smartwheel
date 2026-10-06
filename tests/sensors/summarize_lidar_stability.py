#!/usr/bin/env python3
"""Compare repeatability on common pixel masks; never promote to calibration."""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--study',type=Path,required=True);a=p.parse_args()
    study=a.study;dest=study/'comparison';dest.mkdir(exist_ok=False)
    summary={'status':'DESCRIPTIVE_COMPARISON_NOT_ACCURACY','parameter_changes':False,
             'static_scene_independently_verified':False,'sides':{},'network':{},'source_frames':{}}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(13,8))
    for row,side in enumerate(('left','right')):
        labels=('dual_A',side+'_single','dual_B')
        reports={label:json.loads((study/'analysis'/(label+'_'+side+'.json')).read_text()) for label in labels}
        streams={label:np.load(study/label/(side+'.npz'),allow_pickle=False) for label in labels}
        item={};summary['sides'][side]=item
        for rep in ('raw','filtered'):
            mask=np.logical_and.reduce([np.asarray(reports[label]['representations'][rep]['pixel_valid_fraction'])>=.95 for label in labels])
            curves=[];values=[];details={}
            for label in labels:
                d=reports[label]['representations'][rep]
                span=np.asarray(d['pixel_range_p95_minus_p05_m'],dtype=float)[mask]
                delta=np.asarray(d['pixel_adjacent_abs_delta_p95_m'],dtype=float)[mask]
                details[label]={'common_pixel_count':int(mask.sum()),
                    'median_pixel_range_p95_p05_m':float(np.nanmedian(span)),
                    'p95_pixel_range_p95_p05_m':float(np.nanquantile(span,.95)),
                    'median_pixel_adjacent_abs_delta_p95_m':float(np.nanmedian(delta)),
                    'source_missing_frames':reports[label]['sequence']['missing_frames_between_samples'],
                    'arrival_interval_s':reports[label]['arrival']['positive_interval_summary_s']}
                values.append(details[label]['median_pixel_range_p95_p05_m']*1000)
                if rep=='raw':
                    xyz=streams[label][rep+'_xyz'][:,mask,:].astype(float)
                    ranges=np.linalg.norm(xyz,axis=2);ranges[ranges<=0]=np.nan
                    residual=np.nanmedian(np.abs(ranges-np.nanmedian(ranges,axis=0)),axis=1)*1000
                    host=streams[label]['host_ns']; t=(host-host[0])*1e-9
                    axes[row,1].plot(t,residual,label=label)
                sdk=streams[label]['sdk_frame_id'];dif=np.diff(sdk)
                summary['source_frames'][label+'_'+side]={'sdk_frame_id_delta_unique':np.unique(dif).tolist(),
                    'frame_count':len(sdk),'first_sdk_id':int(sdk[0]),'last_sdk_id':int(sdk[-1])}
            item[rep]=details
            x=np.arange(3)+(-.18 if rep=='raw' else .18)
            axes[row,0].bar(x,values,.36,label=rep)
        for label in labels: streams[label].close()
        axes[row,0].set_xticks(np.arange(3),labels);axes[row,0].set_ylabel('Median pixel temporal P95-P05 (mm)')
        axes[row,0].set_title(side+' | same pixels valid >=95% in all three runs');axes[row,0].legend()
        axes[row,1].set_title(side+' | raw median absolute deviation across common pixels')
        axes[row,1].set_xlabel('Elapsed host time (s)');axes[row,1].set_ylabel('Deviation from each pixel median (mm)');axes[row,1].legend()
    for label in ('dual_A','left_single','right_single','dual_B'):
        d=json.loads((study/label/'capture.json').read_text());net={}
        for group in ('Udp','Ip'):
            before=d['network_before'][group];after=d['network_after'][group]
            net[group]={k:after[k]-before[k] for k in before if k in after}
        summary['network'][label]={'deltas':net,'scope':'host-wide counters, not packet capture',
            'capture_status':d['status'],'paired_counts':{s:v['paired'] for s,v in d['sides'].items()},
            'diagnostic_tail':{s:v['diagnostics'][-1:] for s,v in d['sides'].items()}}
    fig.suptitle('Real stationary-scene assumption | fixed configuration | repeatability, not true range error')
    fig.tight_layout();fig.savefig(dest/'comparison.png',dpi=140);plt.close(fig)
    (dest/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps(summary))


if __name__=='__main__':main()
