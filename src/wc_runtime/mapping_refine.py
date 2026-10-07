"""Reproducible hardware-free bias, wheel/IMU and planar geometry refinement."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def save(path,data):
    path=Path(path);temporary=path.with_name(path.name+'.tmp')
    with temporary.open('w',encoding='utf-8') as f:
        json.dump(data,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(temporary,path)


def load_cell(directory):
    from .compare_native import load_index,digest
    directory=Path(directory);frames=load_index(directory)
    for row in frames.values():
        for kind in ('cloud','odom'):
            if digest(directory/'frontend'/row[kind+'_cdr_file'])!=row[kind+'_cdr_sha256']:
                raise ValueError('resume derived CDR hash mismatch')
    return {'directory':directory,'frames':frames,'summary':read(directory/'summary.json')}


def stage_hashes(directory):
    from .compare_native import digest
    directory=Path(directory)
    return {str(p.relative_to(directory)):digest(p) for p in sorted(directory.rglob('*')) if p.is_file()}


def verify_stage(directory,expected):
    if not expected or stage_hashes(directory)!=expected:
        raise ValueError('completed stage artifact hashes changed or absent: '+str(directory))


def compact_native_report(native,output):
    """Keep high-volume ACK/graph history once, in the original per-cell report."""
    from .compare_native import digest
    result={k:v for k,v in native.items() if k!='cells'}
    result['cells']={}
    for name,cell in native['cells'].items():
        path=Path(output)/'native'/name/'result.json'
        compact={k:v for k,v in cell.items() if k not in ('graphs','processed_info')}
        compact['detailed_result_file']=str(path.relative_to(output))
        compact['detailed_result_sha256']=digest(path)
        compact['processed_info_count']=len(cell.get('processed_info',[]))
        compact['graph_messages_count']=len(cell.get('graphs',[]))
        result['cells'][name]=compact
    return result


def run(args):
    # Imports after CLI parsing so --help never initializes ROS or native code.
    from .storage_policy import StoragePolicy
    from .mapping_compare import load_dataset,write_trajectory,run_native_maps
    from .compare_replay import RecordedPackets,replay_cell
    from .compare_native import digest
    from .offline_motion_adapter import prior_factory
    from .offline_planar_process import validate_process_noise
    from .offline_refinement_inputs import verify_candidate,extract_and_calibrate
    from .offline_geometry_replay import geometry_cell
    root=args.project_root.resolve(strict=True);storage=StoragePolicy(root);storage.check()
    source=storage.resolve(args.dataset).resolve(strict=True);output=storage.resolve(args.output)
    if any(p.is_symlink() for p in (output,*output.parents)) or '..' in output.parts:
        raise ValueError('output must not be linked or contain parent traversal')
    if source==output or source in output.parents or output in source.parents:
        raise ValueError('output must be separate from original source recording')
    if storage.enabled and not output.is_relative_to(storage.archive_root):raise ValueError('output must be under configured data root')
    if output.exists() and not args.resume:raise ValueError('output exists; use new name or explicit --resume')
    if args.resume and not output.is_dir():raise ValueError('resume output does not exist')
    if not 0<args.input_rate_hz<=10:raise ValueError('input rate must be in (0,10] Hz')
    if not 1<=args.domain<=232 or args.domain==83:raise ValueError('isolated nonproduction ROS domain required')
    if not math.isfinite(args.native_wall_interval_s) or not .02<=args.native_wall_interval_s<=60:
        raise ValueError('native wall interval must be finite in [.02,60]s')
    if len(args.native_cells)!=len(set(args.native_cells)):
        raise ValueError('duplicate native cell selection')
    if args.native_map and args.geometry=='off' and 'geometric_motion' in args.native_cells:
        raise ValueError('geometric_motion export requires --geometry on')
    if args.process_noise=='legacy':
        if args.linear_acceleration_psd is not None or args.angular_acceleration_psd is not None:
            raise ValueError('PSD parameters require --process-noise white_acceleration')
        process_noise=None
    else:
        process_noise=validate_process_noise({'model':'continuous_white_acceleration',
            'linear_acceleration_psd_m2_s3':.25 if args.linear_acceleration_psd is None else args.linear_acceleration_psd,
            'angular_acceleration_psd_rad2_s3':.25 if args.angular_acceleration_psd is None else args.angular_acceleration_psd,
            'status':'EXPERIMENTAL_NOT_CALIBRATED'})
    runtime,manifest,hashes=load_dataset(source,allow_partial=args.allow_partial,
        project_root=root,mechanical_initial=args.mechanical_initial)
    runtime['cloud_source']=args.cloud;runtime['prior_template']['offline_experiment']=True
    storage.check();output.mkdir(parents=True,exist_ok=args.resume)
    args._effective_output=output
    report={'status':'RUNNING','dataset':str(source),'input_hashes':hashes,
        'capture_status':manifest.get('status') if manifest else 'UNKNOWN',
        'recording_complete':manifest.get('recording_complete') if manifest else None,
        'absolute_accuracy':'NO_INDEPENDENT_TRUTH','hardware_started':False,
        'control_topics_published':False,'input_rate_hz':args.input_rate_hz,'cloud':args.cloud,
        'geometric_constraints':args.geometry,'process_noise':process_noise,'scope':'SESSION_OFFLINE_REFINEMENT',
        'default_live_estimator_modified':False,'cells':{},'native_map':{'status':'NOT_RUN'}}
    software=output/'refinement_software';software.mkdir(exist_ok=args.resume)
    module_names=('mapping_refine','offline_motion_calibration','offline_motion_adapter',
        'offline_refinement_inputs','offline_geometry_replay','offline_planar_registration','offline_planar_process','compare_replay')
    identities={}
    for name in module_names:
        original=Path(__file__).with_name(name+'.py');target=software/original.name
        if target.exists() and digest(target)!=digest(original):
            raise ValueError('resume implementation changed: '+name+'; use new output')
        if not target.exists():target.write_bytes(original.read_bytes())
        identities[name]=digest(original)
    report['software_sha256']=identities
    packets=RecordedPackets(source,runtime)
    try:
        if args.resume:
            cp=output/'motion_candidate.json';candidate=read(cp)
            if args.motion_candidate and digest(storage.resolve(args.motion_candidate))!=digest(cp):
                raise ValueError('resume candidate mismatch')
        elif args.motion_candidate:
            cp0=storage.resolve(args.motion_candidate).resolve(strict=True);candidate=read(cp0)
            cp=output/'motion_candidate.json';cp.write_bytes(cp0.read_bytes())
        else:
            candidate,cp0,sha=extract_and_calibrate(source,runtime,packets,output/'calibration',input_hashes=hashes)
            cp=output/'motion_candidate.json';cp.write_bytes(Path(cp0).read_bytes())
        proof=verify_candidate(candidate,cp,source,hashes,runtime=runtime,packets=packets)
        save(output/'candidate_verification.json',proof);sha=digest(cp)
        runtime['offline_refinement']={'candidate_sha256':sha,'candidate_id':candidate['candidate_id'],
            'motion_correction_scope':'SESSION_OFFLINE_ONLY'}
        if process_noise is not None:runtime['offline_refinement']['process_noise']=process_noise
        report.update(candidate_sha256=sha,candidate_id=candidate['candidate_id'],controlled_clock=packets.clock.report())
        frozen=output/'frozen_runtime_config.json'
        if frozen.exists():
            # Stage runner and CLI must use exactly the same effective parameters.
            if read(frozen)!=runtime:raise ValueError('resume effective runtime configuration changed')
        else:save(frozen,runtime)
        done=output/'motion_stage_complete.json'
        if done.exists():
            old=read(done)
            if old['status']!='COMPLETE' or old['input_hashes']!=hashes or old['candidate_sha256']!=sha:
                raise ValueError('completed motion stage belongs to different input/candidate')
            verify_stage(output/'calibrated_motion',old.get('artifact_hashes'))
            cell=load_cell(output/'calibrated_motion')
            if cell['summary']['input_rate_hz']!=args.input_rate_hz:raise ValueError('resume input rate changed')
        else:
            save(output/'motion_stage_start.json',{'dataset':str(source),'input_hashes':hashes,'status':'RUNNING',
                'candidate_sha256':sha,'hardware_started':False})
            print(json.dumps({'stage':'REPLAY_MOTION','events':len(packets.events)}),flush=True)
            cell=replay_cell(root,output,runtime,runtime,packets,'calibrated_motion','five_state',True,0,
                args.input_rate_hz,storage_root=storage.archive_root or root,
                prior_factory=prior_factory(candidate,sha,process_noise=process_noise))
            write_trajectory(cell)
            save(done,{'status':'COMPLETE','input_hashes':hashes,'candidate_sha256':sha,
                'cell_summary':cell['summary'],'controlled_clock':packets.clock.report(),'hardware_started':False,
                'artifact_hashes':stage_hashes(cell['directory'])})
        report['cells']['calibrated_motion']=cell['summary'];cells={'calibrated_motion':cell}
    finally:packets.close()
    save(output/'refinement_result.json',report)
    if args.geometry=='on':
        done=output/'geometry_stage_complete.json'
        if done.exists():
            old=read(done)
            if old.get('input_index_sha256')!=digest(output/'calibrated_motion/frontend/index.jsonl'):
                raise ValueError('geometry source index changed')
            verify_stage(output/'geometric_motion',old.get('artifact_hashes'))
            geometry=load_cell(output/'geometric_motion')
        else:
            geometry=geometry_cell(output,cell['directory'])
            save(done,{'status':'COMPLETE','input_index_sha256':digest(output/'calibrated_motion/frontend/index.jsonl'),
                'summary':geometry['summary'],'artifact_hashes':stage_hashes(geometry['directory'])})
        cells['geometric_motion']=geometry;report['cells']['geometric_motion']=geometry['summary']
    save(output/'refinement_result.json',report)
    if args.native_map:
        selected={k:v for k,v in cells.items() if k in args.native_cells}
        if not selected:raise ValueError('requested native cell was not produced')
        done=output/'native_stage_complete.json'
        if done.exists():
            native=read(done)
            if set(native['cells'])!=set(selected):raise ValueError('native selection changed; use new output')
            verify_stage(output/'native',native.get('artifact_hashes'))
        else:
            args.native_rate_hz=args.input_rate_hz;args.native_limit=0
            native=run_native_maps(root,output,selected,sorted(set.intersection(*(set(v['frames']) for v in selected.values()))),
                args,storage_root=storage.archive_root or root)
            native=compact_native_report(native,output)
            native['artifact_hashes']=stage_hashes(output/'native')
            save(done,native)
        report['native_map']=native
    for path,expected in hashes.items():
        if digest(path)!=expected:raise ValueError('original input changed during refinement: '+path)
    storage.check();report['status']='FINALIZING_ARTIFACT_HASHES'
    print(json.dumps({'stage':'FINALIZING_ARTIFACT_HASHES','output':str(output)}),flush=True)
    report['original_inputs_unchanged']=True;save(output/'refinement_result.json',report)
    artifacts={}
    for p in sorted(output.rglob('*')):
        if p.is_file() and p.name not in ('refinement_artifacts.json','refinement_failure.json','refinement_complete.json'):
            artifacts[str(p.relative_to(output))]={'bytes':p.stat().st_size,'sha256':digest(p)}
    save(output/'refinement_artifacts.json',{'status':report['status'],'input_hashes':hashes,'files':artifacts})
    report['status']='OFFLINE_REFINEMENT_COMPLETE_REQUIRES_MAP_REVIEW'
    save(output/'refinement_result.json',report)
    artifacts['refinement_result.json']={'bytes':(output/'refinement_result.json').stat().st_size,
        'sha256':digest(output/'refinement_result.json')}
    save(output/'refinement_artifacts.json',{'status':report['status'],'input_hashes':hashes,'files':artifacts})
    save(output/'refinement_complete.json',{'status':report['status'],
        'result_sha256':digest(output/'refinement_result.json'),'artifact_manifest_sha256':digest(output/'refinement_artifacts.json')})
    print(json.dumps({'stage':'COMPLETE','output':str(output),'absolute_accuracy':report['absolute_accuracy']},ensure_ascii=False),flush=True)
    return 0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--project-root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--motion-candidate',type=Path,help='Validated session-specific candidate; omitted = estimate with disjoint holdout')
    parser.add_argument('--allow-partial',action='store_true')
    parser.add_argument('--mechanical-initial',action='store_true')
    parser.add_argument('--resume',action='store_true',help='Continue only completed, hash-matched stages; partial stages are never silently reused')
    parser.add_argument('--cloud',choices=('raw','filtered'),default='raw')
    parser.add_argument('--input-rate-hz',type=float,default=5.)
    parser.add_argument('--process-noise',choices=('legacy','white_acceleration'),default='legacy',
        help='Explicit offline process model comparison; existing live/default noise semantics are preserved')
    parser.add_argument('--linear-acceleration-psd',type=float,help='White-acceleration intensity in m^2/s^3; experimental default .25')
    parser.add_argument('--angular-acceleration-psd',type=float,help='White-angular-acceleration intensity in rad^2/s^3; experimental default .25')
    parser.add_argument('--geometry',choices=('on','off'),default='off',help='Experimental geometry; off by default until matched-map benefit is demonstrated')
    parser.add_argument('--native-map',action='store_true')
    parser.add_argument('--native-cells',nargs='+',choices=('calibrated_motion','geometric_motion'),default=['calibrated_motion'])
    parser.add_argument('--domain',type=int,default=89)
    parser.add_argument('--native-wall-interval-s',type=float,default=.2)
    args=parser.parse_args(argv)
    try:return run(args)
    except Exception as e:
        if hasattr(args,'_effective_output'):
            try:
                save(args._effective_output/'refinement_failure.json',{'status':'FAILED',
                    'error_type':type(e).__name__,'reason':str(e),'time_unix_ns':time.time_ns()})
            except OSError as save_error:
                print('Unable to persist failure evidence: '+str(save_error),file=sys.stderr,flush=True)
        print(json.dumps({'stage':'REFINEMENT_FAILED','error':type(e).__name__+': '+str(e)},ensure_ascii=False),file=sys.stderr,flush=True)
        return 2


if __name__=='__main__':raise SystemExit(main())
