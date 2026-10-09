"""Hardware-free candidate stage shared by official and five-state comparison."""
import argparse
from pathlib import Path
import shutil
from .storage import atomic_json, resolve_user_destination


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root',type=Path,required=True)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--allow-partial',action='store_true')
    parser.add_argument('--mechanical-initial',action='store_true')
    parser.add_argument('--sides',choices=('all','left','right'))
    parser.add_argument('--cloud',choices=('raw','filtered'),default='raw')
    parser.add_argument('--min-turn-windows',type=int,default=30,
                        help='Offline experiment: minimum 0.1 s windows per turn direction; default 30')
    args=parser.parse_args(argv)
    if args.min_turn_windows < 1: parser.error('--min-turn-windows must be a positive integer')
    source,sg=resolve_user_destination(args.project_root,args.dataset)
    output,og=resolve_user_destination(args.project_root,args.output,must_exist=False)
    if source==output or source.is_relative_to(output) or output.is_relative_to(source):
        raise ValueError('候选输出必须与原始录制分离')
    if output.exists(): raise ValueError('候选目录已存在')
    from wc_runtime.mapping_compare import load_dataset
    from wc_runtime.compare_replay import RecordedPackets
    from wc_runtime.offline_refinement_inputs import extract_and_calibrate,verify_candidate
    runtime,manifest,hashes=load_dataset(source,allow_partial=args.allow_partial,project_root=args.project_root,
                                        mechanical_initial=args.mechanical_initial,sides=args.sides,cloud=args.cloud)
    # Match compare/refine: use a recorded device-time declaration when present.
    runtime['offline_imu_time_mode']='auto'
    packets=RecordedPackets(source,runtime)
    from wc_runtime.offline_imu_time import configure_runtime
    configure_runtime(runtime,packets.clock)
    sg.check();og.check()
    print('离线运动候选：每个转向至少 {} 个有效窗口；其他校验保持不变。'.format(args.min_turn_windows),flush=True)
    candidate,path,digest=extract_and_calibrate(source,runtime,packets,output/'evidence',input_hashes=hashes,
                                               min_turn_windows=args.min_turn_windows)
    proof=verify_candidate(candidate,path,source,hashes,runtime=runtime,packets=packets)
    sg.check();og.check()
    shutil.copyfile(path,output/'motion_candidate.json')
    atomic_json(output/'verification.json',proof)
    print('运动修正候选校验完成: '+str(output/'motion_candidate.json'),flush=True)
    return 0


if __name__=='__main__': raise SystemExit(main())
