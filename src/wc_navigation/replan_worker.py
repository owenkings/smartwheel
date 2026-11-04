"""One isolated Nav2 request per process, with normal owned-child cleanup."""
import argparse
import json
from pathlib import Path
import signal

from .cli import write_json


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',required=True,type=Path)
    parser.add_argument('--result',required=True,type=Path)
    args=parser.parse_args(argv)
    if args.result.exists() or args.request.stat().st_size>1_000_000:
        raise ValueError('fresh result and bounded request required')
    request=json.loads(args.request.read_text(encoding='utf-8'))
    def terminate(signum, frame):
        # Raising through plan_nav2 executes its finally block and reaps only
        # the map/planner process groups owned by this worker.
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        raise RuntimeError('planning worker termination requested')
    signal.signal(signal.SIGTERM,terminate)
    from .nav2_planner import plan_nav2
    try:
        result=plan_nav2(**request)
        response={'status':'PATH','result':result}
    except Exception as exc:
        response={'status':'NAV2_ERROR','error':type(exc).__name__+': '+str(exc)}
    write_json(args.result,response)
    return 0 if response['status']=='PATH' else 1


if __name__=='__main__':raise SystemExit(main())
