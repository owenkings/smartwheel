"""CLI parser/forwarding subprocess with all launch/hardware dispatch replaced by a recorder.

This is expressly a parser test harness, not a test of hardware target identity
or successful ROS launches. Actual supervisor lifecycle is tested separately.
"""

import json
import os
from pathlib import Path
import sys

from wc_runtime import cli


def attempted(kind, *arguments, **keywords):
    path = Path(os.environ['WC_OPS_DISPATCH_LOG'])
    with path.open('a', encoding='utf-8') as output:
        output.write(json.dumps({'kind': kind, 'arguments': repr(arguments), 'keywords': repr(keywords)})+'\n')
    return 0


cli.target = lambda: None
cli.begin = lambda *args, **kwargs: attempted('begin', *args, **kwargs)
cli.device_preflight = lambda: attempted('device_preflight')
cli.subprocess.call = lambda *args, **kwargs: attempted('subprocess.call', *args, **kwargs)
cli.subprocess.Popen = lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError('subprocess launch forbidden in parser probe'))


if __name__ == '__main__':
    raise SystemExit(cli.main(sys.argv[1:]))
