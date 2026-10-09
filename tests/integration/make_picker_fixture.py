"""Explicit synthetic UI fixture, with a known right-to-left translation."""
import json
import sys
from pathlib import Path

right = [[1.5, -1., -1.], [2.1, -1., .2], [1.8, -.2, 1.],
         [2.4, .7, .8], [1.7, 1., -.3], [2.2, .2, -.8]]
translation = [.2, -.3, .1]
left = [[p[i]+translation[i] for i in range(3)] for p in right]
ids = {'left': 'SYNTHETIC-LEFT', 'right': 'SYNTHETIC-RIGHT'}
metadata = {side: {'units': 'm', 'coordinate_convention': 'FLU', 'sensor_id': identity,
    'raw_key': ['synthetic-browser-test', identity, 'synthetic-epoch', 0]} for side, identity in ids.items()}
data = {'schema_version': 1, 'status': 'PREPARED_NOT_VALIDATED', 'source_mode': 'synthetic',
    'units': 'm', 'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'}, 'sensor_ids': ids,
    'training': [{'id': 'SYNTHETIC-BROWSER-FIXTURE', 'left': left, 'right': right,
        'selected_raw_frames': metadata, 'time_quality': 'SYNTHETIC_NO_CLOCK_CLAIM'}], 'validation': []}
with Path(sys.argv[1]).open('x', encoding='utf-8') as output:
    json.dump(data, output)
