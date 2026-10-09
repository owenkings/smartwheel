"""Offline JSONL encoder feedback replay; no ROS or device access."""

import argparse
import json
from pathlib import Path

from .ros_node import FeedbackProcessor


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    processor = FeedbackProcessor(json.loads(args.config.read_text(encoding='utf-8')))
    with args.input.open(encoding='utf-8') as source, args.output.open('x', encoding='utf-8') as target:
        for number, line in enumerate(source, 1):
            if not line.strip():
                continue
            raw, estimate = processor.process(line)
            target.write(json.dumps({'line': number, 'raw': raw,
                                     'estimate': estimate.to_dict() if estimate else None}, allow_nan=False) + '\n')
    print(json.dumps(processor.status(), allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
