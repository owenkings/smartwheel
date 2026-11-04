"""Build a portable read-only HTML view of a frozen navigation snapshot."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

MAX_SNAPSHOT_BYTES = 3_000_000
PLACEHOLDER = '__WC_DATA__'


def _reject_constant(value):
    raise ValueError('snapshot contains nonfinite JSON number: ' + value)


def read_snapshot(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > MAX_SNAPSHOT_BYTES:
        raise ValueError('ordinary bounded snapshot JSON required')
    raw = path.read_bytes()
    value = json.loads(raw.decode('utf-8'), parse_constant=_reject_constant)
    if not isinstance(value, dict):
        raise ValueError('snapshot must be a JSON object')
    for key in ('summary', 'mapping', 'near', 'localization', 'onsite'):
        if not isinstance(value.get(key), dict):
            raise ValueError('snapshot is missing section: ' + key)
    meta = value.get('snapshot')
    if not isinstance(meta, dict) or meta.get('mode') != 'OFFLINE_SNAPSHOT':
        raise ValueError('offline dashboard requires explicit snapshot metadata')
    if meta.get('physical_navigation_validated') is not False:
        raise ValueError('offline dashboard requires explicit unvalidated physical status')
    if meta.get('live_sensor_feed') is not False:
        raise ValueError('offline dashboard cannot imply a live sensor feed')
    return value, raw


def render_html(template, snapshot):
    if template.count(PLACEHOLDER) != 1:
        raise ValueError('dashboard template requires exactly one data placeholder')
    encoded = json.dumps(snapshot, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
    # Raw-text script elements terminate on </script even inside JSON strings.
    # Encode markup characters so a report label cannot inject HTML or scripts.
    encoded = (encoded.replace('&', '\\u0026').replace('<', '\\u003c').replace('>', '\\u003e')
               .replace('\u2028', '\\u2028').replace('\u2029', '\\u2029'))
    return template.replace(PLACEHOLDER, encoded)


def reports_destination(policy, value):
    output = policy.resolve(value)
    logical = Path(policy.logical(output))
    if not logical.parts or logical.parts[0] != 'reports' or output.suffix.lower() != '.html':
        raise ValueError('dashboard output must be an HTML file in configured reports storage')
    if output.exists():
        raise ValueError('refusing to overwrite an existing dashboard snapshot')
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(args.project_root)
    policy.check()
    output = reports_destination(policy, args.output)
    snapshot, raw = read_snapshot(args.snapshot)
    template_path = Path(__file__).with_name('progress_dashboard.html')
    template = template_path.read_text(encoding='utf-8')
    rendered = render_html(template, snapshot)
    if args.snapshot.read_bytes() != raw:
        raise ValueError('snapshot changed while rendering')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        stream.write(rendered)
    policy.check()
    print(json.dumps({'status': 'READ_ONLY_OFFLINE_DASHBOARD_CREATED', 'output': str(output),
                      'snapshot_sha256': hashlib.sha256(raw).hexdigest(),
                      'html_sha256': hashlib.sha256(rendered.encode()).hexdigest(),
                      'physical_navigation_validated': False,
                      'hardware_started': False, 'control_topics_published': False}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
