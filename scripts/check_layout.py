#!/usr/bin/env python3
"""Check maintained directory guides and local Markdown file links."""
import argparse
import json
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit

MAINTAINED = ('src', 'scripts', 'config', 'tests', 'docs', 'vendor_patches')
GENERATED = {'.git', '__pycache__', '.pytest_cache', 'node_modules'}


def check(root):
    root = Path(root).resolve()
    errors = []
    folders = []
    for name in MAINTAINED:
        base = root / name
        if not base.is_dir():
            errors.append('Missing maintained directory: ' + name)
            continue
        for folder in [base, *sorted(p for p in base.rglob('*') if p.is_dir())]:
            relative = folder.relative_to(root)
            if any(part in GENERATED or part.endswith('.egg-info') for part in relative.parts):
                continue
            if relative.parts[:2] in (('config', 'local'), ('config', 'panel_profiles')):
                continue
            if relative.parts[:3] in (('config', 'calibration', 'panel_extrinsics'), ('config', 'calibration', 'panel_live')):
                continue
            folders.append(folder)
            if not (folder / 'README.md').is_file():
                errors.append('Missing README: ' + relative.as_posix())
    documents = [root / 'README.md', root / 'CONTRIBUTING.md']
    documents += [p for folder in folders for p in folder.glob('*.md')]
    for document in documents:
        if not document.is_file():
            errors.append('Missing document: ' + str(document.relative_to(root)))
            continue
        text = document.read_text(encoding='utf-8-sig')
        text = re.sub(r'```.*?```', '', text, flags=re.S)
        for destination in re.findall(r'\]\(([^)]+)\)', text):
            destination = destination.strip().split(' "', 1)[0].strip('<>')
            parsed = urlsplit(destination)
            if not parsed.path or parsed.scheme or destination.startswith(('/', '\\')):
                continue
            target = document.parent / unquote(parsed.path)
            if not target.exists():
                errors.append(str(document.relative_to(root)) + ': missing link ' + destination)
    return {'status': 'PASS' if not errors else 'FAIL', 'directories': 1 + len(folders),
            'documents': len(documents), 'errors': errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    result = check(args.root)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not result['errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
