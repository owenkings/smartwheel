#!/usr/bin/env python3
"""Read-only SDK provenance check; no import, build, sample or device execution.

Only CRLF-to-LF normalization is allowed for source text. Library bytes are
hashed verbatim. The trusted project manifest must be updated through review;
this checker never creates one from unreviewed local input or repairs a checkout.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

TEXT_SUFFIXES = {'.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp', '.hxx', '.inc', '.inl', '.ipp', '.tpp', '.in'}


def require(ok, reason):
    if not ok:
        raise RuntimeError('XT_VENDOR_REVIEW_REQUIRED: '+reason)


def mode(path):
    if path.suffix in TEXT_SUFFIXES:
        return 'text_crlf_to_lf'
    if path.suffix in ('.a', '.so') or '.so.' in path.name:
        return 'binary'
    return None


def checksum(path, normalized=False):
    data = path.read_bytes()
    if normalized:
        data = data.replace(b'\r\n', b'\n')
    return hashlib.sha256(data).hexdigest()


def git(repository, *arguments):
    environment = os.environ.copy()
    # The exported tree may live inside the parent project's Git checkout.
    # Never let discovery, caller GIT_DIR or a worktree override select it.
    for key in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_COMMON_DIR', 'GIT_INDEX_FILE',
                'GIT_OBJECT_DIRECTORY', 'GIT_ALTERNATE_OBJECT_DIRECTORIES'):
        environment.pop(key, None)
    environment['GIT_CEILING_DIRECTORIES'] = str(repository.parent.resolve())
    completed = subprocess.run(['git', '-c', 'core.autocrlf=false', '-C', str(repository), *arguments],
                               capture_output=True, text=True, timeout=30, check=False, env=environment)
    require(completed.returncode == 0, 'git check failed: '+completed.stderr.strip())
    return completed.stdout.strip()


def reverse_check_copy(observed, manifest, patch_path, temporary_root):
    temporary_root = Path(temporary_root).resolve(strict=True)
    temporary = temporary_root/('.xt-vendor-check-'+uuid.uuid4().hex)
    # Only this newly-created directory is ever removed. No SDK path is copied
    # as a symlink or modified; source text is rehashed after normalization.
    temporary.mkdir(mode=0o755)
    try:
        for relative, expected in manifest['files'].items():
            if expected['mode'] != 'text_crlf_to_lf':
                continue
            data = observed[relative].read_bytes().replace(b'\r\n', b'\n')
            require(hashlib.sha256(data).hexdigest() == expected['sha256'], 'source changed while making check copy: '+relative)
            destination = temporary/'xtsdk_cpp/xtsdk'/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
        require(checksum(patch_path) == manifest['patch_sha256'], 'patch changed while making check copy')
        # Apply neither direction. Do not use --ignore-whitespace: only CRLF
        # normalization already covered by manifest policy is permitted.
        git(temporary, 'apply', '--reverse', '--check', str(patch_path))
    finally:
        require(temporary.parent == temporary_root and temporary.resolve() == temporary
                and not temporary.is_symlink(), 'temporary cleanup path changed')
        shutil.rmtree(temporary)


def verify(sdk_root, manifest_path, patch_path, temporary_root=None):
    sdk_root, manifest_path, patch_path = map(lambda p: Path(p).resolve(strict=True),
                                              (sdk_root, manifest_path, patch_path))
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    require(manifest.get('schema_version') == 1, 'unsupported manifest schema')
    repository = sdk_root.parent.parent
    require(sdk_root.relative_to(repository).as_posix() == 'xtsdk_cpp/xtsdk', 'SDK tree location differs')
    marker = repository/'.git'
    checkout = marker.exists() or marker.is_symlink()
    head = None
    if checkout:
        # A malformed/mismatched existing marker must fail, never fall back to
        # exported-snapshot semantics or inherit a different repository's HEAD.
        require(Path(git(repository, 'rev-parse', '--show-toplevel')).resolve() == repository, 'SDK must have its own recorded Git checkout')
        head = git(repository, 'rev-parse', 'HEAD')
        require(head == manifest['sdk_commit'], 'SDK HEAD differs from reviewed commit')
    require(checksum(patch_path) == manifest['patch_sha256'], 'project patch bytes differ from reviewed manifest')
    observed = {path.relative_to(sdk_root).as_posix(): path for path in sdk_root.rglob('*') if path.is_file() and mode(path)}
    require(set(observed) == set(manifest['files']), 'source/header/library file inventory differs')
    evidence = {}
    for relative, expected in manifest['files'].items():
        path = observed[relative]
        require(not path.is_symlink() and path.resolve().is_relative_to(sdk_root), 'symlink/outside input: '+relative)
        require(mode(path) == expected['mode'], 'hash mode differs: '+relative)
        canonical = checksum(path, expected['mode'] == 'text_crlf_to_lf')
        require(canonical == expected['sha256'], 'content mismatch: '+relative)
        evidence[relative] = {'sha256': canonical, 'raw_sha256': checksum(path), 'mode': expected['mode']}
    # Canonical hashes above must ALL pass before making any check-only copy.
    # An exported CRLF tree lacks Git checkout conversion; check the exact same
    # CRLF-to-LF representation as the manifest in a fresh isolated directory.
    reverse_check_copy(observed, manifest, patch_path, temporary_root or manifest_path.parent)
    return {'status': 'VERIFIED_GIT_CHECKOUT' if checkout else 'VERIFIED_EXPORTED_SNAPSHOT',
            'git_checkout_verified': checkout, 'sdk_head': head,
            'sdk_commit_semantics': 'verified_sdk_HEAD' if checkout else 'reviewed_manifest_source_origin',
            'sdk_commit': manifest['sdk_commit'], 'sdk_root': str(sdk_root),
            'manifest_sha256': checksum(manifest_path), 'patch_sha256': checksum(patch_path),
            'files': evidence, 'reverse_patch_check': 'PASS',
            'reverse_patch_check_mode': 'CRLF_TO_LF_COPY', 'temporary_copy_removed': True}


def verify_filter_runtime(runtime_root, manifest_path):
    root, source = Path(runtime_root).resolve(strict=True), Path(manifest_path).resolve(strict=True)
    manifest = json.loads(source.read_text(encoding='utf-8'))
    require(manifest.get('schema_version') == 1 and manifest.get('commit') == '3d3db067ae9bdc0528202c3087bc10fd3b706638', 'filter origin differs')
    require(set(manifest['files']) == {'lib/linux/aarch64/libxtsdk_shared.so', 'lib/linux/x86_64/libxtsdk_shared.so'}, 'filter runtime inventory differs')
    result = {}
    for relative, digest in manifest['files'].items():
        path = root/relative
        require(path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(root), 'filter binary path missing/outside')
        require(checksum(path) == digest, 'filter binary hash mismatch: '+relative)
        result[relative] = digest
    return {'origin': manifest['origin'], 'commit': manifest['commit'], 'manifest_sha256': checksum(source), 'files': result, 'status': 'VERIFIED_PINNED_FILTER_ABI_BUNDLE'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-root', required=True, type=Path)
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--patch', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--filter-root', type=Path)
    parser.add_argument('--filter-manifest', type=Path)
    parser.add_argument('--temporary-root', type=Path)
    parser.add_argument('--build-setting', action='append', default=[])
    parser.add_argument('--project-file', action='append', default=[], type=Path)
    args = parser.parse_args()
    try:
        evidence = verify(args.sdk_root, args.manifest, args.patch, args.temporary_root or args.output.parent)
        if args.filter_root or args.filter_manifest:
            require(args.filter_root is not None and args.filter_manifest is not None, 'both filter root and manifest required')
            evidence['filter_runtime'] = verify_filter_runtime(args.filter_root, args.filter_manifest)
        evidence['build_settings'] = dict(value.split('=', 1) for value in args.build_setting)
        evidence['project_inputs_raw_sha256'] = {str(path.resolve(strict=True)): checksum(path) for path in args.project_file}
        # Build-directory provenance only. Never touch an SDK source or library.
        with args.output.open('w', encoding='utf-8') as output:
            json.dump(evidence, output, indent=2, sort_keys=True)
            output.flush()
            os.fsync(output.fileno())
        print('XT_VENDOR_VERIFIED '+evidence['sdk_commit']+' '+evidence['manifest_sha256']+' '+evidence['status'])
        return 0
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
