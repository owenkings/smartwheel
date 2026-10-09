"""Synthetic Git/source fixtures exercise the production read-only SDK checker.

No actual vendor checkout is modified, downloaded, built or executed.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess

import pytest

CHECKER = Path(__file__).resolve().parents[2]/'src/wc_xt_driver/cmake/verify_vendor.py'
spec = importlib.util.spec_from_file_location('wc_test_vendor_checker', CHECKER)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def command(repository, *arguments):
    return subprocess.check_output(['git', '-C', str(repository), *arguments], stderr=subprocess.STDOUT)


@pytest.fixture
def reviewed(tmp_path):
    repository = tmp_path/'synthetic-sdk'
    sdk = repository/'xtsdk_cpp/xtsdk'
    sdk.mkdir(parents=True)
    source = sdk/'transport.cpp'
    header = sdk/'nested/protocol.hpp'
    header.parent.mkdir()
    source.write_bytes(b'int reviewed_guard = 0;\n')
    header.write_bytes(b'#pragma once\n')
    library = sdk/'lib/linux/aarch64/libxtsdk_shared.so'
    library.parent.mkdir(parents=True)
    library.write_bytes(b'SYNTHETIC INERT BYTES, NOT A LIBRARY\x00\x01')
    command(repository, 'init', '-q')
    command(repository, 'config', 'core.autocrlf', 'false')
    command(repository, 'add', '.')
    command(repository, '-c', 'user.name=Software Fixture', '-c', 'user.email=fixture@wheelchair.invalid', 'commit', '-qm', 'synthetic baseline')
    commit = command(repository, 'rev-parse', 'HEAD').decode().strip()
    source.write_bytes(b'int reviewed_guard = 1;\n')
    patch = tmp_path/'reviewed.patch'
    patch.write_bytes(command(repository, 'diff', '--no-ext-diff', '--binary'))
    manifest = tmp_path/'reviewed.json'
    files = {path.relative_to(sdk).as_posix(): {'mode': checker.mode(path),
              'sha256': checker.checksum(path, checker.mode(path) == 'text_crlf_to_lf')}
             for path in sdk.rglob('*') if path.is_file() and checker.mode(path)}
    manifest.write_text(json.dumps({'schema_version': 1, 'sdk_commit': commit,
                                    'patch_sha256': checker.checksum(patch), 'files': files}))
    return sdk, manifest, patch, source, header, library


def verify(reviewed):
    return checker.verify(*reviewed[:3])


def test_exact_reviewed_input_passes_without_mutating_files(reviewed):
    before = {p: p.read_bytes() for p in reviewed if p.is_file()}
    result = verify(reviewed)
    assert result['status'] == 'VERIFIED_GIT_CHECKOUT'
    assert result['git_checkout_verified'] is True
    assert result['sdk_head'] == result['sdk_commit']
    assert result['reverse_patch_check'] == 'PASS'
    assert result['files']['nested/protocol.hpp']['mode'] == 'text_crlf_to_lf'
    assert before == {p: p.read_bytes() for p in before}


def test_only_crlf_difference_is_allowed(reviewed):
    reviewed[4].write_bytes(b'#pragma once\r\n')
    result = verify(reviewed)
    item = result['files']['nested/protocol.hpp']
    assert item['sha256'] != item['raw_sha256']
    reviewed[4].write_bytes(b'#pragma  once\r\n')
    with pytest.raises(RuntimeError, match='content mismatch: nested/protocol.hpp'):
        verify(reviewed)


@pytest.mark.parametrize('index', [3, 4, 5])
def test_source_recursive_header_and_library_edits_fail(reviewed, index):
    path = reviewed[index]
    path.write_bytes(path.read_bytes()+b'changed')
    with pytest.raises(RuntimeError, match='content mismatch'):
        verify(reviewed)


def test_missing_patch_application_fails(reviewed):
    reviewed[3].write_bytes(b'int reviewed_guard = 0;\n')
    with pytest.raises(RuntimeError, match='content mismatch'):
        verify(reviewed)


def test_patch_bytes_change_fails(reviewed):
    reviewed[2].write_bytes(reviewed[2].read_bytes()+b'\n')
    with pytest.raises(RuntimeError, match='patch bytes differ'):
        verify(reviewed)


def test_different_git_head_fails(reviewed):
    repository = reviewed[0].parent.parent
    command(repository, '-c', 'user.name=Software Fixture', '-c', 'user.email=fixture@wheelchair.invalid',
            'commit', '--allow-empty', '-qm', 'different revision')
    with pytest.raises(RuntimeError, match='HEAD differs'):
        verify(reviewed)


@pytest.mark.parametrize('filename', ['new.cpp', 'new.hpp', 'libextra.a', 'libextra.so.1'])
def test_added_compilation_or_library_input_fails(reviewed, filename):
    (reviewed[0]/filename).write_bytes(b'UNREVIEWED')
    with pytest.raises(RuntimeError, match='inventory differs'):
        verify(reviewed)


def test_reverse_apply_failure_is_not_ignored_even_when_hashes_match(reviewed):
    # A forged synthetic manifest claiming an unapplied source is not enough:
    # the actual approved diff must also be reversibly present.
    source, manifest = reviewed[3], reviewed[1]
    source.write_bytes(b'int reviewed_guard = 0;\n')
    data = json.loads(manifest.read_text())
    data['files']['transport.cpp']['sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    manifest.write_text(json.dumps(data))
    with pytest.raises(RuntimeError, match='git check failed'):
        verify(reviewed)


def test_symlinked_compilation_input_fails(reviewed, tmp_path):
    source = reviewed[3]
    replacement = tmp_path/'outside.cpp'
    replacement.write_bytes(source.read_bytes())
    source.unlink()
    source.symlink_to(replacement)
    with pytest.raises(RuntimeError, match='symlink/outside input'):
        verify(reviewed)


def export_fixture(reviewed, parent):
    sdk = parent/'SDKs/exported/xtsdk_cpp/xtsdk'
    shutil.copytree(reviewed[0], sdk)
    return (sdk, reviewed[1], reviewed[2], sdk/'transport.cpp', sdk/'nested/protocol.hpp',
            sdk/'lib/linux/aarch64/libxtsdk_shared.so')


@pytest.mark.parametrize('parent_is_git', [False, True])
def test_export_snapshot_never_inherits_parent_git_identity(reviewed, tmp_path, parent_is_git):
    parent = tmp_path/'parent-project'
    parent.mkdir()
    if parent_is_git:
        command(parent, 'init', '-q')
        command(parent, '-c', 'user.name=Parent Fixture', '-c', 'user.email=parent@wheelchair.invalid',
                'commit', '--allow-empty', '-qm', 'unrelated project')
    exported = export_fixture(reviewed, parent)
    before = {p: p.read_bytes() for p in exported[0].rglob('*') if p.is_file()}
    result = verify(exported)
    assert result['status'] == 'VERIFIED_EXPORTED_SNAPSHOT'
    assert result['git_checkout_verified'] is False and result['sdk_head'] is None
    assert result['sdk_commit_semantics'] == 'reviewed_manifest_source_origin'
    assert result['reverse_patch_check'] == 'PASS'
    assert not (exported[0].parent.parent/'.git').exists()
    assert before == {p: p.read_bytes() for p in before}


@pytest.mark.parametrize('index', [3, 4, 5])
def test_export_source_header_library_mutations_reject(reviewed, tmp_path, index):
    exported = export_fixture(reviewed, tmp_path/'parent-project')
    exported[index].write_bytes(exported[index].read_bytes()+b'MUTATED')
    with pytest.raises(RuntimeError, match='content mismatch'):
        verify(exported)


def test_export_reverse_check_cannot_silently_skip_paths_in_parent_repo(reviewed, tmp_path):
    parent = tmp_path/'parent-project'
    parent.mkdir()
    command(parent, 'init', '-q')
    exported = export_fixture(reviewed, parent)
    exported[3].write_bytes(b'int reviewed_guard = 0;\n')
    data = json.loads(exported[1].read_text())
    data['files']['transport.cpp']['sha256'] = checker.checksum(exported[3], True)
    exported[1].write_text(json.dumps(data))
    # Hashes now match this synthetic fixture, but reverse application must
    # still fail. A parent-repo prefix skip would incorrectly return zero.
    with pytest.raises(RuntimeError, match='git check failed'):
        verify(exported)


def test_export_ignores_inherited_git_dir_and_worktree_override(reviewed, tmp_path, monkeypatch):
    exported = export_fixture(reviewed, tmp_path/'parent-project')
    monkeypatch.setenv('GIT_DIR', str(reviewed[0].parent.parent/'.git'))
    monkeypatch.setenv('GIT_WORK_TREE', str(reviewed[0].parent.parent))
    result = verify(exported)
    assert result['status'] == 'VERIFIED_EXPORTED_SNAPSHOT'
    assert result['git_checkout_verified'] is False


def test_invalid_existing_git_marker_does_not_fall_back_to_export(reviewed, tmp_path):
    exported = export_fixture(reviewed, tmp_path/'parent-project')
    (exported[0].parent.parent/'.git').write_text('gitdir: /nonexistent-synthetic-git-metadata\n')
    with pytest.raises(RuntimeError, match='git check failed'):
        verify(exported)


def test_crlf_export_reverse_check_uses_only_normalized_temporary_copy(reviewed, tmp_path):
    parent = tmp_path/'parent-project'
    parent.mkdir()
    command(parent, 'init', '-q')
    exported = export_fixture(reviewed, parent)
    for path in exported[0].rglob('*'):
        if path.is_file() and checker.mode(path) == 'text_crlf_to_lf':
            path.write_bytes(path.read_bytes().replace(b'\r\n', b'\n').replace(b'\n', b'\r\n'))
    with pytest.raises(RuntimeError, match='git check failed'):
        checker.git(exported[0].parent.parent, 'apply', '--reverse', '--check', str(exported[2]))
    before = {p: p.read_bytes() for p in exported[0].rglob('*') if p.is_file()}
    result = verify(exported)
    assert result['status'] == 'VERIFIED_EXPORTED_SNAPSHOT'
    assert result['reverse_patch_check_mode'] == 'CRLF_TO_LF_COPY'
    assert result['temporary_copy_removed'] is True
    assert not list(exported[1].parent.glob('.xt-vendor-check-*'))
    assert before == {p: p.read_bytes() for p in before}


@pytest.mark.parametrize('forge_fixture_hash', [False, True])
def test_unapplied_crlf_export_rejects_before_or_during_reverse_check(reviewed, tmp_path, forge_fixture_hash):
    exported = export_fixture(reviewed, tmp_path/'parent-project')
    exported[3].write_bytes(b'int reviewed_guard = 0;\r\n')
    if forge_fixture_hash:
        data = json.loads(exported[1].read_text())
        data['files']['transport.cpp']['sha256'] = checker.checksum(exported[3], True)
        exported[1].write_text(json.dumps(data))
    before = exported[3].read_bytes()
    reason = 'git check failed' if forge_fixture_hash else 'content mismatch'
    with pytest.raises(RuntimeError, match=reason):
        verify(exported)
    assert exported[3].read_bytes() == before
    assert not list(exported[1].parent.glob('.xt-vendor-check-*'))
