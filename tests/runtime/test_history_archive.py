import json
import lzma
import hashlib
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from wc_runtime import history_archive as archive


@pytest.mark.parametrize('pointer', ['state/current.json', '.phase1_runtime/state/current.json',
                                    'config/current.json'])
def test_review_refuses_session_referenced_by_current_or_legacy_pointer(tmp_path,monkeypatch,pointer):
    session=tmp_path/'reports/maps/closed';session.mkdir(parents=True)
    source=session/'bag.db3';source.write_bytes(b'closed recording')
    closure=session/'closed.json';closure.write_text('{}')
    review=dict(session='reports/maps/closed',closure_evidence=[dict(path='reports/maps/closed/closed.json',
        sha256=archive.digest(closure))],files=[dict(path='reports/maps/closed/bag.db3',
        sha256=archive.digest(source),inode=source.stat().st_ino)])
    path=tmp_path/pointer;path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps({'session':str(session)}))
    with pytest.raises(ValueError,match='active configuration references'):
        archive.validate_review(tmp_path,review)


@pytest.mark.parametrize('state', ['state', '.phase1_runtime/state'])
def test_superseded_pointer_migration_receipt_does_not_block_closed_session(tmp_path,monkeypatch,state):
    session=tmp_path/'reports/maps/closed';session.mkdir(parents=True)
    source=session/'bag.db3';source.write_bytes(b'closed recording')
    closure=session/'closed.json';closure.write_text('{}')
    review=dict(session='reports/maps/closed',closure_evidence=[dict(path='reports/maps/closed/closed.json',
        sha256=archive.digest(closure))],files=[dict(path='reports/maps/closed/bag.db3',
        sha256=archive.digest(source),inode=source.stat().st_ino)])
    path=tmp_path/state/'pointer_migrations/previous.json'
    path.parent.mkdir(parents=True);path.write_text(json.dumps({'previous_session':str(session)}))
    monkeypatch.setattr(archive,'Path',lambda *parts: SimpleNamespace(iterdir=lambda:iter(()))
                        if parts==('/proc',) else Path(*parts))
    assert archive.validate_review(tmp_path,review)==review['files']


def test_verified_round_trip_retains_receipt_and_never_overwrites(tmp_path, monkeypatch):
    root = tmp_path
    source = root/'reports/maps/closed/bag/bag_0.db3'
    source.parent.mkdir(parents=True)
    payload = b'original sensor transaction\x00\xff' * 70000
    source.write_bytes(payload)
    receipts = root/'receipts'
    receipts.mkdir()
    monkeypatch.setattr(archive, 'open_project_files', lambda _: set())
    record = archive.archive_one(root, archive.candidates(root)[0], receipts)
    assert record['status'] == 'ARCHIVED'
    assert not source.exists()
    assert archive.hash_decoded(root/record['archive_path']) == (record['original_sha256'], len(payload))
    archive.restore(root, record)
    assert source.read_bytes() == payload
    with pytest.raises(ValueError, match='never overwrites'):
        archive.restore(root, record)


def test_changed_or_open_original_is_retained(tmp_path, monkeypatch):
    source = tmp_path/'reports/maps/closed/bag.db3'
    source.parent.mkdir(parents=True)
    source.write_bytes(b'a' * 1200000)
    row = archive.candidates(tmp_path)[0]
    source.write_bytes(b'b' * 1300000)
    with pytest.raises(ValueError, match='changed since inventory'):
        archive.archive_one(tmp_path, row, tmp_path/'receipts')
    monkeypatch.setattr(archive, 'open_project_files', lambda _: {str(source)})
    with pytest.raises(ValueError, match='currently open'):
        archive.archive_one(tmp_path, archive.candidates(tmp_path)[0], tmp_path/'receipts')
    assert source.read_bytes() == b'b' * 1300000


def test_failed_decode_verification_never_deletes_original(tmp_path, monkeypatch):
    source = tmp_path/'reports/maps/closed/bag.db3'
    source.parent.mkdir(parents=True)
    source.write_bytes(b'a' * 1200000)
    monkeypatch.setattr(archive, 'open_project_files', lambda _: set())
    monkeypatch.setattr(archive, 'hash_decoded', lambda _: ('bad', 1200000))
    with pytest.raises(ValueError, match='round trip differs'):
        archive.archive_one(tmp_path, archive.candidates(tmp_path)[0], tmp_path/'receipts')
    assert source.exists()


def test_path_escape_and_symlink_rejected(tmp_path):
    with pytest.raises(ValueError):
        archive.checked(tmp_path, '../outside')
    (tmp_path/'link').symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError, match='symlink'):
        archive.checked(tmp_path, 'link/payload')


@pytest.mark.parametrize('complete',[True,False])
def test_recover_interrupted_compression_requires_full_original_hash(tmp_path,monkeypatch,complete):
    source=tmp_path/'reports/maps/closed/bag.db3'
    source.parent.mkdir(parents=True);payload=b'raw sensor record'*100000;source.write_bytes(payload)
    row=archive.candidates(tmp_path)[0]
    temporary=tmp_path/('data/history_archive/'+row['path']+'.xz.partial')
    temporary.parent.mkdir(parents=True)
    compressed=lzma.compress(payload)
    temporary.write_bytes(compressed if complete else compressed[:len(compressed)//2])
    receipts=tmp_path/'receipts';receipts.mkdir()
    monkeypatch.setattr(archive,'open_project_files',lambda _:set())
    if complete:
        result=archive.archive_one(tmp_path,row,receipts)
        assert result['recovered_verified_partial'] and result['status']=='ARCHIVED'
        assert archive.hash_decoded(tmp_path/result['archive_path'])==(result['original_sha256'],len(payload))
    else:
        with pytest.raises((lzma.LZMAError,EOFError)):
            archive.archive_one(tmp_path,row,receipts)
        assert source.read_bytes()==payload and temporary.exists()


def retained_archive(tmp_path, monkeypatch):
    """A persisted receipt, without invoking the compressor or any devices."""
    source = tmp_path/'reports/maps/closed/bag.db3'
    source.parent.mkdir(parents=True)
    payload = b'reviewed raw sensor record\x00\xff' * 50000
    source.write_bytes(payload)
    info = source.stat()
    row = dict(path=source.relative_to(tmp_path).as_posix(), bytes=info.st_size,
               mtime_ns=info.st_mtime_ns, inode=info.st_ino, device=info.st_dev,
               nlink=info.st_nlink, sha256=archive.digest(source))
    target = tmp_path/('data/history_archive/'+row['path']+'.xz')
    target.parent.mkdir(parents=True)
    target.write_bytes(lzma.compress(payload))
    receipts = tmp_path/'receipts'
    receipts.mkdir()
    receipt = receipts/(hashlib.sha256(row['path'].encode()).hexdigest()+'.json')
    record = dict(status='VERIFIED_ORIGINAL_RETAINED', path=row['path'],
                  archive_path=target.relative_to(tmp_path).as_posix(), codec='xz',
                  original_bytes=row['bytes'], original_sha256=row['sha256'],
                  original_mtime_ns=row['mtime_ns'], original_mode=0o600,
                  archive_bytes=target.stat().st_size, archive_sha256=archive.digest(target))
    archive.atomic_json(receipt, record)
    monkeypatch.setattr(archive, 'open_project_files', lambda _: set())
    if os.name != 'posix':
        # Directory fsync is tested on Linux; these cases test receipt identity.
        monkeypatch.setattr(archive, 'sync_dir', lambda _: None)
    return source, row, target, receipts, receipt, record, payload


@pytest.mark.parametrize('damage', ['missing', 'bytes', 'receipt_path',
                                  'receipt_archive_path', 'review_hash', 'decoded_hash'])
def test_archived_resume_reverifies_archive_and_review_binding(tmp_path, monkeypatch, damage):
    source, row, target, receipts, receipt, record, payload = retained_archive(tmp_path, monkeypatch)
    source.unlink()
    record['status'] = 'ARCHIVED'
    if damage == 'missing':
        target.unlink()
    elif damage == 'bytes':
        target.write_bytes(b'corrupt archived bytes')
    elif damage == 'receipt_path':
        record['path'] = 'reports/maps/another/bag.db3'
    elif damage == 'receipt_archive_path':
        record['archive_path'] = 'data/history_archive/unrelated.xz'
    elif damage == 'review_hash':
        row['sha256'] = '0' * 64
    elif damage == 'decoded_hash':
        target.write_bytes(lzma.compress(b'x' * len(payload)))
        record.update(archive_bytes=target.stat().st_size, archive_sha256=archive.digest(target))
    archive.atomic_json(receipt, record)
    with pytest.raises(ValueError):
        archive.archive_one(tmp_path, row, receipts)
    assert not source.exists()
    assert json.loads(receipt.read_text()) == record


def test_archived_resume_returns_only_current_verified_replacement(tmp_path, monkeypatch):
    source, row, target, receipts, receipt, record, _ = retained_archive(tmp_path, monkeypatch)
    source.unlink()
    record['status'] = 'ARCHIVED'
    archive.atomic_json(receipt, record)
    assert archive.archive_one(tmp_path, row, receipts) == record
    assert archive.digest(target) == record['archive_sha256']


def test_retained_resume_preserves_same_bytes_with_unapproved_inode(tmp_path, monkeypatch):
    source, row, _, receipts, receipt, record, payload = retained_archive(tmp_path, monkeypatch)
    source.rename(source.with_suffix('.prior'))
    source.write_bytes(payload)
    os.utime(source, ns=(row['mtime_ns'], row['mtime_ns']))
    assert source.stat().st_ino != row['inode']
    with pytest.raises(ValueError, match='identity differs'):
        archive.archive_one(tmp_path, row, receipts)
    assert source.read_bytes() == payload
    assert json.loads(receipt.read_text()) == record


def test_retained_resume_receipt_must_match_current_review_hash(tmp_path, monkeypatch):
    source, row, _, receipts, receipt, record, payload = retained_archive(tmp_path, monkeypatch)
    row['sha256'] = '0' * 64
    with pytest.raises(ValueError, match='receipt differs'):
        archive.archive_one(tmp_path, row, receipts)
    assert source.read_bytes() == payload
    assert json.loads(receipt.read_text()) == record


def test_retained_resume_rechecks_identity_after_final_guard(tmp_path, monkeypatch):
    source, row, _, receipts, receipt, record, payload = retained_archive(tmp_path, monkeypatch)
    calls = 0
    def guard():
        nonlocal calls
        calls += 1
        if calls == 2:
            source.rename(source.with_suffix('.prior'))
            source.write_bytes(payload)
            os.utime(source, ns=(row['mtime_ns'], row['mtime_ns']))
    with pytest.raises(ValueError, match='identity differs'):
        archive.archive_one(tmp_path, row, receipts, guard=guard)
    assert calls == 2 and source.read_bytes() == payload
    assert json.loads(receipt.read_text()) == record


@pytest.mark.parametrize('compress_only', [True, False])
def test_valid_retained_resume_preserves_or_removes_exact_reviewed_source(tmp_path, monkeypatch, compress_only):
    source, row, target, receipts, receipt, record, _ = retained_archive(tmp_path, monkeypatch)
    result = archive.archive_one(tmp_path, row, receipts, compress_only=compress_only)
    assert source.exists() == compress_only
    assert result['status'] == ('VERIFIED_ORIGINAL_RETAINED' if compress_only else 'ARCHIVED')
    assert archive.digest(target) == record['archive_sha256']
    assert json.loads(receipt.read_text()) == result


def test_unreceipted_target_retains_original_and_requires_manual_recovery_review(tmp_path, monkeypatch):
    source, row, target, receipts, receipt, _, payload = retained_archive(tmp_path, monkeypatch)
    receipt.unlink()
    with pytest.raises(ValueError, match='without matching receipt; recovery review required'):
        archive.archive_one(tmp_path, row, receipts)
    assert source.read_bytes() == payload and target.exists() and not receipt.exists()
