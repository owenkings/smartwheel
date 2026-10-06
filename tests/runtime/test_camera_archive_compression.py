"""Lossless v2, bounded corrupt-stream decoding and retained v1 compatibility."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import uuid
import zlib

import pytest

from wc_runtime.source_archive import CameraArchive, MAX_CAMERA_DECODED_BYTES, read_camera_frame


@pytest.fixture
def compression_root():
    parent=Path(__file__).resolve().parent
    root=parent/('.compression_test_'+uuid.uuid4().hex)
    root.mkdir()
    try: yield root
    finally:
        assert root.resolve().parent==parent and root.name.startswith('.compression_test_')
        shutil.rmtree(root)


def test_v2_zero_and_noise_frames_roundtrip_original_bytes_and_hashes(compression_root):
    archive=CameraArchive(compression_root/'camera','left_front','synthetic-42')
    frames=[bytes(320*240*3),os.urandom(320*240*3)]
    for sequence,data in enumerate(frames,1):
        archive.append(data,width=320,height=240,sequence=sequence,host_monotonic_ns=sequence*1000)
    summary=archive.close()
    rows=[json.loads(line) for line in (archive.root/'frames.jsonl').read_text(encoding='utf-8').splitlines()]
    assert summary['schema_version']==2 and summary['compression']=='zlib' and summary['compression_level']==1
    assert summary['uncompressed_bytes']==sum(map(len,frames))
    assert summary['stored_bytes']==sum(row['length'] for row in rows)
    assert rows[0]['length']<len(frames[0])//10
    assert rows[1]['length']>=len(frames[1])  # Incompressible input remains lossless.
    for row,data in zip(rows,frames):
        assert row['uncompressed_length']==len(data)
        assert row['payload_sha256']==hashlib.sha256(data).hexdigest()
        assert read_camera_frame(archive.root,row)==data
    assert summary['captured_frames']==summary['persisted_frames']==2
    assert summary['final_fsync_complete'] and summary['status']=='COMPLETE'


def legacy_row(root):
    data=b'abcdefghijkl'
    (root/'legacy.bgr').write_bytes(b'prefix'+data+b'suffix')
    return data,dict(file='legacy.bgr',offset=6,length=len(data),width=2,height=2,
                     payload_sha256=hashlib.sha256(data).hexdigest())


def test_v1_uncompressed_archive_still_reads(compression_root):
    data,row=legacy_row(compression_root)
    assert read_camera_frame(compression_root,row)==data


def compressed_row(root, stored=None):
    data=b'abcdefghijkl'
    stored=zlib.compress(data,1) if stored is None else stored
    (root/'compressed.bgr').write_bytes(stored)
    return data,dict(schema_version=2,compression='zlib',compression_level=1,
        uncompressed_length=len(data),file='compressed.bgr',offset=0,length=len(stored),width=2,height=2,
        stored_sha256=hashlib.sha256(stored).hexdigest(),payload_sha256=hashlib.sha256(data).hexdigest())


@pytest.mark.parametrize('kind',['stored_hash','decoded_hash','truncated','trailing','bomb','size_metadata','unknown_codec'])
def test_corrupt_v2_or_unbounded_decode_is_rejected(compression_root,kind):
    data,row=compressed_row(compression_root)
    if kind=='stored_hash': row['stored_sha256']='0'*64
    elif kind=='decoded_hash': row['payload_sha256']='0'*64
    elif kind in ('truncated','trailing','bomb'):
        stored=(compression_root/'compressed.bgr').read_bytes()
        if kind=='truncated': stored=stored[:-1]
        elif kind=='trailing': stored+=b'not-another-frame'
        else: stored=zlib.compress(bytes(1024*1024),1)
        _,row=compressed_row(compression_root,stored)
    elif kind=='size_metadata': row['uncompressed_length']+=1
    else: row['compression']='unsupported'
    with pytest.raises(ValueError): read_camera_frame(compression_root,row)


@pytest.mark.parametrize('patch',[
    dict(width=True),dict(height=-1),dict(width=MAX_CAMERA_DECODED_BYTES,height=2),
    dict(offset=-1),dict(length=MAX_CAMERA_DECODED_BYTES+65537),dict(file='../outside.bgr')])
def test_invalid_metadata_rejected_before_large_read(compression_root,patch):
    _,row=compressed_row(compression_root)
    row.update(patch)
    with pytest.raises(ValueError): read_camera_frame(compression_root,row)


def test_unknown_schema_and_wrong_v1_hash_are_rejected(compression_root):
    _,row=legacy_row(compression_root)
    row['schema_version']=99
    with pytest.raises(ValueError): read_camera_frame(compression_root,row)
    row['schema_version']=1;row['payload_sha256']='0'*64
    with pytest.raises(ValueError): read_camera_frame(compression_root,row)
