"""SYNTHETIC real input engine/archive/publication checks; no ROS or devices."""
import copy
import hashlib
import json
import threading

import numpy as np
import pytest

from wc_runtime.mapping_input import MappingInput
from wc_sensors.pointcloud import decode_pointcloud2
from test_mapping_input import START, serialize, settle, tmp_path
from test_mapping_filter import frame, settings


@pytest.fixture
def filtered_owner(tmp_path):
    owners = []
    def make(mode, publication_mode, *, preview=False):
        candidate = settings(mode)
        candidate['mapping_enabled'] = not preview
        candidate['archive_publication_mode'] = publication_mode
        candidate['archive_checkpoint_policy'] = 'on_close'
        candidate['cloud_filter'].update(organized_support={'enabled': False}, ground_height={'enabled': False})
        session = tmp_path/('filter_'+str(len(owners)))
        session.mkdir()
        owner = MappingInput(candidate, tmp_path, session/'input', started_ns=START)
        owners.append(owner)
        settle(owner, START)
        return owner
    yield make
    for owner in owners:
        owner.close()


def send_pair(owner, left, right, published):
    stamp = right.host_monotonic_ns if right is not None else left.host_monotonic_ns
    if right is not None:
        assert not owner.receive_source(left, left.host_monotonic_ns, serialize, published.append,
                                        clock=lambda: left.host_monotonic_ns)
    message = right if right is not None else left
    assert owner.receive_source(message, stamp, serialize, published.append, clock=lambda: stamp)
    settle(owner, stamp)


@pytest.mark.parametrize('mode', ['left', 'all'])
@pytest.mark.parametrize('publication_mode', ['written_before_publish', 'queued_realtime'])
def test_empty_filters_archive_exact_sources_skip_publication_and_resume(filtered_owner, mode, publication_mode):
    owner = filtered_owner(mode, publication_mode)
    published = []
    left = frame([[30, 0, 0]] if mode == 'left' else [[1, 0, 0]])
    right = frame([[30, 0, 0]], side='right', host=START+20_000_000) if mode == 'all' else None
    originals = [serialize(message) for message in (left, right) if message is not None]
    send_pair(owner, left, right, published)
    assert not published and owner.failure_reason is None
    assert owner.filtered_empty_outputs == owner.archived_outputs == 1
    owner.check_stale(START+60_000_000_000)  # Main continuous profile waits without ending.
    first = json.loads((owner.output/'pairs.jsonl').read_text())
    assert first['publication_state'] == 'FILTERED_SOURCE_EMPTY_ARCHIVED_NOT_PUBLISHED'
    assert not first['publication_eligible']
    assert first['cloud_filter']['empty_sides'] == (['right'] if mode == 'all' else ['left'])
    for row, original in zip(first['sources'], originals):
        assert (owner.output/row['cdr_file']).read_bytes() == original
        assert row['cdr_sha256'] == hashlib.sha256(original).hexdigest()
    second_host = START+61_000_000_000
    left = frame([[1, 0, 0], [30, 0, 0]], host=second_host, sequence=1)
    right = frame([[2, 0, 0], [30, 0, 0]], side='right', host=second_host+20_000_000, sequence=1) if mode == 'all' else None
    send_pair(owner, left, right, published)
    assert len(published) == owner.published_frames == 1
    assert np.isfinite(decode_pointcloud2(published[0])).all(axis=1).tolist() == [True, False]*(2 if mode == 'all' else 1)
    owner.close()
    final = json.loads((owner.output/'status.json').read_text(encoding='utf-8'))
    assert final['state'] == 'STOPPED' and final['failure_reason'] is None
    assert final['cloud_filter']['empty_outputs_skipped'] == 1
    assert final['outputs_discarded_on_close'] == 0
    assert final['archived_outputs'] == 2 and final['published_frames'] == 1
    assert final['storage']['checkpoint']['archived_outputs'] == 2
    assert final['storage']['checkpoint']['filtered_empty_outputs'] == 1
    assert final['cloud_filter']['totals_by_side']['left']['frames'] == 2
    assert final['cloud_filter']['totals_by_side']['left']['input_rows'] == 3


@pytest.mark.parametrize('publication_mode', ['written_before_publish', 'queued_realtime'])
def test_preview_keeps_selected_raw_bytes_even_if_filter_switch_requested(filtered_owner, publication_mode):
    owner = filtered_owner('left', publication_mode, preview=True)
    message = frame([[30, 0, 0], [0, 0, 0]])
    published = []
    send_pair(owner, message, None, published)
    assert published[0].data == message.cloud.data
    assert owner.filtered_empty_outputs == 0
    row = json.loads((owner.output/'pairs.jsonl').read_text())
    assert row['cloud_filter']['disabled_reason'] == 'PREVIEW_PRESERVES_SOURCE'
    assert row['output_data_sha256'] == hashlib.sha256(message.cloud.data).hexdigest()


def test_queued_filter_freezes_sources_and_output_before_mutable_publication(filtered_owner, monkeypatch):
    owner = filtered_owner('all', 'queued_realtime')
    entered, release = threading.Event(), threading.Event()
    append = owner.archives['left'].append
    def blocked(*args):
        entered.set()
        assert release.wait(3)
        return append(*args)
    monkeypatch.setattr(owner.archives['left'], 'append', blocked)
    left = frame([[1, 0, 0], [30, 0, 0]])
    right = frame([[2, 0, 0], [30, 0, 0]], side='right', host=START+20_000_000)
    originals = [serialize(left), serialize(right)]
    published = []
    try:
        assert not owner.receive_source(left, START, serialize, published.append, clock=lambda: START)
        assert owner.receive_source(right, right.host_monotonic_ns, serialize, published.append,
                                    clock=lambda: right.host_monotonic_ns)
        assert entered.wait(1) and len(published) == 1
        published_bytes = bytes(published[0].data)
        for message in (left, right):
            message.cloud.data = b'caller changed original'
        published[0].data = b'caller changed published'
        owner.filter_last_report['retained_points'] = 999  # No alias to archive job.
    finally:
        release.set()
    settle(owner, START+20_000_000)
    owner.close()
    row = json.loads((owner.output/'pairs.jsonl').read_text())
    assert row['cloud_filter']['retained_points'] == 2
    assert row['output_data_sha256'] == hashlib.sha256(published_bytes).hexdigest()
    for source, original in zip(row['sources'], originals):
        assert (owner.output/source['cdr_file']).read_bytes() == original
    assert owner.published_frames == 1


def test_empty_archive_prefix_does_not_hide_later_unarchived_publication(filtered_owner, monkeypatch):
    owner = filtered_owner('left', 'queued_realtime')
    send_pair(owner, frame([[30, 0, 0]]), None, [])
    # Certify only the rejected first output, then stall the next actual write.
    owner.checkpoints._commit(owner.checkpoints.snapshot()['written_prefix'], final=False)
    entered, release = threading.Event(), threading.Event()
    append = owner.archives['left'].append
    def blocked(*args):
        entered.set()
        assert release.wait(3)
        return append(*args)
    monkeypatch.setattr(owner.archives['left'], 'append', blocked)
    now = START+200_000_000
    published = []
    try:
        assert owner.receive_source(frame([[1, 0, 0]], host=now, sequence=1), now,
                                    serialize, published.append, clock=lambda: now)
        assert entered.wait(1) and len(published) == 1
        status = owner.status(now)
        assert status['archived_outputs'] == status['published_frames'] == 1
        assert status['published_outputs_not_confirmed_archived'] == 1
        assert status['published_outputs_not_checkpointed'] == 1
    finally:
        release.set()
    settle(owner, now)
    status = owner.status(now)
    assert status['published_outputs_not_confirmed_archived'] == 0
    assert status['published_outputs_not_checkpointed'] == 1
    owner.close()
    assert owner.status(now)['published_outputs_not_checkpointed'] == 0
