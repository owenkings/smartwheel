"""Bounded source-side evidence, independent of a ROS recorder's subscriber.

SourceJournal records successful acquisitions before publication. A synchronized
summary is written only after its FIFO is drained. Missing summaries are UNKNOWN,
not evidence that no source data existed. CameraArchive stores decoded BGR8,
before preview rotation; these bytes are not raw UVC/MJPEG packets.
"""
import hashlib
import json
import os
from pathlib import Path
import queue
import threading
import time
import zlib


MAX_CAMERA_DECODED_BYTES = 64 * 1024**2


def read_camera_frame(directory, row, *, max_decoded_bytes=MAX_CAMERA_DECODED_BYTES):
    """Read v1 BGR or v2 lossless zlib with bounded decoding and both byte hashes."""
    if type(max_decoded_bytes) is not int or not 0 < max_decoded_bytes <= MAX_CAMERA_DECODED_BYTES:
        raise ValueError('invalid camera decode limit')
    width, height = row['width'], row['height']
    if any(type(value) is not int or value <= 0 for value in (width, height)):
        raise ValueError('camera dimensions must be positive integers')
    expected = width * height * 3
    if expected > max_decoded_bytes:
        raise ValueError('camera decoded size exceeds bound')
    offset, length = row['offset'], row['length']
    if (type(offset) is not int or offset < 0 or type(length) is not int
            or not 0 < length <= max_decoded_bytes + 65536):
        raise ValueError('camera stored block exceeds bound or has invalid offset')
    root = Path(directory).resolve()
    if not root.is_dir():
        raise ValueError('camera archive directory absent')
    relative = Path(row['file'])
    path = root / relative
    if relative.is_absolute() or '..' in relative.parts or path.is_symlink() or not path.resolve().is_relative_to(root):
        raise ValueError('camera chunk outside archive')
    if not path.is_file():
        raise ValueError('camera chunk absent')
    with path.open('rb') as stream:
        stream.seek(offset)
        stored = stream.read(length)
    if len(stored) != length:
        raise ValueError('camera stored block truncated')
    compression = row.get('compression', 'none')
    schema = row.get('schema_version', 1)
    if type(schema) is not int or schema not in (1,2):
        raise ValueError('unsupported camera schema')
    if row.get('schema_version') == 2:
        if row.get('uncompressed_length')!=expected or hashlib.sha256(stored).hexdigest()!=row.get('stored_sha256'):
            raise ValueError('camera v2 stored hash or decoded size metadata differs')
    if compression == 'none':
        decoded = stored
    elif compression == 'zlib':
        if row.get('schema_version') != 2 or row.get('uncompressed_length') != expected:
            raise ValueError('camera v2 decoded size metadata differs')
        if hashlib.sha256(stored).hexdigest() != row.get('stored_sha256'):
            raise ValueError('camera stored hash mismatch')
        decoder = zlib.decompressobj()
        try:
            decoded = decoder.decompress(stored, expected + 1)
        except zlib.error as error:
            raise ValueError('camera zlib stream invalid') from error
        if len(decoded) != expected or not decoder.eof or decoder.unconsumed_tail or decoder.unused_data:
            raise ValueError('camera compressed block truncated, oversized or contains trailing data')
    else:
        raise ValueError('unsupported camera compression')
    if len(decoded) != expected or hashlib.sha256(decoded).hexdigest() != row['payload_sha256']:
        raise ValueError('camera decoded bytes differ')
    return decoded


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_name(path.name + '.tmp')
    with temp.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)
    if os.name == 'posix':
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


class SourceJournal:
    BATCH_MAX_ITEMS = 128
    BATCH_MAX_BYTES = 256 * 1024

    class _Queue(queue.Queue):
        """Measure the actual FIFO high-water marks under Queue's own mutex."""
        def __init__(self, max_items):
            super().__init__(maxsize=max_items)
            self.high_watermark_items = self.high_watermark_bytes = self.queued_bytes = 0

        def _put(self, item):
            super()._put(item)
            self.queued_bytes += item[2]
            self.high_watermark_items = max(self.high_watermark_items, len(self.queue))
            self.high_watermark_bytes = max(self.high_watermark_bytes, self.queued_bytes)

        def _get(self):
            item = super()._get()
            self.queued_bytes -= item[2]
            return item

    def __init__(self, root, source_id, *, max_items=4096):
        if type(max_items) is not int or max_items <= 0:
            raise ValueError('source journal requires a positive bounded queue size')
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.source_id = str(source_id)
        self.queue = self._Queue(max_items)
        self.error = None
        self.accepted = self.persisted = self.rejected = 0
        self.counts = {}
        self.write_batch_attempts = self.write_batches_completed = 0
        self.write_batch_total_ns = self.write_batch_max_ns = 0
        self.write_batch_max_items = self.write_batch_max_bytes = 0
        self.closing = False
        self.thread = threading.Thread(target=self._run, name='source-evidence', daemon=True)
        self.thread.start()

    def check(self):
        if self.error:
            raise RuntimeError('SOURCE_ARCHIVE_FAILED: ' + self.error)
        if self.closing:
            raise RuntimeError('source archive already closing')

    def append(self, row):
        self.check()
        encoded = json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n'
        try:
            # Cache only metadata already known by the producer. Re-parsing
            # every encoded record in the disk thread adds CPU/GIL contention.
            self.queue.put_nowait((encoded, row.get('event', 'frame'), len(encoded.encode('utf-8'))))
        except queue.Full:
            self.rejected += 1
            self.error = 'SOURCE_JOURNAL_QUEUE_FULL'
            raise RuntimeError(self.error)
        self.accepted += 1

    def _run(self):
        try:
            with (self.root / 'events.jsonl').open('x', encoding='utf-8') as stream:
                pending = None
                while True:
                    if pending is not None:
                        row, pending = pending, None
                    else:
                        try:
                            row = self.queue.get(timeout=.05)
                        except queue.Empty:
                            if self.closing:
                                break
                            continue
                    batch, batch_bytes = [row], row[2]
                    while len(batch) < self.BATCH_MAX_ITEMS and batch_bytes < self.BATCH_MAX_BYTES:
                        try:
                            row = self.queue.get_nowait()
                        except queue.Empty:
                            break
                        if batch_bytes + row[2] > self.BATCH_MAX_BYTES:
                            pending = row  # One bounded lookahead preserves exact FIFO order.
                            break
                        batch.append(row)
                        batch_bytes += row[2]
                    encoded = ''.join(row[0] for row in batch)
                    self.write_batch_attempts += 1
                    self.write_batch_max_items = max(self.write_batch_max_items, len(batch))
                    self.write_batch_max_bytes = max(self.write_batch_max_bytes, batch_bytes)
                    started = time.monotonic_ns()
                    try:
                        if stream.write(encoded) != len(encoded):
                            raise OSError('source journal short write')
                    finally:
                        elapsed = max(0, time.monotonic_ns() - started)
                        self.write_batch_total_ns += elapsed
                        self.write_batch_max_ns = max(self.write_batch_max_ns, elapsed)
                    self.write_batches_completed += 1
                    for _, kind, _ in batch:
                        self.counts[kind] = self.counts.get(kind, 0) + 1
                    self.persisted += len(batch)
                stream.flush()
                os.fsync(stream.fileno())
        except Exception as error:
            self.error = str(error)

    def close(self, *, source_error=None, timeout=20.):
        self.closing = True
        self.thread.join(timeout)
        if self.thread.is_alive():
            self.error = 'SOURCE_JOURNAL_CLOSE_TIMEOUT'
        value = {'schema_version': 1, 'source_id': self.source_id,
                 'accepted_records': self.accepted, 'persisted_records': self.persisted,
                 'rejected_records': self.rejected, 'event_counts': self.counts,
                 'error': self.error, 'source_error': source_error,
                 'synchronized': not self.error and self.accepted == self.persisted,
                 'closed_normally': not self.error and not source_error,
                  'closed_wall_ns': time.time_ns(),
                  'journal_writer': {'queue_max_items': self.queue.maxsize,
                      'queue_high_watermark_items': self.queue.high_watermark_items,
                      'queue_high_watermark_bytes': self.queue.high_watermark_bytes,
                      'queue_remaining_items': self.queue.qsize(),
                      'batch_max_items': self.BATCH_MAX_ITEMS, 'batch_target_bytes': self.BATCH_MAX_BYTES,
                      'write_batch_attempts': self.write_batch_attempts,
                      'write_batches_completed': self.write_batches_completed,
                      'write_batch_max_items': self.write_batch_max_items,
                      'write_batch_max_bytes': self.write_batch_max_bytes,
                      'write_batch_total_ns': self.write_batch_total_ns,
                      'write_batch_max_ns': self.write_batch_max_ns,
                      'timing_scope': 'buffered stream.write only; final flush/fsync excluded',
                      'oversized_record_policy': 'single record unchanged; byte target may be exceeded'}}
        path = self.root / 'events.jsonl'
        if path.is_file() and not self.thread.is_alive():
            value['events_sha256'] = digest(path)
        atomic_json(self.root / 'summary.json', value)
        if self.error:
            raise RuntimeError(self.error)
        return value


class CameraArchive:
    """Lossless BGR blocks with bounded ingestion and a close-time durability barrier.

    Rotation only flushes/closes the buffered file; it does not claim durability.
    Chunk/index SHA256 is accumulated over the exact written bytes, without disk
    readback. Only after acquisition stops and the FIFO drains do we fsync every
    chunk and the index. A complete summary is committed after that barrier.
    """
    class _Queue(queue.Queue):
        def __init__(self, max_items):
            super().__init__(maxsize=max_items)
            self.high_watermark_items = self.high_watermark_bytes = 0
            self.queued_bytes = self.maximum_frame_bytes = 0

        def _put(self, item):
            super()._put(item)
            self.queued_bytes += len(item[0])
            self.maximum_frame_bytes = max(self.maximum_frame_bytes, len(item[0]))
            self.high_watermark_items = max(self.high_watermark_items, len(self.queue))
            self.high_watermark_bytes = max(self.high_watermark_bytes, self.queued_bytes)

        def _get(self):
            item = super()._get()
            self.queued_bytes -= len(item[0])
            return item

    def __init__(self, root, source_id, epoch, *, max_items=256, chunk_bytes=8 * 1024**2, compression='zlib'):
        if type(max_items) is not int or max_items <= 0:
            raise ValueError('camera archive requires a positive bounded queue size')
        if type(chunk_bytes) is not int or chunk_bytes <= 0:
            raise ValueError('camera archive requires a positive chunk size')
        if compression not in ('zlib', 'none'):
            raise ValueError('camera compression must be zlib or none')
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.source_id, self.epoch = source_id, epoch
        self.chunk_bytes = chunk_bytes
        self.compression = compression
        self.uncompressed_bytes = self.stored_bytes = 0
        self.compression_total_ns = self.compression_max_ns = 0
        # At 320x240 BGR8/30 Hz, 256 payloads use 56.25 MiB and cover
        # about 8.5 s of a transient disk stall. Sustained overload still fails.
        self.queue = self._Queue(max_items)
        self.closing = False
        self.error = None
        self.received = self.persisted = self.dropped = 0
        self.chunks = []
        self.index_sha256 = None
        self.chunks_synchronized = 0
        self.index_synchronized = self.final_fsync_complete = False
        self._close_value = None
        self.thread = threading.Thread(target=self._run, daemon=True, name='camera-archive')
        self.thread.start()

    def _fail(self, reason):
        # Keep the producer's first overflow/size failure even if draining later
        # encounters another I/O error. Either failure prevents COMPLETE.
        if self.error is None:
            self.error = str(reason)

    def append(self, data, **metadata):
        self.received += 1
        if self.closing or self.error:
            self.dropped += 1
            raise RuntimeError(self.error or 'camera archive closed')
        if (any(type(metadata.get(key)) is not int or metadata[key] <= 0 for key in ('width','height'))
                or len(data) != metadata['width'] * metadata['height'] * 3
                or len(data) > MAX_CAMERA_DECODED_BYTES):
            self.dropped += 1
            self._fail('BGR_SIZE_MISMATCH')
            raise ValueError(self.error)
        try:
            self.queue.put_nowait((bytes(data), dict(metadata)))
        except queue.Full:
            self.dropped += 1
            self._fail('CAMERA_ARCHIVE_QUEUE_FULL')
            raise RuntimeError(self.error)

    def _run(self):
        chunk = None
        chunk_name = None
        chunk_hash = None
        index_hash = hashlib.sha256()
        def finish_chunk():
            nonlocal chunk
            size = chunk.tell()
            chunk.flush()
            chunk.close()
            chunk = None
            self.chunks.append({'path': chunk_name, 'sha256': chunk_hash.hexdigest(), 'bytes': size})
        try:
            # Binary output makes the streaming digest cover the exact UTF-8
            # bytes on Linux and Windows, including the final newline.
            with (self.root / 'frames.jsonl').open('xb') as index:
                while True:
                    try:
                        data, metadata = self.queue.get(timeout=.05)
                    except queue.Empty:
                        if self.closing:
                            break
                        continue
                    started = time.monotonic_ns()
                    stored = zlib.compress(data, level=1) if self.compression == 'zlib' else data
                    elapsed = max(0, time.monotonic_ns() - started)
                    self.compression_total_ns += elapsed
                    self.compression_max_ns = max(self.compression_max_ns, elapsed)
                    if chunk is None or chunk.tell() + len(stored) > self.chunk_bytes:
                        if chunk is not None:
                            finish_chunk()
                        chunk_name = 'frames-%06d.bgr' % len(self.chunks)
                        chunk = (self.root / chunk_name).open('xb')
                        chunk_hash = hashlib.sha256()
                    offset = chunk.tell()
                    if chunk.write(stored) != len(stored):
                        raise OSError('camera chunk short write')
                    chunk_hash.update(stored)
                    row = dict(metadata, source_id=self.source_id, stream_epoch=self.epoch,
                               representation='DECODED_BGR8_BEFORE_PREVIEW_ROTATION',
                                schema_version=2, compression=self.compression,
                                compression_level=1 if self.compression == 'zlib' else None,
                                file=chunk_name, offset=offset, length=len(stored),
                                uncompressed_length=len(data), stored_sha256=hashlib.sha256(stored).hexdigest(),
                                payload_sha256=hashlib.sha256(data).hexdigest())
                    encoded = (json.dumps(row, allow_nan=False) + '\n').encode('utf-8')
                    if index.write(encoded) != len(encoded):
                        raise OSError('camera index short write')
                    index_hash.update(encoded)
                    self.persisted += 1
                    self.uncompressed_bytes += len(data)
                    self.stored_bytes += len(stored)
                if chunk is not None:
                    finish_chunk()
                index.flush()
            self.index_sha256 = index_hash.hexdigest()
            # Acquisition is stopped before close() sets closing. No fsync or
            # digest readback stalls the live writer at a chunk boundary.
            for row in self.chunks:
                with (self.root / row['path']).open('r+b') as stream:
                    os.fsync(stream.fileno())
                self.chunks_synchronized += 1
            with (self.root / 'frames.jsonl').open('r+b') as stream:
                os.fsync(stream.fileno())
            self.index_synchronized = self.final_fsync_complete = True
        except Exception as error:
            self._fail(error)
        finally:
            if chunk is not None:
                try:
                    chunk.close()
                except Exception as error:
                    self._fail(error)

    def close(self, source_error=None, timeout=20.):
        if self._close_value is not None:
            if self.error:
                raise RuntimeError(self.error)
            return dict(self._close_value)
        self.closing = True
        self.thread.join(timeout)
        if self.thread.is_alive():
            self._fail('CAMERA_ARCHIVE_CLOSE_TIMEOUT')
            # The writer may still own a chunk/index or be blocked in fsync.
            # Never commit a summary over mutable files from a timed-out worker.
            raise RuntimeError(self.error)
        synchronized = (self.final_fsync_complete and not self.error
                        and self.received == self.persisted and self.dropped == 0)
        value = {'schema_version': 2, 'source_id': self.source_id, 'stream_epoch': self.epoch,
                 'compression': self.compression, 'compression_level': 1 if self.compression == 'zlib' else None,
                 'uncompressed_bytes': self.uncompressed_bytes, 'stored_bytes': self.stored_bytes,
                 'compression_total_ns': self.compression_total_ns, 'compression_max_ns': self.compression_max_ns,
                 'representation': 'DECODED_BGR8_BEFORE_PREVIEW_ROTATION',
                 'captured_frames': self.received, 'persisted_frames': self.persisted,
                 'dropped_frames': self.dropped, 'error': self.error, 'source_error': source_error,
                 'chunks': [dict(row) for row in self.chunks], 'exposure_completeness_verified': False,
                 'synchronized': synchronized,
                 'closed_normally': synchronized and not source_error,
                 'status': 'COMPLETE' if synchronized and not source_error else 'PARTIAL',
                 'final_fsync_complete': self.final_fsync_complete,
                 'chunks_synchronized': self.chunks_synchronized,
                 'index_synchronized': self.index_synchronized,
                  'queue_max_items': self.queue.maxsize,
                  'estimated_queued_bytes': self.queue.maxsize * self.queue.maximum_frame_bytes,
                  'queue_high_watermark_items': self.queue.high_watermark_items,
                  'queue_high_watermark_bytes': self.queue.high_watermark_bytes,
                  'queue_byte_estimate_basis': 'maximum accepted BGR8 payload times capacity; Python metadata and in-flight writer excluded',
                 'persisted_counter_basis': 'WRITE_COMPLETED; durable only after final_fsync_complete'}
        if self.index_sha256 is not None:
            value['index_sha256'] = self.index_sha256
        if not self.final_fsync_complete:
            self._fail('CAMERA_ARCHIVE_DURABILITY_INCOMPLETE')
            value.update(error=self.error, status='PARTIAL', synchronized=False, closed_normally=False)
            # No completion marker exists when data durability failed. Preserve
            # a separate diagnostic when the filesystem still permits writing.
            try:
                atomic_json(self.root / 'partial_summary.json', value)
            except Exception:
                pass
            raise RuntimeError(self.error)
        try:
            atomic_json(self.root / 'summary.json', value)
        except Exception as error:
            self._fail('CAMERA_ARCHIVE_SUMMARY_SYNC_FAILED: ' + str(error))
            value.update(error=self.error, status='PARTIAL', synchronized=False, closed_normally=False)
            # atomic_json may already have renamed the marker before a directory
            # fsync failed. Remove our own marker rather than leave COMPLETE.
            try:
                (self.root / 'summary.json').unlink(missing_ok=True)
                atomic_json(self.root / 'partial_summary.json', value)
            except Exception:
                pass
            raise RuntimeError(self.error) from error
        self._close_value = dict(value)
        if self.error:
            raise RuntimeError(self.error)
        return value
