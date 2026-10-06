"""Bounded application acknowledgements for accepted observations.

Reliable DDS can acknowledge receipt before a KEEP_LAST callback queue processes
the sample. Send one immutable observation until the native graph includes it;
queue at most eight, and latch unavailable/contradictory graph acknowledgements.
"""
from collections import deque
from dataclasses import dataclass


class GraphDeliveryError(ValueError):
    pass


@dataclass
class Pending:
    bundle_id: int
    raw_keys: tuple
    odom_epoch: str
    payload: bytes
    first_sent_ns: int | None = None
    last_sent_ns: int | None = None


class GraphDelivery:
    def __init__(self, session_id, source_mode, calibration_id, clock_models, *,
                 graph_epoch=None, capacity=8, retry_ns=500_000_000, timeout_ns=5_000_000_000):
        if type(capacity) is not int or not 1 <= capacity <= 8 or not 0 < retry_ns < timeout_ns:
            raise ValueError('invalid bounded graph delivery policy')
        self.session_id, self.source_mode, self.calibration_id = session_id, source_mode, calibration_id
        self.clock_models, self.graph_epoch = dict(clock_models), graph_epoch
        self.capacity, self.retry_ns, self.timeout_ns = capacity, retry_ns, timeout_ns
        self.queue = deque()
        self.last_enqueued_id = self.last_acked_id = self.acknowledged = self.attempts = self.retries = 0
        self.failure = None

    def fail(self, reason):
        self.failure = self.failure or reason
        raise GraphDeliveryError(self.failure)

    def ensure_capacity(self):
        if self.failure:
            raise GraphDeliveryError(self.failure)
        if len(self.queue) >= self.capacity:
            self.fail('GRAPH_DELIVERY_QUEUE_FULL')

    def submit(self, bundle_id, raw_keys, odom_epoch, payload):
        self.ensure_capacity()
        if (type(bundle_id) is not int or bundle_id <= self.last_enqueued_id or
                len(raw_keys) != 2 or len(set(raw_keys)) != 2 or not all(raw_keys) or not odom_epoch or
                type(payload) is not bytes or not 0 < len(payload) <= 8*1024*1024):
            self.fail('GRAPH_DELIVERY_INVALID_OBSERVATION')
        self.queue.append(Pending(bundle_id,tuple(raw_keys),odom_epoch,payload))
        self.last_enqueued_id = bundle_id

    def next_payload(self, now_ns):
        if self.failure:
            raise GraphDeliveryError(self.failure)
        if not self.queue:
            return None
        item = self.queue[0]
        if item.first_sent_ns is not None and now_ns-item.first_sent_ns >= self.timeout_ns:
            self.fail('GRAPH_ACK_TIMEOUT')
        if item.last_sent_ns is not None and now_ns-item.last_sent_ns < self.retry_ns:
            return None
        if item.first_sent_ns is None:
            item.first_sent_ns = now_ns
        else:
            self.retries += 1
        item.last_sent_ns = now_ns
        self.attempts += 1
        return item.payload

    def acknowledge(self, snapshot):
        if self.failure:
            raise GraphDeliveryError(self.failure)
        if (snapshot.session_id != self.session_id or snapshot.source_mode != self.source_mode or
                snapshot.calibration_id != self.calibration_id or
                snapshot.left_clock_model_id != self.clock_models['left'] or
                snapshot.right_clock_model_id != self.clock_models['right'] or
                not snapshot.raw_index_complete or not snapshot.raw_index_hash or not snapshot.graph_epoch):
            self.fail('GRAPH_ACK_PROVENANCE_MISMATCH')
        if self.graph_epoch is None:
            self.graph_epoch = snapshot.graph_epoch
        if self.graph_epoch != snapshot.graph_epoch:
            self.fail('GRAPH_ACK_EPOCH_CHANGED')
        if not self.queue:
            return False
        item = self.queue[0]
        matches = [n for n in snapshot.nodes if n.bundle_id == item.bundle_id]
        if not matches:
            if any(n.bundle_id > item.bundle_id for n in snapshot.nodes):
                self.fail('GRAPH_ACK_SKIPPED_PENDING_BUNDLE')
            return False
        if (item.first_sent_ns is None or len(matches) != 1 or matches[0].node_id != item.bundle_id or
                matches[0].odom_epoch != item.odom_epoch or
                sorted(matches[0].raw_keys) != sorted(item.raw_keys)):
            self.fail('GRAPH_ACK_OBSERVATION_MISMATCH')
        self.queue.popleft()
        self.last_acked_id = item.bundle_id
        self.acknowledged += 1
        return True

    def status(self):
        return dict(pending=len(self.queue),pending_bundle_ids=[x.bundle_id for x in self.queue],
                    acknowledged=self.acknowledged,last_acked_id=self.last_acked_id,
                    attempts=self.attempts,retries=self.retries,failure=self.failure)
