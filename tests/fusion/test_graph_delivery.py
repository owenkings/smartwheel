"""Deterministic loss/retry/ACK tests without a sensor or middleware substitute claim."""
from types import SimpleNamespace as N
import pytest
from wc_fusion.graph_delivery import GraphDelivery, GraphDeliveryError


def delivery(**kwargs):
    return GraphDelivery('session','synthetic','cal',{'left':'lc','right':'rc'},graph_epoch='graph',**kwargs)


def put(writer, identity):
    writer.submit(identity,(f'L/{identity}',f'R/{identity}'),'odom',f'immutable/{identity}'.encode())


def snapshot(*ids):
    return N(session_id='session',source_mode='synthetic',calibration_id='cal',
             left_clock_model_id='lc',right_clock_model_id='rc',graph_epoch='graph',
             raw_index_complete=True,raw_index_hash='verified-native-index',
             nodes=[N(node_id=i,bundle_id=i,odom_epoch='odom',raw_keys=[f'L/{i}',f'R/{i}']) for i in ids])


def test_lost_delivery_or_ack_retries_identical_head_before_next():
    writer=delivery()
    put(writer,1);put(writer,2)
    first=writer.next_payload(0)
    assert first==b'immutable/1'
    assert writer.next_payload(499_999_999) is None
    assert writer.next_payload(500_000_000)==first  # first delivery discarded
    assert writer.next_payload(1_000_000_000)==first  # processing ACK discarded
    assert writer.acknowledge(snapshot(1))
    assert not writer.acknowledge(snapshot(1))  # durable duplicate cannot ACK 2
    assert writer.next_payload(1_000_000_001)==b'immutable/2'
    assert writer.acknowledge(snapshot(1,2))
    assert writer.next_payload(1_100_000_000) is None
    assert writer.status()['acknowledged']==2 and writer.status()['retries']==2


def test_capacity_counts_inflight_and_never_overwrites_earlier_observation():
    writer=delivery()
    for identity in range(1,9):put(writer,identity)
    assert writer.next_payload(0)==b'immutable/1'
    with pytest.raises(GraphDeliveryError,match='QUEUE_FULL'):put(writer,9)
    assert writer.status()['pending_bundle_ids']==list(range(1,9))
    with pytest.raises(GraphDeliveryError,match='QUEUE_FULL'):writer.next_payload(2_000_000_000)


def test_no_ack_latches_at_original_bounded_deadline():
    writer=delivery();put(writer,1);writer.next_payload(0)
    assert writer.next_payload(4_999_999_999)==b'immutable/1'
    with pytest.raises(GraphDeliveryError,match='ACK_TIMEOUT'):writer.next_payload(5_000_000_000)
    with pytest.raises(GraphDeliveryError,match='ACK_TIMEOUT'):writer.acknowledge(snapshot(1))


@pytest.mark.parametrize('field,value',[
    ('session_id','foreign'),('source_mode','real'),('calibration_id','wrong'),
    ('left_clock_model_id','wrong'),('right_clock_model_id','wrong'),
    ('graph_epoch','new'),('raw_index_complete',False),('raw_index_hash','')])
def test_wrong_provenance_never_releases_head(field,value):
    writer=delivery();put(writer,1);writer.next_payload(0)
    ack=snapshot(1);setattr(ack,field,value)
    with pytest.raises(GraphDeliveryError):writer.acknowledge(ack)
    assert writer.status()['pending_bundle_ids']==[1] and writer.acknowledged==0


@pytest.mark.parametrize('field,value',[
    ('raw_keys',['L/1','L/1']),('odom_epoch','other'),('node_id',99)])
def test_wrong_original_observation_is_not_an_ack(field,value):
    writer=delivery();put(writer,1);writer.next_payload(0)
    ack=snapshot(1);setattr(ack.nodes[0],field,value)
    with pytest.raises(GraphDeliveryError,match='OBSERVATION_MISMATCH'):writer.acknowledge(ack)


def test_graph_skip_duplicate_and_unsent_ack_are_rejected():
    for ack,sent in ((snapshot(2),True),(snapshot(1,1),True),(snapshot(1),False)):
        writer=delivery();put(writer,1)
        if sent:writer.next_payload(0)
        with pytest.raises(GraphDeliveryError):writer.acknowledge(ack)


def test_sixty_observations_survive_every_third_first_attempt_drop():
    writer=delivery();committed=[];now=0
    for identity in range(1,61):
        put(writer,identity)
        payload=writer.next_payload(now)
        if identity%3==0:
            now+=500_000_000
            assert writer.next_payload(now)==payload
        committed.append(identity)
        assert writer.acknowledge(snapshot(*committed))
        now+=100_000_000
    assert writer.status()['acknowledged']==60 and writer.retries==20


def test_mutable_payload_and_reused_ids_are_rejected():
    writer=delivery()
    with pytest.raises(GraphDeliveryError):writer.submit(1,('L','R'),'odom',bytearray(b'payload'))
    writer=delivery();put(writer,1)
    with pytest.raises(GraphDeliveryError):put(writer,1)
