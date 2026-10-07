"""Real-rclpy publisher discovery regression; no serial devices or control writes.

Run separately in a caller-selected, otherwise unused ROS domain with
ROS_LOCALHOST_ONLY=1. These tests publish only a sentinel feedback String in
that isolated domain; they never instantiate FeedbackSerialLease/MappingWheel.
"""
import os
import threading
import time
import uuid

import pytest

pytest.importorskip('rclpy')
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import get_default_context
from std_msgs.msg import String

from wc_motion.protocol import FeedbackError
from wc_runtime.mapping_wheel import RosOutput


_domain = os.environ.get('ROS_DOMAIN_ID', '')
_isolated = (os.environ.get('ROS_LOCALHOST_ONLY') == '1' and _domain.isdigit()
             and 1 <= int(_domain) <= 232 and int(_domain) != 83)
pytestmark = pytest.mark.skipif(
    not _isolated, reason='requires an explicit local-only non-production ROS domain')


@pytest.fixture
def output():
    # Fail before creating ROS entities if someone invokes this test against the
    # production domain or without an explicit local-only test environment.
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    domain = int(os.environ.get('ROS_DOMAIN_ID', '-1'))
    assert 1 <= domain <= 232 and domain != 83
    assert not get_default_context().ok(), 'run this file in a fresh pytest process'
    publisher = RosOutput()
    assert publisher.context.ok()
    assert not get_default_context().ok()
    try:
        yield publisher
    finally:
        publisher.close()
        assert not publisher.context.ok(), 'private ROS context leaked after close'
        assert not get_default_context().ok(), 'discovery must not initialize global context'


def test_private_context_without_subscriber_reaches_expected_timeout(output):
    assert output.raw.get_subscription_count() == 0
    started = time.monotonic()
    with pytest.raises(FeedbackError, match='recording subscriber discovery timeout'):
        output.wait_for_recorder(lambda: False, timeout_s=.2)
    elapsed = time.monotonic() - started
    assert .18 <= elapsed < 2., 'timeout should be bounded and actually waited'
    assert output.context.ok()
    assert not get_default_context().ok()


def test_delayed_recorder_is_discovered_and_receives_feedback_string(output):
    assert output.raw.get_subscription_count() == 0
    received = []
    state = {}
    executor = None
    sentinel = 'isolated-ros-context-regression-' + uuid.uuid4().hex

    def create_delayed_subscriber():
        try:
            node = rclpy.create_node('test_wheel_recorder_' + uuid.uuid4().hex[:10],
                                     context=output.context)
            state['node'] = node
            state['subscription'] = node.create_subscription(
                String, '/wc_mapping/wheel/feedback_raw',
                lambda message: received.append(message.data),
                QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        except BaseException as error:
            state['error'] = error

    timer = threading.Timer(.2, create_delayed_subscriber)
    timer.start()
    try:
        assert output.wait_for_recorder(lambda: False, timeout_s=5.) is True
        timer.join(timeout=2.)
        assert not timer.is_alive()
        assert 'error' not in state, repr(state.get('error'))
        assert 'node' in state
        # Use a second explicit executor for the recorder node; never touch the
        # uninitialized default ROS context, including in the test itself.
        executor = SingleThreadedExecutor(context=output.context)
        executor.add_node(state['node'])
        output.raw.publish(String(data=sentinel))
        deadline = time.monotonic() + 5.
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert received == [sentinel]
        assert output.wait_for_recording_ack() is True
        assert output.context.ok()
        assert not get_default_context().ok()
    finally:
        timer.cancel()
        timer.join(timeout=5.)
        assert not timer.is_alive(), 'subscriber creation thread did not exit'
        if executor is not None:
            executor.shutdown(timeout_sec=1.)
        if 'node' in state:
            state['node'].destroy_node()


def test_cancelled_discovery_exits_without_initializing_global_context(output):
    assert output.raw.get_subscription_count() == 0
    calls = []

    def cancelled():
        calls.append(time.monotonic())
        return True

    started = time.monotonic()
    with pytest.raises(FeedbackError, match='recording subscriber discovery timeout'):
        output.wait_for_recorder(cancelled, timeout_s=5.)
    assert calls
    assert time.monotonic() - started < 1.
    assert output.context.ok()
    assert not get_default_context().ok()
