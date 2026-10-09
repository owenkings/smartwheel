#!/usr/bin/env python3
"""Actual target ROS publisher/context ABI in isolated domain; no serial access."""
import json
import os
import signal
import time
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
from wc_motion.manual_hardware import RosFeedbackPublisher


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='89' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('This no-device ROS ABI test requires isolated localhost domain 89')
    original={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    publisher=RosFeedbackPublisher('ZLAC8030D-SYNTHETIC-ABI')
    try:
        assert {s:signal.getsignal(s) for s in original}==original
        assert publisher.publisher.topic_name=='/wc_mapping/wheel/feedback_raw'
        assert publisher.publish_record({'schema':'fixture_no_sensor_data','source':'SYNTHETIC_NO_DEVICE'}) is False
        # Native construction/context are exercised without even synthetic wheel
        # messages; the mock tests cover publish payload and failure semantics.
        print(json.dumps({'status':'PASS','test':'NATIVE_ROS_PUBLISHER_CONTEXT_ABI',
                          'hardware_opened':False,'messages_published':0,'domain':89,
                          'signal_handlers_preserved':True,'observed_ns':time.time_ns()}))
    finally:
        publisher.close()
    assert {s:signal.getsignal(s) for s in original}==original


if __name__=='__main__':main()
