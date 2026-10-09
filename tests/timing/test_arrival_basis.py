import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('arrival_audit',Path(__file__).with_name('check_arrival_basis.py'))
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def row(stamp,common=False):
    return dict(header_ns=stamp,host_receive_ns=stamp,nested_header_ns=stamp,host_monotonic_ns=stamp,
                bag_ns=stamp+5,time_source='arrival_only',common_time_valid=common,
                uncertainty_valid=False,device_timestamp_unit='UNKNOWN')


def test_host_batching_is_not_sample_synchronization():
    result=audit.summarize({'left':[row(10),row(20)],'right':[row(13),row(23)],'imu':[row(10),row(10),row(20)]})
    assert result['status']=='ARRIVAL_BASIS_VERIFIED'
    assert result['streams']['imu']['max_frames_sharing_one_host_arrival']==2
    assert result['nearest_right_arrival_minus_left_arrival_ns']['p50']==3
    assert result['measurement_synchronization_validated'] is False


def test_different_header_or_valid_common_clock_is_not_arrival_proof():
    rows={side:[row(10)] for side in ('left','right','imu')}
    rows['imu'][0]['header_ns']=9
    assert audit.summarize(rows)['status']=='TIME_BASIS_CHECK_FAILED'
    rows['imu'][0]=row(10,common=True)
    assert audit.summarize(rows)['status']=='TIME_BASIS_CHECK_FAILED'


def test_missing_sensor_never_passes():
    assert audit.summarize({'left':[row(10)],'right':[],'imu':[row(10)]})['status']=='TIME_BASIS_CHECK_FAILED'
