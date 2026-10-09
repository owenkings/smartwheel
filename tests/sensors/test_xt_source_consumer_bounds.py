"""Full XT image bounds; pure SourceFrame envelopes, no ROS or devices."""
from types import SimpleNamespace as NS
import copy
import struct
import pytest
from wc_runtime.single_mapping_input import SourceGate, InputFailure, MAX_CLOUD_POINTS, MAX_CLOUD_BYTES


def source(count, *, step=16):
    stamp=NS(sec=1_700_000_000,nanosec=0)
    header=NS(stamp=stamp,frame_id='lidar_left')
    payload=bytearray(count*step)
    for i in range(count):struct.pack_into('<4f',payload,i*step,1,2,3,4)
    cloud=NS(header=copy.deepcopy(header),width=count,height=1,point_step=step,row_step=count*step,
        is_bigendian=False,is_dense=False,fields=[NS(name=n,offset=i*4,count=1,datatype=7)
        for i,n in enumerate(('x','y','z','intensity'))],data=payload)
    return NS(session_id='bounds',side='left',sensor_id='sensor-left',units='m',coordinate_convention='FLU',
        time_source='arrival_only',common_time_valid=False,uncertainty_valid=False,clock_model_id='',
        header=header,cloud=cloud,host_receive_time=copy.deepcopy(stamp),common_time_ns=1_700_000_000_000_000_000,
        host_monotonic_ns=10_000_000_000,frame_sequence=0,stream_epoch='e1',source_config_hash='a'*64,
        raw_count=count,valid_count=count)


def gate():return SourceGate('bounds','left','sensor-left',started_ns=0)


def test_full_320_by_240_frame_passes_without_row_removal():
    value=source(MAX_CLOUD_POINTS)
    assert len(value.cloud.data)==MAX_CLOUD_BYTES
    result=gate().inspect(value,value.host_monotonic_ns)
    assert result['raw_count']==76800 and result['invalid_xyz_count']==0


def test_just_over_full_sensor_count_fails_with_explicit_bound():
    value=source(MAX_CLOUD_POINTS+1)
    with pytest.raises(InputFailure,match='CLOUD_SIZE_OUT_OF_BOUNDS'):
        gate().inspect(value,value.host_monotonic_ns)


def test_valid_cloud_layout_cannot_exceed_payload_byte_budget():
    value=source(MAX_CLOUD_POINTS,step=17)
    validator=gate()
    with pytest.raises(InputFailure,match='CLOUD_SIZE_OUT_OF_BOUNDS'):
        validator.inspect(value,value.host_monotonic_ns)
    assert validator.failure_reason=='CLOUD_SIZE_OUT_OF_BOUNDS'
