#!/usr/bin/env python3
"""Verify recorded timestamp semantics without fabricating sample times.

Reads finalized project bags only. No playback, device access, system-clock
change, timestamp interpolation or clock model application is performed.
"""
import argparse
from bisect import bisect_left
from collections import Counter
import json
from pathlib import Path
import time

ROOT = Path('/home/nvidia/wheelchair')
TOPICS = {'/wc_mapping/lidar_left/source_frame': 'left',
          '/wc_mapping/lidar_right/source_frame': 'right',
          '/wc_mapping/imu/source_frame': 'imu'}


def ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def distribution(values):
    if not values:
        return {'count': 0}
    data = sorted(values)
    def percentile(q):
        position = (len(data) - 1) * q
        low = int(position)
        return data[low] + (data[min(low + 1, len(data) - 1)] - data[low]) * (position - low)
    return {'count': len(data), 'min': data[0], 'p50': percentile(.5), 'p95': percentile(.95),
            'p99': percentile(.99), 'max': data[-1]}


def summarize(rows):
    result = {}
    for side, records in rows.items():
        stamps = [r['host_monotonic_ns'] for r in records]
        intervals = [b-a for a,b in zip(stamps,stamps[1:])]
        repeats = Counter(stamps)
        result[side] = {'frames': len(records), 'time_source': dict(Counter(r['time_source'] for r in records)),
            'all_header_equals_host_receive': bool(records) and all(r['header_ns']==r['host_receive_ns'] for r in records),
            'all_nested_header_matches': bool(records) and all(r['header_ns']==r['nested_header_ns'] for r in records),
            'declared_common_time_valid_count': sum(r['common_time_valid'] for r in records),
            'declared_uncertainty_valid_count': sum(r['uncertainty_valid'] for r in records),
            'device_timestamp_unit_counts': dict(Counter(r['device_timestamp_unit'] for r in records)),
            'host_monotonic_interval_ns': distribution(intervals),
            'equal_host_arrival_intervals': sum(v==0 for v in intervals),
            'backward_host_intervals': sum(v<0 for v in intervals),
            'max_frames_sharing_one_host_arrival': max(repeats.values(),default=0),
            'bag_record_minus_host_receive_ns': distribution([r['bag_ns']-r['host_receive_ns'] for r in records]),
            'sample_timestamp_present_count': sum(r.get('sample_timestamp_valid',False) for r in records),
            'dataready_timestamp_present_count': sum(r.get('dataready_timestamp_valid',False) for r in records)}
    right = sorted(r['host_monotonic_ns'] for r in rows.get('right',[]))
    differences = []
    for row in rows.get('left',[]):
        stamp = row['host_monotonic_ns']; index=bisect_left(right,stamp)
        candidates=right[max(0,index-1):min(len(right),index+1)]
        if candidates:
            differences.append(min(candidates,key=lambda t:abs(t-stamp))-stamp)
    proven = all(side in result and result[side]['frames']>0 and
        result[side]['all_header_equals_host_receive'] and result[side]['all_nested_header_matches'] and
        result[side]['time_source']=={'arrival_only':result[side]['frames']} and
        result[side]['declared_common_time_valid_count']==0 and
        result[side]['declared_uncertainty_valid_count']==0 for side in ('left','right','imu'))
    return {'status':'ARRIVAL_BASIS_VERIFIED' if proven else 'TIME_BASIS_CHECK_FAILED',
        'streams':result,'nearest_right_arrival_minus_left_arrival_ns':distribution(differences),
        'nearest_arrival_difference_is_measurement_offset':False,
        'measurement_synchronization_validated':False,'end_to_end_latency_measured':False,
        'limits':['Host arrival pairing is not exposure-time synchronization.',
                  'Equal IMU arrival stamps describe host batching, not simultaneous physical samples.',
                  'Device clock units/epochs cannot be inferred from host timestamp equality.',
                  'This audit does not validate mounting, map accuracy, missing frames or dynamic compensation.']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    bag=args.bag.resolve(strict=True);output=args.output.absolute()
    if not bag.is_relative_to(ROOT/'data/bags') or not (bag/'metadata.yaml').is_file():
        raise ValueError('A finalized bag under the project data/bags directory is required')
    if not output.resolve().is_relative_to(ROOT/'reports') or output.exists() or output.is_symlink():
        raise ValueError('Report must be new and under project reports')
    session=ROOT/'.phase1_runtime/sessions'/bag.name/'record/manifest.json'
    manifest=json.loads(session.read_text())
    if manifest.get('state')!='STOPPED' or manifest.get('exit_code')!=0:
        raise ValueError('Managed recording has not finished normally')
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    classes={v.name:get_message(v.type) for v in reader.get_all_topics_and_types() if v.name in TOPICS}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    rows={side:[] for side in TOPICS.values()}
    while reader.has_next():
        topic,data,record_ns=reader.read_next()
        if topic not in TOPICS:continue
        m=deserialize_message(data,classes[topic]); side=TOPICS[topic]
        if len(rows[side])>=1_000_000:raise ValueError('Bounded audit frame budget exceeded')
        row={'header_ns':ns(m.header.stamp),'host_receive_ns':ns(m.host_receive_time),
             'host_monotonic_ns':int(m.host_monotonic_ns),'bag_ns':int(record_ns),
             'nested_header_ns':ns((m.imu if side=='imu' else m.cloud).header.stamp),
             'common_time_valid':bool(m.common_time_valid),'uncertainty_valid':bool(m.uncertainty_valid),
             'time_source':m.time_source,'device_timestamp_unit':m.device_timestamp_unit}
        if side=='imu':row.update(sample_timestamp_valid=bool(m.sample_timestamp_valid),dataready_timestamp_valid=bool(m.dataready_timestamp_valid))
        rows[side].append(row)
    report={'test':'recorded_arrival_time_semantics','bag':str(bag),'observed_ns':time.time_ns(),**summarize(rows)}
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2))
    return 0 if report['status']=='ARRIVAL_BASIS_VERIFIED' else 1


if __name__=='__main__':
    raise SystemExit(main())
