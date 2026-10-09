"""Deterministic original-clock ordering for offline comparisons, no retiming."""
import math
import copy

ORDER = {'wheel': 0, 'imu': 1, 'source': 2}

def event_key(row):
    return (row['stamp_ns'], ORDER[row['category']], row['sequence'],
            row.get('side', ''), row.get('bag', ''), row.get('message_id', 0))

class SourceClock:
    """Single host-monotonic epoch; physical measurement sync remains unproven."""
    def __init__(self,events):
        if not events: raise ValueError('source clock requires original events')
        first=min(events,key=lambda row:(row['monotonic_ns'],event_key(row)))
        self.monotonic_origin_ns=first['monotonic_ns']; self.ros_origin_ns=first['stamp_ns']
        if type(self.monotonic_origin_ns) is not int or self.monotonic_origin_ns<0:
            raise ValueError('nonnegative source host-monotonic origin required')

    def canonicalize(self,row):
        result=dict(row)
        if type(row['monotonic_ns']) is not int or row['monotonic_ns']<self.monotonic_origin_ns:
            raise ValueError('host-monotonic event predates epoch')
        result['original_source_stamp_ns']=row['stamp_ns']
        result['stamp_ns']=self.ros_origin_ns+row['monotonic_ns']-self.monotonic_origin_ns
        result['clock_mapping']='HOST_MONOTONIC_SINGLE_EPOCH'
        return result

    def report(self):
        result = {'mapping':'HOST_MONOTONIC_SINGLE_EPOCH','monotonic_origin_ns':self.monotonic_origin_ns,
                'ros_origin_ns':self.ros_origin_ns,'physical_measurement_time_validated':False,
                'original_bag_modified':False,'tie_order':'wheel, imu, source; same stream uses source sequence'}
        if hasattr(self, 'imu_time_model'):
            result['imu_time_model'] = copy.deepcopy(self.imu_time_model)
        return result

    def bind_imu_time_model(self, model):
        from .offline_imu_time import validate_model
        if hasattr(self, 'imu_time_model'):
            raise ValueError('offline IMU timing model already frozen')
        self.imu_time_model = copy.deepcopy(validate_model(model))

def select_stamps(stamps, input_rate_hz):
    """Rate-limit cloud consumption only. Never synthesize wheel/IMU observations."""
    if isinstance(input_rate_hz, bool) or not math.isfinite(input_rate_hz) or input_rate_hz <= 0:
        raise ValueError('finite positive input_rate_hz required')
    selected, previous = [], None
    for stamp in sorted(set(stamps)):
        if type(stamp) is not int or stamp < 0:
            raise ValueError('original stamps must be nonnegative integer nanoseconds')
        if previous is None or stamp-previous >= round(1e9/input_rate_hz):
            selected.append(stamp); previous = stamp
    return selected
