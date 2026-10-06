"""Synthetic contract tests; no real validation model, ROS node or device.

All VALIDATED declarations below are deliberately synthetic in-memory fixtures
testing rejection/application logic, and must never be used as live evidence.
"""

from copy import deepcopy
from types import SimpleNamespace
import unittest

import numpy as np

from wc_sensors.clock_adapter import apply_clock_model, ClockAdapterError


SENSOR = 'fixture-sensor-left'
MODEL = 'fixture-clock-1'
HASH = 'a'*64
MAX64 = (1 << 64)-1
MAXI64 = (1 << 63)-1


def source_frame():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=7), frame_id='fixture_lidar'),
        session_id='SYNTHETIC-CONTRACT-TEST', side='left', sensor_id=SENSOR,
        stream_epoch='fixture-boot-1', frame_sequence=19,
        cloud=SimpleNamespace(data=bytearray(b'original-cloud'), header={'stamp': [100, 7]}),
        source_config_hash=HASH, coordinate_convention='FLU', units='m',
        device_timestamp_valid=True, device_timestamp_raw=1001, device_timestamp_unit='sdk_v3_ms',
        device_time_components_valid=True, device_timestamp_seconds=1, device_timestamp_nanoseconds=1_000_000,
        device_time_type='0', device_sync_state='0',
        host_receive_time=SimpleNamespace(sec=100, nanosec=7), host_monotonic_ns=77,
        common_time_valid=False, common_time_ns=100_000_000_007, clock_model_id='',
        time_source='arrival_only', uncertainty_valid=False, uncertainty_ns=0,
        raw_count=1, valid_count=1, diagnostic_flags=['time_model_unvalidated', 'SYNTHETIC_TEST_FIXTURE'])


def model_dict():
    return dict(schema_version=1, source_mode='real', status='VALIDATED', validated=True,
        model_id=MODEL, sensor_id=SENSOR, source_config_hash=HASH, stream_epoch='fixture-boot-1',
        input_timestamp='device_scalar', device_timestamp_unit='sdk_v3_ms',
        scale_numerator=1_000_000, scale_denominator=1, offset_ns=123,
        uncertainty_ns=500, valid_raw_min=1000, valid_raw_max=2000,
        evidence_refs=['fixture://independent-clock-report'],
        unit_evidence_refs=['fixture://source-unit-report'],
        measurement_evidence_refs=['fixture://exposure-reference-report'],
        unit_verified=True, measurement_semantics_verified=True,
        measurement_semantics='sensor frame exposure midpoint under independently checked HDR policy',
        validation_basis='independent_measurement')


def apply(frame=None, model=None):
    return apply_clock_model(source_frame() if frame is None else frame,
                             model_dict() if model is None else model, SENSOR, MODEL)


class ClockAdapterTests(unittest.TestCase):
    def test_scalar_exact_integer_mapping(self):
        result = apply()
        self.assertEqual(result.common_time_ns, 1_001_000_123)
        self.assertEqual(result.clock_model_id, MODEL)
        self.assertTrue(result.common_time_valid)
        self.assertTrue(result.uncertainty_valid)
        self.assertEqual(result.uncertainty_ns, 500)
        self.assertEqual(result.time_source, 'validated_model')

    def test_only_six_fields_change_and_input_remains_unchanged(self):
        frame, model = source_frame(), model_dict()
        before, model_before = deepcopy(frame), deepcopy(model)
        result = apply(frame, model)
        changed = {'common_time_valid', 'common_time_ns', 'clock_model_id',
                   'uncertainty_valid', 'uncertainty_ns', 'time_source'}
        self.assertEqual(vars(frame), vars(before))
        self.assertEqual(model, model_before)
        for name, value in vars(before).items():
            if name not in changed:
                self.assertEqual(getattr(result, name), value, name)
        result.cloud.data[0] = 0
        result.diagnostic_flags.append('modified_copy')
        self.assertEqual(frame.cloud.data, bytearray(b'original-cloud'))
        self.assertEqual(frame.diagnostic_flags, before.diagnostic_flags)

    def test_dictionary_envelope_works_and_is_owned(self):
        frame = vars(source_frame())
        result = apply(frame)
        self.assertEqual(result['common_time_ns'], 1_001_000_123)
        result['cloud'].data[0] = 0
        self.assertEqual(frame['cloud'].data, bytearray(b'original-cloud'))

    def test_sensor_model_and_expected_ids_must_all_match(self):
        for field in ('sensor_id', 'model_id'):
            model = model_dict(); model[field] = 'different'
            with self.subTest(field=field), self.assertRaises(ClockAdapterError):
                apply(model=model)
        frame = source_frame(); frame.sensor_id = 'right-sensor'
        with self.assertRaises(ClockAdapterError):
            apply(frame)
        with self.assertRaises(ClockAdapterError):
            apply_clock_model(source_frame(), model_dict(), SENSOR, 'other-expected-model')
        with self.assertRaises(ClockAdapterError):
            apply_clock_model(source_frame(), model_dict(), 'other-expected-sensor', MODEL)

    def test_different_epoch_and_configuration_rejected(self):
        for field, value in (('stream_epoch', 'reconnected-boot'), ('source_config_hash', 'b'*64)):
            frame = source_frame(); setattr(frame, field, value)
            with self.subTest(field=field), self.assertRaises(ClockAdapterError):
                apply(frame)
            model = model_dict(); model[field] = value
            with self.subTest(model_field=field), self.assertRaises(ClockAdapterError):
                apply(model=model)

    def test_configuration_must_be_explicit_sha256(self):
        for value in ('', 'UNKNOWN', 'fixture', 'sha256:'+'a'*64):
            model = model_dict(); model['source_config_hash'] = value
            with self.subTest(value=value), self.assertRaises(ClockAdapterError):
                apply(model=model)

    def test_rejects_candidate_synthetic_and_arrival_fit_models(self):
        for field, value in (('status', 'CANDIDATE'), ('source_mode', 'synthetic'),
                             ('validated', False), ('validated', 1),
                             ('validation_basis', 'arrival_fit'), ('validation_basis', 'UNKNOWN')):
            model = model_dict(); model[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ClockAdapterError):
                apply(model=model)

    def test_real_model_rejects_synthetic_frame(self):
        frame = source_frame(); frame.time_source = 'synthetic'
        with self.assertRaises(ClockAdapterError):
            apply(frame)
        frame = source_frame(); frame.source_mode = 'synthetic'
        with self.assertRaises(ClockAdapterError):
            apply(frame)

    def test_cannot_apply_twice_or_change_existing_validation(self):
        with self.assertRaises(ClockAdapterError):
            apply(apply())
        for field, value in (('clock_model_id', MODEL), ('common_time_valid', True), ('uncertainty_valid', True)):
            frame = source_frame(); setattr(frame, field, value)
            with self.subTest(field=field), self.assertRaises(ClockAdapterError):
                apply(frame)

    def test_all_evidence_sets_are_required(self):
        for field in ('evidence_refs', 'unit_evidence_refs', 'measurement_evidence_refs'):
            for value in (None, [], '', [''], ['  '], ['UNKNOWN'], ['good', '']):
                model = model_dict(); model[field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ClockAdapterError):
                    apply(model=model)
            model = model_dict(); del model[field]
            with self.assertRaises(ClockAdapterError):
                apply(model=model)

    def test_unit_and_measurement_semantics_must_be_verified(self):
        for field in ('unit_verified', 'measurement_semantics_verified'):
            for value in (False, None, 1, 'true'):
                model = model_dict(); model[field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ClockAdapterError):
                    apply(model=model)
        for value in ('', 'UNKNOWN', 'TBD'):
            model = model_dict(); model['measurement_semantics'] = value
            with self.assertRaises(ClockAdapterError):
                apply(model=model)

    def test_unknown_or_mismatched_scalar_unit_rejected(self):
        for value in ('us', 'ns', 'UNKNOWN_us_or_100us', ''):
            model = model_dict(); model['device_timestamp_unit'] = value
            with self.subTest(value=value), self.assertRaises(ClockAdapterError):
                apply(model=model)
        frame = source_frame(); frame.device_timestamp_valid = False
        with self.assertRaises(ClockAdapterError):
            apply(frame)

    def test_uncertainty_must_be_known_integer_uint64(self):
        for value in (None, -1, 1.0, float('nan'), float('inf'), True, MAX64+1):
            model = model_dict(); model['uncertainty_ns'] = value
            with self.subTest(value=value), self.assertRaises(ClockAdapterError):
                apply(model=model)
        for value in (0, MAX64):
            model = model_dict(); model['uncertainty_ns'] = value
            self.assertEqual(apply(model=model).uncertainty_ns, value)

    def test_both_inclusive_bounds_are_mandatory(self):
        for raw in (1000, 2000):
            frame = source_frame(); frame.device_timestamp_raw = raw
            self.assertEqual(apply(frame).common_time_ns, raw*1_000_000+123)
        for raw in (999, 2001):
            frame = source_frame(); frame.device_timestamp_raw = raw
            with self.assertRaises(ClockAdapterError):
                apply(frame)
        for field in ('valid_raw_min', 'valid_raw_max'):
            for value in (None, float('inf'), 1.0, True):
                model = model_dict(); model[field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ClockAdapterError):
                    apply(model=model)
            model = model_dict(); del model[field]
            with self.assertRaises(ClockAdapterError):
                apply(model=model)
        model = model_dict(); model['valid_raw_max'] = 999
        with self.assertRaises(ClockAdapterError):
            apply(model=model)

    def test_rational_floor_no_float_nanosecond_loss(self):
        raw = 9_000_000_000_000_001
        frame, model = source_frame(), model_dict()
        frame.device_timestamp_raw = raw
        model.update(scale_numerator=1001, scale_denominator=3, offset_ns=-900,
                     valid_raw_min=raw, valid_raw_max=raw)
        result = apply(frame, model)
        self.assertEqual(result.common_time_ns, raw*1001//3-900)
        self.assertEqual(result.device_timestamp_raw, raw)

    def test_components_allow_total_beyond_uint64_and_keep_raw_scalar(self):
        frame, model = source_frame(), model_dict()
        frame.device_timestamp_seconds = MAX64
        frame.device_timestamp_nanoseconds = 999_999_999
        frame.device_timestamp_valid = False
        frame.device_timestamp_unit = 'UNKNOWN_scalar_not_used'
        raw = MAX64*1_000_000_000+999_999_999
        model.update(input_timestamp='device_components_ns', device_timestamp_unit='ns',
                     scale_numerator=1, scale_denominator=1, offset_ns=-raw+789,
                     valid_raw_min=raw, valid_raw_max=raw)
        result = apply(frame, model)
        self.assertEqual(result.common_time_ns, 789)
        self.assertEqual(result.device_timestamp_seconds, MAX64)
        self.assertEqual(result.device_timestamp_nanoseconds, 999_999_999)
        self.assertEqual(result.device_timestamp_raw, frame.device_timestamp_raw)
        self.assertEqual(result.device_timestamp_unit, frame.device_timestamp_unit)

    def test_components_require_canonical_parts_and_proven_ns_unit(self):
        for field, value in (('device_timestamp_seconds', -1), ('device_timestamp_seconds', MAX64+1),
                             ('device_timestamp_seconds', 1.0), ('device_timestamp_nanoseconds', -1),
                             ('device_timestamp_nanoseconds', 1_000_000_000),
                             ('device_timestamp_nanoseconds', True), ('device_time_components_valid', False)):
            frame, model = source_frame(), model_dict()
            model.update(input_timestamp='device_components_ns', device_timestamp_unit='ns', valid_raw_max=10**12)
            setattr(frame, field, value)
            with self.subTest(field=field, value=value), self.assertRaises(ClockAdapterError):
                apply(frame, model)
        model = model_dict(); model['input_timestamp'] = 'device_components_ns'
        with self.assertRaises(ClockAdapterError):
            apply(model=model)

    def test_ros_int64_output_upper_bound_and_negative_output(self):
        frame, model = source_frame(), model_dict()
        model.update(scale_numerator=1, offset_ns=MAXI64-frame.device_timestamp_raw)
        self.assertEqual(apply(frame, model).common_time_ns, MAXI64)
        model['offset_ns'] += 1
        with self.assertRaises(ClockAdapterError):
            apply(frame, model)
        model['offset_ns'] = -frame.device_timestamp_raw-1
        with self.assertRaises(ClockAdapterError):
            apply(frame, model)
        model['offset_ns'] += 1
        self.assertEqual(apply(frame, model).common_time_ns, 0)

    def test_no_float_bool_or_negative_rational_parameters(self):
        for field in ('scale_numerator', 'scale_denominator'):
            for value in (0, -1, 1.0, True, None):
                model = model_dict(); model[field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ClockAdapterError):
                    apply(model=model)
        for value in (float('nan'), 0.0, True):
            model = model_dict(); model['offset_ns'] = value
            with self.assertRaises(ClockAdapterError):
                apply(model=model)

    def test_numpy_integers_convert_to_python_before_arithmetic(self):
        model = model_dict()
        model.update(scale_numerator=np.int64(1_000_000), scale_denominator=np.int64(1),
                     offset_ns=np.int64(123), uncertainty_ns=np.uint64(500))
        frame = source_frame(); frame.device_timestamp_raw = np.uint64(1001)
        self.assertEqual(apply(frame, model).common_time_ns, 1_001_000_123)

    def test_missing_schema_and_invalid_input_selector_rejected(self):
        for change in ({'schema_version': 2}, {'schema_version': True}, {'input_timestamp': 'host_receive_ns'}):
            model = model_dict(); model.update(change)
            with self.assertRaises(ClockAdapterError):
                apply(model=model)
        model = model_dict(); del model['schema_version']
        with self.assertRaises(ClockAdapterError):
            apply(model=model)

    def test_rejected_model_never_modifies_original_message(self):
        frame, model = source_frame(), model_dict()
        before = deepcopy(frame)
        model['valid_raw_max'] = 1000
        with self.assertRaises(ClockAdapterError):
            apply(frame, model)
        self.assertEqual(vars(frame), vars(before))


if __name__ == '__main__':
    unittest.main()
