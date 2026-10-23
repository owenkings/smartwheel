"""Run on Orin; temporary SQLite evidence only, no ROS/hardware access."""
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import types
import unittest
from unittest import mock


from wc_runtime import offline_motion_calibration as cal
from wc_runtime import offline_refinement_inputs as inputs


def synthetic_candidate():
    """Structural validation fixture, never evidence of a real calibration."""
    return {'schema': cal.SCHEMA, 'status': cal.VALIDATED,
        'validation': {'passed': True, 'reasons': []},
        'provenance': {'session_id': 'capture_20261006_194136',
            'imu_device_id': 'H30-test', 'wheel_device_id': 'wheel-test',
            'R_reference_imu': [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            'wheel_conversion': {'fixture': 'synthetic'}},
        'windows': {'warmup': [0., 10.], 'bias_train': [10., 20.],
            'bias_validation': [21., 31.], 'wheel_train': [32., 72.],
            'wheel_validation': [72., 112.]},
        'policy': copy.deepcopy(cal.DEFAULT_POLICY), 'bias_native_rad_s': [0., 0., .0005],
        'wheel_yaw_scale': .9, 'wheel_yaw_speed_coefficient': -.01}


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.root = Path(self.temp.name)
        self.source = self.root/'capture_20261006_194136'
        (self.source/'bag').mkdir(parents=True)
        self.bag = self.source/'bag/bag_0.db3'
        with sqlite3.connect(self.bag) as db:
            db.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT);'
                             'CREATE TABLE messages(id INTEGER,topic_id INTEGER,data BLOB);')
            db.executemany('INSERT INTO topics VALUES(?,?,?)', [
                (1, '/wc_mapping/imu/source_frame', 'wc_interfaces/msg/H30Frame'),
                (2, '/wc_mapping/wheel/feedback_raw', 'std_msgs/msg/String'),
                (3, '/wc_mapping/lidar_left/source_frame', 'wc_interfaces/msg/SourceFrame')])
            db.executemany('INSERT INTO messages VALUES(?,?,?)', [(9, 2, b'wheel'), (2, 1, b'imu'), (3, 3, b'lidar')])
        self.evidence = self.root/'frozen.json'; self.evidence.write_text('{"frozen": true}')
        self.candidate = synthetic_candidate()
        selected, _, _ = inputs.selected_message_evidence(self.source)
        self.candidate['provenance']['sources'] = [selected, {'path': str(self.evidence),
            'sha256': hashlib.sha256(self.evidence.read_bytes()).hexdigest()}]
        self.path = self.root/'candidate.json'; self.freeze()

    def tearDown(self):
        self.temp.cleanup()

    def freeze(self):
        self.candidate['candidate_id'] = cal._candidate_digest(self.candidate)
        self.path.write_text(json.dumps(self.candidate))

    def verify(self, **kwargs):
        return inputs.verify_candidate(self.candidate, self.path, self.source, {}, **kwargs)

    def test_exact_original_payload_order_excludes_lidar(self):
        item, _, counts = inputs.selected_message_evidence(self.source)
        digest = hashlib.sha256((2).to_bytes(8, 'little')+b'imu'+(9).to_bytes(8, 'little')+b'wheel').hexdigest()
        self.assertEqual(item['sha256'], digest)
        self.assertEqual(list(counts.values()), [1, 1])
        self.assertEqual(self.verify()['status'], 'VERIFIED')

    def test_candidate_memory_file_mismatch(self):
        self.candidate['bias_native_rad_s'][2] += .0001
        with self.assertRaisesRegex(ValueError, 'object differs'):
            self.verify()

    def test_candidate_internal_hash_tamper(self):
        self.candidate['wheel_yaw_scale'] = 1.
        self.path.write_text(json.dumps(self.candidate))
        with self.assertRaisesRegex(ValueError, 'content hash mismatch'):
            self.verify()

    def test_changed_real_evidence(self):
        self.evidence.write_text('{"frozen": false}')
        with self.assertRaisesRegex(ValueError, 'source SHA-256 mismatch'):
            self.verify()

    def test_changed_selected_payload(self):
        with sqlite3.connect(self.bag) as db:
            db.execute('UPDATE messages SET data=? WHERE id=9', (b'changed-wheel',))
        with self.assertRaisesRegex(ValueError, 'source SHA-256 mismatch'):
            self.verify()

    def test_arbitrary_pseudo_path_rejected(self):
        self.candidate['provenance']['sources'][0]['path'] += '_anything'
        self.freeze()
        with self.assertRaisesRegex(ValueError, 'unsupported pseudo-path'):
            self.verify()

    def test_other_session_even_rehashed(self):
        self.candidate['provenance']['session_id'] = 'other_session'; self.freeze()
        with self.assertRaisesRegex(ValueError, 'different session or device'):
            self.verify()

    def test_installed_runtime_identity_binding(self):
        p = self.candidate['provenance']
        runtime = {'session_id': self.source.name, 'imu_sensor_id': 'different_device',
            'wheel_device_id': p['wheel_device_id'], 'wheel_candidate': p['wheel_conversion'],
            'imu_mount': {'R_axle_imu': p['R_reference_imu']}}
        with self.assertRaisesRegex(ValueError, 'different session or device'):
            self.verify(runtime=runtime)

    def test_runtime_rotation_binding(self):
        p = self.candidate['provenance']
        runtime = {'session_id': self.source.name, 'imu_sensor_id': p['imu_device_id'],
            'wheel_device_id': p['wheel_device_id'], 'wheel_candidate': p['wheel_conversion'],
            'imu_mount': {'R_axle_imu': [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}}
        with self.assertRaisesRegex(ValueError, 'axes differ'):
            self.verify(runtime=runtime)

    def test_event_index_tamper(self):
        packets = types.SimpleNamespace(events=[{'bag': self.bag.name, 'message_id': 2,
            'category': 'imu', 'topic': '/wc_mapping/imu/source_frame', 'cdr_sha256': '0'*64}])
        with self.assertRaisesRegex(ValueError, 'differs from verified event index'):
            inputs.selected_message_evidence(self.source, packets=packets)

    def test_original_file_unchanged(self):
        before = self.bag.read_bytes()
        self.verify()
        self.assertEqual(before, self.bag.read_bytes())


class ExtractionTests(unittest.TestCase):
    """Exercise immutable extraction; fitting quality is a separate concern."""

    def setUp(self):
        from wc_motion.protocol import crc16
        self.temp = tempfile.TemporaryDirectory(); self.root = Path(self.temp.name)
        # Archive identity must survive renaming the display folder.
        self.source = self.root/'renamed_recording'
        (self.source/'bag').mkdir(parents=True)
        self.bag = self.source/'bag/bag_0.db3'
        self.session = 'capture_20261006_194136'
        self.wheel_id = 'ZLAC8030D-test'
        self.runtime = {'session_id': self.session, 'imu_sensor_id': 'H30-test',
            'wheel_device_id': self.wheel_id,
            'imu_mount': {'R_axle_imu': [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]},
            'wheel_candidate': {'schema': 'wc_wheel_history_preview_v1', 'state': 'UNVALIDATED',
                'formal_odometry_eligible': False, 'provenance': {'fixture': 'synthetic'},
                'device_id': self.wheel_id, 'wheel_radius_m': .165, 'track_width_m': .5,
                'register_to_wheel_rpm': .112, 'max_gap_s': .5, 'max_wheel_speed_m_s': 2.,
                'left_sign': 1, 'right_sign': 1}}
        (self.source/'capture_manifest.json').write_text(json.dumps({
            'session_id': self.session, 'status': 'COMPLETE', 'recording_complete': True,
            'runtime_config_path': 'runtime_config.json'}))
        (self.source/'runtime_config.json').write_text(json.dumps(self.runtime))
        response = b'\x01\x03\x04\x00\x64\x00\x64'
        self.wheel_row = {'schema': 'wc_wheel_feedback_v1', 'status': 'RESPONSE_VALID',
            'device_id': self.wheel_id, 'request_hex': '010320ab0002be2b',
            'response_hex': (response+crc16(response)).hex(), 'stamp_ns': 2_000_000_000,
            'receive_monotonic_ns': 1_000_000_000, 'sequence': 0,
            'stream_epoch': 'synthetic-wheel-epoch', 'time_valid': False, 'time_source': 'arrival_only'}
        vector = lambda x, y, z: types.SimpleNamespace(x=x, y=y, z=z)
        self.imu_message = types.SimpleNamespace(sensor_id='H30-test', session_id=self.session,
            stream_epoch='synthetic-imu-epoch', angular_velocity_valid=True,
            linear_acceleration_valid=True, coordinate_convention='H30_NATIVE_UNVALIDATED',
            time_source='arrival_only', common_time_valid=False, host_monotonic_ns=1_000_000_000,
            frame_sequence=0, imu=types.SimpleNamespace(angular_velocity=vector(0., 0., 0.),
                linear_acceleration=vector(0., 0., 9.80665)))

    def tearDown(self):
        self.temp.cleanup()

    def extract(self, **wheel_updates):
        row = dict(self.wheel_row, **wheel_updates)
        payloads = {2: b'synthetic-imu-cdr', 9: json.dumps(row, sort_keys=True).encode()}
        with sqlite3.connect(self.bag) as db:
            db.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT);'
                             'CREATE TABLE messages(id INTEGER,topic_id INTEGER,data BLOB);')
            db.executemany('INSERT INTO topics VALUES(?,?,?)', [
                (1, '/wc_mapping/imu/source_frame', 'wc_interfaces/msg/H30Frame'),
                (2, '/wc_mapping/wheel/feedback_raw', 'std_msgs/msg/String')])
            db.executemany('INSERT INTO messages VALUES(?,?,?)', [(2, 1, payloads[2]), (9, 2, payloads[9])])
        events = [{'bag': self.bag.name, 'message_id': mid, 'category': category,
            'topic': topic, 'cdr_sha256': hashlib.sha256(payloads[mid]).hexdigest(),
            'monotonic_ns': 1_000_000_000, 'sequence': 0} for mid, category, topic in [
                (2, 'imu', '/wc_mapping/imu/source_frame'), (9, 'wheel', '/wc_mapping/wheel/feedback_raw')]]

        def get(event):
            # ROS decoding is replaced by a fixture; the real extractor still
            # verifies exact SQLite bytes against every indexed CDR digest.
            if event['category'] == 'imu':
                return copy.deepcopy(self.imu_message)
            return types.SimpleNamespace(data=payloads[event['message_id']].decode())

        packets = types.SimpleNamespace(events=events, get=get, clock=types.SimpleNamespace(
            monotonic_origin_ns=1_000_000_000, report=lambda: {'fixture': 'host-monotonic'}))
        self.before = {p.relative_to(self.source): p.read_bytes()
                       for p in self.source.rglob('*') if p.is_file()}
        self.output = self.root/'calibration'
        windows = copy.deepcopy(synthetic_candidate()['windows'])

        def candidate(imu, wheel, **kwargs):
            result = synthetic_candidate()
            result['provenance'] = copy.deepcopy(kwargs['provenance'])
            result['windows'] = copy.deepcopy(kwargs['windows'])
            result['candidate_id'] = cal._candidate_digest(result)
            return result

        with mock.patch.object(inputs, 'select_windows', return_value=windows) as select, \
                mock.patch.object(inputs, 'build_candidate', side_effect=candidate):
            try:
                result = inputs.extract_and_calibrate(self.source, self.runtime, packets, self.output)
                self.derived_imu, self.derived_wheel = select.call_args.args
                return result
            finally:
                after = {p.relative_to(self.source): p.read_bytes()
                         for p in self.source.rglob('*') if p.is_file()}
                self.assertEqual(self.before, after, 'extraction modified original recording')

    def test_missing_wheel_session_uses_frozen_recording_for_derived_samples(self):
        candidate, path, digest = self.extract()
        self.assertNotIn('session_id', self.wheel_row)
        self.assertEqual(self.derived_wheel[0]['session_id'], self.session)
        self.assertEqual(self.derived_wheel[0]['device_id'], self.wheel_id)
        self.assertGreater(self.derived_wheel[0]['v_m_s'], 0.)
        self.assertEqual(self.derived_imu[0]['session_id'], self.session)
        self.assertEqual(candidate['provenance']['session_id'], self.session)
        self.assertEqual(candidate['provenance']['raw_samples'], {'imu': 1, 'wheel': 1})
        self.assertEqual(digest, hashlib.sha256(path.read_bytes()).hexdigest())
        # The fixture fit is accepted through the real candidate evidence audit.
        self.assertFalse((self.output/'rejection.json').exists())

    def test_matching_wheel_session_is_accepted(self):
        self.extract(session_id=self.session)
        self.assertEqual(self.derived_wheel[0]['session_id'], self.session)

    def assert_wheel_rejected(self, **updates):
        with self.assertRaisesRegex(ValueError, 'original wheel identity/time mismatch'):
            self.extract(**updates)
        report = json.loads((self.output/'rejection.json').read_text())
        self.assertEqual(report['status'], 'REJECTED')
        self.assertFalse(report['original_recording_modified'])
        self.assertFalse((self.output/'candidate.json').exists())

    def test_explicit_other_wheel_session_is_rejected(self):
        self.assert_wheel_rejected(session_id='another_capture')

    def test_explicit_null_wheel_session_is_rejected(self):
        self.assert_wheel_rejected(session_id=None)

    def test_explicit_empty_wheel_session_is_rejected(self):
        self.assert_wheel_rejected(session_id='')

    def test_other_wheel_device_is_rejected(self):
        self.assert_wheel_rejected(device_id='ZLAC8030D-other')

    def test_wheel_time_not_matching_index_is_rejected(self):
        self.assert_wheel_rejected(receive_monotonic_ns=1_000_000_001)

    def test_wheel_sequence_not_matching_index_is_rejected(self):
        self.assert_wheel_rejected(sequence=1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
