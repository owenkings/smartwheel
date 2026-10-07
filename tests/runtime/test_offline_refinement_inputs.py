"""Run on Orin; temporary SQLite evidence only, no ROS/hardware access."""
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import types
import unittest


from wc_runtime import offline_motion_calibration as cal
from wc_runtime import offline_refinement_inputs as inputs


def synthetic_candidate():
    """Structural validation fixture, never evidence of a real calibration."""
    return {'schema': cal.SCHEMA, 'status': cal.VALIDATED,
        'validation': {'passed': True, 'reasons': []},
        'provenance': {'session_id': 'synthetic_refine_session',
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
        self.source = self.root/'synthetic_refine_session'
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


if __name__ == '__main__':
    unittest.main(verbosity=2)
