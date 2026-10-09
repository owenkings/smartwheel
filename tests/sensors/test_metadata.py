import unittest
import numpy as np

from wc_sensors.metadata import ClockModel, SourceFrameBuilder, normalize_time_parts, timestamp_ns


def builder(model=None):
    return SourceFrameBuilder("session-a", "left", "serial-left", "lidar_left", "sha256:fixture", model)


def build(subject, raw=100, host=1000, mono=100, points=None, unit="ns"):
    return subject.build([[1, 2, 3]] if points is None else points,
                         device_timestamp_raw=raw, device_timestamp_unit=unit,
                         host_receive_ns=host, host_monotonic_ns=mono)


class MetadataTests(unittest.TestCase):
    def test_normalize_carries_exactly_and_rejects_negative_time(self):
        self.assertEqual(normalize_time_parts(1, 1_000_000_003), (2, 3))
        self.assertEqual(normalize_time_parts(1, -1), (0, 999_999_999))
        with self.assertRaises(ValueError):
            normalize_time_parts(0, -1)

    def test_sdk_ns_fields_reject_noncanonical_and_float(self):
        for seconds, ns in ((1, 1_000_000_000), (-1, 1), (1, -1), (1 << 64, 0)):
            with self.assertRaises(ValueError):
                timestamp_ns(seconds, ns)
        for seconds, ns in ((1.0, 0), (True, 0), (1, 0.1)):
            with self.assertRaises(TypeError):
                timestamp_ns(seconds, ns)

    def test_large_timestamp_precision_is_integer(self):
        self.assertEqual(timestamp_ns(9_000_000_001, 123), 9_000_000_001_000_000_123)

    def test_arrival_is_not_synchronized_measurement(self):
        frame = build(builder(), raw=None, unit="UNKNOWN")
        self.assertEqual(frame["common_time_ns"], 1000)
        self.assertFalse(frame["time_valid"])
        self.assertIsNone(frame["uncertainty_ns"])
        self.assertIsNone(frame["clock_model_id"])
        self.assertEqual(frame["time_source"], "arrival_only")

    def test_sequence_and_key_unique(self):
        source = builder()
        first, second = build(source), build(source, raw=101, host=1001, mono=101)
        self.assertEqual(first["frame_sequence"], 0)
        self.assertEqual(second["frame_sequence"], 1)
        self.assertNotEqual(first["raw_key"], second["raw_key"])
        self.assertEqual(first["raw_key"], f"serial-left/{first['stream_epoch']}/0")

    def test_copy_reused_sdk_buffer_and_count_invalid_points(self):
        source = np.array([[1., 2., 3.], [np.nan, 0., 0.]])
        frame = build(builder(), points=source)
        source[:] = 7
        self.assertEqual(frame["points"][0, 0], 1)
        self.assertTrue(np.isnan(frame["points"][1, 0]))
        self.assertTrue(frame["points"].flags.owndata)
        self.assertEqual(frame["raw_point_count"], 2)
        self.assertEqual(frame["valid_point_count"], 1)

    def test_raw_counter_backward_creates_new_epoch(self):
        source = builder()
        before = build(source, raw=0xFFFF_FFFF)
        after = build(source, raw=1, host=1001, mono=101)
        self.assertNotEqual(before["stream_epoch"], after["stream_epoch"])
        self.assertEqual(after["frame_sequence"], 0)
        self.assertIn("device_time_backward_or_wrap", after["diagnostic_flags"])

    def test_reconnect_invalidates_previous_clock_model(self):
        model = ClockModel("m1", "ns", 1, 1, 900, 10, True)
        source = builder(model)
        before = build(source)
        source.restart()
        after = build(source)
        self.assertTrue(before["time_valid"])
        self.assertFalse(after["time_valid"])
        self.assertIsNone(after["clock_model_id"])

    def test_host_clock_backward_starts_epoch(self):
        source = builder()
        before = build(source)
        after = build(source, raw=101, host=999, mono=101)
        self.assertNotEqual(before["stream_epoch"], after["stream_epoch"])
        self.assertIn("host_clock_backward", after["diagnostic_flags"])

    def test_duplicate_timestamp_flagged(self):
        source = builder()
        build(source)
        duplicate = build(source, host=1001, mono=101)
        self.assertIn("duplicate_device_timestamp", duplicate["diagnostic_flags"])

    def test_source_unit_change_starts_new_epoch(self):
        source = builder()
        before = build(source)
        after = build(source, raw=101, host=1001, mono=101, unit="us")
        self.assertNotEqual(before["stream_epoch"], after["stream_epoch"])
        self.assertIn("device_timestamp_unit_changed", after["diagnostic_flags"])

    def test_rational_clock_preserves_large_integer(self):
        raw = 9_000_000_000_000_001
        model = ClockModel("m", "ticks", 1001, 3, -900, 7, True)
        frame = build(builder(model), raw=raw, unit="ticks")
        self.assertEqual(frame["common_time_ns"], raw * 1001 // 3 - 900)
        self.assertEqual(frame["device_timestamp_raw"], raw)
        self.assertEqual(frame["uncertainty_ns"], 7)
        self.assertTrue(frame["time_valid"])

    def test_unvalidated_or_unknown_error_never_passes(self):
        for validated, uncertainty in ((False, 1), (True, None)):
            frame = build(builder(ClockModel("m", "ns", 1, 1, 0, uncertainty, validated)))
            self.assertFalse(frame["time_valid"])
            self.assertIsNone(frame["uncertainty_ns"])
            self.assertEqual(frame["time_source"], "arrival_only")

    def test_numpy_integer_model_does_not_overflow_int64(self):
        model = ClockModel("m", "ticks", np.int64(1000), np.int64(1), np.int64(0), 1, True)
        self.assertEqual(model.convert(1 << 63, "ticks"), (1 << 63) * 1000)

    def test_prefixed_tf_frame_name_preserved(self):
        source = SourceFrameBuilder("s", "left", "id", "wc_mapping/lidar_left", "hash")
        self.assertEqual(build(source)["frame_id"], "wc_mapping/lidar_left")

    def test_clock_unit_mismatch_and_expiry_fail_closed(self):
        model = ClockModel("m", "us", 1000, 1, 0, 1, True, 0, 90)
        for raw, unit in ((100, "us"), (50, "ns")):
            frame = build(builder(model), raw=raw, unit=unit)
            self.assertFalse(frame["time_valid"])
            self.assertIn("clock_model_not_applicable", frame["diagnostic_flags"])

    def test_bad_identity_side_and_point_shape_rejected(self):
        with self.assertRaises(ValueError):
            SourceFrameBuilder("s", "rear", "id", "f", "hash")
        with self.assertRaises(ValueError):
            SourceFrameBuilder("s", "left", "id/ambiguous", "f", "hash")
        with self.assertRaises(ValueError):
            build(builder(), points=np.zeros((3, 4)))


if __name__ == "__main__":
    unittest.main()
