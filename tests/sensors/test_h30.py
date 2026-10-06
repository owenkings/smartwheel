import struct
import unittest

from wc_sensors.h30 import H30Parser, h30_checksum


def tlv(field_id, value):
    return bytes((field_id, len(value))) + value


def packet(payload, tid=1):
    body = struct.pack("<HB", tid, len(payload)) + payload
    # Independent checksum implementation for test fixtures.
    first = sum(body) & 255
    second = sum((len(body) - i) * value for i, value in enumerate(body)) & 255
    return b"YS" + body + bytes((first, second))


def imu_packet(tid=1, quat=True):
    payload = tlv(0x10, struct.pack("<iii", 0, 0, 9_806_650))
    payload += tlv(0x20, struct.pack("<iii", 180_000_000, 0, -90_000_000))
    if quat:
        payload += tlv(0x41, struct.pack("<iiii", 1_000_000, 0, 0, 0))
    payload += tlv(0x51, struct.pack("<I", 0xFFFF_FFFE))
    return packet(payload, tid)


class H30Tests(unittest.TestCase):
    def test_known_checksum_vector(self):
        self.assertEqual(h30_checksum(bytes((1, 2, 3))), bytes((6, 10)))

    def test_units_and_timestamps_not_guessed(self):
        frame = H30Parser().feed(imu_packet())[0]
        imu = frame.imu_dict()
        self.assertAlmostEqual(imu["linear_acceleration"][2], 9.80665)
        self.assertAlmostEqual(imu["angular_velocity"][0], 3.141592653589793)
        self.assertAlmostEqual(imu["angular_velocity"][2], -1.5707963267948966)
        self.assertEqual(imu["orientation"], (0, 0, 0, 1))
        self.assertEqual(imu["sample_timestamp_raw"], 0xFFFF_FFFE)
        self.assertEqual(imu["device_timestamp_unit"], "UNKNOWN_us_or_100us")
        self.assertFalse(imu["time_valid"])
        self.assertIsNone(imu["uncertainty_ns"])

    def test_every_fragment_boundary(self):
        data = imu_packet()
        for split in range(len(data) + 1):
            parser = H30Parser()
            result = parser.feed(data[:split]) + parser.feed(data[split:])
            self.assertEqual(len(result), 1, split)
            self.assertEqual(result[0].raw_packet, data)

    def test_single_byte_stream_and_multiple_frames(self):
        parser = H30Parser()
        results = []
        for byte in imu_packet(3) + imu_packet(4):
            results.extend(parser.feed(bytes((byte,))))
        self.assertEqual([r.tid for r in results], [3, 4])
        self.assertEqual(parser.buffered_bytes, 0)

    def test_crc_damage_does_not_publish_and_recovers(self):
        data = bytearray(imu_packet(1))
        data[-1] ^= 0x80
        parser = H30Parser()
        results = parser.feed(data + imu_packet(2))
        self.assertEqual([r.tid for r in results], [2])
        self.assertGreaterEqual(parser.stats["checksum_errors"], 1)

    def test_tlv_extends_past_payload_is_rejected(self):
        bad = packet(bytes((0x10, 12)) + b"\0\0")
        parser = H30Parser()
        results = parser.feed(bad + imu_packet(9))
        self.assertEqual([r.tid for r in results], [9])
        self.assertEqual(parser.stats["malformed_tlv"], 1)

    def test_single_trailing_tlv_byte_is_rejected(self):
        parser = H30Parser()
        self.assertEqual(parser.feed(packet(b"\x10")), [])
        self.assertEqual(parser.stats["malformed_tlv"], 1)

    def test_known_tlv_wrong_length(self):
        parser = H30Parser()
        self.assertEqual(parser.feed(packet(tlv(0x41, b"\0" * 12))), [])
        self.assertEqual(parser.stats["malformed_tlv"], 1)

    def test_duplicate_field_id_rejected(self):
        payload = tlv(0x51, struct.pack("<I", 1)) * 2
        parser = H30Parser()
        self.assertEqual(parser.feed(packet(payload)), [])
        self.assertEqual(parser.stats["malformed_tlv"], 1)

    def test_unknown_fields_retained_without_guess(self):
        frame = H30Parser().feed(packet(tlv(0xEE, b"YS\xff\x01")))[0]
        self.assertEqual(frame.raw_tlvs, ((0xEE, b"YS\xff\x01"),))
        self.assertEqual(frame.fields, {})
        self.assertEqual(frame.imu_dict()["orientation_covariance"][0], -1)

    def test_absent_quaternion_never_reuses_previous_frame(self):
        first, second = H30Parser().feed(imu_packet(1) + imu_packet(2, quat=False))
        self.assertTrue(first.imu_dict()["orientation_valid"])
        self.assertIsNone(second.imu_dict()["orientation"])
        self.assertEqual(second.imu_dict()["orientation_covariance"][0], -1)
        self.assertNotIn(0x41, second.present_ids)
        self.assertNotIn("orientation_xyzw", second.fields)

    def test_temperature_only_does_not_repeat_imu(self):
        _, second = H30Parser().feed(imu_packet() + packet(tlv(0x01, struct.pack("<h", -123))))
        self.assertAlmostEqual(second.fields["temperature_c"], -1.23)
        self.assertIsNone(second.imu_dict()["linear_acceleration"])
        self.assertIsNone(second.imu_dict()["angular_velocity"])

    def test_zero_quaternion_is_not_identity(self):
        frame = H30Parser().feed(packet(tlv(0x41, b"\0" * 16)))[0]
        self.assertIsNone(frame.imu_dict()["orientation"])
        self.assertFalse(frame.imu_dict()["orientation_valid"])

    def test_preserves_full_uint32_counter(self):
        frame = H30Parser().feed(packet(tlv(0x52, struct.pack("<I", 0xFFFF_FFFF))))[0]
        self.assertEqual(frame.fields["dataready_timestamp_raw"], 0xFFFF_FFFF)

    def test_input_mutation_cannot_change_parsed_packet(self):
        source = bytearray(imu_packet())
        frame = H30Parser().feed(source)[0]
        before = frame.raw_packet
        source[:] = b"\0" * len(source)
        self.assertEqual(frame.raw_packet, before)
        self.assertTrue(frame.imu_dict()["orientation_valid"])

    def test_noise_buffer_bounded_and_sync_preserved(self):
        parser = H30Parser()
        parser.feed(b"\0" * 10000 + b"Y")
        self.assertEqual(parser.buffered_bytes, 1)
        result = parser.feed(imu_packet()[1:])
        self.assertEqual(len(result), 1)

    def test_false_long_length_recovers_once_candidate_complete(self):
        parser = H30Parser()
        corrupted = b"YS\x01\x00\xff" + b"\0" * 257
        result = parser.feed(corrupted + imu_packet(17))
        self.assertEqual([r.tid for r in result], [17])
        self.assertLessEqual(parser.buffered_bytes, 262)

    def test_reset_discards_incomplete_epoch_data(self):
        parser = H30Parser()
        parser.feed(imu_packet()[:10])
        parser.reset()
        result = parser.feed(imu_packet(55))
        self.assertEqual([r.tid for r in result], [55])
        self.assertEqual(parser.stats["resets"], 1)

    def test_empty_packet_has_no_measurements(self):
        frame = H30Parser().feed(packet(b""))[0]
        self.assertEqual(frame.present_ids, frozenset())
        self.assertFalse(any(frame.imu_dict()["field_validity"].values()))


if __name__ == "__main__":
    unittest.main()
