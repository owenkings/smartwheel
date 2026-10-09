from dataclasses import replace
import unittest

from wc_sensors.preflight import SensorEndpoint, validate_endpoints


def endpoints():
    return [SensorEndpoint("left", "XTM60B00000000000012", "192.168.0.101", "192.168.0.100", 7687),
            SensorEndpoint("right", "XTM60B00000000000013", "192.168.1.101", "192.168.1.100", 7687)]


class EndpointTests(unittest.TestCase):
    def test_historical_same_port_is_unknown_and_rejected(self):
        with self.assertRaises(ValueError):
            validate_endpoints(endpoints())
        with self.assertRaises(ValueError):
            validate_endpoints(endpoints(), allow_same_port_verified_bind=True)

    def test_independent_ports_only_static_eligibility(self):
        devices = endpoints()
        devices[1] = replace(devices[1], receive_port=7688)
        status = validate_endpoints(devices)
        self.assertEqual(status["status"], "STATIC_ELIGIBLE")
        self.assertEqual(status["hardware_validation"], "NOT_RUN")
        self.assertEqual(status["identity_readback"], "UNKNOWN")

    def test_same_port_requires_concrete_binds_and_source_filters(self):
        devices = [replace(item, bind_ip=item.receive_ip, bind_source_verified=True,
                           source_filter_verified=True) for item in endpoints()]
        result = validate_endpoints(devices, allow_same_port_verified_bind=True)
        self.assertTrue(result["same_port_exception"])
        for key, value in (("bind_ip", "0.0.0.0"), ("bind_source_verified", False),
                           ("source_filter_verified", False)):
            modified = [devices[0], replace(devices[1], **{key: value})]
            with self.assertRaises(ValueError):
                validate_endpoints(modified, allow_same_port_verified_bind=True)

    def test_duplicate_sensor_id_rejected_even_with_distinct_ip(self):
        devices = endpoints()
        devices[1] = replace(devices[1], sensor_id=devices[0].sensor_id, receive_port=7688)
        with self.assertRaises(ValueError):
            validate_endpoints(devices)

    def test_duplicate_device_ip_rejected(self):
        devices = endpoints()
        devices[1] = replace(devices[1], device_ip=devices[0].device_ip, receive_port=7688)
        with self.assertRaises(ValueError):
            validate_endpoints(devices)

    def test_missing_side_and_silent_single_degradation_rejected(self):
        with self.assertRaises(ValueError):
            validate_endpoints(endpoints()[:1])
        result = validate_endpoints(endpoints()[:1], source_mode="single_left")
        self.assertEqual(result["source_mode"], "single_left")

    def test_duplicate_bind_tuple_rejected(self):
        left, right = endpoints()
        devices = [replace(left, bind_ip=left.receive_ip, bind_source_verified=True, source_filter_verified=True),
                   replace(right, receive_ip=left.receive_ip, bind_ip=left.receive_ip,
                           bind_source_verified=True, source_filter_verified=True)]
        with self.assertRaises(ValueError):
            validate_endpoints(devices, allow_same_port_verified_bind=True)

    def test_string_verification_flags_cannot_bypass_policy(self):
        devices = endpoints()
        devices[0] = replace(devices[0], bind_source_verified="false")
        with self.assertRaises(TypeError):
            validate_endpoints(devices, allow_same_port_verified_bind=True)


if __name__ == "__main__":
    unittest.main()
