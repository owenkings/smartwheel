import pytest

from h30_imu_driver.mock_protocol import MockImuSample, encode_mock_packet, parse_mock_packet


def test_synthetic_packet_round_trip():
    sample = MockImuSample(1.25, 0.1, -0.2, 9.81, 0.01, 0.02, -0.03)
    parsed = parse_mock_packet(encode_mock_packet(sample))
    assert parsed.timestamp_sec == pytest.approx(sample.timestamp_sec)
    assert parsed.az == pytest.approx(sample.az, rel=1e-6)
    assert parsed.gz == pytest.approx(sample.gz, rel=1e-6)


def test_real_h30_packets_are_not_guessed():
    with pytest.raises(ValueError, match="synthetic"):
        parse_mock_packet(b"unknown vendor packet")

