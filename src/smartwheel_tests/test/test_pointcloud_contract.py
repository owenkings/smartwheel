import numpy as np
import pytest
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

from smartwheel_sensor_api import pointcloud2_from_xyz, pointcloud2_to_xyz, validate_pointcloud_fields


def test_pointcloud2_fields_and_reader_round_trip():
    header = Header(frame_id="xtm60_left_link")
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    intensity = np.array([10.0, 20.0])
    message = pointcloud2_from_xyz(header, points, intensity)
    assert [field.name for field in message.fields] == ["x", "y", "z", "intensity"]
    decoded, decoded_intensity = pointcloud2_to_xyz(message)
    np.testing.assert_allclose(decoded, points)
    np.testing.assert_allclose(decoded_intensity, intensity)


def test_missing_pointcloud_field_is_rejected():
    message = pointcloud2_from_xyz(Header(frame_id="lidar"), np.array([[1.0, 0.0, 0.0]]))
    message.fields = [field for field in message.fields if field.name != "z"]
    with pytest.raises(ValueError, match="missing required fields"):
        validate_pointcloud_fields(message)
