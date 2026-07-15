import numpy as np

from smartwheel_workbench.cloud_color import colors_for_cloud, pack_rgb, xyzrgb_message
from std_msgs.msg import Header


def test_color_priority_rgb_then_intensity_then_height():
    z = np.array([0.0, 1.0], dtype=np.float32)
    rgb = np.array([0x00112233, 0x00AABBCC], dtype=np.uint32)
    np.testing.assert_array_equal(colors_for_cloud(z, packed_rgb=rgb, intensity=np.array([0, 1])), rgb)
    intensity_colors = colors_for_cloud(z, intensity=np.array([0.0, 100.0]))
    assert intensity_colors[0] != intensity_colors[1]
    height_colors = colors_for_cloud(z)
    assert height_colors[0] != height_colors[1]


def test_xyzrgb_message_has_standard_packed_rgb_field():
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    colors = pack_rgb(np.array([255, 1]), np.array([2, 3]), np.array([4, 5]))
    message = xyzrgb_message(Header(frame_id="map"), xyz, colors)
    assert message.width == 2
    assert message.point_step == 16
    assert [field.name for field in message.fields] == ["x", "y", "z", "rgb"]
    records = np.frombuffer(message.data, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    np.testing.assert_array_equal(records["rgb"], colors)
