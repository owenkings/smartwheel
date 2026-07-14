import numpy as np

from smartwheel_map_products.colorizer import CameraFrame, colorize_points


def camera(image):
    intrinsic = np.array([[10.0, 0.0, 5.0], [0.0, 10.0, 5.0], [0.0, 0.0, 1.0]])
    return CameraFrame(image, intrinsic, np.eye(4))


def test_projection_rejects_points_behind_camera():
    image = np.zeros((11, 11, 3), dtype=np.uint8)
    image[5, 5] = [10, 20, 30]
    colors, selected = colorize_points(np.array([[0.0, 0.0, 2.0], [0.0, 0.0, -2.0]]), [camera(image)])
    np.testing.assert_array_equal(colors[0], [10, 20, 30])
    np.testing.assert_array_equal(colors[1], [128, 128, 128])
    assert selected.tolist() == [True, False]


def test_z_buffer_excludes_occluded_point_on_same_pixel():
    image = np.full((11, 11, 3), [100, 110, 120], dtype=np.uint8)
    points = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 3.0]])
    _, selected = colorize_points(points, [camera(image)], occlusion_tolerance_m=0.01)
    assert selected.tolist() == [True, False]

