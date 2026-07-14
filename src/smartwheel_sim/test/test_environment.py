import numpy as np

from smartwheel_sim.environment import IndoorScene, LidarModel
from smartwheel_sim.trajectory import ClosedLoopTrajectory


def test_scene_contains_all_required_feature_classes():
    scene = IndoorScene(spacing_m=0.25)
    assert scene.points.shape[0] > 1000
    assert set(scene.features) == {
        "rectangular_room",
        "l_shaped_corridor",
        "doorway",
        "column",
        "table_legs",
        "boxes_with_varied_height",
    }
    assert scene.points[:, 2].max() > 2.0
    assert np.any((scene.points[:, 2] > 0.6) & (scene.points[:, 2] < 1.0))


def test_trajectory_is_a_complete_closed_loop():
    trajectory = ClosedLoopTrajectory()
    start = trajectory.sample(0.0, 10.0)
    end = trajectory.sample(10.0, 10.0)
    assert start.x == end.x
    assert start.y == end.y
    assert trajectory.length_m > 30.0


def test_flash_lidar_is_narrow_fov_and_deterministic():
    scene = IndoorScene(spacing_m=0.3)
    pose = ClosedLoopTrajectory().sample(1.0, 10.0)
    model = LidarModel((0.3, 0.2, 0.7), (0.0, 0.0, 0.0), max_points=200)
    first, first_i = model.observe(scene.points, pose, 20260714, 3)
    second, second_i = model.observe(scene.points, pose, 20260714, 3)
    np.testing.assert_allclose(first, second)
    np.testing.assert_allclose(first_i, second_i)
    assert 0 < first.shape[0] <= 200
    angles = np.degrees(np.arctan2(first[:, 1], first[:, 0]))
    assert np.max(np.abs(angles)) <= 61.0


def test_occlusion_removes_visible_points():
    scene = IndoorScene(spacing_m=0.25)
    pose = ClosedLoopTrajectory().sample(2.0, 10.0)
    model = LidarModel((0.3, 0.0, 0.7), (0.0, 0.0, 0.0), max_points=10000)
    clear, _ = model.observe(scene.points, pose, 5, 1, occluded=False)
    blocked, _ = model.observe(scene.points, pose, 5, 1, occluded=True)
    assert blocked.shape[0] < clear.shape[0]
