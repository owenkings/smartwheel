import math

import numpy as np

from smartwheel_global_mapping import map_tf_owner, validate_tf_owners
from smartwheel_map_products import export_map_bundle, raycast_occupancy
from smartwheel_sensor_api import apply_transform
from smartwheel_sensor_api.pointcloud import transform_matrix
from smartwheel_sim import ClosedLoopTrajectory, IndoorScene, LidarModel


def test_deterministic_loop_generates_complete_map_bundle(tmp_path):
    seed = 20260714
    scene = IndoorScene(spacing_m=0.28)
    trajectory = ClosedLoopTrajectory()
    left_model = LidarModel((0.32, 0.18, 0.72), (0.0, math.radians(-4.0), math.radians(28.0)), max_points=180)
    right_model = LidarModel((0.32, -0.18, 0.72), (0.0, math.radians(-4.0), math.radians(-28.0)), max_points=180)
    all_points = []
    all_origins = []
    poses = []
    for index, time_sec in enumerate(np.linspace(0.0, 10.0, 70)):
        pose = trajectory.sample(float(time_sec), 10.0)
        map_from_base = transform_matrix([pose.x, pose.y, 0.0], [0.0, 0.0, pose.yaw])
        merged = []
        for model, offset in ((left_model, 11), (right_model, 29)):
            sensor_points, _ = model.observe(scene.points, pose, seed + offset, index)
            base_points = apply_transform(sensor_points, transform_matrix(model.xyz, model.rpy))
            merged.append(apply_transform(base_points, map_from_base))
        points = np.vstack(merged)
        all_points.append(points)
        all_origins.append(np.tile([pose.x, pose.y, 0.0], (points.shape[0], 1)))
        poses.append((float(time_sec), pose.x, pose.y, pose.yaw))
    points = np.vstack(all_points)
    origins = np.vstack(all_origins)
    keys = np.floor(points / 0.08).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    points, origins = points[first], origins[first]
    grid = raycast_occupancy(points, origins, resolution=0.05)
    colors = np.clip((points - points.min(axis=0)) * [17.0, 23.0, 31.0], 0, 255).astype(np.uint8)
    profile = tmp_path / "mock.yaml"
    profile.write_text("profile:\n  mode: mock\n  hardware_enabled: false\n", encoding="utf-8")
    output = export_map_bundle(
        tmp_path / "version",
        points,
        grid,
        poses,
        colors,
        str(profile),
        {"backend": "rtabmap", "mock_lio": True},
        "synthetic_bag",
        {"loop_error_m": math.hypot(poses[-1][1] - poses[0][1], poses[-1][2] - poses[0][2])},
    )
    assert (output / "manifest.json").stat().st_size > 0
    assert points[:, 0].min() < 1.0 and points[:, 0].max() > 13.0
    assert points[:, 1].min() < 1.0 and points[:, 1].max() > 8.0
    assert math.hypot(poses[-1][1] - poses[0][1], poses[-1][2] - poses[0][2]) < 1e-9
    validate_tf_owners([map_tf_owner("rtabmap")])
