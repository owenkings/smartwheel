"""现场外参几何的合成场景回归：竖直方向、找平、左右 4 自由度配准、平面手眼、判定辅助函数。"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_calibration.field import checks, geometry, motion
from wc_calibration.field.capture import wheel_track


def room_points(rng, n=6000):
    """一个 6×4×2.5 m 的房间：地面、四面墙，外加两个箱子，世界系 z 向上。"""
    faces = []
    for _ in range(n // 7):
        faces.append([rng.uniform(-1, 5), rng.uniform(-2, 2), 0.0])
        faces.append([5.0, rng.uniform(-2, 2), rng.uniform(0, 2.5)])
        faces.append([rng.uniform(-1, 5), 2.0, rng.uniform(0, 2.5)])
        faces.append([rng.uniform(-1, 5), -2.0, rng.uniform(0, 2.5)])
        faces.append([2.5, rng.uniform(0.3, 1.0), rng.uniform(0, 0.8)])            # 箱子正面
        faces.append([rng.uniform(2.5, 3.2), 1.0, rng.uniform(0, 0.8)])            # 箱子侧面
        faces.append([rng.uniform(3.0, 3.6), rng.uniform(-1.4, -0.8), 0.9])        # 桌面
    return np.asarray(faces)


def to_sensor(world, T_world_sensor):
    T = np.linalg.inv(T_world_sensor)
    return world @ T[:3, :3].T + T[:3, 3]


def pose(x, y, z, roll, pitch, yaw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def test_level_rotation_maps_up_to_z():
    up = Rotation.from_euler('xy', [3, -2], degrees=True).as_matrix().T @ [0, 0, 1]
    R = geometry.level_rotation(up)
    assert np.allclose(R @ up, [0, 0, 1], atol=1e-9)
    assert np.isclose(np.linalg.det(R), 1)


def test_up_from_wall_normals_recovers_tilt():
    rng = np.random.default_rng(0)
    tilt = Rotation.from_euler('xy', [1.5, -2.0], degrees=True).as_matrix()  # 雷达 <- 世界
    normals = []
    for a in rng.uniform(0, np.pi, 200):
        normals.append(tilt @ [np.cos(a), np.sin(a), 0] + rng.normal(0, 0.003, 3))
    up, uncertainty = geometry.up_from_wall_normals(np.asarray(normals))
    assert np.degrees(np.arccos(up @ (tilt @ [0, 0, 1]))) < 0.2
    assert uncertainty < 0.5


def test_up_from_wall_normals_rejects_too_few():
    with pytest.raises(ValueError):
        geometry.up_from_wall_normals(np.eye(3))


def test_vertical_plane_normals_finds_wall():
    rng = np.random.default_rng(1)
    wall = np.column_stack([np.full(2000, 3.0), rng.uniform(-1, 1, 2000), rng.uniform(-0.8, 0.8, 2000)])
    found = geometry.vertical_plane_normals(wall + rng.normal(0, 0.005, wall.shape))
    assert found and abs(abs(found[0][0]) - 1) < 1e-2


def test_left_right_4dof_recovers_synthetic_extrinsic():
    rng = np.random.default_rng(2)
    world = room_points(rng)
    T_left_right = pose(0.09, -0.46, -0.03, 0.4, 1.2, -4.5)
    up_left = np.array([0, 0, 1.0])
    up_right = T_left_right[:3, :3].T @ up_left
    pairs = []
    for x, y, yaw in ((0.0, 0.0, 0), (0.5, -0.5, 25), (1.0, 0.6, -30), (0.2, 0.8, 50)):
        T_world_left = pose(x, y, 0.8, 0, 0, yaw)
        left = to_sensor(world, T_world_left)
        right = to_sensor(world, T_world_left @ T_left_right)
        left, right = left[left[:, 0] > 0.8], right[right[:, 0] > 0.8]
        pairs.append(geometry.CloudPair(left, right))
    T = geometry.solve_left_right_4dof(pairs, up_left, up_right, np.radians(-3), [0.05, -0.45, 0.0])
    delta = np.linalg.inv(T_left_right) @ T
    assert np.linalg.norm(delta[:3, 3]) < 0.02
    assert np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()) < 0.3
    assert geometry.overlap_residual_mm(pairs, T) < geometry.overlap_residual_mm(pairs, checks.MECHANICAL_T_LEFT_RIGHT)


def test_planar_hand_eye_recovers_yaw():
    rng = np.random.default_rng(3)
    yaw_mount = np.radians(2.0)
    lever = np.array([0.3, 0.23])
    dt, steps = 0.1, 1500
    t = np.arange(steps) * dt
    v = 0.4 + 0.1 * np.sin(t / 7)
    omega = 0.5 * np.sin(t / 3)
    track = dict(t=t, v=v, omega=omega)
    base = motion.wheel_poses(track, t)
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler('z', yaw_mount).as_matrix()
    mount[:2, 3] = lever
    T = []
    for k in range(steps - 1):
        def world(p):
            M = np.eye(4)
            M[:3, :3] = Rotation.from_euler('z', p[2]).as_matrix()
            M[:2, 3] = p[:2]
            return M @ mount
        T.append(np.linalg.inv(world(base[k])) @ world(base[k + 1]))
    odometry = dict(T=np.stack(T), ok=np.ones(steps - 1, bool))
    result = motion.planar_hand_eye(odometry, t, track, np.array([0, 0, 1.0]), bootstrap=20)
    assert abs(result['yaw_deg'] - 2.0) < 0.1
    assert abs(result['wheel_translation_scale'] - 1.0) < 0.01
    assert np.allclose(result['xy_in_base_m'], lever, atol=0.03)


def fake_capture(speeds_l, speeds_r, dt=0.1):
    n = len(speeds_l)
    start = 10**9
    t = start + (np.arange(n) * dt * 1e9).astype(np.int64)
    return dict(manifest=dict(acquisition_window=dict(start_monotonic_ns=int(start),
                                                      end_monotonic_ns=int(t[-1]))),
                wheel=np.column_stack([t, speeds_l, speeds_r]).astype(float),
                wheel_cfg=dict(track_width_m=0.61))


def test_wheel_track_segments_and_distance():
    still = np.zeros(40)
    move = np.full(50, 0.5)
    capture = fake_capture(np.r_[still, move, still], np.r_[still, move, still])
    track = wheel_track(capture)
    assert len(track['segments']) == 2
    a, b = track['segments']
    assert abs(track['s'][b[0]] - track['s'][a[1]] - 2.5) < 0.1


def test_approach_agreement_and_robust_pool():
    assert checks.approach_agreement([0.938, 0.847, 0.921])[0]
    assert not checks.approach_agreement([0.938, 0.847])[0]
    rng = np.random.default_rng(4)
    stops = rng.normal(0, 0.01, (12, 6))
    stops[3] += 0.5
    kept, dropped = checks.robust_pool(stops)
    assert dropped == 1 and len(kept) == 11
