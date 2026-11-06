"""Simulator ground truth: ray casting, collision, odometry error and base limits."""
import math

import numpy as np
import pytest

from wc_navigation.autonomy.kinematics import BaseLimits, DiffDriveSim, OdometryErrorModel, integrate
from wc_navigation.autonomy.world import CircleObstacle, LaserModel, SimWorld, synthetic_names, synthetic_world
from wc_navigation.geometry import Grid


def box_world(size=4.0, resolution=0.05, obstacles=()):
    n = int(round(size / resolution))
    data = np.zeros((n, n), dtype=np.int8)
    data[:2, :] = data[-2:, :] = data[:, :2] = data[:, -2:] = 100  # 0.1 m walls
    return SimWorld(Grid(data, resolution, (0.0, 0.0, 0.0)), obstacles)


def test_raycast_distance_to_axis_aligned_walls():
    world = box_world()
    ranges = world.raycast((2.0, 2.0), [0.0, math.pi / 2, math.pi, -math.pi / 2], 8.0)
    # Inner wall faces are at 0.1 and 3.9 m; quarter-cell sampling error <= 6.25 mm.
    assert np.allclose(ranges, 1.9, atol=0.007)


def test_diagonal_wall_of_corner_touching_cells_does_not_leak():
    data = np.zeros((40, 40), dtype=np.int8)
    for k in range(40):
        data[k, k] = 100  # cells touch only at corners
    world = SimWorld(Grid(data, 0.05, (0.0, 0.0, 0.0)))
    # Rays from below the diagonal toward the upper-left must stop at the diagonal.
    ranges = world.raycast((1.5, 0.5), np.linspace(math.radians(100), math.radians(170), 15), 3.0)
    assert np.isfinite(ranges).all() and ranges.max() < 1.5


def test_outside_map_counts_as_solid_and_no_return_is_inf():
    data = np.zeros((20, 20), dtype=np.int8)
    world = SimWorld(Grid(data, 0.05, (0.0, 0.0, 0.0)))
    assert world.raycast((0.5, 0.5), [0.0], 8.0)[0] == pytest.approx(0.5, abs=0.007)
    assert np.isinf(box_world(size=20.0).raycast((10.0, 10.0), [0.3], 2.0)[0])
    assert world.collides(0.1, 0.5, 0.2)


def test_disc_obstacles_hit_rays_and_reduce_clearance():
    world = box_world(obstacles=[CircleObstacle(3.0, 2.0, 0.25)])
    assert world.raycast((2.0, 2.0), [0.0], 8.0)[0] == pytest.approx(0.75, abs=1e-6)
    assert world.clearance(2.0, 2.0) == pytest.approx(0.75, abs=1e-6)
    assert world.collides(2.5, 2.0, 0.3)


def test_scan_follows_rep117_for_close_and_missing_returns():
    world = box_world()
    laser = LaserModel(mount_x=0.0, mount_y=0.0, angle_min=-0.1, angle_max=0.1, beams=3,
                       range_min=0.3, range_max=1.0, noise_std=0.0)
    far = world.scan((2.0, 2.0, 0.0), laser)
    assert np.isinf(far).all() and (far > 0).all()      # wall at 1.9 m > range_max
    close = world.scan((3.75, 2.0, 0.0), laser)
    assert np.isneginf(close).all()                      # wall at 0.15 m < range_min


def test_laser_mount_offsets_the_ray_origin():
    world = box_world()
    laser = LaserModel(mount_x=0.5, mount_y=0.0, angle_min=-0.01, angle_max=0.01, beams=2, noise_std=0.0)
    ranges = world.scan((2.0, 2.0, 0.0), laser)
    assert np.allclose(ranges, 1.4, atol=0.01)


def test_swept_segment_collision_is_not_skipped():
    world = box_world()
    assert world.segment_collides((1.0, 2.0), (3.9, 2.0), 0.3)
    assert not world.segment_collides((1.0, 2.0), (3.0, 2.0), 0.3)


@pytest.mark.parametrize("name", synthetic_names())
def test_synthetic_worlds_are_deterministic_with_free_start(name):
    a, start = synthetic_world(name)
    b, _ = synthetic_world(name)
    assert np.array_equal(a.solid, b.solid)
    assert a.clearance(*start[:2]) >= 0.65  # assumed 0.55 m body + 0.10 m margin
    assert a.solid[0, :].all() and a.solid[:, -1].all()


def test_office_corridor_is_passable_for_the_planning_envelope():
    from scipy import ndimage
    world, start = synthetic_world("office_floor")
    centers = world.clearance_grid() >= 0.65
    labels, _ = ndimage.label(centers, structure=np.ones((3, 3), dtype=bool))
    col, row = int(start[0] / 0.05), int(start[1] / 0.05)
    reachable = labels == labels[row, col]
    assert reachable[int(6.0 / 0.05), int(18.0 / 0.05)]      # east end of the corridor
    assert reachable[int(2.0 / 0.05), int(15.0 / 0.05)]      # south-east room


def test_clearance_grid_includes_disc_obstacles():
    world = box_world(obstacles=[CircleObstacle(2.0, 2.0, 0.3)])
    grid = world.clearance_grid()
    row = col = int(2.0 / 0.05)
    assert grid[row, col] == 0.0 and grid[row, col + 10] == pytest.approx(0.2, abs=0.04)


def test_world_requires_axis_aligned_grid():
    with pytest.raises(ValueError):
        SimWorld(Grid(np.zeros((4, 4), dtype=np.int8), 0.05, (0.0, 0.0, 0.3)))


def test_base_respects_acceleration_and_command_timeout():
    base = DiffDriveSim((0.0, 0.0, 0.0), BaseLimits(linear_accel=0.5, command_timeout_s=0.5))
    base.set_command(0.6, 0.0, 0.0)
    base.step(0.1, 0.0)
    assert base.v == pytest.approx(0.05)
    t = 0.1
    while t < 3.0:
        base.step(0.01, t)
        t += 0.01
    assert base.v == pytest.approx(0.0)  # command older than 0.5 s brakes to zero
    base.set_command(float("nan"), 1.0, t)
    assert base.command == (0.0, 0.0)


def test_collision_vetoes_motion_and_counts_contact_onset():
    world = box_world()
    base = DiffDriveSim((3.0, 2.0, 0.0))
    t = 0.0
    for _ in range(400):
        base.set_command(0.5, 0.0, t)
        base.step(0.01, t, lambda a, b: world.segment_collides(a, b, 0.55))
        t += 0.01
    assert base.collisions >= 1
    assert world.clearance(*base.pose[:2]) >= 0.55 - 1e-9


def test_unknown_odometry_preset_is_rejected():
    with pytest.raises(ValueError):
        OdometryErrorModel.preset("default")


def test_gyro_fused_preset_drifts_less_in_heading_than_wheel_only():
    finals = {}
    for preset in ("wheel_imu", "wheel_only"):
        base = DiffDriveSim((0.0, 0.0, 0.0), odometry=OdometryErrorModel.preset(preset, seed=3))
        t = 0.0
        for _ in range(3000):  # 30 s turning in place
            base.set_command(0.0, 0.4, t)
            base.step(0.01, t)
            t += 0.01
        finals[preset] = abs(math.atan2(math.sin(base.odom_pose[2] - base.pose[2]),
                                        math.cos(base.odom_pose[2] - base.pose[2])))
    assert finals["wheel_imu"] < finals["wheel_only"] / 2


def test_odometry_error_model_drifts_but_ideal_matches_truth():
    for preset, exact in (("ideal", True), ("wheel_imu", False), ("wheel_only", False)):
        base = DiffDriveSim((1.0, 1.0, 0.0), odometry=OdometryErrorModel.preset(preset, seed=7))
        t = 0.0
        for _ in range(1000):
            base.set_command(0.3, 0.05, t)
            base.step(0.01, t)
            t += 0.01
        moved = (base.pose[0] - 1.0, base.pose[1] - 1.0)
        error = math.dist(moved, base.odom_pose[:2])
        assert (error < 1e-9) is exact


def test_exact_arc_integration():
    x, y, yaw = integrate((0.0, 0.0, 0.0), 1.0, math.pi / 2, 1.0)
    assert (x, y, yaw) == pytest.approx((2 / math.pi, 2 / math.pi, math.pi / 2))
