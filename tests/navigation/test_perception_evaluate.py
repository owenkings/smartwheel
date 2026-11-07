"""Future-path check of the offline obstacle evaluation: recorded axle path, swept outline, scan baseline."""
import math

import numpy as np

from wc_navigation.perception import evaluate as E

MEASURED = [(0.81, 0.2425), (0.2, 0.35), (-0.255, 0.35), (-0.255, -0.35), (0.2, -0.35), (0.81, -0.2425)]
LEVER = (0.323, 0.22875)


def straight_path(speed=0.5, duration=10.0, dt=0.2):
    """Lidar poses for an axle driving straight along +x from x = 0, as the frontend stores them."""
    t = np.arange(0.0, duration + 1e-9, dt)
    axle = np.column_stack([speed * t, np.zeros_like(t), np.zeros_like(t)])
    lidar = axle + [LEVER[0], LEVER[1], 0.0]
    return (t * 1e9).astype(np.int64) + 10 ** 9, lidar


def test_axle_path_undoes_the_lever_arm_and_interpolates():
    stamps, lidar = straight_path()
    path = E.AxlePath(stamps, lidar, LEVER)
    assert np.allclose(path.poses[0], [0.0, 0.0, 0.0])
    assert np.allclose(path.at(1.1)[0], [0.55, 0.0, 0.0])
    assert math.isclose(path.odometer(4.0), 2.0)


def test_points_on_the_future_path_are_hit_and_points_beside_it_are_not():
    stamps, lidar = straight_path()
    path = E.AxlePath(stamps, lidar, LEVER)
    world = np.array([[2.0, 0.0], [2.0, 0.6], [-0.5, 0.0]])     # ahead, beside, behind
    hit = E.swept_hits(world, path, 0.0, MEASURED, horizon_s=4.0)
    assert hit.tolist() == [True, False, False]
    # beyond the horizon: 0.5 m/s * 1 s plus the 0.81 m front does not reach 2 m
    assert not E.swept_hits(world[:1], path, 0.0, MEASURED, horizon_s=1.0)[0]


def test_a_chair_that_does_not_move_sweeps_nothing():
    t = np.arange(0.0, 6.0, 0.2)
    lidar = np.tile([LEVER[0], LEVER[1], 0.0], (len(t), 1))
    path = E.AxlePath((t * 1e9).astype(np.int64) + 10 ** 9, lidar, LEVER)
    assert not E.swept_hits(np.array([[0.7, 0.0]]), path, 0.0, MEASURED, horizon_s=4.0).any()


def test_world_and_base_frames_round_trip():
    pose = (1.0, -2.0, 0.7)
    p = np.array([[0.4, 0.1], [-1.0, 2.0]])
    assert np.allclose(E.to_base(E.to_world(p, pose), pose), p)


def test_scan_points_follow_the_shadow_scan_projection():
    pts = np.array([[2.0, 0.0, 0.0], [3.0, 0.0, 0.0], [2.0, 0.5, -0.7]])   # last one below the 2D band
    xy = E.scan_points(pts, LEVER)
    assert len(xy) == 1 and np.allclose(xy[0], [LEVER[0] + 2.0, LEVER[1]], atol=0.02)
