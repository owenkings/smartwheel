"""Simulator ground truth: ray casting, collision, odometry error and base limits."""
import math

import numpy as np
import pytest

from wc_navigation.autonomy.kinematics import BaseLimits, DiffDriveSim, OdometryErrorModel, integrate
from wc_navigation.autonomy.world import (CircleObstacle, LaserModel, SimWorld, ghost_discs, ghost_scenario,
                                          synthetic_names, synthetic_world)
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


def _ghost_scenario(*ghosts):
    return {"schema_version": 1, "name": "test", "ghosts": list(ghosts)}


def test_ghost_returns_are_in_the_scan_but_not_solid():
    world = box_world()
    laser = LaserModel(mount_x=0.0, mount_y=0.0, angle_min=-0.1, angle_max=0.1, beams=3, noise_std=0.0)
    pose = (1.0, 2.0, 0.0)
    ghosts = ghost_scenario(_ghost_scenario({"id": "g", "frame": "chair", "points": [[1.0, 0.0]]}))
    ranges, hit = world.scan_with_ghosts(pose, laser, None, ghost_discs(ghosts, pose, pose))
    assert ranges[1] == pytest.approx(0.97, abs=1e-6) and hit.tolist() == [False, True, False]
    assert np.allclose(ranges[[0, 2]], world.scan(pose, laser)[[0, 2]])     # the beams beside it see the wall
    assert not world.collides(2.0, 2.0, 0.3) and not world.obstacles          # its place stays free
    too_close, hit = world.scan_with_ghosts(pose, laser, None, ghost_discs(
        ghost_scenario(_ghost_scenario({"id": "g", "frame": "chair", "points": [[0.2, 0.0]]})), pose, pose))
    assert np.isneginf(too_close[1]) and not hit.any()                         # inside range_min: no return


def test_start_ghosts_stay_in_the_world_and_chair_ghosts_follow():
    ghosts = ghost_scenario(_ghost_scenario({"id": "fixed", "frame": "start", "points": [[1.5, 0.1]]},
                                            {"id": "band", "frame": "chair", "radius_m": 0.05, "points": [[1.2, 0.0]]}))
    start = (-0.3, -0.43, math.pi)
    at_start = ghost_discs(ghosts, start, start)
    assert at_start[0] == pytest.approx([-1.8, -0.53, 0.03]) and at_start[1] == pytest.approx([-1.5, -0.43, 0.05])
    moved = ghost_discs(ghosts, (-1.3, -0.43, math.pi / 2), start)
    assert moved[0] == pytest.approx(at_start[0])                             # fixed in the world
    assert moved[1, :2] == pytest.approx([-1.3, 0.77])                        # 1.2 m ahead of the turned chair


def test_runs_without_ghosts_are_unchanged_and_with_ghosts_carry_the_scenario(tmp_path):
    import json
    from pathlib import Path
    from wc_navigation.autonomy.bringup import Supervisor
    from wc_navigation.autonomy.cli import arguments, build_config
    project = Path(__file__).resolve().parents[2]
    base_args = ["--project-root", str(project), "navigate", "--world", "two_rooms", "--start", "1.2", "1.4", "0.0",
                 "--goal", "8.0", "1.4", "0.0"]
    plain = build_config(arguments(base_args))
    assert "ghosts" not in plain
    path = tmp_path / "ghosts.json"
    path.write_text(json.dumps(_ghost_scenario({"id": "band", "frame": "chair", "points": [[1.2, 0.0], [1.2, 0.05]]})))
    config = build_config(arguments(base_args + ["--ghosts", str(path)]))
    assert config["ghosts"]["scenario"]["sha256"] and config["ghosts"]["ghosts"][0]["radius_m"] == 0.03
    assert {k: v for k, v in config.items() if k != "ghosts"} == plain
    out = tmp_path / "run"
    out.mkdir()
    Supervisor(project, out, config).prepare()
    sim_config = json.loads((out / "sim_config.json").read_text())
    assert sim_config["ghosts"] == config["ghosts"]["ghosts"] and "ghosts" not in sim_config["world"]
    path.write_text(json.dumps(_ghost_scenario({"id": "band", "frame": "map", "points": [[1.2, 0.0]]})))
    with pytest.raises(SystemExit):
        arguments(base_args + ["--ghosts", str(path)])


@pytest.mark.parametrize("bad", [
    {"schema_version": 2}, {"name": ""}, {"ghosts": []}, {"colour": "red"},
    {"ghosts": [{"id": "g", "frame": "map", "points": [[1.0, 0.0]]}]},
    {"ghosts": [{"id": "g", "frame": "chair", "radius_m": 0.5, "points": [[1.0, 0.0]]}]},
    {"ghosts": [{"id": "g", "frame": "chair", "points": []}]},
    {"ghosts": [{"id": "g", "frame": "chair", "points": [[1.0, 0.0, 0.5]]}]},
    {"ghosts": [{"id": "g", "frame": "chair", "points": [[1.0, float("nan")]]}]},
    {"ghosts": [{"id": "g", "frame": "chair", "points": [[1.0, 0.0]], "height_m": 0.8}]},
    {"ghosts": [{"id": "g", "frame": "chair", "points": [[1.0, 0.0]]},
                {"id": "g", "frame": "start", "points": [[2.0, 0.0]]}]},
])
def test_ghost_scenarios_are_checked(bad):
    value = dict(_ghost_scenario({"id": "g", "frame": "chair", "points": [[1.0, 0.0]]}), **bad)
    with pytest.raises(ValueError):
        ghost_scenario(value)


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


# -- measured wheel response (--drive-response zlac8030d_20261111) -------------------------------------
def _drive(steps, dt=0.01):
    """Run a WheelDrive through [(seconds, left target, right target), ...]; returns (drive, samples)."""
    from wc_navigation.autonomy.kinematics import WheelDrive, WheelResponse
    drive, samples, t = WheelDrive(WheelResponse()), [], 0.0
    for seconds, left, right in steps:
        for _ in range(int(round(seconds / dt))):
            samples.append((t,) + drive.step(left, right, dt, t))
            t += dt
    return drive, samples


def test_wheel_response_turning_in_place_holds_the_right_wheel_back():
    word = 0.00193                                        # m/s per feedback word (0.112 rpm, 0.1646 m wheel)
    drive, samples = _drive([(3.0, 25 * word, -25 * word)])
    assert samples[-1][1] == pytest.approx(0.93 * 25 * word, rel=0.05)
    assert samples[-1][2] == pytest.approx(-0.37 * 25 * word, rel=0.05)
    assert drive.pivot_s == pytest.approx(3.0, abs=0.02)
    _, straight = _drive([(3.0, 52 * word, 52 * word)])  # both wheels the same way: no shortfall
    assert straight[-1][1] == pytest.approx(52 * word) and straight[-1][2] == pytest.approx(52 * word)


def test_wheel_response_overshoots_after_a_pivot_only_in_the_wound_up_direction():
    word = 0.00193
    # Turning left in place (right wheel forward) for 4 s, then straight ahead: the right wheel hangs back for
    # stuck_s and then overshoots far past the target (11-11: 294-304 words for a 52-29 word target).
    drive, samples = _drive([(4.0, -31 * word, 31 * word), (2.0, 52 * word, 52 * word)])
    after = [s for s in samples if s[0] >= 4.0]
    stuck = [s[2] for s in after if s[0] < 4.0 + 0.6]
    assert max(stuck) < 0.5 * 52 * word
    assert max(s[2] for s in after) > 4 * 52 * word
    assert drive.releases == 1 and len(drive.overspeed) == 1
    assert after[-1][2] == pytest.approx(52 * word, rel=0.1)
    # Turning right (right wheel backward), then straight ahead: no overshoot, the windup is dropped.
    drive, samples = _drive([(4.0, 31 * word, -31 * word), (2.0, 52 * word, 52 * word)])
    assert max(s[2] for s in samples if s[0] >= 4.0) < 1.1 * 52 * word
    assert drive.overspeed == [] and drive.windup == 0.0


def test_wheel_response_is_slow_to_start_at_low_speed():
    word = 0.00193
    _, slow = _drive([(0.5, 12 * word, 12 * word)])
    _, fast = _drive([(0.5, 52 * word, 52 * word)])
    assert slow[-1][1] / (12 * word) < 0.6                # 11-11: 12 words from rest, half speed after 0.8 s
    assert fast[-1][1] / (52 * word) > 0.95


def test_base_with_wheel_response_ramps_like_the_wheel_owner_and_turns_slower_in_place():
    from wc_navigation.autonomy.kinematics import (DRIVE_RESPONSES, WheelDrive, drive_response_config)
    assert drive_response_config("ideal") is None
    config = drive_response_config("zlac8030d_20261111")
    response, ramps = DRIVE_RESPONSES["zlac8030d_20261111"]
    assert config == {"wheel_response": response.as_dict(), "base_limits": ramps}
    with pytest.raises(ValueError):
        drive_response_config("unknown")
    ideal = DiffDriveSim((0.0, 0.0, 0.0), BaseLimits(**ramps))
    measured = DiffDriveSim((0.0, 0.0, 0.0), BaseLimits(**ramps), wheels=WheelDrive(response))
    t = 0.0
    for _ in range(400):                                   # 4 s of 0.3 rad/s in place
        for base in (ideal, measured):
            base.set_command(0.0, 0.3, t)
            base.step(0.01, t)
        t += 0.01
    assert ideal.pose[2] == pytest.approx(1.2 - 0.3 ** 2 / (2 * 0.8), abs=0.01)
    turned = measured.pose[2] / ideal.pose[2]
    assert 0.55 < turned < 0.75                           # (0.93 + 0.37) / 2 of the turn rate
    assert measured.pose[0] < -0.01                       # left turn with the right wheel short: drifts backward
    assert DiffDriveSim((0.0, 0.0, 0.0)).wheels is None   # default unchanged


# -- convex body outline (same polygon as NavProfile.footprint) ----------------------------------
# Front from the blind-zone recording, half width and rear are examples (FRONT_FROM_BLIND_RECORDING_REST_EXAMPLE).
OUTLINE = ((0.79, 0.35), (-0.35, 0.35), (-0.35, -0.35), (0.79, -0.35))


def outline_body():
    from wc_navigation.planning.body import Body
    return Body(outline=OUTLINE, source="POLYGON_FRONT_FROM_BLIND_RECORDING_REST_EXAMPLE")


def _sampled_overlap(polygon, solid, resolution, step):
    """Reference: any sample strictly inside a solid cell lies strictly inside the polygon."""
    from wc_navigation.planning.body import _edge_normals
    rows, cols = np.nonzero(solid)
    if not len(rows):
        return False
    offsets = np.arange(step / 2, resolution, step)
    ox, oy = np.meshgrid(offsets, offsets)
    xs = (cols[:, None] * resolution + ox.ravel()[None, :]).ravel()
    ys = (rows[:, None] * resolution + oy.ravel()[None, :]).ravel()
    normals, c = _edge_normals(polygon)
    return bool(np.any(np.all(np.column_stack([xs, ys]) @ normals.T < c, axis=1)))


def test_outline_cell_test_agrees_with_dense_sampling():
    from wc_navigation.planning.body import grow_outline, polygon_hits_cells
    rng = np.random.default_rng(5)
    body = outline_body()
    hits = misses = 0
    for _ in range(200):
        solid = rng.random((50, 50)) < 0.002
        pose = (rng.uniform(0.9, 1.6), rng.uniform(0.9, 1.6), rng.uniform(-math.pi, math.pi))
        polygon = body.outline_at(pose)
        hit = polygon_hits_cells(polygon, solid, 0.05, (0.0, 0.0))
        if hit:   # a hit must survive sampling of a slightly grown body, a miss must hold for a shrunk one
            assert _sampled_overlap(grow_outline(polygon, 0.004), solid, 0.05, 0.0025)
            hits += 1
        else:
            assert not _sampled_overlap(grow_outline(polygon, -0.004), solid, 0.05, 0.0025)
            misses += 1
    assert hits > 20 and misses > 20


def test_outline_leaving_the_grid_counts_as_a_collision():
    from wc_navigation.planning.body import polygon_hits_cells
    solid = np.zeros((40, 40), dtype=bool)
    body = outline_body()
    assert not polygon_hits_cells(body.outline_at((1.0, 1.0, 0.0)), solid, 0.05, (0.0, 0.0))
    assert polygon_hits_cells(body.outline_at((1.3, 1.0, 0.0)), solid, 0.05, (0.0, 0.0))   # front at x = 2.09 m


def test_outline_fits_a_passage_the_circle_does_not():
    data = np.zeros((40, 120), dtype=np.int8)
    data[:11, :] = data[29:, :] = 100                  # free band y in [0.55, 1.45): 0.90 m wide
    world = SimWorld(Grid(data, 0.05, (0.0, 0.0, 0.0)))
    from wc_navigation.planning.body import Body
    start, end = (1.0, 1.0, 0.0), (4.0, 1.0, 0.0)
    assert not world.swept_body_collides(start, end, outline_body())          # 0.10 m each side
    assert world.swept_body_collides(start, end, Body(radius_m=0.55))
    assert world.body_clearance(start, outline_body()) == pytest.approx(0.10, abs=1e-9)
    assert world.body_clearance(start, Body(radius_m=0.40)) == pytest.approx(0.05, abs=1e-9)


def test_turning_in_place_sweeps_the_long_front_into_a_side_wall():
    data = np.zeros((60, 60), dtype=np.int8)
    data[41:, :] = 100                                 # wall face at y = 2.05 m
    world = SimWorld(Grid(data, 0.05, (0.0, 0.0, 0.0)))
    from wc_navigation.planning.body import Body
    pose = (1.5, 1.5, 0.0)                             # left side 0.20 m from the wall, front 0.79 m ahead
    assert not world.body_collides(pose, outline_body())
    assert world.swept_body_collides(pose, (1.5, 1.5, math.pi / 2), outline_body())
    assert not world.swept_body_collides(pose, (1.5, 1.5, math.pi / 2), Body(radius_m=0.35))   # inscribed circle
    assert world.swept_body_collides(pose, (1.5, 1.5, -0.6), outline_body()) is False          # away from it


def test_swept_outline_does_not_skip_a_thin_wall():
    data = np.zeros((40, 80), dtype=np.int8)
    data[:, 50] = 100                                  # one cell thick, x in [2.50, 2.55)
    world = SimWorld(Grid(data, 0.05, (0.0, 0.0, 0.0)))
    body = outline_body()
    start, end = (1.0, 1.0, 0.0), (3.0, 1.0, 0.0)      # the whole body jumps over the wall in one step
    assert not world.body_collides(start, body) and not world.body_collides(end, body)
    assert world.swept_body_collides(start, end, body)


def test_outline_collides_with_disc_obstacles():
    world = box_world(obstacles=[CircleObstacle(2.85, 2.40, 0.1)])
    body = outline_body()
    assert world.body_collides((2.0, 2.0, 0.0), body)                   # front-left corner (2.79, 2.35) is inside
    assert not world.collides(2.0, 2.0, 0.35)                           # the inscribed circle would miss it
    assert not world.body_collides((2.0, 1.7, 0.0), body)
    assert world.body_clearance((2.0, 1.7, 0.0), body) == pytest.approx(math.hypot(0.06, 0.35) - 0.1, abs=1e-9)


def test_circle_body_keeps_the_original_segment_check():
    from wc_navigation.planning.body import Body
    world = box_world()
    circle = Body(radius_m=0.3)
    for start, end in (((1.0, 2.0, 0.0), (3.9, 2.0, 1.0)), ((1.0, 2.0, 0.0), (3.0, 2.0, -2.0))):
        assert world.swept_body_collides(start, end, circle) == world.segment_collides(start, end, 0.3)


def test_base_with_outline_stops_with_the_front_before_the_wall():
    world = box_world()                                 # inner wall faces at 0.1 and 3.9 m
    body = outline_body()
    base = DiffDriveSim((2.5, 2.0, 0.0))
    t = 0.0
    for _ in range(600):
        base.set_command(0.5, 0.0, t)
        base.step(0.01, t, lambda a, b: world.swept_body_collides(a, b, body))
        t += 0.01
    assert base.collisions >= 1
    front_gap = 3.9 - (base.pose[0] + 0.79)
    assert 0.0 <= front_gap < 0.02
    assert world.body_clearance(base.pose, body) == pytest.approx(front_gap, abs=1e-9)


def test_body_validation_and_config_round_trip():
    from wc_navigation.planning.body import Body
    body = outline_body()
    assert body.inscribed_m == pytest.approx(0.35)
    assert body.circumscribed_m == pytest.approx(math.hypot(0.79, 0.35))
    assert Body.from_config(body.as_config()) == body
    assert Body(outline=tuple(reversed(OUTLINE))).outline == body.outline       # stored counter-clockwise
    for bad in (((0.5, 0.3), (-0.2, 0.3), (0.5, -0.3), (-0.2, -0.3)),        # self-crossing
                ((0.5, 0.3), (0.1, 0.3), (0.1, -0.3), (0.5, -0.3)),          # axle outside
                ((0.5, 0.3), (0.0, 0.0), (-0.2, 0.3))):                       # axle on the outline
        with pytest.raises(ValueError):
            Body(outline=bad)
    with pytest.raises(ValueError):
        Body.from_config({"type": "box"})


def test_simulator_body_comes_from_the_config():
    from wc_navigation.autonomy.sim_node import collision_body
    legacy = collision_body({"collision_radius_m": 0.55})
    assert not legacy.is_polygon and legacy.radius_m == 0.55
    assert collision_body({"collision_radius_m": 0.55, "collision_body": outline_body().as_config()}) == outline_body()


def test_trajectory_gap_uses_the_outline():
    from wc_navigation.autonomy.report import trajectory_metrics
    world = box_world()
    rows = [[float(k), 2.5, 2.0, 0.0, *([float("nan")] * 6), 0.0, 0.0, 0.0, 0.0] for k in range(3)]
    circle = trajectory_metrics(world, rows, 0.55)
    outline = trajectory_metrics(world, rows, 0.55, body=outline_body())
    assert circle["minimum_true_gap_to_body_m"] == pytest.approx(world.clearance(2.5, 2.0) - 0.55)   # unchanged
    assert circle["gap_basis"] == "AXLE_CLEARANCE_MINUS_RADIUS"
    assert outline["minimum_true_gap_to_body_m"] == pytest.approx(3.9 - 2.5 - 0.79, abs=1e-9)
    assert outline["gap_basis"] == "BODY_OUTLINE_AT_TRUE_POSE"
