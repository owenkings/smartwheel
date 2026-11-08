"""Obstacle-cloud wiring: defaults unchanged, Nav2 sources, readback, shadow packing, CLI options, replay message."""
import argparse
import json
import math

import numpy as np
import pytest

from wc_navigation.autonomy import nav2_config
from wc_navigation.perception import nav2_obstacles, shadow_cli_options, shadow_obstacles
from wc_navigation.perception.obstacle_cloud import ObstacleFilter

MEASURED = [(0.81, 0.2425), (0.2, 0.35), (-0.255, 0.35), (-0.255, -0.35), (0.2, -0.35), (0.81, -0.2425)]


def params(profile, mode="navigate"):
    return nav2_config.navigation_parameters(profile, mode=mode, bt_to_pose_path="/tmp/a.xml",
                                             bt_through_poses_path="/tmp/b.xml")


def local(p):
    return p["local_costmap"]["local_costmap"]["ros__parameters"]


def monitor(p):
    return p["collision_monitor"]["ros__parameters"]


@pytest.mark.parametrize("footprint", [(), MEASURED])
def test_default_profile_is_unchanged(footprint):
    profile = nav2_config.NavProfile(footprint=footprint)
    assert profile.obstacle_input == "scan" and profile.obstacle_clouds == ()
    assert "obstacle_input" not in profile.as_dict() and "obstacle_clouds" not in profile.as_dict()
    p = params(profile)
    assert local(p)["plugins"] == ["obstacle_layer", "inflation_layer"] and "voxel_layer" not in local(p)
    assert monitor(p)["observation_sources"] == ["scan"]
    assert monitor(p)["scan"] == {"type": "scan", "topic": "/scan", "enabled": True}
    assert not any(k.startswith("cloud_") for k in monitor(p))
    expected = nav2_config.readback_expectations(profile, "navigate")
    assert "plugins" not in expected["local_costmap/local_costmap"]
    assert "observation_sources" not in expected["collision_monitor"]
    # an old run_config profile (no obstacle keys) still loads
    assert nav2_config.NavProfile(**profile.as_dict()) == profile


def test_cloud_input_replaces_scan_locally_and_leaves_the_global_costmap_alone():
    base = nav2_config.NavProfile(footprint=MEASURED)
    profile = nav2_config.NavProfile(footprint=MEASURED, obstacle_input="cloud", obstacle_clouds=("right", "left"))
    assert profile.obstacle_clouds == ("left", "right")
    p, q = params(profile), params(base)
    lc = local(p)
    assert lc["plugins"] == ["voxel_layer", "inflation_layer"] and "obstacle_layer" not in lc
    voxel = lc["voxel_layer"]
    assert voxel["plugin"] == "nav2_costmap_2d::VoxelLayer"
    assert voxel["observation_sources"] == "cloud_left clear_left cloud_right clear_right"
    for side in ("left", "right"):
        src = voxel["cloud_" + side]
        assert src["topic"] == nav2_obstacles.CLOUD_TOPICS[side] and src["data_type"] == "PointCloud2"
        assert src["marking"] and src["clearing"] and src["sensor_frame"] == ""
        assert src["min_obstacle_height"] == pytest.approx(-0.185 + 0.05)
        clear = voxel["clear_" + side]
        assert clear["topic"] == nav2_obstacles.CLEAR_TOPICS[side]
        assert clear["marking"] is False and clear["clearing"] is True
        assert clear["raytrace_max_range"] > src["raytrace_max_range"]
        assert clear["min_obstacle_height"] < -0.185       # the buffer must not drop low ray ends
    # the grid starts at the lowest obstacle height; 16 layers of 0.1 m cover the 1.6 m window
    assert voxel["origin_z"] == pytest.approx(-0.135) and voxel["z_voxels"] == 16
    assert voxel["origin_z"] + 16 * voxel["z_resolution"] >= voxel["max_obstacle_height"] - 1e-9
    cm = monitor(p)
    assert cm["observation_sources"] == ["cloud_left", "cloud_right"] and "scan" not in cm
    assert cm["cloud_left"]["type"] == "pointcloud" and cm["cloud_left"]["min_height"] == pytest.approx(-0.135)
    assert p["global_costmap"] == q["global_costmap"]
    assert p["controller_server"] == q["controller_server"] and p["planner_server"] == q["planner_server"]
    expected = nav2_config.readback_expectations(profile, "navigate")
    assert expected["local_costmap/local_costmap"]["plugins"] == ["voxel_layer", "inflation_layer"]
    assert expected["collision_monitor"]["observation_sources"] == ["cloud_left", "cloud_right"]
    round_trip = nav2_config.NavProfile(**profile.as_dict())
    assert round_trip == profile


def test_scan_and_cloud_keeps_both():
    profile = nav2_config.NavProfile(obstacle_input="scan_and_cloud", obstacle_clouds=("left",))
    p = params(profile)
    assert local(p)["plugins"] == ["obstacle_layer", "voxel_layer", "inflation_layer"]
    assert monitor(p)["observation_sources"] == ["scan", "cloud_left"] and "scan" in monitor(p)


def test_filtered_scan_feeds_the_costmap_obstacle_layers_only_when_asked():
    def scan_topic(p, which):
        return p[which + "_costmap"][which + "_costmap"]["ros__parameters"]["obstacle_layer"]["scan"]["topic"]
    raw = nav2_config.NavProfile(obstacle_input="scan_and_cloud", obstacle_clouds=("left",))
    filtered = nav2_config.NavProfile(obstacle_input="scan_and_cloud", obstacle_clouds=("left",),
                                      obstacle_scan="filtered")
    assert scan_topic(params(raw), "global") == "/scan" and scan_topic(params(raw), "local") == "/scan"
    p = params(filtered)
    assert scan_topic(p, "global") == nav2_obstacles.FILTERED_SCAN_TOPIC
    assert scan_topic(p, "local") == nav2_obstacles.FILTERED_SCAN_TOPIC
    assert monitor(p)["scan"]["topic"] == "/scan"            # the monitor's own scan source stays raw
    expected = nav2_config.readback_expectations(filtered, "navigate")
    assert expected["global_costmap/global_costmap"]["obstacle_layer.scan.topic"] == nav2_obstacles.FILTERED_SCAN_TOPIC
    assert "obstacle_layer.scan.topic" not in nav2_config.readback_expectations(raw, "navigate")[
        "global_costmap/global_costmap"]
    assert filtered.as_dict()["obstacle_scan"] == "filtered"
    assert nav2_config.NavProfile(**filtered.as_dict()) == filtered
    for bad in ({"obstacle_scan": "filtered"},
                {"obstacle_input": "cloud", "obstacle_clouds": ("right",), "obstacle_scan": "filtered"},
                {"obstacle_input": "cloud", "obstacle_clouds": ("left",), "obstacle_scan": "smoothed"},
                {"obstacle_input": "scan", "obstacle_clouds": ("left", "right"), "obstacle_scan": "filtered"},
                {"obstacle_input": "scan", "obstacle_clouds": ("left",)}):
        with pytest.raises(ValueError):
            nav2_config.NavProfile(**bad)


def test_monitor_can_read_the_clouds_while_the_costmaps_stay_two_dimensional():
    def layer(p, which):
        return p[which + "_costmap"][which + "_costmap"]["ros__parameters"]
    profile = nav2_config.NavProfile(obstacle_input="scan", obstacle_clouds=("left", "right"),
                                     obstacle_scan="filtered", obstacle_monitor="cloud", footprint=MEASURED)
    assert profile.monitor_input == "cloud"
    p, q = params(profile), params(nav2_config.NavProfile(footprint=MEASURED))
    for which in ("local", "global"):
        assert layer(p, which)["plugins"] == layer(q, which)["plugins"] and "voxel_layer" not in layer(p, which)
        assert layer(p, which)["obstacle_layer"]["scan"]["topic"] == nav2_obstacles.FILTERED_SCAN_TOPIC
    assert monitor(p)["observation_sources"] == ["cloud_left", "cloud_right"] and "scan" not in monitor(p)
    expected = nav2_config.readback_expectations(profile, "navigate")
    assert expected["collision_monitor"]["observation_sources"] == ["cloud_left", "cloud_right"]
    assert "plugins" not in expected["local_costmap/local_costmap"]
    assert expected["local_costmap/local_costmap"]["obstacle_layer.scan.topic"] == nav2_obstacles.FILTERED_SCAN_TOPIC
    assert profile.as_dict()["obstacle_monitor"] == "cloud" and nav2_config.NavProfile(**profile.as_dict()) == profile
    # the monitor alone on clouds (raw scan in the costmaps) is allowed too
    alone = nav2_config.NavProfile(obstacle_clouds=("left",), obstacle_monitor="scan_and_cloud")
    assert monitor(params(alone))["observation_sources"] == ["scan", "cloud_left"]
    # a monitor equal to the costmap input is the plain profile
    same = nav2_config.NavProfile(obstacle_input="cloud", obstacle_clouds=("left",), obstacle_monitor="cloud")
    assert same == nav2_config.NavProfile(obstacle_input="cloud", obstacle_clouds=("left",))
    assert "obstacle_monitor" not in same.as_dict()
    assert "obstacle_monitor" not in nav2_config.NavProfile().as_dict()
    for bad in ({"obstacle_monitor": "lidar"}, {"obstacle_monitor": "cloud"},
                {"obstacle_input": "scan", "obstacle_clouds": ("left", "right"), "obstacle_scan": "filtered"}):
        with pytest.raises(ValueError):
            nav2_config.NavProfile(**bad)


def test_filtered_scan_alone_keeps_both_costmaps_two_dimensional():
    def layer(p, which):
        return p[which + "_costmap"][which + "_costmap"]["ros__parameters"]
    profile = nav2_config.NavProfile(obstacle_input="scan", obstacle_clouds=("left",), obstacle_scan="filtered",
                                     footprint=MEASURED)
    p, q = params(profile), params(nav2_config.NavProfile(footprint=MEASURED))
    for which in ("local", "global"):
        assert layer(p, which)["obstacle_layer"]["scan"]["topic"] == nav2_obstacles.FILTERED_SCAN_TOPIC
        assert layer(p, which)["plugins"] == layer(q, which)["plugins"]          # no voxel layer anywhere
        assert "voxel_layer" not in layer(p, which)
    assert monitor(p) == monitor(q)                                              # the monitor keeps /scan
    expected = nav2_config.readback_expectations(profile, "navigate")
    for node in ("local_costmap/local_costmap", "global_costmap/global_costmap"):
        assert expected[node]["obstacle_layer.scan.topic"] == nav2_obstacles.FILTERED_SCAN_TOPIC
    assert "observation_sources" not in expected["collision_monitor"]
    assert profile.as_dict()["obstacle_scan"] == "filtered" and nav2_config.NavProfile(**profile.as_dict()) == profile


def test_filtered_scan_with_planning_on_the_saved_map_only():
    from wc_navigation.planning import stack
    profile = nav2_config.NavProfile(obstacle_input="scan_and_cloud", obstacle_clouds=("left",),
                                     obstacle_scan="filtered", footprint=MEASURED)
    choice = stack.PlanningChoice(global_live_scans=False)
    p = nav2_config.navigation_parameters(profile, mode="navigate", bt_to_pose_path="/tmp/a.xml",
                                          bt_through_poses_path="/tmp/b.xml", planning=choice)
    assert "obstacle_layer" not in p["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]
    assert local(p)["obstacle_layer"]["scan"]["topic"] == nav2_obstacles.FILTERED_SCAN_TOPIC
    expected = nav2_config.readback_expectations(profile, "navigate", planning=choice)
    body = expected["global_costmap/global_costmap"]
    assert "obstacle_layer" not in body["plugins"] and not any(k.startswith("obstacle_layer.") for k in body)
    assert body["inflation_layer.inflate_around_unknown"] is True
    live = nav2_config.readback_expectations(profile, "navigate", planning=stack.PlanningChoice())
    assert live["global_costmap/global_costmap"]["obstacle_layer.scan.topic"] == nav2_obstacles.FILTERED_SCAN_TOPIC


@pytest.mark.parametrize("kwargs", [{"obstacle_input": "lidar"}, {"obstacle_input": "cloud"},
                                    {"obstacle_input": "scan", "obstacle_clouds": ("left",)},
                                    {"obstacle_input": "cloud", "obstacle_clouds": ("left", "left")},
                                    {"obstacle_input": "cloud", "obstacle_clouds": ("middle",)}])
def test_bad_obstacle_options_are_rejected(kwargs):
    with pytest.raises(ValueError):
        nav2_config.NavProfile(**kwargs)


def frame(stamp_ns, x, points):
    T = np.eye(4)
    T[0, 3] = x
    return {"stamp_ns": stamp_ns, "points": np.asarray(points, dtype=np.float32), "odometry_T": T}


def test_shadow_prepare_packs_left_frames_and_unpacks_them():
    lever = [0.323, 0.22875, 0.5915]
    runtime = {"mode": "left", "offline_source_selection": {"selected_sides": ["left"]},
               "mounts": {"left": {"t_axle_lidar_m": lever}}}
    box = np.array([[1.0 + 0.02 * i, 0.02 * j, -0.5 + 0.02 * k] for i in range(4) for j in range(4) for k in range(6)])
    floor = np.array([[1.0 + 0.1 * i, 0.1 * j - 0.5, -0.7765] for i in range(10) for j in range(10)])
    frames = [frame(10 ** 9 + k * 2 * 10 ** 8, 0.1 * k, np.vstack([box, floor])) for k in range(3)]
    out = shadow_obstacles.prepare(frames, runtime, sides=("left",), footprint=MEASURED, consensus=False)
    rec = dict(out, stamps_ns=np.asarray([f["stamp_ns"] for f in frames]))
    assert shadow_obstacles.cloud_sides(rec) == ["left"]
    assert out["obstacle_left_offsets"].tolist()[0] == 0 and len(out["obstacle_left_offsets"]) == 4
    pts = shadow_obstacles.frame_points(rec, "left", 1)
    h = pts[:, 2] + lever[2] + 0.185
    assert len(pts) > 0 and h.min() >= 0.05 and pts.dtype == np.float32
    rays = shadow_obstacles.frame_clearing(rec, "left", 1)
    assert len(rays) > 0 and rays.dtype == np.float32
    assert (rays[:, 2] + lever[2] + 0.185).min() >= 0.15 - 0.01       # never into the lowest layer
    assert len(shadow_obstacles.frame_clearing({}, "left", 0)) == 0
    assert np.allclose(shadow_obstacles.relative_times(rec, "left"), [0.0, 0.2, 0.4])
    settings = shadow_obstacles.settings(rec)
    assert settings["sides"] == ["left"] and settings["lever_basis"]["left"] == "FROZEN_FRONTEND_MOUNT"
    assert settings["totals"]["left"]["frames"] == 3
    assert shadow_obstacles.frame_scan(rec, "left", 0) is None
    with pytest.raises(ValueError):
        shadow_obstacles.prepare(frames, runtime, sides=("right",))
    with_scan = shadow_obstacles.prepare(frames, runtime, sides=("left",), consensus=False, scan=True)
    scans = with_scan["obstacle_left_scan_ranges"]
    assert scans.shape == (3, 161) and np.isfinite(scans).any(axis=1).all()
    assert shadow_obstacles.settings(dict(with_scan))["filtered_scan_topic"] == nav2_obstacles.FILTERED_SCAN_TOPIC


def test_replay_scan_message_has_the_recorded_geometry():
    pytest.importorskip("sensor_msgs")
    from types import SimpleNamespace
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import LaserScan
    from wc_navigation.autonomy.replay_node import Replay
    fake = SimpleNamespace(LaserScan=LaserScan, angle_min=-math.pi / 3, angle_increment=math.pi / 240,
                           range_min=0.3, range_max=8.0)
    ranges = np.full(161, np.inf, dtype=np.float32)
    ranges[80] = 1.25
    msg = Replay.scan_message(fake, ranges, Time(sec=1))
    assert msg.header.frame_id == "laser" and len(msg.ranges) == 161
    assert msg.angle_max == pytest.approx(math.pi / 3) and msg.ranges[80] == pytest.approx(1.25)


def test_replay_stamps_lidar_messages_earlier_by_the_time_offset():
    pytest.importorskip("sensor_msgs")
    import functools
    from types import SimpleNamespace
    from sensor_msgs.msg import LaserScan
    from wc_navigation.autonomy.replay_node import Replay
    sent = []
    fake = SimpleNamespace(LaserScan=LaserScan, angle_min=-math.pi / 3, angle_increment=math.pi / 240,
                           range_min=0.3, range_max=8.0, ranges=np.ones((1, 161), dtype=np.float32),
                           scan_pub=SimpleNamespace(publish=sent.append), lidar_offset_s=0.075)
    fake.scan_message = functools.partial(Replay.scan_message, fake)
    Replay.publish_scan(fake, 0, 120.0)
    fake.lidar_offset_s = 0.0
    Replay.publish_scan(fake, 0, 120.0)
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in sent]
    assert stamps[0] == pytest.approx(119.925, abs=1e-6) and stamps[1] == pytest.approx(120.0, abs=1e-6)


def test_prepare_places_frames_at_the_pose_of_stamp_minus_the_offset():
    lever = [0.323, 0.22875, 0.5915]
    runtime = {"mode": "left", "offline_source_selection": {"selected_sides": ["left"]},
               "mounts": {"left": {"t_axle_lidar_m": lever}}}
    # a real overhang seen from three frames while the chair drives 1 m/s; persistence on
    edge = np.array([[2.0 + 0.02 * i, 0.02 * j - 0.2, 0.2] for i in range(10) for j in range(20)])
    frames = [frame(10 ** 9 + k * 2 * 10 ** 8, 0.2 * k, edge - np.array([0.2 * k, 0.0, 0.0])) for k in range(8)]
    cfg = ObstacleFilter(persist_min_age_s=0.4, persist_max_age_s=1.0)
    plain = shadow_obstacles.prepare(frames, runtime, sides=("left",), consensus=False, cfg=cfg)
    late = shadow_obstacles.prepare(frames, runtime, sides=("left",), consensus=False, cfg=cfg,
                                    lidar_time_offset_s=0.075)
    for out in (plain, late):
        totals = shadow_obstacles.settings(dict(out))["totals"]["left"]
        assert totals["removed_transient"] == 0          # world-fixed: confirmed by the earlier frames
    assert shadow_obstacles.settings(dict(late))["lidar_time_offset_s"] == 0.075
    assert late["obstacle_left_stamps_ns"].tolist() == plain["obstacle_left_stamps_ns"].tolist()
    with pytest.raises(ValueError):
        shadow_obstacles.prepare(frames, runtime, sides=("left",), lidar_time_offset_s=0.3)


def test_symmetric_right_lever_mirrors_the_left_mount():
    assert shadow_obstacles.symmetric_right_lever([0.323, 0.22875, 0.5915]).tolist() == [0.323, -0.22875, 0.5915]


def parse(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--footprint", nargs="+", type=float)
    parser.add_argument("--assumed-radius", type=float, default=0.55)
    parser.add_argument("--global-costmap", choices=["live", "static"], default="live")   # planning.options
    shadow_cli_options.add_arguments(parser)
    args = parser.parse_args(argv)
    shadow_cli_options.check_arguments(parser, args)
    return args


def test_cli_defaults_add_nothing():
    args = parse([])
    assert shadow_cli_options.prepare_kwargs(args) is None
    assert shadow_cli_options.profile_fields(args) == {}
    assert shadow_cli_options.config_entries(args, {}) == {}
    assert shadow_cli_options.recording_entries(args) == {}


def test_cli_lidar_time_offset_reaches_the_recording_and_the_obstacles(tmp_path):
    args = parse(["--lidar-time-offset-ms", "75"])
    assert shadow_cli_options.prepare_kwargs(args) is None              # /scan only: no clouds to prepare
    assert shadow_cli_options.recording_entries(args)["lidar_time_offset_s"] == pytest.approx(0.075)
    assert shadow_cli_options.config_entries(args, {}) == {"lidar_time_offset_ms": 75.0}
    args = parse(["--obstacle-input", "cloud", "--lidar-time-offset-ms", "75"])
    assert shadow_cli_options.prepare_kwargs(args)["lidar_time_offset_s"] == pytest.approx(0.075)
    for bad in ("300", "nan"):
        with pytest.raises(SystemExit):
            parse(["--lidar-time-offset-ms", bad])


def test_cli_cloud_options(tmp_path):
    args = parse(["--obstacle-input", "cloud", "--obstacle-right-cell", str(tmp_path), "--obstacle-right-lever",
                  "symmetric", "--obstacle-filter", json.dumps({"max_range_m": 3.5})])
    kwargs = shadow_cli_options.prepare_kwargs(args)
    assert kwargs["sides"] == ("left", "right") and kwargs["right_lever"] == "symmetric"
    assert kwargs["cfg"].max_range_m == 3.5 and kwargs["radius_m"] == 0.55
    assert shadow_cli_options.profile_fields(args) == {"obstacle_input": "cloud", "obstacle_clouds": ("left", "right"),
                                                       "obstacle_scan": "raw"}
    args = parse(["--obstacle-input", "scan_and_cloud", "--obstacle-right-lever", "0.3,-0.23,0.55"])
    assert shadow_cli_options.prepare_kwargs(args)["sides"] == ("left",)
    assert shadow_cli_options.prepare_kwargs(args)["scan"] is False
    args = parse(["--obstacle-input", "cloud", "--obstacle-scan", "filtered"])
    assert shadow_cli_options.prepare_kwargs(args)["scan"] is True
    assert shadow_cli_options.profile_fields(args)["obstacle_scan"] == "filtered"
    # 2D costmaps on the filtered scan, the monitor on both clouds
    args = parse(["--obstacle-scan", "filtered", "--obstacle-monitor", "cloud", "--obstacle-right-cell", str(tmp_path)])
    assert shadow_cli_options.prepare_kwargs(args)["sides"] == ("left", "right")
    fields = shadow_cli_options.profile_fields(args)
    assert fields["obstacle_monitor"] == "cloud" and fields["obstacle_input"] == "scan"
    assert nav2_config.NavProfile(**fields).monitor_input == "cloud"
    # the filtered scan alone: only the left cloud is prepared (the right cell is its witness)
    args = parse(["--obstacle-scan", "filtered", "--obstacle-right-cell", str(tmp_path)])
    kwargs = shadow_cli_options.prepare_kwargs(args)
    assert kwargs["sides"] == ("left",) and kwargs["scan"] is True and kwargs["right_cell"] == tmp_path
    assert shadow_cli_options.profile_fields(args) == {"obstacle_input": "scan", "obstacle_clouds": ("left",),
                                                       "obstacle_scan": "filtered"}
    nav2_config.NavProfile(**shadow_cli_options.profile_fields(args))


@pytest.mark.parametrize("argv", [["--obstacle-right-cell", "/tmp"], ["--obstacle-input", "cloud", "--obstacle-sides",
                                                                       "right"],
                                  ["--obstacle-scan", "filtered", "--obstacle-sides", "left", "right"],
                                  ["--obstacle-input", "cloud", "--obstacle-right-cell", "/tmp", "--obstacle-sides",
                                   "right", "--obstacle-scan", "filtered"],
                                  ["--obstacle-input", "cloud", "--obstacle-right-lever", "1,2"],
                                  ["--obstacle-input", "cloud", "--obstacle-filter", "{\"min_height_m\": 5}"],
                                  ["--obstacle-input", "cloud", "--obstacle-scan", "filtered", "--global-costmap",
                                   "static"]])
def test_cli_rejects_inconsistent_options(argv):
    with pytest.raises(SystemExit):
        parse(argv)


def test_pointcloud2_message_layout():
    pytest.importorskip("sensor_msgs")
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import PointCloud2, PointField
    from wc_navigation.autonomy.replay_node import pointcloud2
    pts = np.array([[1.0, 2.0, 3.0], [-0.5, 0.25, 0.125]], dtype=np.float32)
    msg = pointcloud2(PointCloud2, PointField, pts, "lidar_left", Time(sec=3, nanosec=5))
    assert msg.header.frame_id == "lidar_left" and msg.width == 2 and msg.point_step == 12
    assert [f.name for f in msg.fields] == ["x", "y", "z"]
    assert np.array_equal(np.frombuffer(bytes(msg.data), dtype="<f4").reshape(-1, 3), pts)


def test_cloud_zone_audit_counts_stop_points():
    lever = np.array([0.323, 0.22875, 0.5915])
    # 1 cm ahead of the measured front edge: inside the 2 cm stop ring
    inside = np.array([[0.82 - lever[0], 0.0 - lever[1], 0.0], [0.82 - lever[0], 0.02 - lever[1], 0.0],
                       [0.82 - lever[0], -0.02 - lever[1], 0.0], [0.82 - lever[0], 0.04 - lever[1], 0.0]])
    rec = {"stamps_ns": np.array([0]), "obstacle_left_stamps_ns": np.array([0, 2 * 10 ** 8]),
           "obstacle_left_offsets": np.array([0, 4, 4]), "obstacle_left_points": inside.astype(np.float32),
           "obstacle_left_lever": lever, "obstacle_settings": np.asarray(json.dumps({"filter": {"axle_height_m": 0.185}}))}
    stop = nav2_config.offset_polygon(MEASURED, nav2_config.STOP_RING_M)
    audit = shadow_obstacles.zone_audit(rec, (0.0, 1.0), stop, nav2_config.slowdown_polygon(
        nav2_config.NavProfile(footprint=MEASURED)), 3)
    assert audit["left"]["frames"] == 2 and audit["left"]["frames_stop_expected"] == 1
    example = audit["left"]["stop_examples"][0]
    assert example["points"] == 4 and math.isclose(example["xy_mean"][0], 0.82, abs_tol=1e-3)
