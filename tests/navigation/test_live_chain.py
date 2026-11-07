import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from wc_navigation.live import config as live_config
from wc_navigation.live.odometry_adapter import LiveOdometry, axle_from_reference, body_twist


# -- configuration --------------------------------------------------------------------------------
def write_cell(tmp_path, *, mode="left", sensor="H30-TEST", radius=0.172):
    cell = tmp_path / "cell"
    cell.mkdir()
    rotation = np.array([[math.cos(0.01), -math.sin(0.01), 0], [math.sin(0.01), math.cos(0.01), 0], [0, 0, 1]])
    base_axle = np.eye(4)
    base_axle[:3, :3] = rotation.T
    base_axle[:3, 3] = [-0.32, -0.23, -0.59]
    runtime = {"mode": mode, "cloud_source": "filtered", "session_id": "capture_x",
               "cloud_filter": {"enabled": True}, "sensor_ids": {"left": "XT-LEFT"},
               "mounts": {"left": {"t_axle_lidar_m": [0.323, 0.22875, 0.5915]}},
               "hardware_setup_provenance": {"sha256": "abc"}}
    wheel = {"schema": "wc_wheel_history_preview_v1", "wheel_radius_m": radius, "track_width_m": 0.61,
             "register_to_wheel_rpm": 0.112, "left_sign": -1, "right_sign": 1, "max_gap_s": 0.5}
    prior = {"motion_model": "planar_ekf", "base_frame": "lidar_left", "imu_sensor_id": sensor,
             "wheel_candidate": wheel, "T_base_axle": base_axle.tolist(), "offline_imu_time_mode": "device_relative",
             "offline_imu_time_model": {"x": 1}, "confirmed_gyro_bias": {"old": True}}
    (cell / "runtime_config.json").write_text(json.dumps(runtime))
    (cell / "prior_config.json").write_text(json.dumps(prior))
    (cell / "prior_initialization.json").write_text(json.dumps({"R_reference_base": rotation.tolist()}))
    setup = tmp_path / "setup.json"
    setup.write_text(json.dumps({"wheel_odometry": {"wheel_radius_m": 0.1646, "track_width_m": 0.599,
                                                    "register_to_wheel_rpm": 0.112}}))
    bias = tmp_path / "bias.json"
    bias.write_text(json.dumps({"status": "INDEPENDENTLY_CONFIRMED", "sensor_id": "H30-TEST",
                                "bias_native_rad_s": [0.0, 0.0, 0.00058]}))
    return cell, setup, bias, rotation


def test_live_config_applies_the_wheel_override_and_bias_with_arrival_timing(tmp_path):
    cell, setup, bias, rotation = write_cell(tmp_path)
    live = live_config.load_live_config(cell, setup, bias)
    wheel = live["prior"]["wheel_candidate"]
    assert (wheel["wheel_radius_m"], wheel["track_width_m"], wheel["left_sign"]) == (0.1646, 0.599, -1)
    assert live["filter"]["wheel_candidate"]["wheel_radius_m"] == 0.1646
    assert live["provenance"]["wheel_values_replaced"]["wheel_radius_m"] == {"frozen": 0.172, "applied": 0.1646}
    assert live["prior"]["confirmed_gyro_bias"]["bias_native_rad_s"][2] == 0.00058
    assert live["prior"]["offline_imu_time_mode"] == "arrival" and "offline_imu_time_model" not in live["prior"]
    assert live["provenance"]["frozen_imu_time_mode"] == "device_relative"
    np.testing.assert_allclose(live["R_reference_base"], rotation)
    assert live["lever_xy"] == [0.323, 0.22875]
    assert len(live["provenance"]["hardware_setup"]["sha256"]) == 64


def test_live_config_rejects_other_imu_bias_and_non_left_frontends(tmp_path):
    cell, setup, bias, _ = write_cell(tmp_path, sensor="H30-OTHER")
    with pytest.raises(live_config.LiveConfigError, match="another IMU"):
        live_config.load_live_config(cell, setup, bias)
    other = tmp_path / "other"
    other.mkdir()
    cell, setup, bias, _ = write_cell(other, mode="all")
    with pytest.raises(live_config.LiveConfigError, match="single-left"):
        live_config.load_live_config(cell, setup, bias)


def test_wheel_override_needs_a_wheel_block_and_positive_lengths():
    frozen = {"wheel_radius_m": 0.172, "track_width_m": 0.61}
    with pytest.raises(live_config.LiveConfigError):
        live_config.merged_wheel(frozen, {})
    with pytest.raises(live_config.LiveConfigError):
        live_config.merged_wheel(frozen, {"wheel_odometry": {"wheel_radius_m": -1}})
    wheel, changed = live_config.merged_wheel(frozen, {"wheel_odometry": {"track_width_m": 0.61}})
    assert wheel == frozen and changed == {}


# -- scan adapter ------------------------------------------------------------------------------------
def organized_cloud(points, height, width):
    from sensor_msgs.msg import PointCloud2, PointField
    cloud = PointCloud2()
    cloud.header.frame_id = "lidar_left"
    cloud.height, cloud.width = height, width
    cloud.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                    for k, n in enumerate(("x", "y", "z", "intensity"))]
    cloud.point_step, cloud.row_step = 16, 16 * width
    cloud.is_bigendian, cloud.is_dense = False, False
    data = np.zeros((height * width, 4), dtype="<f4")
    data[:, :3] = points
    cloud.data = data.tobytes()
    return cloud


def source_frame(cloud, flags=("representation=host_filtered",), side="left", sensor="XT-LEFT"):
    return SimpleNamespace(side=side, sensor_id=sensor, cloud=cloud, raw_count=cloud.height * cloud.width,
                           diagnostic_flags=list(flags), header=cloud.header)


def test_scan_source_checks_side_frame_sensor_and_representation():
    from wc_navigation.live.scan_adapter import ScanRejected, check_source
    cloud = organized_cloud(np.ones((4, 3)), 2, 2)
    check_source(source_frame(cloud), "XT-LEFT")
    for frame in (source_frame(cloud, side="right"), source_frame(cloud, sensor="XT-OTHER"),
                  source_frame(cloud, flags=("representation=before_host_filter",)), source_frame(cloud, flags=())):
        with pytest.raises(ScanRejected):
            check_source(frame, "XT-LEFT")


def test_unfiltered_scan_is_the_offline_projection_of_the_rotated_points():
    from wc_navigation.amcl_replay import build_scan
    from wc_navigation.live.scan_adapter import scan_from_source
    angles = np.linspace(-0.9, 0.9, 40)
    points = np.column_stack([3 * np.cos(angles), 3 * np.sin(angles), np.full(40, 0.1)])
    points[5] = np.nan
    message = source_frame(organized_cloud(points, 4, 10))
    rotation = np.array([[math.cos(0.2), -math.sin(0.2), 0], [math.sin(0.2), math.cos(0.2), 0], [0, 0, 1]])
    config = {"cloud_filter": {"enabled": False}, "mapping_enabled": True}
    ranges, stats = scan_from_source(message, config, {"T_base_axle": np.eye(4).tolist()}, rotation)
    finite = np.isfinite(points).all(axis=1)
    expected, _ = build_scan((points[finite].astype(np.float32).astype(float) @ rotation.T)
                             .astype(np.float32).astype(float))
    np.testing.assert_array_equal(ranges, expected)
    assert np.isinf(ranges).sum() == 161 - stats["observed_bins"] and not np.isnan(ranges).any()


def test_host_filter_drops_isolated_returns_before_projection():
    from wc_navigation.live.scan_adapter import scan_from_source
    grid = np.zeros((4, 10, 3))
    for row in range(4):
        for col in range(10):
            angle = -0.5 + 0.1 * col
            grid[row, col] = [2.0 * math.cos(angle), 2.0 * math.sin(angle), 0.05 * row]
    grid[0, 0] = [6.0, -6.0, 0.0]                       # far from all neighbours: unsupported
    message = source_frame(organized_cloud(grid.reshape(-1, 3), 4, 10))
    config = {"cloud_filter": {"enabled": True, "range": {"enabled": True, "min_m": 0.3, "max_m": 15.0},
                               "organized_support": {"enabled": True, "min_neighbors": 2,
                                                     "distance_offset_m": 0.3, "distance_scale": 0.02},
                               "ground_height": {"enabled": False}}, "mapping_enabled": True,
              "wheel_candidate": {"wheel_radius_m": 0.17}}
    ranges, _ = scan_from_source(message, config, {"T_base_axle": np.eye(4).tolist()}, np.eye(3))
    bin_far = int(np.floor((math.atan2(-6.0, 6.0) + math.pi / 3) / (math.pi / 240) + 0.5))
    assert np.isinf(ranges[bin_far])
    assert np.isfinite(ranges).sum() >= 8 and np.nanmin(ranges[np.isfinite(ranges)]) == pytest.approx(2.0, abs=1e-5)


# -- odometry adapter ----------------------------------------------------------------------------------
def test_axle_pose_is_the_reference_moved_back_by_the_lever():
    pose = np.eye(4)
    yaw = math.pi / 2
    pose[:2, :2] = [[0, -1], [1, 0]]
    pose[:2, 3] = [1.0, 2.0]
    x, y, a = axle_from_reference(pose, (0.3, 0.2))
    assert (x, y) == pytest.approx((1.0 + 0.2, 2.0 - 0.3)) and a == pytest.approx(yaw)
    assert body_twist((0, 0, 0), (0.1, 0, 0.05), 0.5) == pytest.approx((0.2, 0.1), abs=1e-3)
    assert body_twist(None, (1, 1, 1), 0.1) == (0.0, 0.0)


class NotReady(Exception):
    pass


class FakePrior:
    """Motion-prior stand-in: watermark from appended stamps, linear motion along x."""

    def __init__(self, config, session_id, clock):
        self.session_id, self.clock = session_id, clock
        self.imu, self.wheel = [], []
        self.initialization = None
        self.checkpoints = [(1_000_000_000, np.eye(4))]
        self.forwarded = []

    def add_imu(self, **kw):
        self.imu.append({"stamp": kw["stamp_ns"]})
        if self.wheel:
            self.initialization = {"stamp_ns": 1_000_000_000}

    def add_wheel(self, record):
        self.wheel.append({"stamp": record["stamp_ns"]})

    def pose_at(self, stamp):
        if stamp > min(self.imu[-1]["stamp"], self.wheel[-1]["stamp"]):
            raise NotReady("waiting")
        pose = np.eye(4)
        pose[0, 3] = (stamp - 1_000_000_000) * 1e-9
        return pose

    def record_forwarded(self, stamp, pose):
        assert not self.forwarded or stamp > self.forwarded[-1]
        self.forwarded.append(stamp)


def imu_message(stamp_ns, session="capture_x"):
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    vector = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    header = SimpleNamespace(frame_id="imu_h30_native")
    return SimpleNamespace(angular_velocity_valid=True, linear_acceleration_valid=True, uncertainty_valid=False,
                           header=header, imu=SimpleNamespace(header=header, linear_acceleration=vector,
                                                              angular_velocity=vector),
                           session_id=session, host_receive_time=SimpleNamespace(sec=sec, nanosec=nanosec),
                           host_monotonic_ns=stamp_ns, sensor_id="H30-TEST", stream_epoch="e",
                           frame_sequence=stamp_ns // 5_000_000, coordinate_convention="H30_NATIVE_UNVALIDATED",
                           time_source="arrival_only", common_time_valid=False)


def test_odometry_emits_a_fixed_grid_up_to_the_watermark_and_bounds_history():
    odometry = LiveOdometry({}, (0.323, 0.22875), period_s=0.02, prior_factory=FakePrior)
    assert odometry.add_wheel(json.dumps({"stamp_ns": 900_000_000})) is False     # no session yet
    odometry.add_imu(imu_message(1_000_000_000))
    assert odometry.add_wheel(json.dumps({"stamp_ns": 1_000_000_000, "receive_monotonic_ns": 1_000_000_000}))
    odometry.add_imu(imu_message(1_100_000_000))
    first = odometry.poses()
    assert [p[0] for p in first] == [1_000_000_000]
    odometry.add_wheel(json.dumps({"stamp_ns": 1_100_000_000}))
    later = odometry.poses()
    assert [p[0] for p in later] == [1_020_000_000 + 20_000_000 * k for k in range(5)]
    assert later[-1][1] == pytest.approx(0.1 - 0.323)            # x of the axle: reference minus the lever
    assert odometry.prior.forwarded == [1_000_000_000] + [p[0] for p in later]
    assert odometry.poses() == []
    with pytest.raises(ValueError, match="session changed"):
        odometry.add_imu(imu_message(1_200_000_000, session="capture_y"))
    with pytest.raises(ValueError):
        LiveOdometry({}, (0, 0), period_s=0.5)


# -- relay, supervisor helpers and command line ------------------------------------------------------------
def test_relay_configuration_guards_against_loops_and_unapproved_domains(monkeypatch):
    from wc_navigation.live.relay_node import check_relay_config
    monkeypatch.delenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", raising=False)
    assert check_relay_config({"source_domain": 100, "source_prefix": "/relay_source"}, "100") == (100, "/relay_source")
    with pytest.raises(ValueError, match="loop"):
        check_relay_config({"source_domain": 100}, "100")
    with pytest.raises(ValueError, match="APPROVED_BY_USER"):
        check_relay_config({"source_domain": 83}, "100")
    with pytest.raises(ValueError):
        check_relay_config({"source_domain": 100, "source_prefix": "relay"}, "100")
    monkeypatch.setenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", "yes")
    with pytest.raises(ValueError, match="APPROVED_BY_USER"):
        check_relay_config({"source_domain": 83}, "100")
    monkeypatch.setenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", "APPROVED_BY_USER_20261108")
    assert check_relay_config({"source_domain": 83}, "100") == (83, "")


def test_sim_time_is_set_everywhere_in_a_parameter_tree():
    from wc_navigation.autonomy import nav2_config
    from wc_navigation.live.supervisor import with_sim_time
    params = nav2_config.navigation_parameters(nav2_config.NavProfile(), mode="navigate", bt_to_pose_path="a.xml",
                                               bt_through_poses_path="b.xml")
    wall = with_sim_time(params, False)
    found = []

    def walk(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == "use_sim_time":
                    found.append(item)
                walk(item)
    walk(wall)
    assert found and all(v is False for v in found)
    assert params["controller_server"]["ros__parameters"]["use_sim_time"] is True


def test_command_line_requires_explicit_vehicle_inputs_and_goals(tmp_path):
    from wc_navigation.live.cli import arguments
    common = ["--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b"]
    assert arguments(["checklist"]).command == "checklist"
    with pytest.raises(SystemExit):
        arguments(["inputs", *common])
    shadow = ["shadow", *common, "--map", "m.yaml", "--initial-pose", "0", "0", "0"]
    with pytest.raises(SystemExit):
        arguments(shadow + ["--source", "bag", "--bag", "x", ])                 # no goals
    with pytest.raises(SystemExit):
        arguments(shadow + ["--source", "vehicle", "--goal", "1", "2", "0"])    # no sensor domain
    with pytest.raises(SystemExit):
        arguments(shadow + ["--source", "vehicle", "--sensor-domain", "83", "--bag", "x", "--goal", "1", "2", "0"])
    args = arguments(shadow + ["--source", "bag", "--bag", "x", "--goal", "1", "2", "90"])
    assert args.goal == [[1.0, 2.0, 90.0]] and args.gate_correction_reference == "robot"
    assert args.gate_correction_limits == [0.25, 10.0] and args.gate_covariance_limits[0] == 0.50
    with pytest.raises(SystemExit):
        arguments(shadow + ["--source", "bag", "--bag", "x", "--goal", "1", "2", "0", "--amcl-params", "other"])


def test_vehicle_mode_refuses_without_the_dated_user_approval(tmp_path, monkeypatch):
    from wc_navigation.live import cli
    monkeypatch.delenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", raising=False)
    argv = ["shadow", "--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b",
            "--source", "vehicle", "--sensor-domain", "83", "--map", "m.yaml", "--initial-pose", "0", "0", "0",
            "--goal", "1", "2", "0"]
    with pytest.raises(SystemExit, match="APPROVED_BY_USER"):
        cli.main(argv)


def test_relay_source_node_class_cannot_publish_serve_or_call():
    from rcl_interfaces.msg import ParameterEvent
    from rclpy.qos import qos_profile_parameter_events
    from std_msgs.msg import String
    from wc_navigation.live.relay_node import read_only_node_class
    cls = read_only_node_class()
    node = object.__new__(cls)            # methods only; no ROS context is created
    stand_in = cls.create_publisher(node, ParameterEvent, "/parameter_events", qos_profile_parameter_events)
    assert stand_in.publish(ParameterEvent()) is None and stand_in.destroy() is None
    for topic in ("/wc_mapping/imu/source_frame", "/parameter_events_other", "/rosout"):
        with pytest.raises(PermissionError):
            cls.create_publisher(node, String, topic, 10)
    with pytest.raises(PermissionError):
        cls.create_publisher(node, String, "/parameter_events", 10)      # right name, wrong type
    with pytest.raises(PermissionError):
        cls.create_service(node, object, "/srv", lambda request, response: response)
    with pytest.raises(PermissionError):
        cls.create_client(node, object, "/srv")


def test_relay_graph_check_requires_exactly_the_three_subscriptions_and_nothing_else():
    from wc_navigation.live.relay_node import RELAYED_TOPICS, graph_check
    expected = ["/p" + t for t in RELAYED_TOPICS]
    subscriptions = [(t, ["type"]) for t in reversed(expected)]
    assert graph_check([], subscriptions, [], [], expected)["status"] == "PASS"
    assert graph_check([("/parameter_events", ["x"])], subscriptions, [], [], expected)["status"] == "FAIL"
    assert graph_check([], subscriptions[:2], [], [], expected)["status"] == "FAIL"
    assert graph_check([], subscriptions + [("/other", ["x"])], [], [], expected)["status"] == "FAIL"
    assert graph_check([], subscriptions, [("/get_parameters", ["x"])], [], expected)["status"] == "FAIL"
    assert graph_check([], subscriptions, [], [("/x", ["y"])], expected)["status"] == "FAIL"


def test_vehicle_mode_also_requires_the_agreed_shadow_domain(tmp_path, monkeypatch):
    from wc_navigation.live import cli
    monkeypatch.setenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", "APPROVED_BY_USER_20261108")
    monkeypatch.setenv("ROS_DOMAIN_ID", "99")
    argv = ["shadow", "--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b",
            "--source", "vehicle", "--sensor-domain", "83", "--map", "m.yaml", "--initial-pose", "0", "0", "0",
            "--goal", "1", "2", "0"]
    with pytest.raises(SystemExit, match="domain 100"):
        cli.main(argv)
