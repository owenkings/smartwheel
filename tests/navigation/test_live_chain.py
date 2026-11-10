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


def test_frozen_current_configuration_loads_like_a_cell_and_matches_the_reference(tmp_path, monkeypatch):
    import hashlib
    from wc_navigation.live import current_config
    cell, setup, bias, _ = write_cell(tmp_path)
    runtime = json.loads((cell / "runtime_config.json").read_text())
    prior = json.loads((cell / "prior_config.json").read_text())
    setup_text = setup.read_text()
    runtime["hardware_setup_provenance"] = {"source_path": "config/local/project/hardware_setup.json",
                                            "sha256": hashlib.sha256(setup_text.encode()).hexdigest()}
    runtime["wheel_candidate"] = prior["wheel_candidate"]
    resolved = {"root": str(tmp_path), "runtime": runtime, "prior": prior, "snapshots": {"hardware_setup.json": setup_text}}
    monkeypatch.setattr(current_config, "resolve", lambda root, session, path: ({"mode": "left"}, resolved))
    write_offset(tmp_path / "config/calibration/lidar_time_offset.json", 75.0)
    record = current_config.freeze(tmp_path / "frozen", tmp_path, "s1", compare_cell=cell, gyro_bias=bias)
    checks = record["checks"]
    assert checks["hardware_setup_from_local_override"] and checks["live_chain_loads_it"]
    assert checks["lidar_time_offset_ms"] == 75.0 and (tmp_path / "frozen/lidar_time_offset.json").is_file()
    assert not checks["confirmed_gyro_bias_configured"]
    same = checks["against_reference_cell"]
    assert same["T_base_axle_max_abs_diff"] == 0.0 and same["lever_xy_max_abs_diff_m"] == 0.0 and same["cloud_filter_equal"]
    written = json.loads((tmp_path / "frozen" / "frozen_configuration.json").read_text())
    assert written["files_sha256"]["hardware_setup.json"] == runtime["hardware_setup_provenance"]["sha256"]
    # a snapshot that does not match the resolved provenance is refused
    runtime["hardware_setup_provenance"]["sha256"] = "0" * 64
    with pytest.raises(current_config.FreezeError):
        current_config.freeze(tmp_path / "frozen2", tmp_path, "s2")


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


def test_the_obstacle_input_checks_the_right_lidar_by_its_own_side_frame_and_sensor():
    from wc_navigation.live.scan_adapter import ScanRejected, check_source
    cloud = organized_cloud(np.ones((4, 3)), 2, 2)
    cloud.header.frame_id = "lidar_right"
    check_source(source_frame(cloud, side="right", sensor="XT-RIGHT"), "XT-RIGHT", side="right")
    for frame, sensor in ((source_frame(cloud, side="right", sensor="XT-LEFT"), "XT-RIGHT"),
                          (source_frame(cloud, side="left", sensor="XT-RIGHT"), "XT-RIGHT")):
        with pytest.raises(ScanRejected):
            check_source(frame, sensor, side="right")
    with pytest.raises(ScanRejected):                   # a right frame never feeds the left (/scan) side
        check_source(source_frame(cloud, side="right", sensor="XT-RIGHT"), "XT-RIGHT")


def test_reference_points_are_the_clouds_the_scan_is_projected_from():
    from wc_navigation.amcl_replay import build_scan
    from wc_navigation.live.scan_adapter import reference_points, scan_from_source
    angles = np.linspace(-0.9, 0.9, 40)
    points = np.column_stack([3 * np.cos(angles), 3 * np.sin(angles), np.full(40, 0.1)])
    points[7] = np.nan
    message = source_frame(organized_cloud(points, 4, 10))
    rotation = np.array([[math.cos(0.2), -math.sin(0.2), 0], [math.sin(0.2), math.cos(0.2), 0], [0, 0, 1]])
    config = {"cloud_filter": {"enabled": False}, "mapping_enabled": True}
    reference, _ = reference_points(message, config, {"T_base_axle": np.eye(4).tolist()}, rotation)
    finite = np.isfinite(points).all(axis=1)
    assert reference.dtype == np.float32 and len(reference) == finite.sum()
    np.testing.assert_array_equal(reference, (points[finite].astype(np.float32).astype(float) @ rotation.T)
                                  .astype(np.float32))
    ranges, _ = scan_from_source(message, config, {"T_base_axle": np.eye(4).tolist()}, rotation)
    np.testing.assert_array_equal(ranges, build_scan(reference.astype(float))[0])


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
    from wc_navigation.live.relay_node import OUTPUT_PREFIX, check_relay_config
    monkeypatch.delenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", raising=False)
    assert check_relay_config({"source_domain": 100, "source_prefix": "/relay_source"}, "100") == (
        100, "/relay_source", OUTPUT_PREFIX)
    with pytest.raises(ValueError, match="loop"):
        check_relay_config({"source_domain": 100}, "100")
    with pytest.raises(ValueError, match="loop"):
        check_relay_config({"source_domain": 100, "source_prefix": "/p", "output_prefix": "/p"}, "100")
    with pytest.raises(ValueError, match="APPROVED_BY_USER"):
        check_relay_config({"source_domain": 83}, "100")
    with pytest.raises(ValueError):
        check_relay_config({"source_domain": 100, "source_prefix": "relay"}, "100")
    with pytest.raises(ValueError):
        check_relay_config({"source_domain": 100, "source_prefix": "/p", "output_prefix": ""}, "100")
    with pytest.raises(ValueError):
        check_relay_config({"source_domain": 100, "source_prefix": "/p", "wait_for_panel_consumers": "yes"}, "100")
    monkeypatch.setenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", "yes")
    with pytest.raises(ValueError, match="APPROVED_BY_USER"):
        check_relay_config({"source_domain": 83}, "100")
    monkeypatch.setenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", "APPROVED_BY_USER_20261108")
    assert check_relay_config({"source_domain": 83}, "100") == (83, "", OUTPUT_PREFIX)


def test_relay_reads_best_effort_volatile_keep_last_and_waits_for_the_panel_consumers(tmp_path, monkeypatch):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
    from wc_navigation.live import cli, relay_node
    for topic, depth in ((relay_node.LEFT_FILTERED_TOPIC, 3), (relay_node.IMU_TOPIC, 100), (relay_node.WHEEL_TOPIC, 20)):
        qos = relay_node.source_qos(topic)
        assert (qos.reliability, qos.durability, qos.history, qos.depth) == (
            ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE, HistoryPolicy.KEEP_LAST, depth)
    # publishers and another subscriber on every topic, or the relay does not subscribe yet
    ready = {topic: (1, 1) for topic in relay_node.RELAYED_TOPICS}
    assert relay_node.panel_ready(ready)
    assert not relay_node.panel_ready({**ready, relay_node.IMU_TOPIC: (1, 0)})
    assert not relay_node.panel_ready({**ready, relay_node.LEFT_FILTERED_TOPIC: (0, 2)})
    assert cli.OUTPUT_PREFIX == relay_node.OUTPUT_PREFIX


def test_vehicle_relay_waits_for_the_panel_and_the_inputs_read_the_renamed_topics(tmp_path, monkeypatch):
    from wc_navigation.live import cli
    captured = {}

    class Stop(Exception):
        pass

    def fake_run(self):
        captured.update(self.config)
        raise Stop()
    monkeypatch.setenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", "APPROVED_BY_USER_20261108")
    monkeypatch.setenv("ROS_DOMAIN_ID", "100")
    monkeypatch.setenv("WC_AUTONOMY_ROS_DOMAIN", "100")
    from wc_navigation.live import config as live_config, supervisor
    monkeypatch.setattr(live_config, "load_live_config", lambda *a: {"provenance": {}})
    monkeypatch.setattr(supervisor.LiveSupervisor, "run", fake_run)
    monkeypatch.setattr(cli, "WORK_ROOT", tmp_path / "shm")
    monkeypatch.setattr("wc_runtime.storage_policy.StoragePolicy.check", lambda self: None)
    runs = iter(range(10))
    monkeypatch.setattr("wc_runtime.storage_policy.StoragePolicy.resolve",
                        lambda self, value: tmp_path / ("run%d" % next(runs)))
    map_yaml = tmp_path / "m.yaml"
    map_yaml.write_text("image: m.pgm\n")
    write_offset(tmp_path / "config/calibration/lidar_time_offset.json", 75.0)
    argv = ["shadow", "--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b",
            "--source", "vehicle", "--sensor-domain", "83", "--map", str(map_yaml), "--initial-pose", "0", "0", "0",
            "--goal", "1", "2", "0"]
    with pytest.raises(Stop):
        cli.main(argv)
    assert captured["relay"] == {"source_domain": 83, "source_prefix": "", "output_prefix": cli.OUTPUT_PREFIX,
                                 "wait_for_panel_consumers": True, "obstacles": False}
    # window B's smoother feedback, gated by default with a 0.3 s velocity timeout (user 2026-11-10)
    assert (captured["smoother_feedback"], captured["smoother_timeout_s"]) == ("gated", 0.3)
    assert captured["source_prefix"] == cli.OUTPUT_PREFIX and captured["lidar_time_offset_ms"] == 75.0
    assert captured["niceness"] == cli.VEHICLE_SHADOW_NICENESS == 10      # the panel's recording comes first
    # the gate's covariance mode is recorded: lateral by default since 2026-11-09, xy when asked
    assert {k: captured["gate_limits"][k] for k in ("covariance_mode", "slow_std_m", "slow_linear")} == {
        "covariance_mode": "lateral", "slow_std_m": 0.5, "slow_linear": 0.15}
    captured.clear()
    with pytest.raises(Stop):
        cli.main(argv + ["--gate-covariance-mode", "xy"])
    assert captured["gate_limits"]["covariance_mode"] == "xy" and "slow_std_m" not in captured["gate_limits"]


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


def test_lidar_time_offset_is_explicit_and_stamps_the_scan_earlier(tmp_path):
    from wc_navigation.live.cli import arguments
    from wc_navigation.live.scan_adapter import scan_stamp_ns
    assert scan_stamp_ns(1_794_037_921_007_690_136) == 1_794_037_921_007_690_136          # default: as recorded
    assert scan_stamp_ns(1_794_037_921_007_690_136, 75.0) == 1_794_037_920_932_690_136
    assert scan_stamp_ns(1_000_000_000, 75.5) == 924_500_000
    for bad in (300.0, math.nan, math.inf):
        with pytest.raises(ValueError):
            scan_stamp_ns(1_000_000_000, bad)
    common = ["inputs", "--project-root", str(tmp_path), "--cell", str(tmp_path / "cell"), "--hardware-setup", "h",
              "--gyro-bias", "b", "--bag", "x"]
    assert arguments(common + ["--lidar-time-offset-ms", "0"]).lidar_time_offset_ms == 0.0
    with pytest.raises(SystemExit):
        arguments(common + ["--lidar-time-offset-ms", "251"])
    from wc_navigation.live.amcl_cross import arguments as replay_arguments
    replay = ["--project-root", str(tmp_path), "--source-cell", "c", "--map", "m.yaml", "--alignment", "0", "1", "-85"]
    assert replay_arguments(replay).lidar_time_offset_ms == 0.0         # replay tools keep 0


def write_offset(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"schema_version": 1, "kind": "LIDAR_TIME_OFFSET", "status": "USER_APPROVED_20261108",
                                "lidar_time_offset_ms": value}))


def test_live_lidar_time_offset_comes_from_the_approved_configuration(tmp_path):
    from wc_navigation.live.cli import arguments, resolve_lidar_time_offset
    common = ["inputs", "--project-root", str(tmp_path), "--cell", str(tmp_path / "cell"), "--hardware-setup", "h",
              "--gyro-bias", "b", "--bag", "x"]
    with pytest.raises(SystemExit, match="lidar-time-offset-ms"):
        resolve_lidar_time_offset(arguments(common))                    # nothing approved, nothing given
    write_offset(tmp_path / "config/calibration/lidar_time_offset.json", 75.0)
    value, source = resolve_lidar_time_offset(arguments(common))
    assert value == 75.0 and source["source"] == "CONFIGURATION" and len(source["sha256"]) == 64
    write_offset(tmp_path / "cell/lidar_time_offset.json", 70.0)        # a frozen configuration's own copy wins
    assert resolve_lidar_time_offset(arguments(common))[0] == 70.0
    assert resolve_lidar_time_offset(arguments(common + ["--lidar-time-offset-ms", "0"])) == (0.0, {"source": "COMMAND_LINE"})
    write_offset(tmp_path / "cell/lidar_time_offset.json", 400.0)
    with pytest.raises(ValueError):
        resolve_lidar_time_offset(arguments(common))


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


# -- operator stop (Ctrl-C), priority and the read-only relay query ---------------------------------------------
def test_first_signal_stops_the_run_and_later_signals_cannot_interrupt_the_shutdown(capsys):
    import os
    import signal
    import time
    from wc_navigation.live.supervisor import InterruptGuard
    previous = signal.getsignal(signal.SIGINT)
    guard = InterruptGuard().install()
    try:
        with pytest.raises(KeyboardInterrupt):
            os.kill(os.getpid(), signal.SIGINT)
            time.sleep(1.0)
        assert guard.shutting_down
        os.kill(os.getpid(), signal.SIGINT)        # pressed again during the shutdown: printed, not raised
        time.sleep(0.05)
        os.kill(os.getpid(), signal.SIGTERM)
        time.sleep(0.05)
        assert guard.received == ["SIGINT", "SIGINT", "SIGTERM"]
        assert "收尾中" in capsys.readouterr().err
    finally:
        guard.restore()
    assert signal.getsignal(signal.SIGINT) is previous


def test_a_signal_while_a_process_starts_is_held_until_the_process_is_registered(tmp_path, monkeypatch):
    import os
    import signal
    import subprocess
    import time
    from wc_navigation.live import supervisor
    monkeypatch.setenv("WC_AUTONOMY_ROS_DOMAIN", "100")

    class FakeProcess:
        pid, returncode = 4242, None

        def poll(self):
            return self.returncode

    def fake_popen(args, **kwargs):
        os.kill(os.getpid(), signal.SIGINT)       # Ctrl-C exactly while the child is being started
        time.sleep(0.05)
        return FakeProcess()
    monkeypatch.setattr(subprocess, "Popen", fake_popen)
    (tmp_path / "logs").mkdir()
    guard = supervisor.InterruptGuard().install()
    run = supervisor.LiveSupervisor(tmp_path, tmp_path, {}, guard=guard)
    try:
        with pytest.raises(KeyboardInterrupt):
            run.launch("inputs_scan", ["true"])
        assert [name for name, _ in run.processes] == ["inputs_scan"]     # registered, so it will be stopped
        with pytest.raises(KeyboardInterrupt):
            run.launch("amcl", ["true"])                                  # nothing new starts after a stop
        assert len(run.processes) == 1
    finally:
        guard.restore()
        for handle in run.handles:
            handle.close()


def test_stop_all_stops_every_own_process_even_if_one_stop_fails(tmp_path, monkeypatch):
    import subprocess
    from wc_navigation.live import supervisor
    calls = []

    class Process:
        def __init__(self, name):
            self.name, self.code = name, None

        def poll(self):
            return self.code

    def fake_stop(process):
        calls.append(process.name)
        if process.name == "amcl":
            raise subprocess.TimeoutExpired("amcl", 4.0)
        process.code = -15
    monkeypatch.setattr(supervisor, "_stop_owned", fake_stop)
    run = supervisor.LiveSupervisor(tmp_path, tmp_path, {})
    run.processes = [(name, Process(name)) for name in ("recorder", "amcl", "safety_gate")]
    run.stop_all()
    assert calls == ["safety_gate", "amcl", "recorder"]
    assert run.report["process_exit_codes"] == {"safety_gate": -15, "amcl": None, "recorder": -15}
    assert list(run.report["stop_errors"]) == ["amcl"]


def test_result_json_is_written_after_an_operator_stop(tmp_path, monkeypatch):
    from wc_navigation.live import cli, config as live_config, shadow_summary, supervisor
    monkeypatch.setenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", "APPROVED_BY_USER_20261109")
    monkeypatch.setenv("ROS_DOMAIN_ID", "100")
    monkeypatch.setenv("WC_AUTONOMY_ROS_DOMAIN", "100")
    monkeypatch.setattr(live_config, "load_live_config", lambda *a: {"provenance": {}})
    monkeypatch.setattr(cli, "WORK_ROOT", tmp_path / "shm")

    def fake_run(self):
        # the run's processes write under the work directory; result.json must point into the run directory
        (self.output / "logs").mkdir()
        (self.output / "logs" / "amcl.log").write_text("amcl log\n")
        return {"stages": [], "process_exit_codes": {"amcl": -15}, "stopped": supervisor.INTERRUPTED,
                "signals": ["SIGINT"], "logs": {"amcl": str(self.output / "logs" / "amcl.log")}}
    monkeypatch.setattr(supervisor.LiveSupervisor, "run", fake_run)
    summaries = iter([{"commands": {"count": 3}}, RuntimeError("bad csv"), {"commands": {"count": 3}}])

    def fake_summary(config, output):
        value = next(summaries)
        if isinstance(value, Exception):
            raise value
        return value
    monkeypatch.setattr(shadow_summary, "summarize_shadow", fake_summary)
    monkeypatch.setattr("wc_runtime.storage_policy.StoragePolicy.check", lambda self: None)
    runs = iter(range(10))
    monkeypatch.setattr("wc_runtime.storage_policy.StoragePolicy.resolve",
                        lambda self, value: tmp_path / ("run%d" % next(runs)))
    map_yaml = tmp_path / "m.yaml"
    map_yaml.write_text("image: m.pgm\n")
    (tmp_path / "bag").mkdir()
    write_offset(tmp_path / "config/calibration/lidar_time_offset.json", 75.0)
    common = ["shadow", "--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b",
              "--map", str(map_yaml), "--initial-pose", "0", "0", "0", "--goal", "1", "2", "0"]
    vehicle = common + ["--source", "vehicle", "--sensor-domain", "83"]
    assert cli.main(vehicle) == 0                    # Ctrl-C is how a vehicle run ends
    result = json.loads((tmp_path / "run0" / "result.json").read_text())
    assert result["status"] == "COMPLETE" and result["supervisor"]["stopped"] == "OPERATOR_INTERRUPT"
    assert (tmp_path / "run0" / "logs" / "amcl.log").read_text() == "amcl log\n"     # copied in from the work dir
    assert result["supervisor"]["logs"]["amcl"] == str(tmp_path / "run0" / "logs" / "amcl.log")
    assert result["supervisor"]["run_files"]["copied_files"] == 1 and not list((tmp_path / "shm").iterdir())
    assert cli.main(vehicle) == 1                    # the summary failed: result.json is written anyway
    result = json.loads((tmp_path / "run1" / "result.json").read_text())
    assert result["status"] == "INCOMPLETE" and result["summary_error"] == "RuntimeError: bad csv"
    assert cli.main(common + ["--source", "bag", "--bag", str(tmp_path / "bag")]) == 1   # bag stopped early
    result = json.loads((tmp_path / "run2" / "result.json").read_text())
    assert result["status"] == "INCOMPLETE" and result["commands"] == {"count": 3}


def test_a_run_stopped_by_the_start_check_is_incomplete(tmp_path, monkeypatch):
    from wc_navigation.live import cli, config as live_config, shadow_summary, supervisor
    monkeypatch.setenv("WC_AUTONOMY_ROS_DOMAIN", "100")
    monkeypatch.setattr(live_config, "load_live_config", lambda *a: {"provenance": {}})
    monkeypatch.setattr(cli, "WORK_ROOT", tmp_path / "shm")
    seen = {}

    def fake_run(self):
        seen["start_check"] = self.config["start_check"]
        return {"stages": [], "process_exit_codes": {}, "stopped": "INITIAL_POSE_MISMATCH",
                "start_check": {"status": "INITIAL_POSE_MISMATCH"}}
    monkeypatch.setattr(supervisor.LiveSupervisor, "run", fake_run)
    monkeypatch.setattr(shadow_summary, "summarize_shadow", lambda config, output: {"commands": {"count": 0}})
    monkeypatch.setattr("wc_runtime.storage_policy.StoragePolicy.check", lambda self: None)
    monkeypatch.setattr("wc_runtime.storage_policy.StoragePolicy.resolve", lambda self, value: tmp_path / "run0")
    map_yaml = tmp_path / "m.yaml"
    map_yaml.write_text("image: m.pgm\n")
    (tmp_path / "bag").mkdir()
    write_offset(tmp_path / "config/calibration/lidar_time_offset.json", 75.0)
    argv = ["shadow", "--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b",
            "--map", str(map_yaml), "--initial-pose", "0", "0", "0", "--goal", "1", "2", "0", "--source", "bag",
            "--bag", str(tmp_path / "bag")]
    assert cli.main(argv) == 1
    assert seen["start_check"] == "block"                       # the default
    result = json.loads((tmp_path / "run0" / "result.json").read_text())
    assert result["status"] == "INCOMPLETE" and result["supervisor"]["stopped"] == "INITIAL_POSE_MISMATCH"
    with pytest.raises(SystemExit):
        cli.arguments(argv + ["--start-check", "maybe"])
    assert cli.arguments(argv + ["--start-check", "strict"]).start_check == "strict"


def test_vehicle_priority_is_only_ever_lowered(monkeypatch):
    import os
    from wc_navigation.live import supervisor
    state = {"nice": 3}

    def fake_nice(increment):
        state["nice"] += increment
        return state["nice"]
    monkeypatch.setattr(os, "nice", fake_nice)
    assert supervisor.lower_priority(10) == 10 and state["nice"] == 10
    assert supervisor.lower_priority(5) == 10 and state["nice"] == 10


def test_relay_graph_query_needs_the_approval_localhost_only_and_a_valid_prefix(monkeypatch):
    from wc_navigation.live import cli
    monkeypatch.delenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", raising=False)
    with pytest.raises(SystemExit, match="APPROVED_BY_USER"):
        cli.main(["relay-graph", "--sensor-domain", "83"])
    monkeypatch.setenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", "APPROVED_BY_USER_20261109")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "0")
    with pytest.raises(SystemExit, match="LOCALHOST"):
        cli.main(["relay-graph", "--sensor-domain", "83"])
    for bad in (["--source-prefix", "relay"], ["--wait", "0.1"]):
        with pytest.raises(SystemExit):
            cli.arguments(["relay-graph", "--sensor-domain", "83", *bad])
    with pytest.raises(SystemExit):
        cli.arguments(["relay-graph", "--sensor-domain", "300"])


# -- start check ------------------------------------------------------------------------------------------------
def _start_check_scorer():
    """10 x 10 m map, 0.05 m cells: a corridor y -1.2..1.2 along x with a wall across it at x = 4 (only ahead)."""
    from wc_navigation.live.amcl_cross import EndpointScorer
    data = np.zeros((200, 200), dtype=np.int16)                 # origin (-5, -5)
    for y in (-1.2, 1.2):
        data[int((y + 5) / 0.05), :] = 100
    data[int((-1.2 + 5) / 0.05):int((1.2 + 5) / 0.05) + 1, int((4.0 + 5) / 0.05)] = 100
    return EndpointScorer(data, 0.05, (-5.0, -5.0, 0.0))


def _scans_seen_from(pose, scorer, count=10):
    """Ranges of a 161-bin, +-60 deg scan cast from the laser at ``pose`` (laser = base, no lever) on the map."""
    from wc_navigation.autonomy.replay_mapping import ANGLES
    ranges = np.full(len(ANGLES), np.inf)
    for k, angle in enumerate(ANGLES):
        heading = pose[2] + angle
        for r in np.arange(0.3, 8.0, 0.02):
            xy = np.array([[pose[0] + r * math.cos(heading), pose[1] + r * math.sin(heading)]])
            if scorer.lookup(xy)[0] < 0.03:
                ranges[k] = r
                break
    return [(ranges, float(ANGLES[0]), float(ANGLES[1] - ANGLES[0]))] * count


def test_start_check_passes_the_right_heading_and_stops_the_turned_one():
    from wc_navigation.live import start_check
    scorer = _start_check_scorer()
    facing_wall = _scans_seen_from((1.0, 0.0, 0.0), scorer)           # the wall across the corridor is 3 m ahead
    right = start_check.evaluate(scorer, facing_wall, (0.0, 0.0), (1.0, 0.0, 0.0))
    assert right["status"] == "PASS" and right["given"]["best"] > 0.9 and right["turned_180"]["best"] < 0.7
    wrong = start_check.evaluate(scorer, facing_wall, (0.0, 0.0), (1.0, 0.0, math.pi))
    assert wrong["status"] == "INITIAL_POSE_MISMATCH"
    assert start_check.stops_run("INITIAL_POSE_MISMATCH", "block") and not start_check.stops_run("PASS", "strict")
    assert "反了" in start_check.operator_message(wrong, "block")


def test_start_check_reports_a_corridor_view_as_ambiguous():
    from wc_navigation.live import start_check
    scorer = _start_check_scorer()
    along = _scans_seen_from((-3.0, 0.0, math.pi), scorer)          # looking down the corridor away from the wall
    result = start_check.evaluate(scorer, along, (0.0, 0.0), (-3.0, 0.0, math.pi))
    assert result["status"] == "AMBIGUOUS"
    assert not start_check.stops_run("AMBIGUOUS", "block") and start_check.stops_run("AMBIGUOUS", "strict")
    assert not start_check.stops_run("INITIAL_POSE_MISMATCH", "warn")
    assert start_check.evaluate(scorer, along[:3], (0.0, 0.0), (-3.0, 0.0, 0.0))["status"] == "START_CHECK_NO_DATA"
    assert start_check.moved((0.0, 0.0, 0.0), (0.06, 0.0, 0.0)) and not start_check.moved((0, 0, 0), (0.02, 0, 0.01))


def test_run_files_are_copied_from_the_work_directory_and_kept_when_a_copy_fails(tmp_path, monkeypatch):
    import shutil
    from wc_navigation.live.cli import move_run_files
    work, output = tmp_path / "shm" / "wc_live_x", tmp_path / "run"
    (work / "logs").mkdir(parents=True)
    (work / "ros_logs" / "a").mkdir(parents=True)
    (work / "logs" / "gate.log").write_text("g")
    (work / "ros_logs" / "a" / "x.log").write_text("x")
    (work / "commands.csv").write_text("time_s\n1\n")
    output.mkdir()
    (output / "run_config.json").write_text("{}")
    moved = move_run_files(work, output)
    assert moved["copied_files"] == 3 and moved["work_dir_removed"] and not work.exists()
    assert (output / "ros_logs" / "a" / "x.log").read_text() == "x" and (output / "run_config.json").exists()
    (work / "logs").mkdir(parents=True)
    (work / "logs" / "a.log").write_text("a")
    real_copy = shutil.copy2

    def failing_copy(source, target):
        raise OSError("disk full")
    monkeypatch.setattr(shutil, "copy2", failing_copy)
    kept = move_run_files(work, output)
    assert not kept["work_dir_removed"] and kept["errors"] and (work / "logs" / "a.log").exists()
    monkeypatch.setattr(shutil, "copy2", real_copy)


# -- D5: the relay reads the right lidar too when the 3D obstacle input is on -------------------------------------
def test_the_relay_reads_four_topics_only_with_the_obstacle_input(monkeypatch):
    from wc_navigation.live import relay_node
    from wc_navigation.live.config import RIGHT_FILTERED_TOPIC
    assert relay_node.relayed_topics() == relay_node.RELAYED_TOPICS and len(relay_node.RELAYED_TOPICS) == 3
    four = relay_node.relayed_topics(True)
    assert four == relay_node.RELAYED_TOPICS + (RIGHT_FILTERED_TOPIC,)
    assert relay_node.source_qos(RIGHT_FILTERED_TOPIC).depth == 3
    with pytest.raises(ValueError, match="obstacles"):
        relay_node.check_relay_config({"source_domain": 100, "source_prefix": "/p", "obstacles": "yes"}, "100")
    expected = ["/p" + t for t in four]
    subscriptions = [(t, ["type"]) for t in expected]
    assert relay_node.graph_check([], subscriptions, [], [], expected)["status"] == "PASS"
    assert relay_node.graph_check([], subscriptions[:3], [], [], expected)["status"] == "FAIL"    # right lidar missing
    three = ["/p" + t for t in relay_node.RELAYED_TOPICS]
    assert relay_node.graph_check([], subscriptions, [], [], three)["status"] == "FAIL"           # one too many


def test_vehicle_runs_with_the_obstacle_input_relay_the_right_lidar(tmp_path, monkeypatch):
    from wc_navigation.live import cli, config as live_config, supervisor
    captured = {}

    class Stop(Exception):
        pass

    def fake_run(self):
        captured.update(self.config)
        raise Stop()
    monkeypatch.setenv("WC_LIVE_SHADOW_VEHICLE_APPROVAL", "APPROVED_BY_USER_20261110")
    monkeypatch.setenv("ROS_DOMAIN_ID", "100")
    monkeypatch.setenv("WC_AUTONOMY_ROS_DOMAIN", "100")
    monkeypatch.setattr(live_config, "load_live_config", lambda *a: {"provenance": {}})
    monkeypatch.setattr(supervisor.LiveSupervisor, "run", fake_run)
    monkeypatch.setattr(cli, "WORK_ROOT", tmp_path / "shm")
    monkeypatch.setattr("wc_runtime.storage_policy.StoragePolicy.check", lambda self: None)
    monkeypatch.setattr("wc_runtime.storage_policy.StoragePolicy.resolve", lambda self, value: tmp_path / "run0")
    monkeypatch.setattr("wc_navigation.perception.shadow_cli_options.live_entries", lambda *a: {"plan": "stub"})
    map_yaml = tmp_path / "m.yaml"
    map_yaml.write_text("image: m.pgm\n")
    write_offset(tmp_path / "config/calibration/lidar_time_offset.json", 75.0)
    argv = ["shadow", "--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b",
            "--source", "vehicle", "--sensor-domain", "83", "--map", str(map_yaml), "--initial-pose", "0", "0", "0",
            "--goal", "1", "2", "0", "--obstacle-input", "scan", "--obstacle-scan", "filtered",
            "--obstacle-monitor", "cloud"]
    with pytest.raises(Stop):
        cli.main(argv)
    assert captured["relay"]["obstacles"] is True and captured["obstacles"] == {"plan": "stub"}
    assert cli.arguments(["relay-graph", "--sensor-domain", "83", "--obstacles"]).obstacles is True
    with pytest.raises(SystemExit):
        cli.arguments(argv[:-2] + ["--smoother-timeout", "0.1"])             # outside window B's 0.2-5 s


def test_the_shadow_wires_the_gated_smoother_feedback(tmp_path):
    from wc_navigation.autonomy import nav2_config
    from wc_navigation.live.supervisor import LiveSupervisor
    config = {"use_sim_time": False, "map_yaml": "m.yaml", "amcl": {"params": "framework", "changes_from_framework": {}},
              "gate_limits": {"correction_translation_m": 0.25, "correction_yaw_deg": 10.0,
                              "correction_reference": "robot", "covariance_xy_std_m": 0.5,
                              "covariance_yaw_std_deg": 14.3, "covariance_mode": "lateral", "slow_std_m": 0.5,
                              "slow_linear": 0.15},
              "goals": [[1.0, 2.0, 0.0]], "advance_radius_m": 1.0,
              "smoother_feedback": "gated", "smoother_timeout_s": 0.3}
    run = LiveSupervisor(tmp_path, tmp_path, config)
    files = run.prepare_shadow(nav2_config.NavProfile())
    feedback = json.loads(files["feedback"].read_text())
    assert feedback["use_sim_time"] is False and feedback["log_csv"] == str(tmp_path / "velocity_feedback.csv")
    import yaml
    smoother = yaml.safe_load(files["nav2"].read_text())["velocity_smoother"]["ros__parameters"]
    assert (smoother["feedback"], smoother["velocity_timeout"], smoother["use_sim_time"]) == ("CLOSED_LOOP", 0.3, False)
    expected = nav2_config.readback_expectations(nav2_config.NavProfile(), "navigate", planning=run.planning())
    assert expected["velocity_smoother"]["feedback"] == "CLOSED_LOOP"
    open_loop = LiveSupervisor(tmp_path / "b", tmp_path / "b", {**config, "smoother_feedback": "open_loop"})
    (tmp_path / "b").mkdir()
    assert "feedback" not in open_loop.prepare_shadow(nav2_config.NavProfile())


# -- vehicle closed loop (BOARD 2026-11-10 14:40; D.md 16:45) -------------------------------------------------------
def _closed_argv(tmp_path, *extra):
    map_yaml = tmp_path / "m.yaml"
    map_yaml.write_text("image: m.pgm\n")
    return ["shadow", "--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b",
            "--source", "vehicle", "--sensor-domain", "83", "--map", str(map_yaml), "--initial-pose", "0", "0", "0",
            "--goal", "4", "0", "0", "--closed-loop", "--panel-root", str(tmp_path / "panel"),
            "--session-root", str(tmp_path / "session"), "--session-id", "capture_x", *extra]


def test_closed_loop_options_fix_the_first_closed_loop_settings(tmp_path):
    from wc_navigation.live import cli
    args = cli.arguments(_closed_argv(tmp_path))
    assert args.closed_loop and args.monitor_base_shift == "off" and args.start_check == "block"
    assert (args.smoother_feedback, args.smoother_timeout) == ("open_loop", 0.3)     # BOARD 2026-11-10 14:40
    assert args.gate_covariance == {"covariance_mode": "lateral", "slow_std_m": 0.5, "slow_linear": 0.05}
    assert cli.arguments(_closed_argv(tmp_path, "--monitor-base-shift", "off")).monitor_base_shift == "off"
    assert cli.arguments(_closed_argv(tmp_path, "--smoother-feedback", "open_loop")).smoother_feedback == "open_loop"
    assert cli.arguments(_closed_argv(tmp_path, "--start-check", "strict")).start_check == "strict"
    for bad in (["--start-check", "warn"], ["--start-check", "off"], ["--gate-slow-limits", "0.5", "0.15"],
                ["--monitor-base-shift", "on"], ["--monitor-b=on"], ["--smoother-feedback", "gated"],
                ["--smoother-timeout", "0.5"]):
        with pytest.raises(SystemExit):
            cli.arguments(_closed_argv(tmp_path, *bad))       # 0.15 m/s is above the closed loop's 0.10 m/s
    # window C's slowdown-box margin goes with the closed loop, with or without an explicit --monitor-base-shift off
    for extra in (["--monitor-slow-margin", "0.15"], ["--monitor-slow-margin", "0.15", "--monitor-base-shift", "off"]):
        margin = cli.arguments(_closed_argv(tmp_path, *extra))
        assert (margin.monitor_slow_margin, margin.monitor_base_shift) == (0.15, "off")
    argv = _closed_argv(tmp_path)
    with pytest.raises(SystemExit):
        cli.arguments(argv[:argv.index("--session-id")])       # the session must be named
    shadow = argv[:argv.index("--closed-loop")]
    plain = cli.arguments(shadow)
    assert not plain.closed_loop and plain.monitor_base_shift == "on" and plain.gate_covariance["slow_linear"] == 0.15
    assert plain.smoother_feedback == "gated"                  # the shadow keeps its default
    with pytest.raises(SystemExit):
        cli.arguments(shadow + ["--session-id", "capture_x"])  # closed-loop options without --closed-loop


def test_closed_loop_needs_todays_approval_the_bridge_domain_and_the_owner_socket(tmp_path, monkeypatch):
    import socket as socketlib
    from datetime import datetime
    from wc_navigation.live import cli
    args = cli.arguments(_closed_argv(tmp_path))
    (tmp_path / "session").mkdir()
    path = tmp_path / "s" / "auto.sock"
    seen = []
    monkeypatch.setattr("wc_runtime.mapping_auto.auto_socket_path",
                        lambda root, directory, session: seen.append((root, directory, session)) or path)
    monkeypatch.setenv("ROS_DOMAIN_ID", "100")
    monkeypatch.setenv("WC_AUTO_DRIVE_APPROVAL", "APPROVED_BY_USER_20200101")
    with pytest.raises(SystemExit, match="today"):
        cli.closed_loop_settings(args)
    monkeypatch.setenv("WC_AUTO_DRIVE_APPROVAL", "APPROVED_BY_USER_" + datetime.now().strftime("%Y%m%d"))
    with pytest.raises(SystemExit, match="no automatic control socket"):
        cli.closed_loop_settings(args)
    path.parent.mkdir()
    path.write_text("not a socket")
    with pytest.raises(SystemExit, match="no automatic control socket"):
        cli.closed_loop_settings(args)
    path.unlink()
    server = socketlib.socket(socketlib.AF_UNIX, socketlib.SOCK_STREAM)
    try:
        server.bind(str(path))
        settings = cli.closed_loop_settings(args)
    finally:
        server.close()
        path.unlink(missing_ok=True)    # the test runner archives tmp_path and refuses special files (sockets)
    assert settings["auto_socket"] == str(path) and settings["bridge_node"] == "/wc_auto_bridge"
    assert (settings["max_linear"], settings["max_angular"], settings["enable_timeout_s"]) == (0.10, 0.30, 0.5)
    assert seen[-1] == (tmp_path / "panel", tmp_path / "session", "capture_x")
    assert (settings["panel_root"], settings["session_root"]) == (str(tmp_path / "panel"), str(tmp_path / "session"))
    monkeypatch.setenv("ROS_DOMAIN_ID", "99")
    with pytest.raises(SystemExit, match="domain 100"):
        cli.closed_loop_settings(args)


def test_the_closed_loop_gate_and_goals_configuration(tmp_path):
    from wc_navigation.autonomy import nav2_config
    from wc_navigation.autonomy.safety_gate import GateConfig
    from wc_navigation.live.supervisor import LiveSupervisor
    import yaml
    config = {"use_sim_time": False, "map_yaml": "m.yaml", "amcl": {"params": "framework", "changes_from_framework": {}},
              "gate_limits": {"correction_translation_m": 0.25, "correction_yaw_deg": 10.0,
                              "correction_reference": "robot", "covariance_xy_std_m": 0.5,
                              "covariance_yaw_std_deg": 14.3, "covariance_mode": "lateral", "slow_std_m": 0.5,
                              "slow_linear": 0.05},
              "goals": [[4.0, 0.0, 0.0]], "advance_radius_m": 1.0, "smoother_feedback": "open_loop",
              "smoother_timeout_s": 0.3,
              "closed_loop": {"bridge_node": "/wc_auto_bridge", "enable_timeout_s": 0.5, "panel_root": "/p",
                              "session_root": "/s", "session_id": "capture_x"}}
    profile = nav2_config.NavProfile(max_linear=0.10, max_angular=0.30)
    run = LiveSupervisor(tmp_path, tmp_path, config)
    files = run.prepare_shadow(profile)
    # window B 2026-11-10: the rotation speeds are read back in the closed loop on top of Nav2's list
    expected = run.readback_expectations(profile)
    assert expected["controller_server"]["FollowPath.rotate_to_heading_angular_vel"] == 0.30
    assert expected["behavior_server"]["max_rotational_vel"] == 0.30
    assert expected["controller_server"]["FollowPath.desired_linear_vel"] == 0.10      # Nav2's own list kept
    node = json.loads(files["gate"].read_text())
    gate = GateConfig(**node["gate"])
    assert node["clock"] == "monotonic" and node["use_sim_time"] is False
    assert (gate.require_base, gate.enable_source_node, gate.enable_timeout_s, gate.command_timeout_s) == (
        True, "/wc_auto_bridge", 0.5, 0.5)
    assert (gate.max_linear, gate.max_angular, gate.slow_linear, gate.slow_std_m) == (0.10, 0.30, 0.05, 0.5)
    assert json.loads(files["goals"].read_text())["closed_loop"] is True
    assert "feedback" not in files                                 # open loop: no velocity_feedback node
    nav2 = yaml.safe_load(files["nav2"].read_text())
    assert nav2["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"] == 0.10
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    assert (smoother["max_velocity"], smoother["feedback"], smoother["velocity_timeout"]) == (
        [0.10, 0.0, 0.30], "OPEN_LOOP", 0.3)
    shadow = {k: v for k, v in config.items() if k != "closed_loop"}
    (tmp_path / "b").mkdir()
    plain = LiveSupervisor(tmp_path / "b", tmp_path / "b", shadow)
    files = plain.prepare_shadow(nav2_config.NavProfile())
    assert plain.readback_expectations(nav2_config.NavProfile()) == nav2_config.readback_expectations(
        nav2_config.NavProfile(), "navigate", planning=plain.planning())                  # the shadow's list as before
    node = json.loads(files["gate"].read_text())
    assert "clock" not in node and "require_base" not in node["gate"] and "enable_source_node" not in node["gate"]
    assert json.loads(files["goals"].read_text())["closed_loop"] is False


def test_closed_loop_rotation_speeds_must_be_given_and_within_the_angular_limit():
    from wc_navigation.autonomy.bringup import RunError
    from wc_navigation.live.supervisor import closed_loop_rotation_readback

    def params(rotate, spin):
        return {"controller_server": {"ros__parameters": {"FollowPath": {"rotate_to_heading_angular_vel": rotate}}},
                "behavior_server": {"ros__parameters": {"max_rotational_vel": spin}}}
    assert closed_loop_rotation_readback(params(0.3, 0.25), 0.30) == {
        "controller_server": {"FollowPath.rotate_to_heading_angular_vel": 0.3},
        "behavior_server": {"max_rotational_vel": 0.25}}
    for rotate, spin in ((0.4, 0.3), (0.3, 0.31), (None, 0.3), (0.3, True), (0.0, 0.3)):
        with pytest.raises(RunError):
            closed_loop_rotation_readback(params(rotate, spin), 0.30)
    with pytest.raises(RunError, match="missing"):
        closed_loop_rotation_readback({"controller_server": {"ros__parameters": {}}}, 0.30)


def test_the_base_bridge_is_stopped_first(tmp_path, monkeypatch):
    from wc_navigation.live import supervisor
    calls = []

    class Process:
        def __init__(self, name):
            self.name, self.code = name, None

        def poll(self):
            return self.code

    def fake_stop(process):
        calls.append(process.name)
        process.code = -2
    monkeypatch.setattr(supervisor, "_stop_owned", fake_stop)
    run = supervisor.LiveSupervisor(tmp_path, tmp_path, {})
    run.processes = [(name, Process(name)) for name in ("recorder", "amcl", supervisor.BRIDGE, "safety_gate", "goals")]
    run.stop_all()
    assert calls == [supervisor.BRIDGE, "goals", "safety_gate", "amcl", "recorder"]


def test_the_closed_loop_status_line_shows_what_holds_the_chair():
    from wc_navigation.live.supervisor import (BASE_STATUS_TOPIC, GATE_STATUS_TOPIC, GOAL_STATUS_TOPIC,
                                               closed_loop_line, closed_loop_report)
    assert "闸门 未启动" in closed_loop_line({}) and "没有桥的状态" in closed_loop_line({})
    status = {GATE_STATUS_TOPIC: {"state": "DISABLED", "reasons": ["OPERATOR_ENABLE_OFF"], "linear": 0.0,
                                  "angular": 0.0, "gate_id": "gate_1", "seq": 40, "counts": {"DISABLED": 40},
                                  "conditions": {"localization_ok": True, "localization_reason": ""},
                                  "preview": {"state": "ACTIVE", "reasons": ["LINEAR_CLAMPED"], "linear": 0.1,
                                              "angular": 0.02}},
              BASE_STATUS_TOPIC: {"auto_state": "NONE", "base_auto": False, "reason": "BASE_NOT_AUTO",
                                  "enable": False, "estop": None, "estop_request": False},
              GOAL_STATUS_TOPIC: {"state": "NAVIGATING", "goal_index": 0, "goals": 1}}
    line = closed_loop_line(status)
    for part in ("闸门 DISABLED(OPERATOR_ENABLE_OFF)", "影子判断 会放行 0.10 m/s 0.02 rad/s(LINEAR_CLAMPED)", "定位 可信",
                 "底盘 NONE(BASE_NOT_AUTO)", "使能 否", "急停 否", "目标 NAVIGATING 1/1"):
        assert part in line
    status[BASE_STATUS_TOPIC].update(estop_request=True)
    status[GATE_STATUS_TOPIC]["conditions"] = {"localization_ok": False, "localization_reason": "MAP_TO_BASE_TF_STALE"}
    status[GATE_STATUS_TOPIC]["preview"] = {"state": "LOCALIZATION_UNTRUSTED", "reasons": ["MAP_TO_BASE_TF_STALE"]}
    line = closed_loop_line(status)
    assert "急停 按下" in line and "定位 不可信(MAP_TO_BASE_TF_STALE)" in line
    assert "影子判断 LOCALIZATION_UNTRUSTED(MAP_TO_BASE_TF_STALE)" in line
    report = closed_loop_report(status)
    assert (report["gate_id"], report["gate_seq"], report["base_status"]["auto_state"]) == ("gate_1", 40, "NONE")


def test_closed_loop_goals_go_out_at_once_and_are_not_retried_once_the_chair_drove(monkeypatch):
    """GoalSequence without ROS: a stand-in node, action client and TF lookup."""
    import types
    from wc_navigation.live import goals_node

    class Future:
        def __init__(self, value):
            self.value = value

        def done(self):
            return True

        def result(self):
            return self.value

    class Client:
        sent = []

        def __init__(self, *args):
            pass

        def server_is_ready(self):
            return True

        def send_goal_async(self, goal):
            Client.sent.append(goal)
            handle = types.SimpleNamespace(accepted=True, get_result_async=lambda: Future(
                types.SimpleNamespace(status=6)), cancel_goal_async=lambda: None)
            return Future(handle)

    published, clock = [], [10.0]
    node = types.SimpleNamespace(create_publisher=lambda *a: types.SimpleNamespace(publish=published.append),
                                 create_subscription=lambda *a: None, create_timer=lambda *a: None,
                                 get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(
                                     nanoseconds=int(clock[0] * 1e9), to_msg=lambda: None)))
    monkeypatch.setattr("rclpy.action.ActionClient", Client)
    monkeypatch.setattr("tf2_ros.Buffer", lambda: None)
    monkeypatch.setattr("tf2_ros.TransformListener", lambda *a: None)
    monkeypatch.setattr(goals_node, "lookup_pose", lambda *a: ((3.5, 0.0, 0.0), 10.0))   # 0.5 m from the goal
    monkeypatch.setattr(goals_node, "pose_stamped", lambda *a: "pose")
    monkeypatch.setattr(goals_node, "json_message", lambda value: value)
    monkeypatch.setattr("nav2_msgs.action.NavigateToPose", types.SimpleNamespace(
        Goal=lambda: types.SimpleNamespace(pose=None)))
    sequence = goals_node.GoalSequence(node, {"goals": [[4.0, 0.0, 0.0]], "closed_loop": True})
    sequence.tick()                         # at once, although the chair is within the advance radius of the last goal
    assert len(Client.sent) == 1 and published[-1]["state"] == "NAVIGATING"
    sequence.tick()                         # Nav2 aborted (status 6) while standing before the start: re-sent
    assert published[-1]["state"] == "FAILED_RETRYING" and not sequence.finished
    clock[0] += 3.0
    sequence.tick()
    assert len(Client.sent) == 2
    status = types.SimpleNamespace(data=json.dumps({"state": "ACTIVE"}))
    sequence.on_gate_status(status)         # the operator started, the gate passed a command: the chair drove
    for _ in range(3):
        clock[0] += 3.0
        sequence.tick()                     # aborted again: the sequence ends, nothing is re-sent
    assert len(Client.sent) == 2 and sequence.finished and published[-1]["state"] == "FAILED"
    assert [r["reason"] for r in sequence.records] == ["GATE_ACTIVE_FIRST", "NAV2_FAILED"]
