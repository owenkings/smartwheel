import json
import math
from pathlib import Path

import numpy as np
import pytest

from wc_navigation.autonomy import nav2_config, shadow_localization as sl


def base_args(tmp_path):
    return ["--project-root", str(tmp_path), "--source-cell", str(tmp_path), "--map", str(tmp_path / "m.yaml")]


def parse(tmp_path, extra):
    from wc_navigation.autonomy.shadow_cli import arguments
    return arguments(base_args(tmp_path) + extra)


def test_identity_stays_the_default_and_rejects_localization_options(tmp_path):
    args = parse(tmp_path, [])
    assert args.localization == "identity"
    assert sl.config_section(args, tmp_path, None, None) == {}
    for extra in (["--map-alignment", "0", "1", "-85"], ["--amcl-params", "candidate"],
                  ["--initial-pose-offset", "0.1", "0", "0"], ["--gate-correction-limits", "0.4", "8"]):
        with pytest.raises(SystemExit):
            parse(tmp_path, extra)


def test_fixed_and_amcl_need_exactly_one_alignment(tmp_path):
    for mode in ("fixed", "amcl"):
        with pytest.raises(SystemExit):
            parse(tmp_path, ["--localization", mode])
        with pytest.raises(SystemExit):
            parse(tmp_path, ["--localization", mode, "--map-alignment", "0", "1", "-85",
                             "--map-alignment-json", str(tmp_path / "a.json")])
    with pytest.raises(SystemExit):
        parse(tmp_path, ["--localization", "fixed", "--map-alignment", "0", "1", "-85", "--amcl-params", "candidate"])
    for bad in (["--initial-pose-offset", "3", "0", "0"], ["--initial-pose-std", "0", "5"],
                ["--gate-correction-limits", "-1", "5"], ["--map-alignment", "nan", "1", "-85"]):
        extra = ["--localization", "amcl"] + (["--map-alignment", "0", "1", "-85"] if "--map-alignment" not in bad else [])
        with pytest.raises(SystemExit):
            parse(tmp_path, extra + bad)


def recording(n=20):
    times = np.arange(n, dtype=float)
    base = np.column_stack([0.3 * times, np.zeros(n), np.zeros(n)])
    lidar = base + [0.323, 0.22875, 0.0]
    return {"times_s": times, "base_poses": base, "lidar_poses": lidar, "lever_xy": np.array([0.323, 0.22875]),
            "ranges": np.full((n, 161), np.inf, dtype=np.float32)}


def test_amcl_start_is_the_aligned_window_start_plus_the_body_frame_offset(tmp_path):
    args = parse(tmp_path, ["--localization", "amcl", "--map-alignment", "1.0", "2.0", "90",
                            "--initial-pose-offset", "0.5", "0.2", "10", "--amcl-params", "candidate"])
    section = sl.config_section(args, tmp_path, recording(), [3.0, 15.0])
    start = section["amcl"]["aligned_start_map"]
    assert start == pytest.approx([1.0, 2.0 + 0.9, math.pi / 2])          # recorded (0.9, 0, 0) rotated by 90 deg
    assert section["amcl"]["initial_pose_map"] == pytest.approx([1.0 - 0.2, 2.9 + 0.5, math.pi / 2 + math.radians(10)])
    assert section["amcl"]["initial_pose_std"] == pytest.approx([0.25, 0.15])
    assert section["amcl"]["changes_from_framework"]["alpha1"] == 0.05
    assert section["gate_limits"]["defaults"] is True
    assert section["carrot_recording"] == str(tmp_path / sl.MAP_RECORDING)
    assert section["localization"] == sl.LABELS["amcl"]


def test_map_frame_recording_moves_poses_and_keeps_the_odometry_frame(tmp_path):
    config = {"localization_mode": "fixed", "map_alignment": {"x_m": 1.0, "y_m": -1.0, "yaw_deg": 180.0}}
    source = recording(5)
    placed = sl.map_frame_recording(config, source)
    np.testing.assert_allclose(placed["base_poses_odom"], source["base_poses"])
    np.testing.assert_allclose(placed["base_poses"][:, :2], np.column_stack([1.0 - source["base_poses"][:, 0],
                                                                            np.full(5, -1.0)]), atol=1e-12)
    assert abs(abs(placed["base_poses"][0, 2]) - math.pi) < 1e-12
    np.testing.assert_allclose(placed["times_s"], source["times_s"])
    assert sl.map_frame_recording({}, source) is source


def test_gate_config_is_unchanged_without_amcl_and_requires_covariance_with_it(tmp_path):
    profile = nav2_config.NavProfile()
    expected = {"gate": {"max_linear": profile.max_linear, "max_angular": profile.max_angular,
                         "require_covariance": False}}
    assert sl.gate_config(profile, {}) == expected
    assert sl.gate_config(profile, {"localization_mode": "fixed"}) == expected
    args = parse(tmp_path, ["--localization", "amcl", "--map-alignment", "0", "0", "0",
                            "--gate-correction-limits", "0.5", "8", "--gate-covariance-limits", "0.4", "20"])
    section = sl.config_section(args, tmp_path, recording(), [0.0, 10.0])
    config = sl.gate_config(profile, section)
    assert config["gate"]["require_covariance"] is True
    assert config["gate"]["covariance_xy_std_max_m"] == 0.4
    assert config["gate"]["covariance_yaw_std_max_rad"] == pytest.approx(math.radians(20))
    assert config["correction_monitor"] == {"window_s": 2.0, "max_translation_m": 0.5,
                                            "max_yaw_rad": pytest.approx(math.radians(8)), "hold_s": 3.0}


class FakeSupervisor:
    def __init__(self, output):
        self.output = Path(output)

    def write(self, name, value):
        import yaml
        path = self.output / name
        path.write_text(yaml.safe_dump(value) if name.endswith(".yaml") else json.dumps(value), encoding="utf-8")
        return path


def test_localization_files_list_amcl_only_when_used(tmp_path):
    import yaml
    np.savez(tmp_path / "recording.npz", lever_xy=np.array([0.323, 0.22875]))
    profile = nav2_config.NavProfile()
    base = {"map_yaml": str(tmp_path / "m.yaml"), "recording": str(tmp_path / "recording.npz"),
            "map_alignment": {"x_m": 1.0, "y_m": 2.0, "yaw_deg": 90.0}}
    (tmp_path / "fixed").mkdir()
    files = sl.write_files(FakeSupervisor(tmp_path / "fixed"), profile, {**base, "localization_mode": "fixed"})
    mapping = yaml.safe_load(Path(files["mapping"]).read_text())
    assert set(mapping) == {"map_server", "lifecycle_manager_localization"}
    assert mapping["lifecycle_manager_localization"]["ros__parameters"]["node_names"] == ["map_server"]
    helper = json.loads(Path(files["localization_helper"]).read_text())
    assert helper["mode"] == "fixed" and helper["map_odom_xyyaw"] == pytest.approx([1.0, 2.0, math.pi / 2])
    assert helper["lever_xy"] == pytest.approx([0.323, 0.22875])
    (tmp_path / "amcl").mkdir()
    config = {**base, "localization_mode": "amcl", "amcl": {"changes_from_framework": {"alpha1": 0.05, "z_hit": 0.9}}}
    files = sl.write_files(FakeSupervisor(tmp_path / "amcl"), profile, config)
    mapping = yaml.safe_load(Path(files["mapping"]).read_text())
    assert mapping["lifecycle_manager_localization"]["ros__parameters"]["node_names"] == ["map_server", "amcl"]
    amcl = mapping["amcl"]["ros__parameters"]
    assert amcl["alpha1"] == 0.05 and amcl["z_hit"] == 0.9 and amcl["alpha2"] == 0.2
    assert amcl["global_frame_id"] == "map" and amcl["set_initial_pose"] is False


def test_gate_intervals_count_untrusted_time_by_first_reason():
    events = [{"kind": "gate_state", "time_s": 10.0, "value": {"state": "ACTIVE", "reasons": []}},
              {"kind": "replay_state", "time_s": 11.0, "value": {}},
              {"kind": "gate_state", "time_s": 12.0,
               "value": {"state": "LOCALIZATION_UNTRUSTED", "reasons": ["MAP_CORRECTION_CHANGING_FAST"]}},
              {"kind": "gate_state", "time_s": 15.0, "value": {"state": "COMMAND_STALE", "reasons": []}},
              {"kind": "gate_state", "time_s": 18.0,
               "value": {"state": "LOCALIZATION_UNTRUSTED", "reasons": ["COVARIANCE_XY_TOO_LARGE"]}}]
    intervals = sl.gate_intervals(events, 20.0)
    assert len(intervals) == 4
    assert sl.untrusted_time(intervals, 11.0, 19.0) == {"MAP_CORRECTION_CHANGING_FAST": 3.0,
                                                        "COVARIANCE_XY_TOO_LARGE": 1.0}


def test_offline_gate_replay_flags_jumps_and_large_covariance():
    times = np.round(np.arange(0.0, 20.0, 0.1), 3)
    map_odom = np.zeros((len(times), 3))
    map_odom[times >= 5.0, 0] = 0.4                       # one 0.4 m jump at 5 s
    cov_times = np.array([0.0, 8.0])
    cov_values = np.array([[0.01, 0.01, 0.001], [0.25, 0.01, 0.001]])   # std 0.1 m, then 0.5 m
    correction, covariance, events = sl.replay_gate_offline(times, map_odom, cov_times, cov_values, 0.25, 5.0, 0.3)
    assert events == 1
    # the jump stays inside the 2 s window until 7 s, then the 3 s hold runs out at about 9.9 s
    assert not correction[times < 4.95].any() and correction[(times >= 5.0) & (times < 9.8)].all()
    assert not correction[times > 10.0].any()
    assert not covariance[times < 8.0].any() and covariance[times >= 8.0].all()
    relaxed, _, _ = sl.replay_gate_offline(times, map_odom, cov_times, cov_values, 0.5, 5.0, 0.6)
    assert not relaxed.any()


def test_compose_poses_matches_planar_composition():
    from wc_navigation.autonomy.ros_util import compose
    poses = np.array([[1.0, 2.0, 0.3], [-0.5, 0.25, -2.9]])
    transform = (0.7, -1.2, 2.5)
    expected = [compose(transform, tuple(p)) for p in poses]
    np.testing.assert_allclose(sl.compose_poses(transform, poses), expected, atol=1e-12)
