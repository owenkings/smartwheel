from pathlib import Path
import importlib.util

import pytest


def _load_module():
    path = Path(__file__).parents[2] / "wheelchair_3d_mapping" / "launch" / "rtabmap_3d_mapping.launch.py"
    spec = importlib.util.spec_from_file_location("rtabmap_3d_mapping_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _valid(**overrides):
    values = {
        "config_path": "/tmp/rtabmap_params.yaml",
        "points_topic": "/points_merged",
        "odom_topic": "/rtabmap/odom",
        "odom_mode": "icp",
        "frame_id": "base_link",
        "queue_size": 10,
        "bringup_sensors": False,
        "subscribe_scan_cloud": True,
        "subscribe_rgb": False,
        "rgb_topic": "/camera/front/image_raw",
        "camera_info_topic": "/camera/front/camera_info",
        "database_path": "/tmp/smartwheel-rtabmap.db",
    }
    values.update(overrides)
    return values


def test_valid_icp_contract_is_accepted():
    module = _load_module()
    module._validate_launch_contract(**_valid())


@pytest.mark.parametrize(
    "overrides, message",
    [
        ({"odom_mode": "bogus"}, "INVALID_ODOM_MODE"),
        ({"queue_size": 0}, "queue_size"),
        ({"points_topic": "points"}, "points_topic"),
        ({"odom_topic": "odom"}, "odom_topic"),
        ({"frame_id": "/base_link"}, "frame_id"),
        ({"config_path": ""}, "config"),
        ({"odom_mode": "external"}, "INVALID_EXTERNAL_ODOM"),
        ({"subscribe_rgb": True, "rgb_topic": "camera/image"}, "rgb_topic"),
        ({"subscribe_rgb": True, "camera_info_topic": "camera/info"}, "camera_info_topic"),
        ({"database_path": ""}, "database_path"),
        ({"subscribe_scan_cloud": False}, "INVALID_INPUT"),
    ],
)
def test_invalid_contract_fails_before_ros_actions(overrides, message):
    module = _load_module()
    with pytest.raises(RuntimeError, match=message):
        module._validate_launch_contract(**_valid(**overrides))


def test_external_odom_contract_accepts_explicit_topic():
    module = _load_module()
    module._validate_launch_contract(
        **_valid(odom_mode="external", odom_topic="/lio/odom")
    )


def test_sensor_bringup_can_coexist_with_explicit_external_odom_owner():
    module = _load_module()
    module._validate_launch_contract(
        **_valid(
            bringup_sensors=True,
            odom_mode="external",
            odom_topic="/odometry/filtered",
        )
    )


def test_launch_description_keeps_backend_and_viz_ownership_explicit():
    module = _load_module()
    source = Path(module.__file__).read_text(encoding="utf-8")
    assert '"qos_scan": 2' in source
    assert '"qos_odom": 1' in source
    assert 'parameters=[*rtab_params, {"publish_tf": False}]' in source
