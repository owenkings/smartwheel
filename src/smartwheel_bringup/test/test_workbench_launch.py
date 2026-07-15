from pathlib import Path

import yaml


ROOT = Path(__file__).parents[2]
BRINGUP = ROOT / "smartwheel_bringup"


def test_operator_launch_is_mock_only_and_fail_closed():
    source = (BRINGUP / "launch" / "operator_workbench.launch.py").read_text(encoding="utf-8")
    assert 'mode != "mock"' in source
    assert "hardware_enabled" in source
    assert '"workbench_topic_aliases": "true"' in source
    assert '"record_bag": "false"' in source


def test_workbench_launch_declares_required_arguments():
    source = (BRINGUP / "launch" / "operator_workbench.launch.py").read_text(encoding="utf-8")
    for argument in (
        "mode",
        "rviz_config",
        "mapping_backend",
        "hardware_enabled",
        "enable_cameras",
        "enable_teleop",
        "enable_mapping_control",
        "enable_status_panel",
        "enable_map_products",
    ):
        assert f'DeclareLaunchArgument("{argument}"' in source


def test_rviz_config_has_six_plugin_types_and_four_camera_instances():
    config = yaml.safe_load(
        (BRINGUP / "rviz" / "smartwheel_operator_workbench.rviz").read_text(encoding="utf-8")
    )
    panels = config["Panels"]
    classes = [panel["Class"] for panel in panels]
    assert classes.count("SmartWheel/Camera") == 4
    for panel_class in (
        "SmartWheel/2D Occupancy Map",
        "SmartWheel/Teleop",
        "SmartWheel/Mapping Control",
        "SmartWheel/System Status",
        "SmartWheel/Map Products",
    ):
        assert classes.count(panel_class) == 1


def test_central_render_view_uses_map_without_fallback():
    config = yaml.safe_load(
        (BRINGUP / "rviz" / "smartwheel_operator_workbench.rviz").read_text(encoding="utf-8")
    )
    assert config["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"
    assert not any(panel.get("Class", "").endswith("RenderPanel") for panel in config["Panels"])
    displays = config["Visualization Manager"]["Displays"][0]["Displays"]
    global_map = next(display for display in displays if display["Name"] == "Optimized Global Map")
    assert global_map["Color Transformer"] == "RGB8"


def test_map_preview_requires_explicit_version_and_no_hardware():
    source = (BRINGUP / "launch" / "map_preview_workbench.launch.py").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("map_version")' in source
    assert '"hardware_enabled": False' in source
    assert '"preview_version": str(version)' in source
