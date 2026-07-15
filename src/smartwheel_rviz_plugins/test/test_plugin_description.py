from pathlib import Path
from xml.etree import ElementTree


def test_all_panels_are_registered_once():
    root = ElementTree.parse(Path(__file__).parents[1] / "plugin_description.xml").getroot()
    names = [item.attrib["name"] for item in root.findall("class")]
    assert names == [
        "SmartWheel/2D Occupancy Map",
        "SmartWheel/Camera",
        "SmartWheel/Teleop",
        "SmartWheel/Mapping Control",
        "SmartWheel/System Status",
        "SmartWheel/Map Products",
    ]
    assert len({item.attrib["type"] for item in root.findall("class")}) == 6
    assert all(item.attrib["base_class_type"] == "rviz_common::Panel" for item in root.findall("class"))


def test_camera_plugin_is_reusable_not_four_hard_coded_classes():
    text = (Path(__file__).parents[1] / "plugin_description.xml").read_text(encoding="utf-8")
    assert text.count("smartwheel_rviz_plugins::CameraPanel") == 1
