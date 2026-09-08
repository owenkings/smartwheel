from pathlib import Path


ROOT = Path(__file__).parents[2]
LAUNCH_PATH = ROOT / "wheelchair_bringup" / "launch" / "manual_teleop.launch.py"


def test_manual_teleop_rejects_invalid_radar_instead_of_falling_back_to_right():
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'if radar not in ("left", "right", "both"):' in source
    assert "INVALID_RADAR_SELECTION" in source
    assert 'radar = "right"' not in source


def test_manual_teleop_accepts_only_explicit_base_tf_modes():
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'not in ("auto", "true", "false")' in source
    assert "INVALID_BASE_PUBLISH_TF" in source
    assert 'if base_publish_tf_mode == "auto":' in source
    assert "base_publish_tf = base_publish_tf_mode" in source
