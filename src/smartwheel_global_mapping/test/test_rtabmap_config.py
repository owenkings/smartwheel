from pathlib import Path

import yaml


CONFIG = Path(__file__).parents[1] / "config" / "rtabmap_params.yaml"


def test_scan_cloud_profile_uses_icp_proximity_without_visual_hypotheses():
    text = CONFIG.read_text(encoding="utf-8")
    params = yaml.safe_load(text)["rtabmap"]["ros__parameters"]

    assert params["subscribe_scan_cloud"] is True
    assert params["subscribe_rgb"] is False
    assert params["subscribe_depth"] is False
    assert params["Reg/Strategy"] == "1"
    assert params["Rtabmap/LoopThr"] == "1.0"
    assert params["RGBD/AggressiveLoopThr"] == "1.0"
    assert params["RGBD/ProximityBySpace"] == "true"
    assert params["RGBD/ProximityOdomGuess"] == "true"
