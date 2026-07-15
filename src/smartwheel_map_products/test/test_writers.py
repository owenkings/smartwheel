import json

import numpy as np
import yaml

from smartwheel_map_products.occupancy import raycast_occupancy
from smartwheel_map_products.writers import export_map_bundle


def test_map_bundle_contains_nonempty_products(tmp_path):
    points = np.array([[1.0, 0.0, 0.5], [0.0, 1.0, 0.7], [1.0, 1.0, 1.2]])
    origins = np.zeros_like(points)
    grid = raycast_occupancy(points, origins, resolution=0.1)
    profile = tmp_path / "profile.yaml"
    profile.write_text("profile:\n  mode: mock\n", encoding="utf-8")
    output = tmp_path / "map"
    export_map_bundle(
        output,
        points,
        grid,
        [(1.0, 0.0, 0.0, 0.0), (2.0, 0.1, 0.0, 0.0)],
        np.full((3, 3), 127, dtype=np.uint8),
        str(profile),
        {"backend": "test"},
        "/tmp/smartwheel_experiment/rosbag",
        {"point_count": 3},
    )
    expected = (
        "map_geometry.pcd",
        "map_geometry.ply",
        "map_colored.ply",
        "map_2d.pgm",
        "map_2d.png",
        "map_2d.yaml",
        "trajectory.tum",
        "poses.csv",
        "hardware_profile_used.yaml",
        "algorithm_profile_used.yaml",
        "quality_report.json",
        "quality_report.md",
        "manifest.json",
        "bag_path.txt",
    )
    for name in expected:
        assert (output / name).stat().st_size > 0
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["complete"] is True
    assert manifest["externally_managed_files"] == []
    assert (output / "bag_path.txt").read_text(encoding="utf-8").strip() == (
        "/tmp/smartwheel_experiment/rosbag"
    )
    bag_manifest = next(item for item in manifest["files"] if item["path"] == "bag_path.txt")
    assert bag_manifest["bytes"] == (output / "bag_path.txt").stat().st_size
    map_yaml = yaml.safe_load((output / "map_2d.yaml").read_text(encoding="utf-8"))
    assert map_yaml["resolution"] == 0.1
