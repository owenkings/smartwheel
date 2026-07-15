import json
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from smartwheel_workbench.products import load_map_version, list_map_versions, read_ascii_pcd


def _write_version(root: Path) -> Path:
    version = root / "map_20260715_120000"
    version.mkdir()
    (version / "map_geometry.pcd").write_text(
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n"
        "1.0 2.0 0.0\n3.0 4.0 0.5\n",
        encoding="ascii",
    )
    image = np.array([[0, 205], [254, 254]], dtype=np.uint8)
    assert cv2.imwrite(str(version / "map_2d.pgm"), image)
    assert cv2.imwrite(str(version / "map_2d.png"), image)
    (version / "map_2d.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "map_2d.pgm",
                "resolution": 0.05,
                "origin": [-1.0, -2.0, 0.25],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
        ),
        encoding="utf-8",
    )
    (version / "trajectory.tum").write_text(
        "1.0 0 0 0 0 0 0 1\n2.0 1 0 0 0 0 0.70710678 0.70710678\n",
        encoding="ascii",
    )
    (version / "quality_report.json").write_text(
        json.dumps({"point_count": 2, "hardware_validated": False}), encoding="utf-8"
    )
    for name in ("map_geometry.ply", "quality_report.md"):
        (version / name).write_text("test\n", encoding="utf-8")
    (version / "manifest.json").write_text('{"complete": true}\n', encoding="utf-8")
    return version


def _add_colored_ply(version: Path):
    (version / "map_colored.ply").write_text(
        "ply\nformat ascii 1.0\nelement vertex 2\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
        "1.0 2.0 0.0 255 10 20\n3.0 4.0 0.5 30 40 50\n",
        encoding="ascii",
    )


def test_load_map_version_preserves_geometry_and_unknown_cells(tmp_path):
    version = _write_version(tmp_path)
    loaded = load_map_version(version)
    np.testing.assert_allclose(loaded.points, [[1.0, 2.0, 0.0], [3.0, 4.0, 0.5]])
    assert loaded.cells.shape == (2, 2)
    assert {int(value) for value in loaded.cells.ravel()} == {-1, 0, 100}
    assert loaded.origin_yaw == pytest.approx(0.25)
    assert len(loaded.trajectory) == 2


def test_list_only_complete_versions(tmp_path):
    version = _write_version(tmp_path)
    (tmp_path / "incomplete").mkdir()
    assert list_map_versions(tmp_path) == [version]


def test_colored_ply_is_loaded_only_when_geometry_matches(tmp_path):
    version = _write_version(tmp_path)
    _add_colored_ply(version)
    loaded = load_map_version(version)
    np.testing.assert_array_equal(loaded.colors, [[255, 10, 20], [30, 40, 50]])

    text = (version / "map_colored.ply").read_text(encoding="ascii").replace(
        "3.0 4.0 0.5", "30.0 4.0 0.5"
    )
    (version / "map_colored.ply").write_text(text, encoding="ascii")
    with pytest.raises(ValueError, match="geometry does not match"):
        load_map_version(version)


def test_pcd_count_mismatch_is_rejected(tmp_path):
    path = tmp_path / "bad.pcd"
    path.write_text(
        "FIELDS x y z\nPOINTS 2\nDATA ascii\n1 2 3\n", encoding="ascii"
    )
    with pytest.raises(ValueError, match="point count mismatch"):
        read_ascii_pcd(path)
