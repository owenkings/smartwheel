"""The plain progress overview: content validation, escaping, map figure and no-overwrite output."""
import json

import numpy as np
import pytest

from wc_navigation import progress_overview as po


def _content(**changes):
    content = {
        "schema_version": 1, "title": "进展", "updated": "2026-11-08", "status_line": "状态", "safety_line": "没有控制轮椅",
        "ladder": [{"name": "一", "state": "done", "note": "a"}, {"name": "二", "state": "waiting", "note": "b"},
                   {"name": "三", "state": "todo", "note": "c"}],
        "headline": [{"value": "1", "label": "数", "note": "说明"}],
        "lines": [{"key": "A", "name": "数据", "done": ["做完"], "doing": ["在做"], "attention": []}],
        "needs_user": ["去现场"], "sources": [{"label": "看板", "path": "/nonexistent/board.md"}],
    }
    content.update(changes)
    return content


def test_valid_content_renders_every_section_and_escapes_text():
    content = po.validate_content(_content(title="<script>x</script>"))
    page = po.render_html(content)
    assert "<script>x" not in page and "&lt;script&gt;" in page
    for text in ("走到哪一步了", "关键数字", "各条工作线", "需要你做的", "软件就绪，等现场", "✓"):
        assert text in page
    assert "<script" not in page.lower().replace("&lt;script", "")


@pytest.mark.parametrize("change", [
    {"schema_version": 2},
    {"ladder": [{"name": "一", "state": "finished", "note": "a"}] * 3},
    {"ladder": [{"name": "一", "state": "done", "note": "a"}]},
    {"lines": [{"key": "A", "name": "数据", "done": [""], "doing": [], "attention": []}]},
    {"surprise": True},
])
def test_invalid_content_is_rejected(change):
    with pytest.raises(po.ContentError):
        po.validate_content(_content(**change))


def _map(tmp_path):
    from PIL import Image
    pixels = np.full((40, 60), 205, dtype=np.uint8)
    pixels[10:30, 10:50] = 254
    pixels[10, 10:50] = 0
    Image.fromarray(pixels).save(tmp_path / "map.pgm")
    (tmp_path / "map.yaml").write_text("image: map.pgm\nresolution: 0.05\norigin: [-1.0, -1.0, 0.0]\n", encoding="utf-8")
    (tmp_path / "poses.csv").write_text("used_x,used_y\n0.0,0.0\n0.5,0.2\n1.0,0.2\n", encoding="utf-8")
    return tmp_path / "map.yaml", tmp_path / "poses.csv"


def test_map_figure_is_a_cropped_png_with_hashed_inputs(tmp_path):
    map_yaml, poses = _map(tmp_path)
    png, inputs = po.render_map_png(map_yaml, poses, max_width=400)
    assert png.startswith(b"\x89PNG\r\n\x1a\n") and len(inputs) == 3
    from io import BytesIO
    from PIL import Image
    with Image.open(BytesIO(png)) as image:
        assert image.width <= 400 and image.height < image.width  # cropped to the observed area


def test_build_writes_a_new_directory_and_refuses_to_overwrite(tmp_path):
    map_yaml, poses = _map(tmp_path)
    content_path = tmp_path / "content.json"
    content_path.write_text(json.dumps(_content(map={"map_yaml": str(map_yaml), "path_csv": str(poses),
                                                     "caption": "地图"}), ensure_ascii=False), encoding="utf-8")
    output = tmp_path / "out"
    result = po.build(content_path, output)
    assert result["status"] == "OVERVIEW_WRITTEN" and result["missing_sources"] == ["/nonexistent/board.md"]
    page = (output / "index.html").read_text(encoding="utf-8")
    assert "data:image/png;base64," in page and "在用的走廊地图" in page
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["hardware_started"] is False and len(manifest["map_inputs_sha256"]) == 3
    with pytest.raises(FileExistsError):
        po.build(content_path, output)
