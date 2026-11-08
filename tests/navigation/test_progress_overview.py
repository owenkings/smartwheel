"""The plain progress overview: content validation, escaping, map figures and no-overwrite output."""
import json
from io import BytesIO

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


def _figure(**changes):
    figure = {"title": "图", "map_yaml": "/m/map.yaml", "caption": "说明"}
    figure.update(changes)
    return figure


@pytest.mark.parametrize("change", [
    {"schema_version": 2},
    {"ladder": [{"name": "一", "state": "finished", "note": "a"}] * 3},
    {"ladder": [{"name": "一", "state": "done", "note": "a"}]},
    {"lines": [{"key": "A", "name": "数据", "done": [""], "doing": [], "attention": []}]},
    {"surprise": True},
    {"figures": [_figure(colour="red")]},
    {"figures": [_figure(window=[1.0, 0.0, 0.0, 1.0])]},
    {"figures": [_figure(window=[0.0, 0.0, 1.0])]},
    {"figures": [_figure(window=[0.0, 0.0, 500.0, 1.0])]},
    {"figures": [_figure(markers=[{"x": 0.0, "y": 0.0, "label": "长凳"}])]},
    {"figures": [_figure(markers=[{"x": "1", "y": 0.0, "label": "1"}])]},
    {"figures": [_figure()] * 7},
    {"map_table": {"columns": ["指标", "旧图"], "rows": [["伪障碍", "47", "3"]]}},
    {"map_table": {"columns": ["指标"], "rows": [["伪障碍"]]}},
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
    (tmp_path / "poses.csv").write_text("used_x,used_y\n0.0,0.0\n0.5,0.0\n1.0,0.2\n", encoding="utf-8")
    return tmp_path / "map.yaml", tmp_path / "poses.csv"


def _image(png):
    from PIL import Image
    with Image.open(BytesIO(png)) as image:
        return np.asarray(image.convert("RGB"))


def test_whole_map_figure_is_cropped_with_a_larger_copy_and_hashed_inputs(tmp_path):
    map_yaml, poses = _map(tmp_path)
    result = po.render_figure({"map_yaml": str(map_yaml), "path_csv": str(poses)}, page_width=400)
    assert result["page"].startswith(b"\x89PNG\r\n\x1a\n") and len(result["inputs"]) == 3
    page_w, page_h = result["page_size"]
    assert page_w <= 400 and page_h < page_w  # cropped to the observed area
    assert result["full_size"][0] >= page_w and _image(result["full"]).shape[1] == result["full_size"][0]


def test_a_window_shows_the_same_place_and_pads_outside_the_map(tmp_path):
    grid = po._load_grid(_map(tmp_path)[0])
    crop, top, left = po._crop(grid, [-0.5, -0.5, 0.5, 0.5])  # cell edges: columns and rows 10..29
    assert (top, left) == (10, 10) and np.array_equal(crop, grid["pixels"][10:30, 10:30])
    crop, top, left = po._crop(grid, [-2.0, -1.0, 0.0, 1.0])  # the left half lies outside the map
    assert (top, left) == (0, -20) and crop.shape == (40, 40)
    assert (crop[:, :20] == 205).all() and np.array_equal(crop[:, 20:], grid["pixels"][:, :20])


def test_window_figure_draws_the_path_markers_and_a_scale_bar_in_place(tmp_path):
    map_yaml, poses = _map(tmp_path)
    spec = {"map_yaml": str(map_yaml), "path_csv": str(poses), "window": [-0.5, -0.5, 0.5, 0.5],
            "markers": [{"x": -0.25, "y": 0.25, "label": "1"}]}
    result = po.render_figure(spec)
    assert result["page_size"] == (1000, 1000)  # 20 x 20 cells at 50 px per cell
    image = _image(result["page"])
    # the path runs along y = 0 m: image row (40 - 20 - 10) * 50 = 500; x = 0.4 m is column (28 - 10) * 50 = 900
    assert tuple(image[500, 900]) == po.PATH_COLOR
    assert (image == po.MARKER_COLOR).all(axis=2).any()
    corner = image[850:, :300]  # the scale bar sits bottom left on a white box
    assert (corner == po.MAP_COLORS[0]).all(axis=2).any()


def test_build_writes_figures_and_a_table_and_refuses_to_overwrite(tmp_path):
    map_yaml, poses = _map(tmp_path)
    figures = [_figure(title="全图", map_yaml=str(map_yaml), path_csv=str(poses)),
               _figure(title="放大", map_yaml=str(map_yaml), window=[-0.5, -0.5, 0.5, 0.5],
                       markers=[{"x": 0.0, "y": 0.0, "label": "2"}])]
    table = {"columns": ["指标", "旧图", "新图"], "rows": [["伪障碍格", "47", "<3>"]], "note": "来自核对报告"}
    content_path = tmp_path / "content.json"
    content_path.write_text(json.dumps(_content(figures=figures, map_table=table), ensure_ascii=False),
                            encoding="utf-8")
    output = tmp_path / "out"
    result = po.build(content_path, output)
    assert result["status"] == "OVERVIEW_WRITTEN" and result["figures"] == 2
    assert result["missing_sources"] == ["/nonexistent/board.md"]
    page = (output / "index.html").read_text(encoding="utf-8")
    assert '<figure class="fig wide">' in page and '<figure class="fig">' in page  # whole map wide, close-up half
    assert 'src="fig2.png"' in page and 'href="fig2_full.png"' in page and "看原尺寸" in page
    assert '<th scope="col">旧图</th>' in page and "&lt;3&gt;" in page and "来自核对报告" in page
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["hardware_started"] is False and len(manifest["map_inputs_sha256"]) == 3
    assert sorted(manifest["figures_sha256"]) == ["fig1.png", "fig1_full.png", "fig2.png", "fig2_full.png"]
    for name, digest in manifest["figures_sha256"].items():
        assert po._sha256_file(output / name) == digest
    with pytest.raises(FileExistsError):
        po.build(content_path, output)


def test_the_original_single_map_key_still_builds(tmp_path):
    map_yaml, poses = _map(tmp_path)
    content_path = tmp_path / "content.json"
    content_path.write_text(json.dumps(_content(map={"map_yaml": str(map_yaml), "path_csv": str(poses),
                                                     "caption": "地图"}), ensure_ascii=False), encoding="utf-8")
    output = tmp_path / "out"
    po.build(content_path, output)
    page = (output / "index.html").read_text(encoding="utf-8")
    assert "在用的走廊地图" in page and 'src="fig1.png"' in page and (output / "fig1_full.png").is_file()


def test_landing_page_points_to_the_newest_overview_and_keeps_old_snapshots(tmp_path):
    content_path = tmp_path / "content.json"
    content_path.write_text(json.dumps(_content(), ensure_ascii=False), encoding="utf-8")
    root = tmp_path / "dashboards"
    po.build(content_path, root / "20261109_overview")
    newer = tmp_path / "newer.json"
    newer.write_text(json.dumps(_content(updated="2026-11-09 08:00"), ensure_ascii=False), encoding="utf-8")
    po.build(newer, root / "20261108_overview")  # name sorts first, but its content is newer
    (root / "20261107_office_outline").mkdir()
    (root / "20261107_office_outline" / "index.html").write_text("old", encoding="utf-8")
    result = po.write_landing(root)
    page = (root / "index.html").read_text(encoding="utf-8")
    assert result == {"landing": str(root / "index.html"), "overviews": 2, "other_snapshots": 1}
    assert page.index('href="20261108_overview/"') < page.index('href="20261109_overview/"')  # newest content first
    assert 'href="20261107_office_outline/"' in page and "打开最新的进展总览" in page
    assert (root / "20261107_office_outline" / "index.html").read_text(encoding="utf-8") == "old"
