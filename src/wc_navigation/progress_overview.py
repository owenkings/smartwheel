"""Plain one-page progress overview: what has been done, what is next.

The detailed dashboard (``dashboard.py``) shows every archived run. This page
is the front door for the user: the validation ladder, a few headline
numbers, one card per workstream, the corridor maps and what needs the user.
Its text comes from a small, human-curated JSON file. The map figures are
drawn from the map files themselves: the whole observed map or a window given
in metres, the recorded path, numbered markers and a scale bar. Each figure is
written twice, page size and full size for zooming, so maps can be compared
cell by cell. A snapshot is a new directory under ``reports`` holding
``index.html`` (no scripts, no network), the figures, a copy of the content and
a manifest; an existing snapshot is never overwritten. Nothing here starts ROS
or touches the vehicle.
"""
from __future__ import annotations

import argparse
import functools
import hashlib
import html
import json
import math
import re
from io import BytesIO
from pathlib import Path

STATES = {  # state -> (label, mark); the label carries the meaning, colour only repeats it
    "done": ("已完成", "✓"),
    "doing": ("进行中", "▶"),
    "waiting": ("软件就绪，等现场", "◐"),
    "todo": ("未开始", "○"),
}
MAX_TEXT = 400
MAX_CONTENT_BYTES = 256 * 1024
MAP_COLORS = {254: (255, 255, 255), 0: (51, 65, 85), 205: (226, 232, 240)}  # free, occupied, unknown
PATH_COLOR, START_COLOR, MARKER_COLOR = (37, 99, 235), (22, 163, 74), (190, 18, 60)
FIGURE_KEYS = ("title", "map_yaml", "path_csv", "window", "markers", "caption")
MAX_FIGURES, MAX_MARKERS, MAX_WINDOW_M, MAX_CROP_CELLS = 6, 20, 200.0, 4000
MARKER_LABEL = re.compile(r"[0-9A-Za-z]{1,3}")  # drawn into the image, so no fonts beyond ASCII are needed
PAGE_WIDTH_PX, FULL_MAX_PX, FULL_MAX_SCALE = 1000, 4000, 8
LEGEND = ("图例：白色是可通行区域，深灰是墙和障碍，浅灰是没看到的地方；蓝线是建图时推车走的路线，绿色实心圆是路线起点；"
          "红圈加编号是说明里提到的位置；左下角是比例尺。点图打开原尺寸，可以放大看每一格。")


class ContentError(ValueError):
    """The curated overview content is missing a field or has the wrong shape."""


def _text(value, where, *, allow_empty=False):
    if not isinstance(value, str) or (not value.strip() and not allow_empty) or len(value) > MAX_TEXT:
        raise ContentError(f"{where}: expected non-empty text of at most {MAX_TEXT} characters")
    return value.strip()


def _texts(value, where, *, minimum=0, maximum=12):
    if not isinstance(value, list) or not minimum <= len(value) <= maximum:
        raise ContentError(f"{where}: expected a list of {minimum}-{maximum} items")
    return [_text(item, f"{where}[{i}]") for i, item in enumerate(value)]


def _items(value, where, keys, *, minimum, maximum):
    if not isinstance(value, list) or not minimum <= len(value) <= maximum:
        raise ContentError(f"{where}: expected a list of {minimum}-{maximum} items")
    out = []
    for i, item in enumerate(value):
        if not isinstance(item, dict) or set(item) - set(keys):
            raise ContentError(f"{where}[{i}]: expected only the keys {', '.join(keys)}")
        out.append(item)
    return out


def _number(value, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ContentError(f"{where}: expected a finite number")
    return float(value)


def _figure(spec, where):
    """One map figure: map file, optional path, optional window in metres, numbered markers, caption."""
    window = spec.get("window")
    if window is not None:
        if not isinstance(window, list) or len(window) != 4:
            raise ContentError(f"{where}.window: expected [x_min, y_min, x_max, y_max] in metres")
        window = [_number(v, f"{where}.window[{i}]") for i, v in enumerate(window)]
        if not (window[0] < window[2] and window[1] < window[3]) or \
                max(window[2] - window[0], window[3] - window[1]) > MAX_WINDOW_M:
            raise ContentError(f"{where}.window: needs x_min < x_max and y_min < y_max, "
                               f"at most {MAX_WINDOW_M:g} m across")
    markers = []
    for i, marker in enumerate(_items(spec.get("markers", []), f"{where}.markers", ("x", "y", "label"),
                                      minimum=0, maximum=MAX_MARKERS)):
        label = marker.get("label")
        if not isinstance(label, str) or not MARKER_LABEL.fullmatch(label):
            raise ContentError(f"{where}.markers[{i}].label: 1-3 ASCII letters or digits (it is drawn in the image)")
        markers.append({"x": _number(marker.get("x"), f"{where}.markers[{i}].x"),
                        "y": _number(marker.get("y"), f"{where}.markers[{i}].y"), "label": label})
    return {"title": _text(spec.get("title"), f"{where}.title"),
            "map_yaml": _text(spec.get("map_yaml"), f"{where}.map_yaml"),
            "path_csv": _text(spec["path_csv"], f"{where}.path_csv") if spec.get("path_csv") else None,
            "window": window, "markers": markers, "caption": _text(spec.get("caption"), f"{where}.caption")}


def _table(table):
    """A small comparison table (for example the same quality numbers for an old and a new map)."""
    if table is None:
        return None
    if not isinstance(table, dict) or set(table) - {"columns", "rows", "note"}:
        raise ContentError("map_table: expected columns, rows and an optional note")
    columns = _texts(table.get("columns"), "map_table.columns", minimum=2, maximum=6)
    rows = table.get("rows")
    if not isinstance(rows, list) or not 1 <= len(rows) <= 12:
        raise ContentError("map_table.rows: expected 1-12 rows")
    return {"columns": columns,
            "rows": [_texts(row, f"map_table.rows[{i}]", minimum=len(columns), maximum=len(columns))
                     for i, row in enumerate(rows)],
            "note": _text(table["note"], "map_table.note") if table.get("note") else None}


def validate_content(content):
    """Return a normalised copy; raise ContentError on anything unexpected."""
    if not isinstance(content, dict) or content.get("schema_version") != 1:
        raise ContentError("schema_version must be 1")
    allowed = {"schema_version", "title", "updated", "status_line", "safety_line", "ladder", "headline",
               "lines", "needs_user", "map", "figures", "map_table", "sources"}
    if set(content) - allowed:
        raise ContentError("unknown top-level keys: " + ", ".join(sorted(set(content) - allowed)))
    out = {key: _text(content.get(key), key) for key in ("title", "updated", "status_line", "safety_line")}
    out["ladder"] = []
    for i, step in enumerate(_items(content.get("ladder"), "ladder", ("name", "state", "note"), minimum=3, maximum=8)):
        if step.get("state") not in STATES:
            raise ContentError(f"ladder[{i}].state must be one of {', '.join(STATES)}")
        out["ladder"].append({"name": _text(step.get("name"), f"ladder[{i}].name"), "state": step["state"],
                              "note": _text(step.get("note"), f"ladder[{i}].note")})
    out["headline"] = [{k: _text(t.get(k), f"headline[{i}].{k}") for k in ("value", "label", "note")}
                       for i, t in enumerate(_items(content.get("headline", []), "headline",
                                                    ("value", "label", "note"), minimum=0, maximum=6))]
    out["lines"] = []
    for i, line in enumerate(_items(content.get("lines"), "lines", ("key", "name", "done", "doing", "attention"),
                                    minimum=1, maximum=6)):
        out["lines"].append({"key": _text(line.get("key"), f"lines[{i}].key"),
                             "name": _text(line.get("name"), f"lines[{i}].name"),
                             "done": _texts(line.get("done", []), f"lines[{i}].done"),
                             "doing": _texts(line.get("doing", []), f"lines[{i}].doing"),
                             "attention": _texts(line.get("attention", []), f"lines[{i}].attention")})
    out["needs_user"] = _texts(content.get("needs_user", []), "needs_user")
    out["figures"] = []
    map_spec = content.get("map")
    if map_spec is not None:  # the original single figure; older content files still build
        if not isinstance(map_spec, dict) or set(map_spec) - {"map_yaml", "path_csv", "caption"}:
            raise ContentError("map: expected map_yaml, optional path_csv, caption")
        out["figures"].append(_figure(dict(map_spec, title="在用的走廊地图"), "map"))
    for i, spec in enumerate(_items(content.get("figures", []), "figures", FIGURE_KEYS, minimum=0,
                                    maximum=MAX_FIGURES)):
        out["figures"].append(_figure(spec, f"figures[{i}]"))
    if len(out["figures"]) > MAX_FIGURES:
        raise ContentError(f"at most {MAX_FIGURES} map figures in all")
    out["map_table"] = _table(content.get("map_table"))
    out["sources"] = [{"label": _text(s.get("label"), f"sources[{i}].label"),
                       "path": _text(s.get("path"), f"sources[{i}].path")}
                      for i, s in enumerate(_items(content.get("sources", []), "sources", ("label", "path"),
                                                   minimum=0, maximum=20))]
    return out


def _sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_grid(map_yaml):
    """A ROS map_server map: the pixels, resolution, origin and the resolved file paths."""
    import numpy as np
    import yaml
    from PIL import Image

    map_yaml = Path(map_yaml)
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    pgm = (map_yaml.parent / meta["image"]).resolve()
    with Image.open(pgm) as image:
        pixels = np.asarray(image.convert("L"))
    return {"yaml": map_yaml.resolve(), "pgm": pgm, "pixels": pixels, "resolution": float(meta["resolution"]),
            "origin": (float(meta["origin"][0]), float(meta["origin"][1]))}


def _crop(grid, window=None, padding_px=12):
    """Cells to draw and the map-image row and column of their top-left corner.

    Without a window: everything observed plus a margin. With a window in
    metres: exactly that area, so two maps with different grid origins show
    the same place; cells outside the map are drawn as unknown.
    """
    import numpy as np

    pixels, resolution, (origin_x, origin_y) = grid["pixels"], grid["resolution"], grid["origin"]
    height, width = pixels.shape
    if window is None:
        known = np.argwhere(pixels != 205)
        if not len(known):
            raise ContentError("the map has no observed cells")
        top, left = (int(v) for v in np.maximum(known.min(axis=0) - padding_px, 0))
        bottom, right = (int(v) for v in np.minimum(known.max(axis=0) + padding_px + 1, (height, width)))
        return pixels[top:bottom, left:right], top, left
    x_min, y_min, x_max, y_max = window
    eps = 1e-6  # a window edge lying on a cell edge must not pull in the neighbouring cell
    left = math.floor((x_min - origin_x) / resolution + eps)
    right = math.ceil((x_max - origin_x) / resolution - eps)
    top = height - math.ceil((y_max - origin_y) / resolution - eps)
    bottom = height - math.floor((y_min - origin_y) / resolution + eps)
    if max(bottom - top, right - left) > MAX_CROP_CELLS:
        raise ContentError("the window is too large for this map's resolution")
    crop = np.full((bottom - top, right - left), 205, dtype=np.uint8)
    r0, r1, c0, c1 = max(top, 0), min(bottom, height), max(left, 0), min(right, width)
    if r0 < r1 and c0 < c1:
        crop[r0 - top:r1 - top, c0 - left:c1 - left] = pixels[r0:r1, c0:c1]
    return crop, top, left


@functools.lru_cache(maxsize=8)
def _font(size):
    from PIL import ImageFont
    for name in ("DejaVuSans-Bold.ttf", "DejaVuSans.ttf"):  # found in the system font folders by Pillow
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _text_size(font, text):
    """Width, height and the offset of the ink from the drawing origin."""
    try:
        left, top, right, bottom = font.getbbox(text)
        return right - left, bottom - top, left, top
    except AttributeError:  # bitmap fallback font on Pillow < 9.2
        width, height = font.getsize(text)
        return width, height, 0, 0


def _label(draw, x, y, text, font, color):
    """Text in a white box whose top-left corner is (x, y)."""
    x, y = round(x), round(y)
    width, height, dx, dy = _text_size(font, text)
    pad = max(2, height // 4)
    draw.rectangle((x, y, x + width + 2 * pad, y + height + 2 * pad), fill=(255, 255, 255), outline=color)
    draw.text((x + pad - dx, y + pad - dy), text, fill=color, font=font)


def _scale_bar(draw, figure, metres_per_px, font):
    if figure.width < 200:
        return
    fitting = [m for m in (0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100) if m / metres_per_px <= 0.25 * figure.width]
    if not fitting:
        return
    length = fitting[-1]
    bar = length / metres_per_px
    text = f"{length:g} m"
    width, height, dx, dy = _text_size(font, text)
    pad = max(4, height // 3)
    thickness = max(3, height // 4)
    x0, y_bar = 3 * pad, figure.height - 3 * pad
    draw.rectangle((x0 - pad, y_bar - thickness - 2 * pad - height, x0 + max(bar, width) + pad, y_bar + pad),
                   fill=(255, 255, 255), outline=MAP_COLORS[0])
    draw.rectangle((x0, y_bar - thickness, x0 + bar, y_bar), fill=MAP_COLORS[0])
    draw.text((x0 - dx, y_bar - thickness - pad - height - dy), text, fill=MAP_COLORS[0], font=font)


def _draw(crop, grid, top, left, scale, path_points, markers):
    import numpy as np
    from PIL import Image, ImageDraw

    rgb = np.empty(crop.shape + (3,), dtype=np.uint8)
    rgb[:] = MAP_COLORS[205]
    rgb[crop >= 250] = MAP_COLORS[254]
    walls = crop <= 50
    if scale <= 2:  # display only: one-cell walls vanish when a cell is a pixel or two
        thick = walls.copy()
        thick[1:, :] |= walls[:-1, :]
        thick[:-1, :] |= walls[1:, :]
        thick[:, 1:] |= walls[:, :-1]
        thick[:, :-1] |= walls[:, 1:]
        walls = thick
    rgb[walls] = MAP_COLORS[0]
    figure = Image.fromarray(rgb).resize((crop.shape[1] * scale, crop.shape[0] * scale), Image.NEAREST)
    draw = ImageDraw.Draw(figure)
    resolution, (origin_x, origin_y) = grid["resolution"], grid["origin"]
    height = grid["pixels"].shape[0]

    def pixel(x, y):  # metres -> image pixels; cell edges fall on pixel edges
        return ((x - origin_x) / resolution - left) * scale, (height - (y - origin_y) / resolution - top) * scale

    if path_points:
        points = [pixel(x, y) for x, y in path_points]
        draw.line(points, fill=PATH_COLOR, width=max(2, scale), joint="curve")
        sx, sy = points[0]
        radius = max(7, round(0.25 / resolution * scale))
        draw.ellipse((sx - radius, sy - radius, sx + radius, sy + radius), fill=START_COLOR,
                     outline=(255, 255, 255), width=2)
    font = _font(max(13, min(40, round(3.5 * scale))))
    radius, ring = max(8, round(0.15 / resolution * scale)), max(2, scale // 2)
    centres = [pixel(marker["x"], marker["y"]) for marker in markers]
    for mx, my in centres:
        draw.ellipse((mx - radius - 2, my - radius - 2, mx + radius + 2, my + radius + 2), outline=(255, 255, 255),
                     width=ring + 2)
        draw.ellipse((mx - radius, my - radius, mx + radius, my + radius), outline=MARKER_COLOR, width=ring)
    for marker, (mx, my) in zip(markers, centres):  # labels last, so a neighbouring ring never hides one
        _, label_height, _, _ = _text_size(font, marker["label"])
        _label(draw, mx + 0.7 * radius + 2, my - 0.7 * radius - label_height - 6, marker["label"], font, MARKER_COLOR)
    _scale_bar(draw, figure, resolution / scale, font)
    return figure


def render_figure(spec, *, page_width=PAGE_WIDTH_PX, full_max_px=FULL_MAX_PX):
    """One map figure as page-size and full-size PNG bytes, their pixel sizes and the SHA256 of each input."""
    import csv
    from PIL import Image

    grid = _load_grid(spec["map_yaml"])
    crop, top, left = _crop(grid, spec.get("window"))
    inputs = {str(grid["yaml"]): _sha256_file(grid["yaml"]), str(grid["pgm"]): _sha256_file(grid["pgm"])}
    path_points = []
    if spec.get("path_csv"):
        with Path(spec["path_csv"]).open(encoding="utf-8") as stream:
            path_points = [(float(row["used_x"]), float(row["used_y"])) for row in csv.DictReader(stream)]
        inputs[str(Path(spec["path_csv"]).resolve())] = _sha256_file(spec["path_csv"])
    rows, columns = crop.shape
    page_scale = max(1, page_width // columns)
    full_scale = max(page_scale, min(FULL_MAX_SCALE, full_max_px // max(rows, columns)))
    out = {"inputs": inputs}
    for name, scale in (("page", page_scale), ("full", full_scale)):
        figure = _draw(crop, grid, top, left, scale, path_points, spec.get("markers", []))
        if name == "page" and figure.width > page_width:
            figure = figure.resize((page_width, max(1, round(figure.height * page_width / figure.width))),
                                   Image.LANCZOS)
        buffer = BytesIO()
        figure.save(buffer, format="PNG", optimize=True)
        out[name], out[name + "_size"] = buffer.getvalue(), figure.size
    return out


CSS = """
:root{--ink:#0f172a;--muted:#475569;--line:#e2e8f0;--bg:#f8fafc;--done:#15803d;--doing:#1d4ed8;--waiting:#b45309;--todo:#64748b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.65 "PingFang SC","Microsoft YaHei","Noto Sans CJK SC","Noto Sans SC",system-ui,sans-serif}
.page{max-width:1080px;margin:0 auto;padding:28px 20px 48px}
header{margin-bottom:28px}.eyebrow{margin:0;color:var(--muted);font-size:14px}
h1{margin:4px 0 10px;font-size:30px;line-height:1.3}h2{margin:0 0 14px;font-size:21px}
h3{margin:0 0 10px;font-size:18px}h4{margin:12px 0 4px;font-size:14px;color:var(--muted);font-weight:600}
.status{margin:0 0 8px;font-size:18px}.safety{margin:0;padding:10px 14px;border-left:4px solid var(--waiting);background:#fff7ed;border-radius:6px}
section{margin:0 0 30px}
.ladder{list-style:none;margin:0;padding:0;display:grid;grid-template-columns:repeat(var(--steps),minmax(0,1fr));gap:10px}
.step{background:#fff;border:1px solid var(--line);border-top:5px solid var(--todo);border-radius:10px;padding:12px}
.step.done{border-top-color:var(--done)}.step.doing{border-top-color:var(--doing)}.step.waiting{border-top-color:var(--waiting)}
.step-head{display:flex;align-items:center;gap:8px;font-weight:700}.mark{font-size:18px}
.step.done .mark,.step.done .state{color:var(--done)}.step.doing .mark,.step.doing .state{color:var(--doing)}
.step.waiting .mark,.step.waiting .state{color:var(--waiting)}.step.todo .mark,.step.todo .state{color:var(--todo)}
.state{display:block;margin:4px 0 2px;font-size:14px;font-weight:600}.note{margin:0;color:var(--muted);font-size:14px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px}
.tile{background:#fff;border:1px solid var(--line);border-radius:10px;padding:14px}
.value{font-size:26px;font-weight:700;line-height:1.2}.label{font-weight:600;margin-top:4px}
.cards{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}
.card{background:#fff;border:1px solid var(--line);border-radius:10px;padding:16px}
.tag{display:inline-block;min-width:26px;padding:0 7px;margin-right:6px;border-radius:6px;background:var(--ink);color:#fff;text-align:center;font-size:14px}
ul{margin:0;padding-left:22px}li{margin:3px 0}
ul.done li::marker{content:"✓  ";color:var(--done)}ul.doing li::marker{content:"▶  ";color:var(--doing)}
ul.attention li::marker{content:"!  ";color:var(--waiting)}
figure{margin:0;background:#fff;border:1px solid var(--line);border-radius:10px;padding:12px}
figure img{display:block;max-width:100%;height:auto;margin:0 auto}figcaption{margin-top:8px;color:var(--muted);font-size:14px}
.figs{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px;margin-top:12px}.fig.wide{grid-column:1/-1}
.table-wrap{overflow-x:auto;margin-top:16px}table{border-collapse:collapse;width:100%;background:#fff;font-size:15px}
th,td{border:1px solid var(--line);padding:6px 10px;text-align:left;vertical-align:top}thead th{background:#f1f5f9}
.you{background:#fff7ed;border:1px solid #fed7aa;border-radius:10px;padding:16px}
footer{color:var(--muted);font-size:14px}details summary{cursor:pointer;font-weight:600}code{font-size:13px;word-break:break-all}
@media (max-width:820px){.ladder{grid-template-columns:1fr}.cards{grid-template-columns:1fr}.figs{grid-template-columns:1fr}}
"""


def _map_section(content, figures, e):
    blocks = []
    for spec, fig in zip(content["figures"], figures or []):
        (page_w, page_h), (full_w, full_h) = fig["page_size"], fig["full_size"]
        wide = "" if spec["window"] else " wide"  # whole maps span the page; same-window close-ups sit side by side
        blocks.append(f'<figure class="fig{wide}"><h3>{e(spec["title"])}</h3><a href="{e(fig["full"])}">'
                      f'<img src="{e(fig["page"])}" width="{page_w}" height="{page_h}" alt="{e(spec["title"])}"></a>'
                      f'<figcaption>{e(spec["caption"])} <a href="{e(fig["full"])}">看原尺寸（{full_w}×{full_h} 像素）'
                      '</a></figcaption></figure>')
    table = ""
    if content.get("map_table"):
        spec = content["map_table"]
        head = "".join(f'<th scope="col">{e(c)}</th>' for c in spec["columns"])
        body = "".join(f'<tr><th scope="row">{e(row[0])}</th>' + "".join(f"<td>{e(c)}</td>" for c in row[1:])
                       + "</tr>" for row in spec["rows"])
        table = f'<div class="table-wrap"><table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>'
        if spec["note"]:
            table += f'<p class="note">{e(spec["note"])}</p>'
    if not blocks and not table:
        return ""
    legend = f'<p class="note">{e(LEGEND)}</p><div class="figs">{"".join(blocks)}</div>' if blocks else ""
    return f'<section aria-labelledby="map-h"><h2 id="map-h">走廊地图</h2>{legend}{table}</section>'


def render_html(content, figures=None):
    """The page; ``figures`` names the written PNG files of each content figure, in order."""
    e = lambda value: html.escape(value, quote=True)  # noqa: E731 - every text field goes through this
    steps = []
    for i, step in enumerate(content["ladder"], 1):
        label, mark = STATES[step["state"]]
        steps.append(f'<li class="step {step["state"]}"><div class="step-head"><span class="mark" aria-hidden="true">'
                     f'{mark}</span><span>{i}. {e(step["name"])}</span></div><span class="state">{label}</span>'
                     f'<p class="note">{e(step["note"])}</p></li>')
    tiles = "".join(f'<div class="tile"><div class="value">{e(t["value"])}</div><div class="label">{e(t["label"])}'
                    f'</div><p class="note">{e(t["note"])}</p></div>' for t in content["headline"])
    cards = []
    for line in content["lines"]:
        parts = [f'<article class="card"><h3><span class="tag">{e(line["key"])}</span>{e(line["name"])}</h3>']
        for key, title in (("done", "已完成"), ("doing", "正在做"), ("attention", "需要注意")):
            if line[key]:
                parts.append(f'<h4>{title}</h4><ul class="{key}">'
                             + "".join(f"<li>{e(item)}</li>" for item in line[key]) + "</ul>")
        cards.append("".join(parts) + "</article>")
    sections = [
        f'<section aria-labelledby="ladder-h"><h2 id="ladder-h">走到哪一步了</h2>'
        f'<ol class="ladder" style="--steps:{len(steps)}">{"".join(steps)}</ol></section>']
    if tiles:
        sections.append(f'<section aria-labelledby="key-h"><h2 id="key-h">关键数字</h2><div class="tiles">{tiles}</div>'
                        f'</section>')
    sections.append(f'<section aria-labelledby="lines-h"><h2 id="lines-h">各条工作线</h2>'
                    f'<div class="cards">{"".join(cards)}</div></section>')
    sections.append(_map_section(content, figures, e))
    if content["needs_user"]:
        sections.append('<section class="you" aria-labelledby="you-h"><h2 id="you-h">需要你做的</h2><ul>'
                        + "".join(f"<li>{e(item)}</li>" for item in content["needs_user"]) + "</ul></section>")
    sources = "".join(f"<li>{e(s['label'])}：<code>{e(s['path'])}</code></li>" for s in content["sources"])
    footer = (f'<footer><details><summary>数据来源（{len(content["sources"])} 项）</summary><ul>{sources}</ul></details>'
              '<p>离线快照：页面不读实时数据，不连接车辆。</p></footer>')
    return ('<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width, initial-scale=1">'
            f'<title>{e(content["title"])}</title><style>{CSS}</style></head><body><main class="page">'
            f'<header><p class="eyebrow">离线进度快照 · 更新于 {e(content["updated"])}</p><h1>{e(content["title"])}</h1>'
            f'<p class="status">{e(content["status_line"])}</p><p class="safety">{e(content["safety_line"])}</p></header>'
            + "".join(sections) + footer + "</main></body></html>")


def write_landing(dashboard_root):
    """Replace the dashboard root ``index.html`` with a short list of snapshots, newest overview first.

    The root page is only a pointer, so it is replaced atomically each time;
    the snapshot directories themselves are never touched.
    """
    import os
    import tempfile
    root = Path(dashboard_root)
    overviews, others = [], []
    for directory in sorted((p for p in root.iterdir() if p.is_dir() and (p / "index.html").is_file()),
                            key=lambda p: p.name, reverse=True):
        content = directory / "content.json"
        if content.is_file():
            try:
                data = json.loads(content.read_text(encoding="utf-8"))
                overviews.append((directory.name, str(data.get("updated", "")), str(data.get("title", "进展总览"))))
                continue
            except (OSError, ValueError):
                pass
        others.append(directory.name)
    # Newest first by the content's own "updated" stamp (YYYY-MM-DD HH:MM sorts as text), not by directory name.
    overviews.sort(key=lambda item: (item[1], item[0]), reverse=True)
    e = lambda value: html.escape(value, quote=True)  # noqa: E731
    parts = ['<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">'
             '<meta name="viewport" content="width=device-width, initial-scale=1"><title>轮椅导航进展</title>'
             f'<style>{CSS}.big{{display:inline-block;margin:6px 0 4px;padding:12px 18px;border-radius:10px;'
             'background:var(--doing);color:#fff;font-size:18px;font-weight:700;text-decoration:none}'
             '.list a{color:var(--doing)}</style></head><body><main class="page">'
             '<header><p class="eyebrow">离线进度快照目录</p><h1>轮椅导航进展</h1></header>']
    if overviews:
        name, updated, title = overviews[0]
        parts.append(f'<section><a class="big" href="{e(name)}/">打开最新的进展总览</a>'
                     f'<p class="note">{e(title)} · 更新于 {e(updated)}</p></section>')
    if overviews[1:]:
        parts.append('<section class="list"><h2>以前的总览</h2><ul>' + "".join(
            f'<li><a href="{e(n)}/">{e(n)}</a>（{e(u)}）</li>' for n, u, _ in overviews[1:]) + "</ul></section>")
    if others:
        parts.append('<section class="list"><h2>详细看板（旧版，含历史数据）</h2><ul>' + "".join(
            f'<li><a href="{e(n)}/">{e(n)}</a></li>' for n in others) + "</ul></section>")
    parts.append("<footer><p>离线快照：页面不读实时数据，不连接车辆。</p></footer></main></body></html>")
    page = "".join(parts)
    handle, temporary = tempfile.mkstemp(dir=root, prefix=".index.", suffix=".html")
    with os.fdopen(handle, "w", encoding="utf-8") as stream:
        stream.write(page)
    os.chmod(temporary, 0o644)
    os.replace(temporary, root / "index.html")
    return {"landing": str(root / "index.html"), "overviews": len(overviews), "other_snapshots": len(others)}


def build(content_path, output):
    """Validate, render and write a new overview directory; return a summary dict."""
    raw = Path(content_path).read_bytes()
    if len(raw) > MAX_CONTENT_BYTES:
        raise ContentError("content file is too large")
    content = validate_content(json.loads(raw.decode("utf-8")))
    rendered = [render_figure(spec) for spec in content["figures"]]  # before mkdir: a failure leaves nothing behind
    names = [{"page": f"fig{i}.png", "full": f"fig{i}_full.png", "page_size": r["page_size"],
              "full_size": r["full_size"]} for i, r in enumerate(rendered, 1)]
    inputs = {}
    for result in rendered:
        inputs.update(result["inputs"])
    page = render_html(content, names)
    output = Path(output)
    output.mkdir(parents=True)  # refuses an existing directory
    with (output / "content.json").open("xb") as stream:
        stream.write(raw)
    with (output / "index.html").open("x", encoding="utf-8") as stream:
        stream.write(page)
    figures_sha256 = {}
    for name, result in zip(names, rendered):
        for key in ("page", "full"):
            with (output / name[key]).open("xb") as stream:
                stream.write(result[key])
            figures_sha256[name[key]] = hashlib.sha256(result[key]).hexdigest()
    sources = {}
    for source in content["sources"]:
        path = Path(source["path"]).expanduser()
        sources[source["path"]] = ("FILE " + _sha256_file(path)) if path.is_file() else (
            "DIRECTORY" if path.is_dir() else "NOT_FOUND")
    manifest = {"schema_version": 1, "content_sha256": hashlib.sha256(raw).hexdigest(),
                "index_html_sha256": hashlib.sha256(page.encode("utf-8")).hexdigest(),
                "map_inputs_sha256": inputs, "figures_sha256": figures_sha256, "sources": sources,
                "hardware_started": False, "live_data": False}
    with (output / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return {"status": "OVERVIEW_WRITTEN", "output": str(output), "index_bytes": len(page.encode("utf-8")),
            "figures": len(rendered), "missing_sources": sorted(k for k, v in sources.items() if v == "NOT_FOUND")}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--content", type=Path, required=True, help="curated overview JSON (schema_version 1)")
    parser.add_argument("--output-dir", required=True, help="new directory under reports/, e.g. "
                                                            "reports/navigation_dashboard/20261108_overview")
    parser.add_argument("--no-landing", action="store_true",
                        help="do not refresh the index.html in the parent directory (a list of snapshots)")
    args = parser.parse_args(argv)
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(args.project_root)
    policy.check()
    output = policy.resolve(args.output_dir)
    logical = Path(policy.logical(output))
    if not logical.parts or logical.parts[0] != "reports" or output.exists():
        parser.error("--output-dir must be a new directory under reports/")
    try:
        result = build(args.content, output)
        if not args.no_landing:
            result.update(write_landing(output.parent))
    except (ContentError, OSError, ValueError, KeyError) as error:
        print(json.dumps({"status": "FAILED", "error": str(error)}, ensure_ascii=False))
        return 1
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
