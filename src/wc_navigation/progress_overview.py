"""Plain one-page progress overview: what has been done, what is next.

The detailed dashboard (``dashboard.py``) shows every archived run. This page
is the front door for the user: the validation ladder, a few headline
numbers, one card per workstream and what needs the user. Its text comes
from a small, human-curated JSON file; the only figure is the corridor map in
use, drawn from the map files themselves. The page is one self-contained HTML
file (no scripts, no network), written to a new directory under ``reports``;
an existing snapshot is never overwritten. Nothing here starts ROS or touches
the vehicle.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import html
import json
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
PATH_COLOR, START_COLOR = (37, 99, 235), (22, 163, 74)


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


def validate_content(content):
    """Return a normalised copy; raise ContentError on anything unexpected."""
    if not isinstance(content, dict) or content.get("schema_version") != 1:
        raise ContentError("schema_version must be 1")
    allowed = {"schema_version", "title", "updated", "status_line", "safety_line", "ladder", "headline",
               "lines", "needs_user", "map", "sources"}
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
    map_spec = content.get("map")
    if map_spec is not None:
        if not isinstance(map_spec, dict) or set(map_spec) - {"map_yaml", "path_csv", "caption"}:
            raise ContentError("map: expected map_yaml, optional path_csv, caption")
        out["map"] = {"map_yaml": _text(map_spec.get("map_yaml"), "map.map_yaml"),
                      "path_csv": _text(map_spec["path_csv"], "map.path_csv") if map_spec.get("path_csv") else None,
                      "caption": _text(map_spec.get("caption"), "map.caption")}
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


def render_map_png(map_yaml, path_csv=None, *, max_width=1000, padding_px=12):
    """Trinary map cropped to what was observed, with the recorded axle path; PNG bytes and inputs."""
    import numpy as np
    import yaml
    from PIL import Image, ImageDraw

    map_yaml = Path(map_yaml)
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    pgm = (map_yaml.parent / meta["image"]).resolve()
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin"][0]), float(meta["origin"][1])
    with Image.open(pgm) as image:
        pixels = np.asarray(image.convert("L"))
    known = np.argwhere(pixels != 205)
    if not len(known):
        raise ContentError("the map has no observed cells")
    height, width = pixels.shape
    top, left = np.maximum(known.min(axis=0) - padding_px, 0)
    bottom, right = np.minimum(known.max(axis=0) + padding_px + 1, (height, width))
    crop = pixels[top:bottom, left:right]
    rgb = np.empty(crop.shape + (3,), dtype=np.uint8)
    rgb[:] = MAP_COLORS[205]
    rgb[crop >= 250] = MAP_COLORS[254]
    walls = crop <= 50
    thick = walls.copy()  # one-pixel dilation, display only: 5 cm walls are invisible at page scale
    thick[1:, :] |= walls[:-1, :]
    thick[:-1, :] |= walls[1:, :]
    thick[:, 1:] |= walls[:, :-1]
    thick[:, :-1] |= walls[:, 1:]
    rgb[thick] = MAP_COLORS[0]
    scale = max(1, int(max_width // max(crop.shape[1], 1)))
    figure = Image.fromarray(rgb).resize((crop.shape[1] * scale, crop.shape[0] * scale), Image.NEAREST)
    inputs = {str(map_yaml.resolve()): _sha256_file(map_yaml), str(pgm): _sha256_file(pgm)}
    if path_csv:
        import csv
        points = []
        with Path(path_csv).open(encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                x, y = float(row["used_x"]), float(row["used_y"])
                column = (x - origin_x) / resolution - left
                row_px = (height - 1 - (y - origin_y) / resolution) - top
                points.append(((column + 0.5) * scale, (row_px + 0.5) * scale))
        if points:
            draw = ImageDraw.Draw(figure)
            draw.line(points, fill=PATH_COLOR, width=max(2, scale * 2), joint="curve")
            sx, sy = points[0]
            radius = max(8, scale * 6)
            draw.ellipse((sx - radius, sy - radius, sx + radius, sy + radius), fill=START_COLOR,
                         outline=(255, 255, 255), width=2)
        inputs[str(Path(path_csv).resolve())] = _sha256_file(path_csv)
    if figure.width > max_width:
        figure = figure.resize((max_width, max(1, round(figure.height * max_width / figure.width))), Image.LANCZOS)
    out = BytesIO()
    figure.save(out, format="PNG", optimize=True)
    return out.getvalue(), inputs


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
.you{background:#fff7ed;border:1px solid #fed7aa;border-radius:10px;padding:16px}
footer{color:var(--muted);font-size:14px}details summary{cursor:pointer;font-weight:600}code{font-size:13px;word-break:break-all}
@media (max-width:820px){.ladder{grid-template-columns:1fr}.cards{grid-template-columns:1fr}}
"""


def render_html(content, map_png=None):
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
    if map_png is not None and content.get("map"):
        uri = "data:image/png;base64," + base64.b64encode(map_png).decode("ascii")
        caption = e(content["map"]["caption"])
        sections.append(f'<section aria-labelledby="map-h"><h2 id="map-h">在用的走廊地图</h2><figure>'
                        f'<img src="{uri}" alt="{caption}"><figcaption>{caption}</figcaption></figure></section>')
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
    map_png, inputs = None, {}
    if content.get("map"):
        map_png, inputs = render_map_png(content["map"]["map_yaml"], content["map"]["path_csv"])
    page = render_html(content, map_png)
    output = Path(output)
    output.mkdir(parents=True)  # refuses an existing directory
    with (output / "content.json").open("xb") as stream:
        stream.write(raw)
    with (output / "index.html").open("x", encoding="utf-8") as stream:
        stream.write(page)
    sources = {}
    for source in content["sources"]:
        path = Path(source["path"]).expanduser()
        sources[source["path"]] = ("FILE " + _sha256_file(path)) if path.is_file() else (
            "DIRECTORY" if path.is_dir() else "NOT_FOUND")
    manifest = {"schema_version": 1, "content_sha256": hashlib.sha256(raw).hexdigest(),
                "index_html_sha256": hashlib.sha256(page.encode("utf-8")).hexdigest(),
                "map_inputs_sha256": inputs, "sources": sources, "hardware_started": False,
                "live_data": False}
    with (output / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return {"status": "OVERVIEW_WRITTEN", "output": str(output), "index_bytes": len(page.encode("utf-8")),
            "missing_sources": sorted(k for k, v in sources.items() if v == "NOT_FOUND")}


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
