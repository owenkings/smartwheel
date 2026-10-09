/* Organized image uses original point IDs. Display flips never alter XYZ. */
(function (root, factory) {
  "use strict";
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.AmplitudeUI = api;
})(typeof window !== "undefined" ? window : null, function () {
  "use strict";
  const finite = value => typeof value === "number" && Number.isFinite(value);
  function check(ok, message) { if (!ok) throw new Error(message); }
  function validate(layout, points) {
    check(layout && Number.isInteger(layout.width) && Number.isInteger(layout.height) && layout.width > 0 && layout.height > 0 && layout.width * layout.height <= 20000, "幅度图尺寸无效。");
    check(layout.order === "row_major", "幅度图必须明确采用原始逐行索引。");
    check(Array.isArray(layout.amplitude) && layout.amplitude.length === layout.width * layout.height && layout.amplitude.every(v => v === null || finite(v)), "幅度值数量或格式无效。");
    check(points.every(p => Number.isInteger(p.id) && p.id >= 0 && p.id < layout.width * layout.height), "点索引超出幅度图范围。");
    return layout;
  }
  function viewport(layout, width, height, zoom = 1, pan = [0, 0]) {
    check([width, height, zoom, ...pan].every(finite) && width > 0 && height > 0 && zoom > 0, "图像视口无效。");
    const scale = Math.min(width / layout.width, height / layout.height) * zoom;
    return { scale, x: (width - layout.width * scale) / 2 + pan[0], y: (height - layout.height * scale) / 2 + pan[1], width, height };
  }
  function hitPixel(layout, view, x, y, points) {
    if (!finite(x) || !finite(y) || x < 0 || y < 0 || x >= view.width || y >= view.height) return null;
    const displayCol = Math.floor((x - view.x) / view.scale), row = Math.floor((y - view.y) / view.scale);
    if (displayCol < 0 || displayCol >= layout.width || row < 0 || row >= layout.height) return null;
    const col = layout.width - 1 - displayCol, id = row * layout.width + col;
    return { id, row, col, point: points.get(id) || null, amplitude: layout.amplitude[id] };
  }
  function pixelCenter(layout, view, id) {
    return { x: view.x + (layout.width - 1 - id % layout.width + .5) * view.scale, y: view.y + (Math.floor(id / layout.width) + .5) * view.scale };
  }
  function eventPosition(rect, width, height, clientX, clientY) {
    check(rect.width > 0 && rect.height > 0, "画布没有可见尺寸。");
    return [(clientX - rect.left) * width / rect.width, (clientY - rect.top) * height / rect.height];
  }
  function bounds(values) {
    const sorted = values.filter(finite).sort((a, b) => a - b);
    if (!sorted.length) return [0, 1];
    const low = sorted[Math.floor((sorted.length - 1) * .02)], high = sorted[Math.floor((sorted.length - 1) * .98)];
    return high > low ? [low, high] : [low - .5, high + .5];
  }
  function labels(state, side) {
    const entries = state.pairs.map((pair, i) => ({ id: pair[side + "_id"], label: String(i + 1), pending: false }));
    const pending = state.pending[side + "_id"];
    if (pending !== null) entries.push({ id: pending, label: String(state.pairs.length + 1) + "?", pending: true });
    return entries;
  }
  class ImagePanel {
    constructor(side, layout, points, options) {
      this.side = side; this.layout = validate(layout, points); this.options = options;
      this.points = new Map(points.map(p => [p.id, p])); this.zoom = 1; this.pan = [0, 0]; this.mode = "amplitude";
      this.queued = false; this.gesture = null;
      const $ = id => document.getElementById(side + "-" + id);
      this.canvas = $("image"); this.ctx = this.canvas.getContext("2d", { alpha: false });
      this.legend = $("image-legend"); this.readout = $("image-readout");
      this.image = document.createElement("canvas"); this.image.width = layout.width; this.image.height = layout.height;
      $("image-section").hidden = false;
      $("image-mode").addEventListener("change", event => { this.clearHover(); this.mode = event.target.value; this.prepare(); this.schedule(); });
      $("image-reset").addEventListener("click", () => { this.clearHover(); this.zoom = 1; this.pan = [0, 0]; this.schedule(); });
      $("image-plus").addEventListener("click", () => this.scaleAt(1.5, this.canvas.width / 2, this.canvas.height / 2));
      $("image-minus").addEventListener("click", () => this.scaleAt(1 / 1.5, this.canvas.width / 2, this.canvas.height / 2));
      this.prepare(); this.resize(); this.attach();
      if (typeof ResizeObserver !== "undefined") { this.observer = new ResizeObserver(() => this.resize()); this.observer.observe(this.canvas); }
      else window.addEventListener("resize", () => this.resize());
    }
    resize() {
      this.clearHover();
      const rect = this.canvas.getBoundingClientRect();
      this.canvas.width = Math.max(1, Math.min(2048, Math.round(rect.width)));
      this.canvas.height = Math.max(1, Math.min(1000, Math.round(rect.height)));
      this.schedule();
    }
    prepare() {
      const { width, height, amplitude } = this.layout;
      const values = Array.from({ length: width * height }, (_, id) => {
        const point = this.points.get(id);
        return point ? (this.mode === "x" ? point.xyz[0] : amplitude[id]) : null;
      });
      const [lo, hi] = bounds(values), ctx = this.image.getContext("2d"), frame = ctx.createImageData(width, height);
      for (let row = 0; row < height; row++) for (let col = 0; col < width; col++) {
        const value = values[row * width + col], offset = (row * width + width - 1 - col) * 4;
        const color = finite(value) ? Math.round(255 * Math.max(0, Math.min(1, (value - lo) / (hi - lo)))) : null;
        frame.data[offset] = color === null ? 38 : color;
        frame.data[offset + 1] = color === null ? 20 : color;
        frame.data[offset + 2] = color === null ? 43 : color;
        frame.data[offset + 3] = 255;
      }
      ctx.putImageData(frame, 0, 0);
      this.legend.textContent = (this.mode === "x" ? "黑近白远 · 前向 X：" : "黑弱白强 · 回波幅度：") + lo.toFixed(2) + "～" + hi.toFixed(2) + (this.mode === "x" ? " m" : "") + "（有效点 2%～98% 显示范围）· 暗紫色为缺值/无有效 XYZ";
    }
    view() { return viewport(this.layout, this.canvas.width, this.canvas.height, this.zoom, this.pan); }
    schedule() { if (this.queued) return; this.queued = true; requestAnimationFrame(() => { this.queued = false; this.draw(); }); }
    draw() {
      const view = this.view(), ctx = this.ctx;
      ctx.fillStyle = "#08111d"; ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(this.image, view.x, view.y, this.layout.width * view.scale, this.layout.height * view.scale);
      ctx.lineWidth = 2; ctx.font = "bold 13px sans-serif";
      for (const entry of labels(this.options.state(), this.side)) {
        const p = pixelCenter(this.layout, view, entry.id);
        ctx.strokeStyle = entry.pending ? "#ffdf87" : "#47d9e5";
        ctx.beginPath(); ctx.arc(p.x, p.y, 7, 0, Math.PI * 2); ctx.stroke();
        ctx.fillStyle = "#08111d"; ctx.fillRect(p.x + 8, p.y - 17, entry.label.length * 9 + 6, 18);
        ctx.fillStyle = ctx.strokeStyle; ctx.fillText(entry.label, p.x + 11, p.y - 3);
      }
      const previousOrigin = this.canvas.dataset.hoverOrigin;
      for (const key of ["hoverId", "hoverOrigin", "hoverState", "hoverRow", "hoverCol"]) delete this.canvas.dataset[key];
      const preview = this.options.hover && this.options.hover();
      if (preview && preview.side === this.side && this.points.has(preview.id) && this.options.canSelect()) {
        const p = pixelCenter(this.layout, view, preview.id), row = Math.floor(preview.id / this.layout.width), col = preview.id % this.layout.width;
        const inside = p.x >= 0 && p.x < this.canvas.width && p.y >= 0 && p.y < this.canvas.height;
        Object.assign(this.canvas.dataset, {hoverId:String(preview.id), hoverOrigin:preview.origin, hoverState:inside ? "linked" : "outside", hoverRow:String(row), hoverCol:String(col)});
        if (inside) {
          ctx.save(); ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.arc(p.x, p.y, 10, 0, Math.PI * 2);
          ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 4; ctx.stroke(); ctx.strokeStyle = "#8a3bbd"; ctx.lineWidth = 2; ctx.stroke(); ctx.restore();
        }
        if (preview.origin === "cloud") this.readout.textContent = "同侧三维点 #" + preview.id + " · 原始行 " + row + " / 列 " + col + (inside ? " · 虚线仅预览，点击才选取" : " · 对应像素在当前视图之外");
      } else if (previousOrigin === "cloud") this.readout.textContent = this.describe(null);
    }
    clearHover() { if (this.options.clearHover) this.options.clearHover(); }
    hit(event) { const p = this.position(event); return hitPixel(this.layout, this.view(), p[0], p[1], this.points); }
    position(event) { return eventPosition(this.canvas.getBoundingClientRect(), this.canvas.width, this.canvas.height, event.clientX, event.clientY); }
    describe(hit) {
      if (!hit) return "像素保持等比例 · 列按乘坐视角逆序显示 · 原始行列从 0 计数";
      const start = "原始点 #" + hit.id + " · 行 " + hit.row + " / 列 " + hit.col;
      return hit.point ? start + " · XYZ [" + hit.point.xyz.map(v => v.toFixed(4)).join(", ") + "] m · 幅度 " + (hit.amplitude === null ? "缺值" : hit.amplitude.toFixed(2)) : start + " · 无有效 XYZ，不能选点";
    }
    scaleAt(ratio, x, y) {
      this.clearHover();
      const old = this.zoom; this.zoom = Math.max(1, Math.min(12, old * ratio)); ratio = this.zoom / old;
      this.pan = this.pan.map((p, i) => (p - ([x, y][i] - [this.canvas.width, this.canvas.height][i] / 2)) * ratio + [x, y][i] - [this.canvas.width, this.canvas.height][i] / 2);
      if (this.zoom === 1) this.pan = [0, 0]; this.schedule();
    }
    attach() {
      this.canvas.addEventListener("pointerdown", event => {
        if (event.button !== 0) return;
        this.clearHover();
        const [x, y] = this.position(event); this.gesture = { x, y, lastX: x, lastY: y, movement: 0, pointer: event.pointerId };
        this.canvas.setPointerCapture(event.pointerId); this.canvas.focus();
      });
      this.canvas.addEventListener("pointermove", event => {
        const [x, y] = this.position(event), g = this.gesture;
        if (g && g.pointer === event.pointerId) {
          g.movement = Math.max(g.movement, Math.hypot(x - g.x, y - g.y));
          if (g.movement >= 4) { this.clearHover(); this.pan[0] += x - g.lastX; this.pan[1] += y - g.lastY; this.schedule(); }
          g.lastX = x; g.lastY = y;
        }
        const hit = this.hit(event); this.readout.textContent = this.describe(hit);
        if (this.options.onHover) this.options.onHover(this.side, !g && this.options.canSelect() && hit && hit.point ? hit.id : null);
      });
      this.canvas.addEventListener("pointerup", event => {
        const g = this.gesture; this.gesture = null;
        this.clearHover();
        if (!g || g.pointer !== event.pointerId) return;
        if (this.canvas.hasPointerCapture(event.pointerId)) this.canvas.releasePointerCapture(event.pointerId);
        const [x, y] = this.position(event);
        if (Math.max(g.movement, Math.hypot(x - g.x, y - g.y)) >= 4 || event.button !== 0) return;
        const hit = this.hit(event); this.readout.textContent = this.describe(hit);
        if (!this.options.canSelect()) return;
        if (!hit || !hit.point) { this.options.error("这个像素没有有效的原始 XYZ，不能组成对应点。请选择实际表面上的有效像素。"); return; }
        this.options.select(this.side, hit.id);
      });
      this.canvas.addEventListener("pointercancel", () => { this.gesture = null; this.clearHover(); });
      this.canvas.addEventListener("lostpointercapture", () => { this.gesture = null; this.clearHover(); });
      this.canvas.addEventListener("pointerleave", () => { this.clearHover(); if (!this.gesture) this.readout.textContent = this.describe(null); });
      this.canvas.addEventListener("wheel", event => { event.preventDefault(); const [x, y] = this.position(event); this.scaleAt(Math.exp(-Math.max(-200, Math.min(200, event.deltaY)) * .0015), x, y); }, { passive: false });
    }
  }
  return { validate, viewport, hitPixel, pixelCenter, eventPosition, bounds, labels, ImagePanel };
});
