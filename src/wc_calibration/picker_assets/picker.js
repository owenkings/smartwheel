/* Offline picker: source IDs are the only selectable values sent to the server. */
(function (root, factory) {
  "use strict";
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root && root.document) {
    root.PickerUI = api;
    const startPicker = () => { if (root.document.body.dataset.page !== "workbench" && root.document.getElementById("left-canvas")) api.start(); };
    if (root.document.readyState === "loading") root.document.addEventListener("DOMContentLoaded", startPicker);
    else startPicker();
  }
})(typeof window !== "undefined" ? window : null, function () {
  "use strict";
  const SIDES = ["left", "right"];
  const COLORS = { left: [71, 217, 229], right: [255, 181, 107] };
  const BACKGROUND = [8, 17, 29];
  const finite = value => typeof value === "number" && Number.isFinite(value);
  const vector = value => Array.isArray(value) && value.length === 3 && value.every(finite);
  const text = value => typeof value === "string" && value.length > 0 && value.length <= 4096;
  function requireThat(condition, message) { if (!condition) throw new Error(message); }

  function validateScene(scene) {
    requireThat(scene && typeof scene === "object", "点云场景不是有效对象。");
    requireThat(typeof scene.input_hash === "string" && /^[a-f0-9]{64}$/i.test(scene.input_hash), "缺少有效来源哈希。");
    requireThat(text(scene.scene_id) && text(scene.token), "缺少场景标识或会话令牌。");
    requireThat(["real", "synthetic"].includes(scene.source_mode), "必须明确真实或合成来源。");
    requireThat(scene.units === "m", "本页面只接受已明确为米的 FLU 点云，不推测单位。");
    for (const side of SIDES) {
      requireThat(scene.sensor_ids && text(scene.sensor_ids[side]), "缺少左右传感器身份。");
      requireThat(scene.coordinate_conventions && scene.coordinate_conventions[side] === "FLU", "本页面需要明确的原始 FLU 坐标约定，不推测坐标轴。");
      const cloud = scene.clouds && scene.clouds[side];
      requireThat(Array.isArray(cloud) && cloud.length > 0 && cloud.length <= 20000, "每侧须有 1～20000 个有效原始点。");
      const ids = new Set();
      for (const point of cloud) {
        requireThat(point && Number.isSafeInteger(point.id) && point.id >= 0 && point.id < 20000 && !ids.has(point.id), "点 ID 必须是唯一的原始非负索引，且小于 20000。");
        requireThat(vector(point.xyz) && point.xyz.every(v => Math.abs(v) <= 1e9), "点坐标必须是范围内的有限三维数值。");
        ids.add(point.id);
      }
    }
    requireThat(scene.sensor_ids.left !== scene.sensor_ids.right, "左右传感器身份不能相同。");
    if (scene.level_reference != null) validateLevelReference(scene.level_reference);
    return scene;
  }

  function newView() { return { yaw: 0, pitch: 0, center: [0, 0, 0], pan: [0, 0], scale: 100 }; }
  function copyView(view) { return { ...view, center: view.center.slice(), pan: view.pan.slice() }; }
  function sharedView(clouds, sizes, previous = newView()) {
    return fitView(clouds.left.concat(clouds.right), Math.min(sizes.left.width, sizes.right.width), Math.min(sizes.left.height, sizes.right.height), previous);
  }
  function savedResponse(payload, saveRequested) {
    if (!saveRequested) {
      requireThat(payload.saved === false && payload.export_dir === null, "只计算接口必须明确未保存，不能返回保存目录。");
      return false;
    }
    requireThat(text(payload.export_dir) && payload.saved !== false, "服务没有返回有效保存路径；当前结果未确认保存。");
    return true;
  }
  function supports(scene, feature) { return !!(scene && scene.capabilities && scene.capabilities[feature] === true); }
  function assessmentText(value) {
    const labels = { REJECTED: "当前结果不宜使用", EXPLORATORY: "可继续离线探索", NOT_EVALUATED: "尚未完成评估" };
    if (!value) return { level: "NOT_EVALUATED", title: labels.NOT_EVALUATED, summary: "此历史结果未附评估，请重新计算查看。", details: [], usable: "暂无已确认用途", formal: "尚不能作为正式外参使用。" };
    requireThat(value.formal_use === false && Object.hasOwn(labels, value.level), "服务返回的评估状态无效，不能升级为正式外参。");
    return { level: value.level, title: labels[value.level], summary: String(value.summary || ""),
      details: Array.isArray(value.details) ? value.details.map(String) : [], usable: Array.isArray(value.usable_for) && value.usable_for.length ? value.usable_for.map(String).join("；") : "暂无已确认用途", formal: "尚不能作为正式外参使用。" };
  }
  function renderAssessment(element, value) {
    const a = assessmentText(value); element.hidden = false; element.dataset.level = a.level; element.replaceChildren();
    const title = document.createElement("h3"), summary = document.createElement("p"), list = document.createElement("ul"), use = document.createElement("p");
    title.textContent = a.title; summary.textContent = a.summary;
    for (const detail of a.details) { const item = document.createElement("li"); item.textContent = detail; list.append(item); }
    use.textContent = "可用于：" + a.usable + "。" + a.formal;
    element.append(title, summary, list, use);
  }
  function projectPoint(point, view, width, height) {
    if (!point || !vector(point.xyz) || !vector(view.center) || !finite(view.scale) || view.scale <= 0 ||
        !finite(view.yaw) || !finite(view.pitch) || !Array.isArray(view.pan) || view.pan.length !== 2 || !view.pan.every(finite)) return null;
    const [dx, dy, dz] = point.xyz.map((value, i) => value - view.center[i]);
    const cy = Math.cos(view.yaw), sy = Math.sin(view.yaw);
    const cp = Math.cos(view.pitch), sp = Math.sin(view.pitch);
    const rx = dx * cy - dy * sy, ry = dx * sy + dy * cy;
    const result = { id: point.id, side: point.side, x: width / 2 - ry * view.scale + view.pan[0],
      y: height / 2 - (dz * cp - rx * sp) * view.scale + view.pan[1], depth: rx * cp + dz * sp };
    return [result.x, result.y, result.depth].every(finite) ? result : null;
  }
  function fitView(points, width, height, previous = newView()) {
    requireThat(points.length > 0 && width > 0 && height > 0, "适应视图需要点云及非零画布。");
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (const point of points) {
      requireThat(vector(point.xyz), "不能投影无效坐标。");
      for (let i = 0; i < 3; ++i) { lo[i] = Math.min(lo[i], point.xyz[i]); hi[i] = Math.max(hi[i], point.xyz[i]); }
    }
    const view = { ...previous, center: lo.map((v, i) => (v + hi[i]) / 2), pan: [0, 0], scale: 1 };
    let extentX = 0, extentY = 0;
    for (const point of points) {
      const p = projectPoint(point, view, width, height);
      extentX = Math.max(extentX, Math.abs(p.x - width / 2));
      extentY = Math.max(extentY, Math.abs(p.y - height / 2));
    }
    view.scale = Math.min(width * .43 / Math.max(extentX, .01), height * .43 / Math.max(extentY, .01));
    return view;
  }

  // Draw and pick from the SAME depth buffer. Hidden points cannot be selected.
  function rasterize(projected, width, height, radius = 1.8) {
    requireThat(Number.isInteger(width) && Number.isInteger(height) && width > 0 && height > 0 && width <= 4096 && height <= 2160, "画布尺寸超出范围。");
    requireThat(finite(radius) && radius > 0 && radius <= 8, "点绘制半径无效。");
    const indices = new Int32Array(width * height); indices.fill(-1);
    const depths = new Float64Array(width * height); depths.fill(Infinity);
    for (let i = 0; i < projected.length; ++i) {
      const p = projected[i];
      if (!p || ![p.x, p.y, p.depth].every(finite) || p.x < -radius || p.y < -radius || p.x > width + radius || p.y > height + radius) continue;
      const x0 = Math.max(0, Math.floor(p.x - radius)), x1 = Math.min(width - 1, Math.ceil(p.x + radius));
      const y0 = Math.max(0, Math.floor(p.y - radius)), y1 = Math.min(height - 1, Math.ceil(p.y + radius));
      for (let y = y0; y <= y1; ++y) for (let x = x0; x <= x1; ++x) {
        if ((x + .5 - p.x) ** 2 + (y + .5 - p.y) ** 2 > radius ** 2) continue;
        const pixel = y * width + x;
        if (p.depth < depths[pixel]) { depths[pixel] = p.depth; indices[pixel] = i; }
      }
    }
    return { indices, depths, projected, width, height };
  }
  function pickVisible(raster, x, y, radius = 9) {
    if (!raster || ![x, y, radius].every(finite) || radius <= 0 || radius > 30 || x < 0 || y < 0 || x >= raster.width || y >= raster.height) return null;
    let best = null, bestDistance = Infinity, bestDepth = Infinity;
    for (let py = Math.max(0, Math.floor(y - radius)); py <= Math.min(raster.height - 1, Math.ceil(y + radius)); ++py) {
      for (let px = Math.max(0, Math.floor(x - radius)); px <= Math.min(raster.width - 1, Math.ceil(x + radius)); ++px) {
        const distance = (px + .5 - x) ** 2 + (py + .5 - y) ** 2;
        if (distance > radius ** 2) continue;
        const pixel = py * raster.width + px, index = raster.indices[pixel];
        if (index < 0) continue;
        if (distance < bestDistance || (distance === bestDistance && raster.depths[pixel] < bestDepth)) {
          best = raster.projected[index]; bestDistance = distance; bestDepth = raster.depths[pixel];
        }
      }
    }
    return best;
  }

  function emptySelection() { return { pairs: [], pending: { left_id: null, right_id: null } }; }
  function copySelection(state) { return { pairs: state.pairs.map(p => ({ ...p })), pending: { ...state.pending } }; }
  function hasPending(state) { return SIDES.some(side => state.pending[side + "_id"] !== null); }
  class SelectionStore {
    constructor(clouds) {
      this.available = Object.fromEntries(SIDES.map(side => [side, new Set(clouds[side].map(p => p.id))]));
      this.state = emptySelection(); this.history = [];
    }
    commit(next) { this.history.push(copySelection(this.state)); if (this.history.length > 100) this.history.shift(); this.state = next; }
    select(side, id) {
      requireThat(SIDES.includes(side) && this.available[side].has(id), "只能选择当前显示点云中的原始点 ID。");
      const key = side + "_id";
      requireThat(!this.state.pairs.some(pair => pair[key] === id), "这个点已经在已完成的点对中。请先删除那一对再选择。");
      if (this.state.pending[key] === id) return false;
      const next = copySelection(this.state); next.pending[key] = id;
      if (next.pending.left_id !== null && next.pending.right_id !== null) {
        next.pairs.push({ ...next.pending }); next.pending = { left_id: null, right_id: null };
      }
      this.commit(next); return true;
    }
    remove(index) {
      requireThat(Number.isInteger(index) && index >= 0 && index < this.state.pairs.length, "点对索引无效。");
      const next = copySelection(this.state); next.pairs.splice(index, 1); this.commit(next);
    }
    cancelPending() { if (!hasPending(this.state)) return; const next = copySelection(this.state); next.pending = emptySelection().pending; this.commit(next); }
    clear() { if (this.state.pairs.length || hasPending(this.state)) this.commit(emptySelection()); }
    undo() { if (!this.history.length) return false; this.state = this.history.pop(); return true; }
  }
  // Preview candidates obey selection eligibility but never mutate SelectionStore.
  function hoverEligible(store, side, id, mode, busy) {
    return !!store && mode === "pick" && !busy && SIDES.includes(side) && Number.isSafeInteger(id) &&
      store.available[side].has(id) && store.state.pending[side + "_id"] !== id &&
      !store.state.pairs.some(pair => pair[side + "_id"] === id);
  }
  class HoverPreview {
    constructor() { this.current = null; }
    update(store, side, id, origin, mode, busy) {
      const next = ["cloud", "image"].includes(origin) && hoverEligible(store, side, id, mode, busy) ? {side, id, origin} : null;
      if (this.current === null && next === null || this.current && next && this.current.side === next.side && this.current.id === next.id && this.current.origin === next.origin) return false;
      this.current = next; return true;
    }
    clear(side = null, origin = null) {
      if (!this.current || side !== null && this.current.side !== side || origin !== null && this.current.origin !== origin) return false;
      this.current = null; return true;
    }
    forSide(side, store, mode, busy) {
      return this.current && this.current.side === side && hoverEligible(store, side, this.current.id, mode, busy) ? {...this.current} : null;
    }
  }
  function nonCollinear(points) {
    if (points.length < 3 || !points.every(vector)) return false;
    const base = points[0];
    const vectors = points.slice(1).map(p => p.map((x, j) => x - base[j]));
    const lengths = vectors.map(v => Math.hypot(...v));
    const longest = Math.max(...lengths);
    if (longest <= 1e-12) return false;
    const a = vectors[lengths.indexOf(longest)].map(x => x / longest);
    return vectors.some(v => {
      const b = v.map(x => x / longest);
      return Math.hypot(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]) > 1e-8;
    });
  }
  function solveReadiness(state, maps) {
    if (hasPending(state)) return { ok: false, reason: "请补齐当前点对的另一侧，或取消当前未完成点对。" };
    if (state.pairs.length < 3) return { ok: false, reason: "至少需要 3 对不共线点；建议选择 6～10 对分散的同一物理角点。" };
    if (!SIDES.every(side => nonCollinear(state.pairs.map(pair => maps[side].get(pair[side + "_id"]).xyz))))
      return { ok: false, reason: "当前选点共线或重复，无法确定三维刚体变换。请增加偏离这条线的角点。" };
    return { ok: true, reason: "已满足基本计算条件。近共线、误对应及残差仍由后端检查；建议 6～10 对分散角点。" };
  }
  function validateResult(result, count) {
    requireThat(result && ["CANDIDATE", "UNVALIDATED"].includes(result.status) && result.live_eligible === false, "后端必须返回未获实时使用资格的外参初值。");
    requireThat(result.units === "m", "初值结果必须以米为单位。");
    const matrix = result.initial_T_left_right;
    requireThat(Array.isArray(matrix) && matrix.length === 4 && matrix.every(row => Array.isArray(row) && row.length === 4 && row.every(finite)), "外参矩阵必须是有限 4×4 数值。");
    requireThat(matrix[3].every((v, i) => Math.abs(v - (i === 3 ? 1 : 0)) < 1e-8), "外参矩阵末行无效。");
    for (let i = 0; i < 3; ++i) for (let j = 0; j < 3; ++j) {
      const dot = matrix[i].slice(0, 3).reduce((sum, v, k) => sum + v * matrix[j][k], 0);
      requireThat(Math.abs(dot - (i === j ? 1 : 0)) < 1e-5, "返回的旋转矩阵不是刚体旋转。");
    }
    const m = matrix;
    const det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    requireThat(Math.abs(det - 1) < 1e-5, "返回的旋转不能含镜像或缩放。");
    requireThat(Array.isArray(result.residuals_m) && result.residuals_m.length === count && result.residuals_m.every(v => finite(v) && v >= 0) && finite(result.rmse_m) && result.rmse_m >= 0, "残差数量或数值无效。");
    return result;
  }
  function transformPoint(xyz, matrix) {
    requireThat(vector(xyz), "不能变换无效点。");
    const output = matrix.slice(0, 3).map(row => row[0] * xyz[0] + row[1] * xyz[1] + row[2] * xyz[2] + row[3]);
    requireThat(vector(output), "变换后坐标不是有限数值。"); return output;
  }
  function validateRigid(matrix) {
    requireThat(Array.isArray(matrix) && matrix.length === 4 && matrix.every(row => Array.isArray(row) && row.length === 4 && row.every(finite)), "显示参考必须是有限 4×4 刚体变换。");
    requireThat(matrix[3].every((v, i) => Math.abs(v - (i === 3 ? 1 : 0)) < 1e-8), "显示参考末行无效。");
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
      const dot = matrix[i].slice(0, 3).reduce((sum, v, k) => sum + v * matrix[j][k], 0);
      requireThat(Math.abs(dot - (i === j ? 1 : 0)) < 1e-6, "显示参考不得缩放或剪切点云。");
    }
    const m = matrix, det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    requireThat(Math.abs(det - 1) < 1e-6, "显示参考不得含反射。");
    return matrix;
  }
  function validateLevelReference(reference) {
    requireThat(reference && reference.status === "CANDIDATE", "水平显示参考必须明确为候选。");
    validateRigid(reference.T_level_left); return reference;
  }
  function composeRigid(a, b) {
    validateRigid(a); validateRigid(b);
    return a.map(row => b[0].map((_, j) => row.reduce((sum, value, k) => sum + value * b[k][j], 0)));
  }
  function inverseRigid(m) {
    validateRigid(m);
    const out = Array.from({ length: 4 }, (_, i) => Array.from({ length: 4 }, (_, j) => i === j ? 1 : 0));
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) out[i][j] = m[j][i];
      out[i][3] = -out[i].slice(0, 3).reduce((sum, value, j) => sum + value * m[j][3], 0);
    }
    return out;
  }
  // Display coordinates never replace the scene, raw point IDs or saved extrinsics.
  class LevelDisplay {
    constructor(reference) {
      this.available = reference != null;
      this.reference = this.available ? validateLevelReference(reference).T_level_left.map(row => row.slice()) : null;
      this.enabled = this.available;
    }
    setEnabled(value) { requireThat(typeof value === "boolean" && (!value || this.available), "当前场景没有可用水平参考。"); this.enabled = value; }
    toDisplay(raw) { validateRigid(raw); return this.enabled ? composeRigid(this.reference, raw) : raw.map(row => row.slice()); }
    toRaw(display) { validateRigid(display); return this.enabled ? composeRigid(inverseRigid(this.reference), display) : display.map(row => row.slice()); }
    cloud(points, rawTransform = null) {
      const matrix = rawTransform ? this.toDisplay(rawTransform) : this.enabled ? this.reference : null;
      return points.map(point => ({ ...point, xyz: matrix ? transformPoint(point.xyz, matrix) : point.xyz.slice() }));
    }
    overlay(clouds, rawTransform) {
      return this.cloud(clouds.left).map(point => ({ ...point, side: "left" }))
        .concat(this.cloud(clouds.right, rawTransform).map(point => ({ ...point, side: "right" })));
    }
  }
  function isPickGesture(mode, movement, shift, button = 0) { return mode === "pick" && finite(movement) && movement < 4 && !shift && button === 0; }

  function start(options = {}) {
    const $ = id => document.getElementById(id);
    const scope = document.getElementById("picker-app") || document;
    const COLORS = options.light ? {left:[8,128,150],right:[204,112,18]} : {left:[71,217,229],right:[255,181,107]};
    const BACKGROUND = options.light ? [247,250,253] : [8,17,29];
    let scene = null, store = null, maps = null, localBusy = false, externalBusy = false, mode = "browse", result = null, level = new LevelDisplay();
    const isBusy = () => localBusy || externalBusy;
    const panels = {}, imagePanels = {}, hover = new HoverPreview(); let sameScale = true;
    const error = message => { $("error").textContent = String(message); $("error").hidden = false; };
    const clearError = () => { $("error").hidden = true; $("error").textContent = ""; };
    const xyzText = xyz => xyz.map(v => v.toFixed(4)).join(", ");
    const indexText = (side, id) => {
      const layout = scene && scene.organized && scene.organized[side];
      return "#" + id + (layout ? " · 行 " + Math.floor(id / layout.width) + " / 列 " + id % layout.width : "");
    };
    const safeMetadata = value => JSON.stringify(value, null, 2).slice(0, 16000);
    function hoverFor(side) { return hover.forSide(side, store, mode, isBusy()); }
    function redrawHover(previous) {
      for (const side of new Set([previous && previous.side, hover.current && hover.current.side].filter(Boolean))) {
        if (panels[side]) panels[side].schedule();
        if (imagePanels[side]) imagePanels[side].schedule();
      }
    }
    function setHover(side, id, origin) {
      const previous = hover.current;
      if (hover.update(store, side, id, origin, mode, isBusy())) redrawHover(previous);
    }
    function clearHover(side = null, origin = null) {
      const previous = hover.current;
      if (hover.clear(side, origin)) redrawHover(previous);
    }

    class CloudPanel {
      constructor(name, points, selectable) {
        this.name = name; this.canvas = $(name + "-canvas"); this.ctx = this.canvas.getContext("2d", { alpha: false });
        requireThat(this.ctx, "浏览器不支持 Canvas2D。");
        this.points = points; this.selectable = selectable; this.view = newView(); this.raster = null; this.queued = false; this.gesture = null;
        if (level.enabled) { this.view.yaw = .25; this.view.pitch = -.4; }
        this.resize(true); this.attach();
        if (typeof ResizeObserver !== "undefined") { this.observer = new ResizeObserver(() => this.resize()); this.observer.observe(this.canvas); }
        else window.addEventListener("resize", () => this.resize());
      }
      resize(fit = false) {
        const rect = this.canvas.getBoundingClientRect();
        const width = Math.max(1, Math.min(2048, Math.round(rect.width))), height = Math.max(1, Math.min(1200, Math.round(rect.height)));
        if (width !== this.canvas.width || height !== this.canvas.height) { clearHover(); this.canvas.width = width; this.canvas.height = height; fit = true; }
        if (fit) {
          if (sameScale && SIDES.includes(this.name) && panels.left && panels.right) fitShared(this.view);
          else this.view = fitView(this.points, width, height, this.view);
        }
        this.schedule();
      }
      fit(preset) {
        clearHover();
        if (preset === "oblique") { this.view.yaw = .25; this.view.pitch = -.4; }
        if (preset === "front") { this.view.yaw = 0; this.view.pitch = 0; }
        if (preset === "top") { this.view.yaw = 0; this.view.pitch = -Math.PI / 2; }
        if (preset === "side") { this.view.yaw = Math.PI / 2; this.view.pitch = 0; }
        if (sameScale && SIDES.includes(this.name) && panels.left && panels.right) fitShared(this.view);
        else this.view = fitView(this.points, this.canvas.width, this.canvas.height, this.view);
        this.schedule();
      }
      schedule() { if (this.queued) return; this.queued = true; requestAnimationFrame(() => { this.queued = false; this.draw(); }); }
      rasterKey() { return JSON.stringify([this.canvas.width, this.canvas.height, this.view]); }
      ensureRaster() { if (!this.raster || this.rasterPoints !== this.points || this.rasterStamp !== this.rasterKey()) this.draw(); }
      draw() {
        const w = this.canvas.width, h = this.canvas.height;
        const stamp = this.rasterKey();
        if (!this.raster || this.rasterPoints !== this.points || this.rasterStamp !== stamp) {
          const projected = this.points.map(point => projectPoint(point, this.view, w, h));
          this.raster = rasterize(projected, w, h);
          this.rasterPoints = this.points; this.rasterStamp = stamp;
          this.visibleIds = new Set(); this.projectedById = new Map(projected.filter(Boolean).map(point => [point.id, point]));
          this.baseFrame = this.ctx.createImageData(w, h); const rgba = this.baseFrame.data;
          for (let i = 0; i < this.raster.indices.length; ++i) {
            const index = this.raster.indices[i], offset = i * 4;
            if (index >= 0) this.visibleIds.add(projected[index].id);
            const color = index < 0 ? BACKGROUND : COLORS[projected[index].side || this.name];
            rgba[offset] = color[0]; rgba[offset + 1] = color[1]; rgba[offset + 2] = color[2]; rgba[offset + 3] = 255;
          }
        }
        // Hover/selection only repaint overlays over this unchanged source image.
        // They do not reproject, recreate the depth buffer, or rebuild pixel colors.
        this.ctx.putImageData(this.baseFrame, 0, 0);
        this.ctx.font = "11px sans-serif"; this.ctx.fillStyle = "#71869f";
        const frameName = level.enabled && this.name !== "right" ? "水平参考（候选）" : this.name === "right" ? "右雷达原始坐标" : "左雷达原始坐标";
        this.ctx.fillText(frameName + " / " + (this.name === "overlay" ? "外参初值预览" : mode === "pick" ? "选点模式" : "浏览模式"), 12, 20);
        this.ctx.fillText("视图宽度 ≈ " + (w / this.view.scale).toFixed(2) + " 米", 12, h - 12);
        if (this.selectable && store) {
          const labels = store.state.pairs.map((pair, i) => ({ id: pair[this.name + "_id"], label: String(i + 1), pending: false }));
          const pending = store.state.pending[this.name + "_id"];
          if (pending !== null) labels.push({ id: pending, label: String(store.state.pairs.length + 1) + "?", pending: true });
          const byId = this.projectedById;
          for (const entry of labels) {
            const p = byId.get(entry.id);
            if (!p || p.x < 0 || p.x > w || p.y < 0 || p.y > h) continue;
            this.ctx.beginPath(); this.ctx.arc(p.x, p.y, 7, 0, Math.PI * 2); this.ctx.strokeStyle = entry.pending ? "#c47714" : options.light ? "#0f172a" : "#ffffff"; this.ctx.lineWidth = 2; this.ctx.stroke();
            this.ctx.fillStyle = "#08111d"; this.ctx.fillRect(p.x + 8, p.y - 17, entry.label.length * 9 + 6, 18);
            this.ctx.font = "bold 13px sans-serif"; this.ctx.fillStyle = entry.pending ? "#ffdf87" : "#ffffff"; this.ctx.fillText(entry.label, p.x + 11, p.y - 3);
          }
          const preview = hoverFor(this.name), point = preview && byId.get(preview.id), previousOrigin = this.canvas.dataset.hoverOrigin;
          for (const key of ["hoverId", "hoverOrigin", "hoverState"]) delete this.canvas.dataset[key];
          if (preview && point) {
            const inside = point.x >= 0 && point.x < w && point.y >= 0 && point.y < h;
            const state = !inside ? "outside" : this.visibleIds.has(preview.id) ? "visible" : "occluded";
            this.canvas.dataset.hoverId = String(preview.id); this.canvas.dataset.hoverOrigin = preview.origin; this.canvas.dataset.hoverState = state;
            if (inside) {
              this.ctx.save(); this.ctx.setLineDash([5, 4]); this.ctx.beginPath(); this.ctx.arc(point.x, point.y, 10, 0, Math.PI * 2);
              this.ctx.strokeStyle = "#ffffff"; this.ctx.lineWidth = 4; this.ctx.stroke();
              this.ctx.strokeStyle = "#8a3bbd"; this.ctx.lineWidth = 2; this.ctx.stroke(); this.ctx.restore();
            }
            if (preview.origin === "image") $(this.name + "-readout").textContent = "同侧像素点 " + indexText(this.name, preview.id) +
              (state === "occluded" ? " · 投影预览被前景遮挡；虚线不改变三维点击命中" : state === "outside" ? " · 投影在当前视图之外" : " · 虚线为临时投影预览，点击才选取");
          } else if (previousOrigin === "image") $(this.name + "-readout").textContent = "原始坐标 · FLU · 米；只选择光标附近可见点";
        }
      }
      position(event) {
        const rect = this.canvas.getBoundingClientRect();
        return [(event.clientX - rect.left) * this.canvas.width / rect.width, (event.clientY - rect.top) * this.canvas.height / rect.height];
      }
      attach() {
        this.canvas.addEventListener("pointerdown", event => {
          if (event.button !== 0) return;
          clearHover();
          const [x, y] = this.position(event);
          this.gesture = { x, y, lastX: x, lastY: y, movement: 0, shift: event.shiftKey, pointerId: event.pointerId };
          this.canvas.setPointerCapture(event.pointerId); this.canvas.focus();
        });
        this.canvas.addEventListener("pointermove", event => {
          const [x, y] = this.position(event), g = this.gesture;
          if (g && g.pointerId === event.pointerId) {
            g.movement = Math.max(g.movement, Math.hypot(x - g.x, y - g.y)); g.shift = g.shift || event.shiftKey;
            if (g.movement >= 4) {
              clearHover();
              const dx = x - g.lastX, dy = y - g.lastY;
              if (event.shiftKey || g.shift) { this.view.pan[0] += dx; this.view.pan[1] += dy; }
              else { this.view.yaw = (this.view.yaw + dx * .007) % (Math.PI * 2); this.view.pitch = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, this.view.pitch + dy * .007)); }
              linkView(this); this.schedule();
            }
            g.lastX = x; g.lastY = y;
          } else if (this.selectable) {
            this.ensureRaster();
            const p = pickVisible(this.raster, x, y);
            setHover(this.name, p ? p.id : null, "cloud");
            $(this.name + "-readout").textContent = p ? "原始点 " + indexText(this.name, p.id) + " · [" + xyzText(maps[this.name].get(p.id).xyz) + "] 米" : "原始坐标 · FLU · 米；只选择光标附近可见点";
          }
        });
        this.canvas.addEventListener("pointerup", event => {
          const g = this.gesture; this.gesture = null;
          clearHover(this.name, "cloud");
          if (!g || g.pointerId !== event.pointerId) return;
          if (this.canvas.hasPointerCapture(event.pointerId)) this.canvas.releasePointerCapture(event.pointerId);
          const [x, y] = this.position(event), movement = Math.max(g.movement, Math.hypot(x - g.x, y - g.y));
          if (this.selectable && !isBusy() && isPickGesture(mode, movement, g.shift || event.shiftKey, event.button)) {
            // A redraw may still be queued after a prior orbit; use current view before picking.
            this.draw();
            const p = pickVisible(this.raster, x, y);
            if (!p) { error("光标附近没有可见的原始点。请放大点云后重试。"); return; }
            try { if (store.select(this.name, p.id)) selectionChanged(); } catch (e) { error(e.message); }
          }
        });
        this.canvas.addEventListener("pointercancel", () => { this.gesture = null; clearHover(); });
        this.canvas.addEventListener("lostpointercapture", () => { this.gesture = null; clearHover(); });
        this.canvas.addEventListener("pointerleave", () => { clearHover(this.name, "cloud"); if (this.selectable && !this.gesture) $(this.name + "-readout").textContent = "原始坐标 · FLU · 米；只选择光标附近可见点"; });
        this.canvas.addEventListener("wheel", event => {
          event.preventDefault();
          clearHover();
          const [x, y] = this.position(event), old = this.view.scale;
          this.view.scale = Math.max(1e-6, Math.min(1e7, old * Math.exp(-Math.max(-200, Math.min(200, event.deltaY)) * .0015)));
          const ratio = this.view.scale / old;
          this.view.pan[0] = (this.view.pan[0] - (x - this.canvas.width / 2)) * ratio + x - this.canvas.width / 2;
          this.view.pan[1] = (this.view.pan[1] - (y - this.canvas.height / 2)) * ratio + y - this.canvas.height / 2;
          linkView(this); this.schedule();
        }, { passive: false });
      }
    }

    function fitShared(previous) {
      if (!panels.left || !panels.right) return;
      clearHover();
      const view = sharedView({ left: panels.left.points, right: panels.right.points }, {
        left: { width: panels.left.canvas.width, height: panels.left.canvas.height }, right: { width: panels.right.canvas.width, height: panels.right.canvas.height }
      }, previous);
      for (const side of SIDES) { panels[side].view = copyView(view); panels[side].schedule(); }
    }
    function linkView(panel) {
      if (!sameScale || !SIDES.includes(panel.name)) return;
      const other = panels[panel.name === "left" ? "right" : "left"];
      if (other) { other.view = copyView(panel.view); other.schedule(); }
    }
    function selectionChanged() {
      clearHover();
      result = null; $("result-section").hidden = true;
      if (options.onSelectionChange) options.onSelectionChange(store.state.pairs.length);
      $("manual-icp-link").hidden = true;
      $("manual-icp-flow").hidden = true;
      $("saved").hidden = true; $("request-status").textContent = "选点已更新；此前保存的文件不变。";
      clearError(); renderSelection(); Object.values(panels).concat(Object.values(imagePanels)).forEach(panel => panel.schedule());
    }
    function refreshDisplay() {
      clearHover();
      $("display-frame").value = level.enabled ? "level" : "raw";
      $("display-frame").disabled = isBusy() || !level.available;
      $("level-status").textContent = level.enabled ? "左云按水平参考显示（候选）；右面板保留右雷达原始姿态。" : level.available ? "正在显示两侧原始坐标，可切换水平参考。" : "当前场景未提供水平参考，显示原始点云。";
      $("picker-frame-note").textContent = "前视沿 +X 观察，画面左为 +Y、上为 +Z。" + (level.enabled ? "左面板采用水平候选，右面板仍为原始右系；同尺度不等于同一物理参考，叠加预览才应用求得的右→左变换。" : "两侧各自显示在原始坐标系中；相同坐标数值不代表同一物理位置。") + (sameScale ? "两图共用中心、米制缩放和观察角度；旋转、平移和缩放联动。" : "两图各自适应窗口，屏幕物体大小不能直接互比。") + "原生 Z 不是离地高度。表格 XYZ 和保存外参始终使用原始雷达坐标。";
      $("overlay-frame-note").textContent = (level.enabled ? "水平参考（候选）" : "左雷达原始坐标") + " · 拖动旋转 · Shift + 拖动平移 · 滚轮缩放";
      if (panels.left) { panels.left.points = level.cloud(scene.clouds.left); panels.left.resize(true); }
      if (sameScale && panels.left && panels.right) fitShared(panels.left.view);
      if (panels.overlay && result) { panels.overlay.points = level.overlay(scene.clouds, result.initial_T_left_right); panels.overlay.resize(true); }
    }
    function makeCell(row, value, className) { const cell = document.createElement("td"); cell.textContent = value; if (className) cell.className = className; row.append(cell); return cell; }
    function renderSelection() {
      if (!store) return;
      const state = store.state, tbody = $("pair-rows"); tbody.replaceChildren();
      const pointText = (side, id) => id === null ? "等待选择…" : indexText(side, id) + "  [" + xyzText(maps[side].get(id).xyz) + "]";
      const addRow = (pair, index, pending) => {
        const row = document.createElement("tr"); if (pending) row.className = "pending";
        makeCell(row, String(index + 1) + (pending ? "（待补齐）" : ""));
        SIDES.forEach(side => makeCell(row, pointText(side, pair[side + "_id"]), "point-cell"));
        makeCell(row, !pending && result ? (result.residuals_m[index] * 1000).toFixed(2) : "—");
        const cell = makeCell(row, ""), button = document.createElement("button"); button.type = "button"; button.textContent = pending ? "取消这对" : "删除"; button.disabled = isBusy();
        button.addEventListener("click", () => { if (isBusy()) return; if (pending) store.cancelPending(); else store.remove(index); selectionChanged(); });
        cell.append(button); tbody.append(row);
      };
      state.pairs.forEach((pair, index) => addRow(pair, index, false));
      if (hasPending(state)) addRow(state.pending, state.pairs.length, true);
      if (!state.pairs.length && !hasPending(state)) { const row = document.createElement("tr"), cell = makeCell(row, "尚未选点。切换到“选点”，先在任意一侧点击角点，再在另一侧选择同一物理角点。", "empty-row"); cell.colSpan = 5; tbody.append(row); }
      $("pair-count").textContent = state.pairs.length + " 对" + (hasPending(state) ? " · 1 对待补齐" : "");
      const readiness = solveReadiness(state, maps);
      $("selection-hint").textContent = hasPending(state) ? readiness.reason + " 再点同一侧可替换当前未完成点。" : readiness.reason;
      $("calculate").disabled = isBusy() || !readiness.ok || !supports(scene,"calculate"); $("solve").disabled = isBusy() || !readiness.ok; $("export").disabled = isBusy() || !state.pairs.length || hasPending(state);
      $("undo").disabled = isBusy() || !store.history.length; $("clear").disabled = isBusy() || (!state.pairs.length && !hasPending(state));
      $("mode-browse").disabled = isBusy(); $("mode-pick").disabled = isBusy();
      $("display-frame").disabled = isBusy() || !level.available;
    }
    function setMode(next) {
      if (isBusy()) return; clearHover(); mode = next; document.body.classList.toggle("picking", mode === "pick");
      for (const name of ["browse", "pick"]) { $("mode-" + name).classList.toggle("active", mode === name); $("mode-" + name).setAttribute("aria-pressed", String(mode === name)); }
      $("mode-help").textContent = mode === "pick" ? "悬停虚线预览同侧点，点击才确认配对 · 图像拖动平移，点云拖动旋转 · 滚轮缩放" : "图像拖动平移 · 点云拖动旋转，Shift + 拖动平移 · 滚轮缩放";
      Object.values(panels).forEach(panel => panel.schedule());
    }
    async function request(kind) {
      if (!scene || isBusy()) return;
      const state = store.state;
      const calculating = kind === "calculate" || kind === "solve", saveRequested = kind !== "calculate";
      if (!saveRequested && !supports(scene,"calculate")) { error("当前仍连接旧服务，只计算功能不可用。请打开新版服务地址；现有选点仍保留在此页面。"); return; }
      if (hasPending(state) || !state.pairs.length || (calculating && !solveReadiness(state, maps).ok)) return;
      const submittedPairs = state.pairs.map(pair => ({ ...pair }));
      localBusy = true; clearHover(); if (options.onBusy) options.onBusy(true); clearError(); renderSelection(); $("request-status").textContent = kind === "calculate" ? "正在计算，不保存文件…" : kind === "solve" ? "正在计算并保存…" : "正在导出选点…";
      try {
        const response = await fetch("/api/" + kind, { method: "POST", credentials: "same-origin", headers: { "Content-Type": "application/json", "X-Picker-Token": scene.token },
          body: JSON.stringify({ input_hash: scene.input_hash, scene_id: scene.scene_id, pairs: submittedPairs }) });
        let payload; try { payload = await response.json(); } catch (_) { throw new Error("服务返回的内容不是 JSON；选点已保留。"); }
        if (!response.ok || payload.error) throw new Error(typeof payload.error === "string" ? payload.error : "服务请求失败（HTTP " + response.status + "），选点已保留。");
        const saved = savedResponse(payload, saveRequested);
        if (calculating) {
          const checked = validateResult(payload.result, submittedPairs.length);
          const overlayPoints = level.overlay(scene.clouds, checked.initial_T_left_right);
          result = checked; $("result-section").hidden = false;
          $("manual-icp-link").hidden = !!options.unified || !saved;
          $("manual-icp-flow").hidden = !!options.unified || !saved;
          renderAssessment($("picker-assessment"), payload.assessment);
          if (!panels.overlay) panels.overlay = new CloudPanel("overlay", overlayPoints, false);
          else { panels.overlay.points = overlayPoints; panels.overlay.resize(true); }
          $("result-status").textContent = (result.status === "CANDIDATE" ? "初值候选 · 未独立验证" : "残差未通过 · 未独立验证") + (saved ? " · 已保存" : " · 未保存");
          $("rmse").textContent = "拟合 RMSE  " + (result.rmse_m * 1000).toFixed(2) + " mm";
          $("rotation").textContent = result.initial_T_left_right.slice(0, 3).map(row => row.slice(0, 3).map(v => v.toFixed(7).padStart(11)).join("  ")).join("\n");
          $("translation").textContent = "[ " + result.initial_T_left_right.slice(0, 3).map(row => row[3].toFixed(7)).join(", ") + " ]";
          $("result-reasons").textContent = Array.isArray(result.rejection_reasons) && result.rejection_reasons.length ? "后端原因：" + result.rejection_reasons.map(String).join("；") : "拟合结果仅约束这组人工对应点，不是精度验收。";
        }
        if (calculating && options.onResult) options.onResult(payload);
        $("export-path").textContent = saved ? payload.export_dir : ""; $("saved").hidden = !saved;
        $("saved-label").textContent = scene.source_mode === "real" ? "已保存到 Orin（服务端绝对路径）：" : "合成场景已保存到服务端（不是实机验收）：";
        $("request-status").textContent = !saved ? "计算完成，结果仅在本页面显示，尚未保存；满意后可选择“计算并保存”。" : kind === "solve" ? "初值及选点已保存到新版本目录。" : "选点已导出；未计算或启用外参。";
      } catch (e) { error(e.message); $("request-status").textContent = "请求未确认完成，当前选点已保留。"; }
      finally { localBusy = false; if (options.onBusy) options.onBusy(false); renderSelection(); }
    }

    $("mode-browse").addEventListener("click", () => setMode("browse"));
    $("view-scale-mode").addEventListener("change", () => {
      sameScale = $("view-scale-mode").value === "shared";
      if (!scene) return;
      if (sameScale) fitShared(panels.left.view); else SIDES.forEach(side => panels[side].fit("fit"));
      refreshDisplay();
    });
    $("display-frame").addEventListener("change", () => {
      if (!scene || isBusy()) return;
      try { level.setEnabled($("display-frame").value === "level"); refreshDisplay(); clearError(); }
      catch (e) { error(e.message); }
    });
    $("mode-pick").addEventListener("click", () => setMode("pick"));
    $("undo").addEventListener("click", () => { if (!isBusy() && store && store.undo()) selectionChanged(); });
    $("clear").addEventListener("click", () => { if (!isBusy() && store) { store.clear(); selectionChanged(); } });
    $("calculate").addEventListener("click", () => request("calculate"));
    $("solve").addEventListener("click", () => request("solve"));
    $("export").addEventListener("click", () => request("export"));
    $("overlay-fit").addEventListener("click", () => { if (panels.overlay) panels.overlay.fit("fit"); });
    scope.querySelectorAll(".view-tools button").forEach(button => button.addEventListener("click", () => {
      const panel = panels[button.parentElement.dataset.view]; if (panel) panel.fit(button.dataset.preset);
    }));
    document.addEventListener("keydown", event => {
      if (event.target.closest("input,textarea,[contenteditable=true]") || (options.unified && !scope.contains(event.target))) return;
      if (event.key.toLowerCase() === "z" && (event.ctrlKey || event.metaKey) && !event.altKey && !event.shiftKey && !isBusy() && store) { event.preventDefault(); if (store.undo()) selectionChanged(); }
    });
    const input = options.scene ? Promise.resolve(options.scene) : fetch("/api/scene", { credentials: "same-origin", cache: "no-store" })
      .then(async response => { const payload = await response.json(); if (!response.ok || payload.error) throw new Error(typeof payload.error === "string" ? payload.error : "读取场景失败。"); return payload; });
    const ready = input.then(validateScene).then(loaded => {
        scene = loaded; level = new LevelDisplay(scene.level_reference); store = new SelectionStore(scene.clouds);
        $("api-version-note").hidden = supports(scene,"calculate");
        maps = Object.fromEntries(SIDES.map(side => [side, new Map(scene.clouds[side].map(p => [p.id, p]))]));
        $("source-status").textContent = (scene.source_mode === "real" ? "真实点云" : "合成点云 · 仅界面/算法测试，不是实机数据") + " · 场景 " + scene.scene_id + " · 全部有效点直接显示，未抽稀";
        if (scene.time_quality === "NON_SIMULTANEOUS_STATIC_SCENE_ASSUMPTION") {
          $("source-status").textContent += "。左右分别采集：仅在两次采集间轮椅与场景未移动时用于外参初值，不能用于时间标定。";
        }
        $("source-status").classList.toggle("synthetic", scene.source_mode === "synthetic");
        $("source-details").textContent = safeMetadata({ scene_id: scene.scene_id, input_hash: scene.input_hash, source_mode: scene.source_mode, sensor_ids: scene.sensor_ids, units: scene.units, coordinate_conventions: scene.coordinate_conventions, time_quality: scene.time_quality, level_reference: scene.level_reference });
        for (const side of SIDES) { $(side + "-count").textContent = scene.clouds[side].length + " 个有效原始点"; $(side + "-title").title = scene.sensor_ids[side]; panels[side] = new CloudPanel(side, scene.clouds[side], true); }
        for (const side of SIDES) {
          const layout = scene.organized && scene.organized[side];
          if (layout && window.AmplitudeUI) {
            try {
              imagePanels[side] = new window.AmplitudeUI.ImagePanel(side, layout, scene.clouds[side], {
                state: () => store.state, canSelect: () => mode === "pick" && !isBusy(), error,
                hover: () => hoverFor(side), onHover: (selectedSide, id) => setHover(selectedSide, id, "image"), clearHover: () => clearHover(side, "image"),
                select: (selectedSide, id) => { try { if (store.select(selectedSide, id)) selectionChanged(); } catch (e) { error(e.message); } }
              });
              $(side + "-image-missing").hidden = true;
            } catch (e) {
              $(side + "-image-section").hidden = true;
              $(side + "-image-missing").textContent = "幅度图关联无效：" + e.message + " 原始三维选点仍可使用。";
            }
          }
        }
        if (Object.keys(imagePanels).length) setMode("pick");
        refreshDisplay(); renderSelection();
        if (options.onReady) options.onReady(scene);
        return scene;
      }).catch(e => { $("source-status").textContent = "场景读取失败，未启用选点。"; error(e.message); return null; });
    window.addEventListener("blur", () => clearHover());
    document.addEventListener("scroll", () => clearHover(), true);
    document.addEventListener("visibilitychange", () => { if (document.hidden) clearHover(); });
    return {ready, setExternalBusy(value) { externalBusy = !!value; if (externalBusy) clearHover(); if (store) renderSelection(); },
      getState() {return {scene, result, busy:isBusy(), pair_count:store ? store.state.pairs.length : 0, hover:hover.current ? {...hover.current} : null};}};
  }
  return { validateScene, newView, copyView, sharedView, savedResponse, supports, assessmentText, renderAssessment, projectPoint, fitView, rasterize, pickVisible, SelectionStore, HoverPreview, hoverEligible, hasPending, nonCollinear, solveReadiness, validateResult, transformPoint, validateRigid, validateLevelReference, composeRigid, inverseRigid, LevelDisplay, isPickGesture, start };
});
