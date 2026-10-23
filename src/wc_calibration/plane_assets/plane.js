/* Offline plane workbench. Only the server validates and estimates extrinsics. */
(function (root, factory) {
  "use strict";
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root && root.document) {
    root.PlaneWorkbenchUI = api;
    if (root.document.readyState === "loading") root.document.addEventListener("DOMContentLoaded", () => api.start(root));
    else api.start(root);
  }
})(typeof window !== "undefined" ? window : null, function () {
  "use strict";
  const SIDES = ["left", "right"], AXES = { XY: [0, 1], XZ: [0, 2], YZ: [1, 2] };
  const finite = value => typeof value === "number" && Number.isFinite(value);
  const vector = value => Array.isArray(value) && value.length === 3 && value.every(finite);
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  function requireThat(condition, message) { if (!condition) throw new Error(message); }
  function validateState(state) {
    requireThat(state && state.offline_only === true, "服务必须明确为离线工作台。");
    requireThat(typeof state.token === "string" && state.token.length > 0 && Number.isSafeInteger(state.revision) && state.revision >= 0, "服务返回的会话令牌或版本无效。");
    requireThat(Array.isArray(state.poses) && state.poses.length <= 100, "姿态列表无效或过长。");
    const ids = new Set();
    for (const pose of state.poses) {
      requireThat(pose && typeof pose.id === "string" && !ids.has(pose.id), "姿态标识缺失或重复。"); ids.add(pose.id);
      requireThat(["train", "holdout"].includes(pose.role), "姿态必须明确为训练或独立验证。");
      requireThat(Number.isInteger(pose.width) && Number.isInteger(pose.height) && pose.width > 0 && pose.height > 0 && pose.width * pose.height <= 20000, "图像尺寸无效。");
      requireThat(["real", "synthetic"].includes(pose.source_mode), "姿态来源必须明确为实录或合成数据。");
      for (const side of SIDES) {
        requireThat(pose.amplitude && Array.isArray(pose.amplitude[side]) && pose.amplitude[side].length === pose.width * pose.height, "幅度图与原始行列不一致。");
        requireThat(pose.clouds && Array.isArray(pose.clouds[side]) && pose.clouds[side].length === pose.width * pose.height, "点云与原始行列不一致。");
        requireThat(pose.rois && (pose.rois[side] == null || isRoi(pose.rois[side], pose.width, pose.height)), "区域超出原始图像。");
      }
    }
    return state;
  }
  function isRoi(roi, width, height) {
    return Array.isArray(roi) && roi.length === 4 && roi.every(Number.isInteger) && roi[0] >= 0 && roi[1] >= 0 && roi[0] <= roi[2] && roi[1] <= roi[3] && roi[2] < height && roi[3] < width;
  }
  function pixelFromClient(point, rect, width, height) {
    requireThat(rect && finite(rect.left) && finite(rect.top) && rect.width > 0 && rect.height > 0 && point && finite(point.x) && finite(point.y), "无法读取框选坐标。");
    return { col: clamp(Math.floor((point.x - rect.left) / rect.width * width), 0, width - 1), row: clamp(Math.floor((point.y - rect.top) / rect.height * height), 0, height - 1) };
  }
  // Display columns are mirrored. API coordinates are inclusive RAW row/column indices.
  function rawRoiFromDisplayDrag(start, end, width, height) {
    requireThat(Number.isInteger(width) && Number.isInteger(height) && width > 0 && height > 0, "图像尺寸无效。");
    requireThat(start && end && [start.row, start.col, end.row, end.col].every(finite), "框选坐标无效。");
    const rows = [start.row, end.row].map(v => clamp(Math.floor(v), 0, height - 1)).sort((a, b) => a - b);
    const cols = [start.col, end.col].map(v => width - 1 - clamp(Math.floor(v), 0, width - 1)).sort((a, b) => a - b);
    return [rows[0], cols[0], rows[1], cols[1]];
  }
  function displayRectFromRaw(roi, width, height) {
    requireThat(isRoi(roi, width, height), "区域无效。");
    return { x: width - 1 - roi[3], y: roi[0], width: roi[3] - roi[1] + 1, height: roi[2] - roi[0] + 1 };
  }
  function mutationPayload(state, body) {
    requireThat(state && typeof state.token === "string" && Number.isSafeInteger(state.revision), "尚未取得会话版本。");
    requireThat(body && !Object.hasOwn(body, "token") && !Object.hasOwn(body, "revision"), "请求正文不能覆盖会话版本。");
    return { ...body, token: state.token, revision: state.revision };
  }
  function splitPoses(poses) {
    requireThat(Array.isArray(poses) && poses.every(p => p && ["train", "holdout"].includes(p.role)), "用途不明确，不能混入训练。");
    return { train: poses.filter(p => p.role === "train"), holdout: poses.filter(p => p.role === "holdout") };
  }
  function transformFromResult(result) {
    if (!result) return null;
    const matrix = result.initial_T_left_right || result.T_left_right || result.transform;
    if (Array.isArray(matrix) && matrix.length === 4 && matrix.every(row => Array.isArray(row) && row.length === 4 && row.every(finite))) return { R: matrix.slice(0, 3).map(row => row.slice(0, 3)), t: matrix.slice(0, 3).map(row => row[3]) };
    const R = result.R || (result.transform && result.transform.R), t = result.t || (result.transform && result.transform.t);
    return Array.isArray(R) && R.length === 3 && R.every(vector) && vector(t) ? { R, t } : null;
  }
  function transformPoint(point, transform) { return transform.R.map((row, i) => row.reduce((sum, value, j) => sum + value * point[j], transform.t[i])); }
  function percentile(values, q) { if (!values.length) return null; const sorted = values.slice().sort((a, b) => a - b); return sorted[Math.floor((sorted.length - 1) * q)]; }
  function samplePoints(cloud, limit = 10000) { const valid = cloud.filter(vector); const step = Math.max(1, Math.ceil(valid.length / limit)); return valid.filter((_, i) => i % step === 0); }
  function projectionBounds(left, right, view, width, height) {
    const axes = AXES[view]; requireThat(axes && width > 0 && height > 0, "投影视图无效。");
    const lo = [Infinity, Infinity], hi = [-Infinity, -Infinity];
    for (const cloud of [left, right]) for (const point of cloud) for (let j = 0; j < 2; j++) { const value = point[axes[j]]; lo[j] = Math.min(lo[j], value); hi[j] = Math.max(hi[j], value); }
    if (!finite(lo[0])) return null;
    return { axes, center: lo.map((v, i) => (v + hi[i]) / 2), scale: Math.min((width - 70) / Math.max(.1, hi[0] - lo[0]), (height - 70) / Math.max(.1, hi[1] - lo[1])) };
  }
  function start(win) {
    const doc = win.document, $ = id => doc.getElementById(id);
    let state = null, selected = null, busy = false, view = "XY", previewTransform = "initial", resizeObserver = null;
    const drags = { left: null, right: null };
    const fmt = (value, decimals = 2, suffix = "") => finite(value) ? value.toFixed(decimals) + suffix : "—";
    const mm = value => fmt(finite(value) ? value * 1000 : null, 2, " mm");
    const percent = value => fmt(finite(value) ? value * 100 : null, 1, "%");
    const reasonLabels = { INSUFFICIENT_TRAIN_POSES: "训练姿态不足", NORMAL_RANK_DEFICIENT: "平面法向不足以约束三个方向", ILL_CONDITIONED_NORMALS: "平面法向过于接近，几何约束较弱", SAME_FACE_NOT_CONFIRMED: "尚未确认两侧是同一板面", PLANE_FIT_REJECTED: "有平面质量检查未通过", HOLDOUT_REQUIRED: "缺少独立验证姿态", HOLDOUT_FAILED: "独立验证未通过", INSUFFICIENT_POINTS: "框内有效点不足", LOW_SUPPORT: "平面支持率不足", HIGH_RMSE: "平面残差过大", SMALL_SPAN: "框选区域空间跨度不足" };
    Object.assign(reasonLabels, { TOO_FEW_POINTS: "框内有效点不足", TOO_FEW_INLIERS: "平面内点不足", LOW_INLIER_FRACTION: "平面支持率不足，可能混入边缘或背景", INSUFFICIENT_TWO_DIMENSIONAL_SPAN: "框选平面的两个方向跨度不足", ALL_POINT_P95_LIMIT_EXCEEDED: "全部点 P95 残差过大", NO_SUPPORTED_PLANE: "框内未找到足够支持的平面", PLANE_ORIENTATION_UNDEFINED_AT_SENSOR: "平面方向相对雷达原点不明确", TOO_FEW_INDEPENDENT_TRAIN_CAPTURES: "独立训练录制不足，连续帧不算新姿态", TRAIN_NORMALS_RANK_DEFICIENT: "训练平面法向未覆盖三个独立方向", TRAIN_NORMALS_ILL_CONDITIONED: "训练平面法向过于接近，请增加倾斜方向", SAME_FACE_CONFIRMATION_REQUIRED: "尚未确认左右框选为同一块板的同一面", CAPTURE_ROLE_LEAKAGE: "同一录制不能同时用于训练和独立验证", INSUFFICIENT_STABLE_FRAME_PLANES: "录制内通过平面质量检查的帧不足", TEMPORAL_PLANE_VARIATION_EXCEEDED: "录制内平面位置或法向变化过大", INDEPENDENT_HOLDOUT_VALIDATION_NOT_PERFORMED: "独立验证未完成：需要至少 3 个法向独立的验证录制", ICP_NORMAL_EQUATIONS_ILL_CONDITIONED: "ICP 几何约束不足", ICP_TOTAL_CHANGE_LIMIT_EXCEEDED: "ICP 外参改变量超过门限", HOLDOUT_REGRESSION: "ICP 后独立验证残差明显增大" });
    Object.assign(reasonLabels, {
      INVALID_POINT_ARRAY: "点坐标数组无效", POINT_ARRAY_MUST_BE_N_BY_3: "点坐标数组需要三维坐标", NONFINITE_POINTS: "点坐标含无效数值",
      NO_OBSERVATIONS: "尚未导入可用的平面观测", INVALID_OBSERVATION: "平面观测记录无效", MISSING_CAPTURE_ID: "缺少原始录制标识",
      INVALID_ROLE: "训练或验证用途无效", DUPLICATE_OBSERVATION_ID: "平面观测标识重复", INVALID_PLANE: "平面记录无效",
      INVALID_PLANE_OBJECT: "平面记录格式无效", PLANE_FIT_NOT_PASS: "平面质量检查未通过", INVALID_PLANE_SHAPE: "平面法向或中心坐标格式无效",
      NONFINITE_PLANE: "平面参数含无效数值", ZERO_PLANE_NORMAL: "平面法向长度为零", PLANE_CENTROID_EQUATION_INCONSISTENT: "平面方程与中心坐标不一致",
      INCONSISTENT_REPEATED_POSE: "同一录制的重复观测不一致", IMPROPER_ROTATION: "旋转矩阵无效",
      TRAIN_PLANE_RESIDUAL_LIMIT_EXCEEDED: "训练姿态平面残差超过门限", HOLDOUT_PLANE_RESIDUAL_LIMIT_EXCEEDED: "独立验证姿态平面残差超过门限",
      CLOUD_MUST_BE_FINITE_N_BY_3: "框选点云必须包含有效的三维点", TOO_FEW_ROI_POINTS: "框选平面点不足以运行 ICP",
      PLANE_INITIALIZATION_NOT_CANDIDATE: "平面标定初值未通过候选检查", INVALID_INITIAL_TRANSFORM: "初始外参无效",
      TRANSFORM_MUST_BE_FINITE_4_BY_4: "外参需要有效的 4 × 4 矩阵", INVALID_HOMOGENEOUS_ROW: "外参矩阵最后一行无效", INVALID_PROPER_ROTATION: "外参旋转矩阵无效",
      INVALID_ROI_CLOUD: "框选点云无效", ROI_CLOUD_NOT_PLANE_INLIERS: "ICP 输入含不属于所拟合平面的点",
      PROVIDED_INITIAL_TRANSFORM_FAILS_PLANE_GATES: "提供的初始外参未通过平面残差检查",
      REFINED_PLANE_RESIDUAL_LIMIT_EXCEEDED: "ICP 后平面残差超过门限", REFINED_CLOUD_OVERLAP_LIMIT_EXCEEDED: "ICP 后点云覆盖不足"
    });
    const reasonText = value => {
      if (typeof value !== "string") return JSON.stringify(value);
      if (reasonLabels[value]) return reasonLabels[value];
      return value.split(":").map(part => {
        if (reasonLabels[part]) return reasonLabels[part];
        const pose = state && state.poses.find(p => p.id === part || p.capture_id === part);
        if (pose) return pose.label || pose.id;
        if (part === "left" || part === "right") return part === "left" ? "左雷达" : "右雷达";
        const gate = /^INSUFFICIENT_ICP_OVERLAP_AT_([0-9.]+)M$/.exec(part);
        if (gate) return `ICP 对应点覆盖不足（距离门限 ${gate[1]} m）`;
        const observation = /^OBSERVATION_([0-9]+)$/.exec(part);
        return observation ? `第 ${Number(observation[1]) + 1} 条平面观测` : part;
      }).join("：");
    };
    const activePose = () => state && state.poses.find(p => p.id === selected);
    const showError = message => { $("error").textContent = String(message); $("error").hidden = false; $("notice").hidden = true; };
    const showNotice = message => { $("notice").textContent = message; $("notice").hidden = false; $("error").hidden = true; };
    function element(tag, text, className) { const el = doc.createElement(tag); if (text != null) el.textContent = String(text); if (className) el.className = className; return el; }
    function reasons(parent, values) { if (!Array.isArray(values) || !values.length) return; const ul = element("ul", null, "reasons"); values.forEach(value => ul.append(element("li", reasonText(value)))); parent.append(ul); }
    function metrics(parent, pairs) { parent.replaceChildren(); pairs.forEach(([key, value]) => parent.append(element("dt", key), element("dd", value))); }
    function setBusy(value, message) { busy = value; $("app").setAttribute("aria-busy", String(value)); doc.body.classList.toggle("busy", value); $("connection-state").textContent = message || (value ? "正在计算，请稍候…" : "已连接 · 只读取已完成录制"); updateButtons(); }
    function updateButtons() {
      doc.querySelectorAll("[data-mutates]").forEach(button => { button.disabled = busy || !state; });
      const pose = activePose(), ready = !!pose;
      for (const side of SIDES) $("clear-" + side).disabled = busy || !pose || !pose.rois[side];
      $("fit-button").disabled = busy || !ready || !pose.rois.left || !pose.rois.right;
      $("remove-button").disabled = busy || !ready;
      $("active-role").disabled = busy || !ready;
      $("same-face-confirmed").disabled = busy || !ready;
      $("solve-button").disabled = busy || !state || splitPoses(state.poses).train.length < 3;
      $("solve-button").title = state && splitPoses(state.poses).train.length < 3 ? "请先导入至少 3 个训练姿态。" : "服务端会检查平面质量、同一面确认、法向秩及条件数。";
      $("icp-button").disabled = busy || !state || !state.result || state.result.icp_allowed !== true;
      $("save-button").disabled = busy || !state || !state.result;
      doc.querySelectorAll(".pose-entry").forEach(button => { button.disabled = busy; });
    }
    function acceptState(next) {
      validateState(next);
      if (state) requireThat(next.token === state.token && next.revision >= state.revision, "会话来源或版本发生变化，请重新打开本工作台。");
      state = next;
      if (!state.poses.some(p => p.id === selected)) selected = state.poses.length ? state.poses[state.poses.length - 1].id : null;
      render();
    }
    async function loadState() {
      setBusy(true, "正在读取已保存会话…");
      try { const response = await win.fetch("/api/state", { cache: "no-store", credentials: "same-origin" }); const payload = await response.json(); if (!response.ok) throw new Error(payload.error || "无法连接工作台。"); acceptState(payload); $("error").hidden = true; }
      catch (error) { showError(error.message + " 可以刷新本页重连；服务端已保留的框选不会因刷新丢失。"); }
      finally { setBusy(false, state ? null : "连接失败 · 刷新本页重试"); }
    }
    async function mutate(path, body, message) {
      if (busy || !state) return;
      const snapshot = state;
      $("notice").hidden = true; $("error").hidden = true; setBusy(true, message);
      try {
        const response = await win.fetch(path, { method: "POST", headers: { "Content-Type": "application/json", "X-Picker-Token": snapshot.token }, credentials: "same-origin", cache: "no-store", body: JSON.stringify(mutationPayload(snapshot, body)) });
        const payload = await response.json();
        if (!response.ok || payload.error) throw new Error(payload.error || "操作失败。");
        requireThat(payload.state, "服务未返回更新后的会话，不能确认操作完成。"); acceptState(payload.state);
        if (path !== "/api/save") $("save-path").hidden = true;
        if (payload.saved_path) { $("save-path").textContent = "已保存：" + payload.saved_path; $("save-path").hidden = false; showNotice("会话与报告已保存。正式外参未启用。"); }
        return payload;
      } catch (error) { showError(error.message + " 请检查当前页面状态后再操作；本页不会自动重试写入。"); }
      finally { setBusy(false); }
    }
    function drawAmplitude(side) {
      const canvas = $("amplitude-" + side), context = canvas.getContext("2d"), pose = activePose();
      $("empty-" + side).hidden = !!pose;
      if (!pose) { context.clearRect(0, 0, canvas.width, canvas.height); return; }
      if (canvas.width !== pose.width || canvas.height !== pose.height) { canvas.width = pose.width; canvas.height = pose.height; }
      const amplitude = pose.amplitude[side], cloud = pose.clouds[side], values = amplitude.filter((v, i) => finite(v) && vector(cloud[i]));
      const low = percentile(values, .02) ?? 0, high = percentile(values, .98) ?? 1;
      const image = context.createImageData(pose.width, pose.height);
      for (let row = 0; row < pose.height; row++) for (let col = 0; col < pose.width; col++) {
        const raw = row * pose.width + col, target = (row * pose.width + pose.width - 1 - col) * 4;
        const valid = finite(amplitude[raw]) && vector(cloud[raw]);
        const gray = valid ? Math.round(clamp((amplitude[raw] - low) / Math.max(1, high - low), 0, 1) * 255) : 0;
        image.data[target] = valid ? gray : 13; image.data[target + 1] = valid ? gray : 26; image.data[target + 2] = valid ? gray : 36; image.data[target + 3] = 255;
      }
      context.putImageData(image, 0, 0);
      const drag = drags[side], roi = drag ? rawRoiFromDisplayDrag(drag.start, drag.end, pose.width, pose.height) : pose.rois[side];
      if (roi) {
        const rect = displayRectFromRaw(roi, pose.width, pose.height);
        context.fillStyle = side === "left" ? "rgba(0,185,207,.17)" : "rgba(241,159,38,.18)";
        context.fillRect(rect.x, rect.y, rect.width, rect.height);
        context.strokeStyle = side === "left" ? "#23e7f3" : "#ffbd45"; context.lineWidth = .5; context.strokeRect(rect.x + .25, rect.y + .25, Math.max(.5, rect.width - .5), Math.max(.5, rect.height - .5));
      }
    }
    function renderFit(side, pose) {
      const container = $("fit-" + side); container.replaceChildren();
      const fit = pose && pose.fit && pose.fit[side];
      if (!fit) { container.append(element("p", pose ? "框选完成后点击“拟合左右平面”。" : "等待导入录制。", "muted")); return; }
      const passed = fit.status === "PASS";
      container.append(element("div", passed ? "平面质量通过" : "平面质量未通过", "fit-title " + (passed ? "pass-text" : "fail-text")));
      const dl = element("dl", null, "metrics");
      metrics(dl, [["有效点 / 平面内点", `${fit.point_count ?? "—"} / ${fit.inlier_count ?? "—"}`], ["平面支持率", percent(fit.inlier_fraction)], ["内点 RMSE / P95", `${mm(fit.rmse_m)} / ${mm(fit.p95_m)}`], ["全部点 RMSE / P95", `${mm(fit.all_rmse_m)} / ${mm(fit.all_p95_m)}`], ["空间跨度", Array.isArray(fit.span_m) ? fit.span_m.map(mm).join(" × ") : mm(fit.span_m)]]);
      container.append(dl); reasons(container, fit.reasons);
      if (fit.temporal_quality) {
        const temporal = fit.temporal_quality, temporalDl = element("dl", null, "metrics");
        metrics(temporalDl, [["同录制通过帧", `${temporal.pass_frames ?? "—"} / ${temporal.frames ?? "—"}`], ["帧间法向变化 P95", fmt(temporal.normal_angle_p95_deg, 3, "°")], ["帧间平面偏移 P95", mm(temporal.offset_p95_m)]]);
        container.append(element("p", "同录制稳定性（不能代替独立验证）", "help"), temporalDl);
      }
    }
    function renderPoses() {
      const list = $("pose-list"); list.replaceChildren();
      state.poses.forEach((pose, i) => {
        const button = element("button", null, "pose-entry" + (pose.id === selected ? " selected" : "")); button.type = "button"; button.setAttribute("aria-pressed", String(pose.id === selected));
        const name = element("span", `${String(i + 1).padStart(2, "0")}  ${pose.label || pose.id}`, "pose-name");
        button.append(name, element("span", pose.role === "holdout" ? "独立验证 · 不参与求解" : "参与标定", "pose-role"));
        const quality = element("span", null, "pose-quality");
        SIDES.forEach(side => { const fit = pose.fit && pose.fit[side], status = fit ? (fit.status === "PASS" ? "通过" : "未通过") : "待拟合"; quality.append(element("span", (side === "left" ? "左 " : "右 ") + status, "quality-pill" + (fit ? fit.status === "PASS" ? " pass" : " fail" : ""))); });
        button.append(quality); button.addEventListener("click", () => { if (busy) return; selected = pose.id; render(); }); list.append(button);
      });
      $("empty-list").hidden = state.poses.length > 0;
    }
    function renderResult() {
      const result = state.result, transform = transformFromResult(result);
      $("result-empty").hidden = !!result; $("result-content").hidden = !result;
      $("icp-eligibility").textContent = result && result.icp_allowed === true ? "此候选可用于离线 ICP 检查。ICP 结果仍需经过独立验证，不会自动启用。" : "当前尚不允许运行 ICP。请先解决平面质量、法向约束和标定候选的拒绝原因。";
      if (!result) return;
      const accepted = result.status === "CANDIDATE", assessment = $("result-assessment"); assessment.replaceChildren(); assessment.className = "assessment " + (accepted ? "pass" : "fail");
      assessment.append(element("h3", accepted ? "已生成离线候选" : "当前标定候选未通过检查"));
      assessment.append(element("p", result.independent_validation_performed ? "已使用留出的独立姿态进行检查，请结合下表的验证结果评估。" : "尚未进行独立姿态验证；当前结果不能作为正式外参。"));
      reasons(assessment, result.reasons); reasons(assessment, result.warnings);
      $("rotation").textContent = transform ? transform.R.map(row => row.map(v => v.toFixed(7).padStart(11)).join(" ")).join("\n") : "未生成有效旋转矩阵";
      $("translation").textContent = transform ? transform.t.map(v => v.toFixed(7)).join("\n") : "未生成有效平移";
      const split = splitPoses(state.poses), observation = result.observability || {};
      metrics($("solve-stats"), [["训练 / 独立验证姿态", `${split.train.length} / ${split.holdout.length}`], ["法向矩阵秩", observation.rank ?? "—"], ["法向矩阵条件数", fmt(observation.condition, 2)], ["训练法向 RMSE", fmt(result.train_summary && result.train_summary.normal_angle_rmse_deg, 3, "°")], ["训练平面偏移 RMSE", mm(result.train_summary && result.train_summary.offset_rmse_m)], ["独立验证", result.independent_validation_performed ? summaryLabel(result.holdout_summary) : "未进行完整验证"], ["正式启用", "未启用"]]);
      const rows = $("residual-rows"); rows.replaceChildren();
      (Array.isArray(result.per_pose_metrics) ? result.per_pose_metrics : []).forEach(metric => {
        const pose = state.poses.find(p => p.id === metric.id), tr = element("tr", null, metric.role === "holdout" ? "holdout" : "");
        const role = metric.role === "holdout" ? "独立验证" : metric.role === "train" ? "训练" : "用途未知";
        [pose ? pose.label || pose.id : metric.id, role, fmt(metric.normal_angle_deg, 3, "°"), mm(finite(metric.offset_m) ? Math.abs(metric.offset_m) : null), metric.status || summaryLabel(metric)].forEach((value, i) => tr.append(element("td", value, i === 1 ? "role-cell" : "")));
        rows.append(tr);
      });
      if (!rows.children.length) { const row = element("tr"), cell = element("td", "服务端未提供逐姿态残差。"); cell.colSpan = 5; row.append(cell); rows.append(row); }
    }
    function summaryLabel(summary) { if (!summary) return "—"; if (summary.passed === true) return "通过"; if (summary.passed === false) return "未通过"; if (summary.status) return String(summary.status); return "尚无通过结论"; }
    function drawCloud() {
      const canvas = $("cloud-preview"), rect = canvas.getBoundingClientRect(), dpr = Math.min(win.devicePixelRatio || 1, 2);
      const width = Math.max(200, Math.round(rect.width || 1000)), height = Math.max(180, Math.round(rect.height || 420));
      canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr); const context = canvas.getContext("2d"); context.scale(dpr, dpr); context.clearRect(0, 0, width, height);
      const pose = activePose(), icp = state && state.icp;
      const useRefined = previewTransform === "refined" && icp && icp.refined_T_left_right;
      const transform = useRefined ? transformFromResult({ T_left_right: icp.refined_T_left_right }) : transformFromResult(state && state.result);
      $("preview-transform-tools").hidden = !(icp && icp.refined_T_left_right);
      context.font = "12px Microsoft YaHei, sans-serif"; context.fillStyle = "#748fa1";
      if (!pose || !transform) { context.textAlign = "center"; context.fillText(pose ? "计算标定后查看当前姿态的点云叠加" : "导入一个静止姿态开始", width / 2, height / 2); return; }
      const left = samplePoints(pose.clouds.left), right = samplePoints(pose.clouds.right).map(p => transformPoint(p, transform));
      const bounds = projectionBounds(left, right, view, width, height); if (!bounds) return;
      const pixel = point => [width / 2 + (point[bounds.axes[0]] - bounds.center[0]) * bounds.scale, height / 2 - (point[bounds.axes[1]] - bounds.center[1]) * bounds.scale];
      context.strokeStyle = "#e5eef4"; context.lineWidth = 1; context.beginPath(); context.moveTo(30, height / 2); context.lineTo(width - 30, height / 2); context.moveTo(width / 2, 25); context.lineTo(width / 2, height - 25); context.stroke();
      [[left, "rgba(0,143,164,.7)"], [right, "rgba(195,116,14,.65)"]].forEach(([cloud, color]) => { context.fillStyle = color; for (const point of cloud) { const p = pixel(point); context.fillRect(p[0] - 1, p[1] - 1, 2, 2); } });
      context.fillStyle = "#718a9b"; context.textAlign = "left"; context.fillText(`${view[0]} / ${view[1]}（米）· 视图宽约 ${(width / bounds.scale).toFixed(2)} m`, 13, height - 13);
      const label = useRefined ? "ICP " + (icp.status === "CANDIDATE" ? "离线候选" : "未通过结果") : "平面标定初值";
      $("preview-caption").textContent = `${pose.label || pose.id} · ${label} · 左 ${left.length} 点 / 右 ${right.length} 点`;
    }
    function renderIcp() {
      const root = $("icp-result"), icp = state.icp; root.replaceChildren(); root.hidden = !icp; if (!icp) return;
      const assessment = element("div", null, "assessment " + ((icp.status === "CANDIDATE" || icp.accepted === true) ? "pass" : "fail"));
      assessment.append(element("h3", icp.status === "CANDIDATE" ? "ICP 已生成离线候选" : "ICP 微调未通过检查"));
      assessment.append(element("p", icp.independent_validation_performed ? (icp.independent_validation_passed ? "已完成独立验证，检查通过。结果仍为离线候选。" : "已完成独立验证，检查未通过。") : "尚未完成足够且法向独立的验证录制，不能宣称独立验证通过。"));
      reasons(assessment, icp.reasons); reasons(assessment, icp.warnings); root.append(assessment);
      const comparison = element("div", null, "icp-comparison");
      [["微调前", icp.initial_train_summary, icp.initial_holdout_summary], ["微调后", icp.refined_train_summary, icp.refined_holdout_summary]].forEach(([title, train, holdout]) => {
        const card = element("div"), dl = element("dl", null, "metrics"); card.append(element("h3", title));
        metrics(dl, [["训练点到面 RMSE", mm(train && train.point_to_plane_rmse_m)], ["训练平面偏移 RMSE", mm(train && train.offset_rmse_m)], ["训练法向 RMSE", fmt(train && train.normal_angle_rmse_deg, 3, "°")], ["训练最小覆盖率", percent(train && train.minimum_overlap)], ["验证平面偏移 RMSE", mm(holdout && holdout.offset_rmse_m)], ["验证法向 RMSE", fmt(holdout && holdout.normal_angle_rmse_deg, 3, "°")], ["验证最小覆盖率", percent(holdout && holdout.minimum_overlap)]]); card.append(dl); comparison.append(card);
      }); root.append(comparison);
      const extra = element("dl", null, "metrics"); metrics(extra, [["平移变化", mm(icp.change && icp.change.translation_m)], ["旋转变化", fmt(icp.change && icp.change.rotation_deg, 3, "°")], ["独立验证", icp.independent_validation_performed ? (icp.independent_validation_passed ? "通过" : "未通过") : "未完成完整验证"], ["正式启用", "未启用"]]); root.append(extra);
      const perPose = element("table"), header = element("thead"), headerRow = element("tr"), body = element("tbody");
      ["姿态 / 用途", "微调前偏移", "微调后偏移", "微调后法向", "微调后覆盖率"].forEach(text => headerRow.append(element("th", text))); header.append(headerRow);
      (icp.refined_per_pose_metrics || []).forEach(metric => {
        const before = (icp.initial_per_pose_metrics || []).find(m => m.id === metric.id), pose = state.poses.find(p => p.id === metric.id), row = element("tr", null, metric.role === "holdout" ? "holdout" : "");
        [ `${pose ? pose.label || pose.id : metric.id} · ${metric.role === "holdout" ? "独立验证" : "训练"}`, mm(before && before.absolute_offset_m), mm(metric.absolute_offset_m), fmt(metric.normal_angle_deg, 3, "°"), percent(metric.overlap) ].forEach(text => row.append(element("td", text))); body.append(row);
      });
      perPose.append(header, body); const scroller = element("div", null, "table-scroll"); scroller.append(perPose); root.append(scroller);
    }
    function render() {
      $("session-id").textContent = state.session_id || "—"; $("output-root").textContent = state.output_root || "—";
      $("pose-count").textContent = state.poses.length + " 个姿态"; $("revision-state").textContent = "会话版本 " + state.revision;
      $("synthetic-banner").hidden = !state.poses.some(p => p.source_mode === "synthetic"); renderPoses();
      const pose = activePose(); $("active-title").textContent = pose ? pose.label || pose.id : "等待导入录制";
      $("active-meta").textContent = pose ? `录制：${pose.capture_id || pose.id} · ${pose.width} × ${pose.height} 原始行列` : "选择平面内部，避开轮廓、反光和背景。";
      $("selection-tools").hidden = !pose; $("confirm-face-label").hidden = !pose;
      $("same-face-confirmed").checked = !!(pose && pose.same_face_confirmed); $("active-role").value = pose ? pose.role : "train";
      $("source-badge").hidden = !pose; $("source-badge").textContent = pose && pose.source_mode === "synthetic" ? "合成数据" : "实录数据"; $("source-badge").className = "badge" + (pose && pose.source_mode === "synthetic" ? " synthetic" : "");
      SIDES.forEach(side => {
        const roi = pose && pose.rois[side]; $("roi-" + side).textContent = roi ? `原始行 ${roi[0]}–${roi[2]} / 列 ${roi[1]}–${roi[3]} · ${(roi[2] - roi[0] + 1) * (roi[3] - roi[1] + 1)} 个像素` : "尚未框选";
        drawAmplitude(side); renderFit(side, pose);
      }); renderResult(); renderIcp(); updateButtons(); drawCloud();
    }
    $("import-form").addEventListener("submit", async event => {
      event.preventDefault(); const dataset = $("dataset").value.trim(), label = $("label").value.trim(), role = $("import-role").value;
      if (!dataset || !label || !["train", "holdout"].includes(role)) return;
      const oldIds = state ? new Set(state.poses.map(p => p.id)) : new Set();
      const response = await mutate("/api/import", { dataset, label, role }, "正在读取已完成录制并准备点云…");
      if (response) { const added = state.poses.find(p => !oldIds.has(p.id)); if (added) { selected = added.id; render(); } showNotice("录制已导入。请在左右图像中分别框选同一板面内部。"); }
    });
    $("fit-button").addEventListener("click", () => { const pose = activePose(); if (pose) mutate("/api/fit", { pose_id: pose.id, same_face_confirmed: $("same-face-confirmed").checked }, "正在检查左右平面质量…"); });
    $("active-role").addEventListener("change", () => { const pose = activePose(); if (pose) mutate("/api/role", { pose_id: pose.id, role: $("active-role").value }, "正在更新此姿态用途…"); });
    $("same-face-confirmed").addEventListener("change", () => { const pose = activePose(); if (pose && pose.rois.left && pose.rois.right) mutate("/api/fit", { pose_id: pose.id, same_face_confirmed: $("same-face-confirmed").checked }, "正在保存板面确认并检查平面…"); });
    $("remove-button").addEventListener("click", () => { const pose = activePose(); if (pose) mutate("/api/remove", { pose_id: pose.id }, "正在从本会话移除此姿态（保留原始录制）…"); });
    $("solve-button").addEventListener("click", () => mutate("/api/solve", {}, "正在计算多姿态标定并检查独立姿态…"));
    $("icp-button").addEventListener("click", () => mutate("/api/icp", {}, "正在运行离线 ICP 与独立验证…"));
    $("save-button").addEventListener("click", () => mutate("/api/save", {}, "正在保存标定会话与报告…"));
    SIDES.forEach(side => {
      const canvas = $("amplitude-" + side);
      const eventPixel = event => { const pose = activePose(); return pixelFromClient({ x: event.clientX, y: event.clientY }, canvas.getBoundingClientRect(), pose.width, pose.height); };
      canvas.addEventListener("pointerdown", event => {
        if (busy || !activePose() || event.button !== 0) return; event.preventDefault(); const point = eventPixel(event);
        drags[side] = { start: point, end: point, pointerId: event.pointerId, poseId: activePose().id }; canvas.setPointerCapture(event.pointerId); drawAmplitude(side);
      });
      canvas.addEventListener("pointermove", event => { const drag = drags[side]; if (!drag || drag.pointerId !== event.pointerId) return; drag.end = eventPixel(event); drawAmplitude(side); });
      canvas.addEventListener("pointerup", event => {
        const drag = drags[side], pose = activePose(); if (!drag || drag.pointerId !== event.pointerId || !pose) return;
        drag.end = eventPixel(event); const roi = rawRoiFromDisplayDrag(drag.start, drag.end, pose.width, pose.height); drags[side] = null;
        if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId); drawAmplitude(side);
        if (pose.id === drag.poseId) mutate("/api/roi", { pose_id: pose.id, side, roi }, "正在保留框选区域…");
      });
      canvas.addEventListener("pointercancel", event => { if (drags[side] && drags[side].pointerId === event.pointerId) { drags[side] = null; drawAmplitude(side); } });
      $("clear-" + side).addEventListener("click", () => { const pose = activePose(); if (pose) mutate("/api/roi", { pose_id: pose.id, side, roi: null }, "正在清除框选区域…"); });
    });
    doc.querySelectorAll("[data-view]").forEach(button => button.addEventListener("click", () => { view = button.dataset.view; doc.querySelectorAll("[data-view]").forEach(b => b.setAttribute("aria-pressed", String(b === button))); drawCloud(); }));
    $("preview-initial").addEventListener("click", () => { previewTransform = "initial"; $("preview-initial").setAttribute("aria-pressed", "true"); $("preview-refined").setAttribute("aria-pressed", "false"); drawCloud(); });
    $("preview-refined").addEventListener("click", () => { previewTransform = "refined"; $("preview-initial").setAttribute("aria-pressed", "false"); $("preview-refined").setAttribute("aria-pressed", "true"); drawCloud(); });
    if (win.ResizeObserver) { resizeObserver = new win.ResizeObserver(() => { if (state) drawCloud(); }); resizeObserver.observe($("cloud-preview")); }
    else win.addEventListener("resize", () => { if (state) drawCloud(); });
    loadState();
    return { loadState, getState: () => state, disconnect: () => { if (resizeObserver) resizeObserver.disconnect(); } };
  }
  return { validateState, isRoi, pixelFromClient, rawRoiFromDisplayDrag, displayRectFromRaw, mutationPayload, splitPoses, transformFromResult, transformPoint, samplePoints, projectionBounds, start };
});
