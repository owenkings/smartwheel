"use strict";
const assert = require("node:assert/strict");
const P = require("../../src/wc_calibration/picker_assets/picker.js");
let passed = 0;
function test(name, fn) { fn(); passed++; console.log("PASS " + name); }
test("new calculation actions require an explicit server capability", () => {
  assert.equal(P.supports({}, "calculate"), false);
  assert.equal(P.supports({ capabilities: { calculate: "true" } }, "calculate"), false);
  assert.equal(P.supports({ capabilities: { calculate: true } }, "calculate"), true);
  assert.equal(P.supports({ capabilities: { calculate: true } }, "alignment_preview"), false);
});
test("preview cannot claim a saved path and saved requests require evidence", () => {
  assert.equal(P.savedResponse({ saved: false, export_dir: null }, false), false);
  assert.throws(() => P.savedResponse({ saved: true, export_dir: "/tmp/output" }, false));
  assert.throws(() => P.savedResponse({ saved: false, export_dir: null }, true));
  assert.equal(P.savedResponse({ saved: true, export_dir: "/data/result" }, true), true);
  assert.equal(P.savedResponse({ export_dir: "/data/legacy" }, true), true);
});
test("assessment does not turn missing or exploratory metrics into formal eligibility", () => {
  assert.equal(P.assessmentText(null).level, "NOT_EVALUATED");
  const a = P.assessmentText({ level: "EXPLORATORY", summary: "可继续", details: ["独立场景未检验"], usable_for: ["离线叠加"], formal_use: false });
  assert.equal(a.title, "可继续离线探索"); assert.equal(a.usable, "离线叠加");
  assert.match(a.formal, /不能/);
  assert.throws(() => P.assessmentText({ level: "EXPLORATORY", formal_use: true }));
  assert.equal(P.assessmentText({ level: "REJECTED", formal_use: false }).title, "当前结果不宜使用");
});
test("common fit uses one center and meter scale without changing cloud coordinates", () => {
  const clouds = { left: [{ id: 0, xyz: [1, -2, -1] }, { id: 1, xyz: [3, 2, 1] }], right: [{ id: 0, xyz: [2, -.2, -.1] }, { id: 1, xyz: [2, .2, .1] }] };
  const before = JSON.stringify(clouds), sizes = { left: { width: 800, height: 500 }, right: { width: 600, height: 400 } };
  const camera = P.sharedView(clouds, sizes), left = P.copyView(camera), right = P.copyView(camera);
  assert.deepEqual(left, right); assert.notEqual(left.center, right.center); assert.notEqual(left.pan, right.pan);
  const a = { id: 0, xyz: [2, 0, 0] }, b = { id: 1, xyz: [2, 0, 1] };
  const leftDelta = P.projectPoint(a, left, 800, 500).y - P.projectPoint(b, left, 800, 500).y;
  const rightDelta = P.projectPoint(a, right, 600, 400).y - P.projectPoint(b, right, 600, 400).y;
  assert.equal(leftDelta, rightDelta); assert.equal(leftDelta, camera.scale);
  for (const side of ["left", "right"]) for (const point of clouds[side]) {
    const p = P.projectPoint(point, camera, sizes[side].width, sizes[side].height);
    assert.ok(p.x > 0 && p.x < sizes[side].width && p.y > 0 && p.y < sizes[side].height);
  }
  assert.equal(JSON.stringify(clouds), before);
});
test("linked perspective retains common camera parameters through rotation and zoom", () => {
  const view = { ...P.newView(), yaw: .7, pitch: -.3, center: [2, 3, 4], scale: 140, pan: [23, -17] };
  const linked = P.copyView(view);
  for (const xyz of [[0, 0, 0], [5, -.4, 1.2]]) assert.deepEqual(P.projectPoint({ id: 2, xyz }, view, 600, 400), P.projectPoint({ id: 2, xyz }, linked, 600, 400));
});
console.log(`${passed} calculation and view-scale UI checks passed (pure functions / no hardware).`);
