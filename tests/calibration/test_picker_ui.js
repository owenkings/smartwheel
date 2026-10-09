"use strict";
// No DOM, network, packages, or hardware. Run: node tests/calibration/test_picker_ui.js
const assert = require("node:assert/strict");
const ui = require("../../src/wc_calibration/picker_assets/picker.js");
let passed = 0;
function test(name, fn) { fn(); ++passed; process.stdout.write("PASS " + name + "\n"); }
const clone = value => JSON.parse(JSON.stringify(value));
const near = (actual, expected, tolerance = 1e-9) => assert.ok(Math.abs(actual - expected) < tolerance, actual + " != " + expected);
const cloud = [
  { id: 0, xyz: [1, 0, 0] }, { id: 3, xyz: [1, 1, 0] },
  { id: 9, xyz: [1, 0, 1] }, { id: 14, xyz: [2, 0, 0] }
];
function scene() { return { input_hash: "a".repeat(64), scene_id: "fixture", token: "test-token", source_mode: "synthetic", sensor_ids: { left: "lidar-left", right: "lidar-right" }, units: "m", coordinate_conventions: { left: "FLU", right: "FLU" }, time_quality: "UNVALIDATED", clouds: { left: clone(cloud), right: clone(cloud) } }; }
const identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
function result() { return { initial_T_left_right: clone(identity), residuals_m: [0, .001, .002], rmse_m: .0013, status: "CANDIDATE", live_eligible: false, units: "m" }; }
function makeStore() { return new ui.SelectionStore(scene().clouds); }
const maps = Object.fromEntries(["left", "right"].map(side => [side, new Map(cloud.map(p => [p.id, p]))]));

test("valid scene preserves sparse original indices and provenance", () => {
  const data = scene(); assert.equal(ui.validateScene(data), data); assert.deepEqual(data.clouds.left.map(p => p.id), [0, 3, 9, 14]);
});
test("finite coordinates are mandatory, with no string coercion", () => {
  for (const bad of [NaN, Infinity, -Infinity, "1", null]) { const data = scene(); data.clouds.left[0].xyz[0] = bad; assert.throws(() => ui.validateScene(data), /有限/); }
  const data = scene(); data.clouds.right[0].xyz.push(0); assert.throws(() => ui.validateScene(data));
});
test("IDs reject duplicates, arbitrary values and original-index overflow", () => {
  for (const id of [-1, 20000, .5, "3", Number.MAX_SAFE_INTEGER + 1]) { const data = scene(); data.clouds.left[0].id = id; assert.throws(() => ui.validateScene(data), /ID/); }
  const data = scene(); data.clouds.right[0].id = data.clouds.right[1].id; assert.throws(() => ui.validateScene(data), /ID/);
});
test("source and unit must be explicit, no millimetre guess", () => {
  for (const change of [{ units: "mm" }, { source_mode: "unknown" }, { token: "" }, { input_hash: "abc" }]) assert.throws(() => ui.validateScene({ ...scene(), ...change }));
  const data = scene(); data.sensor_ids.right = data.sensor_ids.left; assert.throws(() => ui.validateScene(data));
  const axes = scene(); axes.coordinate_conventions.right = "UNKNOWN"; assert.throws(() => ui.validateScene(axes), /FLU/);
});
test("supports the backend 20000 original-row budget, not just 9600", () => {
  const data = scene(); data.clouds.left = Array.from({ length: 20000 }, (_, id) => ({ id, xyz: [1, id / 20000, 0] }));
  assert.equal(ui.validateScene(data).clouds.left.length, 20000);
  data.clouds.left.push({ id: 20000, xyz: [1, 0, 0] }); assert.throws(() => ui.validateScene(data), /20000/);
});
test("front projection maps +Y left, +Z up and lower X nearer", () => {
  const view = ui.newView(); view.scale = 10;
  const p = ui.projectPoint({ id: 7, xyz: [2, 3, 4] }, view, 100, 100);
  assert.deepEqual(p, { id: 7, side: undefined, x: 20, y: 10, depth: 2 });
});
test("top view is from above, +X up and higher Z nearer", () => {
  const view = ui.newView(); view.pitch = -Math.PI / 2; view.scale = 10;
  const p = ui.projectPoint({ id: 1, xyz: [2, 3, 4] }, view, 100, 100);
  near(p.x, 20); near(p.y, 30); near(p.depth, -4);
});
test("projection obeys center and pan without changing source XYZ", () => {
  const point = { id: 7, xyz: [2, 3, 4] }, before = clone(point), view = ui.newView();
  view.center = [2, 3, 4]; view.pan = [8, -6];
  const p = ui.projectPoint(point, view, 100, 80); near(p.x, 58); near(p.y, 34); near(p.depth, 0); assert.deepEqual(point, before);
  assert.equal(ui.projectPoint({ id: 0, xyz: [NaN, 0, 0] }, view, 100, 80), null);
});
test("fit includes all points under rotation and handles a single-point extent", () => {
  const view = ui.newView(); view.yaw = .7; view.pitch = -.3;
  const fitted = ui.fitView(cloud, 700, 410, view);
  for (const point of cloud) { const p = ui.projectPoint(point, fitted, 700, 410); assert.ok(p.x > 0 && p.x < 700 && p.y > 0 && p.y < 410); }
  const singleton = ui.fitView([cloud[0]], 200, 100); assert.ok(Number.isFinite(singleton.scale) && singleton.scale > 0);
});
test("depth buffer picks the visible original point, never a hidden point", () => {
  const projected = [{ id: 10, x: 20, y: 20, depth: 5 }, { id: 83, x: 20, y: 20, depth: 1 }];
  const raster = ui.rasterize(projected, 50, 50);
  assert.equal(ui.pickVisible(raster, 20, 20).id, 83);
  assert.ok(!Array.from(raster.indices).includes(0));
});
test("nearest visible footprint within radius wins, empty space never creates XYZ", () => {
  const raster = ui.rasterize([{ id: 3, x: 15, y: 20, depth: 2 }, { id: 4, x: 30, y: 20, depth: 1 }], 80, 50);
  assert.equal(ui.pickVisible(raster, 17, 20, 5).id, 3);
  assert.equal(ui.pickVisible(raster, 60, 40, 5), null);
  assert.equal(ui.pickVisible(raster, -1, 20), null);
  assert.equal(ui.pickVisible(raster, NaN, 20), null);
});
test("partly occluded points can be chosen only where their own raster is visible", () => {
  const raster = ui.rasterize([{ id: 9, x: 20, y: 20, depth: 4 }, { id: 10, x: 22, y: 20, depth: 1 }], 50, 50, 2);
  assert.equal(ui.pickVisible(raster, 19.5, 20.5, .6).id, 9);
  assert.equal(ui.pickVisible(raster, 21.5, 20.5, .6).id, 10);
});
test("invalid raster dimensions and radii do not allocate unbounded buffers", () => {
  assert.throws(() => ui.rasterize([], 100000, 100000)); assert.throws(() => ui.rasterize([], 20, 20, Infinity));
  const r = ui.rasterize([{ id: 0, x: Infinity, y: 0, depth: 0 }], 20, 20); assert.equal(ui.pickVisible(r, 0, 0), null);
});
test("click selects only in select mode, not drag, Shift or secondary click", () => {
  assert.equal(ui.isPickGesture("pick", 3.9, false), true);
  assert.equal(ui.isPickGesture("browse", 0, false), false);
  assert.equal(ui.isPickGesture("pick", 4, false), false);
  assert.equal(ui.isPickGesture("pick", 0, true), false);
  assert.equal(ui.isPickGesture("pick", 0, false, 2), false);
  assert.equal(ui.isPickGesture("pick", NaN, false), false);
});
test("same unfinished side replaces pending point before matching", () => {
  const store = makeStore(); store.select("left", 0); store.select("left", 3);
  assert.deepEqual(store.state.pending, { left_id: 3, right_id: null });
  store.select("right", 9); assert.deepEqual(store.state.pairs, [{ left_id: 3, right_id: 9 }]); assert.equal(ui.hasPending(store.state), false);
});
test("either side can start a pair, and same pending click has no extra undo", () => {
  const store = makeStore(); store.select("right", 9); const n = store.history.length;
  assert.equal(store.select("right", 9), false); assert.equal(store.history.length, n);
  store.select("left", 3); assert.deepEqual(store.state.pairs, [{ left_id: 3, right_id: 9 }]);
});
test("cannot reuse a completed point or submit an absent original ID", () => {
  const store = makeStore(); store.select("left", 0); store.select("right", 0); const before = clone(store.state);
  assert.throws(() => store.select("left", 0), /已经/); assert.throws(() => store.select("right", 8), /原始点/); assert.throws(() => store.select("bogus", 3)); assert.deepEqual(store.state, before);
});
test("delete, cancel and clear are undoable and preserve other pairs", () => {
  const store = makeStore(); for (const id of [0, 3]) { store.select("left", id); store.select("right", id); }
  store.remove(0); assert.deepEqual(store.state.pairs, [{ left_id: 3, right_id: 3 }]); store.undo(); assert.equal(store.state.pairs.length, 2);
  store.select("left", 9); store.cancelPending(); assert.equal(ui.hasPending(store.state), false); store.undo(); assert.equal(store.state.pending.left_id, 9);
  const before = clone(store.state); store.clear(); assert.equal(store.state.pairs.length, 0); store.undo(); assert.deepEqual(store.state, before);
});
test("undo a completed pair restores the prior unfinished side", () => {
  const store = makeStore(); store.select("left", 0); store.select("right", 3); store.undo();
  assert.deepEqual(store.state, { pairs: [], pending: { left_id: 0, right_id: null } });
});
test("three noncollinear points may be coplanar; line and repeated points fail", () => {
  assert.equal(ui.nonCollinear([[0, 0, 0], [1, 0, 0], [0, 1, 0]]), true);
  assert.equal(ui.nonCollinear([[0, 0, 0], [1, 0, 0], [2, 0, 0]]), false);
  assert.equal(ui.nonCollinear([[1, 1, 1], [1, 1, 1], [1, 1, 1]]), false);
  assert.equal(ui.nonCollinear([[0, 0, 0], [1, 0, 0]]), false);
});
test("readiness requires complete pairs and both sides noncollinear", () => {
  const store = makeStore(); for (const id of [0, 3, 9]) { store.select("left", id); store.select("right", id); }
  assert.equal(ui.solveReadiness(store.state, maps).ok, true);
  store.select("left", 14); assert.equal(ui.solveReadiness(store.state, maps).ok, false);
});
test("result accepts a rigid candidate but never upgrades live eligibility", () => {
  const value = result(); assert.equal(ui.validateResult(value, 3), value);
  value.status = "UNVALIDATED"; assert.equal(ui.validateResult(value, 3), value);
  value.live_eligible = true; assert.throws(() => ui.validateResult(value, 3), /实时/);
});
test("result rejects nonfinite, mirrored, scaled and malformed transforms", () => {
  for (const bad of [NaN, Infinity]) { const value = result(); value.initial_T_left_right[0][3] = bad; assert.throws(() => ui.validateResult(value, 3)); }
  const mirror = result(); mirror.initial_T_left_right[0][0] = -1; assert.throws(() => ui.validateResult(mirror, 3), /镜像/);
  const scale = result(); scale.initial_T_left_right[0][0] = 2; assert.throws(() => ui.validateResult(scale, 3), /旋转/);
  const row = result(); row.initial_T_left_right[3][0] = 1; assert.throws(() => ui.validateResult(row, 3), /末行/);
});
test("residuals must match selected pair count and remain finite/nonnegative", () => {
  assert.throws(() => ui.validateResult(result(), 4));
  for (const bad of [-.1, NaN, Infinity, "0"]) { const value = result(); value.residuals_m[0] = bad; assert.throws(() => ui.validateResult(value, 3)); }
  const value = result(); value.rmse_m = NaN; assert.throws(() => ui.validateResult(value, 3));
});
test("preview applies right-to-left transform exactly once in metres", () => {
  const matrix = [[0, -1, 0, .6], [1, 0, 0, -.1], [0, 0, 1, .02], [0, 0, 0, 1]];
  const p = ui.transformPoint([1, 2, 3], matrix); near(p[0], -1.4); near(p[1], .9); near(p[2], 3.02);
  const checked = result(); checked.initial_T_left_right = matrix; ui.validateResult(checked, 3);
  assert.deepEqual(ui.transformPoint([1, 2, 3], identity), [1, 2, 3]);
});
process.stdout.write("\n" + passed + " picker UI pure-function tests passed (no DOM / no hardware).\n");
