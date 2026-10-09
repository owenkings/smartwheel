"use strict";
// Pure coordinate-contract checks. Run: node tests/calibration/test_level_display.js
const assert = require("node:assert/strict");
const picker = require("../../src/wc_calibration/picker_assets/picker.js");
const alignment = require("../../src/wc_calibration/picker_assets/alignment.js");
let passed = 0;
const clone = value => JSON.parse(JSON.stringify(value));
function test(name, fn) { fn(); ++passed; process.stdout.write("PASS " + name + "\n"); }
function near(a, b, tolerance = 1e-9) { assert.ok(Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) < tolerance, `${a} != ${b}`); }
function vectorNear(a, b, tolerance) { assert.equal(a.length, b.length); a.forEach((x, i) => near(x, b[i], tolerance)); }
function matrixNear(a, b) { a.forEach((row, i) => vectorNear(row, b[i])); }
const distance = (a, b) => Math.hypot(...a.map((v, i) => v - b[i]));
const identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
// Exact quarter turns and nonzero origins detect multiplication order mistakes.
const L = [[0, 0, 1, .4], [0, 1, 0, -.2], [-1, 0, 0, .7], [0, 0, 0, 1]];
const T = [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]];
const reference = () => ({ status: "CANDIDATE", T_level_left: clone(L), payload_sha256: "b".repeat(64), imu_provenance: { sensor_id: "fixture-imu" } });
const clouds = { left: [{ id: 3, xyz: [-2, 4, 8] }, { id: 170, xyz: [0, 0, 2] }, { id: 19999, xyz: [1, 2, 4] }],
  right: [{ id: 8, xyz: [2, 3, 5] }, { id: 101, xyz: [-2, 1, -1] }, { id: 599, xyz: [0, 0, 1] }] };

test("missing reference retains original display and rejects enabling unavailable leveling", () => {
  for (const value of [undefined, null]) {
    const level = new picker.LevelDisplay(value);
    assert.equal(level.available, false); assert.equal(level.enabled, false);
    assert.deepEqual(level.cloud(clouds.left), clouds.left);
    assert.deepEqual(level.toDisplay(T), T); assert.deepEqual(level.toRaw(T), T);
    assert.throws(() => level.setEnabled(true));
    level.setEnabled(false);
  }
});

test("reference accepts only finite rigid candidates without scale, shear or reflection", () => {
  const cases = [
    { ...reference(), status: "VALIDATED" }, { ...reference(), T_level_left: null },
    { ...reference(), T_level_left: [[1, 0, 0, 0], [0, 2, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]] },
    { ...reference(), T_level_left: [[1, .1, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]] },
    { ...reference(), T_level_left: [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]] }
  ];
  for (const value of [NaN, Infinity, "1"]) {
    const bad = reference(); bad.T_level_left[0][3] = value; cases.push(bad);
  }
  for (const bad of cases) assert.throws(() => new picker.LevelDisplay(bad));
  const level = new picker.LevelDisplay(reference());
  assert.equal(level.enabled, true); assert.equal(level.available, true);
});

test("shared leveling uses L times T, with translation, and converts saved output back to raw T", () => {
  const level = new picker.LevelDisplay(reference()), display = level.toDisplay(T);
  vectorNear(picker.transformPoint([2, 3, 5], display), [8.4, 3.8, 2.7]);
  vectorNear(picker.transformPoint([-2, 4, 8], L), [8.4, 3.8, 2.7]);
  matrixNear(level.toRaw(display), T);
  matrixNear(picker.composeRigid(picker.inverseRigid(L), L), identity);
  matrixNear(picker.composeRigid(L, picker.inverseRigid(L)), identity);
  assert.ok(distance(picker.transformPoint([2, 3, 5], picker.composeRigid(T, L)), [8.4, 3.8, 2.7]) > 1);
});

test("display clouds preserve sparse original IDs, row order, provenance and source XYZ", () => {
  const original = clone(clouds), inputReference = reference(), savedReference = clone(inputReference);
  const level = new picker.LevelDisplay(inputReference), shown = level.cloud(clouds.left);
  assert.deepEqual(shown.map(p => p.id), [3, 170, 19999]);
  assert.deepEqual(clouds, original); assert.deepEqual(inputReference, savedReference);
  vectorNear(shown[0].xyz, [8.4, 3.8, 2.7]);
  shown[0].xyz[0] += 100; shown[0].id = 2;
  inputReference.T_level_left[0][3] += 100;
  assert.deepEqual(clouds, original);
  vectorNear(level.cloud(clouds.left)[0].xyz, [8.4, 3.8, 2.7]);
});

test("leveled picking resolves the original point ID and selection retains raw XYZ", () => {
  const level = new picker.LevelDisplay(reference()), displayed = level.cloud(clouds.left);
  const view = picker.fitView(displayed, 300, 260);
  const projected = displayed.map(p => picker.projectPoint(p, view, 300, 260));
  const raster = picker.rasterize(projected, 300, 260, 1.8), clicked = projected[0];
  const picked = picker.pickVisible(raster, clicked.x, clicked.y);
  assert.equal(picked.id, 3);
  const store = new picker.SelectionStore(clouds);
  store.select("left", picked.id); store.select("right", 8);
  assert.deepEqual(store.state.pairs, [{ left_id: 3, right_id: 8 }]);
  const rawMap = new Map(clouds.left.map(p => [p.id, p.xyz]));
  assert.deepEqual(rawMap.get(store.state.pairs[0].left_id), [-2, 4, 8]);
  assert.deepEqual(clouds.right[0].xyz, [2, 3, 5]);
});

test("common display transform preserves overlap and all Euclidean residuals", () => {
  const level = new picker.LevelDisplay(reference()), overlay = level.overlay(clouds, T);
  assert.deepEqual(overlay.map(p => [p.side, p.id]), [["left", 3], ["left", 170], ["left", 19999], ["right", 8], ["right", 101], ["right", 599]]);
  vectorNear(overlay[0].xyz, overlay[3].xyz);
  for (let i = 0; i < clouds.left.length; i++) {
    near(distance(clouds.left[i].xyz, picker.transformPoint(clouds.right[i].xyz, T)), distance(overlay[i].xyz, overlay[i + clouds.left.length].xyz));
  }
  level.setEnabled(false);
  vectorNear(level.overlay(clouds, T)[0].xyz, clouds.left[0].xyz);
});

test("switching display references leaves raw model, ICP comparison and reset state unchanged", () => {
  const level = new picker.LevelDisplay(reference()), model = new alignment.PoseState(T);
  const next = alignment.poseToMatrix([.7, -.3, 1.2, 12, -22, 37]);
  model.acceptRefinement({ status: "NOT_CONVERGED", live_eligible: false, initial_T_left_right: clone(T), refined_T_left_right: next }, T);
  const saved = JSON.stringify(model);
  for (let i = 0; i < 20; i++) {
    level.setEnabled(i % 2 === 0);
    matrixNear(level.toRaw(level.toDisplay(model.current)), next);
    assert.equal(JSON.stringify(model), saved);
  }
  model.reset(); assert.deepEqual(model.current, T);
});

test("editing displayed U produces the requested display pose while saving a distinct raw T", () => {
  const level = new picker.LevelDisplay(reference()), model = new alignment.PoseState(T);
  const desired = alignment.poseToMatrix([.8, -1.2, 2.3, -18, 37, 91]);
  model.set(level.toRaw(desired));
  matrixNear(level.toDisplay(model.current), desired);
  assert.notDeepEqual(model.current, desired);
  const q = [.2, 1.3, -.7];
  vectorNear(level.cloud([{ id: 7, xyz: q }], model.current)[0].xyz, picker.transformPoint(q, desired));
});

test("display-plane dragging maps back through L inverse and preserves screen displacement", () => {
  const level = new picker.LevelDisplay(reference()), q = [.2, .4, .9];
  for (const [yaw, pitch] of [[0, 0], [.7, -.4], [0, -Math.PI / 2]]) {
    const view = { ...picker.newView(), yaw, pitch, scale: 80 };
    const u = level.toDisplay(T), before = picker.projectPoint({ id: 9, xyz: picker.transformPoint(q, u) }, view, 400, 300);
    const movedRaw = level.toRaw(alignment.dragTranslation(u, 13, -9, view));
    const after = picker.projectPoint({ id: 9, xyz: level.cloud([{ id: 9, xyz: q }], movedRaw)[0].xyz }, view, 400, 300);
    near(after.x - before.x, 13); near(after.y - before.y, -9); near(after.depth, before.depth);
    for (let i = 0; i < 3; i++) vectorNear(movedRaw[i].slice(0, 3), T[i].slice(0, 3));
  }
});

test("direction guides use the same displayed U that the angle controls edit", () => {
  const level = new picker.LevelDisplay(reference()), u = level.toDisplay(T), pose = alignment.matrixToPose(u);
  const q = [.3, -.6, .8], initialPoint = picker.transformPoint(q, u), epsilon = 1e-5;
  for (const index of [3, 4, 5]) {
    const guide = alignment.rotationGuide(u, index), positive = pose.slice(), negative = pose.slice();
    positive[index] += epsilon * 180 / Math.PI; negative[index] -= epsilon * 180 / Math.PI;
    const p = level.cloud([{ id: 5, xyz: q }], level.toRaw(alignment.poseToMatrix(positive)))[0].xyz;
    const n = level.cloud([{ id: 5, xyz: q }], level.toRaw(alignment.poseToMatrix(negative)))[0].xyz;
    const v = initialPoint.map((x, i) => x - guide.pivot[i]), a = guide.axis;
    vectorNear(p.map((x, i) => (x - n[i]) / (2 * epsilon)), [a[1] * v[2] - a[2] * v[1], a[2] * v[0] - a[0] * v[2], a[0] * v[1] - a[1] * v[0]], 2e-8);
  }
});

process.stdout.write(`\n${passed} level display checks passed (synthetic / no DOM / no hardware).\n`);
