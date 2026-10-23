"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const ui = require("../../src/wc_calibration/plane_assets/plane.js");

test("display drag preserves inclusive rows and converts mirrored columns to raw indices", () => {
  assert.deepEqual(ui.rawRoiFromDisplayDrag({ row: 4, col: 20 }, { row: 14, col: 50 }, 160, 60), [4, 109, 14, 139]);
  assert.deepEqual(ui.rawRoiFromDisplayDrag({ row: 14, col: 50 }, { row: 4, col: 20 }, 160, 60), [4, 109, 14, 139]);
  assert.deepEqual(ui.rawRoiFromDisplayDrag({ row: 0, col: 0 }, { row: 59, col: 159 }, 160, 60), [0, 0, 59, 159]);
  assert.deepEqual(ui.rawRoiFromDisplayDrag({ row: -10, col: 200 }, { row: 99, col: -3 }, 160, 60), [0, 0, 59, 159]);
  assert.deepEqual(ui.rawRoiFromDisplayDrag({ row: 8, col: 35 }, { row: 8, col: 35 }, 160, 60), [8, 124, 8, 124]);
});
test("display rectangle includes both ends and round trips without a one-column shift", () => {
  const rect = ui.displayRectFromRaw([4, 109, 14, 139], 160, 60);
  assert.deepEqual(rect, { x: 20, y: 4, width: 31, height: 11 });
  assert.deepEqual(ui.rawRoiFromDisplayDrag({ row: rect.y, col: rect.x }, { row: rect.y + rect.height - 1, col: rect.x + rect.width - 1 }, 160, 60), [4, 109, 14, 139]);
  assert.throws(() => ui.displayRectFromRaw([0, 0, 60, 159], 160, 60));
});
test("pointer conversion uses actual resized image rectangle and clamps outside drags", () => {
  const rect = { left: 100, top: 50, width: 800, height: 300 };
  assert.deepEqual(ui.pixelFromClient({ x: 202, y: 122 }, rect, 160, 60), { row: 14, col: 20 });
  assert.deepEqual(ui.pixelFromClient({ x: 900, y: 350 }, rect, 160, 60), { row: 59, col: 159 });
  assert.deepEqual(ui.pixelFromClient({ x: 0, y: -50 }, rect, 160, 60), { row: 0, col: 0 });
  assert.throws(() => ui.pixelFromClient({ x: 2, y: 2 }, { left: 0, top: 0, width: 0, height: 0 }, 160, 60));
});
test("every mutation binds the current token and revision without caller overrides", () => {
  const state = { token: "bound-session-token", revision: 17 }, body = { pose_id: "p2", roi: [4, 109, 14, 139] };
  assert.deepEqual(ui.mutationPayload(state, body), { ...body, token: "bound-session-token", revision: 17 });
  assert.equal(Object.hasOwn(body, "token"), false);
  assert.throws(() => ui.mutationPayload(state, { revision: 1 }));
  assert.throws(() => ui.mutationPayload(state, { token: "other" }));
  assert.throws(() => ui.mutationPayload(null, {}));
});
test("independent holdout poses remain disjoint from training", () => {
  const poses = [{ id: "t1", role: "train" }, { id: "h1", role: "holdout" }, { id: "t2", role: "train" }];
  const split = ui.splitPoses(poses);
  assert.deepEqual(split.train.map(p => p.id), ["t1", "t2"]);
  assert.deepEqual(split.holdout.map(p => p.id), ["h1"]);
  assert.equal(split.train.some(p => split.holdout.includes(p)), false);
  assert.throws(() => ui.splitPoses([{ id: "unknown", role: null }]));
});
test("point-cloud rendering uses the backend right-to-left matrix and equal metric scale", () => {
  const transform = ui.transformFromResult({ initial_T_left_right: [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]] });
  assert.deepEqual(ui.transformPoint([4, 5, 6], transform), [-4, 6, 9]);
  const projection = ui.projectionBounds([[0, 0, 0]], [[2, 1, 0]], "XY", 470, 270);
  assert.equal(projection.scale, 200);
  assert.deepEqual(projection.center, [1, .5]);
  assert.equal(ui.samplePoints(Array.from({ length: 20000 }, (_, i) => [i, 1, 2])).length, 10000);
  assert.equal(ui.transformFromResult({ initial_T_left_right: [[0, 0]] }), null);
});
