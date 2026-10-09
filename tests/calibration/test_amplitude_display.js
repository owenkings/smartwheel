"use strict";
const assert = require("node:assert/strict");
const A = require("../../src/wc_calibration/picker_assets/amplitude.js");
const P = require("../../src/wc_calibration/picker_assets/picker.js");
const Align = require("../../src/wc_calibration/picker_assets/alignment.js");
const layout = { width: 4, height: 2, order: "row_major", amplitude: [0, 2, null, 4, 5, 6, 7, 8] };
const cloud = Array.from({ length: 8 }, (_, id) => ({ id, xyz: [id + 1, id / 2, id / 3] })).filter(p => p.id !== 2);
const points = new Map(cloud.map(p => [p.id, p]));
assert.equal(A.validate(layout, cloud), layout);
assert.throws(() => A.validate({ ...layout, amplitude: [1] }, cloud));
assert.throws(() => A.validate({ ...layout, order: "column_major" }, cloud));

// Same image pixel scale in X/Y, centered with letterbox. Display-left maps to raw-right.
let view = A.viewport(layout, 400, 300);
assert.equal(view.scale, 100); assert.equal(view.y, 50);
assert.equal(A.hitPixel(layout, view, 0, 50, points).id, 3);
assert.equal(A.hitPixel(layout, view, 399.999, 50, points).id, 0);
assert.equal(A.hitPixel(layout, view, 0, 249.999, points).id, 7);
assert.equal(A.hitPixel(layout, view, 400, 50, points), null);
assert.equal(A.hitPixel(layout, view, 10, 49.999, points), null);
assert.equal(A.hitPixel(layout, view, 10, 250, points), null);
const invalid = A.hitPixel(layout, view, 150, 100, points);
assert.equal(invalid.id, 2); assert.equal(invalid.point, null);
assert.deepEqual(A.eventPosition({ left: 10, top: 20, width: 200, height: 150 }, 400, 300, 160, 95), [300, 150]);

// Every displayed center maps back to the same raw ID after zoom, pan and canvas resize.
for (const [w, h, zoom, pan] of [[400, 300, 1, [0, 0]], [600, 300, 2, [70, -20]], [240, 200, 1.5, [-15, 5]]]) {
  view = A.viewport(layout, w, h, zoom, pan);
  for (let id = 0; id < 8; id++) {
    const center = A.pixelCenter(layout, view, id);
    if (center.x >= 0 && center.x < w && center.y >= 0 && center.y < h) assert.equal(A.hitPixel(layout, view, center.x, center.y, points).id, id);
  }
}

// Image and 3D routes select original IDs in one store; delete and undo update both labels.
const store = new P.SelectionStore({ left: cloud, right: cloud });
const original = JSON.stringify(cloud);
const idFromImage = A.hitPixel(layout, A.viewport(layout, 400, 200), 10, 10, points).id;
store.select("left", idFromImage);
assert.deepEqual(A.labels(store.state, "left"), [{ id: 3, label: "1?", pending: true }]);
store.select("right", 4); // Same store called by 3D.
assert.deepEqual(A.labels(store.state, "left"), [{ id: 3, label: "1", pending: false }]);
assert.deepEqual(A.labels(store.state, "right"), [{ id: 4, label: "1", pending: false }]);
store.remove(0); assert.equal(A.labels(store.state, "left").length, 0);
store.undo(); assert.equal(A.labels(store.state, "right")[0].id, 4);
assert.throws(() => store.select("left", invalid.id));
assert.equal(JSON.stringify(cloud), original, "Display operations must not alter raw XYZ");

assert.deepEqual(A.bounds([null, NaN]), [0, 1]);
assert.ok(A.bounds([5, 5])[1] > A.bounds([5, 5])[0]);
assert.equal(Align.manualInitialRequested({ initial_source: { kind: "saved_manual_initial" } }, "#initial=manual"), true);
assert.equal(Align.manualInitialRequested({ initial_source: { kind: "mechanical" } }, ""), false);
assert.throws(() => Align.manualInitialRequested({ initial_source: { kind: "mechanical" } }, "#initial=manual"), /未找到/);
console.log("amplitude display: reverse-index, edges, letterbox, zoom, invalid XYZ, selection sync passed");
