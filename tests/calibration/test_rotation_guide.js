"use strict";
// Pure geometry only. Run: node tests/calibration/test_rotation_guide.js
const assert = require("node:assert/strict");
const alignment = require("../../src/wc_calibration/picker_assets/alignment.js");
const picker = require("../../src/wc_calibration/picker_assets/picker.js");
const RAD = Math.PI / 180;
let passed = 0;
function test(name, fn) { fn(); ++passed; process.stdout.write("PASS " + name + "\n"); }
function near(a, b, tolerance = 1e-9) {
  assert.ok(Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) <= tolerance,
    `${a} != ${b} (tolerance ${tolerance})`);
}
function vectorNear(a, b, tolerance = 1e-9) {
  assert.equal(a.length, b.length);
  a.forEach((value, i) => near(value, b[i], tolerance));
}
const add = (a, b) => a.map((v, i) => v + b[i]);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const times = (a, s) => a.map(v => v * s);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const move = (m, q) => m.slice(0, 3).map(row => dot(row.slice(0, 3), q) + row[3]);
// Independent axis-angle oracle: no Euler matrices or guide formulas.
function rodrigues(q, axis, angle) {
  return add(add(times(q, Math.cos(angle)), times(cross(axis, q), Math.sin(angle))),
    times(axis, dot(axis, q) * (1 - Math.cos(angle))));
}
const poses = [
  [1.2, -.4, 2.3, 26, -31, 47],
  [-2, .8, .13, -73, 58, -124],
  [.2, -.7, .6, 172, -84, 179],
  [0, 0, 0, -18, 89.9, 94]
];
const probes = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [.3, -.7, 1.2]];

test("zero-pose positive directions follow FLU, including pitch down", () => {
  const m = alignment.poseToMatrix([0, 0, 0, 0, 0, 0]);
  const cases = [
    { index: 3, axis: [1, 0, 0], from: [0, 1, 0], to: [0, 0, 1] },
    { index: 4, axis: [0, 1, 0], from: [1, 0, 0], to: [0, 0, -1] },
    { index: 5, axis: [0, 0, 1], from: [1, 0, 0], to: [0, 1, 0] }
  ];
  for (const c of cases) {
    const guide = alignment.rotationGuide(m, c.index);
    vectorNear(guide.axis, c.axis);
    vectorNear(rodrigues(c.from, guide.axis, Math.PI / 2), c.to);
    const pose = [0, 0, 0, 0, 0, 0]; pose[c.index] = 90;
    vectorNear(move(alignment.poseToMatrix(pose), c.from), c.to);
  }
});

test("compound-pose guide axes predict numerical derivatives of actual parameter changes", () => {
  const epsilon = 1e-5; // radians; central differences avoid a first-order bias.
  for (const input of poses) {
    const m = alignment.poseToMatrix(input), displayed = alignment.matrixToPose(m);
    for (const index of [3, 4, 5]) {
      const guide = alignment.rotationGuide(m, index);
      const plus = displayed.slice(), minus = displayed.slice();
      plus[index] += epsilon / RAD; minus[index] -= epsilon / RAD;
      const mPlus = alignment.poseToMatrix(plus), mMinus = alignment.poseToMatrix(minus);
      for (const q of probes) {
        const derivative = times(sub(move(mPlus, q), move(mMinus, q)), 1 / (2 * epsilon));
        vectorNear(derivative, cross(guide.axis, sub(move(m, q), guide.pivot)), 2e-8);
      }
    }
  }
});

test("finite positive and negative Euler steps equal rotation about the displayed pivot", () => {
  for (const input of poses) {
    const m = alignment.poseToMatrix(input), displayed = alignment.matrixToPose(m);
    for (const index of [3, 4, 5]) for (const degrees of [7.5, -17]) {
      const guide = alignment.rotationGuide(m, index), changed = displayed.slice();
      changed[index] += degrees;
      const next = alignment.poseToMatrix(changed);
      vectorNear(guide.pivot, move(m, [0, 0, 0]));
      vectorNear(move(next, [0, 0, 0]), guide.pivot);
      for (const q of probes) {
        const expected = add(guide.pivot, rodrigues(sub(move(m, q), guide.pivot), guide.axis, degrees * RAD));
        vectorNear(move(next, q), expected, 2e-10);
      }
    }
  }
});

test("guide arcs stay on a unit circle normal to their axes and turn the requested 270 degrees", () => {
  for (const pose of [[0, 0, 0, 0, 0, 0], ...poses]) for (const index of [3, 4, 5]) {
    for (const sign of [-1, 1]) {
      const guide = alignment.rotationGuide(alignment.poseToMatrix(pose), index, sign);
      near(Math.hypot(...guide.axis), 1);
      assert.ok(guide.arc.length >= 4);
      let angle = 0;
      guide.arc.forEach((q, i) => {
        near(Math.hypot(...q), 1); near(dot(q, guide.axis), 0);
        if (i > 0) {
          const previous = guide.arc[i - 1];
          const step = Math.atan2(dot(guide.axis, cross(previous, q)), dot(previous, q));
          assert.ok(sign * step > 0, "arc arrow must follow the selected sign");
          vectorNear(rodrigues(previous, guide.axis, step), q);
          angle += step;
        }
      });
      near(angle, sign * 1.5 * Math.PI);
    }
  }
});

test("front, top and side orthographic projections preserve their expected signed rotation", () => {
  const cases = [
    { name: "front roll", index: 3, yaw: 0, pitch: 0, signedArea: 1, sight: [1, 0, 0] },
    { name: "top yaw", index: 5, yaw: 0, pitch: -Math.PI / 2, signedArea: -1, sight: [0, 0, 1] },
    { name: "side pitch", index: 4, yaw: Math.PI / 2, pitch: 0, signedArea: -1, sight: [0, 1, 0] }
  ];
  for (const c of cases) {
    const view = { ...picker.newView(), yaw: c.yaw, pitch: c.pitch, scale: 40 };
    const project = xyz => picker.projectPoint({ id: 0, xyz }, view, 400, 300);
    const center = project([0, 0, 0]);
    for (const sign of [-1, 1]) {
      const guide = alignment.rotationGuide(alignment.poseToMatrix([0, 0, 0, 0, 0, 0]), c.index, sign);
      const projected = guide.arc.map(project);
      for (let i = 1; i < projected.length; ++i) {
        const a = projected[i - 1], b = projected[i];
        const area = (a.x - center.x) * (b.y - center.y) - (a.y - center.y) * (b.x - center.x);
        assert.ok(area * c.signedArea * sign > 0, c.name + " changed screen direction");
      }
    }
    // No perspective shrink: moving along the sight line cannot change screen XY.
    const q = [.3, -.7, 1.2], nearPoint = project(q), farPoint = project(add(q, times(c.sight, 20)));
    vectorNear([nearPoint.x, nearPoint.y], [farPoint.x, farPoint.y]);
  }
});

test("gimbal-lock guides remain finite and agree with the displayed Euler branch", () => {
  for (const pitch of [-90, 90]) {
    const m = alignment.poseToMatrix([.5, -.7, 1.3, 36, pitch, -28]);
    const displayed = alignment.matrixToPose(m);
    for (const index of [3, 4, 5]) for (const sign of [-1, 1]) {
      const guide = alignment.rotationGuide(m, index, sign);
      assert.ok([...guide.axis, ...guide.pivot, ...guide.arc.flat()].every(Number.isFinite));
      near(Math.hypot(...guide.axis), 1);
      const changed = displayed.slice(); changed[index] += sign * .1;
      for (const q of probes) {
        const expected = add(guide.pivot, rodrigues(sub(move(m, q), guide.pivot), guide.axis, sign * .1 * RAD));
        vectorNear(move(alignment.poseToMatrix(changed), q), expected, 2e-10);
      }
    }
  }
});

test("guide construction does not mutate the matrix or alias its returned data", () => {
  const m = alignment.poseToMatrix(poses[0]), saved = JSON.stringify(m);
  m.forEach(Object.freeze); Object.freeze(m);
  for (const index of [3, 4, 5]) for (const sign of [-1, 1]) {
    const guide = alignment.rotationGuide(m, index, sign);
    guide.pivot[0] += 100; guide.axis[0] += 100; guide.arc[0][0] += 100;
    assert.equal(JSON.stringify(m), saved);
    const fresh = alignment.rotationGuide(m, index, sign);
    vectorNear(fresh.pivot, move(m, [0, 0, 0])); near(Math.hypot(...fresh.axis), 1);
  }
});

process.stdout.write(`\n${passed} rotation guide geometry tests passed (no DOM / no hardware).\n`);
