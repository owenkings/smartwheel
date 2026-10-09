/* Source-bound hover preview: no DOM server, sensor, network or candidate writes. */
"use strict";
const assert = require("node:assert/strict");
const picker = require("../../src/wc_calibration/picker_assets/picker.js");
const image = require("../../src/wc_calibration/picker_assets/amplitude.js");
let count = 0;
function test(name, run) { run(); count++; process.stdout.write("PASS " + name + "\n"); }
const cloud = [0, 1, 2, 4, 5, 7].map(id => ({id, xyz:[2 + id / 10, id / 4, .1]}));
const fixture = () => ({store:new picker.SelectionStore({left:cloud, right:cloud}), hover:new picker.HoverPreview()});
const layout = {width:4, height:2, order:"row_major", amplitude:[1, 2, 3, null, 5, 6, null, 8]};

test("hover neither creates a pair nor changes selection history", () => {
  const {store, hover} = fixture(), before = JSON.stringify(store);
  assert.equal(hover.update(store, "left", 1, "cloud", "pick", false), true);
  assert.deepEqual(hover.forSide("left", store, "pick", false), {side:"left", id:1, origin:"cloud"});
  assert.equal(hover.forSide("right", store, "pick", false), null);
  assert.equal(JSON.stringify(store), before);
  assert.equal(hover.update(store, "left", 1, "cloud", "pick", false), false);
});

test("only valid available raw IDs and source view names are previewed", () => {
  const {store, hover} = fixture();
  for (const id of [-1, 3, NaN, Infinity, "1", null, 1.5]) {
    hover.update(store, "left", id, "image", "pick", false); assert.equal(hover.current, null);
  }
  hover.update(store, "unknown", 1, "image", "pick", false); assert.equal(hover.current, null);
  hover.update(store, "left", 1, "unknown", "pick", false); assert.equal(hover.current, null);
});

test("browse and busy states revoke preview eligibility", () => {
  const {store, hover} = fixture(); hover.update(store, "left", 1, "cloud", "pick", false);
  assert.equal(hover.forSide("left", store, "browse", false), null);
  assert.equal(hover.forSide("left", store, "pick", true), null);
  hover.update(store, "left", 1, "cloud", "browse", false); assert.equal(hover.current, null);
  hover.update(store, "left", 1, "cloud", "pick", true); assert.equal(hover.current, null);
});

test("pending second correspondence previews only the side under the pointer", () => {
  const {store, hover} = fixture(); store.select("left", 1);
  hover.update(store, "right", 5, "image", "pick", false);
  assert.deepEqual(hover.current, {side:"right", id:5, origin:"image"});
  assert.equal(hover.forSide("left", store, "pick", false), null);
  assert.deepEqual(store.state, {pairs:[], pending:{left_id:1, right_id:null}});
  // Hovering is independent of the original ID selected on the other lidar.
  assert.equal(picker.hoverEligible(store, "right", 1, "pick", false), true);
});

test("existing selected points keep their permanent marker; pending replacement remains allowed", () => {
  const {store, hover} = fixture(); store.select("left", 1);
  assert.equal(picker.hoverEligible(store, "left", 1, "pick", false), false);
  assert.equal(picker.hoverEligible(store, "left", 2, "pick", false), true);
  store.select("right", 5);
  assert.equal(picker.hoverEligible(store, "left", 1, "pick", false), false);
  assert.equal(picker.hoverEligible(store, "right", 5, "pick", false), false);
  hover.update(store, "left", 1, "cloud", "pick", false); assert.equal(hover.current, null);
});

test("selection eligibility is rechecked after state changes", () => {
  const {store, hover} = fixture(); hover.update(store, "left", 1, "cloud", "pick", false);
  store.select("left", 1); assert.equal(hover.forSide("left", store, "pick", false), null);
});

test("late leave from another source cannot erase the current source preview", () => {
  const {store, hover} = fixture(); hover.update(store, "right", 5, "image", "pick", false);
  assert.equal(hover.clear("left", "cloud"), false);
  assert.equal(hover.clear("right", "cloud"), false);
  assert.equal(hover.clear("right", "image"), true);
  assert.equal(hover.current, null); assert.equal(hover.clear(), false);
});

test("reversed image columns round-trip to the exact original ID under pan and zoom", () => {
  const points = new Map(cloud.map(point => [point.id, point]));
  const view = image.viewport(layout, 400, 200, 1.5, [-20, 5]);
  for (const id of [1, 2, 5]) {
    const location = image.pixelCenter(layout, view, id), hit = image.hitPixel(layout, view, location.x, location.y, points);
    assert.equal(hit.id, id); assert.equal(hit.row, Math.floor(id / 4)); assert.equal(hit.col, id % 4); assert.equal(hit.point.id, id);
  }
  const ordinary = image.viewport(layout, 400, 200);
  assert.ok(image.pixelCenter(layout, ordinary, 0).x > image.pixelCenter(layout, ordinary, 2).x);
  const missing = image.pixelCenter(layout, ordinary, 3);
  assert.equal(image.hitPixel(layout, ordinary, missing.x, missing.y, points).point, null);
  assert.equal(image.hitPixel(layout, ordinary, -1, 50, points), null);
});

test("image-origin preview cannot change the 3D depth hit-test", () => {
  const projected = [{id:1, x:25, y:25, depth:1}, {id:2, x:25, y:25, depth:4}];
  const raster = picker.rasterize(projected, 50, 50), {store, hover} = fixture();
  hover.update(store, "left", 2, "image", "pick", false);
  assert.equal(hover.current.id, 2); assert.equal(picker.pickVisible(raster, 25, 25).id, 1);
});

test("image draws separate dashed preview and permanent point marker, then removes stale preview", () => {
  const {store, hover} = fixture(); store.select("left", 1);
  hover.update(store, "left", 2, "cloud", "pick", false);
  const calls = [], ctx = new Proxy({}, {get(target, key) {
    if (!(key in target)) target[key] = (...args) => calls.push([key, ...args]); return target[key];
  }});
  const panel = Object.create(image.ImagePanel.prototype);
  Object.assign(panel, {side:"left", layout, zoom:1, pan:[0,0], canvas:{width:400, height:200, dataset:{}}, ctx, image:{},
    points:new Map(cloud.map(point => [point.id, point])), readout:{textContent:""},
    options:{state:()=>store.state, canSelect:()=>true, hover:()=>hover.forSide("left", store, "pick", false)}});
  panel.draw();
  assert.equal(panel.canvas.dataset.hoverId, "2"); assert.equal(panel.canvas.dataset.hoverRow, "0"); assert.equal(panel.canvas.dataset.hoverCol, "2");
  assert.ok(calls.some(call => call[0] === "setLineDash" && JSON.stringify(call[1]) === "[5,4]"));
  assert.ok(calls.some(call => call[0] === "arc" && call[3] === 7));
  assert.ok(calls.some(call => call[0] === "arc" && call[3] === 10));
  assert.match(panel.readout.textContent, /三维点 #2/);
  hover.clear(); calls.length = 0; panel.draw();
  assert.equal(panel.canvas.dataset.hoverId, undefined);
  assert.equal(calls.some(call => call[0] === "arc" && call[3] === 10), false);
  assert.ok(calls.some(call => call[0] === "arc" && call[3] === 7));
});
process.stdout.write(`${count} hover checks passed (software only).\n`);
