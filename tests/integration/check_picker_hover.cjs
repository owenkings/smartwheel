/* Read-only browser QA: frozen scene, local pointer selections, no POST is allowed. */
"use strict";
const fs = require("node:fs"), path = require("node:path"), assert = require("node:assert/strict");
const { chromium } = require(process.env.WC_TEST_PLAYWRIGHT || "playwright");
const args = process.argv.slice(2);
const option = name => { const i = args.indexOf(name); return i < 0 ? undefined : args[i + 1]; };
const url = option("--url"), destination = option("--output"), legacyUrl = option("--legacy-url");
for (const address of [url, legacyUrl].filter(Boolean)) assert.ok(/^http:\/\/127\.0\.0\.1:\d+\/$/.test(address), "Use a reviewed loopback frozen-data server");
assert.ok(url && destination, "--url and --output are required");
const output = path.resolve(destination); fs.mkdirSync(output, { recursive: false });
const deferred = () => { let resolve; const promise = new Promise(r => { resolve = r; }); return { promise, resolve }; };
(async () => {
  const report = { status: "FAIL", source: "frozen_scene_read_only", hardware_started: false, posts: [], actions: [], skipped: [] };
  let browser, page, held = null;
  try {
    browser = await chromium.launch({ headless: true, executablePath: process.env.WC_TEST_CHROME });
    const context = await browser.newContext({ viewport: { width: 1440, height: 1100 }, deviceScaleFactor: 1 });
    const errors = [];
    context.on("page", p => p.on("pageerror", e => errors.push(String(e))));
    await context.addInitScript(() => {
      // Test-only observation of real canvas calls; no scene or controller is replaced.
      window.__hoverCanvasStrokes = Object.create(null);
      const p = CanvasRenderingContext2D.prototype;
      const begin = p.beginPath, arc = p.arc, stroke = p.stroke, fill = p.fillRect, put = p.putImageData;
      p.beginPath = function (...args) { this.__testLastArc = null; return begin.apply(this, args); };
      p.arc = function (x, y, radius, ...args) { this.__testLastArc = { x, y, radius }; return arc.call(this, x, y, radius, ...args); };
      p.stroke = function (...args) {
        if (/^(left|right)-(image|canvas)$/.test(this.canvas.id) && this.__testLastArc) {
          const rows = window.__hoverCanvasStrokes[this.canvas.id] || (window.__hoverCanvasStrokes[this.canvas.id] = []);
          rows.push({ ...this.__testLastArc, dash: this.getLineDash(), color: this.strokeStyle });
          if (rows.length > 200) rows.shift();
        }
        return stroke.apply(this, args);
      };
      p.fillRect = function (x, y, width, height) {
        if (x === 0 && y === 0 && width >= this.canvas.width && height >= this.canvas.height) window.__hoverCanvasStrokes[this.canvas.id] = [];
        return fill.call(this, x, y, width, height);
      };
      p.putImageData = function (...args) { window.__hoverCanvasStrokes[this.canvas.id] = []; return put.apply(this, args); };
    });
    await context.route("**/api/**", async route => {
      const request = route.request(), api = new URL(request.url()).pathname;
      if (request.method() === "POST") { report.posts.push(api); return route.abort("blockedbyclient"); }
      if (held && api === "/api/alignment-bootstrap" && !held.intercepted) {
        const gate = held; gate.intercepted = true;
        const response = await route.fetch(); gate.hit.resolve(); await gate.release.promise;
        return route.fulfill({ response });
      }
      return route.continue();
    });
    page = await context.newPage(); page.setDefaultTimeout(20000);
    await page.goto(url);
    await page.locator("#left-count").filter({ hasText: "个有效原始点" }).waitFor();
    const scene = await (await page.request.get(url + "api/scene")).json();
    assert.ok(scene.organized?.left && scene.organized?.right, "Primary fixture needs organized images");
    report.scene = { scene_id: scene.scene_id, input_hash: scene.input_hash, source_mode: scene.source_mode, sensor_ids: scene.sensor_ids };
    const pairText = () => page.locator("#pair-count").textContent();
    async function expectedPairs(count, pending = false) {
      await page.waitForFunction(({ count, pending }) => {
        const text = document.querySelector("#pair-count").textContent;
        return text.startsWith(count + " 对") && text.includes("待补齐") === pending;
      }, { count, pending });
    }
    async function noHover() {
      await page.waitForFunction(() => ["left", "right"].every(side => ["image", "canvas"].every(kind => !document.getElementById(side + "-" + kind).hasAttribute("data-hover-id"))));
    }
    async function hoverIs(side, id, origin) {
      await page.waitForFunction(({ side, id, origin }) => {
        const image = document.getElementById(side + "-image"), cloud = document.getElementById(side + "-canvas");
        return image.dataset.hoverId === String(id) && cloud.dataset.hoverId === String(id) && image.dataset.hoverOrigin === origin && cloud.dataset.hoverOrigin === origin;
      }, { side, id, origin });
      const other = side === "left" ? "right" : "left";
      assert.equal(await page.locator(`#${other}-image`).getAttribute("data-hover-id"), null, "No opposite-sensor correspondence may be inferred by hover");
      assert.equal(await page.locator(`#${other}-canvas`).getAttribute("data-hover-id"), null);
    }
    async function chooseSelect(id, value) {
      const native = page.locator("#" + id);
      if (await native.inputValue() === value) return;
      assert.equal(await native.isDisabled(), false, "Required display mode must be available");
      if (await page.locator("#" + id + "-trigger").count()) {
        const index = await native.evaluate((select, value) => [...select.options].findIndex(o => o.value === value), value);
        await page.locator("#" + id + "-trigger").click();
        await page.locator("#" + id + "-option-" + index).click();
      } else await native.selectOption(value);
      assert.equal(await native.inputValue(), value);
    }
    async function resetProjection() {
      if (await page.locator("#display-frame").isEnabled()) await chooseSelect("display-frame", "raw");
      await chooseSelect("view-scale-mode", "shared");
      await page.locator('.view-tools[data-view="left"] [data-preset="front"]').click();
    }
    async function canvasPosition(side, kind, point) {
      const canvas = page.locator(`#${side}-${kind}`); await canvas.scrollIntoViewIfNeeded();
      const box = await canvas.boundingBox(), size = await canvas.evaluate(c => ({ width: c.width, height: c.height }));
      return { x: box.x + point.x * box.width / size.width, y: box.y + point.y * box.height / size.height };
    }
    async function imagePosition(side, id) {
      const canvas = page.locator(`#${side}-image`); await canvas.scrollIntoViewIfNeeded();
      return canvas.evaluate((c, arg) => {
        const rect = c.getBoundingClientRect(), view = window.AmplitudeUI.viewport(arg.layout, c.width, c.height);
        const point = window.AmplitudeUI.pixelCenter(arg.layout, view, arg.id);
        return { x: rect.left + point.x * rect.width / c.width, y: rect.top + point.y * rect.height / c.height,
          rawCol: arg.id % arg.layout.width, rawRow: Math.floor(arg.id / arg.layout.width), displayCol: arg.layout.width - 1 - arg.id % arg.layout.width };
      }, { layout: scene.organized[side], id });
    }
    async function moveImage(side, id) { const p = await imagePosition(side, id); await page.mouse.move(p.x, p.y); return p; }
    async function clickImage(side, id) { const p = await imagePosition(side, id); await page.mouse.click(p.x, p.y); }
    async function screenshotHover(file, side, id, origin, cloudPoint = null) {
      // Element screenshots may scroll first, which correctly clears hover. Re-hover
      // only after the whole two-sensor grid is positioned inside the viewport.
      const grid = page.locator(".cloud-grid"); await grid.scrollIntoViewIfNeeded();
      await page.mouse.move(2, 2);
      if (origin === "cloud") { const p = await canvasPosition(side, "canvas", cloudPoint); await page.mouse.move(p.x, p.y); }
      else await moveImage(side, id);
      await hoverIs(side, id, origin);
      await grid.screenshot({ path: path.join(output, file) });
    }
    async function cloudTarget(side, excluded = []) {
      const canvas = page.locator(`#${side}-canvas`); await canvas.scrollIntoViewIfNeeded();
      const target = await page.evaluate(({ scene, side, excluded }) => {
        const ui = window.PickerUI, sizes = Object.fromEntries(["left", "right"].map(s => { const c = document.getElementById(s + "-canvas"); return [s, { width: c.width, height: c.height }]; }));
        const size = sizes[side], view = ui.sharedView(scene.clouds, sizes, ui.newView());
        const projected = scene.clouds[side].map(p => ui.projectPoint(p, view, size.width, size.height));
        const raster = ui.rasterize(projected, size.width, size.height);
        const candidates = projected.filter(p => p && p.x > 25 && p.y > 25 && p.x < size.width - 25 && p.y < size.height - 25);
        candidates.sort((a, b) => Math.hypot(a.x - size.width / 2, a.y - size.height / 2) - Math.hypot(b.x - size.width / 2, b.y - size.height / 2));
        for (const candidate of candidates) {
          const hit = ui.pickVisible(raster, candidate.x, candidate.y);
          if (hit && !excluded.includes(hit.id)) return { id: hit.id, x: candidate.x, y: candidate.y };
        }
        return null;
      }, { scene, side, excluded });
      assert.ok(target, "Need at least one depth-visible cloud target");
      return { ...target, screen: await canvasPosition(side, "canvas", target) };
    }
    await resetProjection(); await page.locator("#mode-pick").click(); await expectedPairs(0);
    const target = await cloudTarget("left");
    await page.mouse.move(target.screen.x, target.screen.y); await hoverIs("left", target.id, "cloud");
    assert.equal(await page.locator("#left-canvas").getAttribute("data-hover-state"), "visible");
    await page.waitForFunction(() => (window.__hoverCanvasStrokes["left-image"] || []).some(s => s.dash.length && Math.abs(s.radius - 10) < .1));
    const rawCol = target.id % scene.organized.left.width, rawRow = Math.floor(target.id / scene.organized.left.width);
    assert.equal(await page.locator("#left-image").getAttribute("data-hover-col"), String(rawCol));
    assert.equal(await page.locator("#left-image").getAttribute("data-hover-row"), String(rawRow));
    await expectedPairs(0); report.actions.push("real_depth_visible_3d_hover_links_same_sensor_raw_pixel_and_dashed_ring");
    await screenshotHover("01_cloud_to_pixel_hover.png", "left", target.id, "cloud", target);
    await page.mouse.move(2, 2); await noHover();

    const rightIds = scene.clouds.right.map(p => p.id);
    const rightId = rightIds.find(id => id % scene.organized.right.width >= Math.floor(scene.organized.right.width * .65)) ?? rightIds[0];
    const imagePoint = await moveImage("right", rightId); await hoverIs("right", rightId, "image");
    assert.equal(imagePoint.displayCol, scene.organized.right.width - 1 - imagePoint.rawCol);
    assert.equal(await page.locator("#right-image").getAttribute("data-hover-col"), String(imagePoint.rawCol));
    assert.ok(["visible", "occluded", "outside"].includes(await page.locator("#right-canvas").getAttribute("data-hover-state")));
    await expectedPairs(0); report.actions.push("pixel_hover_reverses_screen_column_back_to_original_right_id_without_cross_sensor_guess");
    await screenshotHover("02_pixel_to_cloud_hover.png", "right", rightId, "image");
    await page.mouse.move(2, 2); await noHover(); report.actions.push("pointer_leave_clears_both_linked_markers");

    let invalid = null;
    for (const side of ["left", "right"]) {
      const valid = new Set(scene.clouds[side].map(p => p.id)), layout = scene.organized[side];
      const id = Array.from({ length: layout.width * layout.height }, (_, id) => id).find(id => !valid.has(id));
      if (id !== undefined) { invalid = { side, id }; break; }
    }
    if (invalid) {
      await moveImage("left", target.id); await hoverIs("left", target.id, "image");
      await moveImage(invalid.side, invalid.id); await noHover(); await expectedPairs(0);
      await clickImage(invalid.side, invalid.id); await noHover(); await expectedPairs(0);
      await page.locator("#error").filter({ hasText: "没有有效的原始 XYZ" }).waitFor();
      report.invalid_pixel = invalid;
      report.actions.push("invalid_xyz_pixel_on_either_sensor_clears_hover_and_click_rejected_without_pair");
    } else report.skipped.push("invalid_pixel: fixture has no invalid XYZ pixel");
    const occluded = await page.evaluate(({ scene }) => {
      const ui = window.PickerUI, sizes = Object.fromEntries(["left", "right"].map(side => { const c = document.getElementById(side + "-canvas"); return [side, { width: c.width, height: c.height }]; }));
      const view = ui.sharedView(scene.clouds, sizes, ui.newView());
      for (const side of ["left", "right"]) {
        const size = sizes[side], projected = scene.clouds[side].map(p => ui.projectPoint(p, view, size.width, size.height));
        const raster = ui.rasterize(projected, size.width, size.height), visible = new Set();
        for (const index of raster.indices) if (index >= 0) visible.add(projected[index].id);
        const point = projected.find(p => p && p.x > 12 && p.y > 12 && p.x < size.width - 12 && p.y < size.height - 12 && !visible.has(p.id));
        if (point) return { side, id: point.id };
      }
      return null;
    }, { scene });
    if (occluded) {
      await moveImage(occluded.side, occluded.id); await hoverIs(occluded.side, occluded.id, "image");
      assert.equal(await page.locator(`#${occluded.side}-canvas`).getAttribute("data-hover-state"), "occluded");
      assert.match(await page.locator(`#${occluded.side}-readout`).textContent(), /遮挡|投影预览/);
      await expectedPairs(0); report.occluded_pixel = occluded;
      await screenshotHover("02b_occluded_pixel_projection.png", occluded.side, occluded.id, "image");
      report.actions.push("occluded_original_image_point_previews_projection_with_explicit_occlusion_and_no_pair");
    } else report.skipped.push("occluded_pixel: no fully occluded projected point in this fixture and view");
    await moveImage("left", target.id); await hoverIs("left", target.id, "image");
    const scrollBefore = await page.evaluate(() => window.scrollY);
    await page.evaluate(() => window.scrollBy(0, 80));
    await page.waitForFunction(before => window.scrollY !== before, scrollBefore);
    await noHover(); await expectedPairs(0);
    report.actions.push("document_scroll_clears_linked_hover_without_selecting");
    const blank = await page.evaluate(({ scene }) => {
      const ui = window.PickerUI, sizes = Object.fromEntries(["left", "right"].map(side => { const c = document.getElementById(side + "-canvas"); return [side, { width: c.width, height: c.height }]; }));
      const view = ui.sharedView(scene.clouds, sizes, ui.newView()), size = sizes.left;
      const raster = ui.rasterize(scene.clouds.left.map(p => ui.projectPoint(p, view, size.width, size.height)), size.width, size.height);
      for (const x of [4, 15, size.width - 15, size.width - 4]) for (const y of [4, 15, size.height - 15, size.height - 4]) if (!ui.pickVisible(raster, x, y)) return { x, y };
      return null;
    }, { scene });
    assert.ok(blank, "Scene projection needs empty canvas space");
    const beforeBlank = await cloudTarget("left"); await page.mouse.move(beforeBlank.screen.x, beforeBlank.screen.y); await hoverIs("left", beforeBlank.id, "cloud");
    const blankScreen = await canvasPosition("left", "canvas", blank); await page.mouse.move(blankScreen.x, blankScreen.y); await noHover();
    report.actions.push("empty_3d_space_has_no_preview");
    await page.locator("#mode-browse").click(); await moveImage("left", target.id); await noHover(); await clickImage("left", target.id); await expectedPairs(0);
    const browseTarget = await cloudTarget("left"); await page.mouse.move(browseTarget.screen.x, browseTarget.screen.y); await noHover();
    report.actions.push("browse_mode_does_not_create_hover_or_correspondences");
    await page.locator("#mode-pick").click();

    // A real drag changes the view only; release never commits a correspondence.
    const dragTarget = await cloudTarget("left"); await page.mouse.move(dragTarget.screen.x, dragTarget.screen.y); await hoverIs("left", dragTarget.id, "cloud");
    await page.mouse.down(); await page.mouse.move(dragTarget.screen.x + 35, dragTarget.screen.y + 25, { steps: 5 }); await page.mouse.up();
    await noHover(); await expectedPairs(0); await resetProjection();
    report.actions.push("cloud_drag_clears_preview_without_selection_commit");
    const pixelDrag = await moveImage("left", target.id); await hoverIs("left", target.id, "image");
    await page.mouse.down(); await page.mouse.move(pixelDrag.x + 24, pixelDrag.y + 18, { steps: 5 }); await page.mouse.up();
    await noHover(); await expectedPairs(0); await page.locator("#left-image-reset").click();
    report.actions.push("image_drag_clears_preview_without_selection_commit");

    await clickImage("left", target.id); await expectedPairs(0, true); await noHover();
    const beforePending = await page.locator("#pair-rows").textContent();
    await moveImage("right", rightId); await hoverIs("right", rightId, "image"); await expectedPairs(0, true);
    assert.equal(await page.locator("#pair-rows").textContent(), beforePending, "Hover must not complete a pending pair");
    await page.waitForFunction(() => (window.__hoverCanvasStrokes["left-image"] || []).some(s => !s.dash.length && Math.abs(s.radius - 7) < .1));
    await screenshotHover("03_pending_left_hover_right_no_pair.png", "right", rightId, "image");
    await clickImage("right", rightId); await expectedPairs(1); await noHover();
    const completePair = await page.locator("#pair-rows").textContent();
    await moveImage("left", target.id); await noHover();
    const unusedLeft = scene.clouds.left.find(p => p.id !== target.id).id;
    await moveImage("left", unusedLeft); await hoverIs("left", unusedLeft, "image");
    await page.waitForFunction(() => {
      const strokes = window.__hoverCanvasStrokes["left-image"] || [];
      return strokes.some(s => !s.dash.length && Math.abs(s.radius - 7) < .1) && strokes.some(s => s.dash.length && Math.abs(s.radius - 10) < .1);
    });
    assert.equal(await page.locator("#pair-rows").textContent(), completePair);
    report.actions.push("pending_hover_does_not_pair_click_pairs_selected_solid_rings_survive_new_dashed_preview");

    // GET-only latency exercises the real global busy path without running or saving a calculation.
    if (await page.locator("#load-saved-candidate").count()) {
      held = { intercepted: false, hit: deferred(), release: deferred() }; const gate = held;
      await page.locator("#load-saved-candidate").click();
      let timer;
      try { await Promise.race([gate.hit.promise, new Promise((_, reject) => { timer = setTimeout(() => reject(new Error("History GET was not intercepted")), 20000); })]); }
      finally { clearTimeout(timer); }
      try {
        assert.equal(await page.locator("#clear").isDisabled(), true, "Global busy disables picker edits");
        await noHover(); await moveImage("left", unusedLeft); await noHover();
        await clickImage("left", unusedLeft); await expectedPairs(1);
        assert.equal(await page.locator("#pair-rows").textContent(), completePair);
      } finally { gate.release.resolve(); held = null; }
      await page.waitForFunction(() => !document.querySelector("#clear").disabled);
      report.actions.push("read_only_delayed_history_get_busy_clears_hover_and_blocks_selection");
    } else report.skipped.push("busy: standalone legacy picker has no GET-only history action");
    await page.locator("#clear").click(); await expectedPairs(0); await noHover();
    await page.screenshot({ path: path.join(output, "04_final_clean_selection.png"), fullPage: true });
    if (legacyUrl) {
      const legacy = await context.newPage(); await legacy.goto(legacyUrl);
      await legacy.locator("#left-count").filter({ hasText: "个有效原始点" }).waitFor();
      const oldScene = await (await legacy.request.get(legacyUrl + "api/scene")).json();
      assert.ok(!oldScene.organized, "Legacy fixture should omit organized image metadata");
      for (const side of ["left", "right"]) {
        assert.equal(await legacy.locator(`#${side}-image-section`).isVisible(), false);
        assert.equal(await legacy.locator(`#${side}-image-missing`).isVisible(), true);
        assert.equal(await legacy.locator(`#${side}-canvas`).isVisible(), true);
      }
      await legacy.screenshot({ path: path.join(output, "05_legacy_3d_only.png"), fullPage: true });
      report.actions.push("legacy_scene_retains_3d_views_and_explicit_missing_image_notice");
    }
    assert.deepEqual(report.posts, [], "No POST, calculation, candidate write or hardware action is permitted");
    assert.deepEqual(errors, []); report.page_errors = errors; report.status = "PASS";
  } catch (error) {
    report.error = String(error.stack || error); process.exitCode = 1;
    if (page) { report.failure_text = await page.locator("body").innerText().catch(() => "unavailable"); await page.screenshot({ path: path.join(output, "failure.png"), fullPage: true }).catch(() => {}); }
  } finally {
    if (held) held.release.resolve(); if (browser) await browser.close();
    fs.writeFileSync(path.join(output, "result.json"), JSON.stringify(report, null, 2));
    console.log(JSON.stringify({ status: report.status, actions: report.actions, posts: report.posts, skipped: report.skipped, error: report.error }));
  }
})();
