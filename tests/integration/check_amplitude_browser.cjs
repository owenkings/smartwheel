/* Canvas interaction QA against a frozen loopback dataset; never starts hardware. */
"use strict";
const fs = require("node:fs"), path = require("node:path"), assert = require("node:assert/strict");
const { chromium } = require(process.env.WC_TEST_PLAYWRIGHT || "playwright");
const math = require("../../src/wc_calibration/picker_assets/picker.js");
const argv = process.argv.slice(2);
function option(name) { const i = argv.indexOf(name); return i >= 0 ? argv[i + 1] : undefined; }
const url = option("--url"), directory = option("--output"), synthetic = argv.includes("--synthetic");
assert.ok(/^http:\/\/127\.0\.0\.1:\d+\/$/.test(url), "Use a reviewed loopback frozen-data server");
assert.ok(directory, "--output is required; put artifacts on configured USB storage");
const output = path.resolve(directory); fs.mkdirSync(output, { recursive: false });
(async () => {
  const result = { status: "FAIL", source: synthetic ? "synthetic" : "real_read_only", starts_devices: false, actions: [] };
  let browser;
  try {
    browser = await chromium.launch({ headless: true, executablePath: process.env.WC_TEST_CHROME });
    const context = await browser.newContext({ viewport: { width: 1440, height: 1100 }, deviceScaleFactor: 1 });
    const page = await context.newPage(), errors = [];
    context.on("page", p => p.on("pageerror", e => errors.push(String(e))));
    page.on("pageerror", e => errors.push(String(e)));
    const posts = [];
    await context.route("**/api/**", async route => {
      if (route.request().method() === "POST") {
        posts.push(route.request().url());
        if (!synthetic) return route.abort("blockedbyclient");
      }
      return route.continue();
    });
    await page.goto(url);
    await page.locator("#left-count").filter({ hasText: "个有效原始点" }).waitFor();
    const scene = await (await page.request.get(url + "api/scene")).json();
    assert.equal(scene.source_mode, synthetic ? "synthetic" : "real");
    assert.ok(scene.organized?.left && scene.organized?.right, "Fixture must contain actual organized metadata");
    result.scene = { scene_id: scene.scene_id, input_hash: scene.input_hash, source_mode: scene.source_mode,
      layouts: Object.fromEntries(["left", "right"].map(s => [s, { width: scene.organized[s].width, height: scene.organized[s].height }])) };
    for (const side of ["left", "right"]) await page.locator(`#${side}-image-section`).waitFor({ state: "visible" });
    const count = async n => page.waitForFunction(n => document.querySelector("#pair-count").textContent.startsWith(n + " 对"), n);
    const clear = async () => { if (await page.locator("#clear").isEnabled()) await page.locator("#clear").click(); await count(0); };
    async function imagePoint(side, id, zoom = 1, edge = false) {
      const canvas = page.locator(`#${side}-image`); await canvas.scrollIntoViewIfNeeded();
      const hit = await canvas.evaluate((c, arg) => {
        const r = c.getBoundingClientRect(), l = arg.layout;
        const v = window.AmplitudeUI.viewport(l, c.width, c.height, arg.zoom, [0, 0]);
        const p = window.AmplitudeUI.pixelCenter(l, v, arg.id);
        if (arg.edge) p.x = v.x + .15; // leftmost display column -> original last column
        return { x: r.left + p.x * r.width / c.width, y: r.top + p.y * r.height / c.height,
          inside: p.x >= 0 && p.x < c.width && p.y >= 0 && p.y < c.height };
      }, { layout: scene.organized[side], id, zoom, edge });
      assert.ok(hit.inside, "Requested pixel must be in the visible viewport");
      await page.mouse.click(hit.x, hit.y);
    }
    async function cloudPoint(side, id) {
      await page.locator("#view-scale-mode").selectOption("independent");
      if (await page.locator("#display-frame").isEnabled()) await page.locator("#display-frame").selectOption("raw");
      await page.locator(`.view-tools[data-view="${side}"] [data-preset="front"]`).click();
      const canvas = page.locator(`#${side}-canvas`); await canvas.scrollIntoViewIfNeeded();
      const box = await canvas.boundingBox(), size = await canvas.evaluate(c => ({ width: c.width, height: c.height }));
      const view = math.fitView(scene.clouds[side], size.width, size.height, math.newView());
      const p = math.projectPoint(scene.clouds[side].find(p => p.id === id), view, size.width, size.height);
      await page.mouse.click(box.x + p.x * box.width / size.width, box.y + p.y * box.height / size.height);
    }
    await page.screenshot({ path: path.join(output, "loaded.png"), fullPage: true });
    await page.locator("#mode-browse").click();
    const firstLeft = scene.clouds.left[0].id, firstRight = scene.clouds.right[0].id;
    await imagePoint("left", firstLeft);
    assert.equal((await page.locator("#pair-count").innerText()).trim(), "0 对");
    result.actions.push("browse_image_click_does_not_select");
    await page.locator("#mode-pick").click();
    if (synthetic) {
      const edgeId = scene.organized.left.width - 1;
      assert.ok(scene.clouds.left.some(p => p.id === edgeId));
      await imagePoint("left", edgeId, 1, true); await imagePoint("right", edgeId); await count(1);
      assert.match(await page.locator("#pair-rows").innerText(), new RegExp("#" + edgeId + "\\s"));
      result.actions.push("left_display_edge_maps_to_last_original_column");
      await clear();
      assert.ok(!scene.clouds.left.some(p => p.id === 6), "Synthetic invalid ID 6 must have no XYZ");
      await imagePoint("left", 6);
      await page.locator("#error").filter({ hasText: "没有有效的原始 XYZ" }).waitFor();
      assert.equal((await page.locator("#pair-count").innerText()).trim(), "0 对");
      result.actions.push("invalid_xyz_pixel_rejected");
      await page.locator("#left-image-plus").click();
      await imagePoint("left", 4, 1.5); await imagePoint("right", 4); await count(1);
      assert.match(await page.locator("#pair-rows").innerText(), /#4\s/);
      await page.locator("#left-image-reset").click();
      await page.locator("#pair-rows button").first().click(); await count(0);
      await page.locator("#undo").click(); await count(1); await clear();
      result.actions.push("zoom_pixel_selection_delete_undo_clear");
      await imagePoint("left", 0); await cloudPoint("right", 0); await count(1);
      assert.match(await page.locator("#pair-rows").innerText(), /#0\s/); await clear();
      result.actions.push("image_and_original_3d_share_selection_store");
      for (let id = 0; id < 6; id++) { await imagePoint("left", id); await imagePoint("right", id); await count(id + 1); }
      const waiting = page.waitForResponse(r => r.url().endsWith("/api/solve") && r.request().method() === "POST");
      await page.locator("#solve").click(); const response = await waiting, payload = await response.json();
      assert.equal(response.status(), 200); assert.equal(payload.result.status, "CANDIDATE");
      assert.equal(payload.result.live_eligible, false); assert.ok(payload.result.rmse_m < 1e-10);
      const matrix = payload.result.initial_T_left_right;
      for (let i = 0; i < 3; i++) assert.ok(Math.abs(matrix[i][3] - [.2, -.3, .1][i]) < 1e-10);
      result.solve = payload.result; result.solve_path = payload.export_dir;
      await page.locator("#manual-icp-link").waitFor({ state: "visible" });
      await page.screenshot({ path: path.join(output, "solved.png"), fullPage: true });
      for (const preset of ["front", "top", "side"]) await page.locator(`.view-tools[data-view="overlay"] [data-preset="${preset}"]`).click();
      const newTab = context.waitForEvent("page"); await page.locator("#manual-icp-link").click();
      const alignment = await newTab; await alignment.waitForLoadState();
      await alignment.locator("#pose-state").filter({ hasText: "已采用此场景最近保存的对应点初值" }).waitFor();
      assert.ok(alignment.url().endsWith("/alignment#initial=manual"));
      // The matrix and provenance live in a collapsed details element. Read its
      // stored text, not layout-dependent innerText, after the explicit ready state.
      const current = JSON.parse(await alignment.locator("#current-matrix").textContent());
      for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) assert.ok(Math.abs(current[i][j] - matrix[i][j]) < 1e-9, "Raw saved T must be applied once");
      assert.match(await alignment.locator("#alignment-provenance").textContent(), /saved_manual_initial/);
      await alignment.screenshot({ path: path.join(output, "manual_initial_in_icp.png"), fullPage: true });
      result.actions.push("six_image_pairs_solve_known_translation_and_handoff_raw_matrix_once");
    } else {
      await imagePoint("left", firstLeft); await imagePoint("right", firstRight); await count(1);
      await page.locator("#left-image-mode").selectOption("x");
      assert.match(await page.locator("#left-image-legend").innerText(), /前向 X/);
      await page.screenshot({ path: path.join(output, "real_selection_and_forward_x.png"), fullPage: true });
      await page.locator("#left-image-mode").selectOption("amplitude"); await clear();
      assert.deepEqual(posts, []); result.actions.push("real_image_select_pair_switch_x_clear_no_saves");
    }
    result.page_errors = errors; assert.deepEqual(errors, []); result.status = "PASS";
  } catch (error) { result.error = String(error.stack || error); throw error; }
  finally {
    if (browser) await browser.close();
    fs.writeFileSync(path.join(output, "result.json"), JSON.stringify(result, null, 2));
    console.log(JSON.stringify({ status: result.status, source: result.source, actions: result.actions, error: result.error }));
  }
})();
