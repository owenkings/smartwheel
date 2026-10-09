/* Browser QA using actual scene IDs. Real runs never export or run ICP. */
"use strict";
const fs = require("node:fs"), path = require("node:path"), assert = require("node:assert/strict");
const { chromium } = require(process.env.WC_TEST_PLAYWRIGHT || "playwright");
const args = process.argv.slice(2);
function option(name) { const i = args.indexOf(name); return i < 0 ? undefined : args[i + 1]; }
const url = option("--url"), destination = option("--output"), pairsFile = option("--pairs-file"), synthetic = args.includes("--synthetic");
assert.ok(/^http:\/\/127\.0\.0\.1:\d+\/$/.test(url), "Only a reviewed loopback frozen-data server is allowed");
assert.ok(destination, "--output must name a new QA artifact directory on configured storage");
const output = path.resolve(destination); fs.mkdirSync(output, { recursive: false });
(async () => {
  const report = { status: "FAIL", source: synthetic ? "synthetic" : "real_read_only_calculation", starts_devices: false, actions: [], posts: [] };
  let browser;
  try {
    browser = await chromium.launch({ headless: true, executablePath: process.env.WC_TEST_CHROME });
    const context = await browser.newContext({ viewport: { width: 1440, height: 1100 }, deviceScaleFactor: 1 });
    const page = await context.newPage(), errors = [];
    page.on("pageerror", e => errors.push(String(e)));
    await context.route("**/api/**", route => {
      const request = route.request();
      if (request.method() === "POST") {
        const api = new URL(request.url()).pathname; report.posts.push(api);
        if (api !== "/api/calculate" && !(synthetic && api === "/api/alignment-preview")) {
          report.actions.push("BLOCKED_UNAUTHORIZED_POST:" + api); return route.abort("blockedbyclient");
        }
      }
      return route.continue();
    });
    await page.goto(url);
    await page.locator("#left-count").filter({ hasText: "个有效原始点" }).waitFor();
    const scene = await (await page.request.get(url + "api/scene")).json();
    assert.equal(scene.source_mode, synthetic ? "synthetic" : "real");
    assert.equal(scene.capabilities.calculate, true); assert.equal(scene.capabilities.alignment_preview, true);
    assert.equal(await page.locator("#view-scale-mode").inputValue(), "shared");
    assert.equal(await page.locator("#api-version-note").isVisible(), false);
    assert.equal(await page.locator("#calculate").isDisabled(), true);
    report.scene = { scene_id: scene.scene_id, input_hash: scene.input_hash, sensor_ids: scene.sensor_ids };
    report.actions.push("defaults_to_common_metric_scale_and_advertised_calculation_capability");
    await page.screenshot({ path: path.join(output, "loaded_shared_scale.png"), fullPage: true });
    let pairs;
    if (pairsFile) {
      const selection = JSON.parse(fs.readFileSync(pairsFile, "utf8"));
      assert.ok(Array.isArray(selection.pairs), "selected_pairs.json must contain original ID pairs");
      if (selection.provenance?.prepared_input_hash) assert.equal(selection.provenance.prepared_input_hash, scene.input_hash, "Do not replay IDs from a different prepared input");
      pairs = selection.pairs; report.pairs_file = path.resolve(pairsFile);
    } else if (synthetic) pairs = Array.from({ length: 6 }, (_, id) => ({ left_id: id, right_id: id }));
    if (pairs) {
      assert.ok(pairs.length >= 3, "Need at least three source pairs");
      await page.locator("#mode-pick").click();
      async function clickPixel(side, id) {
        assert.ok(scene.clouds[side].some(p => p.id === id), `ID ${id} must have original XYZ for ${side}`);
        const layout = scene.organized[side], canvas = page.locator(`#${side}-image`);
        assert.ok(layout, "Amplitude layout required for browser image-point verification");
        await canvas.scrollIntoViewIfNeeded();
        const position = await canvas.evaluate((c, arg) => {
          const bounds = c.getBoundingClientRect(), view = window.AmplitudeUI.viewport(arg.layout, c.width, c.height);
          const pixel = window.AmplitudeUI.pixelCenter(arg.layout, view, arg.id);
          return { x: bounds.left + pixel.x * bounds.width / c.width, y: bounds.top + pixel.y * bounds.height / c.height };
        }, { layout, id });
        await page.mouse.click(position.x, position.y);
      }
      for (const pair of pairs) { await clickPixel("left", pair.left_id); await clickPixel("right", pair.right_id); }
      await page.waitForFunction(n => document.querySelector("#pair-count").textContent.startsWith(n + " 对"), pairs.length);
      assert.equal(await page.locator("#calculate").isEnabled(), true);
      const responsePromise = page.waitForResponse(r => r.url().endsWith("/api/calculate") && r.request().method() === "POST");
      await page.locator("#calculate").click(); const response = await responsePromise, payload = await response.json();
      assert.equal(response.status(), 200); assert.equal(payload.saved, false); assert.equal(payload.export_dir, null);
      assert.equal(payload.assessment.formal_use, false);
      if (!synthetic) assert.equal(payload.assessment.level, "REJECTED", "Known real selected-pair fixture should be reported rejected");
      await page.locator("#picker-assessment").waitFor({ state: "visible" });
      await page.locator("#request-status").filter({ hasText: "尚未保存" }).waitFor();
      assert.equal(await page.locator("#saved").isVisible(), false);
      assert.equal(await page.locator("#manual-icp-link").isVisible(), false);
      assert.equal(await page.locator("#manual-icp-flow").isVisible(), false);
      assert.match(await page.locator("#result-status").innerText(), /未保存/);
      report.calculation = { saved: payload.saved, export_dir: payload.export_dir, status: payload.result.status,
        assessment: payload.assessment, pair_count: pairs.length, rmse_m: payload.result.rmse_m, max_residual_m: payload.result.max_residual_m };
      report.actions.push("actual_amplitude_clicks_calculate_without_saving_or_saved_icp_link");
      await page.screenshot({ path: path.join(output, "calculated_unsaved_assessment.png"), fullPage: true });
    } else report.actions.push("no_pairs_file_real_calculation_not_run");
    const alignment = await context.newPage(); alignment.on("pageerror", e => errors.push(String(e)));
    await alignment.goto(url + "alignment");
    await alignment.locator("#alignment-assessment").waitFor({ state: "visible" });
    await alignment.locator("#alignment-source").filter({ hasText: synthetic ? "合成点云" : "真实冻结点云" }).waitFor();
    report.bootstrap_assessment = await alignment.locator("#alignment-assessment").innerText();
    report.actions.push("alignment_bootstrap_assessment_visible");
    if (synthetic) {
      const waiting = alignment.waitForResponse(r => r.url().endsWith("/api/alignment-preview") && r.request().method() === "POST");
      await alignment.locator("#preview-refine").click(); const response = await waiting, preview = await response.json();
      assert.equal(response.status(), 200); assert.equal(preview.saved, false); assert.equal(preview.export_dir, null);
      assert.equal(preview.assessment.formal_use, false); assert.equal(preview.assessment.level, "REJECTED");
      assert.ok(["INSUFFICIENT_OVERLAP", "DEGENERATE"].includes(preview.result.status), "Small synthetic scene must not pass ICP candidate checks");
      await alignment.locator("#saved-path").filter({ hasText: "本次仅计算" }).waitFor();
      report.icp_preview = { status: preview.result.status, saved: preview.saved, export_dir: preview.export_dir, assessment: preview.assessment };
      report.actions.push("small_synthetic_icp_rejection_still_unsaved");
    }
    await alignment.screenshot({ path: path.join(output, "alignment_assessment.png"), fullPage: true });
    assert.ok(report.posts.every(api => api === "/api/calculate" || synthetic && api === "/api/alignment-preview"));
    report.page_errors = errors; assert.deepEqual(errors, []); report.status = "PASS";
  } catch (error) { report.error = String(error.stack || error); throw error; }
  finally {
    if (browser) await browser.close();
    fs.writeFileSync(path.join(output, "result.json"), JSON.stringify(report, null, 2));
    console.log(JSON.stringify({ status: report.status, source: report.source, actions: report.actions, posts: report.posts, error: report.error }));
  }
})();
