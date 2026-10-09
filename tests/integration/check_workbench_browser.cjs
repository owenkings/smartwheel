/* Unified browser QA. Real fixtures permit calculations only, never candidate writes. */
"use strict";
const fs = require("node:fs"), path = require("node:path"), crypto = require("node:crypto"), assert = require("node:assert/strict");
const { chromium } = require(process.env.WC_TEST_PLAYWRIGHT || "playwright");
const args = process.argv.slice(2);
const option = name => { const i = args.indexOf(name); return i < 0 ? undefined : args[i + 1]; };
const url = option("--url"), destination = option("--output"), pairsFile = option("--pairs-file");
const candidateRoot = option("--candidate-root"), synthetic = args.includes("--synthetic"), exerciseSave = args.includes("--exercise-save");
assert.ok(/^http:\/\/127\.0\.0\.1:\d+\/$/.test(url), "Use a reviewed loopback frozen-data server");
assert.ok(destination, "--output must be a NEW QA directory on configured storage");
assert.ok(!exerciseSave || synthetic, "--exercise-save is synthetic-only");
assert.ok(synthetic || candidateRoot, "Real verification needs --candidate-root to check no candidate files changed");
const output = path.resolve(destination); fs.mkdirSync(output, { recursive: false });
const hash = bytes => crypto.createHash("sha256").update(bytes).digest("hex");
function snapshotCandidates() {
  if (!candidateRoot) return null;
  const records = {};
  function visit(directory) {
    if (!fs.existsSync(directory)) return;
    for (const entry of fs.readdirSync(directory, { withFileTypes: true })) {
      const file = path.join(directory, entry.name), stat = fs.lstatSync(file), relative = path.relative(candidateRoot, file);
      assert.equal(stat.isSymbolicLink(), false, "Do not follow candidate symlinks");
      if (entry.isDirectory()) { records[relative] = "directory"; visit(file); }
      else if (entry.isFile()) records[relative] = { bytes: stat.size, sha256: hash(fs.readFileSync(file)) };
    }
  }
  for (const name of ["manual_points", "alignment_candidates"]) visit(path.join(candidateRoot, name));
  return records;
}
function sameMatrix(actual, expected, label) {
  assert.equal(actual.length, 4, label);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++)
    assert.ok(Math.abs(actual[i][j] - expected[i][j]) < 1e-12, `${label}: [${i},${j}]`);
}
function deferred() { let resolve; const promise = new Promise(r => { resolve = r; }); return { promise, resolve }; }

(async () => {
    const report = { status: "FAIL", source: synthetic ? "synthetic" : "real_calculate_only", hardware_started: false,
    actions: [], posts: [], exercise_save: exerciseSave, started_page_count: 1 };
  const beforeFiles = snapshotCandidates();
  let browser, debugPage, held = null;
  try {
    browser = await chromium.launch({ headless: true, executablePath: process.env.WC_TEST_CHROME });
    const context = await browser.newContext({ viewport: { width: 1440, height: 1100 }, deviceScaleFactor: 1 });
    const page = await context.newPage(), errors = []; debugPage = page; report.request_failures=[]; page.on("requestfailed",r=>report.request_failures.push({url:r.url(),error:r.failure()}));
    page.setDefaultTimeout(30000);
    async function waitForMatrix(expected) {
      await page.waitForFunction(matrix => {
        try {
          const value = JSON.parse(document.querySelector("#a-current-matrix").textContent);
          return Array.isArray(value) && value.length === 4 && value.every((row, i) =>
            Array.isArray(row) && row.length === 4 && row.every((v, j) => Math.abs(v - matrix[i][j]) < 1e-12));
        } catch (_) { return false; }
      }, expected);
      return JSON.parse(await page.locator("#a-current-matrix").textContent());
    }
    context.on("page", p => { if (p !== page) report.actions.push("UNEXPECTED_NEW_TAB"); p.on("pageerror", e => errors.push(String(e))); });
    page.on("pageerror", e => errors.push(String(e)));
    const permitted = new Set(["/api/calculate", "/api/alignment-preview"]);
    if (exerciseSave) permitted.add("/api/alignment-save-result");
    await context.route("**/api/**", async route => {
      const request = route.request(), api = new URL(request.url()).pathname;
      if (request.method() !== "POST") return route.continue();
      const body = request.postDataJSON();
      report.posts.push({ api, body });
      if (!permitted.has(api)) {
        report.actions.push("BLOCKED_UNAUTHORIZED_POST:" + api);
        return route.abort("blockedbyclient");
      }
      if (held && held.api === api && !held.intercepted) {
        const gate = held; gate.intercepted = true;
        const response = await route.fetch({ timeout: 180000 });
        gate.hit.resolve(); await gate.release.promise;
        return route.fulfill({ response });
      }
      return route.continue();
    });
    await page.goto(url);
    await page.locator("#left-count").filter({ hasText: "个有效原始点" }).waitFor();
    await page.locator("#a-alignment-source").filter({ hasText: synthetic ? "合成点云" : "真实冻结点云" }).waitFor();
    assert.ok(await page.locator("body").evaluate(b => b.dataset.page === "workbench" || b.classList.contains("workbench")));
    const duplicateIds = await page.evaluate(() => {
      const counts = new Map(); document.querySelectorAll("[id]").forEach(e => counts.set(e.id, (counts.get(e.id) || 0) + 1));
      return [...counts].filter(([, count]) => count > 1);
    });
    assert.deepEqual(duplicateIds, [], "Merged picker/alignment must not have duplicate DOM IDs");
    for (const id of ["input-step", "initial-step", "refine-step", "review-step", "workflow-summary", "workflow-status"])
      assert.equal(await page.locator("#" + id).count(), 1, "Workflow anchor missing: " + id);
    assert.equal(await page.locator('a[target="_blank"][href^="/alignment"]').count(), 0, "Workflow must not send users to another tab");
    await page.evaluate(() => { window.__wcWorkbenchDocumentSentinel = "source-bound-single-document"; });
    const scene = await (await page.request.get(url + "api/scene")).json();
    assert.equal(scene.source_mode, synthetic ? "synthetic" : "real");
    assert.equal(scene.capabilities.calculate, true);
    report.scene = { scene_id: scene.scene_id, input_hash: scene.input_hash, sensor_ids: scene.sensor_ids };
    assert.equal(await page.locator("#calculate").isDisabled(), true);
    assert.equal(await page.locator("#use-initial").isDisabled(), true);
    assert.equal(await page.locator("#a-refine").isDisabled(), true, "Saving cached ICP requires a calculation");
    report.actions.push("single_page_unique_ids_stage_anchors_and_unsaved_defaults");
    await page.screenshot({ path: path.join(output, "01_workbench_loaded.png"), fullPage: true });

    let pairs;
    if (pairsFile) {
      const selection = JSON.parse(fs.readFileSync(pairsFile, "utf8"));
      assert.ok(Array.isArray(selection.pairs));
      if (selection.provenance?.prepared_input_hash) assert.equal(selection.provenance.prepared_input_hash, scene.input_hash);
      if (selection.provenance?.scene_id) assert.equal(selection.provenance.scene_id, scene.scene_id);
      if (selection.sensor_ids) assert.deepEqual(selection.sensor_ids, scene.sensor_ids);
      pairs = selection.pairs;
    } else {
      assert.ok(synthetic, "Real calculations require source-bound --pairs-file");
      pairs = Array.from({ length: 6 }, (_, id) => ({ left_id: id, right_id: id }));
    }
    async function imagePoint(side, id) {
      assert.ok(scene.clouds[side].some(p => p.id === id));
      const canvas = page.locator(`#${side}-image`); await canvas.scrollIntoViewIfNeeded();
      const position = await canvas.evaluate((c, arg) => {
        const rect = c.getBoundingClientRect(), view = window.AmplitudeUI.viewport(arg.layout, c.width, c.height);
        const p = window.AmplitudeUI.pixelCenter(arg.layout, view, arg.id);
        return { x: rect.left + p.x * rect.width / c.width, y: rect.top + p.y * rect.height / c.height };
      }, { layout: scene.organized[side], id });
      await page.mouse.click(position.x, position.y);
    }
    await page.locator("#mode-pick").click();
    for (const pair of pairs) { await imagePoint("left", pair.left_id); await imagePoint("right", pair.right_id); }
    await page.waitForFunction(n => document.querySelector("#pair-count").textContent.startsWith(n + " 对"), pairs.length);

    async function delayedCalculation(button, api, whileHeld) {
      held = { api, intercepted: false, hit: deferred(), release: deferred() };
      const gate = held;
      const pending = page.waitForResponse(r => r.url().endsWith(api) && r.request().method() === "POST", { timeout: 180000 });
      await page.locator(button).click();
      let timeout;
      try {
        await Promise.race([gate.hit.promise, new Promise((_, reject) => { timeout = setTimeout(() => reject(new Error("Calculation did not reach delayed response")), 180000); })]);
      } finally { clearTimeout(timeout); }
      try { await whileHeld(); } finally { gate.release.resolve(); held = null; }
      const response = await pending, payload = await response.json();
      assert.equal(response.status(), 200); assert.equal(payload.saved, false); assert.equal(payload.export_dir, null);
      assert.equal(payload.assessment.formal_use, false);
      return payload;
    }
    const manual = await delayedCalculation("#calculate", "/api/calculate", async () => {
      for (const id of ["calculate", "use-initial", "clear", "a-preview-refine"])
        assert.equal(await page.locator("#" + id).isDisabled(), true, "Shared busy state must lock " + id);
    });
    await page.locator("#picker-assessment").waitFor({ state: "visible" });
    await page.waitForFunction(() => !document.querySelector("#use-initial").disabled);
    assert.equal(await page.locator("#saved").isVisible(), false);
    if (!synthetic) assert.equal(manual.assessment.level, "REJECTED", "Known real pairs must show rejection, not formal acceptance");
    report.manual = { status: manual.result.status, saved: manual.saved, rmse_m: manual.result.rmse_m, assessment: manual.assessment };
    const initial = manual.result.initial_T_left_right;
    await page.locator("#use-initial").click();
    sameMatrix(await waitForMatrix(initial), initial, "unsaved initial passed exactly once");
    if (await page.locator("#use-initial").isEnabled()) {
      await page.locator("#use-initial").click();
      sameMatrix(JSON.parse(await page.locator("#a-current-matrix").textContent()), initial, "repeat handoff must not compose twice");
    }
    assert.equal(await page.evaluate(() => window.__wcWorkbenchDocumentSentinel), "source-bound-single-document");
    assert.equal(context.pages().length, 1);
    report.actions.push("unsaved_manual_initial_handed_to_same_page_exactly_once_without_navigation");
    await page.screenshot({ path: path.join(output, "02_unsaved_initial_handoff.png"), fullPage: true });

    const preview = await delayedCalculation("#a-preview-refine", "/api/alignment-preview", async () => {
      for (const id of ["calculate", "use-initial", "clear", "a-preview-refine", "a-refine", "load-saved-candidate"])
        assert.equal(await page.locator("#" + id).isDisabled(), true, "Shared busy state must lock " + id);
    });
    assert.ok(typeof preview.calculation_id === "string" && preview.calculation_id);
    const submitted = report.posts.filter(p => p.api === "/api/alignment-preview").at(-1).body;
    assert.equal(submitted.scene_id, scene.scene_id); assert.equal(submitted.input_hash, scene.input_hash);
    sameMatrix(submitted.initial_T_left_right, initial, "ICP gets the raw matrix, not rounded RPY or a composed display transform");
    await page.locator("#a-saved-path").filter({ hasText: "未保存" }).waitFor();
    assert.equal(await page.locator("#a-refine").isEnabled(), true);
    report.preview = { status: preview.result.status, calculation_id: preview.calculation_id, assessment: preview.assessment };
    report.actions.push("icp_preview_source_bound_unsaved_cache_and_global_busy");
    await page.screenshot({ path: path.join(output, "03_icp_assessment_unsaved.png"), fullPage: true });

    // Wrong-scene requests are safe non-persisting requests; no candidate save is attempted on real data.
    const wrong = await page.request.post(url + "api/calculate", { headers: { "X-Picker-Token": scene.token, Origin: new URL(url).origin },
      data: { input_hash: scene.input_hash, scene_id: scene.scene_id + "-FOREIGN", pairs } });
    assert.equal(wrong.status(), 422); assert.ok((await wrong.json()).error);
    report.actions.push("foreign_scene_nonpersisting_request_rejected");

    if (exerciseSave) {
      const beforePreviewCount = report.posts.filter(p => p.api === "/api/alignment-preview").length;
      const pending = page.waitForResponse(r => r.url().endsWith("/api/alignment-save-result") && r.request().method() === "POST");
      await page.locator("#a-refine").click(); const response = await pending, saved = await response.json();
      assert.equal(response.status(), 200); assert.equal(saved.saved, true); assert.ok(saved.export_dir);
      assert.deepEqual(saved.result, preview.result, "Save must preserve the exact reviewed ICP result");
      assert.equal(report.posts.filter(p => p.api === "/api/alignment-preview").length, beforePreviewCount, "Save must not rerun ICP");
      const savePost = report.posts.filter(p => p.api === "/api/alignment-save-result").at(-1);
      assert.deepEqual(Object.keys(savePost.body).sort(), ["calculation_id", "input_hash", "scene_id"]);
      assert.equal(savePost.body.calculation_id, preview.calculation_id);
      report.saved = { export_dir: saved.export_dir, calculation_id: preview.calculation_id };
      await page.locator("#a-saved-path").filter({ hasText: "已保存" }).waitFor();
      report.actions.push("explicit_synthetic_cached_save_has_no_second_icp");
    }

    // A pose edit invalidates the cached-save action and the displayed geometry assessment.
    const xInput = page.locator("#a-pose-x"), originalX = Number(await xInput.inputValue());
    await xInput.fill(String(originalX + .01)); await page.locator("#a-apply-pose").click();
    assert.equal(await page.locator("#a-refine").isDisabled(), true);
    assert.equal(await page.locator("#a-assessment-stale").isVisible(), true);
    report.actions.push("pose_edit_invalidates_cached_save_and_marks_assessment_stale");

    // Editing manual correspondences cannot silently reuse an initial from the old selection.
    await page.locator("#pair-rows button").first().click();
    assert.equal(await page.locator("#use-initial").isDisabled(), true);
    assert.equal(await page.locator("#result-section").isVisible(), false);
    const workflowText = await page.locator("#workflow-summary").textContent() + " " + await page.locator("#workflow-status").textContent();
    assert.match(workflowText, /重新|变化|失效|过期|尚未|已修改|更新/);
    report.actions.push("selection_edit_invalidates_handoff_and_workflow_summary");

    if (exerciseSave) {
      await page.reload();
      await page.locator("#a-alignment-source").filter({ hasText: "合成点云" }).waitFor();
      assert.equal(await page.locator("#load-saved-candidate").isEnabled(), true);
      // Saved ICP is offered as history, never silently installed as the live workspace state.
      const bootstrap = await (await page.request.get(url + "api/alignment-bootstrap")).json();
      const expectedInitial = bootstrap.prepared_initial_T_left_right || [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
      sameMatrix(await waitForMatrix(expectedInitial), expectedInitial,
        "bootstrap must retain explicit initial rather than automatically using saved refined candidate");
      const beforeLoadCount = report.posts.length;
      const loadResponse = page.waitForResponse(r => r.url().endsWith("/api/alignment-bootstrap") && r.request().method() === "GET");
      await page.locator("#load-saved-candidate").click();
      assert.equal((await loadResponse).status(), 200);
      await page.locator("#workflow-status").filter({ hasText: "已明确恢复" }).waitFor();
      sameMatrix(await waitForMatrix(preview.result.refined_T_left_right), preview.result.refined_T_left_right,
        "explicit saved candidate restore uses raw matrix once");
      assert.equal(report.posts.length, beforeLoadCount, "Loading history must not calculate or save");
      report.actions.push("history_restored_only_by_explicit_button_no_extra_post");
    }
    assert.equal(context.pages().length, 1);
    assert.ok(report.posts.every(p => permitted.has(p.api)));
    assert.deepEqual(errors, []); report.page_errors = errors;
    if (!synthetic) {
      assert.deepEqual(snapshotCandidates(), beforeFiles, "Real candidate files must remain byte-identical");
      report.candidate_files_unchanged = true;
    }
    await page.screenshot({ path: path.join(output, "04_final_state.png"), fullPage: true });
    report.status = "PASS";
  } catch (error) { report.error = String(error.stack || error); if(debugPage) { report.failure_text=await debugPage.locator("body").innerText().catch(()=>"unavailable"); } process.exitCode = 1; }
  finally {
    if (held) held.release.resolve();
    if (browser) await browser.close();
    fs.writeFileSync(path.join(output, "result.json"), JSON.stringify(report, null, 2));
    console.log(JSON.stringify({ status: report.status, source: report.source, actions: report.actions,
      posts: report.posts.map(p => p.api), error: report.error }));
  }
})();
