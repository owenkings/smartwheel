/* Real frozen-cloud UI QA: GET only. Every POST is blocked, including saves/calculations. */
"use strict";
const fs = require("node:fs"), path = require("node:path"), assert = require("node:assert/strict");
const { chromium } = require(process.env.WC_TEST_PLAYWRIGHT || "playwright");
const args = process.argv.slice(2);
const option = name => { const i = args.indexOf(name); return i < 0 ? undefined : args[i + 1]; };
const url = option("--url"), destination = option("--output");
assert.ok(/^http:\/\/127\.0\.0\.1:\d+\/$/.test(url), "Use a reviewed loopback frozen-data workbench");
assert.ok(destination, "--output must be a NEW directory on configured USB storage");
const output = path.resolve(destination);
fs.mkdirSync(output, { recursive: false });
const selectIds = ["display-frame", "view-scale-mode", "left-image-mode", "right-image-mode", "a-display-frame", "a-point-radius"];

(async () => {
  const report = { status: "FAIL", scope: "real_frozen_data_ui_get_only", hardware_started: false,
    calculations_requested: false, saves_requested: false, actions: [], requests: [], posts_blocked: [], fixtures: [] };
  let browser;
  try {
    browser = await chromium.launch({ headless: true, executablePath: process.env.WC_TEST_CHROME });
    const context = await browser.newContext({ viewport: { width: 1440, height: 1000 }, deviceScaleFactor: 1 });
    await context.route("**/*", route => {
      const request = route.request();
      if (!["GET", "HEAD"].includes(request.method())) {
        report.posts_blocked.push({ method: request.method(), url: request.url() });
        return route.abort("blockedbyclient");
      }
      return route.continue();
    });
    const page = await context.newPage(), errors = [];
    page.setDefaultTimeout(30000);
    page.on("pageerror", error => errors.push(String(error)));
    page.on("request", request => {
      if (request.url().startsWith(url)) report.requests.push({ method: request.method(), path: new URL(request.url()).pathname });
    });
    await page.goto(url);
    await page.locator("#left-count").filter({ hasText: "个有效原始点" }).waitFor();
    await page.locator("#a-alignment-source").filter({ hasText: "真实冻结点云" }).waitFor();
    const scene = await (await page.request.get(url + "api/scene")).json();
    assert.equal(scene.source_mode, "real", "This check must use frozen real data without any POST");
    report.scene = { scene_id: scene.scene_id, input_hash: scene.input_hash, sensor_ids: scene.sensor_ids };
    const saveIds = ["solve", "export", "a-refine", "a-save-manual"];
    const saveStateBefore = Object.fromEntries(await Promise.all(saveIds.map(async id =>
      [id, await page.locator("#" + id).isDisabled()])));
    const native = id => page.locator("#" + id);
    const trigger = id => page.locator("#" + id + "-trigger");
    const menu = id => page.locator("#" + id + "-listbox");
    const openedMenus = () => page.locator('[role="listbox"]:visible');
    const value = id => native(id).inputValue();
    const changes = id => page.evaluate(key => window.__selectQaChanges[key], id);
    async function options(id) { return native(id).evaluate(el => Array.from(el.options, o => ({ text: o.textContent.trim(), value: o.value, disabled: o.disabled }))); }
    async function optionAt(id, index) { return menu(id).getByRole("option").nth(index); }
    async function expanded(id, open) {
      await page.waitForFunction(({ key, expected }) => document.getElementById(key + "-trigger").getAttribute("aria-expanded") === String(expected),
        { key: id, expected: open });
      assert.equal(await menu(id).isVisible(), open);
      if (open) {
        assert.equal(await menu(id).getAttribute("role"), "listbox");
        assert.equal(await menu(id).evaluate(node => node.parentElement === document.body), true,
          id + ": an opened listbox is a body portal");
      }
    }
    async function activeIndex(id) {
      return menu(id).getByRole("option").evaluateAll(nodes => nodes.findIndex(node => node.classList.contains("is-active")));
    }
    async function waitActive(id, index) {
      await page.waitForFunction(({ key, wanted }) => Array.from(document.querySelectorAll('#' + key + '-listbox [role="option"]'))
        .findIndex(n => n.classList.contains("is-active")) === wanted, { key: id, wanted: index });
    }
    async function clickTrigger(id) {
      await trigger(id).scrollIntoViewIfNeeded(); await trigger(id).click(); await expanded(id, true);
    }
    async function escape(id) { await page.keyboard.press("Escape"); await expanded(id, false); }
    async function assertMenuInside(id, label) {
      const box = await menu(id).boundingBox(), viewport = page.viewportSize();
      assert.ok(box, label + ": menu visible");
      assert.ok(box.x >= -1 && box.y >= -1 && box.x + box.width <= viewport.width + 1 && box.y + box.height <= viewport.height + 1,
        label + ": menu must remain inside viewport " + JSON.stringify({ box, viewport }));
      report.actions.push(label); return box;
    }
    async function assertOptionLabelsFit(id) {
      const metrics = await menu(id).locator('.w-select-option-label').evaluateAll(nodes => nodes.map(node => ({
        text: node.textContent.trim(), clientWidth: node.clientWidth, scrollWidth: node.scrollWidth,
        fontWeight: getComputedStyle(node).fontWeight
      })));
      assert.equal(metrics.length, (await options(id)).length, id + ": each option has a measurable label");
      for (const item of metrics) assert.ok(item.clientWidth > 0 && item.scrollWidth <= item.clientWidth + 1,
        id + ": desktop option must not be truncated " + JSON.stringify(item));
      (report.option_label_metrics ||= []).push({ select_id: id, viewport: page.viewportSize(), options: metrics });
    }

    await page.waitForFunction(ids => ids.every(id => {
      const select = document.getElementById(id), button = document.getElementById(id + "-trigger");
      return select && select.classList.contains("w-select-native") && button && button.getAttribute("role") === "combobox";
    }), selectIds);
    const actualIds = await page.locator("select").evaluateAll(nodes => nodes.map(n => n.id).sort());
    assert.deepEqual(actualIds, selectIds.slice().sort(), "All six existing selects must be enhanced");
    for (const id of selectIds) {
      assert.equal(await native(id).isVisible(), false, id + ": native remains available but visually hidden");
      assert.equal(await trigger(id).getAttribute("role"), "combobox");
      assert.equal(await trigger(id).getAttribute("aria-controls"), id + "-listbox");
      assert.equal(await trigger(id).isDisabled(), await native(id).isDisabled(), id + ": disabled states match");
      // Listboxes may be created on open and removed on close; only the native
      // control and its accessible trigger must exist before interaction.
      assert.equal(await menu(id).isVisible(), false);
    }
    await page.evaluate(ids => {
      window.__selectQaChanges = {};
      for (const id of ids) {
        window.__selectQaChanges[id] = 0;
        document.getElementById(id).addEventListener("change", () => window.__selectQaChanges[id]++);
      }
    }, selectIds);
    for (const id of ["display-frame", "a-display-frame"]) {
      assert.equal(await native(id).isDisabled(), true, "This real fixture has no level reference: " + id);
      assert.equal(await trigger(id).isDisabled(), true);
      await trigger(id).evaluate(button => button.click());
      assert.equal(await menu(id).isVisible(), false);
    }
    report.actions.push("six_selects_enhanced_with_portal_aria_and_disabled_reference_controls");

    // Mouse selection: application receives exactly one native change event.
    assert.equal(await value("view-scale-mode"), "shared");
    await clickTrigger("view-scale-mode");
    await assertOptionLabelsFit("view-scale-mode");
    const viewOptions = await options("view-scale-mode");
    assert.equal(await menu("view-scale-mode").getByRole("option").count(), viewOptions.length);
    const selected = await optionAt("view-scale-mode", 0);
    assert.equal(await selected.getAttribute("aria-selected"), "true");
    assert.ok(await selected.evaluate(node => node.classList.contains("is-selected")));
    const theme = await menu("view-scale-mode").evaluate(node => {
      const menuStyle = getComputedStyle(node), selectedStyle = getComputedStyle(node.querySelector('[role="option"].is-selected'));
      return { background: menuStyle.backgroundColor, color: menuStyle.color, selectedBackground: selectedStyle.backgroundColor };
    });
    const rgb = String(theme.background).match(/[\d.]+/g).map(Number);
    assert.ok(rgb.length >= 3 && (rgb[0] + rgb[1] + rgb[2]) / 3 >= 210 && (rgb.length < 4 || rgb[3] > .9), "Menu must have a solid light surface");
    report.menu_theme = theme;
    const hovered = await optionAt("view-scale-mode", 1);
    await hovered.hover(); await waitActive("view-scale-mode", 1);
    assert.equal(await selected.getAttribute("aria-selected"), "true", "Hover is not a selection");
    assert.equal(await value("view-scale-mode"), "shared");
    await page.screenshot({ path: path.join(output, "01_light_menu_hover_selected.png") });
    const countBeforeMouse = await changes("view-scale-mode");
    await hovered.click(); await expanded("view-scale-mode", false);
    assert.equal(await value("view-scale-mode"), "independent");
    assert.equal(await changes("view-scale-mode"), countBeforeMouse + 1);
    assert.match(await trigger("view-scale-mode").innerText(), /各自适应/);
    report.actions.push("mouse_choice_changes_native_once_selected_and_hover_are_distinct");

    // Keyboard navigation is tentative until Enter; Home/End and Arrow keys skip no valid options.
    await trigger("view-scale-mode").focus(); await page.keyboard.press("Enter"); await expanded("view-scale-mode", true);
    const countBeforeKeys = await changes("view-scale-mode");
    await page.keyboard.press("Home"); await waitActive("view-scale-mode", 0);
    await page.keyboard.press("End"); await waitActive("view-scale-mode", 1);
    await page.keyboard.press("ArrowUp"); await waitActive("view-scale-mode", 0);
    await page.keyboard.press("ArrowDown"); await waitActive("view-scale-mode", 1);
    assert.equal(await value("view-scale-mode"), "independent");
    assert.equal(await changes("view-scale-mode"), countBeforeKeys);
    await page.keyboard.press("Home"); await page.keyboard.press("Enter"); await expanded("view-scale-mode", false);
    assert.equal(await value("view-scale-mode"), "shared");
    assert.equal(await changes("view-scale-mode"), countBeforeKeys + 1);
    assert.equal(await trigger("view-scale-mode").evaluate(node => document.activeElement === node), true);
    report.actions.push("keyboard_arrows_home_end_enter_commit_once_and_restore_trigger_focus");

    const escapeValue = await value("left-image-mode"), escapeChanges = await changes("left-image-mode");
    await clickTrigger("left-image-mode"); await assertOptionLabelsFit("left-image-mode"); await page.keyboard.press("End");
    assert.equal(await value("left-image-mode"), escapeValue);
    await escape("left-image-mode");
    assert.equal(await value("left-image-mode"), escapeValue); assert.equal(await changes("left-image-mode"), escapeChanges);
    await trigger("left-image-mode").focus(); await page.keyboard.press("Space"); await expanded("left-image-mode", true);
    await page.keyboard.press("End"); await page.keyboard.press("Tab"); await expanded("left-image-mode", false);
    assert.equal(await value("left-image-mode"), escapeValue); assert.equal(await changes("left-image-mode"), escapeChanges);
    assert.equal(await trigger("left-image-mode").evaluate(node => document.activeElement === node), false, "Tab must not trap focus");
    report.actions.push("escape_and_tab_close_without_committing_tentative_option");

    await clickTrigger("left-image-mode");
    const xOption = await optionAt("left-image-mode", 1); await xOption.click();
    assert.equal(await value("left-image-mode"), "x");
    assert.equal(await changes("left-image-mode"), escapeChanges + 1);
    await page.locator("#left-image-legend").filter({ hasText: "前向 X" }).waitFor();
    report.actions.push("actual_amplitude_x_selection_updates_application_legend");

    await clickTrigger("view-scale-mode");
    await trigger("left-image-mode").scrollIntoViewIfNeeded(); await trigger("left-image-mode").click();
    await expanded("left-image-mode", true);
    assert.equal(await openedMenus().count(), 1);
    assert.equal(await menu("view-scale-mode").isVisible(), false);
    await page.locator("#input-heading").click();
    await expanded("left-image-mode", false); assert.equal(await openedMenus().count(), 0);
    report.actions.push("single_open_menu_and_outside_click_close");

    // Programmatic state is a browser-only fixture; it does not call a service or choose an extrinsic.
    report.fixtures.push("native.value + bubbling change, native.disabled and option.disabled in browser only");
    await native("left-image-mode").evaluate(select => { select.value = "amplitude"; select.dispatchEvent(new Event("change", { bubbles: true })); });
    await page.waitForFunction(() => document.querySelector("#left-image-mode-trigger").textContent.includes("回波幅度"));
    await clickTrigger("left-image-mode");
    assert.ok(await (await optionAt("left-image-mode", 0)).evaluate(node => node.classList.contains("is-selected")));
    await escape("left-image-mode");
    await native("view-scale-mode").evaluate(select => { select.disabled = true; });
    await page.waitForFunction(() => document.querySelector("#view-scale-mode-trigger").disabled);
    await trigger("view-scale-mode").evaluate(button => button.click());
    assert.equal(await menu("view-scale-mode").isVisible(), false);
    await native("view-scale-mode").evaluate(select => { select.disabled = false; });
    await page.waitForFunction(() => !document.querySelector("#view-scale-mode-trigger").disabled);
    await clickTrigger("view-scale-mode");
    await native("view-scale-mode").evaluate(select => { select.disabled = true; });
    await expanded("view-scale-mode", false);
    assert.equal(await trigger("view-scale-mode").isDisabled(), true);
    await native("view-scale-mode").evaluate(select => { select.disabled = false; select.options[1].disabled = true; });
    await page.waitForFunction(() => !document.querySelector("#view-scale-mode-trigger").disabled);
    await clickTrigger("view-scale-mode");
    const disabledOption = await optionAt("view-scale-mode", 1);
    assert.equal(await disabledOption.getAttribute("aria-disabled"), "true");
    assert.ok(await disabledOption.evaluate(node => node.classList.contains("is-disabled")));
    const beforeDisabledClick = await changes("view-scale-mode");
    await disabledOption.evaluate(node => node.click());
    assert.equal(await value("view-scale-mode"), "shared");
    assert.equal(await changes("view-scale-mode"), beforeDisabledClick);
    await page.keyboard.press("End"); assert.equal(await activeIndex("view-scale-mode"), 0, "Disabled option skipped");
    await escape("view-scale-mode");
    await native("view-scale-mode").evaluate(select => { select.options[1].disabled = false; });
    report.actions.push("programmatic_value_change_and_disabled_states_sync_with_native_control");

    // A business handler may reject a requested value during native change.
    // The visible trigger must reflect the final native value, not the tentative
    // option captured before dispatch. The fixture is removed even on failure.
    report.fixtures.push("temporary native change handler reverts one requested view-scale value");
    await native("view-scale-mode").evaluate(select => {
      select.value = "shared"; select.dispatchEvent(new Event("change", { bubbles: true }));
      window.__selectQaRollback = event => { event.target.value = "shared"; };
      select.addEventListener("change", window.__selectQaRollback);
    });
    try {
      const rollbackCount = await changes("view-scale-mode");
      await clickTrigger("view-scale-mode");
      await (await optionAt("view-scale-mode", 1)).click(); await expanded("view-scale-mode", false);
      assert.equal(await value("view-scale-mode"), "shared");
      assert.equal(await changes("view-scale-mode"), rollbackCount + 1);
      await page.waitForFunction(() => document.querySelector("#view-scale-mode-trigger").textContent.includes("同一米制"));
      await clickTrigger("view-scale-mode");
      assert.ok(await (await optionAt("view-scale-mode", 0)).evaluate(node => node.classList.contains("is-selected")));
      await escape("view-scale-mode");
      report.actions.push("business_change_rollback_reconciles_trigger_and_selected_option");
    } finally {
      await native("view-scale-mode").evaluate(select => {
        select.removeEventListener("change", window.__selectQaRollback);
        delete window.__selectQaRollback;
        // Restore both the control and the application's actual rendering mode.
        select.value = "shared"; select.dispatchEvent(new Event("change", { bubbles: true }));
      });
    }

    const explicitLabel = page.locator('label[for="view-scale-mode"], label[for="view-scale-mode-trigger"]');
    assert.equal(await explicitLabel.count(), 1);
    await explicitLabel.click();
    assert.equal(await trigger("view-scale-mode").evaluate(node => document.activeElement === node), true, "Label focuses enhanced trigger");
    if (await menu("view-scale-mode").isVisible()) await escape("view-scale-mode");
    report.actions.push("native_explicit_label_focuses_accessible_combobox");

    const radiusBefore = await changes("a-point-radius");
    await clickTrigger("a-point-radius");
    await (await optionAt("a-point-radius", 2)).click(); await expanded("a-point-radius", false);
    assert.equal(await value("a-point-radius"), "2.6"); assert.equal(await changes("a-point-radius"), radiusBefore + 1);
    assert.match(await trigger("a-point-radius").innerText(), /2\.6/);
    report.actions.push("actual_point_radius_menu_updates_native_setting_once");

    await page.setViewportSize({ width: 390, height: 740 });
    await clickTrigger("left-image-mode");
    await assertMenuInside("left-image-mode", "narrow_viewport_menu_inside_screen");
    await page.screenshot({ path: path.join(output, "02_narrow_menu.png") });
    await escape("left-image-mode");
    report.fixtures.push("temporarily fix point-radius trigger to viewport edges to verify popup clamping");
    const originalStyle = await trigger("a-point-radius").getAttribute("style");
    for (const placement of ["bottom-right", "top-left"]) {
      await trigger("a-point-radius").evaluate((button, where) => {
        button.style.cssText = "position:fixed;z-index:10010;width:180px;max-width:calc(100vw - 8px);" +
          (where === "bottom-right" ? "right:4px;bottom:4px;left:auto;top:auto" : "left:4px;top:4px;right:auto;bottom:auto");
      }, placement);
      await trigger("a-point-radius").click(); await expanded("a-point-radius", true);
      await assertMenuInside("a-point-radius", placement + "_menu_inside_screen");
      await page.screenshot({ path: path.join(output, "03_edge_" + placement + ".png") });
      await escape("a-point-radius");
    }
    await trigger("a-point-radius").evaluate((button, previous) => {
      if (previous === null) button.removeAttribute("style"); else button.setAttribute("style", previous);
    }, originalStyle);
    await page.setViewportSize({ width: 1440, height: 1000 });

    await page.locator("#review-step").scrollIntoViewIfNeeded();
    const saveText = await page.locator("#review-step").innerText();
    assert.match(saveText, /候选/);
    assert.match(saveText, /不会启用正式外参|未启用|不.*正式配置/);
    assert.equal(await page.locator("#a-refine").isDisabled(), true, "No ICP requested, so saving stays disabled");
    const saveStateAfter = Object.fromEntries(await Promise.all(saveIds.map(async id =>
      [id, await page.locator("#" + id).isDisabled()])));
    assert.deepEqual(saveStateAfter, saveStateBefore, "Dropdown rendering choices cannot change save eligibility");
    report.save_disabled_states = { before: saveStateBefore, after: saveStateAfter };
    await page.screenshot({ path: path.join(output, "04_save_section_unchanged.png") });
    report.actions.push("save_section_still_explains_candidate_only_no_config_activation");
    assert.equal(context.pages().length, 1);
    assert.deepEqual(report.posts_blocked, [], "Dropdown interactions must not even attempt a POST");
    assert.deepEqual(errors, []); report.page_errors = errors;
    report.native_change_counts = await page.evaluate(() => window.__selectQaChanges);
    report.status = "PASS";
  } catch (error) {
    report.error = String(error.stack || error); process.exitCode = 1;
  } finally {
    if (browser) await browser.close();
    fs.writeFileSync(path.join(output, "result.json"), JSON.stringify(report, null, 2));
    console.log(JSON.stringify({ status: report.status, actions: report.actions, posts_blocked: report.posts_blocked, error: report.error }));
  }
})();
