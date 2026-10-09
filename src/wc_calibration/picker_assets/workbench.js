/* One frozen dataset, one page, explicit transitions; no browser-side calibration activation. */
(function (root, factory) {
  "use strict";
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root && root.document) {
    root.WorkbenchUI = api;
    const run = () => { if (document.body.dataset.page === "workbench") api.start(); };
    if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", run); else run();
  }
})(typeof window !== "undefined" ? window : null, function () {
  "use strict";
  const copy = value => JSON.parse(JSON.stringify(value));
  function assertSameSource(scene, identity) {
    if (!identity || identity.input_hash !== scene.input_hash || identity.scene_id !== scene.scene_id)
      throw new Error("结果不属于当前数据和场景，未传入下一步。");
  }
  class WorkflowState {
    constructor() { this.revision = 0; this.manual = null; this.active = false; this.stale = false; this.icp = null; this.pairs = 0; }
    selectionChanged(count) { this.revision++; this.pairs = count; this.manual = null; this.stale = this.active; }
    calculated(scene, payload) {
      const p = payload.selection && payload.selection.provenance;
      assertSameSource(scene, {input_hash: p && p.prepared_input_hash, scene_id: p && p.scene_id});
      if (!payload.result || payload.result.live_eligible !== false) throw new Error("缺少候选状态，未进入下一步。");
      this.manual = copy(payload); this.manualRevision = this.revision;
    }
    acceptInitial() {
      if (!this.manual || this.manualRevision !== this.revision) throw new Error("选点已变化，请重新计算当前初值。");
      this.active = true; this.stale = false; this.icp = null;
    }
    restore() { this.active = true; this.stale = false; this.icp = null; }
  }
  function initialBootstrap(payload) {
    const prepared = payload.prepared_initial_T_left_right;
    return {...payload, initial_T_left_right: prepared || [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
      initial_source: {kind: prepared ? "PREPARED_INPUT_INITIAL" : "identity_for_manual_adjustment_only",
        status: "NOT_ACCEPTED_FOR_ALIGNMENT", path: null}, initial_assessment: null};
  }
  const selectManagers = new WeakMap();
  function enhanceSelects(doc = document) {
    if (selectManagers.has(doc)) return selectManagers.get(doc);
    const win = doc.defaultView || window, entries = [];
    let active = null, pending = false, sequence = 0;
    const queue = win.queueMicrotask ? fn => win.queueMicrotask(fn) : fn => Promise.resolve().then(fn);
    const disabled = select => select.disabled || select.matches(":disabled");
    const visibleOption = option => !!option && !option.hidden && !(option.parentElement && option.parentElement.tagName === "OPTGROUP" && option.parentElement.hidden);
    const enabledOption = option => visibleOption(option) && !option.disabled && !(option.parentElement && option.parentElement.tagName === "OPTGROUP" && option.parentElement.disabled);
    function nameOf(select) {
      const explicit = select.getAttribute("aria-label");
      if (explicit) return explicit;
      const references = select.getAttribute("aria-labelledby");
      if (references) {
        const text = references.split(/\s+/).map(id => doc.getElementById(id)?.textContent || "").join(" ").trim();
        if (text) return text;
      }
      const labels = Array.from(select.labels || []).map(label => {
        const copy = label.cloneNode(true); copy.querySelectorAll("select,button,.w-select").forEach(child => child.remove());
        return copy.textContent.trim();
      }).filter(Boolean);
      return labels.join(" ") || select.title || "选择选项";
    }
    function focus(entry) { if (entry.trigger.isConnected && !entry.trigger.disabled) entry.trigger.focus({ preventScroll: true }); }
    function close(restoreFocus = false) {
      if (!active) return;
      const entry = active; active = null; entry.typeText = "";
      entry.trigger.setAttribute("aria-expanded", "false"); entry.trigger.removeAttribute("aria-activedescendant");
      if (entry.menu) entry.menu.remove(); entry.menu = null; entry.items = [];
      if (restoreFocus) focus(entry);
    }
    function viewport() {
      const visual = win.visualViewport;
      return {left: visual ? visual.offsetLeft : 0, top: visual ? visual.offsetTop : 0,
        width: visual ? visual.width : win.innerWidth || doc.documentElement.clientWidth,
        height: visual ? visual.height : win.innerHeight || doc.documentElement.clientHeight};
    }
    function place(entry) {
      if (active !== entry || !entry.menu) return;
      if (!entry.select.isConnected || disabled(entry.select)) { close(false); return; }
      const rect = entry.trigger.getBoundingClientRect(), bounds = viewport(), margin = 8, gap = 5;
      const leftEdge = bounds.left + margin, topEdge = bounds.top + margin;
      const rightEdge = bounds.left + bounds.width - margin, bottomEdge = bounds.top + bounds.height - margin;
      if (rect.bottom < bounds.top || rect.top > bounds.top + bounds.height || rect.right < bounds.left || rect.left > bounds.left + bounds.width) { close(false); return; }
      const availableWidth = Math.max(1, bounds.width - 2 * margin);
      const style = entry.menu.style;
      // Let the longest option determine the popup width; only narrow viewports
      // should force truncation. Keep the trigger/minimum width as a lower bound.
      style.position = "fixed"; style.width = "max-content";
      style.minWidth = Math.min(Math.max(rect.width, 180), availableWidth) + "px";
      style.maxWidth = availableWidth + "px";
      const wanted = Math.min(320, entry.menu.scrollHeight || 320);
      const below = bottomEdge - rect.bottom - gap, above = rect.top - topEdge - gap;
      const upward = below < wanted && above > below;
      const room = Math.max(1, Math.min(320, upward ? above : below, bounds.height - 2 * margin));
      style.maxHeight = room + "px";
      // Measure after applying the height limit so a scrollbar is included in
      // the actual border-box width before clamping against the viewport edge.
      const width = entry.menu.getBoundingClientRect().width;
      style.left = Math.max(leftEdge, Math.min(rect.left, rightEdge - width)) + "px";
      const height = Math.min(entry.menu.scrollHeight || room, room);
      style.top = Math.max(topEdge, Math.min(upward ? rect.top - gap - height : rect.bottom + gap, bottomEdge - height)) + "px";
    }
    function enabledIndices(entry) { return Array.from(entry.select.options).flatMap((option, index) => enabledOption(option) ? [index] : []); }
    function highlight(entry, index, reveal = true) {
      if (active !== entry || !entry.menu) return;
      entry.activeIndex = index;
      for (const item of entry.items) {
        const selected = Number(item.dataset.index) === index;
        item.classList.toggle("is-active", selected); item.dataset.active = String(selected);
      }
      const item = entry.items.find(item => Number(item.dataset.index) === index);
      if (!item) { entry.trigger.removeAttribute("aria-activedescendant"); return; }
      entry.trigger.setAttribute("aria-activedescendant", item.id);
      if (reveal) {
        const itemRect = item.getBoundingClientRect(), menuRect = entry.menu.getBoundingClientRect();
        if (itemRect.top < menuRect.top) entry.menu.scrollTop -= menuRect.top - itemRect.top;
        else if (itemRect.bottom > menuRect.bottom) entry.menu.scrollTop += itemRect.bottom - menuRect.bottom;
      }
    }
    function commit(entry, index, expectedOption = null) {
      if (active !== entry || disabled(entry.select) || !enabledOption(entry.select.options[index]) || (expectedOption && entry.select.options[index] !== expectedOption)) return;
      const changed = entry.select.selectedIndex !== index;
      entry.select.selectedIndex = index; close(true);
      if (changed) {
        entry.select.dispatchEvent(new win.Event("input", {bubbles:true}));
        entry.select.dispatchEvent(new win.Event("change", {bubbles:true}));
      }
      // Existing handlers may reject a change and restore the native value.
      sync();
    }
    function menuContents(entry) {
      entry.menu.replaceChildren(); entry.items = []; let lastGroup = null;
      Array.from(entry.select.options).forEach((option, index) => {
        if (!visibleOption(option)) return;
        const group = option.parentElement.tagName === "OPTGROUP" ? option.parentElement : null;
        if (group && group !== lastGroup) {
          const heading = doc.createElement("div"); heading.className = "w-select-group"; heading.textContent = group.label; heading.setAttribute("role", "presentation"); entry.menu.append(heading);
        }
        lastGroup = group;
        const item = doc.createElement("div"), label = doc.createElement("span"), check = doc.createElement("span");
        const selected = index === entry.select.selectedIndex, unavailable = !enabledOption(option);
        item.id = entry.select.id + "-option-" + index; item.className = "w-select-option"; item.dataset.index = String(index);
        item.setAttribute("role", "option"); item.setAttribute("aria-selected", String(selected)); item.setAttribute("aria-disabled", String(unavailable));
        item.classList.toggle("is-selected", selected); item.classList.toggle("is-disabled", unavailable);
        label.className = "w-select-option-label"; label.textContent = option.label;
        check.className = "w-select-check"; check.textContent = selected ? "✓" : ""; check.setAttribute("aria-hidden", "true");
        item.append(label, check); entry.menu.append(item); entry.items.push(item);
        item.addEventListener("pointermove", () => { if (enabledOption(option)) highlight(entry, index, false); });
        item.addEventListener("pointerdown", event => { if (event.button === 0) event.preventDefault(); });
        item.addEventListener("click", () => commit(entry, index, option));
      });
    }
    function syncEntry(entry) {
      if (!entry.select.isConnected) { if (active === entry) close(false); return; }
      const select = entry.select, option = select.options[select.selectedIndex], name = nameOf(select);
      const text = option ? option.label : "请选择", unavailable = disabled(select);
      const signature = JSON.stringify([select.selectedIndex, unavailable, name, select.title, select.getAttribute("aria-describedby"),
        Array.from(select.options, option => [option.label, option.value, option.disabled, option.hidden,
          option.parentElement.tagName === "OPTGROUP" ? [option.parentElement.label, option.parentElement.disabled, option.parentElement.hidden] : null])]);
      if (signature === entry.signature) return;
      entry.signature = signature; entry.label.textContent = text; entry.trigger.disabled = unavailable;
      entry.trigger.setAttribute("aria-label", name + "：" + text); entry.trigger.title = select.title || "";
      entry.wrapper.classList.toggle("is-disabled", unavailable);
      const description = select.getAttribute("aria-describedby");
      if (description) entry.trigger.setAttribute("aria-describedby", description); else entry.trigger.removeAttribute("aria-describedby");
      if (active === entry) {
        if (unavailable) { close(false); return; }
        const indices = enabledIndices(entry), previous = entry.activeIndex;
        menuContents(entry); place(entry); highlight(entry, indices.includes(previous) ? previous : indices.includes(select.selectedIndex) ? select.selectedIndex : indices[0] ?? -1);
      }
    }
    function sync() { for (const entry of entries) syncEntry(entry); }
    function scheduleSync() { if (!pending) { pending = true; queue(() => { pending = false; sync(); }); } }
    function open(entry) {
      sync(); if (disabled(entry.select)) return false;
      if (active === entry) return true;
      close(false); active = entry; entry.typeText = ""; entry.typeTime = 0;
      const menu = doc.createElement("div"); menu.className = "w-select-menu"; menu.id = entry.select.id + "-listbox";
      menu.setAttribute("role", "listbox"); menu.setAttribute("aria-label", nameOf(entry.select)); entry.menu = menu;
      doc.body.append(menu); menuContents(entry); entry.trigger.setAttribute("aria-expanded", "true"); focus(entry);
      place(entry); const indices = enabledIndices(entry);
      highlight(entry, indices.includes(entry.select.selectedIndex) ? entry.select.selectedIndex : indices[0] ?? -1);
      return true;
    }
    function move(entry, key, wasOpen) {
      const indices = enabledIndices(entry); if (!indices.length) return;
      const current = indices.indexOf(entry.activeIndex);
      const index = key === "Home" ? indices[0] : key === "End" ? indices.at(-1) : !wasOpen ?
        (current >= 0 ? entry.activeIndex : key === "ArrowUp" ? indices.at(-1) : indices[0]) :
        indices[Math.max(0, Math.min(indices.length - 1, current + (key === "ArrowUp" ? -1 : 1)))];
      highlight(entry, index);
    }
    function search(entry, character) {
      const now = Date.now(); entry.typeText = now - entry.typeTime > 700 ? character : entry.typeText + character; entry.typeTime = now;
      const repeated = [...entry.typeText].every(letter => letter === character), query = (repeated ? character : entry.typeText).toLocaleLowerCase();
      const indices = enabledIndices(entry), current = indices.indexOf(entry.activeIndex);
      const start = repeated ? current + 1 : Math.max(0, current);
      for (let offset = 0; offset < indices.length; offset++) {
        const index = indices[(start + offset + indices.length) % indices.length];
        if (entry.select.options[index].label.trim().toLocaleLowerCase().startsWith(query)) { highlight(entry, index); break; }
      }
    }
    for (const select of doc.querySelectorAll("select")) {
      if (select.multiple || select.size > 1) continue; // Multi-selection retains its native interaction contract.
      if (!select.id) { let id; do { id = "w-native-select-" + (++sequence); } while (doc.getElementById(id)); select.id = id; }
      const wrapper = doc.createElement("span"), trigger = doc.createElement("button"), label = doc.createElement("span"), arrow = doc.createElement("span");
      const labels = Array.from(select.labels || []);
      wrapper.className = "w-select"; trigger.className = "w-select-trigger"; trigger.id = select.id + "-trigger"; trigger.type = "button";
      trigger.setAttribute("role", "combobox"); trigger.setAttribute("aria-haspopup", "listbox"); trigger.setAttribute("aria-expanded", "false"); trigger.setAttribute("aria-controls", select.id + "-listbox");
      label.className = "w-select-label"; arrow.className = "w-select-arrow"; arrow.textContent = "⌄"; arrow.setAttribute("aria-hidden", "true"); trigger.append(label, arrow);
      select.before(wrapper); wrapper.append(select, trigger); select.classList.add("w-select-native"); select.hidden = true; select.tabIndex = -1; select.setAttribute("aria-hidden", "true");
      const entry = {select, wrapper, trigger, label, menu:null, items:[], activeIndex:-1, signature:null, typeText:"", typeTime:0}; entries.push(entry);
      trigger.addEventListener("click", () => { if (active === entry) close(false); else open(entry); });
      trigger.addEventListener("keydown", event => {
        if (disabled(select)) return;
        const key = event.key, wasOpen = active === entry;
        if (key === "Tab") { if (wasOpen) close(false); return; }
        if (key === "Escape") { if (wasOpen) { event.preventDefault(); event.stopPropagation(); close(true); } return; }
        if (["ArrowDown", "ArrowUp", "Home", "End"].includes(key)) {
          event.preventDefault(); if (open(entry)) move(entry, key, wasOpen); return;
        }
        if (key === "Enter" || key === " ") {
          event.preventDefault(); if (!wasOpen) open(entry); else if (entry.activeIndex >= 0) commit(entry, entry.activeIndex); return;
        }
        if (!event.ctrlKey && !event.metaKey && !event.altKey && [...key].length === 1) { event.preventDefault(); if (open(entry)) search(entry, key); }
      });
      for (const labelElement of labels) labelElement.addEventListener("click", event => {
        if (event.target.closest("button,a,input,select,textarea")) return;
        event.preventDefault(); focus(entry);
      });
      for (const type of ["input", "change"]) select.addEventListener(type, scheduleSync);
      if (win.MutationObserver) {
        entry.observer = new win.MutationObserver(scheduleSync);
        entry.observer.observe(select, {attributes:true, childList:true, characterData:true, subtree:true,
          attributeFilter:["disabled", "selected", "value", "label", "hidden", "title", "aria-label", "aria-labelledby", "aria-describedby"]});
      }
    }
    doc.addEventListener("pointerdown", event => { if (active && !active.trigger.contains(event.target) && !active.menu.contains(event.target)) close(false); }, true);
    doc.addEventListener("focusin", event => { if (active && !active.trigger.contains(event.target) && !active.menu.contains(event.target)) close(false); });
    doc.addEventListener("scroll", event => { if (active && !active.menu.contains(event.target)) place(active); }, true);
    win.addEventListener("resize", () => { if (active) place(active); });
    if (win.visualViewport) for (const type of ["resize", "scroll"]) win.visualViewport.addEventListener(type, () => { if (active) place(active); });
    if (win.MutationObserver) {
      const fieldsets = new win.MutationObserver(records => { if (records.some(record => record.target.tagName === "FIELDSET")) scheduleSync(); });
      fieldsets.observe(doc.body, {attributes:true, subtree:true, attributeFilter:["disabled"]});
    }
    const manager = {sync, close, count:entries.length}; selectManagers.set(doc, manager); sync(); return manager;
  }
  function start() {
    const $ = id => document.getElementById(id), flow = new WorkflowState();
    const selects = enhanceSelects(document);
    let scene = null, picker = null, alignment = null, pickerBusy = false, alignmentBusy = false;
    let poseDirty = false, fatal = false, message = "正在读取同一组冻结点云…", lockPicker = null, lockAlign = null;
    const busy = () => pickerBusy || alignmentBusy;
    function notify(text) { message = text; render(); }
    function node(tag, text, className) { const element = document.createElement(tag); element.textContent = text; if (className) element.className = className; return element; }
    function row(table, label, value) { const tr = document.createElement("tr"); tr.append(node("th", label), node("td", value)); table.append(tr); }
    function summary() {
      const output = $("workflow-summary"); if (!output) return;
      output.replaceChildren();
      const status = flow.stale ? "选点已变化，下游结果已过期" : poseDirty ? "姿态已调整，请重新计算" : flow.icp ?
        (flow.icp.saved ? "结果已保存为离线候选" : "已计算，尚未保存") : flow.manual ? "人工初值已计算" : "等待计算结果";
      output.append(node("h3", status));
      const table = document.createElement("table"), tbody = document.createElement("tbody"); table.append(tbody);
      row(tbody, "当前点对", flow.pairs + " 对");
      if (flow.manual) {
        const r = flow.manual.result;
        row(tbody, "人工拟合", "RMSE " + (r.rmse_m*100).toFixed(2) + " cm · 最大 " + (r.max_residual_m*100).toFixed(2) + " cm");
        row(tbody, "初值结论", flow.manual.assessment ? flow.manual.assessment.summary : "尚未独立验证");
        row(tbody, "初值留存", flow.manual.saved ? flow.manual.export_dir : "只计算，未保存");
      }
      if (flow.icp) {
        const r = flow.icp.result, m = r.final_metrics || {};
        row(tbody, "ICP / 当前姿态", flow.stale || poseDirty ? "以下为旧结果，不能代表当前输入" : (flow.icp.assessment ? flow.icp.assessment.summary : r.status));
        row(tbody, "最近邻 RMSE", Number.isFinite(m.nn_rmse_m) ? (m.nn_rmse_m*100).toFixed(2) + " cm（仅门限内匹配）" : "无有效指标");
        row(tbody, "候选留存", flow.icp.saved ? flow.icp.export_dir : "只计算，未保存");
      }
      row(tbody, "正式使用", "未启用；实时建图与录包融合配置不变");
      output.append(table);
    }
    function render() {
      $("workflow-status").textContent = message;
      $("workflow-status").dataset.state = fatal ? "error" : busy() ? "busy" : flow.stale ? "stale" : "ready";
      $("use-initial").disabled = fatal || busy() || !flow.manual || flow.manualRevision !== flow.revision;
      $("load-saved-candidate").disabled = fatal || !scene || busy();
      const p = alignmentBusy || fatal, a = pickerBusy || flow.stale || fatal;
      if (picker && p !== lockPicker) { lockPicker = p; picker.setExternalBusy(p); }
      if (alignment && a !== lockAlign) { lockAlign = a; alignment.setExternalBusy(a); }
      if ($("summary-pairs")) $("summary-pairs").textContent = flow.pairs + " 对";
      if ($("summary-state")) $("summary-state").textContent = flow.stale ? "结果已过期" : busy() ? "计算中" : flow.icp ? "候选待复核" : flow.manual ? "初值已计算" : "等待选点";
      summary();
      selects.sync();
    }
    const bootstrap = fetch("/api/alignment-bootstrap", {credentials:"same-origin", cache:"no-store"})
      .then(async response => { const payload = await response.json(); if (!response.ok || payload.error) throw new Error(payload.error || "无法读取场景");
        if (!payload.scene.capabilities || !payload.scene.capabilities.unified_workbench || !payload.scene.capabilities.alignment_save_result)
          throw new Error("当前仍是旧网页服务，请使用新版服务地址打开此工作台。");
        scene = window.PickerUI.validateScene(payload.scene); return initialBootstrap(payload);
      });
    picker = window.PickerUI.start({unified:true, light:true, scene:bootstrap.then(p => p.scene),
      onReady() { notify("先在左右图中选取同一物理位置，再向下计算初值。"); },
      onSelectionChange(count) { flow.selectionChanged(count); notify(flow.stale ? "选点已变化，先重新计算初值并传入下一步；旧 ICP 结果已过期。" : "已选择 " + count + " 对；完成选点后计算初值。"); },
      onBusy(value) { pickerBusy = value; if (value) message = "正在计算或保存人工初值…"; render(); },
      onResult(payload) { try { flow.calculated(scene, payload); notify("初值计算完成。查看残差后，点击“用此初值继续调整”；无需先保存。"); } catch (error) { notify(error.message); } }
    });
    alignment = window.AlignmentUI.start(window.PickerUI, {prefix:"a-", unified:true, light:true, bootstrap,
      onBusy(value) { alignmentBusy = value; if (value) message = "正在处理当前配准结果，请稍候…"; render(); },
      onChange(state) { poseDirty = !!state.dirty; render(); },
      onResult(payload) { flow.icp = copy(payload); poseDirty = false; notify(payload.saved ? "当前结果已保存为候选，未启用正式外参。" : "配准计算完成，向下查看结论；需要留存时再保存本次结果。"); }
    });
    $("use-initial").addEventListener("click", async () => {
      if (fatal || busy() || !flow.manual) return;
      const prior = {active:flow.active,stale:flow.stale,icp:flow.icp};
      try {
        await alignment.ready;
        flow.acceptInitial(); render();
        alignment.setInitial({scene, input_hash:scene.input_hash, scene_id:scene.scene_id,
          result:flow.manual.result, assessment:flow.manual.assessment, saved:flow.manual.saved, export_dir:flow.manual.export_dir});
        poseDirty = false;
        notify("当前初值已传入下方，未写入正式配置。可调整右云，或直接只计算 ICP。");
        $("refine-step").scrollIntoView({behavior:"smooth",block:"start"});
      } catch (error) { Object.assign(flow,prior); notify(error.message); }
    });
    $("load-saved-candidate").addEventListener("click", async () => {
      if (fatal || busy()) return;
      const oldStale = flow.stale;
      try {
        await alignment.ready; flow.stale = false; render();
        const loaded = await alignment.loadSavedCandidate();
        if (!loaded) { flow.stale = oldStale; notify("当前场景没有可恢复的候选，继续选点并计算即可。"); return; }
        // The controller emits onResult for a loaded saved result; keep it in the summary.
        const restored = flow.icp; flow.restore(); flow.icp = restored; poseDirty = false;
        notify("已明确恢复当前场景的历史候选，仅用于本页复查。新的人工初值仍需点击传入。");
      } catch (error) { flow.stale = oldStale; notify(error.message); }
    });
    Promise.all([picker.ready, alignment.ready]).then(loaded => { if (!scene || !loaded[0] || !loaded[1]) throw new Error("场景尚未就绪"); render(); })
      .catch(error => { fatal = true; notify("加载失败：" + error.message); });
    bootstrap.catch(error => { fatal = true; notify("加载失败：" + error.message); });
    document.querySelectorAll('.step-nav a[href^="#"]').forEach(link => link.addEventListener("click", () => {
      document.querySelectorAll(".step-nav a").forEach(a => a.removeAttribute("aria-current")); link.setAttribute("aria-current", "step");
    }));
    if (typeof IntersectionObserver !== "undefined") {
      const observer = new IntersectionObserver(entries => entries.forEach(entry => { if (entry.isIntersecting) {
        document.querySelectorAll(".step-nav a").forEach(link => { if (link.hash === "#"+entry.target.id) link.setAttribute("aria-current","step"); else link.removeAttribute("aria-current"); });
      }}), {rootMargin:"-100px 0px -60% 0px", threshold:0});
      for (const id of ["input-step","initial-step","refine-step","review-step"]) observer.observe($(id));
    }
    render();
  }
  return {WorkflowState, assertSameSource, initialBootstrap, enhanceSelects, start};
});
