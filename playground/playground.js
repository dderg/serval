// Interactive playground: paste gcode, tweak the planner config, watch the
// real pipeline re-plan live. Planning runs in a worker (playground-worker.js
// wrapping the motion-playground WASM); rendering is the shared
// TrajectoryView. Two slots (A/B) each hold a full gcode+config state:
// spacebar flips between them Lightroom-style, with the inactive slot's plan
// drawn as the ghost for comparison.
import { TrajectoryView, initWasm, setupSplitter } from "./trajectory-view.js";

const STORAGE_KEY = "motionPlayground.state";
const PRESETS_KEY = "motionPlayground.presets";
const ACTIVE_PRESET_KEY = "motionPlayground.activePreset";
const GCODE_COLLAPSED_KEY = "motionPlayground.gcodeCollapsed";
const SETTINGS_COLLAPSED_KEY = "motionPlayground.settingsCollapsed";
const DEBOUNCE_MS = 250;

const CONFIG_FIELDS = [
  { id: "max_velocity", required: true },
  { id: "max_accel", required: true },
  // Exactly one of corner_deviation (canonical) / square_corner_velocity
  // (legacy alias) must be set — the planner validates and reports it.
  { id: "corner_deviation", required: false },
  { id: "square_corner_velocity", required: false },
  { id: "max_jerk", required: true },
  { id: "max_path_deviation", required: false },
  { id: "max_accel_deviation", required: false },
  { id: "max_extrude_only_velocity", required: false },
  { id: "max_extrude_only_accel", required: false },
];

const SIM_FIELDS = ["sim-x-freq", "sim-x-zeta", "sim-y-freq", "sim-y-zeta"];
const SIM_DEFAULT_ZETA = 0.1;

// A real print excerpt (Voron cube layer 5, from the snapshot cases), shipped
// as a sibling asset so the static bundle stays gcode-free JS. The inline
// fallback keeps the page usable if the fetch fails.
async function defaultGcode() {
  try {
    const resp = await fetch("./default.gcode");
    if (resp.ok) return await resp.text();
  } catch (e) { /* offline / stripped-down deploy — fall through */ }
  return [
    "; Motion playground — edit me. G0/G1, G90/G91, G92, M82/M83.",
    "G90",
    "G1 X0 Y0 F9000",
    "G1 X40 Y0",
    "G1 X40 Y40",
    "G1 X0 Y40",
    "G1 X0 Y0",
  ].join("\n") + "\n";
}

let view = null;
let worker = null;
let seq = 0;
let lastAppliedSeq = 0;
let debounceTimer = null;
let currentSnapshot = null; // last good plan of the active slot, as raw JSON
let lastPlanMs = null;
let firstPlan = true;

// Slot = { state: {gcode, config}, snapshot: json|null }. B starts unset;
// clicking its button copies the current state in.
const slots = { A: { state: null, snapshot: null }, B: null };
let activeSlot = "A";
const seqSlot = new Map(); // plan seq -> slot it was requested for

function otherSlot(name) {
  return name === "A" ? "B" : "A";
}

function ghostSnapshot() {
  const other = slots[otherSlot(activeSlot)];
  return other ? other.snapshot : null;
}

function newWorker() {
  return new Worker(new URL("./playground-worker.js", import.meta.url), { type: "module" });
}

// A standby worker is always kept warm (its wasm init starts at creation), so
// cancelling a slow in-flight plan swaps workers instead of paying a cold
// module load before the replacement plan can start.
let standbyWorker = null;

function spawnWorker() {
  worker?.terminate();
  worker = standbyWorker ?? newWorker();
  standbyWorker = newWorker();
  worker.onmessage = onWorkerMessage;
}

function onWorkerMessage(e) {
  const { seq: s, snapshot, planMs, partial, error } = e.data;
  if (partial) {
    if (s !== seq || seqSlot.get(s) !== activeSlot) return;
    setPathSpinner(false);
    view.setData(snapshot, ghostSnapshot(), { keepView: !firstPlan });
    return;
  }
  const forSlot = seqSlot.get(s);
  for (const k of seqSlot.keys()) if (k <= s) seqSlot.delete(k);
  if (s < lastAppliedSeq) return;
  lastAppliedSeq = s;
  if (s === seq) {
    setPlanning(false);
    setPathSpinner(false);
  }
  if (error) {
    // A wasm panic poisons the instance; a fresh worker keeps later edits
    // planning instead of failing on a dead module.
    if (/unreachable|RuntimeError/i.test(error)) spawnWorker();
    showError(error);
    return;
  }
  clearError();
  if (forSlot && slots[forSlot]) slots[forSlot].snapshot = snapshot;
  if (forSlot !== activeSlot) return;
  lastPlanMs = planMs;
  currentSnapshot = snapshot;
  view.setData(snapshot, ghostSnapshot(), { keepView: !firstPlan });
  firstPlan = false;
}

// Dense polygon gcode can take seconds to plan (clothoid fitting is the
// dominant cost); without a pending indicator a slow re-plan is
// indistinguishable from a dead page.
function setPlanning(on) {
  const el = document.getElementById("meta");
  el.classList.toggle("planning", on);
  if (on && !el.textContent) el.textContent = "planning…";
}

// The spinner covers only the gap between requesting a plan and the first
// streamed partial — once pieces start arriving, watching the path grow is
// the progress indicator.
function setPathSpinner(on) {
  document.getElementById("path-spinner").classList.toggle("on", on);
}

function showError(message) {
  const el = document.getElementById("error");
  el.textContent = message;
  el.classList.add("on");
}

function clearError() {
  document.getElementById("error").classList.remove("on");
}

function readConfig() {
  const config = {};
  for (const f of CONFIG_FIELDS) {
    const raw = document.getElementById(`cfg-${f.id}`).value.trim();
    if (raw === "") {
      if (f.required) throw new Error(`${f.id} is required`);
      continue;
    }
    const v = Number(raw);
    if (!Number.isFinite(v)) throw new Error(`${f.id}: not a number`);
    config[f.id] = f.integer ? Math.round(v) : v;
  }
  config.post_processor_config = document.getElementById("cfg-post_processor_config").value;
  return config;
}

function requestPlan() {
  let config;
  try {
    config = readConfig();
  } catch (e) {
    showError(e.message);
    return;
  }
  saveState();
  // The wasm plan blocks the worker thread, so a stale in-flight plan can only
  // be cancelled by replacing the worker — the warm standby makes that cheap.
  if (seq > lastAppliedSeq) spawnWorker();
  seq += 1;
  seqSlot.set(seq, activeSlot);
  setPlanning(true);
  setPathSpinner(true);
  worker.postMessage({ seq, gcode: document.getElementById("gcode").value, config });
}

function schedulePlan() {
  clearTimeout(debounceTimer);
  debounceTimer = setTimeout(requestPlan, DEBOUNCE_MS);
}

// -- A/B slots -------------------------------------------------------------------
function syncControls() {
  for (const name of ["A", "B"]) {
    const btn = document.getElementById(`slot-${name.toLowerCase()}`);
    btn.classList.toggle("on", activeSlot === name);
    btn.classList.toggle("empty", slots[name] == null);
    btn.title = slots[name] == null
      ? `Copy the current state into slot ${name} and switch to it`
      : activeSlot === name
        ? `Slot ${name} — active (shift-click the other slot to clear it)`
        : `Switch to slot ${name} (space)`;
  }
  updateMeta();
}

function updateMeta() {
  if (!view.data) return;
  const planTime = lastPlanMs != null ? `  planned in ${lastPlanMs.toFixed(0)}ms` : "";
  const slotTag = slots.B != null ? `[${activeSlot}]  ` : "";
  document.getElementById("meta").textContent =
    slotTag +
    `t=${view.data.traversal_time().toFixed(3)}s  ` +
    `[${view.curvatureSummary()}]  ` +
    `${view.data.point_count()} pts${planTime}`;
}

function switchSlot(target) {
  if (target === activeSlot) return;
  if (slots[target] == null) {
    slots[target] = { state: captureState(), snapshot: currentSnapshot };
  }
  slots[activeSlot].state = captureState();
  activeSlot = target;
  applyState(slots[target].state);
  syncControls();
  // A slot's state only changes while it is active, so a snapshot cached at
  // switch-away time is still the plan of exactly this state.
  if (slots[target].snapshot) {
    currentSnapshot = slots[target].snapshot;
    lastPlanMs = null;
    view.setData(currentSnapshot, ghostSnapshot(), { keepView: true });
    saveState();
  } else {
    requestPlan();
  }
}

function clearSlot(name) {
  if (name === activeSlot || slots[name] == null) return;
  slots[name] = null;
  if (currentSnapshot) view.setData(currentSnapshot, null, { keepView: true });
  syncControls();
  saveState();
}

function onSlotClick(name, event) {
  if (event.shiftKey) clearSlot(name);
  else switchSlot(name);
}

function toggleSlots() {
  if (slots.B == null) return;
  switchSlot(otherSlot(activeSlot));
}

// -- Toolhead resonance simulation (view only, never triggers a re-plan) -------
function readSimAxis(prefix) {
  const freq = Number(document.getElementById(`${prefix}-freq`).value);
  if (!Number.isFinite(freq) || freq <= 0) return null;
  const zetaRaw = document.getElementById(`${prefix}-zeta`).value.trim();
  const zeta = zetaRaw === "" ? SIM_DEFAULT_ZETA : Number(zetaRaw);
  if (!Number.isFinite(zeta) || zeta < 0 || zeta > 1) return null;
  return { freq, zeta };
}

function applySim() {
  view.setSimParams({ x: readSimAxis("sim-x"), y: readSimAxis("sim-y") });
}

function onSimInput() {
  applySim();
  saveState();
}

// -- Persistence ---------------------------------------------------------------
function captureState() {
  const config = {};
  for (const f of CONFIG_FIELDS) {
    config[f.id] = document.getElementById(`cfg-${f.id}`).value;
  }
  config.post_processor_config = document.getElementById("cfg-post_processor_config").value;
  const sim = {};
  for (const id of SIM_FIELDS) {
    sim[id] = document.getElementById(id).value;
  }
  return { gcode: document.getElementById("gcode").value, config, sim };
}

function applyState(state) {
  document.getElementById("gcode").value = state.gcode || "";
  // Always write every field: on reload the browser's form restoration
  // repopulates typed inputs, which must lose to the incoming state — and to
  // the HTML defaults after a reset or a case that omits an optional limit.
  for (const f of CONFIG_FIELDS) {
    const el = document.getElementById(`cfg-${f.id}`);
    const value = state.config?.[f.id];
    el.value = value != null ? value : f.required ? el.defaultValue : "";
  }
  const ppEl = document.getElementById("cfg-post_processor_config");
  const savedPp = state.config?.post_processor_config;
  ppEl.value = savedPp != null ? savedPp : ppEl.defaultValue;
  for (const id of SIM_FIELDS) {
    document.getElementById(id).value = state.sim?.[id] ?? "";
  }
  applySim();
}

function saveState() {
  slots[activeSlot].state = captureState();
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({
      active: activeSlot,
      slots: { A: slots.A.state, B: slots.B ? slots.B.state : null },
    }));
  } catch (e) { /* quota / private mode — persistence is best-effort */ }
  renderPresetDrawer();
}

async function restoreState() {
  let saved = null;
  try {
    saved = JSON.parse(localStorage.getItem(STORAGE_KEY));
  } catch (e) { /* corrupted — start fresh */ }
  if (saved && saved.slots) {
    slots.A = { state: saved.slots.A, snapshot: null };
    slots.B = saved.slots.B ? { state: saved.slots.B, snapshot: null } : null;
    activeSlot = saved.active === "B" && slots.B != null ? "B" : "A";
    applyState(slots[activeSlot].state || { gcode: await defaultGcode() });
  } else if (saved && (saved.gcode !== undefined || saved.config)) {
    applyState(saved); // pre-slots shape — migrate into slot A
  } else {
    applyState({ gcode: await defaultGcode() });
  }
  slots[activeSlot].state = captureState();
}

// -- Presets -------------------------------------------------------------------
function loadPresets() {
  try {
    return JSON.parse(localStorage.getItem(PRESETS_KEY)) || {};
  } catch (e) {
    return {};
  }
}

function storePresets(presets) {
  try {
    localStorage.setItem(PRESETS_KEY, JSON.stringify(presets));
  } catch (e) { /* quota / private mode — persistence is best-effort */ }
}

function activePresetName() {
  return localStorage.getItem(ACTIVE_PRESET_KEY) || "";
}

function setActivePreset(name) {
  try {
    if (name) localStorage.setItem(ACTIVE_PRESET_KEY, name);
    else localStorage.removeItem(ACTIVE_PRESET_KEY);
  } catch (e) { /* best-effort */ }
  renderPresetDrawer();
}

function drawerOpen() {
  return document.getElementById("preset-drawer").classList.contains("open");
}

function setDrawerOpen(on) {
  document.getElementById("preset-drawer").classList.toggle("open", on);
  document.getElementById("preset-toggle").classList.toggle("active", on);
  if (on) renderPresetDrawer();
}

function savePresetAs(name) {
  const presets = loadPresets();
  presets[name] = captureState();
  storePresets(presets);
  setActivePreset(name);
}

function presetRowButton(label, title, danger, onClick) {
  const btn = document.createElement("button");
  btn.textContent = label;
  btn.title = title;
  if (danger) btn.classList.add("danger");
  btn.addEventListener("click", onClick);
  return btn;
}

function renderPresetDrawer() {
  if (!drawerOpen()) return;
  const list = document.getElementById("preset-list");
  const presets = loadPresets();
  const names = Object.keys(presets).sort();
  const active = activePresetName();
  const currentJson = JSON.stringify(captureState());
  list.replaceChildren();
  if (names.length === 0) {
    const empty = document.createElement("div");
    empty.className = "empty";
    empty.textContent = "No presets yet — name the current state above and save it.";
    list.appendChild(empty);
  }
  for (const name of names) {
    const row = document.createElement("div");
    row.className = "preset-row";
    if (name === active) row.classList.add("active");
    const label = document.createElement("span");
    label.className = "pname";
    label.textContent = name;
    label.title = name;
    row.appendChild(label);
    if (name === active && JSON.stringify(presets[name]) !== currentJson) {
      const dirty = document.createElement("span");
      dirty.className = "dirty";
      dirty.title = "The current state has unsaved changes against this preset";
      dirty.textContent = "●";
      row.appendChild(dirty);
    }
    row.appendChild(presetRowButton("Load", "Replace the current state with this preset", false, () => {
      applyState(presets[name]);
      setActivePreset(name);
      invalidateActiveSlotPlan();
      requestPlan();
    }));
    row.appendChild(presetRowButton("Save", "Overwrite this preset with the current state", false, () => {
      savePresetAs(name);
    }));
    row.appendChild(presetRowButton("Delete", "Delete this preset", true, () => {
      const all = loadPresets();
      delete all[name];
      storePresets(all);
      if (activePresetName() === name) setActivePreset("");
      else renderPresetDrawer();
    }));
    list.appendChild(row);
  }
}

function syncPresetCreate() {
  const name = document.getElementById("preset-name").value.trim();
  const btn = document.getElementById("preset-create");
  btn.disabled = name === "";
  btn.textContent = name !== "" && loadPresets()[name] ? "Overwrite" : "Save";
}

function createPresetFromInput() {
  const input = document.getElementById("preset-name");
  const name = input.value.trim();
  if (name === "") return;
  savePresetAs(name);
  input.value = "";
  syncPresetCreate();
}

// Loading a preset or case rewrites the active slot's state, so its cached
// snapshot no longer matches until the re-plan lands.
function invalidateActiveSlotPlan() {
  slots[activeSlot].snapshot = null;
}

// -- Snapshot cases ------------------------------------------------------------
async function initCases() {
  const group = document.getElementById("case-group");
  let cases;
  try {
    const resp = await fetch("/api/playground/cases");
    if (!resp.ok) throw new Error(`cases: ${resp.status}`);
    cases = await resp.json();
  } catch (e) {
    // Statically hosted (no server) — the control has nothing to load.
    group.style.display = "none";
    return;
  }
  const sel = document.getElementById("case-select");
  for (const c of cases) {
    sel.add(new Option(c.name, c.name));
  }
  sel.addEventListener("change", onCaseSelected);
}

async function loadCaseIntoEditor(name) {
  const url =
    "/api/playground/case/" +
    name.split("/").map(encodeURIComponent).join("/");
  let resp;
  try {
    resp = await fetch(url);
  } catch (e) {
    showError(`failed to load case ${name}: ${e.message}`);
    return false;
  }
  if (!resp.ok) {
    const err = await resp.json().catch(() => ({}));
    showError(err.error || `failed to load case ${name}`);
    return false;
  }
  const payload = await resp.json();
  // A case brings gcode+config; the sim overlay is a view preference and
  // survives the load.
  applyState({ gcode: payload.gcode, config: payload.config, sim: captureState().sim });
  setActivePreset("");
  invalidateActiveSlotPlan();
  requestPlan();
  return true;
}

async function onCaseSelected() {
  const sel = document.getElementById("case-select");
  const name = sel.value;
  sel.value = "";
  if (name === "") return;
  await loadCaseIntoEditor(name);
}

// -- Pane collapsing ---------------------------------------------------------
function setGcodeCollapsed(on) {
  document.querySelector(".app").classList.toggle("gcode-collapsed", on);
  const btn = document.getElementById("gcode-collapse");
  btn.textContent = on ? "⟩ gcode" : "⟨";
  btn.title = on ? "Expand the gcode pane" : "Collapse the gcode pane";
  try {
    localStorage.setItem(GCODE_COLLAPSED_KEY, on ? "1" : "0");
  } catch (e) { /* best-effort */ }
}

function setSettingsCollapsed(on) {
  document.querySelector(".left-col").classList.toggle("settings-collapsed", on);
  const btn = document.getElementById("settings-collapse");
  btn.textContent = on ? "⌄ settings" : "⌃";
  btn.title = on ? "Expand the settings pane" : "Collapse the settings pane";
  try {
    localStorage.setItem(SETTINGS_COLLAPSED_KEY, on ? "1" : "0");
  } catch (e) { /* best-effort */ }
}

// -- Init ------------------------------------------------------------------------
async function main() {
  await initWasm();

  view = new TrajectoryView({ hiddenSeriesKey: "motionPlayground.hiddenSeries" });
  view.onChanged = syncControls;
  setupSplitter("motionPlayground.pathSplit");
  spawnWorker();
  await restoreState();

  document.getElementById("gcode").addEventListener("input", schedulePlan);
  for (const f of CONFIG_FIELDS) {
    document.getElementById(`cfg-${f.id}`).addEventListener("input", schedulePlan);
  }
  document.getElementById("cfg-post_processor_config").addEventListener("input", schedulePlan);
  for (const id of SIM_FIELDS) {
    document.getElementById(id).addEventListener("input", onSimInput);
  }

  document.getElementById("reset-everything").addEventListener("click", () => {
    localStorage.clear();
    location.reload();
  });
  document.getElementById("preset-toggle").addEventListener("click", () => setDrawerOpen(!drawerOpen()));
  document.getElementById("drawer-close").addEventListener("click", () => setDrawerOpen(false));
  document.getElementById("preset-create").addEventListener("click", createPresetFromInput);
  const presetName = document.getElementById("preset-name");
  presetName.addEventListener("input", syncPresetCreate);
  presetName.addEventListener("keydown", (e) => {
    if (e.key === "Enter") createPresetFromInput();
    if (e.key === "Escape") setDrawerOpen(false);
  });
  initCases();

  setGcodeCollapsed(localStorage.getItem(GCODE_COLLAPSED_KEY) === "1");
  document.getElementById("gcode-collapse").addEventListener("click", () => {
    setGcodeCollapsed(!document.querySelector(".app").classList.contains("gcode-collapsed"));
  });
  setSettingsCollapsed(localStorage.getItem(SETTINGS_COLLAPSED_KEY) === "1");
  document.getElementById("settings-collapse").addEventListener("click", () => {
    setSettingsCollapsed(!document.querySelector(".left-col").classList.contains("settings-collapsed"));
  });

  document.getElementById("slot-a").addEventListener("click", (e) => onSlotClick("A", e));
  document.getElementById("slot-b").addEventListener("click", (e) => onSlotClick("B", e));
  document.getElementById("reset-zoom").addEventListener("click", () => view.resetZoom());

  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
      setDrawerOpen(false);
      return;
    }
    if (e.target.tagName === "TEXTAREA" || e.target.tagName === "INPUT") return;
    if (e.key === " " || e.key === "b" || e.key === "B") {
      e.preventDefault();
      toggleSlots();
    }
  });

  syncControls();

  // ?case=<name> deep-links from the snapshot viewer's "Playground" button.
  // The param is consumed once — after that the page follows saved state.
  const caseParam = new URLSearchParams(window.location.search).get("case");
  if (caseParam) {
    history.replaceState(null, "", window.location.pathname);
    if (await loadCaseIntoEditor(caseParam)) return;
  }
  requestPlan();
}

main();
