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
const DEBOUNCE_MS = 250;

const CONFIG_FIELDS = [
  { id: "max_velocity", required: true },
  { id: "max_accel", required: true },
  { id: "square_corner_velocity", required: true },
  { id: "max_jerk", required: true },
  { id: "max_path_deviation", required: false },
  { id: "max_accel_deviation", required: false },
  { id: "max_extrude_only_velocity", required: false },
  { id: "max_extrude_only_accel", required: false },
];

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

// -- Persistence ---------------------------------------------------------------
function captureState() {
  const config = {};
  for (const f of CONFIG_FIELDS) {
    config[f.id] = document.getElementById(`cfg-${f.id}`).value;
  }
  config.post_processor_config = document.getElementById("cfg-post_processor_config").value;
  return { gcode: document.getElementById("gcode").value, config };
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
}

function saveState() {
  slots[activeSlot].state = captureState();
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({
      active: activeSlot,
      slots: { A: slots.A.state, B: slots.B ? slots.B.state : null },
    }));
  } catch (e) { /* quota / private mode — persistence is best-effort */ }
  syncPresetControls();
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
  refreshPresetSelect(name);
}

function refreshPresetSelect(selected) {
  const sel = document.getElementById("preset-select");
  const presets = loadPresets();
  sel.replaceChildren(new Option("Presets…", ""));
  for (const name of Object.keys(presets).sort()) {
    sel.add(new Option(name, name));
  }
  sel.value = selected && presets[selected] ? selected : "";
  syncPresetControls();
}

// The Save button reflects where the current state stands against the active
// preset: disabled when in sync, "Save ●" when the state has drifted.
function syncPresetControls() {
  const name = document.getElementById("preset-select").value;
  const saveBtn = document.getElementById("preset-save");
  document.getElementById("preset-delete").disabled = name === "";
  if (name === "") {
    saveBtn.disabled = true;
    saveBtn.textContent = "Save";
    return;
  }
  const dirty =
    JSON.stringify(loadPresets()[name]) !== JSON.stringify(captureState());
  saveBtn.disabled = !dirty;
  saveBtn.textContent = dirty ? "Save ●" : "Save";
}

function savePreset() {
  const name = document.getElementById("preset-select").value;
  if (name === "") {
    saveAsPreset();
    return;
  }
  const presets = loadPresets();
  presets[name] = captureState();
  storePresets(presets);
  setActivePreset(name);
}

function saveAsPreset() {
  const name = prompt("Preset name:", activePresetName());
  if (name == null) return;
  const trimmed = name.trim();
  if (trimmed === "") return;
  const presets = loadPresets();
  presets[trimmed] = captureState();
  storePresets(presets);
  setActivePreset(trimmed);
}

function deletePreset() {
  const name = document.getElementById("preset-select").value;
  if (name === "") return;
  const presets = loadPresets();
  delete presets[name];
  storePresets(presets);
  setActivePreset("");
}

function onPresetSelected() {
  const name = document.getElementById("preset-select").value;
  setActivePreset(name);
  if (name === "") return;
  const preset = loadPresets()[name];
  if (preset) {
    applyState(preset);
    invalidateActiveSlotPlan();
    requestPlan();
  }
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
  applyState({ gcode: payload.gcode, config: payload.config });
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

// -- Gcode pane collapse ---------------------------------------------------------
function setGcodeCollapsed(on) {
  document.querySelector(".app").classList.toggle("gcode-collapsed", on);
  const btn = document.getElementById("gcode-collapse");
  btn.textContent = on ? "⟩ gcode" : "⟨";
  btn.title = on ? "Expand the gcode pane" : "Collapse the gcode pane";
  try {
    localStorage.setItem(GCODE_COLLAPSED_KEY, on ? "1" : "0");
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

  document.getElementById("reset-everything").addEventListener("click", () => {
    localStorage.clear();
    location.reload();
  });
  refreshPresetSelect(activePresetName());
  document.getElementById("preset-select").addEventListener("change", onPresetSelected);
  document.getElementById("preset-save").addEventListener("click", savePreset);
  document.getElementById("preset-saveas").addEventListener("click", saveAsPreset);
  document.getElementById("preset-delete").addEventListener("click", deletePreset);
  initCases();

  setGcodeCollapsed(localStorage.getItem(GCODE_COLLAPSED_KEY) === "1");
  document.getElementById("gcode-collapse").addEventListener("click", () => {
    setGcodeCollapsed(!document.querySelector(".app").classList.contains("gcode-collapsed"));
  });

  document.getElementById("slot-a").addEventListener("click", (e) => onSlotClick("A", e));
  document.getElementById("slot-b").addEventListener("click", (e) => onSlotClick("B", e));
  document.getElementById("reset-zoom").addEventListener("click", () => view.resetZoom());
  document.getElementById("toggle-peaks").addEventListener("click", (e) => {
    e.target.classList.toggle("active", !view.showPeaks);
    view.setShowPeaks(!view.showPeaks);
  });

  document.addEventListener("keydown", (e) => {
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
