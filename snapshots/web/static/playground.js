// Interactive playground: paste gcode, tweak the planner config, watch the
// real pipeline re-plan live. Planning runs in a worker (playground-worker.js
// wrapping the motion-playground WASM); rendering is the shared
// TrajectoryView. "Pin baseline" freezes the current plan as the before
// variant so config changes can be A/B-flipped exactly like snapshot review.
import { TrajectoryView, initWasm, setupSplitter } from "./trajectory-view.js";

const STORAGE_KEY = "motionPlayground.state";
const DEBOUNCE_MS = 250;

const CONFIG_FIELDS = [
  { id: "max_velocity", required: true },
  { id: "max_accel", required: true },
  { id: "square_corner_velocity", required: true },
  { id: "max_jerk", required: true },
  { id: "max_path_deviation", required: false },
  { id: "max_accel_deviation", required: false },
  { id: "arc_fit", required: false, integer: true },
  { id: "max_extrude_only_velocity", required: false },
  { id: "max_extrude_only_accel", required: false },
];

function defaultGcode() {
  const lines = [
    "; Motion playground — edit me. G0/G1, G90/G91, G92, M82/M83.",
    "G90",
    "G1 X0 Y0 F9000",
    "G1 X40 Y0",
    "G1 X40 Y40",
    "G1 X0 Y40",
    "G1 X0 Y0",
    "; faceted arc — [arc_fit] fuses these 24 segments into one true arc;",
    "; clear min_run_facets to see the raw clothoid-per-corner version",
    "; (the fit budget derives from square_corner_velocity² / max_accel)",
    "G0 X65 Y20",
  ];
  const n = 96;
  for (let k = 1; k <= n / 4; k++) {
    const a = (2 * Math.PI * k) / n;
    const x = 60 + 5 * Math.cos(a);
    const y = 20 + 5 * Math.sin(a);
    lines.push(`G1 X${x.toFixed(3)} Y${y.toFixed(3)}`);
  }
  return lines.join("\n") + "\n";
}

let view = null;
let worker = null;
let seq = 0;
let lastAppliedSeq = 0;
let debounceTimer = null;
let currentSnapshot = null; // last good plan, as raw JSON — what Pin freezes
let pinnedSnapshot = null;
let lastPlanMs = null;
let firstPlan = true;

function spawnWorker() {
  worker?.terminate();
  worker = new Worker(new URL("./playground-worker.js", import.meta.url), { type: "module" });
  worker.onmessage = (e) => {
    const { seq: s, snapshot, planMs, error } = e.data;
    if (s < lastAppliedSeq) return;
    lastAppliedSeq = s;
    if (s === seq) setPlanning(false);
    if (error) {
      // A wasm panic poisons the instance; a fresh worker keeps later edits
      // planning instead of failing on a dead module.
      if (/unreachable|RuntimeError/i.test(error)) spawnWorker();
      showError(error);
      return;
    }
    clearError();
    lastPlanMs = planMs;
    currentSnapshot = snapshot;
    view.setData(snapshot, pinnedSnapshot, { keepView: !firstPlan });
    firstPlan = false;
  };
}

// Dense polygon gcode can take seconds to plan (clothoid fitting is the
// dominant cost); without a pending indicator a slow re-plan is
// indistinguishable from a dead page.
function setPlanning(on) {
  const el = document.getElementById("meta");
  el.classList.toggle("planning", on);
  if (on && !el.textContent) el.textContent = "planning…";
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
  const gcode = document.getElementById("gcode").value;
  saveState(gcode);
  seq += 1;
  setPlanning(true);
  worker.postMessage({ seq, gcode, config });
}

function schedulePlan() {
  clearTimeout(debounceTimer);
  debounceTimer = setTimeout(requestPlan, DEBOUNCE_MS);
}

// -- Pin / variant chrome ------------------------------------------------------
function syncControls() {
  const pinBtn = document.getElementById("pin");
  pinBtn.classList.toggle("pinned", pinnedSnapshot != null);
  pinBtn.textContent = pinnedSnapshot != null ? "Unpin" : "Pin baseline";

  const btn = document.getElementById("toggle-variant");
  const hasPin = view.hasBefore();
  btn.disabled = !hasPin;
  btn.classList.toggle("after", hasPin && view.variant === "after");
  btn.classList.toggle("before", hasPin && view.variant === "before");
  btn.textContent = !hasPin ? "Current" : view.variant === "before" ? "Pinned" : "Current";

  updateMeta();
}

function updateMeta() {
  if (!view.data) return;
  const planTime = lastPlanMs != null ? `  planned in ${lastPlanMs.toFixed(0)}ms` : "";
  document.getElementById("meta").textContent =
    `t=${view.data.traversal_time().toFixed(3)}s  ` +
    `[${view.segmentSummary()}]  ` +
    `${view.data.point_count()} pts${planTime}`;
}

function togglePin() {
  pinnedSnapshot = pinnedSnapshot == null ? currentSnapshot : null;
  view.setBaseline(pinnedSnapshot);
}

// -- Persistence ---------------------------------------------------------------
function saveState(gcode) {
  const config = {};
  for (const f of CONFIG_FIELDS) {
    config[f.id] = document.getElementById(`cfg-${f.id}`).value;
  }
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({ gcode, config }));
  } catch (e) { /* quota / private mode — persistence is best-effort */ }
}

function restoreState() {
  let state = null;
  try {
    state = JSON.parse(localStorage.getItem(STORAGE_KEY));
  } catch (e) { /* corrupted — start fresh */ }
  document.getElementById("gcode").value = state?.gcode || defaultGcode();
  for (const f of CONFIG_FIELDS) {
    const saved = state?.config?.[f.id];
    if (saved != null && saved !== "") {
      document.getElementById(`cfg-${f.id}`).value = saved;
    }
  }
}

// -- Init ------------------------------------------------------------------------
async function main() {
  await initWasm();

  view = new TrajectoryView();
  view.onChanged = syncControls;
  setupSplitter("motionPlayground.pathSplit");
  spawnWorker();
  restoreState();

  document.getElementById("gcode").addEventListener("input", schedulePlan);
  for (const f of CONFIG_FIELDS) {
    document.getElementById(`cfg-${f.id}`).addEventListener("input", schedulePlan);
  }

  document.getElementById("pin").addEventListener("click", togglePin);
  document.getElementById("toggle-variant").addEventListener("click", () => view.toggleVariant());
  document.getElementById("reset-zoom").addEventListener("click", () => view.resetZoom());
  document.getElementById("toggle-peaks").addEventListener("click", (e) => {
    e.target.classList.toggle("active", !view.showPeaks);
    view.setShowPeaks(!view.showPeaks);
  });

  document.addEventListener("keydown", (e) => {
    if (e.target.tagName === "TEXTAREA" || e.target.tagName === "INPUT") return;
    if (e.key === " " || e.key === "b" || e.key === "B") {
      e.preventDefault();
      view.toggleVariant();
    }
  });

  requestPlan();
}

main();
