"use strict";

const REFRESH_MS = 5000;
const MOONRAKER_KEY = "servoCalMoonrakerUrl";
const PALETTE = ["#4fb3ff", "#e05a4f", "#4caf50", "#d9a441", "#b388ff", "#4fd8c4"];
const RESONANCE_BAND_HZ = [20, 450];

const state = {
  runs: [],
  details: new Map(), // name -> {mtime_utc, has_results, manifest, results}
  overlaySelected: new Set(),
  formRun: null,
  psdStep: null,
  psdContext: null, // {names, plots} from the last overlay draw, reused by the step selector
  drive: {
    data: null, // last /api/drive_state response (params, motors, config_pins, age_s)
    fetchedAtMs: null, // Date.now() when data was fetched, for a client-ticking age display
    pending: {}, // param name -> raw number (all motors) or {motor: raw} (touched motors only)
    dirty: new Set(), // autofill-target param names the user has edited directly this session
    expanded: new Set(), // mixed param names toggled open to per-motor inputs
  },
  sentLog: [], // {time, label, lines, results} — every G-code batch sent this session
};

async function api(path, opts) {
  const resp = await fetch(path, opts);
  const text = await resp.text();
  if (!resp.ok) {
    throw new Error(`${path}: HTTP ${resp.status}: ${text}`);
  }
  return text.length ? JSON.parse(text) : null;
}

function detailIsFresh(name, run) {
  const cached = state.details.get(name);
  if (!cached) return false;
  return cached.mtime_utc === run.mtime_utc && cached.has_results === run.has_results;
}

async function ensureDetail(run) {
  if (detailIsFresh(run.name, run)) return;
  const manifest = await api(`/api/runs/${encodeURIComponent(run.name)}/manifest`);
  const results = run.has_results
    ? await api(`/api/runs/${encodeURIComponent(run.name)}/results`)
    : null;
  state.details.set(run.name, {
    mtime_utc: run.mtime_utc,
    has_results: run.has_results,
    manifest,
    results,
  });
}

function journalParams(manifest) {
  return (manifest && manifest.ambient && manifest.ambient.journal_params) || {};
}

function motorCount(journal) {
  return Object.keys(journal).length;
}

function ambientDiff(prevManifest, curManifest) {
  const prev = journalParams(prevManifest);
  const cur = journalParams(curManifest);
  const multiMotor = motorCount(prev) > 1 || motorCount(cur) > 1;
  const parts = [];
  const motors = new Set([...Object.keys(prev), ...Object.keys(cur)]);
  for (const motor of [...motors].sort()) {
    const prevAddrs = prev[motor] || {};
    const curAddrs = cur[motor] || {};
    const addrs = new Set([...Object.keys(prevAddrs), ...Object.keys(curAddrs)]);
    for (const addr of [...addrs].sort()) {
      const before = prevAddrs[addr];
      const after = curAddrs[addr];
      if (before === after) continue;
      const label = multiMotor ? `${motor}.${addr}` : addr;
      const beforeText = before === undefined ? "?" : before;
      const afterText = after === undefined ? "?" : after;
      parts.push(`${label}: ${beforeText}→${afterText}`);
    }
  }
  return parts.join(", ");
}

function stepHeadline(stepResult) {
  const drives = Object.values(stepResult.drives || {});
  const resonance = drives.some((d) => d.resonance && d.resonance.detected);
  let peakHz = 0;
  for (const d of drives) {
    if (d.resonance && d.resonance.detected && d.resonance.peak_hz > peakHz) {
      peakHz = d.resonance.peak_hz;
    }
  }
  let worstOvershoot = 0;
  let ferrPeak = 0;
  for (const d of drives) {
    for (const mv of d.metrics.moves || []) {
      worstOvershoot = Math.max(worstOvershoot, mv.overshoot);
      ferrPeak = Math.max(ferrPeak, mv.ferr_peak);
    }
  }
  return { name: stepResult.name, resonance, peakHz, worstOvershoot, ferrPeak, flags: stepResult.flags };
}

function stepCellHtml(stepResult, recommendedName) {
  const h = stepHeadline(stepResult);
  const isRec = h.name === recommendedName;
  const resBadge = h.resonance
    ? `<span class="badge resonance">res ${h.peakHz.toFixed(1)}Hz</span>`
    : "";
  const flagBadges = h.flags
    .filter((f) => f !== "resonance_detected")
    .map((f) => {
      const cls = f === "torque_saturated" ? "torque" : "truncated";
      return `<span class="badge ${cls}">${f}</span>`;
    })
    .join("");
  const nameHtml = isRec
    ? `<span class="badge step">${h.name}</span>`
    : `<span class="step-name">${h.name}</span>`;
  return (
    `<div>${nameHtml} ov <span class="num">${h.worstOvershoot.toFixed(0)}</span>` +
    ` ferr <span class="num">${h.ferrPeak.toFixed(0)}</span> ${resBadge}${flagBadges}</div>`
  );
}

function renderTable() {
  const tbody = document.getElementById("journal-body");
  tbody.innerHTML = "";
  state.runs.forEach((run, i) => {
    const detail = state.details.get(run.name);
    const manifest = detail && detail.manifest;
    const results = detail && detail.results;
    const prevManifest = i + 1 < state.runs.length ? (state.details.get(state.runs[i + 1].name) || {}).manifest : null;
    const diff = manifest ? ambientDiff(prevManifest, manifest) : "";

    const tr = document.createElement("tr");
    if (results && results.verdict.recommended_step) tr.classList.add("recommended");

    const checkTd = document.createElement("td");
    const check = document.createElement("input");
    check.type = "checkbox";
    check.checked = state.overlaySelected.has(run.name);
    check.disabled = !run.has_results;
    check.addEventListener("change", () => {
      if (check.checked) state.overlaySelected.add(run.name);
      else state.overlaySelected.delete(run.name);
    });
    checkTd.appendChild(check);
    tr.appendChild(checkTd);

    const timeTd = document.createElement("td");
    timeTd.textContent = run.mtime_utc;
    tr.appendChild(timeTd);

    const expTd = document.createElement("td");
    expTd.textContent = `${run.experiment}/${run.tag}${run.axis ? " " + run.axis : ""}`;
    tr.appendChild(expTd);

    const diffTd = document.createElement("td");
    diffTd.className = diff ? "diff" : "diff empty";
    diffTd.textContent = diff || "—";
    if (diff) diffTd.title = diff;
    tr.appendChild(diffTd);

    const stepsTd = document.createElement("td");
    stepsTd.className = "steps";
    if (results) {
      const rec = results.verdict.recommended_step;
      stepsTd.innerHTML = results.steps.map((s) => stepCellHtml(s, rec)).join("");
    } else if (run.verdict) {
      stepsTd.innerHTML = `<span class="note">has results, loading…</span>`;
    } else {
      stepsTd.innerHTML = `<span class="note">no results.json yet</span>`;
    }
    tr.appendChild(stepsTd);

    const verdictTd = document.createElement("td");
    const v = run.verdict;
    verdictTd.textContent = v ? (v.recommended_step ? `→ ${v.recommended_step}: ${v.reason}` : `none: ${v.reason}`) : "—";
    tr.appendChild(verdictTd);

    const actionTd = document.createElement("td");
    const prefillBtn = document.createElement("button");
    prefillBtn.textContent = "load → form";
    prefillBtn.disabled = !manifest;
    prefillBtn.addEventListener("click", () => loadRerunForm(run.name));
    actionTd.appendChild(prefillBtn);
    if (!run.has_results) {
      const analyzeBtn = document.createElement("button");
      analyzeBtn.textContent = "analyze";
      analyzeBtn.addEventListener("click", () => triggerAnalyze(run.name));
      actionTd.appendChild(analyzeBtn);
    }
    tr.appendChild(actionTd);

    tbody.appendChild(tr);
  });
  document.getElementById("overlay-count").textContent = String(state.overlaySelected.size);
}

async function triggerAnalyze(name) {
  await api(`/api/runs/${encodeURIComponent(name)}/analyze`, { method: "POST" });
  await refresh();
}

async function refresh() {
  const runs = await api("/api/runs");
  state.runs = runs;
  await Promise.all(runs.map((r) => ensureDetail(r).catch((e) => console.error(e))));
  renderTable();
}

// --- overlay drill-down ---------------------------------------------------

function pickSeries(step) {
  if (step.combined) {
    return { y: step.combined.on_ferr_mm, label: "on-axis ferr (mm)" };
  }
  const firstDrive = Object.values(step.drives)[0];
  return { y: firstDrive ? firstDrive.ferr_counts : [], label: "ferr (counts)" };
}

function drawChart(canvas, traces, yLabel) {
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
  const pad = { l: 46, r: 8, t: 8, b: 22 };
  let tMin = Infinity, tMax = -Infinity, yMin = Infinity, yMax = -Infinity;
  for (const tr of traces) {
    for (let i = 0; i < tr.t.length; i++) {
      tMin = Math.min(tMin, tr.t[i]);
      tMax = Math.max(tMax, tr.t[i]);
      yMin = Math.min(yMin, tr.y[i]);
      yMax = Math.max(yMax, tr.y[i]);
    }
  }
  if (!isFinite(tMin) || !isFinite(yMin)) return;
  if (yMin === yMax) { yMin -= 1; yMax += 1; }
  const x = (t) => pad.l + ((t - tMin) / (tMax - tMin || 1)) * (w - pad.l - pad.r);
  const y = (v) => h - pad.b - ((v - yMin) / (yMax - yMin || 1)) * (h - pad.t - pad.b);

  ctx.strokeStyle = "#29313a";
  ctx.fillStyle = "#8a97a3";
  ctx.font = "10px monospace";
  ctx.beginPath();
  for (let i = 0; i <= 4; i++) {
    const v = yMin + ((yMax - yMin) * i) / 4;
    const py = y(v);
    ctx.moveTo(pad.l, py);
    ctx.lineTo(w - pad.r, py);
    ctx.fillText(v.toFixed(2), 2, py + 3);
  }
  for (let i = 0; i <= 4; i++) {
    const t = tMin + ((tMax - tMin) * i) / 4;
    const px = x(t);
    ctx.fillText(t.toFixed(2) + "s", px, h - 6);
  }
  ctx.stroke();

  for (const tr of traces) {
    ctx.strokeStyle = tr.color;
    ctx.lineWidth = 1.25;
    ctx.beginPath();
    for (let i = 0; i < tr.t.length; i++) {
      const px = x(tr.t[i]);
      const py = y(tr.y[i]);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();
  }
  ctx.fillStyle = "#8a97a3";
  ctx.fillText(yLabel, pad.l, 10);
}

async function drawOverlay() {
  const names = [...state.overlaySelected];
  const container = document.getElementById("charts");
  container.innerHTML = "";
  if (names.length === 0) {
    container.innerHTML = '<p class="note">tick runs in the journal, then Overlay</p>';
    state.psdContext = null;
    drawPsdSection([], [], []);
    return;
  }
  const plots = await Promise.all(
    names.map((n) => api(`/api/runs/${encodeURIComponent(n)}/plot_series`))
  );
  const stepNames = [...new Set(plots.flatMap((p) => p.steps.map((s) => s.name)))];

  for (const stepName of stepNames) {
    const box = document.createElement("div");
    box.className = "chart-box";
    const title = document.createElement("h3");
    title.textContent = stepName;
    box.appendChild(title);
    const canvas = document.createElement("canvas");
    canvas.width = 640;
    canvas.height = 220;
    box.appendChild(canvas);
    const legend = document.createElement("div");
    legend.className = "legend";

    const traces = [];
    let yLabel = "";
    plots.forEach((p, i) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step) return;
      const { y: series, label } = pickSeries(step);
      yLabel = label;
      const color = PALETTE[i % PALETTE.length];
      traces.push({ t: step.t_s, y: series, color });
      const item = document.createElement("span");
      item.innerHTML = `<span class="swatch" style="background:${color}"></span>${names[i]}`;
      legend.appendChild(item);
    });
    drawChart(canvas, traces, yLabel);
    box.appendChild(legend);
    container.appendChild(box);
  }

  state.psdContext = { names, plots };
  drawPsdSection(names, plots, stepNames);
}

// --- following-error PSD overlay ------------------------------------------

function newestSelectedRunName(names) {
  const selected = new Set(names);
  const found = state.runs.find((r) => selected.has(r.name));
  return found ? found.name : names[0];
}

function defaultPsdStep(names, plots, stepNames) {
  const plotByName = new Map(names.map((n, i) => [n, plots[i]]));
  const newest = newestSelectedRunName(names);
  const newestPlot = plotByName.get(newest);
  const newestSteps = newestPlot ? newestPlot.steps.map((s) => s.name) : stepNames;
  const detail = state.details.get(newest);
  const recommended = detail && detail.results && detail.results.verdict.recommended_step;
  if (recommended && newestSteps.includes(recommended)) return recommended;
  if (newestSteps.length) return newestSteps[newestSteps.length - 1];
  return stepNames[stepNames.length - 1];
}

function psdTraces(names, plots, stepName) {
  const traces = [];
  plots.forEach((p, i) => {
    const step = p.steps.find((s) => s.name === stepName);
    if (!step || !step.psd) return;
    const color = PALETTE[i % PALETTE.length];
    const driveNames = Object.keys(step.psd.per_drive);
    if (driveNames.length) {
      traces.push({
        freq: step.psd.freq_hz,
        y: step.psd.per_drive[driveNames[0]],
        color,
        dashed: false,
        label: `${names[i]} (${driveNames[0]})`,
      });
    }
    if (step.psd.accel) {
      traces.push({
        freq: step.psd.accel.freq_hz,
        y: step.psd.accel.psd,
        color,
        dashed: true,
        label: `${names[i]} (accel)`,
      });
    }
  });
  return traces;
}

function drawPsdChart(canvas, traces, band) {
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
  const pad = { l: 46, r: 8, t: 8, b: 22 };
  const EPS = 1e-6;
  let fMin = Infinity, fMax = -Infinity, logMin = Infinity, logMax = -Infinity;
  for (const tr of traces) {
    for (let i = 0; i < tr.freq.length; i++) {
      const f = tr.freq[i];
      const lv = Math.log10(Math.max(tr.y[i], EPS));
      fMin = Math.min(fMin, f);
      fMax = Math.max(fMax, f);
      logMin = Math.min(logMin, lv);
      logMax = Math.max(logMax, lv);
    }
  }
  if (!isFinite(fMin) || !isFinite(logMin)) return;
  if (logMin === logMax) { logMin -= 1; logMax += 1; }
  const x = (f) => pad.l + ((f - fMin) / (fMax - fMin || 1)) * (w - pad.l - pad.r);
  const yOfLog = (lv) => h - pad.b - ((lv - logMin) / (logMax - logMin || 1)) * (h - pad.t - pad.b);
  const y = (v) => yOfLog(Math.log10(Math.max(v, EPS)));

  if (band) {
    const [blo, bhi] = band;
    const bandLo = Math.max(blo, fMin);
    const bandHi = Math.min(bhi, fMax);
    if (bandHi > bandLo) {
      ctx.fillStyle = "rgba(217, 164, 65, 0.10)";
      ctx.fillRect(x(bandLo), pad.t, x(bandHi) - x(bandLo), h - pad.t - pad.b);
    }
  }

  ctx.strokeStyle = "#29313a";
  ctx.fillStyle = "#8a97a3";
  ctx.font = "10px monospace";
  ctx.beginPath();
  for (let i = 0; i <= 4; i++) {
    const lv = logMin + ((logMax - logMin) * i) / 4;
    const py = yOfLog(lv);
    ctx.moveTo(pad.l, py);
    ctx.lineTo(w - pad.r, py);
    ctx.fillText(`1e${lv.toFixed(1)}`, 2, py + 3);
  }
  for (let i = 0; i <= 4; i++) {
    const f = fMin + ((fMax - fMin) * i) / 4;
    const px = x(f);
    ctx.fillText(f.toFixed(0) + "Hz", px, h - 6);
  }
  ctx.stroke();

  for (const tr of traces) {
    ctx.strokeStyle = tr.color;
    ctx.lineWidth = 1.25;
    ctx.setLineDash(tr.dashed ? [4, 3] : []);
    ctx.beginPath();
    for (let i = 0; i < tr.freq.length; i++) {
      const px = x(tr.freq[i]);
      const py = y(tr.y[i]);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();
  }
  ctx.setLineDash([]);

  if (band) {
    const [blo, bhi] = band;
    traces.forEach((tr, idx) => {
      let bestI = -1, bestV = -Infinity;
      for (let i = 0; i < tr.freq.length; i++) {
        if (tr.freq[i] >= blo && tr.freq[i] < bhi && tr.y[i] > bestV) {
          bestV = tr.y[i];
          bestI = i;
        }
      }
      if (bestI < 0) return;
      const px = x(tr.freq[bestI]);
      const py = y(tr.y[bestI]);
      ctx.fillStyle = tr.color;
      ctx.beginPath();
      ctx.arc(px, py, 2.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillText(`${tr.freq[bestI].toFixed(0)}Hz`, px + 4, py - 4 - (idx % 3) * 10);
    });
  }

  ctx.fillStyle = "#8a97a3";
  ctx.fillText("log10 psd (counts²/Hz)", pad.l, 10);
}

function renderPsdChart(names, plots, stepName) {
  const container = document.getElementById("psd-charts");
  container.innerHTML = "";
  if (!stepName) {
    container.innerHTML = '<p class="note">tick runs in the journal, then Overlay</p>';
    return;
  }
  const box = document.createElement("div");
  box.className = "chart-box";
  const title = document.createElement("h3");
  title.textContent = `following-error PSD — ${stepName}`;
  box.appendChild(title);
  const canvas = document.createElement("canvas");
  canvas.width = 640;
  canvas.height = 220;
  box.appendChild(canvas);
  const legend = document.createElement("div");
  legend.className = "legend";
  names.forEach((n, i) => {
    const item = document.createElement("span");
    item.innerHTML = `<span class="swatch" style="background:${PALETTE[i % PALETTE.length]}"></span>${n}`;
    legend.appendChild(item);
  });
  drawPsdChart(canvas, psdTraces(names, plots, stepName), RESONANCE_BAND_HZ);
  box.appendChild(legend);
  container.appendChild(box);
}

function drawPsdSection(names, plots, stepNames) {
  const select = document.getElementById("psd-step-select");
  if (names.length === 0 || stepNames.length === 0) {
    select.innerHTML = "";
    renderPsdChart(names, plots, null);
    return;
  }
  select.innerHTML = stepNames.map((s) => `<option value="${s}">${s}</option>`).join("");
  const wanted = state.psdStep && stepNames.includes(state.psdStep)
    ? state.psdStep
    : defaultPsdStep(names, plots, stepNames);
  select.value = wanted;
  state.psdStep = wanted;
  renderPsdChart(names, plots, wanted);
}

// --- drive tuning panel -----------------------------------------------------
//
// Renders purely from GET /api/drive_state (servo_tuning.PANEL_PARAMS shape,
// docs/rewrite/servo-tuning-profiles.md). Pure helpers first — display/raw
// unit conversion, autofill derivation, changed-param diffing — the logic a
// Rust test asserts is present and exercisable without a browser; DOM
// rendering and event wiring follow.

const GROUP_ORDER = ["gains", "filters", "notch", "load", "experimental"];
const OTHER_GROUP = "other";
const AUTOFILL_SOURCE_PARAM = "speed_gain";
const DRIVE_REFRESH_POLL_MS = 1000;
const DRIVE_REFRESH_TIMEOUT_MS = 15000;

function rawToDisplay(raw, scale) {
  return raw / scale;
}

function displayToRaw(display, scale) {
  return Math.round(display * scale);
}

function deriveGainPositionFromSpeed(speedGainRaw) {
  return Math.round(speedGainRaw * 1.6);
}

function deriveGainIntegralFromSpeed(speedGainRaw) {
  return Math.round(1250000 / speedGainRaw);
}

const AUTOFILL_FORMULAS = {
  gain_position_from_speed: deriveGainPositionFromSpeed,
  gain_integral_from_speed: deriveGainIntegralFromSpeed,
};

function paramGroupSection(param) {
  return GROUP_ORDER.includes(param.group) ? param.group : OTHER_GROUP;
}

function groupParams(params) {
  const sections = new Map([...GROUP_ORDER, OTHER_GROUP].map((g) => [g, []]));
  for (const p of params) sections.get(paramGroupSection(p)).push(p);
  return sections;
}

function motorRawValues(motors, cCode) {
  return Object.keys(motors)
    .sort()
    .map((m) => motors[m][cCode]);
}

function valuesAgree(values) {
  return values.length > 0 && values.every((v) => v === values[0]);
}

function pinnedEntries(configPins, cCode) {
  const out = {};
  for (const motor of Object.keys(configPins || {}).sort()) {
    const pins = configPins[motor] || {};
    if (Object.prototype.hasOwnProperty.call(pins, cCode)) out[motor] = pins[cCode];
  }
  return out;
}

/// Which mapped params differ from the drive_state's original per-motor
/// readings, given this session's pending edits. `pending[name]` is either a
/// raw number (the row applies the same value to every motor) or a
/// `{motor: raw}` map (the row was expanded and only some motors were
/// touched) — the shape `buildServoTuneCommands` expands into gcode.
function diffChangedParams(params, motors, pending) {
  const changed = [];
  for (const p of params) {
    const pend = pending[p.name];
    if (pend === undefined) continue;
    const cCode = p.c_code;
    if (typeof pend === "object") {
      const perMotor = [];
      for (const motor of Object.keys(pend).sort()) {
        const orig = motors[motor] ? motors[motor][cCode] : undefined;
        if (orig !== pend[motor]) perMotor.push({ motor, value: pend[motor] });
      }
      if (perMotor.length) changed.push({ name: p.name, perMotor });
    } else {
      const values = motorRawValues(motors, cCode);
      const orig = valuesAgree(values) ? values[0] : undefined;
      if (orig !== pend) changed.push({ name: p.name, value: pend });
    }
  }
  return changed;
}

function buildServoTuneCommands(changed) {
  const lines = [];
  for (const c of changed) {
    if (c.perMotor) {
      for (const { motor, value } of c.perMotor) {
        lines.push(`SERVO_TUNE PARAM=${c.name} VALUE=${value} MOTORS=${motor}`);
      }
    } else {
      lines.push(`SERVO_TUNE PARAM=${c.name} VALUE=${c.value}`);
    }
  }
  return lines;
}

function paramByName(name) {
  return state.drive.data.params.find((p) => p.name === name);
}

function currentSpeedGainRaw() {
  const pending = state.drive.pending[AUTOFILL_SOURCE_PARAM];
  if (pending !== undefined) return pending;
  const speedParam = paramByName(AUTOFILL_SOURCE_PARAM);
  const values = motorRawValues(state.drive.data.motors, speedParam.c_code);
  if (valuesAgree(values)) return values[0];
  const motorNames = Object.keys(state.drive.data.motors).sort();
  return Object.fromEntries(motorNames.map((m, i) => [m, values[i]]));
}

function applyFormula(formula, speedRawOrMap) {
  return typeof speedRawOrMap === "object"
    ? Object.fromEntries(Object.entries(speedRawOrMap).map(([m, v]) => [m, formula(v)]))
    : formula(speedRawOrMap);
}

/// speed_gain changed: push a derived value into every autofill target that
/// the user hasn't dirtied (edited directly) this session.
function propagateAutofill(speedRawOrMap) {
  for (const param of state.drive.data.params) {
    const formula = AUTOFILL_FORMULAS[param.autofill];
    if (!formula || state.drive.dirty.has(param.name)) continue;
    state.drive.pending[param.name] = applyFormula(formula, speedRawOrMap);
  }
}

function rederiveAutofillTarget(name) {
  const formula = AUTOFILL_FORMULAS[paramByName(name).autofill];
  if (!formula) return;
  state.drive.pending[name] = applyFormula(formula, currentSpeedGainRaw());
}

function formatAge(ageS) {
  if (ageS < 60) return `${ageS.toFixed(0)}s`;
  const m = Math.floor(ageS / 60);
  const s = Math.round(ageS % 60);
  return `${m}m${s}s`;
}

function currentDriveAgeS() {
  if (!state.drive.data) return null;
  return state.drive.data.age_s + (Date.now() - state.drive.fetchedAtMs) / 1000;
}

function renderDriveBanner() {
  const el = document.getElementById("drive-state-banner");
  const data = state.drive.data;
  if (!data) {
    el.innerHTML = '<span class="note">loading drive state…</span>';
    return;
  }
  el.innerHTML =
    `<span class="note">drive state ${formatAge(currentDriveAgeS())} old ` +
    `(dumped ${data.created_utc})</span> ` +
    `<button id="drive-refresh-btn">refresh</button>` +
    `<span id="drive-refresh-status" class="note"></span>`;
  document.getElementById("drive-refresh-btn").addEventListener("click", refreshDriveState);
}

function renderDriveRow(param, section, motors, configPins) {
  const cCode = param.c_code;
  const motorNames = Object.keys(motors).sort();
  const rawValues = motorRawValues(motors, cCode);
  const agree = valuesAgree(rawValues);
  const pins = pinnedEntries(configPins, cCode);
  const pinnedNames = Object.keys(pins);
  const pinBadge = pinnedNames.length
    ? `<span class="badge pin" title="restart re-applies this">pin ${[...new Set(pinnedNames.map((m) => pins[m]))].join("/")}</span>`
    : "";
  const groupHint = section === OTHER_GROUP ? `<span class="hint">(${param.group})</span>` : "";
  const rederiveLink = state.drive.dirty.has(param.name)
    ? ` <a href="#" class="rederive" data-param="${param.name}" title="restore the autofill link">re-derive</a>`
    : "";

  let body;
  if (agree) {
    const pending = state.drive.pending[param.name];
    const raw = typeof pending === "number" ? pending : rawValues[0];
    body =
      `<input type="number" step="any" class="drive-input" data-param="${param.name}" data-mode="all" value="${rawToDisplay(raw, param.scale)}">` +
      `<span class="hint">raw ${raw}</span>`;
  } else if (!state.drive.expanded.has(param.name)) {
    body = `<span class="badge mixed" data-param="${param.name}" data-role="expand">mixed — click to edit per motor</span>`;
  } else {
    const pendObj = typeof state.drive.pending[param.name] === "object" ? state.drive.pending[param.name] : {};
    body =
      motorNames
        .map((m) => {
          const raw = pendObj[m] !== undefined ? pendObj[m] : motors[m][cCode];
          return (
            `<div class="per-motor"><label>${m}</label>` +
            `<input type="number" step="any" class="drive-input" data-param="${param.name}" data-mode="motor" data-motor="${m}" value="${rawToDisplay(raw, param.scale)}">` +
            `<span class="hint">raw ${raw}</span></div>`
          );
        })
        .join("") + `<a href="#" class="collapse-mixed" data-param="${param.name}">collapse</a>`;
  }

  return (
    `<div class="param-row" data-param="${param.name}">` +
    `<div class="param-label">${param.name}${param.unit ? ` <span class="unit">${param.unit}</span>` : ""}${pinBadge}${groupHint}${rederiveLink}</div>` +
    `<div class="param-body">${body}</div>` +
    `</div>`
  );
}

function updateApplyState() {
  const btn = document.getElementById("drive-apply-btn");
  const label = document.getElementById("drive-changed-count");
  const changed = diffChangedParams(state.drive.data.params, state.drive.data.motors, state.drive.pending);
  btn.disabled = changed.length === 0;
  label.textContent = changed.length ? `${changed.length} param(s) changed` : "no changes pending";
}

function bindDriveRowEvents() {
  const container = document.getElementById("drive-groups");
  container.querySelectorAll("input.drive-input").forEach((input) => {
    input.addEventListener("change", onDriveInputChange);
  });
  container.querySelectorAll('.badge.mixed[data-role="expand"]').forEach((el) => {
    el.addEventListener("click", () => {
      state.drive.expanded.add(el.dataset.param);
      renderDriveGroups();
    });
  });
  container.querySelectorAll("a.collapse-mixed").forEach((el) => {
    el.addEventListener("click", (e) => {
      e.preventDefault();
      state.drive.expanded.delete(el.dataset.param);
      renderDriveGroups();
    });
  });
  container.querySelectorAll("a.rederive").forEach((el) => {
    el.addEventListener("click", (e) => {
      e.preventDefault();
      state.drive.dirty.delete(el.dataset.param);
      rederiveAutofillTarget(el.dataset.param);
      renderDriveGroups();
    });
  });
}

function onDriveInputChange(e) {
  const input = e.target;
  const name = input.dataset.param;
  const param = paramByName(name);
  const display = parseFloat(input.value);
  if (Number.isNaN(display)) return;
  const raw = displayToRaw(display, param.scale);
  if (input.dataset.mode === "motor") {
    const existing = typeof state.drive.pending[name] === "object" ? { ...state.drive.pending[name] } : {};
    existing[input.dataset.motor] = raw;
    state.drive.pending[name] = existing;
  } else {
    state.drive.pending[name] = raw;
  }
  if (name === AUTOFILL_SOURCE_PARAM) {
    propagateAutofill(state.drive.pending[name]);
  } else {
    state.drive.dirty.add(name);
  }
  renderDriveGroups();
}

function renderDriveGroups() {
  const container = document.getElementById("drive-groups");
  const data = state.drive.data;
  if (!data) {
    container.innerHTML = '<p class="note">loading drive state…</p>';
    document.getElementById("drive-apply-btn").disabled = true;
    document.getElementById("drive-changed-count").textContent = "";
    return;
  }
  const sections = groupParams(data.params);
  const parts = [];
  for (const [group, params] of sections) {
    if (!params.length) continue;
    const rows = params.map((p) => renderDriveRow(p, group, data.motors, data.config_pins)).join("");
    parts.push(`<div class="param-group"><h3>${group}</h3>${rows}</div>`);
  }
  container.innerHTML = parts.join("");
  bindDriveRowEvents();
  updateApplyState();
}

async function loadDriveState() {
  try {
    const data = await api("/api/drive_state");
    state.drive.data = data;
    state.drive.fetchedAtMs = Date.now();
  } catch (e) {
    state.drive.data = null;
    console.error(e);
  }
  state.drive.pending = {};
  state.drive.dirty = new Set();
  state.drive.expanded = new Set();
  renderDriveBanner();
  renderDriveGroups();
}

async function refreshDriveState() {
  const statusEl = document.getElementById("drive-refresh-status");
  const priorAge = state.drive.data ? currentDriveAgeS() : Infinity;
  statusEl.textContent = " dumping…";
  await runGcode(["SERVO_DUMP_TUNING"], "refresh");
  const deadline = Date.now() + DRIVE_REFRESH_TIMEOUT_MS;
  while (Date.now() < deadline) {
    await new Promise((resolve) => setTimeout(resolve, DRIVE_REFRESH_POLL_MS));
    let data;
    try {
      data = await api("/api/drive_state");
    } catch (e) {
      continue;
    }
    if (data.age_s < priorAge) {
      state.drive.data = data;
      state.drive.fetchedAtMs = Date.now();
      state.drive.pending = {};
      state.drive.dirty = new Set();
      state.drive.expanded = new Set();
      renderDriveBanner();
      renderDriveGroups();
      return;
    }
  }
  statusEl.textContent = " refresh timed out — drive_state.json never got newer";
}

async function applyDriveChanges() {
  const changed = diffChangedParams(state.drive.data.params, state.drive.data.motors, state.drive.pending);
  if (!changed.length) return;
  await runGcode(buildServoTuneCommands(changed), "apply");
  await loadDriveState();
}

// --- re-run form -----------------------------------------------------------

function reconstructCommand(manifest) {
  const tag = manifest.tag || "cal";
  const axis = manifest.axis || "X";
  const iterations = (manifest.stroke_plan && manifest.stroke_plan.iterations) || 1;
  const sweptKeys = (manifest.steps || []).map((s) => Object.keys(s.swept || {}));
  const commonKeys = sweptKeys.reduce((a, b) => a.filter((k) => b.includes(k)), sweptKeys[0] || []);

  switch (manifest.experiment) {
    case "gain_sweep": {
      const values = manifest.steps.map((s) => s.swept.speed).join(",");
      return `SERVO_CALIBRATE_GAINS SPEED_GAINS=${values} AXIS=${axis} ITERATIONS=${iterations} TAG=${tag}`;
    }
    case "refine_sweep": {
      const param = commonKeys.length === 1 ? commonKeys[0] : "speed";
      const values = manifest.steps.map((s) => s.swept[param]).join(",");
      return `SERVO_REFINE_GAIN PARAM=${param} VALUES=${values} AXIS=${axis} ITERATIONS=${iterations} TAG=${tag}`;
    }
    case "inertia_sweep": {
      const values = manifest.steps.map((s) => s.swept.ratio ?? Object.values(s.swept)[0]).join(",");
      return `SERVO_SWEEP_INERTIA RATIOS=${values} AXIS=${axis} ITERATIONS=${iterations} TAG=${tag}`;
    }
    case "accel_sweep": {
      const values = manifest.steps.map((s) => s.swept.accel ?? Object.values(s.swept)[0]).join(",");
      return `SERVO_SWEEP_ACCEL ACCELS=${values} AXIS=${axis} ITERATIONS=${iterations} TAG=${tag}`;
    }
    default:
      return `; ${manifest.experiment} has no known reconstruction — edit by hand`;
  }
}

function loadRerunForm(name) {
  const detail = state.details.get(name);
  if (!detail || !detail.manifest) return;
  state.formRun = name;
  document.getElementById("form-run-name").textContent = name;
  document.getElementById("sweep-command").value = reconstructCommand(detail.manifest);
}

function moonrakerUrl() {
  const input = document.getElementById("moonraker-url");
  return input.value.replace(/\/+$/, "");
}

function appendSentLog(entry) {
  state.sentLog.push(entry);
  const container = document.getElementById("sent-log");
  const ok = entry.results.length > 0 && entry.results.every((r) => r.ok);
  const div = document.createElement("div");
  div.className = "sent-entry";
  div.innerHTML =
    `<div class="sent-head">${entry.time} — ${entry.label} — ${entry.lines.length} line(s) — ` +
    `<span class="${ok ? "status-ok" : "status-err"}">${ok ? "ok" : "error"}</span></div>` +
    entry.lines
      .map((l, i) => {
        const r = entry.results[i];
        const suffix = r ? ` <span class="hint">HTTP ${r.status}</span>` : "";
        return `<div class="sent-line">${l}${suffix}</div>`;
      })
      .join("");
  container.appendChild(div);
  container.scrollTop = container.scrollHeight;
}

/// Sends `lines` (already-built gcode) through the existing Moonraker
/// plumbing — used by the drive panel's Apply, the sweep row's Run, and the
/// manual textarea's Run alike, so every batch lands in the same session
/// log regardless of where it came from. Also mirrors `lines` into the
/// textarea first: it stays the single place to see (and, for the manual
/// path, edit) exactly what is about to be sent.
async function runGcode(lines, label) {
  document.getElementById("gcode-textarea").value = lines.join("\n");
  const base = moonrakerUrl();
  const statusEl = document.getElementById("run-status");
  statusEl.textContent = "";
  const entry = { time: new Date().toISOString(), label, lines: [], results: [] };
  for (const line of lines) {
    const url = `${base}/printer/gcode/script?script=${encodeURIComponent(line)}`;
    entry.lines.push(line);
    try {
      const resp = await fetch(url, { method: "POST" });
      const text = await resp.text();
      const cls = resp.ok ? "status-ok" : "status-err";
      statusEl.innerHTML += `<div class="${cls}">${line} -> HTTP ${resp.status} ${text.slice(0, 200)}</div>`;
      entry.results.push({ ok: resp.ok, status: resp.status });
      if (!resp.ok) break;
    } catch (e) {
      statusEl.innerHTML += `<div class="status-err">${line} -> ${e}</div>`;
      entry.results.push({ ok: false, status: 0 });
      break;
    }
  }
  appendSentLog(entry);
}

function manualGcodeLines() {
  return document
    .getElementById("gcode-textarea")
    .value.split("\n")
    .map((l) => l.trim())
    .filter((l) => l.length && !l.startsWith(";"));
}

async function runSweep() {
  const line = document.getElementById("sweep-command").value.trim();
  if (!line || line.startsWith(";")) return;
  await runGcode([line], "sweep");
}

function initForm() {
  const input = document.getElementById("moonraker-url");
  input.value = localStorage.getItem(MOONRAKER_KEY) || `http://${location.hostname}:7125`;
  input.addEventListener("change", () => localStorage.setItem(MOONRAKER_KEY, input.value));
  document.getElementById("run-gcode").addEventListener("click", () => runGcode(manualGcodeLines(), "manual"));
  document.getElementById("drive-apply-btn").addEventListener("click", applyDriveChanges);
  document.getElementById("run-sweep-btn").addEventListener("click", runSweep);
  document.getElementById("overlay-btn").addEventListener("click", drawOverlay);
  document.getElementById("psd-step-select").addEventListener("change", (e) => {
    state.psdStep = e.target.value;
    if (state.psdContext) renderPsdChart(state.psdContext.names, state.psdContext.plots, state.psdStep);
  });
}

async function tick() {
  try {
    await refresh();
  } catch (e) {
    console.error(e);
  }
  renderDriveBanner();
}

initForm();
tick();
loadDriveState();
setInterval(tick, REFRESH_MS);
setInterval(renderDriveBanner, 1000);
