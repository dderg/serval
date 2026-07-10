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
    tr.appendChild(diffTd);

    const stepsTd = document.createElement("td");
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

function paramLines(manifest) {
  const journal = journalParams(manifest);
  const lines = [];
  for (const motor of Object.keys(journal).sort()) {
    for (const addr of Object.keys(journal[motor]).sort()) {
      lines.push({ motor, addr, value: journal[motor][addr] });
    }
  }
  return lines;
}

function renderParamRows(rows) {
  const container = document.getElementById("param-rows");
  container.innerHTML = "";
  rows.forEach((r, i) => {
    const row = document.createElement("div");
    row.className = "row";
    row.innerHTML =
      `<label>${r.motor}</label>` +
      `<input type="text" data-role="addr" data-i="${i}" value="${r.addr}">` +
      `<input type="number" data-role="value" data-i="${i}" value="${r.value}">`;
    container.appendChild(row);
  });
}

function currentParamRows(manifest) {
  const rows = paramLines(manifest);
  return rows.map((r, i) => {
    const addrInput = document.querySelector(`#param-rows input[data-role="addr"][data-i="${i}"]`);
    const valueInput = document.querySelector(`#param-rows input[data-role="value"][data-i="${i}"]`);
    return {
      motor: r.motor,
      addr: addrInput ? addrInput.value : r.addr,
      value: valueInput ? valueInput.value : r.value,
    };
  });
}

function rebuildTextarea(manifest) {
  const rows = currentParamRows(manifest);
  const paramCmds = rows.map(
    (r) => `SERVO_PARAM SERVO=${r.motor} SET=${r.addr} VALUE=${r.value} TYPE=u16`
  );
  const sweep = reconstructCommand(manifest);
  document.getElementById("gcode-textarea").value = [...paramCmds, "", sweep].join("\n");
}

function loadRerunForm(name) {
  const detail = state.details.get(name);
  if (!detail || !detail.manifest) return;
  state.formRun = name;
  document.getElementById("form-run-name").textContent = name;
  renderParamRows(paramLines(detail.manifest));
  rebuildTextarea(detail.manifest);
  document.getElementById("param-rows").addEventListener(
    "input",
    () => rebuildTextarea(detail.manifest),
    { once: false }
  );
}

function moonrakerUrl() {
  const input = document.getElementById("moonraker-url");
  return input.value.replace(/\/+$/, "");
}

async function runGcode() {
  const lines = document
    .getElementById("gcode-textarea")
    .value.split("\n")
    .map((l) => l.trim())
    .filter((l) => l.length && !l.startsWith(";"));
  const base = moonrakerUrl();
  const statusEl = document.getElementById("run-status");
  statusEl.textContent = "";
  for (const line of lines) {
    const url = `${base}/printer/gcode/script?script=${encodeURIComponent(line)}`;
    try {
      const resp = await fetch(url, { method: "POST" });
      const text = await resp.text();
      const cls = resp.ok ? "status-ok" : "status-err";
      statusEl.innerHTML += `<div class="${cls}">${line} -> HTTP ${resp.status} ${text.slice(0, 200)}</div>`;
      if (!resp.ok) break;
    } catch (e) {
      statusEl.innerHTML += `<div class="status-err">${line} -> ${e}</div>`;
      break;
    }
  }
}

function initForm() {
  const input = document.getElementById("moonraker-url");
  input.value = localStorage.getItem(MOONRAKER_KEY) || `http://${location.hostname}:7125`;
  input.addEventListener("change", () => localStorage.setItem(MOONRAKER_KEY, input.value));
  document.getElementById("run-gcode").addEventListener("click", runGcode);
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
}

initForm();
tick();
setInterval(tick, REFRESH_MS);
