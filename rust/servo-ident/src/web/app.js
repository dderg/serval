"use strict";

const REFRESH_MS = 5000;
const MOONRAKER_KEY = "servoCalMoonrakerUrl";
const PALETTE = ["#4fb3ff", "#e05a4f", "#4caf50", "#d9a441", "#b388ff", "#4fd8c4"];
const RESONANCE_BAND_HZ = [20, 450];
const INITIAL_SELECTED_RUNS = 3;
const PEAK_MIN_SEPARATION_HZ = 15;
const PEAK_LIST_SIZE = 3;

// Each page serves one calibration activity with only the tools that
// activity needs (docs/plans/servo-calibration-automation.md, second demo
// review): the interleaved tuning loop is navigation between pages, not
// scrolling within one.
const PAGE_DEFS = {
  gains: {
    label: "gains",
    groups: ["gains"],
    experiments: ["gain_sweep", "refine_sweep", "gain_ladder"],
    charts: ["psd"],
    intro: "find the highest speed gain without resonance or torque rail",
    templates: [
      {
        label: "ladder…",
        command: "SERVO_GAIN_LADDER SAFE=550 START=700 STEP=50 MAX=900 AXIS=X ITERATIONS=1",
        title: "climb from START by STEP until a rung flags, then revert to SAFE",
      },
    ],
  },
  notches: {
    label: "notches",
    groups: ["notch"],
    experiments: ["gain_sweep", "refine_sweep", "gain_ladder"],
    charts: ["psd"],
    peaks: true,
    intro: "kill the resonances the PSD shows so gains can go higher",
    templates: [
      {
        label: "harvest…",
        command: "SERVO_HARVEST_NOTCHES AXIS=X MODE=2",
        title:
          "hand notches 1-2 to the drive's adaptive tuning, stroke, read back what it chose, lock",
      },
    ],
  },
  observers: {
    label: "observers",
    groups: ["filters", "speed_observer", "disturbance_observer"],
    experiments: null,
    charts: ["time"],
    intro: "disturbance rejection and filtering — judge in the time domain",
  },
  dynamics: {
    label: "dynamics",
    groups: ["load"],
    experiments: ["tracking", "inertia_grid"],
    charts: [],
    fitRunner: true,
    intro: "identify the load, then let feedforward carry it",
  },
  live: {
    label: "live",
    live: true,
    intro: "following error streamed off the growing capture file",
  },
  journal: {
    label: "journal",
    journal: true,
  },
};
const DEFAULT_PAGE = "gains";
const LIVE_STATUS_POLL_MS = 1000;
const LIVE_TAIL_POLL_MS = 400;
const LIVE_WINDOW_S = 10;
const MOONRAKER_HEALTH_POLL_MS = 5000;

const state = {
  page: DEFAULT_PAGE,
  runs: [],
  details: new Map(), // name -> {mtime_utc, has_results, manifest, results}
  plotSeries: new Map(), // name -> {mtime_utc, data}
  selected: new Set(),
  autoSelected: false,
  psdStep: null,
  gcodeText: "",
  drive: {
    data: null, // last /api/drive_state response (params, motors, config_pins, age_s)
    fetchedAtMs: null, // Date.now() when data was fetched, for a client-ticking age display
    pending: {}, // param name -> {motor: raw} — edits not yet applied
    dirty: new Set(), // autofill-target param names the user has edited directly this session
  },
  live: {
    name: null, // capture file currently streamed
    nextOffset: null, // null = attach at EOF (offset=end); numeric afterwards
    fsHz: null,
    t: [], // seconds since stream start, one per kept point
    perDrive: {}, // drive -> ferr values, same length as t
    timers: [], // interval ids cleared on page switch
    polling: false,
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

function el(id) {
  return document.getElementById(id);
}

// --- run data ---------------------------------------------------------------

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

async function ensurePlotSeries(name) {
  const run = state.runs.find((r) => r.name === name);
  const cached = state.plotSeries.get(name);
  if (cached && run && cached.mtime_utc === run.mtime_utc) return cached.data;
  const data = await api(`/api/runs/${encodeURIComponent(name)}/plot_series`);
  state.plotSeries.set(name, { mtime_utc: run ? run.mtime_utc : null, data });
  return data;
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

function pageRuns(def) {
  if (!def.experiments) return state.runs;
  return state.runs.filter((r) => def.experiments.includes(r.experiment));
}

function shortTime(mtimeUtc) {
  const m = /T(\d{2}:\d{2}:\d{2})/.exec(mtimeUtc);
  return m ? m[1] : mtimeUtc;
}

function verdictCellHtml(run, results) {
  if (!results) {
    return run.has_results
      ? '<span class="note">loading…</span>'
      : '<span class="note">no results yet</span>';
  }
  const v = results.verdict;
  const flags = [...new Set(results.steps.flatMap((s) => s.flags))];
  const flagBadges = flags
    .map((f) => {
      const cls =
        f === "resonance_detected" ? "resonance" : f === "torque_saturated" ? "torque" : "truncated";
      return `<span class="badge ${cls}" title="${f}">${f.split("_")[0]}</span>`;
    })
    .join("");
  const head = v.recommended_step
    ? `<span class="badge step">${v.recommended_step}</span>`
    : `<span class="badge none">none</span>`;
  return `${head}${flagBadges}<span class="hint" title="${v.reason}"> ${v.reason}</span>`;
}

// --- page shell ---------------------------------------------------------------

function currentPageDef() {
  return PAGE_DEFS[state.page] || PAGE_DEFS[DEFAULT_PAGE];
}

function pageFromHash() {
  const m = /^#\/?([a-z]+)/.exec(location.hash || "");
  return m && PAGE_DEFS[m[1]] ? m[1] : DEFAULT_PAGE;
}

function renderTabs() {
  const nav = el("page-tabs");
  nav.innerHTML = Object.entries(PAGE_DEFS)
    .map(
      ([key, def]) =>
        `<a href="#/${key}" class="tab${key === state.page ? " active" : ""}">${def.label}</a>`
    )
    .join("");
}

function controlsSectionsHtml(def) {
  const parts = [];
  if (def.groups) {
    parts.push(
      `<section class="panel">` +
        `<div class="section-head"><h2>drive tuning</h2></div>` +
        `<div id="drive-groups"></div>` +
        `<div id="pending-preview" class="pending-preview"></div>` +
        `<div class="row"><button id="drive-apply-btn" disabled>apply</button>` +
        `<span class="note" id="drive-changed-count"></span></div>` +
        `</section>`
    );
  }
  if (def.fitRunner) {
    parts.push(
      `<section class="sweep">` +
        `<div class="section-head"><h2>fit dynamics</h2></div>` +
        `<div class="row"><input type="text" id="sweep-command" value="SERVO_FIT_DYNAMICS AXIS=X">` +
        `<button id="run-sweep-btn">run</button></div>` +
        `<p class="note">strokes the axis, fits inertia/friction per drive, prints the ` +
        `recommended inertia ratio and writes the feedforward profile.</p>` +
        `</section>`
    );
  } else {
    const templates = (def.templates || [])
      .map(
        (t, i) =>
          `<button class="template-btn" data-template="${i}" title="${t.title}">${t.label}</button>`
      )
      .join("");
    parts.push(
      `<section class="sweep">` +
        `<div class="section-head"><h2>sweep</h2><span class="note" id="form-run-name"></span></div>` +
        `<div class="row"><input type="text" id="sweep-command" ` +
        `placeholder="select a run to prefill, or type a command">` +
        `<button id="run-sweep-btn">run</button>${templates}</div>` +
        `</section>`
    );
  }
  parts.push(
    `<section class="session">` +
      `<details class="gcode-details"><summary>manual g-code</summary>` +
      `<textarea id="gcode-textarea" spellcheck="false"></textarea>` +
      `<div class="row"><button id="run-gcode">run</button></div>` +
      `</details>` +
      `<div id="run-status" class="status-line"></div>` +
      `<div class="section-head"><h2>session log</h2></div>` +
      `<div id="sent-log" class="sent-log"></div>` +
      `</section>`
  );
  return parts.join("");
}

function analysisSectionsHtml(def) {
  const parts = [];
  parts.push(
    `<section class="runs-section">` +
      `<div class="section-head"><h2>runs</h2>` +
      `<span class="note">${def.experiments ? def.experiments.join(", ") : "all experiments"} — click a row to chart it</span></div>` +
      `<div class="table-wrap runs-wrap"><table><thead><tr>` +
      `<th></th><th>time</th><th>tag</th><th>ambient diff vs previous</th><th>verdict</th><th></th>` +
      `</tr></thead><tbody id="journal-body"></tbody></table></div>` +
      `</section>`
  );
  if (def.charts && def.charts.includes("psd")) {
    parts.push(
      `<section class="psd-section">` +
        `<div class="section-head"><h2>following-error PSD</h2>` +
        `<div class="chips" id="psd-step-chips"></div></div>` +
        `<div class="charts" id="psd-charts"><p class="note">select runs above</p></div>` +
        `</section>`
    );
  }
  if (def.peaks) {
    parts.push(
      `<section class="peaks-section">` +
        `<div class="section-head"><h2>detected peaks</h2><span class="note" id="peaks-run"></span></div>` +
        `<div id="peak-list"><p class="note">select runs above</p></div>` +
        `</section>`
    );
  }
  if (def.charts && def.charts.includes("time")) {
    parts.push(
      `<section class="time-section">` +
        `<div class="section-head"><h2>time domain — following error</h2></div>` +
        `<div class="charts" id="charts"><p class="note">select runs above</p></div>` +
        `</section>`
    );
  }
  return parts.join("");
}

function liveShellHtml() {
  return (
    `<div class="workspace">` +
    `<main class="analysis">` +
    `<section class="live-section">` +
    `<div class="section-head"><h2>live following error — per motor</h2>` +
    `<span class="note" id="live-status">no capture yet</span></div>` +
    `<div class="charts" id="live-charts">` +
    `<p class="note">charts appear when a running capture writes new samples — ` +
    `old capture files are never replayed</p>` +
    `</div>` +
    `</section>` +
    `</main>` +
    `<aside class="controls">` +
    `<section class="sweep">` +
    `<div class="section-head"><h2>capture</h2></div>` +
    `<div class="row"><input type="text" id="live-start-command" ` +
    `value="SERVO_CAPTURE_START NAME=live AXIS=X">` +
    `<button id="live-start-btn">start</button>` +
    `<button id="live-stop-btn">stop</button></div>` +
    `<p class="note">start begins an open-ended capture; the chart tails the ` +
    `growing file. stop leaves a normal analyzable .scap in the captures root.</p>` +
    `</section>` +
    `<section class="session">` +
    `<details class="gcode-details"><summary>manual g-code</summary>` +
    `<textarea id="gcode-textarea" spellcheck="false"></textarea>` +
    `<div class="row"><button id="run-gcode">run</button></div>` +
    `</details>` +
    `<div id="run-status" class="status-line"></div>` +
    `<div class="section-head"><h2>session log</h2></div>` +
    `<div id="sent-log" class="sent-log"></div>` +
    `</section>` +
    `</aside>` +
    `</div>`
  );
}

function renderPage() {
  renderTabs();
  const def = currentPageDef();
  const root = el("page-root");
  stopLivePolling();
  if (def.live) {
    root.innerHTML = liveShellHtml();
    bindPageEvents();
    bindLiveEvents();
    renderSentLog();
    startLivePolling();
    return;
  }
  if (def.journal) {
    root.innerHTML =
      `<div class="workspace single">` +
      `<main class="analysis">` +
      `<section class="runs-section">` +
      `<div class="section-head"><h2>journal — every run</h2></div>` +
      `<div class="table-wrap journal-wrap"><table><thead><tr>` +
      `<th></th><th>time</th><th>experiment/tag</th><th>ambient diff vs previous</th><th>verdict</th><th></th>` +
      `</tr></thead><tbody id="journal-body"></tbody></table></div>` +
      `</section>` +
      `<section class="session">` +
      `<details class="gcode-details"><summary>manual g-code</summary>` +
      `<textarea id="gcode-textarea" spellcheck="false"></textarea>` +
      `<div class="row"><button id="run-gcode">run</button></div>` +
      `</details>` +
      `<div id="run-status" class="status-line"></div>` +
      `<div class="section-head"><h2>session log</h2></div>` +
      `<div id="sent-log" class="sent-log"></div>` +
      `</section>` +
      `</main></div>`;
  } else {
    root.innerHTML =
      `<div class="workspace">` +
      `<main class="analysis">${analysisSectionsHtml(def)}</main>` +
      `<aside class="controls">${controlsSectionsHtml(def)}</aside>` +
      `</div>`;
  }
  bindPageEvents();
  renderRuns();
  renderDriveGroups();
  renderSentLog();
  redrawCharts();
}

function bindPageEvents() {
  const gcode = el("gcode-textarea");
  if (gcode) {
    gcode.value = state.gcodeText;
    gcode.addEventListener("input", () => {
      state.gcodeText = gcode.value;
    });
  }
  const runBtn = el("run-gcode");
  if (runBtn) runBtn.addEventListener("click", () => runGcode(manualGcodeLines(), "manual"));
  const applyBtn = el("drive-apply-btn");
  if (applyBtn) applyBtn.addEventListener("click", applyDriveChanges);
  const sweepBtn = el("run-sweep-btn");
  if (sweepBtn) sweepBtn.addEventListener("click", runSweep);
  const def = currentPageDef();
  document.querySelectorAll("button.template-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      const t = def.templates[Number(btn.dataset.template)];
      const sweep = el("sweep-command");
      if (t && sweep) {
        sweep.value = t.command;
        const label = el("form-run-name");
        if (label) label.textContent = "template — edit values before running";
      }
    });
  });
}

// --- runs table ---------------------------------------------------------------

function renderRuns() {
  const tbody = el("journal-body");
  if (!tbody) return;
  const def = currentPageDef();
  const runs = def.journal ? state.runs : pageRuns(def);
  tbody.innerHTML = "";
  runs.forEach((run) => {
    const globalIdx = state.runs.indexOf(run);
    const detail = state.details.get(run.name);
    const manifest = detail && detail.manifest;
    const results = detail && detail.results;
    const prevManifest =
      globalIdx + 1 < state.runs.length
        ? (state.details.get(state.runs[globalIdx + 1].name) || {}).manifest
        : null;
    const diff = manifest ? ambientDiff(prevManifest, manifest) : "";

    const tr = document.createElement("tr");
    if (state.selected.has(run.name)) tr.classList.add("selected");
    if (run.has_results) tr.classList.add("selectable");
    tr.addEventListener("click", () => {
      if (!run.has_results) return;
      if (state.selected.has(run.name)) state.selected.delete(run.name);
      else state.selected.add(run.name);
      renderRuns();
      redrawCharts();
    });

    const dotTd = document.createElement("td");
    const colorIdx = selectedRunNames().indexOf(run.name);
    dotTd.innerHTML =
      colorIdx >= 0
        ? `<span class="swatch" style="background:${PALETTE[colorIdx % PALETTE.length]}"></span>`
        : "";
    tr.appendChild(dotTd);

    const timeTd = document.createElement("td");
    timeTd.textContent = shortTime(run.mtime_utc);
    timeTd.title = `${run.name} — ${run.mtime_utc}`;
    tr.appendChild(timeTd);

    const expTd = document.createElement("td");
    expTd.textContent = def.journal
      ? `${run.experiment}/${run.tag}${run.axis ? " " + run.axis : ""}`
      : `${run.tag}${run.axis ? " " + run.axis : ""}`;
    expTd.title = run.experiment;
    tr.appendChild(expTd);

    const diffTd = document.createElement("td");
    diffTd.className = diff ? "diff" : "diff empty";
    diffTd.textContent = diff || "—";
    if (diff) diffTd.title = diff;
    tr.appendChild(diffTd);

    const verdictTd = document.createElement("td");
    verdictTd.className = "verdict";
    verdictTd.innerHTML = verdictCellHtml(run, results);
    tr.appendChild(verdictTd);

    const actionTd = document.createElement("td");
    actionTd.className = "actions";
    const prefillBtn = document.createElement("button");
    prefillBtn.textContent = "→ sweep";
    prefillBtn.title = "prefill the sweep command from this run";
    prefillBtn.disabled = !manifest;
    prefillBtn.addEventListener("click", (e) => {
      e.stopPropagation();
      loadRerunForm(run.name);
    });
    actionTd.appendChild(prefillBtn);
    if (!run.has_results) {
      const analyzeBtn = document.createElement("button");
      analyzeBtn.textContent = "analyze";
      analyzeBtn.addEventListener("click", (e) => {
        e.stopPropagation();
        triggerAnalyze(run.name);
      });
      actionTd.appendChild(analyzeBtn);
    }
    tr.appendChild(actionTd);

    tbody.appendChild(tr);
  });
}

async function triggerAnalyze(name) {
  await api(`/api/runs/${encodeURIComponent(name)}/analyze`, { method: "POST" });
  await refresh();
}

function selectedRunNames() {
  return state.runs.filter((r) => state.selected.has(r.name)).map((r) => r.name);
}

/// First data load: preselect the newest few analyzed runs and prefill the
/// sweep command from the newest one, so the charts and the re-run box are
/// populated before any clicking.
function autoSelectInitialRuns() {
  if (state.autoSelected) return;
  const withResults = state.runs.filter((r) => r.has_results);
  if (!withResults.length) return;
  state.autoSelected = true;
  for (const run of withResults.slice(0, INITIAL_SELECTED_RUNS)) {
    state.selected.add(run.name);
  }
  const sweep = el("sweep-command");
  if (sweep && !sweep.value) loadRerunForm(withResults[0].name);
}

async function refresh() {
  const runs = await api("/api/runs");
  state.runs = runs;
  await Promise.all(runs.map((r) => ensureDetail(r).catch((e) => console.error(e))));
  const known = new Set(runs.map((r) => r.name));
  for (const name of [...state.selected]) {
    if (!known.has(name)) state.selected.delete(name);
  }
  autoSelectInitialRuns();
  renderRuns();
  await redrawCharts();
}

// --- chart drawing ------------------------------------------------------------

function pickSeries(step) {
  if (step.combined) {
    return { y: step.combined.on_ferr_mm, label: "on-axis ferr (mm)" };
  }
  const firstDrive = Object.values(step.drives)[0];
  return { y: firstDrive ? firstDrive.ferr_counts : [], label: "ferr (counts)" };
}

function drawChart(canvas, traces, yLabel, fixedY) {
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
  if (fixedY) {
    yMin = fixedY.yMin;
    yMax = fixedY.yMax;
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

function drawTimeDomain(names, plots) {
  const container = el("charts");
  if (!container) return;
  container.innerHTML = "";
  if (names.length === 0) {
    container.innerHTML = '<p class="note">select runs above</p>';
    return;
  }
  const stepNames = [...new Set(plots.flatMap((p) => p.steps.map((s) => s.name)))];
  for (const stepName of stepNames) {
    const box = document.createElement("div");
    box.className = "chart-box";
    const title = document.createElement("h3");
    title.textContent = stepName;
    box.appendChild(title);
    const canvas = document.createElement("canvas");
    canvas.width = 860;
    canvas.height = 200;
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
}

// --- following-error PSD --------------------------------------------------

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
        run: names[i],
      });
    }
    if (step.psd.accel) {
      traces.push({
        freq: step.psd.accel.freq_hz,
        y: step.psd.accel.psd,
        color,
        dashed: true,
        label: `${names[i]} (accel)`,
        run: names[i],
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
  const container = el("psd-charts");
  if (!container) return;
  container.innerHTML = "";
  if (!stepName) {
    container.innerHTML = '<p class="note">select runs above</p>';
    return;
  }
  const box = document.createElement("div");
  box.className = "chart-box";
  const canvas = document.createElement("canvas");
  canvas.width = 860;
  canvas.height = 280;
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

function renderPsdChips(stepNames) {
  const container = el("psd-step-chips");
  if (!container) return;
  container.innerHTML = "";
  for (const stepName of stepNames) {
    const chip = document.createElement("button");
    chip.className = "chip" + (stepName === state.psdStep ? " active" : "");
    chip.textContent = stepName;
    chip.addEventListener("click", () => {
      state.psdStep = stepName;
      redrawCharts();
    });
    container.appendChild(chip);
  }
}

// --- PSD peak list (notches page) -------------------------------------------

/// Greedy spaced peak-picking inside the resonance band: repeatedly take
/// the highest remaining bin at least PEAK_MIN_SEPARATION_HZ away from
/// every already-taken peak.
function findPsdPeaks(freq, psd, band, count) {
  const [blo, bhi] = band;
  const candidates = [];
  for (let i = 0; i < freq.length; i++) {
    if (freq[i] >= blo && freq[i] < bhi) candidates.push({ freq: freq[i], power: psd[i] });
  }
  candidates.sort((a, b) => b.power - a.power);
  const peaks = [];
  for (const c of candidates) {
    if (peaks.length >= count) break;
    if (peaks.every((p) => Math.abs(p.freq - c.freq) >= PEAK_MIN_SEPARATION_HZ)) {
      peaks.push(c);
    }
  }
  return peaks;
}

function notchSlotStates() {
  const params = state.drive.data ? state.drive.data.params : [];
  const slots = [];
  for (let n = 1; n <= 5; n++) {
    const freqParam = params.find((p) => p.name === `notch_${n}_freq`);
    if (!freqParam) continue;
    const motors = motorNames(state.drive.data.motors);
    const values = motors.map((m) => cellRaw(freqParam, m));
    const parked = values.every((v) => v === 8000);
    slots.push({ n, freqParam, parked, adaptive: n <= 2, current: values[0] });
  }
  return slots;
}

function proposePeakIntoSlot(slot, peakFreq) {
  const raw = Math.round(peakFreq);
  const targets = motorNames(state.drive.data.motors);
  const existing = { ...(state.drive.pending[slot.freqParam.name] || {}) };
  for (const m of targets) existing[m] = raw;
  state.drive.pending[slot.freqParam.name] = existing;
  renderDriveGroups();
}

function renderPeakList(names, plots) {
  const container = el("peak-list");
  if (!container) return;
  const runLabel = el("peaks-run");
  if (!names.length || !state.psdStep) {
    container.innerHTML = '<p class="note">select runs above</p>';
    if (runLabel) runLabel.textContent = "";
    return;
  }
  const newest = newestSelectedRunName(names);
  const plot = plots[names.indexOf(newest)];
  const step = plot && plot.steps.find((s) => s.name === state.psdStep);
  if (!step || !step.psd) {
    container.innerHTML = '<p class="note">no PSD for this step</p>';
    if (runLabel) runLabel.textContent = "";
    return;
  }
  if (runLabel) runLabel.textContent = `${newest} / ${state.psdStep}`;
  const driveNames = Object.keys(step.psd.per_drive);
  const peaks = findPsdPeaks(
    step.psd.freq_hz,
    step.psd.per_drive[driveNames[0]],
    RESONANCE_BAND_HZ,
    PEAK_LIST_SIZE
  );
  if (!peaks.length) {
    container.innerHTML = '<p class="note">no peaks in the 20–450 Hz band</p>';
    return;
  }
  const slots = state.drive.data ? notchSlotStates() : [];
  container.innerHTML = peaks
    .map((p) => {
      const buttons = slots
        .map((s) => {
          const label = s.parked ? `→ notch ${s.n}` : `→ notch ${s.n} (${s.current}Hz)`;
          const title = s.adaptive
            ? `notch ${s.n} is adaptive while adaptive_notch_mode is ${s.n <= 1 ? "1 or 2" : "2"} — the drive will overwrite it`
            : `set notch_${s.n}_freq to ${Math.round(p.freq)} on all motors (width/depth stay yours)`;
          return `<button class="peak-slot" data-slot="${s.n}" data-freq="${p.freq}" title="${title}">${label}</button>`;
        })
        .join("");
      return (
        `<div class="peak-row"><span class="peak-freq">${p.freq.toFixed(1)} Hz</span>` +
        `<span class="hint">${p.power.toExponential(1)} counts²/Hz</span>${buttons}</div>`
      );
    })
    .join("");
  container.querySelectorAll("button.peak-slot").forEach((btn) => {
    btn.addEventListener("click", () => {
      const slot = notchSlotStates().find((s) => s.n === Number(btn.dataset.slot));
      if (slot) proposePeakIntoSlot(slot, Number(btn.dataset.freq));
    });
  });
}

/// Redraw the current page's chart sections from the run selection. Plot
/// series are cached per run mtime, so reselecting is cheap.
async function redrawCharts() {
  const def = currentPageDef();
  if (def.journal) return;
  const names = selectedRunNames();
  const plots = [];
  const okNames = [];
  for (const n of names) {
    try {
      plots.push(await ensurePlotSeries(n));
      okNames.push(n);
    } catch (e) {
      console.error(e);
    }
  }
  const stepNames = [...new Set(plots.flatMap((p) => p.steps.map((s) => s.name)))];
  if (!state.psdStep || !stepNames.includes(state.psdStep)) {
    state.psdStep = stepNames.length ? defaultPsdStep(okNames, plots, stepNames) : null;
  }
  if (def.charts && def.charts.includes("psd")) {
    renderPsdChips(stepNames);
    renderPsdChart(okNames, plots, state.psdStep);
  }
  if (def.peaks) renderPeakList(okNames, plots);
  if (def.charts && def.charts.includes("time")) drawTimeDomain(okNames, plots);
}

// --- live tail ----------------------------------------------------------------
//
// Polls /api/live for the newest flat capture, then streams
// /api/live/<name>?offset=<next_offset> — the offset handshake is the
// server's contract (always record-aligned). Attaching uses `offset=end`,
// so the charts only ever show samples written after the stream started:
// an idle old capture draws nothing instead of masquerading as live data.
// A new capture name resets the stream; each motor gets its own stacked
// chart over the last LIVE_WINDOW_S seconds, all on a shared y-scale so
// the noisy motor stands out.

function bindLiveEvents() {
  el("live-start-btn").addEventListener("click", () => {
    const line = el("live-start-command").value.trim();
    if (line) runGcode([line], "live");
  });
  el("live-stop-btn").addEventListener("click", () => runGcode(["SERVO_CAPTURE_STOP"], "live"));
}

function resetLiveStream(name) {
  state.live.name = name;
  state.live.nextOffset = null;
  state.live.fsHz = null;
  state.live.t = [];
  state.live.perDrive = {};
  const container = el("live-charts");
  if (container) {
    container.innerHTML =
      '<p class="note">charts appear when a running capture writes new samples — ' +
      "old capture files are never replayed</p>";
  }
}

async function pollLiveStatus() {
  let status;
  try {
    status = await api("/api/live");
  } catch (e) {
    const label = el("live-status");
    if (label) label.textContent = String(e);
    return;
  }
  const label = el("live-status");
  if (!status.capture) {
    if (label) label.textContent = "no capture in the captures root yet — press start";
    return;
  }
  const cap = status.capture;
  if (cap.name !== state.live.name) resetLiveStream(cap.name);
  if (label) {
    const kb = (cap.size_bytes / 1024).toFixed(0);
    const growing = cap.age_s !== null && cap.age_s < 3;
    label.textContent = growing
      ? `recording ${cap.name} — ${kb} KiB`
      : `${cap.name} idle ${formatAge(cap.age_s)} — not recording, press start`;
  }
}

async function pollLiveTail() {
  if (state.live.polling || !state.live.name) return;
  state.live.polling = true;
  try {
    const offset = state.live.nextOffset === null ? "end" : state.live.nextOffset;
    const payload = await api(
      `/api/live/${encodeURIComponent(state.live.name)}?offset=${offset}`
    );
    appendLiveSamples(payload);
    drawLiveCharts();
  } catch (e) {
    console.error(e);
  } finally {
    state.live.polling = false;
  }
}

function appendLiveSamples(payload) {
  state.live.nextOffset = payload.next_offset;
  state.live.fsHz = payload.fs_hz;
  const n = payload.moving.length;
  if (!n) return;
  const dt = payload.stride / payload.fs_hz;
  const t0 = payload.first_record / payload.fs_hz;
  for (let i = 0; i < n; i++) state.live.t.push(t0 + i * dt);
  for (const [drive, series] of Object.entries(payload.drives)) {
    if (!state.live.perDrive[drive]) {
      state.live.perDrive[drive] = new Array(state.live.t.length - n).fill(0);
    }
    state.live.perDrive[drive].push(...series.ferr);
  }
  const cutoff = state.live.t[state.live.t.length - 1] - LIVE_WINDOW_S;
  let drop = 0;
  while (drop < state.live.t.length && state.live.t[drop] < cutoff) drop++;
  if (drop > 0) {
    state.live.t.splice(0, drop);
    for (const series of Object.values(state.live.perDrive)) series.splice(0, drop);
  }
}

function liveChartId(drive) {
  return `live-canvas-${drive}`;
}

function ensureLiveChartBoxes(drives) {
  const container = el("live-charts");
  if (!container) return false;
  const have = [...container.querySelectorAll("canvas")].map((c) => c.id).join();
  const want = drives.map(liveChartId).join();
  if (have !== want) {
    container.innerHTML = drives
      .map(
        (d, i) =>
          `<div class="chart-box">` +
          `<h3><span class="swatch" style="background:${PALETTE[i % PALETTE.length]}"></span>` +
          `${d} <span class="note" id="live-peak-${d}"></span></h3>` +
          `<canvas id="${liveChartId(d)}" width="860" height="130"></canvas>` +
          `</div>`
      )
      .join("");
  }
  return true;
}

function drawLiveCharts() {
  if (!state.live.t.length) return;
  const drives = Object.keys(state.live.perDrive).sort();
  if (!drives.length || !ensureLiveChartBoxes(drives)) return;
  let yMin = Infinity;
  let yMax = -Infinity;
  const peaks = {};
  for (const d of drives) {
    let peak = 0;
    for (const v of state.live.perDrive[d]) {
      if (v < yMin) yMin = v;
      if (v > yMax) yMax = v;
      const mag = Math.abs(v);
      if (mag > peak) peak = mag;
    }
    peaks[d] = peak;
  }
  drives.forEach((d, i) => {
    const canvas = el(liveChartId(d));
    if (!canvas) return;
    drawChart(
      canvas,
      [{ t: state.live.t, y: state.live.perDrive[d], color: PALETTE[i % PALETTE.length] }],
      "ferr (counts)",
      { yMin, yMax }
    );
    const label = el(`live-peak-${d}`);
    if (label) label.textContent = `peak |ferr| ${peaks[d]}`;
  });
}

function startLivePolling() {
  pollLiveStatus();
  state.live.timers = [
    setInterval(pollLiveStatus, LIVE_STATUS_POLL_MS),
    setInterval(pollLiveTail, LIVE_TAIL_POLL_MS),
  ];
}

function stopLivePolling() {
  for (const id of state.live.timers) clearInterval(id);
  state.live.timers = [];
}

// --- drive tuning grid --------------------------------------------------------
//
// Renders purely from GET /api/drive_state (servo_tuning.PANEL_PARAMS shape,
// docs/rewrite/servo-tuning-profiles.md) as a param × motor grid: one column
// per motor plus an "all" setter, so a 4-motor bench never needs the same
// value typed four times and every edit's motor scope is visible. Each page
// shows only its own param groups. Pure helpers first — display/raw unit
// conversion, autofill derivation, changed-cell diffing, SERVO_TUNE line
// building (always with an explicit MOTORS= list) — the logic a Rust test
// asserts is present and exercisable without a browser; DOM rendering and
// event wiring follow.

const GROUP_ORDER = ["gains", "filters", "notch", "speed_observer", "disturbance_observer", "load"];
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

function motorNames(motors) {
  return Object.keys(motors).sort();
}

function motorRawValues(motors, cCode) {
  return motorNames(motors).map((m) => motors[m][cCode]);
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

/// Effective raw value of one grid cell: the session's pending edit if any,
/// else the drive's reading from the last dump.
function cellRaw(param, motor) {
  const pend = state.drive.pending[param.name];
  if (pend && pend[motor] !== undefined) return pend[motor];
  return state.drive.data.motors[motor][param.c_code];
}

/// Which cells differ from the drive_state's per-motor readings, given this
/// session's pending edits. `pending[name]` is always a `{motor: raw}` map —
/// the "all" column just writes every motor at once.
function diffChangedParams(params, motors, pending) {
  const changed = [];
  for (const p of params) {
    const pend = pending[p.name];
    if (pend === undefined) continue;
    const cells = [];
    for (const motor of Object.keys(pend).sort()) {
      if (motors[motor][p.c_code] !== pend[motor]) cells.push({ motor, value: pend[motor] });
    }
    if (cells.length) changed.push({ name: p.name, cells });
  }
  return changed;
}

/// One SERVO_TUNE line per (param, value), motors grouped — the MOTORS= list
/// is always explicit so the log and the preview state exactly which drives
/// a write targets.
function buildServoTuneCommands(changed) {
  const lines = [];
  for (const c of changed) {
    const byValue = new Map();
    for (const { motor, value } of c.cells) {
      if (!byValue.has(value)) byValue.set(value, []);
      byValue.get(value).push(motor);
    }
    for (const [value, motorList] of byValue) {
      lines.push(`SERVO_TUNE PARAM=${c.name} VALUE=${value} MOTORS=${motorList.join(",")}`);
    }
  }
  return lines;
}

function paramByName(name) {
  return state.drive.data.params.find((p) => p.name === name);
}

/// speed_gain's effective per-motor raws — the input every autofill formula
/// maps over.
function currentSpeedGainByMotor() {
  const speedParam = paramByName(AUTOFILL_SOURCE_PARAM);
  const out = {};
  for (const m of motorNames(state.drive.data.motors)) {
    out[m] = cellRaw(speedParam, m);
  }
  return out;
}

/// speed_gain changed: push derived per-motor values into every autofill
/// target the user hasn't dirtied (edited directly) this session.
function propagateAutofill() {
  const speedByMotor = currentSpeedGainByMotor();
  for (const param of state.drive.data.params) {
    const formula = AUTOFILL_FORMULAS[param.autofill];
    if (!formula || state.drive.dirty.has(param.name)) continue;
    state.drive.pending[param.name] = Object.fromEntries(
      Object.entries(speedByMotor).map(([m, v]) => [m, formula(v)])
    );
  }
}

function rederiveAutofillTarget(name) {
  const formula = AUTOFILL_FORMULAS[paramByName(name).autofill];
  if (!formula) return;
  state.drive.pending[name] = Object.fromEntries(
    Object.entries(currentSpeedGainByMotor()).map(([m, v]) => [m, formula(v)])
  );
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

/// The refresh button must render even with no drive state at all —
/// SERVO_DUMP_TUNING is what creates drive_state.json in the first place,
/// so hiding the button behind loaded data would deadlock a fresh bench.
/// Rebuilt only once so the 1 s age ticker doesn't wipe the refresh
/// status text mid-dump.
function renderDriveBanner() {
  const banner = el("drive-state-banner");
  if (!el("drive-refresh-btn")) {
    banner.innerHTML =
      `<span class="note" id="drive-age"></span> ` +
      `<button id="drive-refresh-btn" title="SERVO_DUMP_TUNING and re-read">refresh</button>` +
      `<span id="drive-refresh-status" class="note"></span>`;
    el("drive-refresh-btn").addEventListener("click", refreshDriveState);
  }
  el("drive-age").textContent = state.drive.data
    ? `drive state ${formatAge(currentDriveAgeS())} old`
    : "no drive state yet — press refresh to read the drives";
}

function shortMotorLabel(motor) {
  return motor.replace(/^motor_/, "");
}

function cellInputHtml(param, motor) {
  const raw = cellRaw(param, motor);
  const original = state.drive.data.motors[motor][param.c_code];
  const cls = ["cell-input"];
  if (raw !== original) cls.push("pending");
  const others = motorNames(state.drive.data.motors)
    .filter((m) => m !== motor)
    .map((m) => cellRaw(param, m));
  if (others.some((v) => v !== raw)) cls.push("drift");
  const titleText = `${motor} — raw ${raw}${raw !== original ? ` (drive has ${original})` : ""}`;
  if (param.options) {
    const opts = Object.entries(param.options)
      .map(
        ([v, label]) =>
          `<option value="${v}"${Number(v) === raw ? " selected" : ""}>${v}: ${label}</option>`
      )
      .join("");
    return `<select class="${cls.join(" ")}" data-param="${param.name}" data-motor="${motor}" title="${titleText}">${opts}</select>`;
  }
  return `<input type="number" step="any" class="${cls.join(" ")}" data-param="${param.name}" data-motor="${motor}" value="${rawToDisplay(raw, param.scale)}" title="${titleText}">`;
}

function allInputHtml(param) {
  const values = motorNames(state.drive.data.motors).map((m) => cellRaw(param, m));
  const agree = valuesAgree(values);
  if (param.options) {
    const opts =
      `<option value=""${agree ? "" : " selected"} disabled>${agree ? "" : "mixed"}</option>` +
      Object.entries(param.options)
        .map(
          ([v, label]) =>
            `<option value="${v}"${agree && Number(v) === values[0] ? " selected" : ""}>${v}: ${label}</option>`
        )
        .join("");
    return `<select class="cell-input all" data-param="${param.name}" data-motor="*" title="set all motors">${opts}</select>`;
  }
  const display = agree ? rawToDisplay(values[0], param.scale) : "";
  return `<input type="number" step="any" class="cell-input all" data-param="${param.name}" data-motor="*" value="${display}" placeholder="${agree ? "" : "mixed"}" title="set all motors">`;
}

function paramLabelHtml(param, section) {
  const pins = pinnedEntries(state.drive.data.config_pins, param.c_code);
  const pinnedNames = Object.keys(pins);
  const pinBadge = pinnedNames.length
    ? `<span class="badge pin" title="pinned in config — a restart re-applies ${[...new Set(pinnedNames.map((m) => pins[m]))].join("/")}">pin</span>`
    : "";
  const groupHint = section === OTHER_GROUP ? `<span class="hint">(${param.group})</span>` : "";
  const rederiveLink = state.drive.dirty.has(param.name)
    ? ` <a href="#" class="rederive" data-param="${param.name}" title="restore the autofill link">re-derive</a>`
    : "";
  const unit = param.unit ? ` <span class="unit">${param.unit}</span>` : "";
  return `<span title="${param.description} (${param.c_code})">${param.name}</span>${unit}${pinBadge}${groupHint}${rederiveLink}`;
}

/// Adaptive-notch recipe as one-click actions (A6-EC manual 7.10 + the
/// bench's own macros): reset the notch parameters, hand notches 1-2 to the
/// drive, or take them back (0 keeps whatever the drive last wrote).
const NOTCH_QUICK_ACTIONS = [
  { label: "reset notch params", value: 3 },
  { label: "1 adaptive", value: 1 },
  { label: "2 adaptive", value: 2 },
  { label: "disable adaptive", value: 0 },
];

function notchQuickActionsHtml() {
  return (
    '<div class="quick-actions">' +
    NOTCH_QUICK_ACTIONS.map(
      (a) =>
        `<button class="quick-action" data-value="${a.value}" title="SERVO_TUNE PARAM=adaptive_notch_mode VALUE=${a.value} MOTORS=&lt;all&gt;">${a.label}</button>`
    ).join("") +
    "</div>"
  );
}

function renderDriveGroups() {
  const container = el("drive-groups");
  if (!container) return;
  const def = currentPageDef();
  const data = state.drive.data;
  if (!data) {
    container.innerHTML =
      '<p class="note">no drive state yet — press refresh in the top bar ' +
      "to read every mapped parameter off the drives (SERVO_DUMP_TUNING)</p>";
    updateApplyState();
    return;
  }
  const motors = motorNames(data.motors);
  const headerCells =
    `<th class="param-col"></th>` +
    motors.map((m) => `<th title="${m}">${shortMotorLabel(m)}</th>`).join("") +
    `<th class="all-col">all</th>`;
  const sections = groupParams(data.params);
  const parts = [];
  for (const [group, params] of sections) {
    if (!params.length) continue;
    if (def.groups && group !== OTHER_GROUP && !def.groups.includes(group)) continue;
    const rows = params
      .map((p) => {
        const cells = motors.map((m) => `<td>${cellInputHtml(p, m)}</td>`).join("");
        return (
          `<tr data-param="${p.name}">` +
          `<td class="param-col">${paramLabelHtml(p, group)}</td>` +
          cells +
          `<td class="all-col">${allInputHtml(p)}</td>` +
          `</tr>`
        );
      })
      .join("");
    const extras = group === "notch" ? notchQuickActionsHtml() : "";
    parts.push(
      `<div class="param-group"><h3>${group.replace(/_/g, " ")}</h3>${extras}` +
        `<table class="param-grid"><thead><tr>${headerCells}</tr></thead><tbody>${rows}</tbody></table></div>`
    );
  }
  container.innerHTML = parts.join("");
  bindDriveGridEvents();
  updateApplyState();
}

function bindDriveGridEvents() {
  const container = el("drive-groups");
  container.querySelectorAll(".cell-input").forEach((input) => {
    input.addEventListener("change", onDriveCellChange);
  });
  container.querySelectorAll("a.rederive").forEach((elink) => {
    elink.addEventListener("click", (e) => {
      e.preventDefault();
      state.drive.dirty.delete(elink.dataset.param);
      rederiveAutofillTarget(elink.dataset.param);
      renderDriveGroups();
    });
  });
  container.querySelectorAll("button.quick-action").forEach((btn) => {
    btn.addEventListener("click", () => {
      const motors = motorNames(state.drive.data.motors).join(",");
      runGcode(
        [`SERVO_TUNE PARAM=adaptive_notch_mode VALUE=${btn.dataset.value} MOTORS=${motors}`],
        "notch"
      ).then(refreshDriveState);
    });
  });
}

function onDriveCellChange(e) {
  const input = e.target;
  const name = input.dataset.param;
  const param = paramByName(name);
  const raw = param.options
    ? parseInt(input.value, 10)
    : displayToRaw(parseFloat(input.value), param.scale);
  if (Number.isNaN(raw)) return;
  const targets =
    input.dataset.motor === "*" ? motorNames(state.drive.data.motors) : [input.dataset.motor];
  const existing = { ...(state.drive.pending[name] || {}) };
  for (const m of targets) existing[m] = raw;
  state.drive.pending[name] = existing;
  if (name === AUTOFILL_SOURCE_PARAM) {
    propagateAutofill();
  } else if (param.autofill) {
    state.drive.dirty.add(name);
  }
  renderDriveGroups();
}

function updateApplyState() {
  const btn = el("drive-apply-btn");
  const label = el("drive-changed-count");
  const preview = el("pending-preview");
  if (!btn || !label || !preview) return;
  if (!state.drive.data) {
    btn.disabled = true;
    label.textContent = "";
    preview.innerHTML = "";
    return;
  }
  const changed = diffChangedParams(state.drive.data.params, state.drive.data.motors, state.drive.pending);
  const lines = buildServoTuneCommands(changed);
  btn.disabled = lines.length === 0;
  label.textContent = lines.length ? `${lines.length} write(s) pending` : "no changes pending";
  preview.innerHTML = lines.map((l) => `<div class="pending-line">${l}</div>`).join("");
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
  renderDriveBanner();
  renderDriveGroups();
}

async function refreshDriveState() {
  const statusEl = el("drive-refresh-status");
  const priorAge = state.drive.data ? currentDriveAgeS() : Infinity;
  if (statusEl) statusEl.textContent = " dumping…";
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
      renderDriveBanner();
      renderDriveGroups();
      return;
    }
  }
  const late = el("drive-refresh-status");
  if (late) late.textContent = " refresh timed out — drive_state.json never got newer";
}

/// Apply sends the previewed SERVO_TUNE batch, then re-dumps the drives —
/// SERVO_TUNE readback-verifies each write but does not rewrite
/// drive_state.json, so without the re-dump the grid would snap back to
/// stale values and the apply would look like a no-op.
async function applyDriveChanges() {
  const changed = diffChangedParams(state.drive.data.params, state.drive.data.motors, state.drive.pending);
  const lines = buildServoTuneCommands(changed);
  if (!lines.length) return;
  await runGcode(lines, "apply");
  await refreshDriveState();
}

// --- sweep re-run ---------------------------------------------------------

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
    case "gain_ladder": {
      const speeds = manifest.steps.map((s) => s.swept.speed);
      const safe = speeds[0];
      const start = speeds.length > 1 ? speeds[1] : safe;
      const step = speeds.length > 2 ? speeds[2] - speeds[1] : 50;
      const max = speeds[speeds.length - 1];
      return `SERVO_GAIN_LADDER SAFE=${safe} START=${start} STEP=${step} MAX=${max} AXIS=${axis} ITERATIONS=${iterations} TAG=${tag}`;
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
  const label = el("form-run-name");
  if (label) label.textContent = `from ${name}`;
  const sweep = el("sweep-command");
  if (sweep) sweep.value = reconstructCommand(detail.manifest);
}

// --- moonraker plumbing + session log ---------------------------------------

function moonrakerUrl() {
  return el("moonraker-url").value.replace(/\/+$/, "");
}

/// Every button on every page posts G-code through Moonraker, so a broken
/// URL or missing cors_domains entry silently kills the whole dashboard.
/// This badge in the topbar turns that failure mode into words.
async function pollMoonrakerHealth() {
  const badge = el("moonraker-health");
  if (!badge) return;
  try {
    const resp = await fetch(`${moonrakerUrl()}/server/info`);
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const info = (await resp.json()).result;
    badge.className = "mr-health ok";
    badge.textContent = `klippy ${info.klippy_state || "unknown"}`;
  } catch (e) {
    badge.className = "mr-health err";
    badge.textContent = "moonraker unreachable — bad URL, moonraker down, or origin missing from cors_domains";
  }
}

function sentEntryHtml(entry) {
  const ok = entry.results.length > 0 && entry.results.every((r) => r.ok);
  return (
    `<div class="sent-entry">` +
    `<div class="sent-head">${shortTime(entry.time)} — ${entry.label} — ` +
    `<span class="${ok ? "status-ok" : "status-err"}">${ok ? "ok" : "error"}</span></div>` +
    entry.lines
      .map((l, i) => {
        const r = entry.results[i];
        const suffix = r && !r.ok ? ` <span class="status-err">HTTP ${r.status}</span>` : "";
        return `<div class="sent-line">${l}${suffix}</div>`;
      })
      .join("") +
    `</div>`
  );
}

function renderSentLog() {
  const container = el("sent-log");
  if (!container) return;
  container.innerHTML = state.sentLog.length
    ? state.sentLog.map(sentEntryHtml).join("")
    : '<p class="note">nothing sent yet</p>';
  container.scrollTop = container.scrollHeight;
}

/// Sends `lines` (already-built gcode) through the shared Moonraker
/// plumbing — the grid's Apply, the notch quick actions, the sweep row and
/// the manual textarea all land in the same session log, which survives
/// page switches.
async function runGcode(lines, label) {
  const base = moonrakerUrl();
  const statusEl = el("run-status");
  if (statusEl) statusEl.textContent = "";
  const entry = { time: new Date().toISOString(), label, lines: [], results: [] };
  for (const line of lines) {
    const url = `${base}/printer/gcode/script?script=${encodeURIComponent(line)}`;
    entry.lines.push(line);
    try {
      const resp = await fetch(url, { method: "POST" });
      const text = await resp.text();
      if (!resp.ok && statusEl) {
        statusEl.innerHTML += `<div class="status-err">${line} -> HTTP ${resp.status} ${text.slice(0, 200)}</div>`;
      }
      entry.results.push({ ok: resp.ok, status: resp.status });
      if (!resp.ok) break;
    } catch (e) {
      if (statusEl) statusEl.innerHTML += `<div class="status-err">${line} -> ${e}</div>`;
      entry.results.push({ ok: false, status: 0 });
      break;
    }
  }
  state.sentLog.push(entry);
  renderSentLog();
}

function manualGcodeLines() {
  const gcode = el("gcode-textarea");
  if (!gcode) return [];
  return gcode.value
    .split("\n")
    .map((l) => l.trim())
    .filter((l) => l.length && !l.startsWith(";"));
}

async function runSweep() {
  const sweep = el("sweep-command");
  if (!sweep) return;
  const line = sweep.value.trim();
  if (!line || line.startsWith(";")) return;
  await runGcode([line], "sweep");
}

// --- boot -------------------------------------------------------------------

function initShell() {
  const input = el("moonraker-url");
  input.value = localStorage.getItem(MOONRAKER_KEY) || `http://${location.hostname}:7125`;
  input.addEventListener("change", () => {
    localStorage.setItem(MOONRAKER_KEY, input.value);
    pollMoonrakerHealth();
  });
  pollMoonrakerHealth();
  setInterval(pollMoonrakerHealth, MOONRAKER_HEALTH_POLL_MS);
  window.addEventListener("hashchange", () => {
    state.page = pageFromHash();
    renderPage();
  });
  state.page = pageFromHash();
  renderPage();
}

async function tick() {
  try {
    await refresh();
  } catch (e) {
    console.error(e);
  }
  renderDriveBanner();
}

initShell();
tick();
loadDriveState();
setInterval(tick, REFRESH_MS);
setInterval(renderDriveBanner, 1000);
