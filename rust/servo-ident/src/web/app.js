"use strict";

const REFRESH_MS = 5000;
const MOONRAKER_KEY = "servoCalMoonrakerUrl";
const CONSOLE_HISTORY_KEY = "servoCalConsoleHistory";
const CONSOLE_HISTORY_MAX = 500;
const PALETTE = ["#4fb3ff", "#e05a4f", "#4caf50", "#d9a441", "#b388ff", "#4fd8c4"];
const RESONANCE_BAND_HZ = [20, 450];
const PSD_MAX_FREQ_HZ = 500;
const INITIAL_SELECTED_RUNS = 1;
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
    experiments: ["tracking", "inertia_grid", "differential"],
    charts: ["frf"],
    intro: "identify the load, then let feedforward carry it",
    templates: [
      {
        label: "fit…",
        command: "SERVO_FIT_DYNAMICS AXIS=X",
        title:
          "strokes the axis, fits inertia/friction per drive, prints the recommended " +
          "inertia ratio and writes the feedforward profile",
      },
    ],
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
const MOONRAKER_HEALTH_POLL_MS = 5000;

const state = {
  page: DEFAULT_PAGE,
  runs: [],
  details: new Map(), // name -> {mtime_utc, has_results, manifest, results}
  plotSeries: new Map(), // name -> {mtime_utc, data}
  selected: new Set(),
  autoSelected: false,
  stepFilter: null, // null = every step; otherwise a Set of visible step names
  console: {
    text: "", // current input line, survives page switches
    history: loadConsoleHistory(),
    cursor: null, // history index while navigating; null = editing a fresh line
    draft: "", // the fresh line stashed when history navigation starts
    search: null, // {query, pos, saved, failed} while ctrl+r reverse search is live
  },
  drive: {
    data: null, // last /api/drive_state response (params, motors, config_pins, age_s)
    fetchedAtMs: null, // Date.now() when data was fetched, for a client-ticking age display
    pending: {}, // param name -> {motor: raw} — edits not yet applied
    dirty: new Set(), // autofill-target param names the user has edited directly this session
    notchPerMotor: false, // compact one-value-per-notch grid unless toggled
    adaptiveOpen: false, // the adaptive-recipes fold survives re-renders
  },
  live: {
    cursor: null, // last next_cycle from /api/live_tap; null = attach now
    fsHz: null,
    cycle0: null, // first streamed cycle_index — the chart's t=0
    lastCycle: null, // cycle_index of the last kept sample, for gap breaks
    t: [], // seconds since stream start, one per kept point
    perDrive: {}, // tap drive name -> {ferr, torque} arrays (null = gap break)
    windowS: 10, // seconds kept and drawn, set by the slider
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
  parts.push(consoleSectionHtml(def));
  return parts.join("");
}

function consoleSectionHtml(def) {
  const templates = (def.templates || [])
    .map(
      (t, i) =>
        `<button class="template-btn" data-template="${i}" title="${t.title}">${t.label}</button>`
    )
    .join("");
  return (
    `<section class="session">` +
    `<div class="section-head"><h2>console</h2>` +
    `<span class="note" id="form-run-name"></span>${templates}</div>` +
    `<div id="sent-log" class="sent-log"></div>` +
    `<div id="run-status" class="status-line"></div>` +
    `<div class="console-line"><span class="console-prompt">›</span>` +
    `<textarea id="console-input" rows="1" spellcheck="false" ` +
    `placeholder="g-code — enter runs, shift+enter multiline, ↑/↓ history, ctrl+r search"></textarea></div>` +
    `<div id="console-search" class="console-search"></div>` +
    `</section>`
  );
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
  if (def.charts && def.charts.includes("frf")) {
    parts.push(
      `<section class="frf-section" id="frf-section" hidden>` +
        `<div class="section-head"><h2>differential belt FRF</h2>` +
        `<span class="note" id="frf-meta"></span></div>` +
        `<div class="charts" id="frf-charts"></div>` +
        `<div id="frf-modes"></div>` +
        `</section>`
    );
  }
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
    `<label class="live-window">window ` +
    `<input type="range" id="live-window" min="2" max="30" step="1" value="${state.live.windowS}">` +
    `<span id="live-window-value">${state.live.windowS} s</span></label>` +
    `<span class="note" id="live-status">connecting to the telemetry tap…</span></div>` +
    `<div class="charts" id="live-charts">` +
    `<p class="note">streams straight from the drives the moment the tap answers — ` +
    `no capture, no file</p>` +
    `</div>` +
    `</section>` +
    `<section class="live-section">` +
    `<div class="section-head"><h2>live actual torque — per motor</h2></div>` +
    `<div class="charts" id="live-torque-charts"></div>` +
    `</section>` +
    `</main>` +
    `<aside class="controls">` +
    `<section class="sweep">` +
    `<div class="section-head"><h2>record to file</h2>` +
    `<span class="note" id="live-file-status"></span></div>` +
    `<div class="row"><input type="text" id="live-start-command" ` +
    `value="SERVO_CAPTURE_START NAME=live AXIS=X">` +
    `<button id="live-start-btn">record</button>` +
    `<button id="live-stop-btn">stop</button></div>` +
    `<p class="note">viewing needs no recording. record when you want an ` +
    `analyzable .scap in the captures root; stop finalizes it.</p>` +
    `</section>` +
    consoleSectionHtml({}) +
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
      consoleSectionHtml({}) +
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
  bindConsole();
  const applyBtn = el("drive-apply-btn");
  if (applyBtn) applyBtn.addEventListener("click", applyDriveChanges);
  const def = currentPageDef();
  document.querySelectorAll("button.template-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      const t = def.templates[Number(btn.dataset.template)];
      if (t) {
        setConsoleValue(t.command, true);
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
    tr.addEventListener("click", (ev) => {
      if (!run.has_results) return;
      if (ev.shiftKey) {
        if (state.selected.has(run.name)) state.selected.delete(run.name);
        else state.selected.add(run.name);
      } else if (state.selected.has(run.name) && state.selected.size === 1) {
        state.selected.delete(run.name);
      } else {
        state.selected = new Set([run.name]);
      }
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
    prefillBtn.textContent = "→ console";
    prefillBtn.title = "prefill the console with this run's command";
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
  if (!state.console.text) loadRerunForm(withResults[0].name);
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

/// Renders at the device pixel ratio so lines stay vector-crisp on hidpi
/// displays: the backing store is sized to the CSS box × dpr and the
/// context scaled back, while all layout math stays in CSS pixels.
function drawChart(canvas, traces, yLabel, fixedY) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || canvas.width;
  const h = canvas.clientHeight || canvas.height;
  const backingW = Math.round(w * dpr);
  const backingH = Math.round(h * dpr);
  if (canvas.width !== backingW || canvas.height !== backingH) {
    canvas.width = backingW;
    canvas.height = backingH;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
  const pad = { l: 46, r: 8, t: 8, b: 22 };
  let tMin = Infinity, tMax = -Infinity, yMin = Infinity, yMax = -Infinity;
  for (const tr of traces) {
    for (let i = 0; i < tr.t.length; i++) {
      if (tr.y[i] === null) continue;
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
    let penDown = false;
    for (let i = 0; i < tr.t.length; i++) {
      if (tr.y[i] === null) {
        penDown = false;
        continue;
      }
      const px = x(tr.t[i]);
      const py = y(tr.y[i]);
      if (penDown) ctx.lineTo(px, py);
      else ctx.moveTo(px, py);
      penDown = true;
    }
    ctx.stroke();
  }
  ctx.fillStyle = "#8a97a3";
  ctx.fillText(yLabel, pad.l, 10);
}

function drawTimeDomain(names, plots, steps) {
  const container = el("charts");
  if (!container) return;
  container.innerHTML = "";
  if (names.length === 0) {
    container.innerHTML = '<p class="note">select runs above</p>';
    return;
  }
  for (const stepName of steps) {
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

/// The peak list needs one step to harvest from: the newest selected run's
/// recommended step when it is visible, else its last visible step.
function peakStep(names, plots, steps) {
  const newest = newestSelectedRunName(names);
  const plot = plots[names.indexOf(newest)];
  const present = plot
    ? steps.filter((s) => plot.steps.some((x) => x.name === s))
    : [];
  const detail = state.details.get(newest);
  const recommended = detail && detail.results && detail.results.verdict.recommended_step;
  const step =
    recommended && present.includes(recommended)
      ? recommended
      : present.length
        ? present[present.length - 1]
        : null;
  return { newest, step };
}

function mixColor(hex, targetHex, t) {
  const c = parseInt(hex.slice(1), 16);
  const g = parseInt(targetHex.slice(1), 16);
  const mix = (shift) => {
    const a = (c >> shift) & 0xff;
    const b = (g >> shift) & 0xff;
    return Math.round(a + (b - a) * t);
  };
  return `#${((mix(16) << 16) | (mix(8) << 8) | mix(0)).toString(16).padStart(6, "0")}`;
}

/// One run selected: each step gets its own palette color. Several runs:
/// each run keeps its table-swatch hue and its steps ramp toward white, so
/// runs stay distinguishable and the step chips are the clutter valve.
function traceStyle(names, steps, runIdx, stepIdx) {
  if (names.length === 1) {
    return { color: PALETTE[stepIdx % PALETTE.length], name: steps[stepIdx] };
  }
  const base = PALETTE[runIdx % PALETTE.length];
  const ramp = steps.length > 1 ? (0.55 * stepIdx) / (steps.length - 1) : 0;
  const name =
    steps.length === 1 ? names[runIdx] : `${names[runIdx]} · ${steps[stepIdx]}`;
  return { color: mixColor(base, "#ffffff", ramp), name };
}

/// The interesting servo/mechanical modes all live below 500 Hz; drawing the
/// full Nyquist span squished them into the left quarter of the chart.
function clipToPsdBand(freq, y) {
  let end = freq.length;
  while (end > 0 && freq[end - 1] > PSD_MAX_FREQ_HZ) end--;
  return { freq: freq.slice(0, end), y: y.slice(0, end) };
}

/// Welch PSD -> single-sided tone amplitude: a sinusoid of amplitude A puts
/// A²/2 of power into its bin's equivalent noise bandwidth, so
/// A = sqrt(2 · psd · ENBW) with ENBW = 1.5·Δf for the analyzer's Hann window.
const WELCH_HANN_ENBW_BINS = 1.5;

function psdToAmplitude(freq, psd) {
  if (freq.length < 2) throw new Error("psd grid too short for a bin width");
  const factor = Math.sqrt(2 * WELCH_HANN_ENBW_BINS * (freq[1] - freq[0]));
  return psd.map((p) => Math.sqrt(p) * factor);
}

function countsPerMm(runName, driveName) {
  const detail = state.details.get(runName);
  const motors = detail && detail.manifest ? detail.manifest.motors : [];
  const motor = motors.find((m) => m.name === driveName);
  if (!motor || !motor.counts_per_mm) {
    throw new Error(`${runName}: manifest has no counts_per_mm for ${driveName}`);
  }
  return motor.counts_per_mm;
}

function psdFerrTraces(names, plots, steps) {
  const traces = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName, j) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step || !step.psd) return;
      const driveNames = Object.keys(step.psd.per_drive);
      if (!driveNames.length) return;
      const style = traceStyle(names, steps, i, j);
      const clipped = clipToPsdBand(step.psd.freq_hz, step.psd.per_drive[driveNames[0]]);
      const umPerCount = 1000 / countsPerMm(names[i], driveNames[0]);
      traces.push({
        freq: clipped.freq,
        y: psdToAmplitude(clipped.freq, clipped.y).map((a) => a * umPerCount),
        color: style.color,
        dashed: false,
        label: `${style.name} (${driveNames[0]})`,
        run: names[i],
      });
    });
  });
  return traces;
}

function psdAccelTraces(names, plots, steps) {
  const traces = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName, j) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step || !step.psd || !step.psd.accel) return;
      const style = traceStyle(names, steps, i, j);
      const clipped = clipToPsdBand(step.psd.accel.freq_hz, step.psd.accel.psd);
      traces.push({
        freq: clipped.freq,
        y: psdToAmplitude(clipped.freq, clipped.y),
        color: style.color,
        dashed: false,
        label: `${style.name} (accel)`,
        run: names[i],
      });
    });
  });
  return traces;
}

function fmtLinear(v) {
  if (v === 0) return "0";
  const a = Math.abs(v);
  return a >= 1000 || a < 0.01 ? v.toExponential(1) : v.toPrecision(3);
}

function drawPsdChart(canvas, traces, band, yTitle, hover, opts) {
  opts = opts || {};
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || canvas.width;
  const h = canvas.clientHeight || canvas.height;
  const backingW = Math.round(w * dpr);
  const backingH = Math.round(h * dpr);
  if (canvas.width !== backingW || canvas.height !== backingH) {
    canvas.width = backingW;
    canvas.height = backingH;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
  const pad = { l: 46, r: 8, t: 8, b: 22 };
  const EPS = 1e-6;
  const toV = (raw) => (opts.linear ? raw : Math.log10(Math.max(raw, EPS)));
  let fMin = Infinity, fMax = -Infinity, vMin = Infinity, vMax = -Infinity;
  for (const tr of traces) {
    for (let i = 0; i < tr.freq.length; i++) {
      const f = tr.freq[i];
      const v = toV(tr.y[i]);
      fMin = Math.min(fMin, f);
      fMax = Math.max(fMax, f);
      vMin = Math.min(vMin, v);
      vMax = Math.max(vMax, v);
    }
  }
  if (opts.zeroFloor) vMin = 0;
  if (opts.fixedY) {
    vMin = opts.fixedY.yMin;
    vMax = opts.fixedY.yMax;
  }
  if (!isFinite(fMin) || !isFinite(vMin)) return;
  if (vMin === vMax) { vMin -= 1; vMax += 1; }
  const x = (f) => pad.l + ((f - fMin) / (fMax - fMin || 1)) * (w - pad.l - pad.r);
  const yOfV = (v) => h - pad.b - ((v - vMin) / (vMax - vMin || 1)) * (h - pad.t - pad.b);
  const y = (raw) => yOfV(toV(raw));

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
    const v = vMin + ((vMax - vMin) * i) / 4;
    const py = yOfV(v);
    ctx.moveTo(pad.l, py);
    ctx.lineTo(w - pad.r, py);
    ctx.fillText(opts.linear ? fmtLinear(v) : `1e${v.toFixed(1)}`, 2, py + 3);
  }
  for (let i = 0; i <= 4; i++) {
    const f = fMin + ((fMax - fMin) * i) / 4;
    const px = x(f);
    ctx.fillText(f.toFixed(0) + "Hz", px, h - 6);
  }
  ctx.stroke();

  if (opts.threshold != null) {
    ctx.strokeStyle = "#d9a441";
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    const py = y(opts.threshold);
    ctx.moveTo(pad.l, py);
    ctx.lineTo(w - pad.r, py);
    ctx.stroke();
    ctx.setLineDash([]);
  }

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

  if (opts.markers) {
    ctx.setLineDash([3, 3]);
    opts.markers.forEach((m, idx) => {
      if (m.freq < fMin || m.freq > fMax) return;
      const px = x(m.freq);
      ctx.strokeStyle = "#b388ff";
      ctx.beginPath();
      ctx.moveTo(px, pad.t);
      ctx.lineTo(px, h - pad.b);
      ctx.stroke();
      ctx.fillStyle = "#b388ff";
      ctx.fillText(m.label, px + 4, pad.t + 12 + (idx % 3) * 10);
    });
    ctx.setLineDash([]);
  }

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
  ctx.fillText(yTitle, pad.l, 10);

  if (hover) {
    let best = null;
    for (const tr of traces) {
      for (let i = 0; i < tr.freq.length; i++) {
        const dx = x(tr.freq[i]) - hover.mx;
        const dy = y(tr.y[i]) - hover.my;
        const d = dx * dx + dy * dy;
        if (!best || d < best.d) best = { d, tr, i };
      }
    }
    if (best) {
      const px = x(best.tr.freq[best.i]);
      const py = y(best.tr.y[best.i]);
      ctx.strokeStyle = "#4a5560";
      ctx.beginPath();
      ctx.moveTo(px, pad.t);
      ctx.lineTo(px, h - pad.b);
      ctx.stroke();
      ctx.fillStyle = best.tr.color;
      ctx.beginPath();
      ctx.arc(px, py, 3, 0, Math.PI * 2);
      ctx.fill();
      const value = opts.linear
        ? fmtLinear(best.tr.y[best.i])
        : best.tr.y[best.i].toExponential(2);
      const text = `${best.tr.freq[best.i].toFixed(1)} Hz  ${value}  ${best.tr.label}`;
      const tw = ctx.measureText(text).width;
      const tx = Math.min(Math.max(px + 8, pad.l), w - pad.r - tw - 8);
      const ty = Math.max(py - 10, pad.t + 12);
      ctx.fillStyle = "#0d1117";
      ctx.fillRect(tx - 4, ty - 10, tw + 8, 14);
      ctx.strokeStyle = "#4a5560";
      ctx.strokeRect(tx - 4, ty - 10, tw + 8, 14);
      ctx.fillStyle = "#e6edf3";
      ctx.fillText(text, tx, ty);
    }
  }
}

/// Redraws with the cursor position on every move — the readout follows
/// the nearest sample, so exact peak frequencies are readable instead of
/// eyeballed off the axis.
function attachPsdHover(canvas, traces, band, yTitle, opts) {
  canvas.addEventListener("mousemove", (e) => {
    drawPsdChart(canvas, traces, band, yTitle, { mx: e.offsetX, my: e.offsetY }, opts);
  });
  canvas.addEventListener("mouseleave", () => drawPsdChart(canvas, traces, band, yTitle, null, opts));
}

function psdBox(title, traces, band, yTitle, opts) {
  const box = document.createElement("div");
  box.className = "chart-box";
  const head = document.createElement("h3");
  head.textContent = title;
  box.appendChild(head);
  const canvas = document.createElement("canvas");
  canvas.width = 860;
  canvas.height = 280;
  box.appendChild(canvas);
  drawPsdChart(canvas, traces, band, yTitle, null, opts);
  attachPsdHover(canvas, traces, band, yTitle, opts);
  const legend = document.createElement("div");
  legend.className = "legend";
  traces.forEach((tr) => {
    const item = document.createElement("span");
    item.innerHTML = `<span class="swatch" style="background:${tr.color}"></span>${tr.label}`;
    legend.appendChild(item);
  });
  box.appendChild(legend);
  return box;
}

function renderPsdChart(names, plots, steps) {
  const container = el("psd-charts");
  if (!container) return;
  container.innerHTML = "";
  if (!names.length || !steps.length) {
    container.innerHTML = '<p class="note">select runs above</p>';
    return;
  }
  const psdOpts = { linear: true, zeroFloor: true };
  const ferr = psdFerrTraces(names, plots, steps);
  container.appendChild(
    psdBox("following error", ferr, RESONANCE_BAND_HZ, "ferr amplitude (µm)", psdOpts)
  );
  const accel = psdAccelTraces(names, plots, steps);
  if (accel.length) {
    container.appendChild(
      psdBox("accelerometer", accel, RESONANCE_BAND_HZ, "accel amplitude", psdOpts)
    );
  }
}

function visibleStepNames(stepNames) {
  if (!state.stepFilter) return stepNames;
  const kept = stepNames.filter((s) => state.stepFilter.has(s));
  return kept.length ? kept : stepNames;
}

function renderPsdChips(stepNames) {
  const container = el("psd-step-chips");
  if (!container) return;
  container.innerHTML = "";
  const all = document.createElement("button");
  all.className = "chip" + (state.stepFilter ? "" : " active");
  all.textContent = "all";
  all.title = "show every step";
  all.addEventListener("click", () => {
    state.stepFilter = null;
    redrawCharts();
  });
  container.appendChild(all);
  for (const stepName of stepNames) {
    const chip = document.createElement("button");
    const inFilter = state.stepFilter && state.stepFilter.has(stepName);
    chip.className = "chip" + (inFilter ? " active" : "");
    chip.textContent = stepName;
    chip.title = "click: only this step — shift+click: add/remove it";
    chip.addEventListener("click", (ev) => {
      if (ev.shiftKey) {
        const next = new Set(state.stepFilter || stepNames);
        if (next.has(stepName)) next.delete(stepName);
        else next.add(stepName);
        state.stepFilter = next.size === 0 || next.size === stepNames.length ? null : next;
      } else if (inFilter && state.stepFilter.size === 1) {
        state.stepFilter = null;
      } else {
        state.stepFilter = new Set([stepName]);
      }
      redrawCharts();
    });
    container.appendChild(chip);
  }
}

// --- differential belt FRF (dynamics page) -----------------------------------

const FRF_BOXES = [
  { key: "mag_db", title: "magnitude", yTitle: "|H| (dB)" },
  { key: "phase_deg", title: "phase", yTitle: "phase (deg)" },
  { key: "coherence", title: "coherence", yTitle: "coherence" },
  { key: "torque_db", title: "torque FRF", yTitle: "torque (dB)" },
];

function differentialSeries(step) {
  const d = step.differential;
  if (!d) return null;
  const n = d.freq_hz.length;
  for (const spec of FRF_BOXES) {
    const arr = d[spec.key];
    if (!Array.isArray(arr) || arr.length !== n) {
      throw new Error(
        `${step.name}: differential.${spec.key} length ${Array.isArray(arr) ? arr.length : "missing"} != freq_hz length ${n}`
      );
    }
  }
  return d;
}

function frfTraces(names, plots, stepName, key) {
  const traces = [];
  plots.forEach((p, i) => {
    const step = p.steps.find((s) => s.name === stepName);
    const d = step && differentialSeries(step);
    if (!d) return;
    traces.push({
      freq: d.freq_hz,
      y: d[key],
      color: PALETTE[i % PALETTE.length],
      dashed: false,
      label: names[i],
      run: names[i],
    });
  });
  return traces;
}

function frfModeMarkers(modes) {
  return modes.map((m) => ({
    freq: m.freq_hz,
    label: `${m.freq_hz.toFixed(1)} Hz${m.damping == null ? "" : ` ζ=${m.damping.toFixed(3)}`}`,
  }));
}

function frfModeTableHtml(modes) {
  if (!modes.length) return '<p class="note">no modes detected</p>';
  const rows = modes
    .map(
      (m) =>
        `<tr><td>${m.freq_hz.toFixed(1)} Hz</td><td>${m.gain_db.toFixed(1)} dB</td>` +
        `<td>${m.damping == null ? "—" : m.damping.toFixed(3)}</td><td>${m.coherence.toFixed(2)}</td></tr>`
    )
    .join("");
  return (
    `<table class="mode-table"><thead><tr>` +
    `<th>freq</th><th>|H|</th><th>damping</th><th>coherence</th>` +
    `</tr></thead><tbody>${rows}</tbody></table>`
  );
}

function differentialResultStep(runName, stepName) {
  const detail = state.details.get(runName);
  const step =
    detail && detail.results && detail.results.steps.find((s) => s.name === stepName);
  return (step && step.differential) || null;
}

/// The newest selected run with a differential step drives the mode markers,
/// the coherence threshold, and the mode table; every selected run's traces
/// overlay on the four shared-x boxes.
function renderFrfCharts(names, plots) {
  const section = el("frf-section");
  if (!section) return;
  const container = el("frf-charts");
  const modesEl = el("frf-modes");
  const meta = el("frf-meta");
  container.innerHTML = "";
  modesEl.innerHTML = "";
  meta.textContent = "";
  const stepNames = [
    ...new Set(plots.flatMap((p) => p.steps.filter((s) => s.differential).map((s) => s.name))),
  ];
  if (!stepNames.length) {
    section.hidden = true;
    return;
  }
  section.hidden = false;
  const metaParts = [];
  for (const stepName of stepNames) {
    let ref = null;
    let refName = null;
    for (let i = 0; i < plots.length; i++) {
      const step = plots[i].steps.find((s) => s.name === stepName);
      const d = step && differentialSeries(step);
      if (d) {
        ref = d;
        refName = names[i];
        break;
      }
    }
    for (const spec of FRF_BOXES) {
      const opts = { linear: true };
      if (spec.key === "mag_db") opts.markers = frfModeMarkers(ref.modes);
      if (spec.key === "coherence") {
        opts.fixedY = { yMin: 0, yMax: 1.05 };
        opts.threshold = ref.coherence_min;
      }
      container.appendChild(
        psdBox(
          `${stepName} — ${spec.title}`,
          frfTraces(names, plots, stepName, spec.key),
          null,
          spec.yTitle,
          opts
        )
      );
    }
    const result = differentialResultStep(refName, stepName);
    const label = result
      ? `${result.pair.join(" vs ")} — ${result.segments} Welch segments`
      : refName;
    modesEl.innerHTML += `<h3>${stepName} modes — ${label}</h3>${frfModeTableHtml(ref.modes)}`;
    metaParts.push(label);
  }
  meta.textContent = metaParts.join(" · ");
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

function renderPeakList(names, plots, steps) {
  const container = el("peak-list");
  if (!container) return;
  const runLabel = el("peaks-run");
  if (!names.length || !steps.length) {
    container.innerHTML = '<p class="note">select runs above</p>';
    if (runLabel) runLabel.textContent = "";
    return;
  }
  const picked = peakStep(names, plots, steps);
  const plot = plots[names.indexOf(picked.newest)];
  const step = plot && picked.step && plot.steps.find((s) => s.name === picked.step);
  if (!step || !step.psd) {
    container.innerHTML = '<p class="note">no PSD for this step</p>';
    if (runLabel) runLabel.textContent = "";
    return;
  }
  if (runLabel) runLabel.textContent = `${picked.newest} / ${picked.step}`;
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
  if (state.stepFilter && !stepNames.some((s) => state.stepFilter.has(s))) {
    state.stepFilter = null;
  }
  const steps = visibleStepNames(stepNames);
  if (def.charts && def.charts.includes("frf")) renderFrfCharts(okNames, plots);
  if (def.charts && def.charts.includes("psd")) {
    renderPsdChips(stepNames);
    renderPsdChart(okNames, plots, steps);
  }
  if (def.peaks) renderPeakList(okNames, plots, steps);
  if (def.charts && def.charts.includes("time")) drawTimeDomain(okNames, plots, steps);
}

// --- live tap ------------------------------------------------------------------
//
// Streams from GET /api/live_tap the moment the page opens — the server
// relays the ethercat-rt telemetry tap, so viewing needs no capture, no
// file, and no G-code. The cursor handshake mirrors the run-file tail:
// the first poll attaches "now" and returns only a cursor, every later
// poll sends it back and gets just the new samples. A cycle_index jump
// (drops under backpressure, tap reconnect) becomes a null break in the
// series — the chart shows a gap, never stale data drawn as live. Each
// motor gets its own stacked chart over the slider's window, all on a
// shared y-scale so the noisy motor stands out.

function bindLiveEvents() {
  el("live-start-btn").addEventListener("click", () => {
    const line = el("live-start-command").value.trim();
    if (line) runGcode([line], "live");
  });
  el("live-stop-btn").addEventListener("click", () => runGcode(["SERVO_CAPTURE_STOP"], "live"));
  const slider = el("live-window");
  slider.addEventListener("input", () => {
    state.live.windowS = Number(slider.value);
    el("live-window-value").textContent = `${state.live.windowS} s`;
    trimLiveWindow();
    drawLiveCharts();
  });
}

async function pollLiveFileStatus() {
  const label = el("live-file-status");
  if (!label) return;
  let status;
  try {
    status = await api("/api/live");
  } catch (e) {
    label.textContent = String(e);
    return;
  }
  if (!status.capture) {
    label.textContent = "nothing recorded yet";
    return;
  }
  const cap = status.capture;
  const growing = cap.age_s !== null && cap.age_s < 3;
  label.textContent = growing
    ? `recording ${cap.name} — ${(cap.size_bytes / 1024).toFixed(0)} KiB`
    : `last: ${cap.name} (${formatAge(cap.age_s)} ago)`;
}

async function pollLiveTap() {
  if (state.live.polling) return;
  state.live.polling = true;
  try {
    const query = state.live.cursor === null ? "" : `?since_cycle=${state.live.cursor}`;
    const payload = await api(`/api/live_tap${query}`);
    const label = el("live-status");
    if (payload.status !== "streaming") {
      if (label) {
        label.textContent =
          payload.status === "unreachable"
            ? `telemetry tap unreachable — ${payload.reason}`
            : "connecting to the telemetry tap…";
      }
      return;
    }
    if (label) label.textContent = `streaming at ${(payload.fs_hz / 1000).toFixed(1)} kHz`;
    appendTapSamples(payload);
    drawLiveCharts();
  } catch (e) {
    const label = el("live-status");
    if (label) label.textContent = String(e);
  } finally {
    state.live.polling = false;
  }
}

function appendTapSamples(payload) {
  state.live.cursor = payload.next_cycle;
  state.live.fsHz = payload.fs_hz;
  const drives = Object.keys(payload.drives || {});
  const n = drives.length ? payload.drives[drives[0]].ferr.length : 0;
  if (!n) return;
  if (state.live.cycle0 === null) state.live.cycle0 = payload.first_cycle;
  for (const drive of drives) {
    if (!state.live.perDrive[drive]) {
      state.live.perDrive[drive] = {
        ferr: new Array(state.live.t.length).fill(null),
        torque: new Array(state.live.t.length).fill(null),
      };
    }
  }
  const stride = payload.stride;
  const gapThreshold = stride * 3;
  for (let i = 0; i < n; i++) {
    const cycle = payload.first_cycle + i * stride;
    if (state.live.lastCycle !== null && cycle - state.live.lastCycle > gapThreshold) {
      state.live.t.push((state.live.lastCycle + stride - state.live.cycle0) / payload.fs_hz);
      for (const drive of drives) {
        state.live.perDrive[drive].ferr.push(null);
        state.live.perDrive[drive].torque.push(null);
      }
    }
    state.live.t.push((cycle - state.live.cycle0) / payload.fs_hz);
    for (const drive of drives) {
      state.live.perDrive[drive].ferr.push(payload.drives[drive].ferr[i]);
      state.live.perDrive[drive].torque.push(payload.drives[drive].torque[i] / 10);
    }
    state.live.lastCycle = cycle;
  }
  trimLiveWindow();
}

function trimLiveWindow() {
  if (!state.live.t.length) return;
  const cutoff = state.live.t[state.live.t.length - 1] - state.live.windowS;
  let drop = 0;
  while (drop < state.live.t.length && state.live.t[drop] < cutoff) drop++;
  if (drop > 0) {
    state.live.t.splice(0, drop);
    for (const series of Object.values(state.live.perDrive)) {
      series.ferr.splice(0, drop);
      series.torque.splice(0, drop);
    }
  }
}

/// slot0..slotN are the tap's honest names (the RT process never sees
/// klippy's motor names); drive_state.json's slots map recovers the
/// motor name when a dump has run.
function liveDriveLabel(tapName) {
  const slots = state.drive.data && state.drive.data.slots;
  if (!slots) return tapName;
  const match = /^slot(\d+)$/.exec(tapName);
  if (!match) return tapName;
  const slot = Number(match[1]);
  for (const [motor, s] of Object.entries(slots)) {
    if (s === slot) return motor;
  }
  return tapName;
}

function ensureLiveChartBoxes(containerId, idPrefix, drives) {
  const container = el(containerId);
  if (!container) return false;
  const have = [...container.querySelectorAll("canvas")].map((c) => c.id).join();
  const want = drives.map((d) => `${idPrefix}-canvas-${d}`).join();
  if (have !== want) {
    container.innerHTML = drives
      .map(
        (d, i) =>
          `<div class="chart-box">` +
          `<h3><span class="swatch" style="background:${PALETTE[i % PALETTE.length]}"></span>` +
          `<span id="${idPrefix}-name-${d}">${liveDriveLabel(d)}</span> ` +
          `<span class="note" id="${idPrefix}-peak-${d}"></span></h3>` +
          `<canvas id="${idPrefix}-canvas-${d}" width="860" height="130"></canvas>` +
          `</div>`
      )
      .join("");
  }
  return true;
}

function drawLiveChartGroup(containerId, idPrefix, drives, channel, yLabel, peakFmt) {
  if (!ensureLiveChartBoxes(containerId, idPrefix, drives)) return;
  let yMin = Infinity;
  let yMax = -Infinity;
  const peaks = {};
  for (const d of drives) {
    let peak = 0;
    for (const v of state.live.perDrive[d][channel]) {
      if (v === null) continue;
      if (v < yMin) yMin = v;
      if (v > yMax) yMax = v;
      const mag = Math.abs(v);
      if (mag > peak) peak = mag;
    }
    peaks[d] = peak;
  }
  if (!isFinite(yMin)) return;
  drives.forEach((d, i) => {
    const canvas = el(`${idPrefix}-canvas-${d}`);
    if (!canvas) return;
    drawChart(
      canvas,
      [
        {
          t: state.live.t,
          y: state.live.perDrive[d][channel],
          color: PALETTE[i % PALETTE.length],
        },
      ],
      yLabel,
      { yMin, yMax }
    );
    const name = el(`${idPrefix}-name-${d}`);
    if (name) name.textContent = liveDriveLabel(d);
    const label = el(`${idPrefix}-peak-${d}`);
    if (label) label.textContent = peakFmt(peaks[d]);
  });
}

function drawLiveCharts() {
  if (!state.live.t.length) return;
  const drives = Object.keys(state.live.perDrive).sort();
  if (!drives.length) return;
  drawLiveChartGroup(
    "live-charts",
    "live",
    drives,
    "ferr",
    "ferr (counts)",
    (p) => `peak |ferr| ${p}`
  );
  drawLiveChartGroup(
    "live-torque-charts",
    "live-torque",
    drives,
    "torque",
    "torque (% rated)",
    (p) => `peak |torque| ${p.toFixed(1)}%`
  );
}

function startLivePolling() {
  state.live.cursor = null;
  state.live.cycle0 = null;
  state.live.lastCycle = null;
  state.live.t = [];
  state.live.perDrive = {};
  pollLiveFileStatus();
  pollLiveTap();
  state.live.timers = [
    setInterval(pollLiveFileStatus, LIVE_STATUS_POLL_MS),
    setInterval(pollLiveTap, LIVE_TAIL_POLL_MS),
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
// shows only its own param groups. Every cell shows and takes the RAW
// register value exactly as stored on the drive — the unit label names the
// LSB (e.g. "0.1 Hz") instead of the UI converting, so what you type is
// what SERVO_TUNE writes. Pure helpers first — autofill derivation,
// changed-cell diffing, SERVO_TUNE line building (always with an explicit
// MOTORS= list) — the logic a Rust test asserts is present and exercisable
// without a browser; DOM rendering and event wiring follow.

const GROUP_ORDER = ["gains", "filters", "notch", "speed_observer", "disturbance_observer", "load"];
const OTHER_GROUP = "other";
const AUTOFILL_SOURCE_PARAM = "speed_gain";
const DRIVE_REFRESH_POLL_MS = 1000;
const DRIVE_REFRESH_TIMEOUT_MS = 15000;

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
  return `<input type="number" step="1" class="${cls.join(" ")}" data-param="${param.name}" data-motor="${motor}" value="${raw}" title="${titleText}">`;
}

function allInputHtml(param) {
  const motors = motorNames(state.drive.data.motors);
  const values = motors.map((m) => cellRaw(param, m));
  const agree = valuesAgree(values);
  const cls = ["cell-input", "all"];
  if (motors.some((m) => cellRaw(param, m) !== state.drive.data.motors[m][param.c_code])) {
    cls.push("pending");
  }
  const title = agree
    ? "set all motors"
    : `set all motors — currently ${motors.map((m, i) => `${shortMotorLabel(m)}=${values[i]}`).join(" ")}`;
  if (param.options) {
    const opts =
      `<option value=""${agree ? "" : " selected"} disabled>${agree ? "" : "mixed"}</option>` +
      Object.entries(param.options)
        .map(
          ([v, label]) =>
            `<option value="${v}"${agree && Number(v) === values[0] ? " selected" : ""}>${v}: ${label}</option>`
        )
        .join("");
    return `<select class="${cls.join(" ")}" data-param="${param.name}" data-motor="*" title="${title}">${opts}</select>`;
  }
  const display = agree ? values[0] : "";
  return `<input type="number" step="1" class="${cls.join(" ")}" data-param="${param.name}" data-motor="*" value="${display}" placeholder="${agree ? "" : "mixed"}" title="${title}">`;
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

/// Adaptive-notch recipe (A6-EC manual 7.10): reset the notch parameters,
/// hand notches 1-2 to the drive, or take them back (0 keeps whatever the
/// drive last wrote). Each button only STAGES adaptive_notch_mode for all
/// motors — the write happens through the apply button like any grid edit.
const NOTCH_QUICK_ACTIONS = [
  { label: "reset notch params", value: 3 },
  { label: "1 adaptive", value: 1 },
  { label: "2 adaptive", value: 2 },
  { label: "disable adaptive", value: 0 },
];

function notchQuickActionsHtml() {
  return (
    `<details class="adaptive-actions"${state.drive.adaptiveOpen ? " open" : ""}>` +
    "<summary>adaptive notch recipes</summary>" +
    '<div class="quick-actions">' +
    NOTCH_QUICK_ACTIONS.map(
      (a) =>
        `<button class="quick-action" data-value="${a.value}" title="stages adaptive_notch_mode=${a.value} for all motors — nothing is written until apply">${a.label}</button>`
    ).join("") +
    '</div><p class="hint">stages adaptive_notch_mode — review in the pending list, then apply</p>' +
    "</details>"
  );
}

const NOTCH_ROW_KINDS = ["freq", "width", "depth"];

function notchMatrix(params) {
  const byKey = new Map();
  const nums = new Set();
  const leftover = [];
  for (const p of params) {
    const m = /^notch_(\d+)_(freq|width|depth)$/.exec(p.name);
    if (m) {
      nums.add(Number(m[1]));
      byKey.set(`${m[1]}:${m[2]}`, p);
    } else {
      leftover.push(p);
    }
  }
  return { nums: [...nums].sort((a, b) => a - b), byKey, leftover };
}

/// The compact notch view: one column per notch, freq/width/depth rows, one
/// input per cell that stages the value for every motor (notches are
/// per-axis physics — on corexy every motor sees the same belt, so
/// per-motor notch tables are noise; the per-motor toggle remains for
/// drives that genuinely disagree).
function notchCompactHtml(params) {
  const { nums, byKey, leftover } = notchMatrix(params);
  const head =
    `<th class="param-col"></th>` + nums.map((n) => `<th>notch ${n}</th>`).join("");
  const rows = NOTCH_ROW_KINDS.map((kind) => {
    const first = byKey.get(`${nums[0]}:${kind}`);
    const unit = first && first.unit ? ` <span class="unit">${first.unit}</span>` : "";
    const cells = nums
      .map((n) => {
        const p = byKey.get(`${n}:${kind}`);
        return `<td>${p ? allInputHtml(p) : ""}</td>`;
      })
      .join("");
    return `<tr><td class="param-col">${kind}${unit}</td>${cells}</tr>`;
  }).join("");
  return {
    table: `<table class="param-grid notch-grid"><thead><tr>${head}</tr></thead><tbody>${rows}</tbody></table>`,
    leftover,
  };
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
  const perMotorRows = (params, group) =>
    params
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
  const perMotorTable = (params, group) =>
    `<table class="param-grid"><thead><tr>${headerCells}</tr></thead>` +
    `<tbody>${perMotorRows(params, group)}</tbody></table>`;
  const parts = [];
  for (const [group, params] of sections) {
    if (!params.length) continue;
    if (def.groups && group !== OTHER_GROUP && !def.groups.includes(group)) continue;
    if (group === "notch" && !state.drive.notchPerMotor) {
      const compact = notchCompactHtml(params);
      parts.push(
        `<div class="param-group"><h3>notch ` +
          `<a href="#" class="notch-view-toggle hint">per-motor view</a></h3>` +
          compact.table +
          (compact.leftover.length ? perMotorTable(compact.leftover, group) : "") +
          notchQuickActionsHtml() +
          `</div>`
      );
      continue;
    }
    const toggle =
      group === "notch"
        ? ` <a href="#" class="notch-view-toggle hint">compact view</a>`
        : "";
    const extras = group === "notch" ? notchQuickActionsHtml() : "";
    parts.push(
      `<div class="param-group"><h3>${group.replace(/_/g, " ")}${toggle}</h3>` +
        perMotorTable(params, group) +
        extras +
        `</div>`
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
  container.querySelectorAll("details.adaptive-actions").forEach((d) => {
    d.addEventListener("toggle", () => {
      state.drive.adaptiveOpen = d.open;
    });
  });
  container.querySelectorAll("a.notch-view-toggle").forEach((elink) => {
    elink.addEventListener("click", (e) => {
      e.preventDefault();
      state.drive.notchPerMotor = !state.drive.notchPerMotor;
      renderDriveGroups();
    });
  });
  container.querySelectorAll("button.quick-action").forEach((btn) => {
    btn.addEventListener("click", () => {
      const staged = { ...(state.drive.pending.adaptive_notch_mode || {}) };
      for (const m of motorNames(state.drive.data.motors)) {
        staged[m] = Number(btn.dataset.value);
      }
      state.drive.pending.adaptive_notch_mode = staged;
      renderDriveGroups();
    });
  });
}

function onDriveCellChange(e) {
  const input = e.target;
  const name = input.dataset.param;
  const param = paramByName(name);
  const raw = parseInt(input.value, 10);
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

/// The stroke's SPEED/ACCEL must ride along on a re-run: they shape the
/// excitation, so a "same sweep" at the command defaults is not the same
/// sweep and its results are not comparable to the original.
function strokeSuffix(manifest, includeAccel) {
  const plan = manifest.stroke_plan || {};
  let suffix = "";
  if (plan.speed != null) suffix += ` SPEED=${plan.speed}`;
  if (includeAccel && plan.accel != null) suffix += ` ACCEL=${plan.accel}`;
  return suffix;
}

function reconstructCommand(manifest) {
  const tag = manifest.tag || "cal";
  const axis = manifest.axis || "X";
  const iterations = (manifest.stroke_plan && manifest.stroke_plan.iterations) || 1;
  const sweptKeys = (manifest.steps || []).map((s) => Object.keys(s.swept || {}));
  const commonKeys = sweptKeys.reduce((a, b) => a.filter((k) => b.includes(k)), sweptKeys[0] || []);
  const common = `AXIS=${axis} ITERATIONS=${iterations} TAG=${tag}`;

  switch (manifest.experiment) {
    case "gain_sweep": {
      const values = manifest.steps.map((s) => s.swept.speed).join(",");
      return `SERVO_CALIBRATE_GAINS SPEED_GAINS=${values} ${common}${strokeSuffix(manifest, true)}`;
    }
    case "gain_ladder": {
      const speeds = manifest.steps.map((s) => s.swept.speed);
      const safe = speeds[0];
      const start = speeds.length > 1 ? speeds[1] : safe;
      const step = speeds.length > 2 ? speeds[2] - speeds[1] : 50;
      const max = speeds[speeds.length - 1];
      return `SERVO_GAIN_LADDER SAFE=${safe} START=${start} STEP=${step} MAX=${max} ${common}${strokeSuffix(manifest, true)}`;
    }
    case "refine_sweep": {
      const param = commonKeys.length === 1 ? commonKeys[0] : "speed";
      const values = manifest.steps.map((s) => s.swept[param]).join(",");
      return `SERVO_REFINE_GAIN PARAM=${param} VALUES=${values} ${common}${strokeSuffix(manifest, true)}`;
    }
    case "inertia_sweep": {
      const values = manifest.steps.map((s) => s.swept.ratio ?? Object.values(s.swept)[0]).join(",");
      return `SERVO_SWEEP_INERTIA RATIOS=${values} ${common}${strokeSuffix(manifest, true)}`;
    }
    case "accel_sweep": {
      const values = manifest.steps.map((s) => s.swept.accel ?? Object.values(s.swept)[0]).join(",");
      return `SERVO_SWEEP_ACCEL ACCELS=${values} ${common}${strokeSuffix(manifest, false)}`;
    }
    case "differential": {
      const plan = manifest.stroke_plan;
      return (
        `SERVO_MEASURE_DIFFERENTIAL BELT=${plan.belt} FREQ_START=${plan.freq_start} ` +
        `FREQ_END=${plan.freq_end} AMPLITUDE=${plan.amplitude} DURATION=${plan.duration} ` +
        `RAMP=${plan.ramp} DWELL_MS=${plan.dwell_ms} NAME=${tag}`
      );
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
  setConsoleValue(reconstructCommand(detail.manifest), false);
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

/// One click, no confirmation: an accidental stop costs a FIRMWARE_RESTART,
/// a confirm dialog in a real emergency costs the machine.
async function emergencyStop() {
  const entry = { time: new Date().toISOString(), label: "e-stop", lines: ["emergency_stop"], results: [] };
  try {
    const resp = await fetch(`${moonrakerUrl()}/printer/emergency_stop`, { method: "POST" });
    entry.results.push({ ok: resp.ok, status: resp.status });
  } catch (e) {
    entry.results.push({ ok: false, status: 0 });
  }
  state.sentLog.push(entry);
  renderSentLog();
  pollMoonrakerHealth();
}

function escapeHtml(s) {
  return s
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
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
        return (
          `<div class="sent-line" data-line="${escapeHtml(l)}" ` +
          `title="click to insert into the console">${escapeHtml(l)}${suffix}</div>`
        );
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
  container.onclick = (ev) => {
    const line = ev.target.closest(".sent-line");
    if (line) setConsoleValue(line.dataset.line, true);
  };
  container.scrollTop = container.scrollHeight;
}

/// Sends `lines` (already-built gcode) through the shared Moonraker
/// plumbing — the grid's Apply and the console land in the same session
/// log, which survives page switches.
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

// --- console ------------------------------------------------------------------

/// The catch only forgives corrupt localStorage JSON — anything else (a
/// mistyped key, a TDZ const) must surface, not quietly reset the history.
function loadConsoleHistory() {
  const raw = localStorage.getItem(CONSOLE_HISTORY_KEY) || "[]";
  let parsed;
  try {
    parsed = JSON.parse(raw);
  } catch (e) {
    return [];
  }
  return Array.isArray(parsed) ? parsed.filter((l) => typeof l === "string") : [];
}

function pushConsoleHistory(entry) {
  const hist = state.console.history;
  if (hist[hist.length - 1] !== entry) hist.push(entry);
  if (hist.length > CONSOLE_HISTORY_MAX) hist.splice(0, hist.length - CONSOLE_HISTORY_MAX);
  localStorage.setItem(CONSOLE_HISTORY_KEY, JSON.stringify(hist));
}

function bindConsole() {
  const input = el("console-input");
  if (!input) return;
  input.value = state.console.text;
  autosizeConsole(input);
  input.addEventListener("input", () => {
    state.console.text = input.value;
    autosizeConsole(input);
  });
  input.addEventListener("keydown", consoleKeydown);
  input.addEventListener("blur", () => exitConsoleSearch(true));
}

function autosizeConsole(input) {
  input.style.height = "auto";
  input.style.height = `${input.scrollHeight}px`;
}

function setConsoleValue(text, focus) {
  state.console.text = text;
  const input = el("console-input");
  if (!input) return;
  input.value = text;
  input.selectionStart = input.selectionEnd = text.length;
  autosizeConsole(input);
  if (focus) input.focus();
}

function caretOnFirstLine(input) {
  return input.value.lastIndexOf("\n", input.selectionStart - 1) === -1;
}

function caretOnLastLine(input) {
  return input.value.indexOf("\n", input.selectionEnd) === -1;
}

function consoleKeydown(ev) {
  const input = ev.target;
  const c = state.console;
  if (c.search) {
    consoleSearchKeydown(ev, input);
    return;
  }
  if (ev.key === "Enter" && !ev.shiftKey) {
    ev.preventDefault();
    submitConsole();
    return;
  }
  if (ev.ctrlKey && ev.key === "r") {
    ev.preventDefault();
    c.search = { query: "", pos: c.history.length - 1, saved: input.value, failed: false };
    renderConsoleSearch();
    return;
  }
  const back = (ev.ctrlKey && ev.key === "p") || (ev.key === "ArrowUp" && caretOnFirstLine(input));
  const fwd = (ev.ctrlKey && ev.key === "n") || (ev.key === "ArrowDown" && caretOnLastLine(input));
  if (back || fwd) {
    ev.preventDefault();
    historyStep(back ? -1 : 1);
    return;
  }
  if (ev.ctrlKey && ev.key === "c" && input.selectionStart === input.selectionEnd) {
    ev.preventDefault();
    c.cursor = null;
    setConsoleValue("", true);
  }
}

function historyStep(dir) {
  const c = state.console;
  if (!c.history.length) return;
  if (c.cursor === null) {
    if (dir > 0) return;
    c.draft = c.text;
    c.cursor = c.history.length;
  }
  const next = c.cursor + dir;
  if (next < 0) return;
  if (next >= c.history.length) {
    c.cursor = null;
    setConsoleValue(c.draft, true);
    return;
  }
  c.cursor = next;
  setConsoleValue(c.history[next], true);
}

function consoleSearchKeydown(ev, input) {
  const s = state.console.search;
  if (ev.ctrlKey && ev.key === "r") {
    ev.preventDefault();
    searchHistory(s.pos - 1);
    return;
  }
  if (ev.key === "Escape" || (ev.ctrlKey && ev.key === "g")) {
    ev.preventDefault();
    exitConsoleSearch(false);
    return;
  }
  if (ev.key === "Enter" && !ev.shiftKey) {
    ev.preventDefault();
    exitConsoleSearch(true);
    submitConsole();
    return;
  }
  if (ev.key === "Backspace") {
    ev.preventDefault();
    s.query = s.query.slice(0, -1);
    searchHistory(state.console.history.length - 1);
    return;
  }
  if (ev.key.length === 1 && !ev.ctrlKey && !ev.metaKey && !ev.altKey) {
    ev.preventDefault();
    s.query += ev.key;
    searchHistory(s.pos);
    return;
  }
  if (ev.key !== "Shift" && ev.key !== "CapsLock") exitConsoleSearch(true);
}

function searchHistory(fromIdx) {
  const s = state.console.search;
  const hist = state.console.history;
  if (!s.query) {
    s.pos = hist.length - 1;
    s.failed = false;
    renderConsoleSearch();
    return;
  }
  let idx = Math.min(fromIdx, hist.length - 1);
  while (idx >= 0 && !hist[idx].includes(s.query)) idx--;
  s.failed = idx < 0;
  if (idx >= 0) {
    s.pos = idx;
    setConsoleValue(hist[idx], true);
  }
  renderConsoleSearch();
}

function exitConsoleSearch(keep) {
  const c = state.console;
  if (!c.search) return;
  const saved = c.search.saved;
  c.search = null;
  if (!keep) setConsoleValue(saved, true);
  renderConsoleSearch();
}

function renderConsoleSearch() {
  const box = el("console-search");
  if (!box) return;
  const s = state.console.search;
  box.textContent = s
    ? `(reverse-i-search) '${s.query}'${s.failed ? " — no match" : ""}`
    : "";
}

async function submitConsole() {
  const raw = state.console.text.trim();
  const lines = raw
    .split("\n")
    .map((l) => l.trim())
    .filter((l) => l.length && !l.startsWith(";"));
  if (!lines.length) return;
  pushConsoleHistory(raw);
  state.console.cursor = null;
  state.console.draft = "";
  setConsoleValue("", true);
  await runGcode(lines, "console");
}

// --- boot -------------------------------------------------------------------

function initShell() {
  el("estop-btn").addEventListener("click", emergencyStop);
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
