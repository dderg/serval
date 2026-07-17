"use strict";

const REFRESH_MS = 5000;
const MOONRAKER_KEY = "servoCalMoonrakerUrl";
const CONSOLE_HISTORY_KEY = "servoCalConsoleHistory";
const HELP_CACHE_KEY = "servoCalGcodeHelp";
const CONSOLE_HISTORY_MAX = 500;
const PALETTE = ["#4fb3ff", "#e05a4f", "#4caf50", "#d9a441", "#b388ff", "#4fd8c4"];
const RESONANCE_BAND_HZ = [20, 450];
const PSD_MAX_FREQ_KEY = "servoCalPsdMaxFreqHz";
const MOTOR_VIEW_KEY = "servoCalMotorView";
const PSD_MAX_FREQ_CHOICES_HZ = [250, 500, 750, 1000, 1500];
const PSD_MAX_FREQ_DEFAULT_HZ = 750;
const INITIAL_SELECTED_RUNS = 1;
const PEAK_MIN_SEPARATION_HZ = 15;
const PEAK_LIST_SIZE = 3;

// Each page serves one calibration activity with only the tools that
// activity needs (docs/plans/servo-calibration-automation.md, second demo
// review): the interleaved tuning loop is navigation between pages, not
// scrolling within one.
const PAGE_DEFS = {
  gains: {
    // gains and notches are one tuning loop, not two — the resonances the
    // PSD shows are what keep gains from going higher, so the gains and notch
    // grids, the peak list, and the metrics-vs-gain chart share one page.
    label: "gains",
    groups: ["gains", "notch"],
    experiments: ["gain_sweep", "refine_sweep", "gain_ladder", "tracking"],
    charts: ["psd"],
    intro:
      "find the highest speed gain without resonance or torque rail, then " +
      "notch out whatever resonance the PSD shows so gains can go higher",
    metrics: true,
    sweepChart: true,
    peaks: true,
    templates: [
      {
        label: "ladder…",
        command: "SERVO_GAIN_LADDER SAFE=550 START=700 STEP=50 MAX=900 AXIS=X ITERATIONS=1",
        title: "climb from START by STEP until a rung flags, then revert to SAFE",
      },
      {
        label: "tracking…",
        command: "SERVO_MEASURE_TRACKING AXIS=X SPEED=100 ACCEL=3000 ITERATIONS=3",
        title:
          "single stroke run with capture — the before/after check for any tuning " +
          "change; per-drive overshoot/settle land in the tracking metrics table",
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
    experiments: ["tracking", "inertia_grid", "differential"],
    charts: ["frf"],
    metrics: true,
    intro: "identify the load, then let feedforward carry it",
    templates: [
      {
        label: "fit…",
        command: "SERVO_FIT_DYNAMICS",
        title:
          "strokes the axis, fits inertia/friction per drive, prints the recommended " +
          "inertia ratio and writes the feedforward profile",
      },
      {
        label: "tracking…",
        command: "SERVO_MEASURE_TRACKING AXIS=X SPEED=100 ACCEL=3000 ITERATIONS=3",
        title:
          "single stroke run with capture — the before/after check for any tuning " +
          "change; per-drive overshoot/settle land in the tracking metrics table",
      },
    ],
  },
  strain: {
    label: "strain",
    strain: true,
    experiments: ["strain_map"],
    intro: "map differential belt torque across the bed — elastic strain and friction",
    templates: [
      {
        label: "map…",
        command: "SERVO_MEASURE_STRAIN_MAP LINE_SPACING=20 SPEED=50 ACCEL=1000 TAG=strain",
        title:
          "raster the bed with slow strokes — parks at the region center and runs " +
          "SERVO_SYNC first so every map shares a preload zero; omit X/Y_START/END " +
          "to cover the whole probed region",
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
  docs: {
    label: "docs",
    docs: true,
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
  pinned: new Set(), // runs that stay selected when a plain click switches runs
  runColors: new Map(), // run name -> palette color, kept while the run stays selected
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
  strain: {
    selected: null, // run name shown on the strain page; auto-picks the newest
    compare: new Set(), // extra run names diffed against `selected` when dimensions match
    cache: new Map(), // name -> {mtime_utc, data} from /api/runs/<name>/strain
    field: "elastic", // which half to chart: elastic (fwd+back)/2 or friction (fwd-back)/2
  },
  sentLog: [], // {time, label, lines, results} — every G-code batch sent this session
  help: {
    commands: null, // SERVO_* name -> cmd_*_help string, straight from klippy
    fetchedUtc: null,
    cached: false, // true when `commands` came from localStorage, not a live fetch
    error: null,
    pending: false,
    klippyState: null, // last /server/info klippy_state, to refetch after a RESTART
  },
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

function ambientNotches(manifest) {
  return (manifest && manifest.ambient && manifest.ambient.notches) || null;
}

function flatAmbient(manifest, includeNotches) {
  const flat = {};
  for (const [motor, addrs] of Object.entries(journalParams(manifest))) {
    flat[motor] = { ...addrs };
  }
  if (!includeNotches) return flat;
  for (const [motor, state] of Object.entries(ambientNotches(manifest) || {})) {
    const dst = (flat[motor] = flat[motor] || {});
    for (const [key, value] of Object.entries(state)) {
      if (value && typeof value === "object") {
        for (const [field, v] of Object.entries(value)) dst[`${key}.${field}`] = v;
      } else {
        dst[`notch_${key}`] = value;
      }
    }
  }
  return flat;
}

function motorCount(journal) {
  return Object.keys(journal).length;
}

function ambientDiff(prevManifest, curManifest) {
  // Notch state only diffs when both runs recorded it - a null/legacy
  // previous manifest would otherwise flood the column with ?->value
  // lines for every notch field.
  const bothNotches = !!(ambientNotches(prevManifest) && ambientNotches(curManifest));
  const prev = flatAmbient(prevManifest, bothNotches);
  const cur = flatAmbient(curManifest, bothNotches);
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
    `placeholder="g-code — enter runs, tab completes, ↑/↓ history, ctrl+r search"></textarea></div>` +
    `<div id="console-search" class="console-search"></div>` +
    `<div id="console-help" class="console-help"></div>` +
    `</section>`
  );
}

/// The charts that fold drives into one trace (avg PSD, worst-drive sweep
/// metrics, combined time domain) all obey this one switch; per-motor
/// expands them into a trace per drive, and "avg" (where offered) shows
/// the mean over drives instead of the worst.
function motorView() {
  const v = localStorage.getItem(MOTOR_VIEW_KEY);
  return v === "per-motor" || v === "avg" ? v : "agg";
}

function motorViewPerMotor() {
  return motorView() === "per-motor";
}

/// Sections whose aggregate is already an average (PSD, combined time
/// domain) don't offer a separate "avg" chip; there, the stored "avg"
/// view lights up the aggregate chip.
function motorViewEffective(withAvg) {
  const view = motorView();
  return !withAvg && view === "avg" ? "agg" : view;
}

function motorViewToggleHtml(aggLabel, withAvg = false) {
  const effective = motorViewEffective(withAvg);
  const chip = (v, label) =>
    `<button class="chip motor-view-btn${effective === v ? " active" : ""}" data-view="${v}">${label}</button>`;
  return (
    `<span class="chips motor-view-chips${withAvg ? " with-avg" : ""}">` +
    chip("agg", aggLabel) +
    (withAvg ? chip("avg", "avg") : "") +
    chip("per-motor", "per-motor") +
    `</span>`
  );
}

function syncMotorViewChips() {
  document.querySelectorAll(".motor-view-chips").forEach((group) => {
    const effective = motorViewEffective(group.classList.contains("with-avg"));
    group.querySelectorAll(".motor-view-btn").forEach((b) => {
      b.classList.toggle("active", b.dataset.view === effective);
    });
  });
}

function sectionHeadHtml(title, toolsHtml) {
  return (
    `<div class="section-head"><h2>${title}</h2></div>` +
    (toolsHtml ? `<div class="section-tools">${toolsHtml}</div>` : "")
  );
}

function analysisSectionsHtml(def) {
  const parts = [];
  parts.push(
    `<section class="runs-section">` +
      sectionHeadHtml(
        "runs",
        `<span class="note">${def.experiments ? def.experiments.join(", ") : "all experiments"} — click a row to chart it</span>`
      ) +
      `<div class="table-wrap runs-wrap"><table><thead><tr>` +
      `<th></th><th>time</th><th>tag</th><th>ambient diff vs previous</th><th>note</th><th></th>` +
      `</tr></thead><tbody id="journal-body"></tbody></table></div>` +
      `</section>`
  );
  if (def.metrics) {
    parts.push(
      `<section class="metrics-section">` +
        sectionHeadHtml(
          "tracking metrics",
          motorViewToggleHtml("worst drive", true) +
            `<span class="note">worst move of each step — ` +
            `overshoot/settle measured over the dwell after each move</span>`
        ) +
        `<div id="metrics-table"><p class="note">select runs above</p></div>` +
        `</section>`
    );
  }
  if (def.sweepChart) {
    parts.push(
      `<section class="sweep-metrics-section">` +
        sectionHeadHtml(
          "metrics vs gain",
          motorViewToggleHtml("worst drive", true) +
            `<span class="note">● solid: overshoot, dashed: ferr rms, ` +
            `dotted: ferr peak; red rung: step flagged resonance/torque</span>`
        ) +
        `<div class="charts" id="sweep-metrics-chart"><p class="note">select runs above</p></div>` +
        `</section>`
    );
  }
  if (def.charts && def.charts.includes("frf")) {
    parts.push(
      `<section class="frf-section" id="frf-section" hidden>` +
        sectionHeadHtml("differential belt FRF", `<span class="note" id="frf-meta"></span>`) +
        `<div class="charts" id="frf-charts"></div>` +
        `<div id="frf-modes"></div>` +
        `</section>`
    );
  }
  if (def.charts && def.charts.includes("psd")) {
    parts.push(
      `<section class="psd-section">` +
        sectionHeadHtml(
          "following-error PSD",
          motorViewToggleHtml("avg") +
            `<label class="note">to <select id="psd-max-freq">` +
            PSD_MAX_FREQ_CHOICES_HZ.map(
              (f) =>
                `<option value="${f}"${f === psdMaxFreqHz() ? " selected" : ""}>${f}</option>`
            ).join("") +
            `</select> Hz</label>` +
            `<div class="chips" id="psd-step-chips"></div>`
        ) +
        `<div class="charts" id="psd-charts"><p class="note">select runs above</p></div>` +
        `</section>`
    );
  }
  if (def.peaks) {
    parts.push(
      `<section class="peaks-section">` +
        sectionHeadHtml("detected peaks", `<span class="note" id="peaks-run"></span>`) +
        `<div id="peak-list"><p class="note">select runs above</p></div>` +
        `</section>`
    );
  }
  if (def.charts && def.charts.includes("time")) {
    parts.push(
      `<section class="time-section">` +
        sectionHeadHtml(
          "time domain — following error",
          motorViewToggleHtml("combined") + `<div class="chips" id="time-step-chips"></div>`
        ) +
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
    sectionHeadHtml(
      "live following error — per motor",
      `<label class="live-window">window ` +
        `<input type="range" id="live-window" min="2" max="30" step="1" value="${state.live.windowS}">` +
        `<span id="live-window-value">${state.live.windowS} s</span></label>` +
        `<span class="note" id="live-status">connecting to the telemetry tap…</span>`
    ) +
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

/// Collapsible analysis sections (accordion). Each `.analysis .section-head`
/// holds only the section's h2 and is the sole click target; it toggles a
/// `.collapsed` class on its parent `<section>`. Controls (chips, selects,
/// notes) live in a `.section-tools` row below the head, so they collapse
/// with the content and never sit inside the fold hitbox. Collapse state
/// is keyed by page + heading text and persists in localStorage.
const ACCORDION_KEY = "servoCal.collapsedSections";

function loadCollapsedSections() {
  try {
    return new Set(JSON.parse(localStorage.getItem(ACCORDION_KEY) || "[]"));
  } catch {
    return new Set();
  }
}

function sectionLabel(head) {
  const h = head.querySelector("h2");
  return `${state.page}::${h ? h.textContent.trim() : head.textContent.trim()}`;
}

function applyAccordionState() {
  const collapsed = loadCollapsedSections();
  document.querySelectorAll("#page-root .analysis .section-head").forEach((head) => {
    head.classList.add("has-caret");
    const section = head.parentElement;
    if (section && section.tagName === "SECTION") {
      if (collapsed.has(sectionLabel(head))) section.classList.add("collapsed");
      else section.classList.remove("collapsed");
    }
  });
}

/// Bound once at boot: one delegated listener survives every page rebuild.
function bindAccordionToggle() {
  document.addEventListener("click", (e) => {
    const head = e.target.closest(".analysis .section-head");
    if (!head) return;
    const section = head.parentElement;
    if (!section || section.tagName !== "SECTION") return;
    section.classList.toggle("collapsed");
    const collapsed = loadCollapsedSections();
    const label = sectionLabel(head);
    if (section.classList.contains("collapsed")) collapsed.add(label);
    else collapsed.delete(label);
    localStorage.setItem(ACCORDION_KEY, JSON.stringify([...collapsed]));
  });
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
    applyAccordionState();
    return;
  }
  if (def.strain) {
    root.innerHTML = strainShellHtml(def);
    bindPageEvents();
    document.querySelectorAll("button.strain-field-btn").forEach((btn) => {
      btn.addEventListener("click", () => {
        state.strain.field = btn.dataset.field;
        redrawStrain();
      });
    });
    renderSentLog();
    redrawStrain();
    applyAccordionState();
    return;
  }
  if (def.docs) {
    root.innerHTML = docsShellHtml();
    bindPageEvents();
    renderDocsList();
    renderSentLog();
    applyAccordionState();
    if (!state.help.commands || state.help.cached) fetchMacroHelp();
    return;
  }
  if (def.journal) {
    root.innerHTML =
      `<div class="workspace single">` +
      `<main class="analysis">` +
      `<section class="runs-section">` +
      `<div class="section-head"><h2>journal — every run</h2></div>` +
      `<div class="table-wrap journal-wrap"><table><thead><tr>` +
      `<th></th><th>time</th><th>experiment/tag</th><th>ambient diff vs previous</th><th>note</th><th></th>` +
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
  applyAccordionState();
}

/// Drag handles on every header cell of the run tables. The first drag
/// freezes the browser's auto layout into explicit widths and switches the
/// table to fixed layout, so a column can shrink below its content (cells
/// ellipsize) instead of forcing horizontal scroll.
function makeColumnsResizable(table) {
  const ths = [...table.querySelectorAll("thead th")];
  const freezeLayout = () => {
    if (table.style.tableLayout === "fixed") return;
    for (const th of ths) th.style.width = `${th.offsetWidth}px`;
    table.style.tableLayout = "fixed";
  };
  ths.forEach((th) => {
    const grip = document.createElement("span");
    grip.className = "col-resizer";
    th.appendChild(grip);
    grip.addEventListener("mousedown", (e) => {
      e.preventDefault();
      e.stopPropagation();
      freezeLayout();
      const startX = e.pageX;
      const startW = th.offsetWidth;
      const onMove = (ev) => {
        th.style.width = `${Math.max(24, startW + ev.pageX - startX)}px`;
      };
      const onUp = () => {
        document.removeEventListener("mousemove", onMove);
        document.removeEventListener("mouseup", onUp);
      };
      document.addEventListener("mousemove", onMove);
      document.addEventListener("mouseup", onUp);
    });
  });
}

function bindPageEvents() {
  bindConsole();
  document
    .querySelectorAll(".runs-wrap table, .journal-wrap table")
    .forEach(makeColumnsResizable);
  const applyBtn = el("drive-apply-btn");
  if (applyBtn) applyBtn.addEventListener("click", applyDriveChanges);
  const psdMax = el("psd-max-freq");
  if (psdMax) {
    psdMax.addEventListener("change", () => {
      localStorage.setItem(PSD_MAX_FREQ_KEY, psdMax.value);
      redrawCharts();
    });
  }
  document.querySelectorAll("button.motor-view-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      localStorage.setItem(MOTOR_VIEW_KEY, btn.dataset.view);
      syncMotorViewChips();
      redrawCharts();
    });
  });
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
  const editing = document.activeElement;
  if (editing && editing.classList.contains("run-note-input") && tbody.contains(editing)) {
    return;
  }
  const def = currentPageDef();
  const runs = def.journal ? state.runs : pageRuns(def);
  tbody.innerHTML = "";
  runs.forEach((run) => {
    const globalIdx = state.runs.indexOf(run);
    const detail = state.details.get(run.name);
    const manifest = detail && detail.manifest;
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
        if (state.selected.has(run.name)) {
          state.selected.delete(run.name);
          state.pinned.delete(run.name);
        } else {
          state.selected.add(run.name);
        }
      } else {
        const unpinnedOthers = [...state.selected].filter(
          (n) => n !== run.name && !state.pinned.has(n)
        );
        if (
          state.selected.has(run.name) &&
          !state.pinned.has(run.name) &&
          unpinnedOthers.length === 0
        ) {
          state.selected.delete(run.name);
        } else {
          state.selected = new Set([...state.pinned, run.name]);
        }
      }
      syncRunColors();
      renderRuns();
      redrawCharts();
    });

    const dotTd = document.createElement("td");
    if (state.runColors.has(run.name)) {
      const swatch = document.createElement("span");
      swatch.className = "swatch";
      swatch.style.background = runColor(run.name);
      dotTd.appendChild(swatch);
    }
    if (run.has_results) {
      const pinBtn = document.createElement("button");
      pinBtn.className = state.pinned.has(run.name) ? "pin-toggle pinned" : "pin-toggle";
      pinBtn.textContent = "📌";
      pinBtn.title = state.pinned.has(run.name)
        ? "unpin — plain clicks will deselect this run again"
        : "pin — keep this run selected while plain clicks switch other runs";
      pinBtn.addEventListener("click", (e) => {
        e.stopPropagation();
        if (state.pinned.has(run.name)) {
          state.pinned.delete(run.name);
        } else {
          state.pinned.add(run.name);
          state.selected.add(run.name);
        }
        syncRunColors();
        renderRuns();
        redrawCharts();
      });
      dotTd.appendChild(pinBtn);
    }
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

    tr.appendChild(noteCell(run));

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

/// Click-to-edit note cell: shows the saved note (or a faint "add note…"
/// hint), swaps to an input on click, saves to POST /api/runs/<name>/note
/// on Enter/blur, and cancels on Escape. Clicks stop propagating so
/// editing a note never toggles the row's chart selection.
function noteCell(run) {
  const td = document.createElement("td");
  td.className = run.note ? "run-note" : "run-note empty";
  td.textContent = run.note || "add note…";
  td.title = run.note ? `${run.note} — click to edit` : "click to add a note";
  td.addEventListener("click", (e) => {
    e.stopPropagation();
    if (td.querySelector("input")) return;
    const input = document.createElement("input");
    input.type = "text";
    input.className = "run-note-input";
    input.value = run.note || "";
    td.textContent = "";
    td.appendChild(input);
    input.focus();
    let done = false;
    const finish = (save) => {
      if (done) return;
      done = true;
      const text = input.value;
      input.remove();
      if (save) {
        saveNote(run, text);
      } else {
        renderRuns();
      }
    };
    input.addEventListener("keydown", (ev) => {
      ev.stopPropagation();
      if (ev.key === "Enter") finish(true);
      if (ev.key === "Escape") finish(false);
    });
    input.addEventListener("blur", () => finish(true));
    input.addEventListener("click", (ev) => ev.stopPropagation());
  });
  return td;
}

async function saveNote(run, text) {
  try {
    const saved = await api(`/api/runs/${encodeURIComponent(run.name)}/note`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ note: text }),
    });
    run.note = saved.note || null;
  } catch (e) {
    console.error(e);
    alert(`saving note failed: ${e.message}`);
  }
  renderRuns();
}

async function triggerAnalyze(name) {
  await api(`/api/runs/${encodeURIComponent(name)}/analyze`, { method: "POST" });
  await refresh();
}

function selectedRunNames() {
  return state.runs.filter((r) => state.selected.has(r.name)).map((r) => r.name);
}

/// Colors stick to runs for as long as they stay selected: deselecting one
/// frees its color without touching anyone else's, and a newly selected run
/// takes the least-used palette color instead of everything reshuffling.
function syncRunColors() {
  for (const name of [...state.runColors.keys()]) {
    if (!state.selected.has(name)) state.runColors.delete(name);
  }
  for (const name of selectedRunNames()) {
    if (!state.runColors.has(name)) state.runColors.set(name, leastUsedColor());
  }
}

function leastUsedColor() {
  const counts = new Map(PALETTE.map((c) => [c, 0]));
  for (const c of state.runColors.values()) counts.set(c, counts.get(c) + 1);
  return PALETTE.reduce((best, c) => (counts.get(c) < counts.get(best) ? c : best));
}

function runColor(name) {
  const color = state.runColors.get(name);
  if (!color) throw new Error(`${name}: no color assigned — run is not selected`);
  return color;
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
  for (const name of [...state.pinned]) {
    if (!known.has(name)) state.pinned.delete(name);
  }
  autoSelectInitialRuns();
  syncRunColors();
  renderRuns();
  await redrawCharts();
}

// --- chart drawing ------------------------------------------------------------

function pickSeries(runName, step) {
  if (motorViewPerMotor()) {
    const drives = Object.entries(step.drives);
    return drives.map(([drive, d], k) => ({
      y: d.ferr_counts.map((c) => c * (1000 / countsPerMm(runName, drive))),
      label: "ferr (µm)",
      suffix: ` (${drive})`,
      ramp: driveRamp(drives.length, k),
    }));
  }
  if (step.combined) {
    return [{ y: step.combined.on_ferr_mm, label: "on-axis ferr (mm)", suffix: "", ramp: 0 }];
  }
  const firstDrive = Object.values(step.drives)[0];
  return [
    {
      y: firstDrive ? firstDrive.ferr_counts : [],
      label: "ferr (counts)",
      suffix: "",
      ramp: 0,
    },
  ];
}

/// Renders at the device pixel ratio so lines stay vector-crisp on hidpi
/// displays: the backing store is sized to the CSS box × dpr and the
/// context scaled back, while all layout math stays in CSS pixels.
function drawChart(canvas, traces, yLabel, fixedY, xUnit, marks, opts) {
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

  const fmtTick = (v, span) => (Math.abs(span) >= 20 ? v.toFixed(0) : v.toFixed(2));
  // Tooltip readout: keep one more decimal than the axis ticks so the
  // hovered step's metrics read exact, not rounded to a gridline scale.
  const fmtVal = (v) => (Math.abs(v) >= 1000 ? v.toFixed(0) : v.toFixed(1));
  ctx.strokeStyle = "#29313a";
  ctx.fillStyle = "#8a97a3";
  ctx.font = "10px monospace";
  ctx.beginPath();
  for (let i = 0; i <= 4; i++) {
    const v = yMin + ((yMax - yMin) * i) / 4;
    const py = y(v);
    ctx.moveTo(pad.l, py);
    ctx.lineTo(w - pad.r, py);
    ctx.fillText(fmtTick(v, yMax - yMin), 2, py + 3);
  }
  for (let i = 0; i <= 4; i++) {
    const t = tMin + ((tMax - tMin) * i) / 4;
    const px = x(t);
    ctx.fillText(fmtTick(t, tMax - tMin) + (xUnit == null ? "s" : xUnit), px, h - 6);
  }
  ctx.stroke();

  for (const m of marks || []) {
    if (m.x < tMin || m.x > tMax) continue;
    ctx.strokeStyle = m.color;
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(x(m.x), pad.t);
    ctx.lineTo(x(m.x), h - pad.b);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  for (const tr of traces) {
    ctx.strokeStyle = tr.color;
    ctx.lineWidth = 1.25;
    ctx.setLineDash(tr.dash || []);
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
    ctx.setLineDash([]);
    if (tr.points) {
      ctx.fillStyle = tr.color;
      for (let i = 0; i < tr.t.length; i++) {
        if (tr.y[i] === null) continue;
        ctx.beginPath();
        ctx.arc(x(tr.t[i]), y(tr.y[i]), 3, 0, 2 * Math.PI);
        ctx.fill();
      }
    }
  }
  ctx.fillStyle = "#8a97a3";
  ctx.fillText(yLabel, pad.l, 10);
  if (opts.hover) {
    // Like the PSD chart: snap to the single nearest point (by 2D distance),
    // draw a vertical line through it, and read out that one point's values.
    // Not every trace at that x — the hovered point, for that run/metric.
    let best = null;
    for (const tr of traces) {
      for (let i = 0; i < tr.t.length; i++) {
        if (tr.y[i] === null) continue;
        const dx = x(tr.t[i]) - opts.hover.mx;
        const dy = y(tr.y[i]) - opts.hover.my;
        const d = dx * dx + dy * dy;
        if (!best || d < best.d) best = { d, tr, i };
      }
    }
    if (best) {
      const px = x(best.tr.t[best.i]);
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
      const xUnitSuffix = xUnit == null ? "s" : xUnit;
      const swept = opts.xTitle ? opts.xTitle + " = " : "";
      const lab = best.tr.label != null ? best.tr.label : "";
      const text = `${swept}${fmtTick(best.tr.t[best.i], tMax - tMin)}${xUnitSuffix}  ${fmtVal(best.tr.y[best.i])} ${yLabel}${lab ? "  " + lab : ""}`;
      ctx.font = "11px monospace";
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

/// Redraws with the cursor on every move so the readout snaps to the nearest
/// sweep step — the discrete metrics-vs-gain variant of the PSD hover.
/// drawChart reruns the full axis/line pass each redraw; it's cheap.
function attachChartHover(canvas, traces, yLabel, fixedY, xUnit, marks, opts) {
  const redraw = (hover) =>
    drawChart(canvas, traces, yLabel, fixedY, xUnit, marks, hover ? { ...opts, hover } : opts);
  canvas.addEventListener("mousemove", (e) => redraw({ mx: e.offsetX, my: e.offsetY }));
  canvas.addEventListener("mouseleave", () => redraw(null));
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
      for (const series of pickSeries(names[i], step)) {
        yLabel = series.label;
        const color = mixColor(runColor(names[i]), "#ffffff", series.ramp);
        traces.push({ t: step.t_s, y: series.y, color });
        const item = document.createElement("span");
        item.innerHTML =
          `<span class="swatch" style="background:${color}"></span>` +
          `${names[i]}${series.suffix}`;
        legend.appendChild(item);
      }
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

/// One run selected: each step gets its own palette color, rotated so the
/// first step is exactly the run's table-swatch color — the swatch and the
/// chart must never disagree, whatever color the run ended up holding.
/// Several runs: each run keeps its table-swatch hue and its steps ramp
/// toward white, so runs stay distinguishable and the step chips are the
/// clutter valve.
function traceStyle(names, steps, runIdx, stepIdx) {
  if (names.length === 1) {
    const base = runColor(names[0]);
    const baseIdx = PALETTE.indexOf(base);
    if (baseIdx < 0) throw new Error(`${base}: run color is not in the palette`);
    return {
      color: PALETTE[(baseIdx + stepIdx) % PALETTE.length],
      name: steps[stepIdx],
    };
  }
  const base = runColor(names[runIdx]);
  const ramp = steps.length > 1 ? (0.55 * stepIdx) / (steps.length - 1) : 0;
  const name =
    steps.length === 1 ? names[runIdx] : `${names[runIdx]} · ${steps[stepIdx]}`;
  return { color: mixColor(base, "#ffffff", ramp), name };
}

/// Drawing the full Nyquist span squishes the servo/mechanical modes into
/// the left quarter of the chart, so the user picks the band ceiling.
function psdMaxFreqHz() {
  const stored = Number(localStorage.getItem(PSD_MAX_FREQ_KEY));
  return PSD_MAX_FREQ_CHOICES_HZ.includes(stored)
    ? stored
    : PSD_MAX_FREQ_DEFAULT_HZ;
}

function clipToPsdBand(freq, y) {
  const maxHz = psdMaxFreqHz();
  let end = freq.length;
  while (end > 0 && freq[end - 1] > maxHz) end--;
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

// --- tracking metrics table -----------------------------------------------
//
// The replacement for the old gain-report PNG's "metrics vs gain" panel:
// results.json already carries per-drive, per-move ferr/overshoot/settle and
// the torque summary — this table is the view on top of them.

function driveMoveSummary(metrics) {
  const s = {
    ferrPeak: 0,
    ferrRms: 0,
    overshoot: 0,
    settleWorstMs: null,
    neverSettled: false,
    truncated: false,
  };
  for (const mv of metrics.moves) {
    s.ferrPeak = Math.max(s.ferrPeak, mv.ferr_peak);
    s.ferrRms = Math.max(s.ferrRms, mv.ferr_rms);
    s.overshoot = Math.max(s.overshoot, mv.overshoot);
    if (mv.settle_ms != null) {
      if (s.settleWorstMs == null || mv.settle_ms > s.settleWorstMs) {
        s.settleWorstMs = mv.settle_ms;
      }
    } else if (mv.settle_window_truncated) {
      s.truncated = true;
    } else {
      s.neverSettled = true;
    }
  }
  return s;
}

function settleCellHtml(s) {
  if (s.neverSettled) return `<span class="badge resonance">never</span>`;
  const truncatedBadge =
    `<span class="badge truncated" title="the capture ended inside a move's ` +
    `settle window, so the worst settle may be underestimated">truncated</span>`;
  if (s.settleWorstMs == null) return s.truncated ? truncatedBadge : "—";
  const value = `${s.settleWorstMs.toFixed(1)} ms`;
  return s.truncated ? `${value} ${truncatedBadge}` : value;
}

function torqueCellHtml(tq) {
  const peak = `${tq.peak_pct_rated.toFixed(0)}%`;
  if (!tq.rail_detected) return peak;
  return (
    `${peak} <span class="badge torque" title="on the rail ${tq.rail_pct_moving.toFixed(1)}% ` +
    `of moving time (${tq.rail_ms.toFixed(0)} ms, longest burst ${tq.longest_burst_ms.toFixed(0)} ms)">rail</span>`
  );
}

function metricsDriveRow(name, stepName, drive, dr) {
  const umPerCount = 1000 / countsPerMm(name, drive);
  const s = driveMoveSummary(dr.metrics);
  return {
    run: name,
    step: stepName,
    drive,
    ferrPeakUm: s.ferrPeak * umPerCount,
    ferrRmsUm: s.ferrRms * umPerCount,
    overshootUm: s.overshoot * umPerCount,
    settle: {
      settleWorstMs: s.settleWorstMs,
      neverSettled: s.neverSettled,
      truncated: s.truncated,
    },
    torque: dr.metrics.torque,
  };
}

/// One row per (run, step) folded over drives: "agg" keeps the worst drive
/// per metric, "avg" the mean. Rail badges survive both folds — a railed
/// drive is a railed step no matter the view.
function foldDriveRows(driveRows, view) {
  const fold = (values) =>
    view === "avg"
      ? values.reduce((a, b) => a + b, 0) / values.length
      : Math.max(...values);
  const settled = driveRows
    .map((r) => r.settle.settleWorstMs)
    .filter((v) => v != null);
  const worstTorque = driveRows.reduce((a, r) =>
    r.torque.peak_pct_rated > a.torque.peak_pct_rated ? r : a
  ).torque;
  return {
    run: driveRows[0].run,
    step: driveRows[0].step,
    drive: view === "avg" ? "avg" : "worst",
    ferrPeakUm: fold(driveRows.map((r) => r.ferrPeakUm)),
    ferrRmsUm: fold(driveRows.map((r) => r.ferrRmsUm)),
    overshootUm: fold(driveRows.map((r) => r.overshootUm)),
    settle: {
      settleWorstMs: settled.length ? fold(settled) : null,
      neverSettled: driveRows.some((r) => r.settle.neverSettled),
      truncated: driveRows.some((r) => r.settle.truncated),
    },
    torque:
      view === "avg"
        ? {
            ...worstTorque,
            peak_pct_rated: fold(driveRows.map((r) => r.torque.peak_pct_rated)),
          }
        : worstTorque,
  };
}

function metricsTableRows(names, steps) {
  const view = motorView();
  const rows = [];
  for (const name of names) {
    const detail = state.details.get(name);
    if (!detail || !detail.results) continue;
    for (const step of detail.results.steps) {
      if (!steps.includes(step.name)) continue;
      const driveRows = Object.entries(step.drives).map(([drive, dr]) =>
        metricsDriveRow(name, step.name, drive, dr)
      );
      if (!driveRows.length) continue;
      if (view === "per-motor") rows.push(...driveRows);
      else rows.push(foldDriveRows(driveRows, view));
    }
  }
  return rows;
}

/// Red tint scaled to where the value sits between the column's best and
/// worst — the cheap-to-scan replacement for reading 4-drives-per-step
/// numbers one by one. Identical columns get no tint.
function heatCellStyle(value, min, max) {
  if (!(max > min)) return "";
  const alpha = (0.32 * (value - min)) / (max - min);
  return alpha < 0.02 ? "" : ` style="background:rgba(224,90,79,${alpha.toFixed(3)})"`;
}

function renderMetricsTable(names, steps) {
  const container = el("metrics-table");
  if (!container) return;
  const rows = metricsTableRows(names, steps);
  if (!rows.length) {
    container.innerHTML = '<p class="note">select runs above</p>';
    return;
  }
  const columns = ["ferrPeakUm", "ferrRmsUm", "overshootUm"];
  const bounds = {};
  for (const c of columns) {
    const values = rows.map((r) => r[c]);
    bounds[c] = { min: Math.min(...values), max: Math.max(...values) };
  }
  const heat = (c, r) => heatCellStyle(r[c], bounds[c].min, bounds[c].max);
  const stepColors = new Map();
  for (const r of rows) {
    if (!stepColors.has(r.step)) {
      stepColors.set(r.step, PALETTE[stepColors.size % PALETTE.length]);
    }
  }
  const body = rows
    .map((r, i) => {
      const swatch = `<span class="swatch" style="background:${runColor(r.run)}"></span>`;
      const stepColor = stepColors.get(r.step);
      const prev = rows[i - 1];
      const groupStart = !prev || prev.run !== r.run || prev.step !== r.step;
      return (
        `<tr${groupStart && i > 0 ? ' class="group-start"' : ""}>` +
        `<td class="run-cell" style="border-left:3px solid ${stepColor};padding-left:6px" ` +
        `title="${r.run}">${swatch}${r.run}</td>` +
        `<td style="color:${stepColor}">${r.step}</td><td>${r.drive}</td>` +
        `<td class="num"${heat("ferrPeakUm", r)}>${r.ferrPeakUm.toFixed(1)}</td>` +
        `<td class="num"${heat("ferrRmsUm", r)}>${r.ferrRmsUm.toFixed(1)}</td>` +
        `<td class="num"${heat("overshootUm", r)}>${r.overshootUm.toFixed(1)}</td>` +
        `<td class="num">${settleCellHtml(r.settle)}</td>` +
        `<td class="num">${torqueCellHtml(r.torque)}</td></tr>`
      );
    })
    .join("");
  container.innerHTML =
    `<table class="metrics-table"><thead><tr>` +
    `<th>run</th><th>step</th><th>drive</th>` +
    `<th class="num">ferr peak (µm)</th><th class="num">ferr rms (µm)</th>` +
    `<th class="num">overshoot (µm)</th><th class="num">settle</th>` +
    `<th class="num">torque peak</th>` +
    `</tr></thead><tbody>${body}</tbody></table>`;
}

// --- metrics vs gain chart --------------------------------------------------
//
// The old gain-report PNG's "metrics vs gain" panel: one x position per
// sweep step (the swept gain value from the manifest), overshoot / ferr
// per step maxed over drives, flagged steps marked as red rungs.

function sweptAxisKey(manifest) {
  if (!manifest || manifest.steps.length < 2) return null;
  const keys = Object.keys(manifest.steps[0].swept || {}).filter((k) =>
    manifest.steps.every((s) => typeof (s.swept || {})[k] === "number")
  );
  const varying = keys.filter(
    (k) => new Set(manifest.steps.map((s) => s.swept[k])).size > 1
  );
  if (!varying.length) return null;
  return varying.includes("speed") ? "speed" : varying[0];
}

function sweepMetricsSeries(names) {
  const series = [];
  for (const name of names) {
    const detail = state.details.get(name);
    if (!detail || !detail.results || !detail.manifest) continue;
    const key = sweptAxisKey(detail.manifest);
    if (!key) continue;
    const sweptByStep = new Map(detail.manifest.steps.map((s) => [s.name, s.swept[key]]));
    const perDrivePoints = new Map();
    for (const step of detail.results.steps) {
      if (!sweptByStep.has(step.name)) continue;
      const flagged = step.flags.some(
        (f) => f === "resonance_detected" || f === "torque_saturated"
      );
      const view = motorView();
      const driveValues = Object.entries(step.drives).map(([drive, dr]) => {
        const umPerCount = 1000 / countsPerMm(name, drive);
        const s = driveMoveSummary(dr.metrics);
        return {
          drive,
          overshootUm: s.overshoot * umPerCount,
          ferrRmsUm: s.ferrRms * umPerCount,
          ferrPeakUm: s.ferrPeak * umPerCount,
        };
      });
      const stepPoints = new Map();
      if (view === "per-motor") {
        for (const v of driveValues) stepPoints.set(v.drive, v);
      } else if (driveValues.length) {
        const fold = (f) =>
          view === "avg"
            ? driveValues.reduce((a, v) => a + f(v), 0) / driveValues.length
            : Math.max(...driveValues.map(f));
        stepPoints.set(view === "avg" ? "avg" : "worst drive", {
          overshootUm: fold((v) => v.overshootUm),
          ferrRmsUm: fold((v) => v.ferrRmsUm),
          ferrPeakUm: fold((v) => v.ferrPeakUm),
        });
      }
      for (const [drive, p] of stepPoints) {
        if (!perDrivePoints.has(drive)) perDrivePoints.set(drive, []);
        perDrivePoints.get(drive).push({ x: sweptByStep.get(step.name), flagged, ...p });
      }
    }
    for (const [drive, points] of perDrivePoints) {
      if (points.length < 2) continue;
      points.sort((a, b) => a.x - b.x);
      series.push({ run: name, drive, key, points });
    }
  }
  return series;
}

function renderSweepMetricsChart(names) {
  const container = el("sweep-metrics-chart");
  if (!container) return;
  const series = sweepMetricsSeries(names);
  if (!series.length) {
    container.innerHTML =
      '<p class="note">select a gain sweep / ladder run above (tracking runs have a single step — read them in the metrics table)</p>';
    return;
  }
  container.innerHTML = "";
  const box = document.createElement("div");
  box.className = "chart-box";
  const title = document.createElement("h3");
  const viewLabel = { agg: "worst-drive", avg: "avg", "per-motor": "per-motor" }[motorView()];
  title.textContent = `${viewLabel} metrics vs swept ${series[0].key} (µm)`;
  box.appendChild(title);
  const canvas = document.createElement("canvas");
  canvas.width = 860;
  canvas.height = 260;
  box.appendChild(canvas);
  const legend = document.createElement("div");
  legend.className = "legend";
  const traces = [];
  const marks = [];
  series.forEach((s) => {
    const runSeries = series.filter((x) => x.run === s.run);
    const color = mixColor(
      runColor(s.run),
      "#ffffff",
      driveRamp(runSeries.length, runSeries.indexOf(s))
    );
    const t = s.points.map((p) => p.x);
    const label = motorViewPerMotor() ? `${s.run} · ${s.drive}` : s.run;
    traces.push({ t, y: s.points.map((p) => p.overshootUm), color, points: true, label: `${label} overshoot` });
    traces.push({ t, y: s.points.map((p) => p.ferrRmsUm), color, dash: [6, 4], label: `${label} ferr rms` });
    traces.push({ t, y: s.points.map((p) => p.ferrPeakUm), color, dash: [2, 3], label: `${label} ferr peak` });
    for (const p of s.points) if (p.flagged) marks.push({ x: p.x, color: "#e05a4f" });
    const item = document.createElement("span");
    item.innerHTML = `<span class="swatch" style="background:${color}"></span>${label}`;
    legend.appendChild(item);
  });
  const sweepOpts = { xTitle: series[0].key };
  drawChart(canvas, traces, "µm", null, "", marks, sweepOpts);
  attachChartHover(canvas, traces, "µm", null, "", marks, sweepOpts);
  box.appendChild(legend);
  container.appendChild(box);
}

function driveRamp(count, idx) {
  return count > 1 ? (0.5 * idx) / (count - 1) : 0;
}

/// Per-drive PSDs are counts²/Hz on drives whose counts_per_mm may differ,
/// so averaging happens in µm²/Hz — each drive converted first, then the
/// power mean — and only then collapses to a tone amplitude.
function psdFerrUm2(step, runName, drive) {
  const umPerCount = 1000 / countsPerMm(runName, drive);
  return step.psd.per_drive[drive].map((p) => p * umPerCount * umPerCount);
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
      const pushTrace = (psdUm2, color, label) => {
        const clipped = clipToPsdBand(step.psd.freq_hz, psdUm2);
        traces.push({
          freq: clipped.freq,
          y: psdToAmplitude(clipped.freq, clipped.y),
          color,
          dashed: false,
          label,
          run: names[i],
        });
      };
      if (motorViewPerMotor()) {
        driveNames.forEach((drive, k) => {
          pushTrace(
            psdFerrUm2(step, names[i], drive),
            mixColor(style.color, "#ffffff", driveRamp(driveNames.length, k)),
            `${style.name} (${drive})`
          );
        });
        return;
      }
      const avgUm2 = new Array(step.psd.freq_hz.length).fill(0);
      for (const drive of driveNames) {
        psdFerrUm2(step, names[i], drive).forEach((v, n) => (avgUm2[n] += v));
      }
      pushTrace(
        avgUm2.map((v) => v / driveNames.length),
        style.color,
        `${style.name} (avg of ${driveNames.length})`
      );
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

/// The one step filter drives every chart that splits by step (PSD, time
/// domain, metrics), so its chips render into every section that has a
/// container for them — otherwise a filter picked on one page silently
/// shapes another page's chart with no control in sight.
function renderStepChips(stepNames) {
  for (const id of ["psd-step-chips", "time-step-chips"]) {
    const container = el(id);
    if (container) fillStepChips(container, stepNames);
  }
}

function fillStepChips(container, stepNames) {
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
      color: runColor(names[i]),
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

// --- strain map (strain page) -------------------------------------------------
//
// Renders GET /api/runs/<name>/strain: per raster line, the elastic
// (direction-symmetric) differential belt torque binned along the sweep.
// All four heatmaps (belt × sweep orientation) draw in BED coordinates —
// horizontal = bed x, vertical = bed y increasing upward, square aspect —
// on one symmetric diverging scale, so a feature at some bed position
// lines up across every panel. X-sweep lines are horizontal bands at
// their swept y; Y-sweep lines are vertical bands at their swept x.

const STRAIN_NEUTRAL = "#1f2630";
const STRAIN_NEG = "#4fb3ff";
const STRAIN_POS = "#e05a4f";
const STRAIN_HEAT_PAD = { l: 40, r: 8, t: 16, b: 26 };
const STRAIN_HEAT_CANVAS_W = 430;
const STRAIN_LINE_SPACING_FALLBACK_MM = 20;

function strainShellHtml(def) {
  return (
    `<div class="workspace">` +
    `<main class="analysis">` +
    `<section class="runs-section">` +
    sectionHeadHtml(
      "strain runs",
      `<span class="note">strain_map — click to map, shift+click a second run to diff (matching dimensions)</span>`
    ) +
    `<div class="table-wrap runs-wrap"><table><thead><tr>` +
    `<th>time</th><th>tag</th><th></th>` +
    `</tr></thead><tbody id="strain-run-body"></tbody></table></div>` +
    `</section>` +
    `<section>` +
    sectionHeadHtml(
      "strain map",
      `<button class="strain-field-btn" data-field="elastic">elastic</button>` +
        `<button class="strain-field-btn" data-field="friction" ` +
        `title="the direction-dependent half: (forward - backward)/2 — what a position-keyed offset cannot cancel">friction</button>` +
        `<span class="note" id="strain-summary"></span>`
    ) +
    `<div id="strain-heatmaps" class="strain-grid"></div>` +
    `<div id="strain-scale"></div>` +
    `</section>` +
    `<section>` +
    sectionHeadHtml(
      "per-line elastic profiles",
      `<span class="note">one polyline per raster line, in bed coordinates</span>`
    ) +
    `<div class="charts" id="strain-profiles"></div>` +
    `</section>` +
    `<section>` +
    sectionHeadHtml(
      "per-line DC offset — mean elastic",
      `<span class="note">a line-to-line offset is trapped preload, not local strain</span>`
    ) +
    `<div class="strain-grid" id="strain-dc"></div>` +
    `</section>` +
    `</main>` +
    `<aside class="controls">${controlsSectionsHtml(def)}</aside>` +
    `</div>`
  );
}

async function ensureStrain(name) {
  const run = state.runs.find((r) => r.name === name);
  const cached = state.strain.cache.get(name);
  if (cached && run && cached.mtime_utc === run.mtime_utc) return cached.data;
  const data = await api(`/api/runs/${encodeURIComponent(name)}/strain`);
  state.strain.cache.set(name, { mtime_utc: run ? run.mtime_utc : null, data });
  return data;
}

function runTag(name) {
  const r = state.runs.find((x) => x.name === name);
  return r ? `${r.tag}${r.axis ? " " + r.axis : ""}` : name;
}

/// Builds per-belt elastic/friction diff arrays (a − b); a null on either
/// side propagates, since an unbinned cell has no meaning to subtract.
function buildDiffLine(base, cmp) {
  return {
    name: base.name,
    swept: base.swept,
    bin_centers: base.bin_centers,
    belts: base.belts.map((bb, bi) => {
      const cb = cmp.belts[bi];
      return {
        pair: bb.pair,
        elastic: cb ? pointwiseDiff(bb.elastic, cb.elastic) : nulls(bb.elastic.length),
        friction: cb ? pointwiseDiff(bb.friction, cb.friction) : nulls(bb.friction.length),
      };
    }),
  };
}

function pointwiseDiff(a, b) {
  const out = new Array(a.length);
  for (let i = 0; i < a.length; i++) {
    out[i] = a[i] === null || b[i] === null ? null : a[i] - b[i];
  }
  return out;
}

function nulls(n) {
  return new Array(n).fill(null);
}

/// Compression of a strain map's geometry: line names (which encode
/// orientation + swept coordinate), per-line bin centers, and belt pairs.
/// Two maps with an identical signature can be subtracted cell-by-cell.
function strainSignature(data) {
  return JSON.stringify({
    l: data.lines.map((l) => ({ n: l.name, b: l.bin_centers, p: l.belts.map((x) => x.pair) })),
  });
}

function renderStrainRuns() {
  const tbody = el("strain-run-body");
  if (!tbody) return;
  const runs = pageRuns(currentPageDef());
  if (!runs.some((r) => r.name === state.strain.selected)) {
    state.strain.selected = runs.length ? runs[0].name : null;
  }
  const known = new Set(runs.map((r) => r.name));
  for (const name of [...state.strain.compare]) {
    if (!known.has(name)) state.strain.compare.delete(name);
  }
  tbody.innerHTML = "";
  for (const run of runs) {
    const tr = document.createElement("tr");
    tr.classList.add("selectable");
    if (run.name === state.strain.selected) tr.classList.add("selected");
    if (state.strain.compare.has(run.name)) tr.classList.add("compare");
    tr.addEventListener("click", (ev) => {
      if (ev.shiftKey) {
        if (run.name === state.strain.selected) return;
        if (state.strain.compare.has(run.name)) state.strain.compare.delete(run.name);
        else state.strain.compare.add(run.name);
      } else {
        state.strain.selected = run.name;
        state.strain.compare.clear();
      }
      redrawStrain();
    });
    const timeTd = document.createElement("td");
    timeTd.textContent = shortTime(run.mtime_utc);
    timeTd.title = `${run.name} — ${run.mtime_utc}`;
    tr.appendChild(timeTd);
    const tagTd = document.createElement("td");
    tagTd.textContent = runTag(run.name);
    tr.appendChild(tagTd);
    const actionTd = document.createElement("td");
    actionTd.className = "actions";
    const prefillBtn = document.createElement("button");
    prefillBtn.textContent = "→ console";
    prefillBtn.title = "prefill the console with this run's command";
    prefillBtn.disabled = !(state.details.get(run.name) || {}).manifest;
    prefillBtn.addEventListener("click", (e) => {
      e.stopPropagation();
      loadRerunForm(run.name);
    });
    actionTd.appendChild(prefillBtn);
    tr.appendChild(actionTd);
    tbody.appendChild(tr);
  }
}

function strainColor(t) {
  const clamped = Math.max(-1, Math.min(1, t));
  return clamped < 0
    ? mixColor(STRAIN_NEUTRAL, STRAIN_NEG, -clamped)
    : mixColor(STRAIN_NEUTRAL, STRAIN_POS, clamped);
}

function sweptEntry(line) {
  const entries = Object.entries(line.swept || {});
  return entries.length ? entries[0] : ["?", 0];
}

function strainLineLabel(line) {
  const [key, value] = sweptEntry(line);
  return `${key}=${Number(value).toFixed(0)}`;
}

/// Raster lines grouped by sweep orientation and ordered by the swept
/// coordinate, so heatmap bands lay out like the bed does.
function strainGroups(data) {
  const bySwept = (a, b) => sweptEntry(a)[1] - sweptEntry(b)[1];
  const group = (orientation, prefix, title) => ({
    orientation,
    title,
    lines: data.lines.filter((l) => l.name.startsWith(prefix)).sort(bySwept),
  });
  return [group("x", "xline", "X sweep"), group("y", "yline", "Y sweep")].filter(
    (g) => g.lines.length
  );
}

/// Bed-frame geometry: the sweep coordinate is shifted to start at 0, so
/// its bed origin is the manifest's stroke_plan start; when the plan is
/// unavailable it is recovered from the run itself — each orientation's
/// sweep starts where the OTHER orientation's raster lines sit, so their
/// minimum swept value is the origin. Band thickness is the raster pitch.
function strainBedGeometry(groups, plan) {
  const sweptOf = (orientation) => {
    const g = groups.find((x) => x.orientation === orientation);
    return g ? g.lines.map((l) => sweptEntry(l)[1]) : [];
  };
  const minGap = (vals) => {
    const s = [...vals].sort((a, b) => a - b);
    let gap = Infinity;
    for (let i = 1; i < s.length; i++) gap = Math.min(gap, s[i] - s[i - 1]);
    return isFinite(gap) && gap > 0 ? gap : STRAIN_LINE_SPACING_FALLBACK_MM;
  };
  const xBands = sweptOf("y");
  const yBands = sweptOf("x");
  const spacing = plan.line_spacing || Math.min(minGap(xBands), minGap(yBands));
  const bandHalf = spacing / 2;
  const x0 = plan.x_start != null ? plan.x_start : xBands.length ? Math.min(...xBands) : 0;
  const y0 = plan.y_start != null ? plan.y_start : yBands.length ? Math.min(...yBands) : 0;
  const xs = [];
  const ys = [];
  for (const g of groups) {
    for (const line of g.lines) {
      const half = lineBinWidth(line) / 2;
      const c = line.bin_centers;
      const swept = sweptEntry(line)[1];
      const sweepOrigin = g.orientation === "x" ? x0 : y0;
      const along = [sweepOrigin + c[0] - half, sweepOrigin + c[c.length - 1] + half];
      const across = [swept - bandHalf, swept + bandHalf];
      if (g.orientation === "x") {
        xs.push(...along);
        ys.push(...across);
      } else {
        ys.push(...along);
        xs.push(...across);
      }
    }
  }
  const xlo = Math.min(...xs);
  const ylo = Math.min(...ys);
  return {
    x0,
    y0,
    bandHalf,
    xlo,
    xhi: Math.max(Math.max(...xs), xlo + 1),
    ylo,
    yhi: Math.max(Math.max(...ys), ylo + 1),
  };
}

function strainStats(data) {
  let maxElastic = 0;
  let maxFriction = 0;
  let fricSum = 0;
  let fricN = 0;
  for (const line of data.lines) {
    for (const belt of line.belts) {
      for (const v of belt.elastic) {
        if (v !== null) maxElastic = Math.max(maxElastic, Math.abs(v));
      }
      for (const v of belt.friction) {
        if (v !== null) {
          maxFriction = Math.max(maxFriction, Math.abs(v));
          fricSum += Math.abs(v);
          fricN++;
        }
      }
    }
  }
  return { maxElastic, maxFriction, meanFriction: fricN ? fricSum / fricN : 0 };
}

function lineBinWidth(line) {
  const c = line.bin_centers;
  return c.length > 1 ? c[1] - c[0] : 2 * c[0];
}

function drawStrainHeatmap(canvas, group, beltIdx, vmax, geo) {
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
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
  const pad = STRAIN_HEAT_PAD;
  const px = (mm) => pad.l + ((mm - geo.xlo) / (geo.xhi - geo.xlo)) * (w - pad.l - pad.r);
  const py = (mm) => h - pad.b - ((mm - geo.ylo) / (geo.yhi - geo.ylo)) * (h - pad.t - pad.b);

  ctx.font = "10px monospace";
  for (const line of group.lines) {
    const swept = sweptEntry(line)[1];
    const half = lineBinWidth(line) / 2;
    const sweepOrigin = group.orientation === "x" ? geo.x0 : geo.y0;
    line.bin_centers.forEach((center, b) => {
      const v = line.belts[beltIdx][state.strain.field][b];
      if (v === null) return;
      ctx.fillStyle = strainColor(v / vmax);
      const lo = sweepOrigin + center - half;
      const hi = sweepOrigin + center + half;
      if (group.orientation === "x") {
        const top = py(swept + geo.bandHalf);
        ctx.fillRect(px(lo), top + 0.5, px(hi) - px(lo), py(swept - geo.bandHalf) - top - 1);
      } else {
        const left = px(swept - geo.bandHalf);
        ctx.fillRect(left + 0.5, py(hi), px(swept + geo.bandHalf) - left - 1, py(lo) - py(hi));
      }
    });
  }

  ctx.fillStyle = "#8a97a3";
  for (let i = 0; i <= 4; i++) {
    const xmm = geo.xlo + ((geo.xhi - geo.xlo) * i) / 4;
    ctx.fillText(xmm.toFixed(0), Math.min(px(xmm) - 6, w - 20), h - pad.b + 12);
    const ymm = geo.ylo + ((geo.yhi - geo.ylo) * i) / 4;
    ctx.fillText(ymm.toFixed(0), 2, Math.max(py(ymm) + 3, pad.t + 8));
  }
  ctx.fillText("bed x (mm)", pad.l + (w - pad.l - pad.r) / 2 - 30, h - 4);
  ctx.fillText("bed y (mm)", pad.l, 11);
}

function strainHeatmapBox(title, group, beltIdx, vmax, geo) {
  const box = document.createElement("div");
  box.className = "chart-box";
  const head = document.createElement("h3");
  head.textContent = title;
  box.appendChild(head);
  const canvas = document.createElement("canvas");
  const pad = STRAIN_HEAT_PAD;
  const plotW = STRAIN_HEAT_CANVAS_W - pad.l - pad.r;
  const plotH = plotW * ((geo.yhi - geo.ylo) / (geo.xhi - geo.xlo));
  canvas.width = STRAIN_HEAT_CANVAS_W;
  canvas.height = Math.round(pad.t + pad.b + plotH);
  box.appendChild(canvas);
  drawStrainHeatmap(canvas, group, beltIdx, vmax, geo);
  return box;
}

function strainScaleHtml(vmax) {
  const stops = [];
  for (let i = 0; i <= 8; i++) stops.push(strainColor(i / 4 - 1));
  const what =
    state.strain.field === "friction"
      ? "friction (direction-dependent) differential torque"
      : "elastic differential torque";
  return (
    `<div class="strain-scale"><span>−${vmax.toFixed(1)}%</span>` +
    `<span class="bar" style="background:linear-gradient(90deg,${stops.join(",")})"></span>` +
    `<span>+${vmax.toFixed(1)}%</span>` +
    `<span class="hint">${what}, % rated — null bins stay dark</span></div>`
  );
}

function strainProfileBox(title, beltIdx, group, vmax, geo) {
  const box = document.createElement("div");
  box.className = "chart-box";
  const head = document.createElement("h3");
  head.textContent = title;
  box.appendChild(head);
  const canvas = document.createElement("canvas");
  canvas.width = 860;
  canvas.height = 300;
  box.appendChild(canvas);
  const lines = group.lines;
  const sweepOrigin = group.orientation === "x" ? geo.x0 : geo.y0;
  const ramp = (i) =>
    mixColor(
      PALETTE[beltIdx % PALETTE.length],
      "#ffffff",
      lines.length > 1 ? (0.65 * i) / (lines.length - 1) : 0
    );
  const traces = lines.map((line, i) => ({
    t: line.bin_centers.map((c) => sweepOrigin + c),
    y: line.belts[beltIdx][state.strain.field],
    color: ramp(i),
  }));
  drawChart(
    canvas,
    traces,
    `${state.strain.field} (%) vs bed ${group.orientation}`,
    { yMin: -vmax, yMax: vmax },
    "mm"
  );
  const legend = document.createElement("div");
  legend.className = "legend";
  lines.forEach((line, i) => {
    const item = document.createElement("span");
    item.innerHTML = `<span class="swatch" style="background:${ramp(i)}"></span>${line.name}`;
    legend.appendChild(item);
  });
  box.appendChild(legend);
  return box;
}

function meanElastic(line, beltIdx) {
  const kept = line.belts[beltIdx].elastic.filter((v) => v !== null);
  if (!kept.length) return null;
  return kept.reduce((a, b) => a + b, 0) / kept.length;
}

function drawStrainDcBars(canvas, labels, values) {
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
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
  const pad = { l: 46, r: 8, t: 8, b: 40 };
  const vmax = Math.max(1e-6, ...values.filter((v) => v !== null).map(Math.abs));
  const y = (v) => pad.t + ((vmax - v) / (2 * vmax)) * (h - pad.t - pad.b);
  const slot = (w - pad.l - pad.r) / labels.length;

  ctx.font = "10px monospace";
  ctx.fillStyle = "#8a97a3";
  for (const v of [-vmax, 0, vmax]) {
    ctx.fillText(v.toFixed(1), 2, y(v) + 3);
  }
  ctx.strokeStyle = "#29313a";
  ctx.beginPath();
  ctx.moveTo(pad.l, y(0));
  ctx.lineTo(w - pad.r, y(0));
  ctx.stroke();

  values.forEach((v, i) => {
    const cx = pad.l + (i + 0.5) * slot;
    if (v !== null) {
      ctx.fillStyle = strainColor(v / vmax);
      const top = Math.min(y(0), y(v));
      ctx.fillRect(cx - slot * 0.35, top, slot * 0.7, Math.max(1, Math.abs(y(v) - y(0))));
    }
    ctx.save();
    ctx.translate(cx + 3, h - pad.b + 10);
    ctx.rotate(-Math.PI / 4);
    ctx.fillStyle = "#8a97a3";
    ctx.textAlign = "right";
    ctx.fillText(labels[i], 0, 0);
    ctx.restore();
  });
  ctx.fillStyle = "#8a97a3";
  ctx.textAlign = "left";
  ctx.fillText("mean elastic (%)", pad.l, 10);
}

function strainDcBox(title, beltIdx, lines) {
  const box = document.createElement("div");
  box.className = "chart-box";
  const head = document.createElement("h3");
  head.textContent = title;
  box.appendChild(head);
  const canvas = document.createElement("canvas");
  canvas.width = 430;
  canvas.height = 210;
  box.appendChild(canvas);
  drawStrainDcBars(
    canvas,
    lines.map(strainLineLabel),
    lines.map((line) => meanElastic(line, beltIdx))
  );
  return box;
}

/// Shared geometry for a strain map: ordered groups (sweep orientation),
/// bed-frame extents, and the belt pair names. Diffing two maps reuses the
/// selected run's geometry since both must match its signature to compare.
function strainGeometry(data) {
  const groups = strainGroups(data);
  const detail = state.details.get(state.strain.selected);
  const plan = (detail && detail.manifest && detail.manifest.stroke_plan) || {};
  const geo = strainBedGeometry(groups, plan);
  const pairs = data.lines[0].belts.map((b) => b.pair);
  return { groups, geo, pairs };
}

function renderStrainCharts(data) {
  const heatmaps = el("strain-heatmaps");
  const profiles = el("strain-profiles");
  const dc = el("strain-dc");
  const summary = el("strain-summary");
  heatmaps.innerHTML = "";
  profiles.innerHTML = "";
  dc.innerHTML = "";
  if (!data.lines.length) {
    summary.textContent = "run has no lines";
    el("strain-scale").innerHTML = "";
    return;
  }
  const stats = strainStats(data);
  const vmax = Math.max(
    1e-6,
    state.strain.field === "friction" ? stats.maxFriction : stats.maxElastic
  );
  summary.textContent =
    `max |elastic| ${stats.maxElastic.toFixed(1)}% · ` +
    `mean |friction| ${stats.meanFriction.toFixed(1)}%`;
  document.querySelectorAll("button.strain-field-btn").forEach((btn) => {
    btn.disabled = btn.dataset.field === state.strain.field;
  });
  el("strain-scale").innerHTML = strainScaleHtml(vmax);
  const { groups, geo, pairs } = strainGeometry(data);
  pairs.forEach((pair, beltIdx) => {
    for (const group of groups) {
      const title = `${pair} — ${group.title}`;
      heatmaps.appendChild(strainHeatmapBox(title, group, beltIdx, vmax, geo));
      profiles.appendChild(strainProfileBox(title, beltIdx, group, vmax, geo));
      dc.appendChild(strainDcBox(title, beltIdx, group.lines));
    }
  });
}

/// Appends a diverging diff heatmap (selected − compare) per belt×orientation
/// for every compare run whose strain signature matches the selected run's.
/// A mismatched run is named in the summary instead of drawn — subtracting
/// cells only means something when both maps bin the bed identically.
async function renderStrainDiffs(selectedData) {
  const heatmaps = el("strain-heatmaps");
  if (!heatmaps || !state.strain.compare.size) return;
  if (!selectedData || !selectedData.lines.length) return;
  const baseName = state.strain.selected;
  const base = strainGeometry(selectedData);
  const baseSig = strainSignature(selectedData);
  const baseTag = runTag(baseName);
  const field = state.strain.field;
  const skips = [];
  for (const name of [...state.strain.compare]) {
    let cmp;
    try {
      cmp = await ensureStrain(name);
    } catch (e) {
      skips.push(`${runTag(name)}: ${String(e)}`);
      continue;
    }
    if (state.strain.selected !== baseName || !el("strain-heatmaps")) return;
    if (!cmp || !cmp.lines.length || strainSignature(cmp) !== baseSig) {
      skips.push(`${runTag(name)}: dimensions differ`);
      continue;
    }
    const cmpGroups = strainGroups(cmp);
    const aligned = [];
    let mismatch = false;
    for (const bg of base.groups) {
      const cg = cmpGroups.find((g) => g.orientation === bg.orientation);
      if (!cg || cg.lines.length !== bg.lines.length) {
        mismatch = true;
        break;
      }
      aligned.push(cg);
    }
    if (mismatch || aligned.length !== base.groups.length || state.strain.selected !== baseName) {
      skips.push(`${runTag(name)}: layout mismatch`);
      continue;
    }
    const cmpTag = runTag(name);
    let vmaxDiff = 1e-6;
    for (let gi = 0; gi < base.groups.length; gi++) {
      const bg = base.groups[gi];
      const cg = aligned[gi];
      for (let li = 0; li < bg.lines.length; li++) {
        for (let bi = 0; bi < bg.lines[li].belts.length; bi++) {
          for (const v of pointwiseDiff(bg.lines[li].belts[bi][field], cg.lines[li].belts[bi][field])) {
            if (v !== null) vmaxDiff = Math.max(vmaxDiff, Math.abs(v));
          }
        }
      }
    }
    base.pairs.forEach((pair, beltIdx) => {
      if (state.strain.selected !== baseName || !el("strain-heatmaps")) return;
      for (let gi = 0; gi < base.groups.length; gi++) {
        const bg = base.groups[gi];
        const cg = aligned[gi];
        const diffLines = bg.lines.map((bl, li) => buildDiffLine(bl, cg.lines[li]));
        const diffGroup = { orientation: bg.orientation, title: bg.title, lines: diffLines };
        const title = `Δ ${baseTag} − ${cmpTag} · ${pair} — ${bg.title}`;
        heatmaps.appendChild(strainHeatmapBox(title, diffGroup, beltIdx, vmaxDiff, base.geo));
      }
    });
    if (state.strain.selected === baseName && el("strain-heatmaps")) {
      const scaleEl = document.createElement("div");
      scaleEl.className = "strain-scale";
      scaleEl.style.gridColumn = "1 / -1";
      const stops = [];
      for (let i = 0; i <= 8; i++) stops.push(strainColor(i / 4 - 1));
      scaleEl.innerHTML =
        `<span>−${vmaxDiff.toFixed(1)}%</span>` +
        `<span class="bar" style="background:linear-gradient(90deg,${stops.join(",")})"></span>` +
        `<span>+${vmaxDiff.toFixed(1)}%</span>` +
        `<span class="hint">Δ scale for ${cmpTag} — ${field}, null bins stay dark</span>`;
      heatmaps.appendChild(scaleEl);
    }
  }
  if (skips.length) {
    const note = el("strain-summary");
    if (note) note.textContent += `  · skipped: ${skips.join("; ")}`;
  }
}

async function redrawStrain() {
  renderStrainRuns();
  if (!el("strain-heatmaps")) return;
  const summary = el("strain-summary");
  const name = state.strain.selected;
  if (!name) {
    summary.textContent = "no strain_map runs yet — run SERVO_STRAIN_MAP first";
    el("strain-heatmaps").innerHTML = "";
    el("strain-scale").innerHTML = "";
    el("strain-profiles").innerHTML = "";
    el("strain-dc").innerHTML = "";
    return;
  }
  let data;
  try {
    data = await ensureStrain(name);
  } catch (e) {
    summary.textContent = String(e);
    return;
  }
  if (state.strain.selected !== name || !el("strain-heatmaps")) return;
  renderStrainCharts(data);
  await renderStrainDiffs(data);
}

// --- PSD peak list (gains page) ---------------------------------------------

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
  if (def.strain) {
    await redrawStrain();
    return;
  }
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
  if (def.metrics || def.sweepChart) {
    const onPage = new Set(pageRuns(def).map((r) => r.name));
    const pageNames = okNames.filter((n) => onPage.has(n));
    if (def.metrics) renderMetricsTable(pageNames, steps);
    if (def.sweepChart) renderSweepMetricsChart(pageNames);
  }
  renderStepChips(stepNames);
  if (def.charts && def.charts.includes("frf")) renderFrfCharts(okNames, plots);
  if (def.charts && def.charts.includes("psd")) {
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

/// Apply sends the previewed SERVO_TUNE batch, then reloads
/// drive_state.json — SERVO_TUNE readback-verifies each write and patches
/// the file in place, so re-reading it is enough; the full
/// SERVO_DUMP_TUNING drive re-read stays behind the refresh button.
async function applyDriveChanges() {
  const changed = diffChangedParams(state.drive.data.params, state.drive.data.motors, state.drive.pending);
  const lines = buildServoTuneCommands(changed);
  if (!lines.length) return;
  await runGcode(lines, "apply");
  await loadDriveState();
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

/// Old manifests predate the recorded `command` field; rebuilding from the
/// manifest is a lossy fallback that only knows the parameters listed here.
function reconstructCommand(manifest) {
  if (manifest.command) return manifest.command;
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
    case "strain_map": {
      const plan = manifest.stroke_plan;
      return (
        `SERVO_MEASURE_STRAIN_MAP LINE_SPACING=${plan.line_spacing} SPEED=${plan.speed} ` +
        `ACCEL=${plan.accel} X_START=${plan.x_start} X_END=${plan.x_end} ` +
        `Y_START=${plan.y_start} Y_END=${plan.y_end} DWELL_MS=${plan.dwell_ms} ` +
        `${plan.zero_sync ? "" : "SYNC=0 "}TAG=${tag}`
      );
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
    const ks = info.klippy_state || "unknown";
    if (ks === "ready" && state.help.klippyState !== "ready") fetchMacroHelp();
    state.help.klippyState = ks;
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
        const responses = ((entry.responses && entry.responses[i]) || [])
          .map((m) => {
            const cls = m.startsWith("!!") ? "resp-line resp-err" : "resp-line";
            return `<div class="${cls}">${escapeHtml(m)}</div>`;
          })
          .join("");
        return (
          `<div class="sent-line" data-line="${escapeHtml(l)}" ` +
          `title="click to insert into the console">${escapeHtml(l)}${suffix}</div>${responses}`
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

/// Timestamps in Moonraker's gcode store are server clock, so diffing
/// against its own latest entry needs no client/server clock agreement.
async function latestGcodeStoreTime(base) {
  const resp = await fetch(`${base}/server/gcode_store?count=1`);
  if (!resp.ok) throw new Error(`gcode_store HTTP ${resp.status}`);
  const store = (await resp.json()).result.gcode_store;
  return store.length ? store[store.length - 1].time : 0;
}

async function fetchGcodeResponses(base, sinceTime) {
  const resp = await fetch(`${base}/server/gcode_store?count=500`);
  if (!resp.ok) throw new Error(`gcode_store HTTP ${resp.status}`);
  const store = (await resp.json()).result.gcode_store;
  return store
    .filter((e) => e.type === "response" && e.time > sinceTime)
    .map((e) => e.message);
}

/// Sends `lines` (already-built gcode) through the shared Moonraker
/// plumbing — the grid's Apply and the console land in the same session
/// log, which survives page switches. `/printer/gcode/script` blocks
/// until the command finishes, and klippy's respond_info output only
/// travels the websocket — so each line's responses are harvested from
/// `/server/gcode_store` afterwards and echoed under the sent line.
async function runGcode(lines, label) {
  const base = moonrakerUrl();
  const statusEl = el("run-status");
  if (statusEl) statusEl.textContent = "";
  const entry = { time: new Date().toISOString(), label, lines: [], results: [], responses: [] };
  state.sentLog.push(entry);
  for (const line of lines) {
    const url = `${base}/printer/gcode/script?script=${encodeURIComponent(line)}`;
    entry.lines.push(line);
    let sentAt = null;
    try {
      sentAt = await latestGcodeStoreTime(base);
    } catch (e) {
      console.error(e);
    }
    let ok = false;
    try {
      const resp = await fetch(url, { method: "POST" });
      const text = await resp.text();
      if (!resp.ok && statusEl) {
        statusEl.innerHTML += `<div class="status-err">${line} -> HTTP ${resp.status} ${text.slice(0, 200)}</div>`;
      }
      ok = resp.ok;
      entry.results.push({ ok: resp.ok, status: resp.status });
    } catch (e) {
      if (statusEl) statusEl.innerHTML += `<div class="status-err">${line} -> ${e}</div>`;
      entry.results.push({ ok: false, status: 0 });
    }
    let responses = [];
    if (sentAt !== null) {
      try {
        responses = await fetchGcodeResponses(base, sentAt);
      } catch (e) {
        console.error(e);
      }
    }
    entry.responses.push(responses);
    renderSentLog();
    if (!ok) break;
  }
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
    renderConsoleHelp();
  });
  input.addEventListener("keyup", renderConsoleHelp);
  input.addEventListener("click", renderConsoleHelp);
  input.addEventListener("keydown", consoleKeydown);
  input.addEventListener("blur", () => exitConsoleSearch(true));
  renderConsoleHelp();
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
  renderConsoleHelp();
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
  if (ev.key === "Tab" && !ev.shiftKey && !ev.ctrlKey && !ev.altKey) {
    ev.preventDefault();
    consoleTabComplete(input);
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

// --- macro docs -----------------------------------------------------------------

/// The macro documentation IS klippy's cmd_*_help strings, fetched from the
/// running instance over Moonraker's /printer/gcode/help — so it can never
/// drift from the code that executes the command. localStorage keeps the
/// last good copy readable while klippy is down, which is exactly when a
/// failed run sends you looking for the docs.
async function fetchMacroHelp() {
  const h = state.help;
  if (h.pending) return;
  h.pending = true;
  try {
    const resp = await fetch(`${moonrakerUrl()}/printer/gcode/help`);
    if (!resp.ok) throw new Error(`gcode/help HTTP ${resp.status}`);
    const all = (await resp.json()).result;
    const commands = {};
    for (const [name, text] of Object.entries(all)) {
      if (name.startsWith("SERVO_")) commands[name] = text;
    }
    h.commands = commands;
    h.fetchedUtc = new Date().toISOString();
    h.cached = false;
    h.error = null;
    localStorage.setItem(
      HELP_CACHE_KEY,
      JSON.stringify({ fetched_utc: h.fetchedUtc, commands })
    );
  } catch (e) {
    h.error = String(e);
    if (!h.commands) loadCachedMacroHelp();
  } finally {
    h.pending = false;
  }
  renderDocsList();
  renderConsoleHelp();
}

/// The catch only forgives corrupt localStorage JSON, same contract as
/// loadConsoleHistory.
function loadCachedMacroHelp() {
  const raw = localStorage.getItem(HELP_CACHE_KEY);
  if (!raw) return;
  let parsed;
  try {
    parsed = JSON.parse(raw);
  } catch (e) {
    return;
  }
  if (!parsed || typeof parsed.commands !== "object" || parsed.commands === null) return;
  state.help.commands = parsed.commands;
  state.help.fetchedUtc = parsed.fetched_utc || null;
  state.help.cached = true;
}

/// Every cmd_*_help string ends in a "Params NAME (default) ..." tail — the
/// one convention this rendering leans on. A string without the marker just
/// renders as prose.
function splitMacroHelp(text) {
  const m = /\bParams\b/.exec(text);
  if (!m) return { prose: text.trim(), params: null };
  return {
    prose: text.slice(0, m.index).trim(),
    params: text.slice(m.index + m[0].length).trim(),
  };
}

/// Tokenizes a Params tail into param chips and plain-text runs. UPPERCASE
/// words are params (an optional =A|B suffix lists choices), a following
/// (...) group is that param's default, anything else — "as
/// SERVO_MEASURE_INERTIA plus" — stays literal text.
function parseParamsTail(tail) {
  const items = [];
  const tokens = tail.split(/\s+/).filter((t) => t.length);
  let i = 0;
  while (i < tokens.length) {
    const tok = tokens[i];
    const clean = tok.replace(/[.,;]$/, "");
    const eq = clean.indexOf("=");
    const name = eq < 0 ? clean : clean.slice(0, eq);
    if (/^[A-Z][A-Z0-9_]*$/.test(name)) {
      items.push({
        kind: "param",
        name,
        choices: eq < 0 ? null : clean.slice(eq + 1),
        dflt: null,
      });
      i++;
      continue;
    }
    if (tok.startsWith("(")) {
      let group = tok;
      while (!group.endsWith(")") && i + 1 < tokens.length) {
        i++;
        group += ` ${tokens[i]}`;
      }
      i++;
      const dflt = group.replace(/^\(/, "").replace(/\)$/, "");
      const last = items[items.length - 1];
      if (last && last.kind === "param" && last.dflt === null) last.dflt = dflt;
      else items.push({ kind: "text", text: group });
      continue;
    }
    const last = items[items.length - 1];
    if (last && last.kind === "text") last.text += ` ${tok}`;
    else items.push({ kind: "text", text: tok });
    i++;
  }
  return items;
}

function paramChipsHtml(items) {
  const known = state.help.commands || {};
  return items
    .map((it) => {
      if (it.kind === "text") {
        return `<span class="param-text">${escapeHtml(it.text)}</span>`;
      }
      let label = escapeHtml(it.name);
      if (it.choices) label += `<span class="param-extra">=${escapeHtml(it.choices)}</span>`;
      if (it.dflt) label += ` <span class="param-extra">(${escapeHtml(it.dflt)})</span>`;
      if (known[it.name]) {
        return `<a class="chip param-chip xref" href="#/docs/${it.name}">${label}</a>`;
      }
      return `<span class="chip param-chip">${label}</span>`;
    })
    .join("");
}

function docsShellHtml() {
  return (
    `<div class="workspace single">` +
    `<main class="analysis">` +
    `<section class="docs-section">` +
    `<div class="section-head"><h2>calibration macros</h2>` +
    `<span class="note" id="docs-status"></span></div>` +
    `<div id="docs-list"></div>` +
    `</section>` +
    consoleSectionHtml({}) +
    `</main></div>`
  );
}

function docsDeepLinkTarget() {
  const m = /^#\/docs\/([A-Za-z0-9_]+)/.exec(location.hash || "");
  return m ? m[1].toUpperCase() : null;
}

function firstSentence(prose) {
  const cut = prose.indexOf(". ");
  return cut < 0 ? prose : prose.slice(0, cut + 1);
}

function macroDocHtml(name, text, open) {
  const { prose, params } = splitMacroHelp(text);
  const items = params ? parseParamsTail(params) : [];
  return (
    `<details class="macro-doc" id="doc-${escapeHtml(name)}"${open ? " open" : ""}>` +
    `<summary><span class="macro-name">${escapeHtml(name)}</span>` +
    `<span class="hint" title="${escapeHtml(firstSentence(prose))}">` +
    `${escapeHtml(firstSentence(prose))}</span></summary>` +
    `<div class="macro-body">` +
    `<p class="macro-prose">${escapeHtml(prose)}</p>` +
    (items.length
      ? `<div class="chips param-chips">${paramChipsHtml(items)}</div>`
      : "") +
    `</div></details>`
  );
}

function renderDocsList() {
  const list = el("docs-list");
  if (!list) return;
  const h = state.help;
  const status = el("docs-status");
  if (status) {
    if (h.commands && !h.cached) {
      status.textContent =
        `the running klippy's cmd_*_help strings, fetched ${shortTime(h.fetchedUtc)}`;
    } else if (h.commands) {
      status.innerHTML =
        `cached copy${h.fetchedUtc ? ` from ${shortTime(h.fetchedUtc)}` : ""} — ` +
        `klippy unreachable <button id="docs-retry">retry</button>`;
    } else if (h.pending) {
      status.textContent = "fetching from klippy…";
    } else {
      status.innerHTML =
        `${escapeHtml(h.error || "not fetched yet")} <button id="docs-retry">retry</button>`;
    }
  }
  if (!h.commands) {
    list.innerHTML = `<p class="note">no macro help yet — is klippy up and the moonraker URL right?</p>`;
  } else {
    const target = docsDeepLinkTarget();
    const openNow = new Set(
      Array.from(list.querySelectorAll("details.macro-doc[open]")).map((d) =>
        d.id.slice("doc-".length)
      )
    );
    const firstRender = !list.dataset.rendered;
    list.innerHTML = Object.entries(h.commands)
      .map(([name, text]) =>
        macroDocHtml(name, text, name === target || openNow.has(name))
      )
      .join("");
    list.dataset.rendered = "1";
    if (firstRender && target && h.commands[target]) {
      const entry = el(`doc-${target}`);
      if (entry) entry.scrollIntoView({ block: "start" });
    }
  }
  const retry = el("docs-retry");
  if (retry) retry.addEventListener("click", fetchMacroHelp);
}

function consoleCaretLine(input) {
  const caret = input.selectionStart;
  const text = input.value;
  const start = text.lastIndexOf("\n", caret - 1) + 1;
  let end = text.indexOf("\n", caret);
  if (end < 0) end = text.length;
  return { line: text.slice(start, end), start, caretInLine: caret - start };
}

function lineCommand(line) {
  return (line.trim().split(/\s+/)[0] || "").toUpperCase();
}

function macroParamNames(cmdName) {
  const known = state.help.commands || {};
  const text = known[cmdName];
  if (!text) return null;
  const { params } = splitMacroHelp(text);
  if (!params) return [];
  return parseParamsTail(params)
    .filter((it) => it.kind === "param" && !known[it.name])
    .map((it) => it.name);
}

/// What tab completion would complete at the current caret: SERVO_* command
/// names for the line's first word, otherwise the command's param names not
/// already given on the line. A token with "=" is a value — nothing to
/// complete there.
function consoleCompletion(input) {
  const none = { candidates: [] };
  const h = state.help;
  if (!h.commands) return none;
  const { line, start, caretInLine } = consoleCaretLine(input);
  const tokenStart = line.lastIndexOf(" ", caretInLine - 1) + 1;
  const token = line.slice(tokenStart, caretInLine);
  if (token.includes("=")) return none;
  const up = token.toUpperCase();
  const common = { lineStart: start, tokenStart, tokenLen: token.length };
  if (!line.slice(0, tokenStart).trim().length) {
    if (!up.length) return none;
    return {
      ...common,
      candidates: Object.keys(h.commands).filter((n) => n.startsWith(up)),
      suffix: " ",
    };
  }
  const names = macroParamNames(lineCommand(line));
  if (!names) return none;
  const taken = new Set(
    Array.from(line.matchAll(/([A-Za-z][A-Za-z0-9_]*)=/g), (m) => m[1].toUpperCase())
  );
  return {
    ...common,
    candidates: names.filter((n) => n.startsWith(up) && !taken.has(n)),
    suffix: "=",
  };
}

function longestCommonPrefix(names) {
  let prefix = names[0];
  for (const n of names.slice(1)) {
    while (!n.startsWith(prefix)) prefix = prefix.slice(0, -1);
  }
  return prefix;
}

function consoleTabComplete(input) {
  const c = consoleCompletion(input);
  if (!c.candidates.length) return;
  const replacement =
    c.candidates.length === 1
      ? c.candidates[0] + c.suffix
      : longestCommonPrefix(c.candidates);
  const from = c.lineStart + c.tokenStart;
  const text = input.value;
  setConsoleValue(
    text.slice(0, from) + replacement + text.slice(from + c.tokenLen),
    true
  );
  input.selectionStart = input.selectionEnd = from + replacement.length;
  renderConsoleHelp();
}

/// The terminal-style help under the prompt: one dim description line and a
/// usage line of the command's params. The param whose value the caret is in
/// is highlighted; while a param name is being typed, every candidate the
/// prefix still matches is highlighted.
function renderConsoleHelp() {
  const box = el("console-help");
  const input = el("console-input");
  if (!box || !input) return;
  const { line, caretInLine } = consoleCaretLine(input);
  const first = lineCommand(line);
  if (!first.startsWith("SERVO")) {
    box.innerHTML = "";
    return;
  }
  const h = state.help;
  if (!h.commands) {
    if (!h.pending && !h.error) fetchMacroHelp();
    box.innerHTML = `<div class="hint">${
      h.error ? `macro help unavailable — ${escapeHtml(h.error)}` : "fetching macro help…"
    }</div>`;
    return;
  }
  const helpText = h.commands[first];
  if (!helpText) {
    const matches = Object.keys(h.commands).filter((n) => n.startsWith(first));
    box.innerHTML = matches.length
      ? `<div class="console-help-cands">${matches.map(escapeHtml).join("  ")}</div>`
      : "";
    return;
  }
  const tokenStart = line.lastIndexOf(" ", caretInLine - 1) + 1;
  let tokenEnd = line.indexOf(" ", caretInLine);
  if (tokenEnd < 0) tokenEnd = line.length;
  const caretToken = line.slice(tokenStart, tokenEnd);
  const onFirstWord = !line.slice(0, tokenStart).trim().length;
  const activeName = !onFirstWord && caretToken.includes("=")
    ? caretToken.split("=")[0].toUpperCase()
    : null;
  const typedPrefix = !onFirstWord && !caretToken.includes("=")
    ? line.slice(tokenStart, caretInLine).toUpperCase()
    : "";
  const { prose, params } = splitMacroHelp(helpText);
  const items = params ? parseParamsTail(params) : [];
  const usage = items
    .map((it) => {
      if (it.kind === "text") return `<span class="dim">${escapeHtml(it.text)}</span>`;
      let cls = "p";
      if (it.name === activeName) cls += " active";
      else if (typedPrefix.length && it.name.startsWith(typedPrefix)) cls += " match";
      let s = `<span class="${cls}">${escapeHtml(it.name)}`;
      if (it.choices) s += `<span class="dim">=${escapeHtml(it.choices)}</span>`;
      if (it.dflt) s += `<span class="dim">(${escapeHtml(it.dflt)})</span>`;
      return `${s}</span>`;
    })
    .join(" ");
  box.innerHTML =
    `<div class="console-help-desc"><a href="#/docs/${first}" ` +
    `title="open in the docs tab">${first}</a>` +
    `<span class="dim"> — ${escapeHtml(prose)}</span>` +
    (h.cached ? `<span class="hint"> (cached — klippy unreachable)</span>` : "") +
    `</div>` +
    (usage ? `<div class="console-help-usage">${usage}</div>` : "");
}

// --- boot -------------------------------------------------------------------

function initShell() {
  el("estop-btn").addEventListener("click", emergencyStop);
  const input = el("moonraker-url");
  input.value = localStorage.getItem(MOONRAKER_KEY) || `http://${location.hostname}:7125`;
  input.addEventListener("change", () => {
    localStorage.setItem(MOONRAKER_KEY, input.value);
    pollMoonrakerHealth();
    fetchMacroHelp();
  });
  loadCachedMacroHelp();
  fetchMacroHelp();
  pollMoonrakerHealth();
  setInterval(pollMoonrakerHealth, MOONRAKER_HEALTH_POLL_MS);
  bindAccordionToggle();
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
