import { el, mustEl, resetRenderState } from "./api";
import { psdMaxFreqHz } from "./charts-core";
import { bindConsole, setConsoleValue } from "./console";
import { fetchMacroHelp, docsShellHtml, renderDocsList } from "./docs";
import { renderDriveGroups } from "./drive";
import { bindLaunchpad, launchpadShellHtml } from "./launchpad";
import { bindLiveEvents, startLivePolling, stopLivePolling } from "./live";
import { renderSentLog } from "./moonraker";
import { redrawCharts } from "./peaks";
import { renderRuns } from "./runs";
import { PSD_MAX_FREQ_KEY, MOTOR_VIEW_KEY, PSD_MAX_FREQ_CHOICES_HZ, PAGE_DEFS, DEFAULT_PAGE, state } from "./state";
import type { PageDef } from "./state";
import { strainShellHtml, redrawStrain } from "./strain";

// --- page shell ---------------------------------------------------------------

function currentPageDef(): PageDef {
  return PAGE_DEFS[state.page] || PAGE_DEFS[DEFAULT_PAGE];
}

function pageFromHash() {
  const m = /^#\/?([a-z]+)/.exec(location.hash || "");
  return m && PAGE_DEFS[m[1]] ? m[1] : DEFAULT_PAGE;
}

function renderTabs() {
  const nav = mustEl("page-tabs");
  nav.innerHTML = Object.entries(PAGE_DEFS)
    .map(
      ([key, def]) =>
        `<a href="#/${key}" class="tab${key === state.page ? " active" : ""}">${def.label}</a>`
    )
    .join("");
}

function controlsSectionsHtml(def: PageDef): string {
  const parts: string[] = [];
  if (def.groups) {
    parts.push(
      `<section class="panel">` +
        `<div class="section-head"><h2>drive tuning</h2></div>` +
        `<div id="drive-panel"></div>` +
        `</section>`
    );
  }
  parts.push(consoleSectionHtml(def));
  return parts.join("");
}

function consoleSectionHtml(def: Partial<PageDef>): string {
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
function motorViewEffective(withAvg: boolean): string {
  const view = motorView();
  return !withAvg && view === "avg" ? "agg" : view;
}

function motorViewToggleHtml(aggLabel: string, withAvg = false): string {
  const effective = motorViewEffective(withAvg);
  const chip = (v: string, label: string) =>
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
  document.querySelectorAll<HTMLElement>(".motor-view-chips").forEach((group) => {
    const effective = motorViewEffective(group.classList.contains("with-avg"));
    group.querySelectorAll<HTMLElement>(".motor-view-btn").forEach((b) => {
      b.classList.toggle("active", b.dataset.view === effective);
    });
  });
}

function sectionHeadHtml(title: string, toolsHtml: string | null): string {
  return (
    `<div class="section-head"><h2>${title}</h2></div>` +
    (toolsHtml ? `<div class="section-tools">${toolsHtml}</div>` : "")
  );
}

function analysisSectionsHtml(def: PageDef): string {
  const parts: string[] = [];
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
  if (def.charts && def.charts.includes("path")) {
    parts.push(
      `<section class="path-section" id="path-section" hidden>` +
        sectionHeadHtml(
          "toolpath — commanded vs actual",
          `<button id="path-fit">fit</button>` +
            `<span class="note" id="path-note"></span>`
        ) +
        `<div class="spatial-box"><canvas id="path-canvas"></canvas></div>` +
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
  if (def.charts && def.charts.includes("ringdown")) {
    parts.push(
      `<section class="ringdown-section" id="ringdown-section" hidden>` +
        sectionHeadHtml(
          "ring-down after stop",
          `<span class="note" id="ringdown-meta"></span>`
        ) +
        `<div class="charts" id="ringdown-charts"></div>` +
        `<div id="ringdown-modes"></div>` +
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
          motorViewToggleHtml("combined") +
            `<div class="chips" id="time-motor-chips"></div>` +
            `<div class="chips" id="time-step-chips"></div>`
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
      "live toolpath — commanded vs actual",
      `<button id="live-freeze-btn" title="space toggles">freeze</button>` +
        `<span class="note live-timing-bad" id="live-freeze-badge"></span>` +
        `<button id="live-spatial-fit">fit</button>` +
        `<span class="note" id="live-spatial-note">waiting for the tap…</span>`
    ) +
    `<div class="spatial-box"><canvas id="live-spatial-canvas"></canvas></div>` +
    `</section>` +
    `<section class="live-section">` +
    sectionHeadHtml(
      "live following error — per motor",
      `<span class="chips live-unit-chips">` +
        `<button class="chip" id="live-unit-um" data-unit="µm">µm</button>` +
        `<button class="chip" id="live-unit-counts" data-unit="counts">counts</button>` +
        `</span>` +
        `<span class="note" id="live-unit-hint"></span>` +
        `<label class="live-window">window ` +
        `<input type="range" id="live-window" min="1" max="30" step="1" value="${state.live.windowS}">` +
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

function loadCollapsedSections(): Set<string> {
  try {
    return new Set(JSON.parse(localStorage.getItem(ACCORDION_KEY) || "[]"));
  } catch {
    return new Set();
  }
}

function sectionLabel(head: HTMLElement): string {
  const h = head.querySelector("h2");
  return `${state.page}::${(h ? h.textContent : head.textContent)?.trim() ?? ""}`;
}

function applyAccordionState() {
  const collapsed = loadCollapsedSections();
  document.querySelectorAll<HTMLElement>("#page-root .analysis .section-head").forEach((head) => {
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
    const head = (e.target as HTMLElement).closest<HTMLElement>(".analysis .section-head");
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
  resetRenderState();
  renderTabs();
  const def = currentPageDef();
  const root = mustEl("page-root");
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
    document.querySelectorAll<HTMLButtonElement>("button.strain-field-btn").forEach((btn) => {
      btn.addEventListener("click", () => {
        state.strain.field = btn.dataset.field === "friction" ? "friction" : "elastic";
        redrawStrain();
      });
    });
    renderSentLog();
    redrawStrain();
    applyAccordionState();
    return;
  }
  if (def.launchpad) {
    root.innerHTML = launchpadShellHtml();
    bindPageEvents();
    bindLaunchpad();
    renderSentLog();
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
function makeColumnsResizable(table: HTMLTableElement) {
  const ths = [...table.querySelectorAll<HTMLTableCellElement>("thead th")];
  const freezeLayout = () => {
    if (table.style.tableLayout === "fixed") return;
    for (const th of ths) th.style.width = `${th.offsetWidth}px`;
    table.style.tableLayout = "fixed";
  };
  ths.forEach((th) => {
    const grip = document.createElement("span");
    grip.className = "col-resizer";
    th.appendChild(grip);
    grip.addEventListener("mousedown", (e: MouseEvent) => {
      e.preventDefault();
      e.stopPropagation();
      freezeLayout();
      const startX = e.pageX;
      const startW = th.offsetWidth;
      const onMove = (ev: MouseEvent) => {
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
    .querySelectorAll<HTMLTableElement>(".runs-wrap table, .journal-wrap table")
    .forEach(makeColumnsResizable);
  const psdMax = el<HTMLSelectElement>("psd-max-freq");
  if (psdMax) {
    psdMax.addEventListener("change", () => {
      localStorage.setItem(PSD_MAX_FREQ_KEY, psdMax.value);
      redrawCharts();
    });
  }
  document.querySelectorAll<HTMLButtonElement>("button.motor-view-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      localStorage.setItem(MOTOR_VIEW_KEY, btn.dataset.view ?? "agg");
      syncMotorViewChips();
      redrawCharts();
    });
  });
  const def = currentPageDef();
  document.querySelectorAll<HTMLButtonElement>("button.template-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      const t = (def.templates || [])[Number(btn.dataset.template)];
      if (t) {
        setConsoleValue(t.command, true);
        const label = el("form-run-name");
        if (label) label.textContent = "template — edit values before running";
      }
    });
  });
}

export { currentPageDef, pageFromHash, renderTabs, controlsSectionsHtml, consoleSectionHtml, motorView, motorViewPerMotor, motorViewEffective, motorViewToggleHtml, syncMotorViewChips, sectionHeadHtml, analysisSectionsHtml, liveShellHtml, ACCORDION_KEY, loadCollapsedSections, sectionLabel, applyAccordionState, bindAccordionToggle, renderPage, makeColumnsResizable, bindPageEvents };
