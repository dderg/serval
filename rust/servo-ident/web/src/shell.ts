import { html } from "htm/preact";
import { useEffect, useRef, useState } from "preact/hooks";
import { el } from "./api";
import { bindFerrUnitToggle } from "./charts-core";
import { ConsolePanel } from "./console";
import { fetchMacroHelp, loadCachedMacroHelp, DocsPage } from "./docs";
import { renderDriveBanner } from "./drive";
import { LaunchpadPad } from "./launchpad";
import { pollRtHealth, LivePage } from "./live";
import { pollMoonrakerHealth, emergencyStop } from "./moonraker";
import { redrawCharts } from "./peaks";
import { TunePage, JournalPage } from "./runs";
import { MOONRAKER_KEY, MOONRAKER_HEALTH_POLL_MS, RT_HEALTH_POLL_MS, PSD_MAX_FREQ_KEY, MOTOR_VIEW_KEY, PAGE_DEFS, DEFAULT_PAGE, state } from "./state";
import type { PageDef } from "./state";
import { StrainPage } from "./strain";

// --- page shell ---------------------------------------------------------------

function currentPageDef(): PageDef {
  return PAGE_DEFS[state.page] || PAGE_DEFS[DEFAULT_PAGE];
}

function pageFromHash() {
  const m = /^#\/?([a-z]+)/.exec(location.hash || "");
  return m && PAGE_DEFS[m[1]] ? m[1] : DEFAULT_PAGE;
}



/// The charts that fold drives into one trace (avg PSD, worst-drive sweep
/// metrics, combined time domain) all obey this one switch; per-motor
/// expands them into a trace per drive, and "avg" (where offered) shows
/// the mean over drives instead of the worst.
function motorView() {
  const v = localStorage.getItem(MOTOR_VIEW_KEY);
  return v === "per-motor" || v === "avg" || v === "cartesian" ? v : "agg";
}

function motorViewPerMotor() {
  return motorView() === "per-motor";
}

/// Sections whose aggregate is already an average (PSD, combined time
/// domain) don't offer a separate "avg" chip; there, the stored "avg"
/// view lights up the aggregate chip. Likewise "cartesian" exists only
/// where a section offers it (the PSD chart) and reads as the aggregate
/// everywhere else.
function motorViewEffective(withAvg: boolean, withCartesian = false): string {
  const view = motorView();
  if (view === "avg" && !withAvg) return "agg";
  if (view === "cartesian" && !withCartesian) return "agg";
  return view;
}

function motorViewToggleHtml(aggLabel: string, withAvg = false, withCartesian = false): string {
  const effective = motorViewEffective(withAvg, withCartesian);
  const chip = (v: string, label: string) =>
    `<button class="chip motor-view-btn${effective === v ? " active" : ""}" data-view="${v}">${label}</button>`;
  return (
    `<span class="chips motor-view-chips${withAvg ? " with-avg" : ""}${withCartesian ? " with-cartesian" : ""}">` +
    chip("agg", aggLabel) +
    (withAvg ? chip("avg", "avg") : "") +
    chip("per-motor", "per-motor") +
    (withCartesian ? chip("cartesian", "cartesian") : "") +
    `</span>`
  );
}

function syncMotorViewChips() {
  document.querySelectorAll<HTMLElement>(".motor-view-chips").forEach((group) => {
    const effective = motorViewEffective(
      group.classList.contains("with-avg"),
      group.classList.contains("with-cartesian")
    );
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

function bindAnalysisControls() {
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
  bindFerrUnitToggle("psd", redrawCharts);
  bindFerrUnitToggle("time", redrawCharts);
  document.querySelectorAll<HTMLButtonElement>("button.motor-view-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      localStorage.setItem(MOTOR_VIEW_KEY, btn.dataset.view ?? "agg");
      syncMotorViewChips();
      redrawCharts();
    });
  });
}

function useRoute(): string {
  const [page, setPage] = useState(pageFromHash);
  useEffect(() => {
    const onHash = () => setPage(pageFromHash());
    window.addEventListener("hashchange", onHash);
    return () => window.removeEventListener("hashchange", onHash);
  }, []);
  return page;
}

function Tabs() {
  const page = useRoute();
  return html`<nav class="tabs" id="page-tabs">
    ${Object.entries(PAGE_DEFS).map(
      ([key, def]) =>
        html`<a key=${key} href=${`#/${key}`} class=${key === page ? "tab active" : "tab"}>${def.label}</a>`
    )}
  </nav>`;
}

function Topbar() {
  const urlRef = useRef<HTMLInputElement>(null);
  useEffect(() => {
    const input = urlRef.current;
    if (input) input.value = localStorage.getItem(MOONRAKER_KEY) || `http://${location.hostname}:7125`;
    loadCachedMacroHelp();
    fetchMacroHelp();
    pollMoonrakerHealth();
    pollRtHealth();
    renderDriveBanner();
    const mr = setInterval(pollMoonrakerHealth, MOONRAKER_HEALTH_POLL_MS);
    const rt = setInterval(pollRtHealth, RT_HEALTH_POLL_MS);
    const banner = setInterval(renderDriveBanner, 1000);
    return () => {
      clearInterval(mr);
      clearInterval(rt);
      clearInterval(banner);
    };
  }, []);
  const onUrlChange = () => {
    const input = urlRef.current;
    if (!input) return;
    localStorage.setItem(MOONRAKER_KEY, input.value);
    pollMoonrakerHealth();
    fetchMacroHelp();
  };
  return html`<header class="topbar">
    <h1>servo-cal</h1>
    <${Tabs} />
    <label class="moonraker">moonraker
      <input ref=${urlRef} type="text" id="moonraker-url" size="24" onChange=${onUrlChange} />
    </label>
    <span id="moonraker-health" class="mr-health" title="checked every few seconds via GET /server/info"></span>
    <span id="rt-health" class="rt-health" title="EtherCAT endpoint RT-loop health from the live telemetry tap: cycles skipped, frames past the SYNC0 latch, and the current frame margin"></span>
    <div id="drive-state-banner" class="banner"></div>
    <button id="estop-btn" class="estop" title="emergency stop — POST /printer/emergency_stop, fires on the first click" onClick=${emergencyStop}>STOP</button>
  </header>`;
}

function PageOutlet() {
  const page = useRoute();
  state.page = PAGE_DEFS[page] ? page : DEFAULT_PAGE;
  const def = currentPageDef();
  if (def.strain) return html`<${StrainPage} def=${def} />`;
  if (def.live)
    return html`<${LivePage}
      aside=${html`<${ConsolePanel} templates=${def.templates} /><${LaunchpadPad} />`}
    />`;
  if (def.journal) return html`<${JournalPage} />`;
  if (def.docs) return html`<${DocsPage} />`;
  return html`<${TunePage} />`;
}

function App() {
  useEffect(() => {
    bindAccordionToggle();
  }, []);
  return html`<${Topbar} />
    <div id="page-root"><${PageOutlet} /></div>`;
}

export { App, currentPageDef, pageFromHash, motorView, motorViewPerMotor, motorViewEffective, motorViewToggleHtml, syncMotorViewChips, sectionHeadHtml, ACCORDION_KEY, loadCollapsedSections, sectionLabel, applyAccordionState, bindAccordionToggle, makeColumnsResizable, bindAnalysisControls };
