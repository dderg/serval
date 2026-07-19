import { html } from "htm/preact";
import { useEffect, useRef, useState } from "preact/hooks";
import { useQuery } from "@tanstack/preact-query";
import { ambientDiff, el, resetRenderState, shortTime } from "./api";
import {
  runsData,
  detailData,
  pageRuns,
  runsQuery,
  useSaveNote,
  useDeleteRun,
  useAnalyzeRun,
} from "./queries/runs";
import { ConsolePanel } from "./console";
import { DrivePanel, loadRerunForm } from "./drive";
import { redrawCharts } from "./peaks";
import { renderSentLog } from "./moonraker";
import { LaunchpadPad } from "./launchpad";
import {
  MetricsSection,
  SweepMetricsSection,
  PsdSection,
  AccelPsdSection,
} from "./metrics";
import { FrfSection, RingdownSection } from "./dynamics";
import { SectionHead, TimeDomainSection, PathSection } from "./charts-core";
import { applyAccordionState, bindAnalysisControls, currentPageDef } from "./shell";
import { PALETTE, INITIAL_SELECTED_RUNS, state } from "./state";
import { notify, useStore } from "./store";
import type { PageDef } from "./state";
import type { RunSummary } from "./api/runs";

function toggleRunSelection(run: RunSummary, ev: MouseEvent) {
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
  notify();
  redrawCharts();
}

function togglePin(run: RunSummary) {
  if (state.pinned.has(run.name)) {
    state.pinned.delete(run.name);
  } else {
    state.pinned.add(run.name);
    state.selected.add(run.name);
  }
  syncRunColors();
  notify();
  redrawCharts();
}

function DotCell({ run }: { run: RunSummary }) {
  const swatch = state.runColors.has(run.name)
    ? html`<span class="swatch" style=${{ background: runColor(run.name) }}></span>`
    : null;
  const pinned = state.pinned.has(run.name);
  const pin = run.has_results
    ? html`<button
        class=${pinned ? "pin-toggle pinned" : "pin-toggle"}
        title=${pinned
          ? "unpin — plain clicks will deselect this run again"
          : "pin — keep this run selected while plain clicks switch other runs"}
        onClick=${(e: MouseEvent) => {
          e.stopPropagation();
          togglePin(run);
        }}
      >
        📌
      </button>`
    : null;
  return html`<td>${swatch}${pin}</td>`;
}

function NoteCell({ run }: { run: RunSummary }) {
  const [editing, setEditing] = useState(false);
  const inputRef = useRef<HTMLInputElement>(null);
  const doneRef = useRef(false);
  const note = useSaveNote();
  useEffect(() => {
    if (editing && inputRef.current) inputRef.current.focus();
  }, [editing]);
  if (!editing) {
    return html`<td
      class=${run.note ? "run-note" : "run-note empty"}
      title=${run.note ? `${run.note} — click to edit` : "click to add a note"}
      onClick=${(e: MouseEvent) => {
        e.stopPropagation();
        doneRef.current = false;
        setEditing(true);
      }}
    >
      ${run.note || "add note…"}
    </td>`;
  }
  const finish = (save: boolean) => {
    if (doneRef.current || !inputRef.current) return;
    doneRef.current = true;
    const text = inputRef.current.value;
    setEditing(false);
    if (save) note.mutate({ name: run.name, text });
  };
  return html`<td class="run-note" onClick=${(e: MouseEvent) => e.stopPropagation()}>
    <input
      ref=${inputRef}
      type="text"
      class="run-note-input"
      defaultValue=${run.note || ""}
      onKeyDown=${(ev: KeyboardEvent) => {
        ev.stopPropagation();
        if (ev.key === "Enter") finish(true);
        if (ev.key === "Escape") finish(false);
      }}
      onBlur=${() => finish(true)}
      onClick=${(ev: MouseEvent) => ev.stopPropagation()}
    />
  </td>`;
}

interface MenuState {
  run: RunSummary | null;
  x: number;
  y: number;
}

const menu: MenuState = { run: null, x: 0, y: 0 };

function openContextMenu(run: RunSummary, x: number, y: number) {
  menu.run = run;
  menu.x = x;
  menu.y = y;
  notify();
}

function closeContextMenu() {
  if (!menu.run) return;
  menu.run = null;
  notify();
}

function ContextMenu() {
  useStore();
  const ref = useRef<HTMLDivElement>(null);
  const del = useDeleteRun();
  const analyze = useAnalyzeRun();
  useEffect(() => {
    if (!menu.run) return;
    const onPointerDown = (ev: MouseEvent) => {
      if (ref.current && ev.target instanceof Node && ref.current.contains(ev.target)) return;
      closeContextMenu();
    };
    const onKeyDown = (ev: KeyboardEvent) => {
      if (ev.key === "Escape") closeContextMenu();
    };
    document.addEventListener("mousedown", onPointerDown);
    document.addEventListener("keydown", onKeyDown);
    document.addEventListener("scroll", closeContextMenu, true);
    window.addEventListener("blur", closeContextMenu);
    return () => {
      document.removeEventListener("mousedown", onPointerDown);
      document.removeEventListener("keydown", onKeyDown);
      document.removeEventListener("scroll", closeContextMenu, true);
      window.removeEventListener("blur", closeContextMenu);
    };
  }, [menu.run]);
  if (!menu.run) return null;
  const run = menu.run;
  const pinned = state.pinned.has(run.name);
  const detail = detailData(run.name);
  const width = Math.max(document.documentElement.clientWidth - 8, 0);
  const height = Math.max(document.documentElement.clientHeight - 8, 0);
  const style = {
    left: `${Math.min(menu.x, width)}px`,
    top: `${Math.min(menu.y, height)}px`,
  };
  const item = (label: string, onClick: () => void, opts?: { disabled?: boolean; danger?: boolean }) =>
    html`<button
      class=${[
        "context-menu-item",
        opts?.danger ? "danger" : "",
        opts?.disabled ? "disabled" : "",
      ]
        .filter(Boolean)
        .join(" ")}
      disabled=${opts?.disabled || null}
      onClick=${() => {
        closeContextMenu();
        onClick();
      }}
    >
      ${label}
    </button>`;
  return html`<div class="context-menu" style=${style} ref=${ref}>
    ${run.has_results ? item(pinned ? "unpin" : "pin", () => togglePin(run)) : null}
    ${item("→ console", () => loadRerunForm(run.name), { disabled: !detail?.manifest })}
    ${!run.has_results ? item("analyze", () => analyze.mutate(run.name)) : null}
    ${item("delete", () => del.mutate(run.name), { danger: true })}
  </div>`;
}


function RunRow({ run, def }: { run: RunSummary; def: PageDef }) {
  const analyze = useAnalyzeRun();
  const runs = runsData();
  const globalIdx = runs.findIndex((r) => r.name === run.name);
  const detail = detailData(run.name);
  const manifest = detail && detail.manifest;
  const prevManifest =
    globalIdx >= 0 && globalIdx + 1 < runs.length
      ? (detailData(runs[globalIdx + 1].name)?.manifest ?? null)
      : null;
  const diff = manifest ? ambientDiff(prevManifest, manifest) : "";
  const cls = [
    state.selected.has(run.name) ? "selected" : "",
    run.has_results ? "selectable" : "",
  ]
    .filter(Boolean)
    .join("");
  return html`<tr
    class=${cls || null}
    onClick=${(ev: MouseEvent) => toggleRunSelection(run, ev)}
    onContextMenu=${(ev: MouseEvent) => {
      ev.preventDefault();
      openContextMenu(run, ev.clientX, ev.clientY);
    }}
  >
    <${DotCell} run=${run} />
    <td title=${`${run.name} — ${run.mtime_utc}`}>${shortTime(run.mtime_utc)}</td>
    <td title=${run.experiment}>
      ${def.journal
        ? `${run.experiment}/${run.tag}${run.axis ? " " + run.axis : ""}`
        : `${run.tag}${run.axis ? " " + run.axis : ""}`}
    </td>
    <td class=${diff ? "diff" : "diff empty"} title=${diff || null}>${diff || "—"}</td>
    <${NoteCell} run=${run} />
    <td class="actions">
      <button
        title="prefill the console with this run's command"
        disabled=${!manifest}
        onClick=${(e: MouseEvent) => {
          e.stopPropagation();
          loadRerunForm(run.name);
        }}
      >
        → console
      </button>
      ${run.has_results
        ? null
        : html`<button
            onClick=${(e: MouseEvent) => {
              e.stopPropagation();
              analyze.mutate(run.name);
            }}
          >
            analyze
          </button>`}
    </td>
  </tr>`;
}

function RunsTable() {
  useStore();
  const def = currentPageDef();
  const runs = def.journal ? runsData() : pageRuns(def);
  return runs.map((run) => html`<${RunRow} key=${run.name} run=${run} def=${def} />`);
}


function RunsBody() {
  useQuery({ ...runsQuery(), notifyOnChangeProps: ["data"] });
  return html`<${RunsTable} />`;
}

function usePageBootstrap(withCharts: boolean) {
  useEffect(() => {
    resetRenderState();
    bindAnalysisControls();
    renderSentLog();
    applyAccordionState();
    if (withCharts) void redrawCharts();
  }, []);
}

function TunePage() {
  const def = currentPageDef();
  usePageBootstrap(true);
  const runsNote = `<span class="note">${
    def.experiments ? def.experiments.join(", ") : "all experiments"
  } — click a row to chart it</span>`;
  return html`<div class="workspace">
      <main class="analysis">
        <section class="runs-section">
          <${SectionHead} title="runs" tools=${runsNote} />
          <div class="table-wrap runs-wrap">
            <table>
              <thead>
                <tr>
                  <th></th><th>time</th><th>tag</th>
                  <th>ambient diff vs previous</th><th>note</th><th></th>
                </tr>
              </thead>
              <tbody id="journal-body"><${RunsBody} /></tbody>
            </table>
          </div>
        </section>
        <${MetricsSection} />
        <${SweepMetricsSection} />
        <${PathSection} />
        <${FrfSection} />
        <${RingdownSection} />
        <${PsdSection} />
        <${AccelPsdSection} />
        <${TimeDomainSection} />
      </main>
      <aside class="controls">
        <section class="panel">
          <div class="section-head"><h2>drive tuning</h2></div>
          <div id="drive-panel"><${DrivePanel} /></div>
        </section>
        <${ConsolePanel} templates=${def.templates} />
        <${LaunchpadPad} />
      </aside>
    </div>
    <${ContextMenu} />`;
}

function JournalPage() {
  usePageBootstrap(false);
  return html`<div class="workspace single">
      <main class="analysis">
        <section class="runs-section">
          <div class="section-head"><h2>journal — every run</h2></div>
          <div class="table-wrap journal-wrap">
            <table>
              <thead>
                <tr>
                  <th></th><th>time</th><th>experiment/tag</th>
                  <th>ambient diff vs previous</th><th>note</th><th></th>
                </tr>
              </thead>
              <tbody id="journal-body"><${RunsBody} /></tbody>
            </table>
          </div>
        </section>
        <${ConsolePanel} />
        <${LaunchpadPad} />
      </main>
    </div>
    <${ContextMenu} />`;
}

export async function reconcileRuns(runs: RunSummary[]) {
  const known = new Set(runs.map((r) => r.name));
  for (const name of [...state.selected]) {
    if (!known.has(name)) state.selected.delete(name);
  }
  for (const name of [...state.pinned]) {
    if (!known.has(name)) state.pinned.delete(name);
  }
  autoSelectInitialRuns();
  syncRunColors();
  notify();
  await redrawCharts();
}

function selectedRunNames() {
  return runsData().filter((r) => state.selected.has(r.name)).map((r) => r.name);
}

function syncRunColors() {
  for (const name of [...state.runColors.keys()]) {
    if (!state.selected.has(name)) state.runColors.delete(name);
  }
  for (const name of selectedRunNames()) {
    if (!state.runColors.has(name)) state.runColors.set(name, leastUsedColor());
  }
}

function leastUsedColor(): string {
  const counts = new Map(PALETTE.map((c) => [c, 0]));
  for (const c of state.runColors.values()) counts.set(c, (counts.get(c) ?? 0) + 1);
  return PALETTE.reduce((best, c) => ((counts.get(c) ?? 0) < (counts.get(best) ?? 0) ? c : best));
}

function runColor(name: string): string {
  const color = state.runColors.get(name);
  if (!color) throw new Error(`${name}: no color assigned — run is not selected`);
  return color;
}

function autoSelectInitialRuns() {
  if (state.autoSelected) return;
  const withResults = runsData().filter((r) => r.has_results);
  if (!withResults.length) return;
  state.autoSelected = true;
  for (const run of withResults.slice(0, INITIAL_SELECTED_RUNS)) {
    state.selected.add(run.name);
  }
  if (!state.console.text) loadRerunForm(withResults[0].name);
}

export { selectedRunNames, runColor, TunePage, JournalPage };
export { startRunsPolling } from "./queries/runs";
