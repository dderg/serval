import { html, render } from "htm/preact";
import { useEffect, useRef, useState } from "preact/hooks";
import { useQuery, QueryObserver } from "@tanstack/preact-query";
import type { VNode } from "preact";
import { api, detailData, ensureDetail, runsData, ambientDiff, el, pageRuns, shortTime } from "./api";
import { queryClient, queryKeys, QueryRoot } from "./query-client";
import { loadRerunForm } from "./drive";
import { redrawCharts } from "./peaks";
import { currentPageDef } from "./shell";
import { PALETTE, INITIAL_SELECTED_RUNS, state } from "./state";
import { notify, useStore } from "./store";
import type { PageDef } from "./state";
import type { NotePayload, RunSummary } from "./wire";

const RUNS_POLL_MS = 5000;

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
  renderRuns();
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
  renderRuns();
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
    if (save) saveNote(run, text);
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
    ${!run.has_results ? item("analyze", () => triggerAnalyze(run.name)) : null}
    ${item("delete", () => deleteRun(run), { danger: true })}
  </div>`;
}

let mountedMenuHost: HTMLElement | null = null;

function ensureContextMenuMounted() {
  if (mountedMenuHost) return;
  const host = document.createElement("div");
  document.body.appendChild(host);
  mountedMenuHost = host;
  render(html`<${ContextMenu} />`, host);
}

function RunRow({ run, def }: { run: RunSummary; def: PageDef }) {
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
      ensureContextMenuMounted();
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
              triggerAnalyze(run.name);
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

async function fetchRuns(): Promise<RunSummary[]> {
  const prev = runsData();
  let runs: RunSummary[];
  try {
    runs = await api<RunSummary[]>("/api/runs");
  } catch (e) {
    console.error(e);
    throw e;
  }
  if (runsUnchanged(prev, runs)) return prev;
  await Promise.all(runs.map((r) => ensureDetail(r).catch((e) => console.error(e))));
  queryClient.setQueryData(queryKeys.runs, runs);
  await reconcileRuns(runs);
  return runs;
}

function runsUnchanged(prev: RunSummary[], next: RunSummary[]): boolean {
  return (
    next.length === prev.length &&
    next.every((run, i) => {
      const current = prev[i];
      const verdictsMatch =
        run.verdict === current.verdict ||
        (run.verdict !== null &&
          current.verdict !== null &&
          run.verdict.recommended_step === current.verdict.recommended_step &&
          run.verdict.reason === current.verdict.reason &&
          run.verdict.flags.length === current.verdict.flags.length &&
          run.verdict.flags.every((flag, j) => flag === current.verdict!.flags[j]));
      return (
        run.name === current.name &&
        run.mtime_utc === current.mtime_utc &&
        run.experiment === current.experiment &&
        run.tag === current.tag &&
        run.axis === current.axis &&
        run.has_results === current.has_results &&
        verdictsMatch &&
        run.note === current.note
      );
    })
  );
}

async function reconcileRuns(runs: RunSummary[]) {
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

function RunsRoot() {
  useQuery({
    queryKey: queryKeys.runs,
    queryFn: fetchRuns,
    notifyOnChangeProps: ["data"],
  });
  return html`<${RunsTable} />`;
}

let runsObserver: QueryObserver<RunSummary[]> | null = null;

function startRunsPolling() {
  if (runsObserver) return;
  runsObserver = new QueryObserver<RunSummary[]>(queryClient, {
    queryKey: queryKeys.runs,
    queryFn: fetchRuns,
    refetchInterval: RUNS_POLL_MS,
    refetchIntervalInBackground: false,
  });
  runsObserver.subscribe(() => {});
}

let mountedRunsBody: HTMLElement | null = null;

function renderRuns() {
  const tbody = el("journal-body");
  if (!tbody) return;
  if (mountedRunsBody !== tbody) {
    if (mountedRunsBody) render(null as unknown as VNode, mountedRunsBody);
    mountedRunsBody = tbody;
    render(html`<${QueryRoot}><${RunsRoot} /><//>`, tbody);
  }
  notify();
}

async function saveNote(run: RunSummary, text: string) {
  const optimistic = text.trim() || null;
  queryClient.setQueryData<RunSummary[]>(queryKeys.runs, (runs) =>
    runs ? runs.map((r) => (r.name === run.name ? { ...r, note: optimistic } : r)) : runs
  );
  try {
    const saved = await api<NotePayload>(`/api/runs/${encodeURIComponent(run.name)}/note`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ note: text }),
    });
    queryClient.setQueryData<RunSummary[]>(queryKeys.runs, (runs) =>
      runs ? runs.map((r) => (r.name === run.name ? { ...r, note: saved.note || null } : r)) : runs
    );
  } catch (e) {
    console.error(e);
    alert(`saving note failed: ${e instanceof Error ? e.message : e}`);
    await queryClient.invalidateQueries({ queryKey: queryKeys.runs });
  }
}

async function deleteRun(run: RunSummary) {
  try {
    await api(`/api/runs/${encodeURIComponent(run.name)}`, { method: "DELETE" });
  } catch (e) {
    console.error(e);
    alert(`deleting ${run.name} failed: ${e instanceof Error ? e.message : e}`);
    return;
  }
  queryClient.removeQueries({ queryKey: ["runs", run.name] });
  const remaining = runsData().filter((r) => r.name !== run.name);
  queryClient.setQueryData(queryKeys.runs, remaining);
  await reconcileRuns(remaining);
  await queryClient.invalidateQueries({ queryKey: queryKeys.runs });
}

async function triggerAnalyze(name: string) {
  await api(`/api/runs/${encodeURIComponent(name)}/analyze`, { method: "POST" });
  await queryClient.invalidateQueries({ queryKey: queryKeys.runs });
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

export { renderRuns, selectedRunNames, runColor, startRunsPolling };
