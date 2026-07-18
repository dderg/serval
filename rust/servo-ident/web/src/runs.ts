import { html, render, useEffect, useRef, useState } from "htm/preact/standalone";
import { api, ensureDetail, ambientDiff, el, pageRuns, shortTime } from "./api";
import { loadRerunForm } from "./drive";
import { redrawCharts } from "./peaks";
import { currentPageDef } from "./shell";
import { PALETTE, INITIAL_SELECTED_RUNS, state } from "./state";
import { notify, useStore } from "./store";

// --- runs table ---------------------------------------------------------------
//
// Preact owns #journal-body: renderRuns() mounts the table component into it
// once per page build, and every later call just notifies the store — the
// component re-renders from state with keyed rows, so an in-progress note
// edit keeps its input and focus across the periodic refresh.

function toggleRunSelection(run, ev) {
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

function togglePin(run) {
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

function DotCell({ run }) {
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
        onClick=${(e) => {
          e.stopPropagation();
          togglePin(run);
        }}
      >
        📌
      </button>`
    : null;
  return html`<td>${swatch}${pin}</td>`;
}

/// Click-to-edit note cell: shows the saved note (or a faint "add note…"
/// hint), swaps to an input on click, saves to POST /api/runs/<name>/note
/// on Enter/blur, and cancels on Escape. Clicks stop propagating so
/// editing a note never toggles the row's chart selection.
function NoteCell({ run }) {
  const [editing, setEditing] = useState(false);
  const inputRef = useRef(null);
  const doneRef = useRef(false);
  useEffect(() => {
    if (editing && inputRef.current) inputRef.current.focus();
  }, [editing]);
  if (!editing) {
    return html`<td
      class=${run.note ? "run-note" : "run-note empty"}
      title=${run.note ? `${run.note} — click to edit` : "click to add a note"}
      onClick=${(e) => {
        e.stopPropagation();
        doneRef.current = false;
        setEditing(true);
      }}
    >
      ${run.note || "add note…"}
    </td>`;
  }
  const finish = (save) => {
    if (doneRef.current) return;
    doneRef.current = true;
    const text = inputRef.current.value;
    setEditing(false);
    if (save) saveNote(run, text);
  };
  return html`<td class="run-note" onClick=${(e) => e.stopPropagation()}>
    <input
      ref=${inputRef}
      type="text"
      class="run-note-input"
      defaultValue=${run.note || ""}
      onKeyDown=${(ev) => {
        ev.stopPropagation();
        if (ev.key === "Enter") finish(true);
        if (ev.key === "Escape") finish(false);
      }}
      onBlur=${() => finish(true)}
      onClick=${(ev) => ev.stopPropagation()}
    />
  </td>`;
}

function RunRow({ run, def }) {
  const globalIdx = state.runs.indexOf(run);
  const detail = state.details.get(run.name);
  const manifest = detail && detail.manifest;
  const prevManifest =
    globalIdx + 1 < state.runs.length
      ? (state.details.get(state.runs[globalIdx + 1].name) || {}).manifest
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
    onClick=${(ev) => toggleRunSelection(run, ev)}
    onContextMenu=${(ev) => {
      ev.preventDefault();
      deleteRun(run);
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
        onClick=${(e) => {
          e.stopPropagation();
          loadRerunForm(run.name);
        }}
      >
        → console
      </button>
      ${run.has_results
        ? null
        : html`<button
            onClick=${(e) => {
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
  const runs = def.journal ? state.runs : pageRuns(def);
  return runs.map((run) => html`<${RunRow} key=${run.name} run=${run} def=${def} />`);
}

let mountedRunsBody = null;

function renderRuns() {
  const tbody = el("journal-body");
  if (!tbody) return;
  if (mountedRunsBody !== tbody) {
    if (mountedRunsBody) render(null, mountedRunsBody);
    mountedRunsBody = tbody;
    render(html`<${RunsTable} />`, tbody);
  }
  notify();
}

/// The note shows up the moment Enter is pressed, then the POST confirms
/// (or a refresh rolls back) in the background.
function applyNoteLocally(name, note) {
  const current = state.runs.find((r) => r.name === name);
  if (current) current.note = note;
  renderRuns();
}

async function saveNote(run, text) {
  applyNoteLocally(run.name, text.trim() || null);
  try {
    const saved = await api(`/api/runs/${encodeURIComponent(run.name)}/note`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ note: text }),
    });
    applyNoteLocally(run.name, saved.note || null);
  } catch (e) {
    console.error(e);
    alert(`saving note failed: ${e.message}`);
    await refresh();
  }
}

async function deleteRun(run) {
  const ok = confirm(
    `Delete run ${run.name}?\n\nRemoves its whole directory — captures, results, note.`
  );
  if (!ok) return;
  try {
    await api(`/api/runs/${encodeURIComponent(run.name)}`, { method: "DELETE" });
  } catch (e) {
    console.error(e);
    alert(`deleting ${run.name} failed: ${e.message}`);
    return;
  }
  state.runs = state.runs.filter((r) => r.name !== run.name);
  state.selected.delete(run.name);
  state.pinned.delete(run.name);
  state.details.delete(run.name);
  state.plotSeries.delete(run.name);
  syncRunColors();
  renderRuns();
  redrawCharts();
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

export { renderRuns, applyNoteLocally, saveNote, deleteRun, triggerAnalyze, selectedRunNames, syncRunColors, leastUsedColor, runColor, autoSelectInitialRuns, refresh };
