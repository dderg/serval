import { api, el, ensureDetail, ambientDiff, pageRuns, shortTime } from "./api.js";
import { loadRerunForm } from "./drive.js";
import { redrawCharts } from "./peaks.js";
import { currentPageDef } from "./shell.js";
import { PALETTE, INITIAL_SELECTED_RUNS, state } from "./state.js";

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

    tr.addEventListener("contextmenu", (ev) => {
      ev.preventDefault();
      deleteRun(run);
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

/// The note shows up the moment Enter is pressed: it goes into
/// state.pendingNotes, which renderRuns/refresh overlay onto whatever the
/// server returns, then the POST confirms (or rolls back) in the background.
/// Without the overlay, the periodic refresh replaces state.runs with
/// server data that predates the save — during a long calibration the POST
/// can sit queued behind it, blanking the note until the run finishes.
function applyNoteLocally(name, note) {
  const current = state.runs.find((r) => r.name === name);
  if (current) current.note = note;
  renderRuns();
}

async function saveNote(run, text) {
  const note = text.trim() || null;
  state.pendingNotes.set(run.name, note);
  applyNoteLocally(run.name, note);
  try {
    const saved = await api(`/api/runs/${encodeURIComponent(run.name)}/note`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ note: text }),
    });
    if (state.pendingNotes.get(run.name) === note) {
      state.pendingNotes.delete(run.name);
      applyNoteLocally(run.name, saved.note || null);
    }
  } catch (e) {
    console.error(e);
    if (state.pendingNotes.get(run.name) === note) {
      state.pendingNotes.delete(run.name);
      renderRuns();
    }
    alert(`saving note failed: ${e.message}`);
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
  state.pendingNotes.delete(run.name);
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
  for (const [name, note] of state.pendingNotes) {
    const run = state.runs.find((r) => r.name === name);
    if (run) run.note = note;
  }
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

export { renderRuns, noteCell, applyNoteLocally, saveNote, deleteRun, triggerAnalyze, selectedRunNames, syncRunColors, leastUsedColor, runColor, autoSelectInitialRuns, refresh };
