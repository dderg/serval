import { api, el } from "./api";
import { createPathView } from "./path-view";
import type { PathTrace } from "./path-view";
import { PALETTE, state } from "./state";
import type { PlotPath, PlotSeries, RunPath } from "./wire";

// --- run toolpath chart (tune page) ------------------------------------------
//
// The captured commanded vs actual XY path per step, in mm through the
// manifest's spatial frame — the recorded twin of the live spatial view.
// One trace pair per (selected run, visible step): dashed commanded, solid
// actual, both in the run's list color. The section stays hidden for runs
// whose captures carry no XY path (single-axis, pre-spatial manifests).
//
// plot_series carries a ~4000-point preview of each step's path; the first
// time the section is visible for a selected run, the full-resolution path
// is fetched from /api/runs/<name>/path (cached per run mtime in
// state.pathFull) and the traces are rebuilt once from it. Until it lands —
// or if the fetch fails — the preview keeps rendering, with the load state
// surfaced in the section note.

const CMD_WIDTH = 1;
const ACT_WIDTH = 1.75;
const CMD_DASH = [5, 3];

function stepFullPath(full: RunPath | undefined, stepName: string): PlotPath | null {
  if (!full) return null;
  const step = full.steps.find((s) => s.name === stepName);
  return step ? step.path : null;
}

function pathTraces(
  names: string[],
  plots: PlotSeries[],
  steps: string[],
  colors: Map<string, string>,
  fullByRun?: Map<string, RunPath>
): PathTrace[] {
  const traces: PathTrace[] = [];
  names.forEach((name, i) => {
    const color = colors.get(name) ?? PALETTE[i % PALETTE.length];
    for (const step of plots[i].steps) {
      if (!steps.includes(step.name)) continue;
      const path = stepFullPath(fullByRun?.get(name), step.name) ?? step.path;
      if (!path) continue;
      traces.push({
        xs: path.cmd_x_mm,
        ys: path.cmd_y_mm,
        color,
        width: CMD_WIDTH,
        dash: CMD_DASH,
      });
      traces.push({ xs: path.act_x_mm, ys: path.act_y_mm, color, width: ACT_WIDTH });
    }
  });
  return traces;
}

const runView = createPathView();
const pendingFetches = new Set<string>();
let lastRender: { names: string[]; plots: PlotSeries[]; steps: string[] } | null = null;
let unfoldListenerBound = false;

function freshFullEntry(name: string) {
  const run = state.runs.find((r) => r.name === name);
  const cached = state.pathFull.get(name);
  if (cached && run && cached.mtime_utc === run.mtime_utc) return cached;
  return null;
}

function readyFullPaths(names: string[]): Map<string, RunPath> {
  const map = new Map<string, RunPath>();
  for (const name of names) {
    const entry = freshFullEntry(name);
    if (entry && entry.data) map.set(name, entry.data);
  }
  return map;
}

function rerenderLast() {
  if (lastRender) renderPathChart(lastRender.names, lastRender.plots, lastRender.steps);
}

function ensureFullPaths(names: string[]) {
  for (const name of names) {
    if (freshFullEntry(name) || pendingFetches.has(name)) continue;
    pendingFetches.add(name);
    const run = state.runs.find((r) => r.name === name);
    const mtime = run ? run.mtime_utc : null;
    api(`/api/runs/${encodeURIComponent(name)}/path`)
      .then((data: RunPath) => {
        state.pathFull.set(name, { mtime_utc: mtime, data, error: null });
      })
      .catch((e) => {
        state.pathFull.set(name, { mtime_utc: mtime, data: null, error: String(e) });
      })
      .finally(() => {
        pendingFetches.delete(name);
        rerenderLast();
      });
  }
}

function fullResNote(names: string[], full: Map<string, RunPath>): string {
  if (names.some((n) => pendingFetches.has(n))) return " — loading full-resolution path…";
  const failed = names.map(freshFullEntry).find((e) => e && e.error);
  if (failed) return ` — full-res path failed (showing preview): ${failed.error}`;
  if ([...full.values()].some((rp) => rp.steps.some((s) => s.truncated))) {
    return " — full-res path truncated by the server point cap";
  }
  return "";
}

function repaintPathChart(canvas: HTMLCanvasElement, traces: PathTrace[], statusNote: string) {
  runView.render(canvas, traces);
  const note = el("path-note");
  if (note) {
    const zoomHint = runView.isManual()
      ? ""
      : " — ctrl+wheel zooms, scroll or drag pans, double-click refits";
    const text = `dashed: commanded, solid: actual, color per run${zoomHint}${statusNote}`;
    if (note.textContent !== text) note.textContent = text;
  }
}

function bindUnfoldListener() {
  if (unfoldListenerBound) return;
  unfoldListenerBound = true;
  document.addEventListener("click", (e) => {
    if (!(e.target as HTMLElement).closest("#path-section .section-head")) return;
    setTimeout(rerenderLast, 0);
  });
}

function renderPathChart(names: string[], plots: PlotSeries[], steps: string[]) {
  lastRender = { names, plots, steps };
  const section = el("path-section");
  const canvas = el<HTMLCanvasElement>("path-canvas");
  if (!section || !canvas) return;
  const full = readyFullPaths(names);
  const traces = pathTraces(names, plots, steps, state.runColors, full);
  if (!traces.length) {
    section.hidden = true;
    return;
  }
  section.hidden = false;
  bindUnfoldListener();
  const namesWithPath = names.filter((name, i) => plots[i].steps.some((s) => s.path));
  if (!section.classList.contains("collapsed")) ensureFullPaths(namesWithPath);
  const statusNote = fullResNote(namesWithPath, full);
  runView.bind(canvas, el("path-fit"), () => repaintPathChart(canvas, traces, statusNote));
  repaintPathChart(canvas, traces, statusNote);
}

export { pathTraces, stepFullPath, renderPathChart };
