import { el } from "./api";
import { fillFilterChips, mixColor } from "./charts-core";
import { createPathView, nearestTrace } from "./path-view";
import type { PathTrace, RenderedView, Viewport } from "./path-view";
import { ensureFullPaths, freshPathError, isPathPending, readyFullPaths } from "./queries/path";
import { PALETTE, state } from "./state";
import type { PlotPath } from "./wire";
import type { PlotSeries, RunPath } from "./api/runs";

// --- run toolpath chart (tune page) ------------------------------------------
//
// The captured commanded vs actual XY path per step, in mm through the
// manifest's spatial frame — the recorded twin of the live spatial view.
// One trace pair per (selected run, visible step): dashed commanded, solid
// actual. Within a run, steps ramp from the run's list color toward white so
// sweep strokes stay tellable apart while runs stay grouped; the legend chips
// toggle individual (run, step) pairs. The section stays hidden for runs
// whose captures carry no XY path (single-axis, pre-spatial manifests).
//
// plot_series carries a ~4000-point preview of each step's path; the first
// time the section is visible for a selected run, the full-resolution path
// is fetched and cached per run mtime by queries/path (ensureFullPaths)
// and the traces are rebuilt once from it. Until it lands —
// or if the fetch fails — the preview keeps rendering, with the load state
// surfaced in the section note.

const CMD_WIDTH = 1;
const ACT_WIDTH = 1.75;
const CMD_DASH = [5, 3];
const STEP_RAMP_MAX = 0.55;
const HOVER_REACH_PX = 8;
const HOVER_DIM = 0.6;
const HOVER_WIDTH_BONUS = 0.75;
const BG_COLOR = "#0d1117";

interface PathEntry {
  run: string;
  step: string;
  kind: "commanded" | "actual";
  label: string;
  trace: PathTrace;
}

function entryKey(run: string, step: string): string {
  return `${run}\0${step}`;
}

function stepFullPath(full: RunPath | undefined, stepName: string): PlotPath | null {
  if (!full) return null;
  const step = full.steps.find((s) => s.name === stepName);
  return step ? step.path : null;
}

function pathEntries(
  names: string[],
  plots: PlotSeries[],
  steps: string[],
  colors: Map<string, string>,
  fullByRun?: Map<string, RunPath>
): PathEntry[] {
  const entries: PathEntry[] = [];
  names.forEach((name, i) => {
    const base = colors.get(name) ?? PALETTE[i % PALETTE.length];
    const visible = plots[i].steps.filter(
      (s) => steps.includes(s.name) && (stepFullPath(fullByRun?.get(name), s.name) ?? s.path)
    );
    visible.forEach((step, stepIdx) => {
      const path = stepFullPath(fullByRun?.get(name), step.name) ?? step.path;
      if (!path) throw new Error(`step ${step.name}: path vanished between filter and build`);
      const ramp = visible.length > 1 ? (STEP_RAMP_MAX * stepIdx) / (visible.length - 1) : 0;
      const color = mixColor(base, "#ffffff", ramp);
      const label = names.length > 1 ? `${name} · ${step.name}` : step.name;
      entries.push({
        run: name,
        step: step.name,
        kind: "commanded",
        label,
        trace: { xs: path.cmd_x_mm, ys: path.cmd_y_mm, color, width: CMD_WIDTH, dash: CMD_DASH },
      });
      entries.push({
        run: name,
        step: step.name,
        kind: "actual",
        label,
        trace: { xs: path.act_x_mm, ys: path.act_y_mm, color, width: ACT_WIDTH },
      });
    });
  });
  return entries;
}

function pathTraces(
  names: string[],
  plots: PlotSeries[],
  steps: string[],
  colors: Map<string, string>,
  fullByRun?: Map<string, RunPath>
): PathTrace[] {
  return pathEntries(names, plots, steps, colors, fullByRun).map((e) => e.trace);
}

const runView = createPathView();
let pairFilter: Set<string> | null = null;
let lastRender: { names: string[]; plots: PlotSeries[]; steps: string[] } | null = null;
let unfoldListenerBound = false;
let hoverListenersBound = false;
let current: { entries: PathEntry[]; statusNote: string } = { entries: [], statusNote: "" };
let hover: { entryIndex: number; pointIndex: number; px: number; py: number } | null = null;

function rerenderLast() {
  if (lastRender) renderPathChart(lastRender.names, lastRender.plots, lastRender.steps);
}

function fullResNote(names: string[], full: Map<string, RunPath>): string {
  if (names.some(isPathPending)) return " — loading full-resolution path…";
  const failed = names.map(freshPathError).find((e) => e);
  if (failed) return ` — full-res path failed (showing preview): ${failed}`;
  if ([...full.values()].some((rp) => rp.steps.some((s) => s.truncated))) {
    return " — full-res path truncated by the server point cap";
  }
  return "";
}

function shownEntries(): PathEntry[] {
  if (!pairFilter) return current.entries;
  const filter = pairFilter;
  const kept = current.entries.filter((e) => filter.has(entryKey(e.run, e.step)));
  return kept.length ? kept : current.entries;
}

function styledTraces(entries: PathEntry[]): PathTrace[] {
  const hoveredIdx = hover ? hover.entryIndex : -1;
  return entries.map((e, i) => {
    if (hoveredIdx < 0) return e.trace;
    if (i === hoveredIdx) return { ...e.trace, width: e.trace.width + HOVER_WIDTH_BONUS };
    return { ...e.trace, color: mixColor(e.trace.color, BG_COLOR, HOVER_DIM) };
  });
}

function tracePointPx(
  trace: PathTrace,
  pointIndex: number,
  view: Viewport,
  w: number,
  h: number
): { x: number; y: number } | null {
  if (pointIndex >= trace.xs.length) return null;
  const mx = trace.xs[pointIndex];
  const my = trace.ys[pointIndex];
  if (mx === null || my === null) return null;
  return { x: (mx - view.cx) / view.mmPerPx + w / 2, y: h / 2 - (my - view.cy) / view.mmPerPx };
}

function drawPairMarkers(entries: PathEntry[], rendered: RenderedView) {
  if (!hover) return;
  const e = entries[hover.entryIndex];
  const { ctx, view, w, h } = rendered;
  const sibling = entries.find(
    (s) => s !== e && s.run === e.run && s.step === e.step && s.kind !== e.kind
  );
  const here = tracePointPx(e.trace, hover.pointIndex, view, w, h);
  const there = sibling ? tracePointPx(sibling.trace, hover.pointIndex, view, w, h) : null;
  if (here && there) {
    ctx.strokeStyle = e.trace.color;
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 3]);
    ctx.beginPath();
    ctx.moveTo(here.x, here.y);
    ctx.lineTo(there.x, there.y);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  for (const [pt, entry] of [
    [here, e],
    [there, sibling],
  ] as const) {
    if (!pt || !entry) continue;
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, 3.5, 0, Math.PI * 2);
    ctx.fillStyle = entry.kind === "actual" ? entry.trace.color : BG_COLOR;
    ctx.fill();
    ctx.strokeStyle = entry.trace.color;
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }
}

function drawHoverReadout(entries: PathEntry[]) {
  if (!hover) return;
  const rendered = runView.lastRendered();
  if (!rendered) return;
  const e = entries[hover.entryIndex];
  const { ctx, w } = rendered;
  drawPairMarkers(entries, rendered);
  const text = `${e.label} — ${e.kind}`;
  ctx.font = "11px monospace";
  const tw = ctx.measureText(text).width;
  const cursorNearTopRight = hover.px > w - tw - 40 && hover.py < 40;
  const bx = cursorNearTopRight ? 9 : w - tw - 14;
  const by = 18;
  ctx.fillStyle = "rgba(13, 17, 23, 0.85)";
  ctx.fillRect(bx - 5, by - 12, tw + 10, 17);
  ctx.strokeStyle = e.trace.color;
  ctx.lineWidth = 1;
  ctx.strokeRect(bx - 5, by - 12, tw + 10, 17);
  ctx.fillStyle = "#e6edf3";
  ctx.fillText(text, bx, by);
}

function repaintPathChart(canvas: HTMLCanvasElement) {
  const entries = shownEntries();
  runView.render(canvas, styledTraces(entries));
  drawHoverReadout(entries);
  const note = el("path-note");
  if (note) {
    const zoomHint = runView.isManual()
      ? ""
      : " — ctrl+wheel zooms, scroll or drag pans, double-click refits";
    const text = `dashed: commanded, solid: actual${zoomHint}${current.statusNote}`;
    if (note.textContent !== text) note.textContent = text;
  }
}

function updateHover(canvas: HTMLCanvasElement, e: MouseEvent | null) {
  const entries = shownEntries();
  const rendered = runView.lastRendered();
  let next: typeof hover = null;
  if (e && rendered && !runView.gestureActive()) {
    const rect = canvas.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const hit = nearestTrace(
      entries.map((en) => en.trace),
      rendered.view,
      rendered.w,
      rendered.h,
      px,
      py,
      HOVER_REACH_PX
    );
    if (hit) next = { entryIndex: hit.traceIndex, pointIndex: hit.pointIndex, px, py };
  }
  const changed =
    (hover === null) !== (next === null) ||
    (hover !== null && next !== null && hover.entryIndex !== next.entryIndex);
  const moved = hover !== null && next !== null && (hover.px !== next.px || hover.py !== next.py);
  hover = next;
  if (changed || moved) repaintPathChart(canvas);
}

function bindHoverListeners(canvas: HTMLCanvasElement) {
  if (hoverListenersBound) return;
  hoverListenersBound = true;
  canvas.addEventListener("mousemove", (e) => updateHover(canvas, e));
  canvas.addEventListener("mouseleave", () => updateHover(canvas, null));
}

function renderLegend(canvas: HTMLCanvasElement) {
  const container = el("path-legend");
  if (!container) return;
  const items = current.entries
    .filter((e) => e.kind === "actual")
    .map((e) => ({ key: entryKey(e.run, e.step), label: e.label, swatch: e.trace.color }));
  container.hidden = items.length <= 1;
  fillFilterChips(
    container,
    "all",
    "show every trace",
    "trace",
    items,
    () => pairFilter,
    (next) => {
      pairFilter = next;
    },
    () => {
      hover = null;
      renderLegend(canvas);
      repaintPathChart(canvas);
    }
  );
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
  const entries = pathEntries(names, plots, steps, state.runColors, full);
  if (!entries.length) {
    section.hidden = true;
    return;
  }
  section.hidden = false;
  bindUnfoldListener();
  bindHoverListeners(canvas);
  const namesWithPath = names.filter((name, i) => plots[i].steps.some((s) => s.path));
  if (!section.classList.contains("collapsed")) ensureFullPaths(namesWithPath, rerenderLast);
  current = { entries, statusNote: fullResNote(namesWithPath, full) };
  hover = null;
  renderLegend(canvas);
  runView.bind(canvas, el("path-fit"), () => repaintPathChart(canvas));
  repaintPathChart(canvas);
}

export { pathTraces, pathEntries, stepFullPath, renderPathChart };
