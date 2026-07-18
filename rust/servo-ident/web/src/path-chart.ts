import { el } from "./api";
import { createPathView } from "./path-view";
import type { PathTrace } from "./path-view";
import { PALETTE, state } from "./state";
import type { PlotSeries } from "./wire";

// --- run toolpath chart (tune page) ------------------------------------------
//
// The captured commanded vs actual XY path per step, in mm through the
// manifest's spatial frame — the recorded twin of the live spatial view.
// One trace pair per (selected run, visible step): dashed commanded, solid
// actual, both in the run's list color. The section stays hidden for runs
// whose captures carry no XY path (single-axis, pre-spatial manifests).

const CMD_WIDTH = 1;
const ACT_WIDTH = 1.75;
const CMD_DASH = [5, 3];

function pathTraces(
  names: string[],
  plots: PlotSeries[],
  steps: string[],
  colors: Map<string, string>
): PathTrace[] {
  const traces: PathTrace[] = [];
  names.forEach((name, i) => {
    const color = colors.get(name) ?? PALETTE[i % PALETTE.length];
    for (const step of plots[i].steps) {
      if (!steps.includes(step.name) || !step.path) continue;
      traces.push({
        xs: step.path.cmd_x_mm,
        ys: step.path.cmd_y_mm,
        color,
        width: CMD_WIDTH,
        dash: CMD_DASH,
      });
      traces.push({ xs: step.path.act_x_mm, ys: step.path.act_y_mm, color, width: ACT_WIDTH });
    }
  });
  return traces;
}

const runView = createPathView();

function renderPathChart(names: string[], plots: PlotSeries[], steps: string[]) {
  const section = el("path-section");
  const canvas = el<HTMLCanvasElement>("path-canvas");
  if (!section || !canvas) return;
  const traces = pathTraces(names, plots, steps, state.runColors);
  if (!traces.length) {
    section.hidden = true;
    return;
  }
  section.hidden = false;
  runView.bind(canvas, el("path-fit"), () => renderPathChart(names, plots, steps));
  runView.render(canvas, traces);
  const note = el("path-note");
  if (note) {
    const zoomHint = runView.isManual()
      ? ""
      : " — ctrl+wheel zooms, scroll or drag pans, double-click refits";
    note.textContent = `dashed: commanded, solid: actual, color per run${zoomHint}`;
  }
}

export { pathTraces, renderPathChart };
