import { ensurePlotSeries, pageRuns } from "./queries/runs";
import { drawTimeDomain } from "./charts-core";
import { renderFrfCharts, renderRingdownCharts } from "./dynamics";
import { renderMetricsTable, renderSweepMetricsChart, renderPsdChart, renderAccelPsdChart, visibleStepNames, renderStepChips, renderMotorChips } from "./metrics";
import { renderPathChart } from "./path-chart";
import { selectedRunNames } from "./runs";
import { currentPageDef } from "./shell";
import { state } from "./state";
import type { PlotSeries } from "./api/runs";

/// Redraw the current page's chart sections from the run selection. Plot
/// series are cached per run mtime, so reselecting is cheap.
async function redrawCharts() {
  const def = currentPageDef();
  if (def.journal) return;
  if (def.strain) return;
  const names = selectedRunNames();
  const plots: PlotSeries[] = [];
  const okNames: string[] = [];
  for (const n of names) {
    try {
      plots.push(await ensurePlotSeries(n));
      okNames.push(n);
    } catch (e) {
      console.error(e);
    }
  }
  const stepNames = [...new Set(plots.flatMap((p) => p.steps.map((s) => s.name)))];
  const filter = state.stepFilter;
  if (filter && !stepNames.some((s) => filter.has(s))) {
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
  if (def.charts && def.charts.includes("time")) {
    const timeMotors = [...new Set(plots.flatMap((p) => p.steps.flatMap((s) => Object.keys(s.drives))))];
    if (state.motorFilter && !timeMotors.some((m) => state.motorFilter!.has(m))) {
      state.motorFilter = null;
    }
    renderMotorChips(timeMotors);
  }
  if (def.charts && def.charts.includes("path")) renderPathChart(okNames, plots, steps);
  if (def.charts && def.charts.includes("frf")) renderFrfCharts(okNames, plots);
  if (def.charts && def.charts.includes("ringdown")) renderRingdownCharts(okNames, plots);
  if (def.charts && def.charts.includes("psd")) {
    renderPsdChart(okNames, plots, steps);
    renderAccelPsdChart(okNames, plots, steps);
  }
  if (def.charts && def.charts.includes("time")) drawTimeDomain(okNames, plots, steps);
}

export { redrawCharts };
