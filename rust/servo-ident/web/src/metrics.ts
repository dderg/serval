import { el, payloadUnchanged, runDataSig } from "./api";
import { mixColor, traceStyle, clipToPsdBand, psdMaxFreqHz, psdToAmplitude, countsPerMm } from "./charts-core";
import { psdPlot, timeSeriesPlot } from "./uplot-chart";
import type { FixedY, FreqMarker, Mark, PsdTrace, TimeTrace } from "./uplot-chart";
import type { DriveMetrics, Manifest, PlotSeries, PlotStep, TorqueMetrics } from "./wire";
import { redrawCharts } from "./peaks";
import { runColor } from "./runs";
import { motorView, motorViewPerMotor } from "./shell";
import { PALETTE, RESONANCE_BAND_HZ, state } from "./state";

// --- tracking metrics table -----------------------------------------------
//
// The replacement for the old gain-report PNG's "metrics vs gain" panel:
// results.json already carries per-drive, per-move ferr/overshoot/settle and
// the torque summary — this table is the view on top of them.

interface MoveSummary {
  ferrPeak: number;
  ferrRms: number;
  overshoot: number;
  settleWorstMs: number | null;
  neverSettled: boolean;
  truncated: boolean;
}

interface SettleSummary {
  settleWorstMs: number | null;
  neverSettled: boolean;
  truncated: boolean;
}

interface MetricsRow {
  run: string;
  step: string;
  drive: string;
  ferrPeakUm: number;
  ferrRmsUm: number;
  overshootUm: number;
  settle: SettleSummary;
  torque: TorqueMetrics;
}

function driveMoveSummary(metrics: DriveMetrics): MoveSummary {
  const s: MoveSummary = {
    ferrPeak: 0,
    ferrRms: 0,
    overshoot: 0,
    settleWorstMs: null,
    neverSettled: false,
    truncated: false,
  };
  for (const mv of metrics.moves) {
    s.ferrPeak = Math.max(s.ferrPeak, mv.ferr_peak);
    s.ferrRms = Math.max(s.ferrRms, mv.ferr_rms);
    s.overshoot = Math.max(s.overshoot, mv.overshoot);
    if (mv.settle_ms != null) {
      if (s.settleWorstMs == null || mv.settle_ms > s.settleWorstMs) {
        s.settleWorstMs = mv.settle_ms;
      }
    } else if (mv.settle_window_truncated) {
      s.truncated = true;
    } else {
      s.neverSettled = true;
    }
  }
  return s;
}

function settleCellHtml(s: SettleSummary): string {
  if (s.neverSettled) return `<span class="badge resonance">never</span>`;
  const truncatedBadge =
    `<span class="badge truncated" title="the capture ended inside a move's ` +
    `settle window, so the worst settle may be underestimated">truncated</span>`;
  if (s.settleWorstMs == null) return s.truncated ? truncatedBadge : "—";
  const value = `${s.settleWorstMs.toFixed(1)} ms`;
  return s.truncated ? `${value} ${truncatedBadge}` : value;
}

function torqueCellHtml(tq: TorqueMetrics): string {
  const peak = `${tq.peak_pct_rated.toFixed(0)}%`;
  if (!tq.rail_detected) return peak;
  return (
    `${peak} <span class="badge torque" title="on the rail ${tq.rail_pct_moving.toFixed(1)}% ` +
    `of moving time (${tq.rail_ms.toFixed(0)} ms, longest burst ${tq.longest_burst_ms.toFixed(0)} ms)">rail</span>`
  );
}

function metricsDriveRow(name: string, stepName: string, drive: string, dr: { metrics: DriveMetrics }): MetricsRow {
  const umPerCount = 1000 / countsPerMm(name, drive);
  const s = driveMoveSummary(dr.metrics);
  return {
    run: name,
    step: stepName,
    drive,
    ferrPeakUm: s.ferrPeak * umPerCount,
    ferrRmsUm: s.ferrRms * umPerCount,
    overshootUm: s.overshoot * umPerCount,
    settle: {
      settleWorstMs: s.settleWorstMs,
      neverSettled: s.neverSettled,
      truncated: s.truncated,
    },
    torque: dr.metrics.torque,
  };
}

/// One row per (run, step) folded over drives: "agg" keeps the worst drive
/// per metric, "avg" the mean. Rail badges survive both folds — a railed
/// drive is a railed step no matter the view.
function foldDriveRows(driveRows: MetricsRow[], view: string): MetricsRow {
  const fold = (values: number[]) =>
    view === "avg"
      ? values.reduce((a, b) => a + b, 0) / values.length
      : Math.max(...values);
  const settled = driveRows
    .map((r) => r.settle.settleWorstMs)
    .filter((v): v is number => v != null);
  const worstTorque = driveRows.reduce((a, r) =>
    r.torque.peak_pct_rated > a.torque.peak_pct_rated ? r : a
  ).torque;
  return {
    run: driveRows[0].run,
    step: driveRows[0].step,
    drive: view === "avg" ? "avg" : "worst",
    ferrPeakUm: fold(driveRows.map((r) => r.ferrPeakUm)),
    ferrRmsUm: fold(driveRows.map((r) => r.ferrRmsUm)),
    overshootUm: fold(driveRows.map((r) => r.overshootUm)),
    settle: {
      settleWorstMs: settled.length ? fold(settled) : null,
      neverSettled: driveRows.some((r) => r.settle.neverSettled),
      truncated: driveRows.some((r) => r.settle.truncated),
    },
    torque:
      view === "avg"
        ? {
            ...worstTorque,
            peak_pct_rated: fold(driveRows.map((r) => r.torque.peak_pct_rated)),
          }
        : worstTorque,
  };
}

function metricsTableRows(names: string[], steps: string[]): MetricsRow[] {
  const view = motorView();
  const rows: MetricsRow[] = [];
  for (const name of names) {
    const detail = state.details.get(name);
    if (!detail || !detail.results) continue;
    for (const step of detail.results.steps) {
      if (!steps.includes(step.name)) continue;
      const driveRows = Object.entries(step.drives).map(([drive, dr]) =>
        metricsDriveRow(name, step.name, drive, dr)
      );
      if (!driveRows.length) continue;
      if (view === "per-motor") rows.push(...driveRows);
      else rows.push(foldDriveRows(driveRows, view));
    }
  }
  return rows;
}

/// Red tint scaled to where the value sits between the column's best and
/// worst — the cheap-to-scan replacement for reading 4-drives-per-step
/// numbers one by one. Identical columns get no tint.
function heatCellStyle(value: number, min: number, max: number): string {
  if (!(max > min)) return "";
  const alpha = (0.32 * (value - min)) / (max - min);
  return alpha < 0.02 ? "" : ` style="background:rgba(224,90,79,${alpha.toFixed(3)})"`;
}

function renderMetricsTable(names: string[], steps: string[]) {
  const container = el("metrics-table");
  if (!container) return;
  if (payloadUnchanged("metrics-table", { runs: runDataSig(names), steps, view: motorView() })) {
    return;
  }
  const rows = metricsTableRows(names, steps);
  if (!rows.length) {
    container.innerHTML = '<p class="note">select runs above</p>';
    return;
  }
  const columns = ["ferrPeakUm", "ferrRmsUm", "overshootUm"] as const;
  type HeatColumn = (typeof columns)[number];
  const bounds: Record<HeatColumn, { min: number; max: number }> = {} as Record<HeatColumn, { min: number; max: number }>;
  for (const c of columns) {
    const values = rows.map((r) => r[c]);
    bounds[c] = { min: Math.min(...values), max: Math.max(...values) };
  }
  const heat = (c: HeatColumn, r: MetricsRow) => heatCellStyle(r[c], bounds[c].min, bounds[c].max);
  const stepColors = new Map<string, string>();
  for (const r of rows) {
    if (!stepColors.has(r.step)) {
      stepColors.set(r.step, PALETTE[stepColors.size % PALETTE.length]);
    }
  }
  const body = rows
    .map((r, i) => {
      const swatch = `<span class="swatch" style="background:${runColor(r.run)}"></span>`;
      const stepColor = stepColors.get(r.step)!;
      const prev = rows[i - 1];
      const groupStart = !prev || prev.run !== r.run || prev.step !== r.step;
      return (
        `<tr${groupStart && i > 0 ? ' class="group-start"' : ""}>` +
        `<td class="run-cell" style="border-left:3px solid ${stepColor};padding-left:6px" ` +
        `title="${r.run}">${swatch}${r.run}</td>` +
        `<td style="color:${stepColor}">${r.step}</td><td>${r.drive}</td>` +
        `<td class="num"${heat("ferrPeakUm", r)}>${r.ferrPeakUm.toFixed(1)}</td>` +
        `<td class="num"${heat("ferrRmsUm", r)}>${r.ferrRmsUm.toFixed(1)}</td>` +
        `<td class="num"${heat("overshootUm", r)}>${r.overshootUm.toFixed(1)}</td>` +
        `<td class="num">${settleCellHtml(r.settle)}</td>` +
        `<td class="num">${torqueCellHtml(r.torque)}</td></tr>`
      );
    })
    .join("");
  container.innerHTML =
    `<table class="metrics-table"><thead><tr>` +
    `<th>run</th><th>step</th><th>drive</th>` +
    `<th class="num">ferr peak (µm)</th><th class="num">ferr rms (µm)</th>` +
    `<th class="num">overshoot (µm)</th><th class="num">settle</th>` +
    `<th class="num">torque peak</th>` +
    `</tr></thead><tbody>${body}</tbody></table>`;
}

// --- metrics vs gain chart --------------------------------------------------
//
// The old gain-report PNG's "metrics vs gain" panel: one x position per
// sweep step (the swept gain value from the manifest), overshoot / ferr
// per step maxed over drives, flagged steps marked as red rungs.

function sweptAxisKey(manifest: Manifest | null): string | null {
  if (!manifest || manifest.steps.length < 2) return null;
  const keys = Object.keys(manifest.steps[0].swept || {}).filter((k) =>
    manifest.steps.every((s) => typeof (s.swept || {})[k] === "number")
  );
  const varying = keys.filter(
    (k) => new Set(manifest.steps.map((s) => (s.swept || {})[k])).size > 1
  );
  if (!varying.length) return null;
  return varying.includes("speed") ? "speed" : varying[0];
}

interface SweepPoint {
  x: number;
  flagged: boolean;
  overshootUm: number;
  ferrRmsUm: number;
  ferrPeakUm: number;
}

interface SweepSeries {
  run: string;
  drive: string;
  key: string;
  points: SweepPoint[];
}

function sweepMetricsSeries(names: string[]): SweepSeries[] {
  const series: SweepSeries[] = [];
  for (const name of names) {
    const detail = state.details.get(name);
    if (!detail || !detail.results || !detail.manifest) continue;
    const key = sweptAxisKey(detail.manifest);
    if (!key) continue;
    const sweptByStep = new Map(detail.manifest.steps.map((s) => [s.name, (s.swept || {})[key]]));
    const perDrivePoints = new Map<string, SweepPoint[]>();
    for (const step of detail.results.steps) {
      if (!sweptByStep.has(step.name)) continue;
      const flagged = step.flags.some(
        (f) => f === "resonance_detected" || f === "torque_saturated"
      );
      const view = motorView();
      const driveValues = Object.entries(step.drives).map(([drive, dr]) => {
        const umPerCount = 1000 / countsPerMm(name, drive);
        const s = driveMoveSummary(dr.metrics);
        return {
          drive,
          overshootUm: s.overshoot * umPerCount,
          ferrRmsUm: s.ferrRms * umPerCount,
          ferrPeakUm: s.ferrPeak * umPerCount,
        };
      });
      const stepPoints = new Map<string, { overshootUm: number; ferrRmsUm: number; ferrPeakUm: number }>();
      if (view === "per-motor") {
        for (const v of driveValues) stepPoints.set(v.drive, v);
      } else if (driveValues.length) {
        const fold = (f: (v: { overshootUm: number; ferrRmsUm: number; ferrPeakUm: number }) => number) =>
          view === "avg"
            ? driveValues.reduce((a, v) => a + f(v), 0) / driveValues.length
            : Math.max(...driveValues.map(f));
        stepPoints.set(view === "avg" ? "avg" : "worst drive", {
          overshootUm: fold((v) => v.overshootUm),
          ferrRmsUm: fold((v) => v.ferrRmsUm),
          ferrPeakUm: fold((v) => v.ferrPeakUm),
        });
      }
      for (const [drive, p] of stepPoints) {
        if (!perDrivePoints.has(drive)) perDrivePoints.set(drive, []);
        perDrivePoints.get(drive)!.push({ x: sweptByStep.get(step.name)!, flagged, ...p });
      }
    }
    for (const [drive, points] of perDrivePoints) {
      if (points.length < 2) continue;
      points.sort((a, b) => a.x - b.x);
      series.push({ run: name, drive, key, points });
    }
  }
  return series;
}

function renderSweepMetricsChart(names: string[]) {
  const container = el("sweep-metrics-chart");
  if (!container) return;
  if (payloadUnchanged("sweep-metrics", { runs: runDataSig(names), view: motorView() })) {
    return;
  }
  const series = sweepMetricsSeries(names);
  if (!series.length) {
    container.innerHTML =
      '<p class="note">select a gain sweep / ladder run above (tracking runs have a single step — read them in the metrics table)</p>';
    return;
  }
  container.innerHTML = "";
  const box = document.createElement("div");
  box.className = "chart-box";
  const title = document.createElement("h3");
  const viewLabel = ({ agg: "worst-drive", avg: "avg", "per-motor": "per-motor" } as Record<string, string>)[motorView()];
  title.textContent = `${viewLabel} metrics vs swept ${series[0].key} (µm)`;
  box.appendChild(title);
  const plotHost = document.createElement("div");
  box.appendChild(plotHost);
  const legend = document.createElement("div");
  legend.className = "legend";
  const traces: TimeTrace[] = [];
  const marks: Mark[] = [];
  series.forEach((s) => {
    const runSeries = series.filter((x) => x.run === s.run);
    const color = mixColor(
      runColor(s.run),
      "#ffffff",
      driveRamp(runSeries.length, runSeries.indexOf(s))
    );
    const t = s.points.map((p) => p.x);
    const label = motorViewPerMotor() ? `${s.run} · ${s.drive}` : s.run;
    traces.push({ t, y: s.points.map((p) => p.overshootUm), color, points: true, label: `${label} overshoot` });
    traces.push({ t, y: s.points.map((p) => p.ferrRmsUm), color, dash: [6, 4], label: `${label} ferr rms` });
    traces.push({ t, y: s.points.map((p) => p.ferrPeakUm), color, dash: [2, 3], label: `${label} ferr peak` });
    for (const p of s.points) if (p.flagged) marks.push({ x: p.x, color: "#e05a4f" });
    const item = document.createElement("span");
    item.innerHTML = `<span class="swatch" style="background:${color}"></span>${label}`;
    legend.appendChild(item);
  });
  timeSeriesPlot(plotHost, {
    width: container.clientWidth || 860,
    height: 260,
    yLabel: "µm",
    xUnit: "",
    xTitle: series[0].key,
    marks,
    traces,
    hover: true,
  });
  box.appendChild(legend);
  container.appendChild(box);
}

function driveRamp(count: number, idx: number): number {
  return count > 1 ? (0.5 * idx) / (count - 1) : 0;
}

/// Per-drive PSDs are counts²/Hz on drives whose counts_per_mm may differ,
/// so averaging happens in µm²/Hz — each drive converted first, then the
/// power mean — and only then collapses to a tone amplitude.
function psdFerrUm2(step: PlotStep, runName: string, drive: string): number[] {
  const umPerCount = 1000 / countsPerMm(runName, drive);
  if (!step.psd) throw new Error(`${step.name}: step has no psd`);
  return step.psd.per_drive[drive].map((p) => p * umPerCount * umPerCount);
}

function psdFerrTraces(names: string[], plots: PlotSeries[], steps: string[]): PsdTrace[] {
  const traces: PsdTrace[] = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName, j) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step || !step.psd) return;
      const psd = step.psd;
      const driveNames = Object.keys(psd.per_drive);
      if (!driveNames.length) return;
      const style = traceStyle(names, steps, i, j);
      const pushTrace = (psdUm2: number[], color: string, label: string) => {
        const clipped = clipToPsdBand(psd.freq_hz, psdUm2);
        traces.push({
          freq: clipped.freq,
          y: psdToAmplitude(clipped.freq, clipped.y),
          color,
          dashed: false,
          label,
          run: names[i],
        });
      };
      if (motorViewPerMotor()) {
        driveNames.forEach((drive, k) => {
          pushTrace(
            psdFerrUm2(step, names[i], drive),
            mixColor(style.color, "#ffffff", driveRamp(driveNames.length, k)),
            `${style.name} (${drive})`
          );
        });
        return;
      }
      const avgUm2: number[] = new Array(psd.freq_hz.length).fill(0);
      for (const drive of driveNames) {
        psdFerrUm2(step, names[i], drive).forEach((v, n) => (avgUm2[n] += v));
      }
      pushTrace(
        avgUm2.map((v) => v / driveNames.length),
        style.color,
        `${style.name} (avg of ${driveNames.length})`
      );
    });
  });
  return traces;
}

function psdAccelTraces(names: string[], plots: PlotSeries[], steps: string[]): PsdTrace[] {
  const traces: PsdTrace[] = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName, j) => {
      const step = p.steps.find((s) => s.name === stepName);
      const accel = step && step.psd && step.psd.accel;
      if (!accel) return;
      const style = traceStyle(names, steps, i, j);
      const clipped = clipToPsdBand(accel.freq_hz, accel.psd);
      traces.push({
        freq: clipped.freq,
        y: psdToAmplitude(clipped.freq, clipped.y),
        color: style.color,
        dashed: false,
        label: `${style.name} (accel)`,
        run: names[i],
      });
    });
  });
  return traces;
}

function fmtLinear(v: number): string {
  if (v === 0) return "0";
  const a = Math.abs(v);
  return a >= 1000 || a < 0.01 ? v.toExponential(1) : v.toPrecision(3);
}

interface PsdBoxOpts {
  linear?: boolean;
  zeroFloor?: boolean;
  fixedY?: FixedY | null;
  threshold?: number | null;
  markers?: FreqMarker[] | null;
}

function psdBox(
  title: string,
  traces: PsdTrace[],
  band: [number, number] | null,
  yTitle: string,
  opts?: PsdBoxOpts
): HTMLDivElement {
  opts = opts || {};
  const box = document.createElement("div");
  box.className = "chart-box";
  const head = document.createElement("h3");
  head.textContent = title;
  box.appendChild(head);
  const plotHost = document.createElement("div");
  box.appendChild(plotHost);
  if (traces.length) {
    psdPlot(plotHost, {
      width: 860,
      height: 280,
      traces,
      band,
      yTitle,
      linear: opts.linear,
      zeroFloor: opts.zeroFloor,
      fixedY: opts.fixedY,
      threshold: opts.threshold,
      markers: opts.markers,
      formatValue: opts.linear ? fmtLinear : (v: number) => v.toExponential(2),
    });
  }
  const legend = document.createElement("div");
  legend.className = "legend";
  traces.forEach((tr) => {
    const item = document.createElement("span");
    item.innerHTML = `<span class="swatch" style="background:${tr.color}"></span>${tr.label}`;
    legend.appendChild(item);
  });
  box.appendChild(legend);
  return box;
}

function renderPsdChart(names: string[], plots: PlotSeries[], steps: string[]) {
  const container = el("psd-charts");
  if (!container) return;
  const sig = { runs: runDataSig(names), steps, view: motorView(), maxHz: psdMaxFreqHz() };
  if (payloadUnchanged("psd-charts", sig)) return;
  container.innerHTML = "";
  if (!names.length || !steps.length) {
    container.innerHTML = '<p class="note">select runs above</p>';
    return;
  }
  const psdOpts = { linear: true, zeroFloor: true };
  const ferr = psdFerrTraces(names, plots, steps);
  container.appendChild(
    psdBox("following error", ferr, RESONANCE_BAND_HZ, "ferr amplitude (µm)", psdOpts)
  );
  const accel = psdAccelTraces(names, plots, steps);
  if (accel.length) {
    container.appendChild(
      psdBox("accelerometer", accel, RESONANCE_BAND_HZ, "accel amplitude", psdOpts)
    );
  }
}

function visibleStepNames(stepNames: string[]): string[] {
  if (!state.stepFilter) return stepNames;
  const filter = state.stepFilter;
  const kept = stepNames.filter((s) => filter.has(s));
  return kept.length ? kept : stepNames;
}

/// The one step filter drives every chart that splits by step (PSD, time
/// domain, metrics), so its chips render into every section that has a
/// container for them — otherwise a filter picked on one page silently
/// shapes another page's chart with no control in sight.
/// Motor chips gate which drives the per-motor time charts draw; the combined
/// view folds drives together, so the chips vanish outside per-motor mode.
function renderMotorChips(motorNames: string[]) {
  const container = el("time-motor-chips");
  if (!container) return;
  const show = motorViewPerMotor() && motorNames.length > 1;
  const filter = state.motorFilter ? [...state.motorFilter] : null;
  if (payloadUnchanged("time-motor-chips", { motorNames, filter, show })) return;
  container.innerHTML = "";
  if (!show) return;
  const all = document.createElement("button");
  all.className = "chip" + (state.motorFilter ? "" : " active");
  all.textContent = "all motors";
  all.title = "show every motor";
  all.addEventListener("click", () => {
    state.motorFilter = null;
    redrawCharts();
  });
  container.appendChild(all);
  for (const motor of motorNames) {
    const chip = document.createElement("button");
    const inFilter = state.motorFilter && state.motorFilter.has(motor);
    chip.className = "chip" + (inFilter ? " active" : "");
    chip.textContent = motor;
    chip.title = "click: only this motor — shift+click: add/remove it";
    chip.addEventListener("click", (ev) => {
      if (ev.shiftKey) {
        const next = new Set(state.motorFilter || motorNames);
        if (next.has(motor)) next.delete(motor);
        else next.add(motor);
        state.motorFilter = next.size === 0 || next.size === motorNames.length ? null : next;
      } else if (inFilter && state.motorFilter && state.motorFilter.size === 1) {
        state.motorFilter = null;
      } else {
        state.motorFilter = new Set([motor]);
      }
      redrawCharts();
    });
    container.appendChild(chip);
  }
}

function renderStepChips(stepNames: string[]) {
  for (const id of ["psd-step-chips", "time-step-chips"]) {
    const container = el(id);
    if (!container) continue;
    const filter = state.stepFilter ? [...state.stepFilter] : null;
    if (payloadUnchanged(`step-chips-${id}`, { stepNames, filter })) continue;
    fillStepChips(container, stepNames);
  }
}

function fillStepChips(container: HTMLElement, stepNames: string[]) {
  container.innerHTML = "";
  const all = document.createElement("button");
  all.className = "chip" + (state.stepFilter ? "" : " active");
  all.textContent = "all";
  all.title = "show every step";
  all.addEventListener("click", () => {
    state.stepFilter = null;
    redrawCharts();
  });
  container.appendChild(all);
  for (const stepName of stepNames) {
    const chip = document.createElement("button");
    const inFilter = state.stepFilter && state.stepFilter.has(stepName);
    chip.className = "chip" + (inFilter ? " active" : "");
    chip.textContent = stepName;
    chip.title = "click: only this step — shift+click: add/remove it";
    chip.addEventListener("click", (ev) => {
      if (ev.shiftKey) {
        const next = new Set(state.stepFilter || stepNames);
        if (next.has(stepName)) next.delete(stepName);
        else next.add(stepName);
        state.stepFilter = next.size === 0 || next.size === stepNames.length ? null : next;
      } else if (inFilter && state.stepFilter && state.stepFilter.size === 1) {
        state.stepFilter = null;
      } else {
        state.stepFilter = new Set([stepName]);
      }
      redrawCharts();
    });
    container.appendChild(chip);
  }
}

export type { MetricsRow, PsdBoxOpts, SweepSeries };
export { driveMoveSummary, settleCellHtml, torqueCellHtml, metricsDriveRow, foldDriveRows, metricsTableRows, heatCellStyle, renderMetricsTable, sweptAxisKey, sweepMetricsSeries, renderSweepMetricsChart, driveRamp, psdFerrUm2, psdFerrTraces, psdAccelTraces, fmtLinear, psdBox, renderPsdChart, visibleStepNames, renderStepChips, fillStepChips };
