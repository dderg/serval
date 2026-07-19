import { html } from "htm/preact";
import { el, payloadUnchanged } from "./api";
import { detailData, runDataSig } from "./queries/runs";
import {
  fillFilterChips,
  mixColor,
  traceStyle,
  clipToPsdBand,
  psdMaxFreqHz,
  psdToAmplitude,
  countsPerMm,
  ferrUnitAvailability,
  resolvedFerrUnit,
  syncFerrUnitUi,
  ferrUnitToggleHtml,
  SectionHead,
} from "./charts-core";
import { psdPlot, timeSeriesPlot } from "./uplot-chart";
import type { FixedY, FreqMarker, Mark, PsdTrace, TimeTrace } from "./uplot-chart";
import type { DriveResult, Manifest, PlotSeries, PlotStep } from "./api/runs";
import { redrawCharts } from "./peaks";
import { runColor } from "./runs";
import { motorView, motorViewPerMotor, motorViewToggleHtml } from "./shell";
import { PSD_MAX_FREQ_CHOICES_HZ, RESONANCE_BAND_HZ, state } from "./state";
import type { FerrUnit } from "./units";

type DriveMetrics = DriveResult["metrics"];
type TorqueMetrics = DriveMetrics["torque"];

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
    const detail = detailData(name);
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
  const rowsByRun = new Map<string, MetricsRow[]>();
  for (const row of rows) {
    const runRows = rowsByRun.get(row.run);
    if (runRows) runRows.push(row);
    else rowsByRun.set(row.run, [row]);
  }
  const body = [...rowsByRun.entries()]
    .map(([run, runRows], runIndex) =>
      runRows
        .map((r, rowIndex) => {
          const runCell =
            rowIndex === 0
              ? `<td class="run-cell" rowspan="${runRows.length}" ` +
                `style="border-left:3px solid ${runColor(run)};padding-left:6px" ` +
                `title="${run}"><span class="swatch" style="background:${runColor(run)}"></span>${run}</td>`
              : "";
          return (
            `<tr${runIndex > 0 && rowIndex === 0 ? ' class="group-start"' : ""}>` +
            runCell +
            `<td>${r.step}</td><td>${r.drive}</td>` +
            `<td class="num"${heat("ferrPeakUm", r)}>${r.ferrPeakUm.toFixed(1)}</td>` +
            `<td class="num"${heat("ferrRmsUm", r)}>${r.ferrRmsUm.toFixed(1)}</td>` +
            `<td class="num"${heat("overshootUm", r)}>${r.overshootUm.toFixed(1)}</td>` +
            `<td class="num">${settleCellHtml(r.settle)}</td>` +
            `<td class="num">${torqueCellHtml(r.torque)}</td></tr>`
          );
        })
        .join("")
    )
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
    const detail = detailData(name);
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
/// so averaging in µm happens in µm²/Hz — each drive converted first, then
/// the power mean — and only then collapses to a tone amplitude. In counts
/// mode the raw counts²/Hz values feed the average directly.
function psdFerrScaled(step: PlotStep, runName: string, drive: string, unit: FerrUnit): number[] {
  if (!step.psd) throw new Error(`${step.name}: step has no psd`);
  if (unit === "counts") return step.psd.per_drive[drive];
  const umPerCount = 1000 / countsPerMm(runName, drive);
  return step.psd.per_drive[drive].map((p) => p * umPerCount * umPerCount);
}

function psdDrivePairs(names: string[], plots: PlotSeries[], steps: string[]): [string, string][] {
  const pairs: [string, string][] = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step || !step.psd) return;
      for (const drive of Object.keys(step.psd.per_drive)) pairs.push([names[i], drive]);
    });
  });
  return pairs;
}

function psdFerrTraces(names: string[], plots: PlotSeries[], steps: string[], unit: FerrUnit): PsdTrace[] {
  const traces: PsdTrace[] = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName, j) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step || !step.psd) return;
      const psd = step.psd;
      const driveNames = Object.keys(psd.per_drive);
      if (!driveNames.length) return;
      const style = traceStyle(names, steps, i, j);
      const pushTrace = (psdScaled: number[], color: string, label: string) => {
        const clipped = clipToPsdBand(psd.freq_hz, psdScaled);
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
            psdFerrScaled(step, names[i], drive, unit),
            mixColor(style.color, "#ffffff", driveRamp(driveNames.length, k)),
            `${style.name} (${drive})`
          );
        });
        return;
      }
      const avgScaled: number[] = new Array(psd.freq_hz.length).fill(0);
      for (const drive of driveNames) {
        psdFerrScaled(step, names[i], drive, unit).forEach((v, n) => (avgScaled[n] += v));
      }
      pushTrace(
        avgScaled.map((v) => v / driveNames.length),
        style.color,
        `${style.name} (avg of ${driveNames.length})`
      );
    });
  });
  return traces;
}

const CARTESIAN_UM2_PER_MM2 = 1e6;

function uniformCountsPerMm(runName: string): number {
  const detail = detailData(runName);
  const motors = (detail && detail.manifest && detail.manifest.motors) || [];
  const values = [
    ...new Set(motors.map((m) => m.counts_per_mm).filter((v): v is number => v != null && v > 0)),
  ];
  if (values.length !== 1) {
    throw new Error(
      `${runName}: cartesian counts view needs one shared counts_per_mm, manifest has ${values.length}`
    );
  }
  return values[0];
}

/// Cartesian-mode PSDs arrive in mm²/Hz (motor counts already projected
/// through the spatial frame server-side); µm view scales power by 1000²,
/// counts view maps back through the run's single counts_per_mm.
function psdCartesianScaled(psdMm2: number[], runName: string, unit: FerrUnit): number[] {
  if (unit === "counts") {
    const cpm = uniformCountsPerMm(runName);
    return psdMm2.map((p) => p * cpm * cpm);
  }
  return psdMm2.map((p) => p * CARTESIAN_UM2_PER_MM2);
}

function psdCartesianTraces(names: string[], plots: PlotSeries[], steps: string[], unit: FerrUnit): PsdTrace[] {
  const traces: PsdTrace[] = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName, j) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step || !step.psd || !step.psd.cartesian) return;
      const cartesian = step.psd.cartesian;
      const style = traceStyle(names, steps, i, j);
      const modes = Object.entries(cartesian);
      modes.forEach(([mode, psd], k) => {
        const clipped = clipToPsdBand(step.psd.freq_hz, psdCartesianScaled(psd, names[i], unit));
        traces.push({
          freq: clipped.freq,
          y: psdToAmplitude(clipped.freq, clipped.y),
          color: mixColor(style.color, "#ffffff", driveRamp(modes.length, k)),
          dashed: false,
          label: `${style.name} (${mode})`,
          run: names[i],
        });
      });
    });
  });
  return traces;
}

const ACCEL_AXIS_KEYS = ["psd_x", "psd_y", "psd_z"] as const;
const ACCEL_AXIS_LABELS = ["x", "y", "z"] as const;
const ACCEL_TRACE_KEYS = ["total", ...ACCEL_AXIS_LABELS] as const;
const ACCEL_TOTAL_WIDTH = 2.25;

function psdAccelTraces(names: string[], plots: PlotSeries[], steps: string[]): PsdTrace[] {
  const traces: PsdTrace[] = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName, j) => {
      const step = p.steps.find((s) => s.name === stepName);
      const accel = step && step.psd && step.psd.accel;
      if (!accel) return;
      const style = traceStyle(names, steps, i, j);
      const pushTrace = (psd: number[], color: string, label: string, opts: { dashed: boolean; width?: number }) => {
        const clipped = clipToPsdBand(accel.freq_hz, psd);
        traces.push({
          freq: clipped.freq,
          y: psdToAmplitude(clipped.freq, clipped.y),
          color,
          dashed: opts.dashed,
          width: opts.width,
          label,
          run: names[i],
        });
      };
      const visible = (k: string) => !state.accelAxisFilter || state.accelAxisFilter.has(k);
      if (visible("total")) {
        pushTrace(accel.psd, style.color, `${style.name} (total)`, { dashed: false, width: ACCEL_TOTAL_WIDTH });
      }
      ACCEL_AXIS_KEYS.forEach((key, k) => {
        if (!visible(ACCEL_AXIS_LABELS[k])) return;
        pushTrace(
          accel[key],
          mixColor(style.color, "#ffffff", driveRamp(ACCEL_AXIS_KEYS.length + 1, k + 1)),
          `${style.name} (${ACCEL_AXIS_LABELS[k]})`,
          { dashed: true }
        );
      });
    });
  });
  return traces;
}

function renderAccelAxisChips(show: boolean) {
  const container = el("accel-axis-chips");
  if (!container) return;
  const filter = state.accelAxisFilter ? [...state.accelAxisFilter] : null;
  if (payloadUnchanged("accel-axis-chips", { filter, show })) return;
  container.innerHTML = "";
  if (!show) return;
  fillFilterChips(
    container,
    "all",
    "show the total and every axis",
    "trace",
    ACCEL_TRACE_KEYS.map((k) => ({ key: k, label: k })),
    () => state.accelAxisFilter,
    (next) => {
      state.accelAxisFilter = next;
    },
    redrawCharts
  );
}

function renderAccelPsdChart(names: string[], plots: PlotSeries[], steps: string[]) {
  const section = el("accel-psd-section");
  const container = el("accel-psd-charts");
  if (!section || !container) return;
  const hasAccel = plots.some((p) =>
    p.steps.some((s) => steps.includes(s.name) && s.psd && s.psd.accel)
  );
  section.hidden = !hasAccel;
  renderAccelAxisChips(hasAccel);
  const filter = state.accelAxisFilter ? [...state.accelAxisFilter] : null;
  const sig = { runs: runDataSig(names), steps, maxHz: psdMaxFreqHz(), filter };
  if (payloadUnchanged("accel-psd-charts", sig)) return;
  container.innerHTML = "";
  if (!hasAccel) return;
  const traces = psdAccelTraces(names, plots, steps);
  container.appendChild(
    psdBox("accelerometer", traces, RESONANCE_BAND_HZ, "accel amplitude (mm/s²)", {
      linear: true,
      zeroFloor: true,
    })
  );
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
  const availability = ferrUnitAvailability(psdDrivePairs(names, plots, steps));
  syncFerrUnitUi("psd", availability);
  const unit = resolvedFerrUnit(availability);
  const sig = { runs: runDataSig(names), steps, view: motorView(), maxHz: psdMaxFreqHz(), unit };
  if (payloadUnchanged("psd-charts", sig)) return;
  container.innerHTML = "";
  if (!names.length || !steps.length) {
    container.innerHTML = '<p class="note">select runs above</p>';
    return;
  }
  const psdOpts = { linear: true, zeroFloor: true };
  const hasCartesian = plots.some((p) =>
    p.steps.some((s) => steps.includes(s.name) && s.psd && s.psd.cartesian)
  );
  const cartesianBtn = document.querySelector<HTMLButtonElement>(
    '.psd-section .motor-view-btn[data-view="cartesian"]'
  );
  if (cartesianBtn) {
    cartesianBtn.disabled = !hasCartesian;
    cartesianBtn.title = hasCartesian
      ? "project per-motor error through the spatial frame into cartesian axis modes"
      : "unavailable — the selected runs' manifests carry no spatial frame";
  }
  if (motorView() === "cartesian") {
    if (!hasCartesian) {
      container.innerHTML =
        '<p class="note">cartesian view needs a run whose manifest carries a spatial frame</p>';
      return;
    }
    const cartesian = psdCartesianTraces(names, plots, steps, unit);
    container.appendChild(
      psdBox("following error — cartesian modes", cartesian, RESONANCE_BAND_HZ, `ferr amplitude (${unit})`, psdOpts)
    );
    return;
  }
  const ferr = psdFerrTraces(names, plots, steps, unit);
  container.appendChild(
    psdBox("following error", ferr, RESONANCE_BAND_HZ, `ferr amplitude (${unit})`, psdOpts)
  );
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
  fillFilterChips(
    container,
    "all motors",
    "show every motor",
    "motor",
    motorNames.map((m) => ({ key: m, label: m })),
    () => state.motorFilter,
    (next) => {
      state.motorFilter = next;
    },
    redrawCharts
  );
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
  fillFilterChips(
    container,
    "all",
    "show every step",
    "step",
    stepNames.map((s) => ({ key: s, label: s })),
    () => state.stepFilter,
    (next) => {
      state.stepFilter = next;
    },
    redrawCharts
  );
}

function MetricsSection() {
  const tools =
    motorViewToggleHtml("worst drive", true) +
    `<span class="note">worst move of each step — ` +
    `overshoot/settle measured over the dwell after each move</span>`;
  return html`<section class="metrics-section">
    <${SectionHead} title="tracking metrics" tools=${tools} />
    <div id="metrics-table"></div>
  </section>`;
}

function SweepMetricsSection() {
  const tools =
    motorViewToggleHtml("worst drive", true) +
    `<span class="note">● solid: overshoot, dashed: ferr rms, ` +
    `dotted: ferr peak; red rung: step flagged resonance/torque</span>`;
  return html`<section class="sweep-metrics-section">
    <${SectionHead} title="metrics vs gain" tools=${tools} />
    <div class="charts" id="sweep-metrics-chart"></div>
  </section>`;
}

function PsdSection() {
  const psdMax =
    `<label class="note">to <select id="psd-max-freq">` +
    PSD_MAX_FREQ_CHOICES_HZ.map(
      (f) => `<option value="${f}"${f === psdMaxFreqHz() ? " selected" : ""}>${f}</option>`
    ).join("") +
    `</select> Hz</label>`;
  const tools =
    motorViewToggleHtml("avg", false, true) +
    psdMax +
    ferrUnitToggleHtml("psd") +
    `<div class="chips" id="psd-step-chips"></div>`;
  return html`<section class="psd-section">
    <${SectionHead} title="following-error PSD" tools=${tools} />
    <div class="charts" id="psd-charts"></div>
  </section>`;
}

function AccelPsdSection() {
  const tools =
    `<span class="note">per-axis accelerometer spectra; solid: x+y+z total</span>` +
    `<div class="chips" id="accel-axis-chips"></div>`;
  return html`<section class="accel-psd-section" id="accel-psd-section" hidden>
    <${SectionHead} title="accel PSD" tools=${tools} />
    <div class="charts" id="accel-psd-charts"></div>
  </section>`;
}

export type { MetricsRow, PsdBoxOpts, SweepSeries };
export { driveMoveSummary, settleCellHtml, torqueCellHtml, metricsDriveRow, foldDriveRows, metricsTableRows, heatCellStyle, renderMetricsTable, sweptAxisKey, sweepMetricsSeries, renderSweepMetricsChart, driveRamp, psdFerrScaled, psdDrivePairs, psdFerrTraces, uniformCountsPerMm, psdCartesianScaled, psdCartesianTraces, psdAccelTraces, renderAccelPsdChart, fmtLinear, psdBox, renderPsdChart, visibleStepNames, renderStepChips, renderMotorChips, fillStepChips, MetricsSection, SweepMetricsSection, PsdSection, AccelPsdSection };
