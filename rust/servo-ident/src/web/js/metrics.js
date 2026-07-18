import { el, payloadUnchanged, runDataSig } from "./api.js";
import { drawChart, attachChartHover, mixColor, traceStyle, clipToPsdBand, psdMaxFreqHz, psdToAmplitude, countsPerMm } from "./charts-core.js";
import { redrawCharts } from "./peaks.js";
import { runColor } from "./runs.js";
import { motorView, motorViewPerMotor } from "./shell.js";
import { PALETTE, RESONANCE_BAND_HZ, state } from "./state.js";

// --- tracking metrics table -----------------------------------------------
//
// The replacement for the old gain-report PNG's "metrics vs gain" panel:
// results.json already carries per-drive, per-move ferr/overshoot/settle and
// the torque summary — this table is the view on top of them.

function driveMoveSummary(metrics) {
  const s = {
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

function settleCellHtml(s) {
  if (s.neverSettled) return `<span class="badge resonance">never</span>`;
  const truncatedBadge =
    `<span class="badge truncated" title="the capture ended inside a move's ` +
    `settle window, so the worst settle may be underestimated">truncated</span>`;
  if (s.settleWorstMs == null) return s.truncated ? truncatedBadge : "—";
  const value = `${s.settleWorstMs.toFixed(1)} ms`;
  return s.truncated ? `${value} ${truncatedBadge}` : value;
}

function torqueCellHtml(tq) {
  const peak = `${tq.peak_pct_rated.toFixed(0)}%`;
  if (!tq.rail_detected) return peak;
  return (
    `${peak} <span class="badge torque" title="on the rail ${tq.rail_pct_moving.toFixed(1)}% ` +
    `of moving time (${tq.rail_ms.toFixed(0)} ms, longest burst ${tq.longest_burst_ms.toFixed(0)} ms)">rail</span>`
  );
}

function metricsDriveRow(name, stepName, drive, dr) {
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
function foldDriveRows(driveRows, view) {
  const fold = (values) =>
    view === "avg"
      ? values.reduce((a, b) => a + b, 0) / values.length
      : Math.max(...values);
  const settled = driveRows
    .map((r) => r.settle.settleWorstMs)
    .filter((v) => v != null);
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

function metricsTableRows(names, steps) {
  const view = motorView();
  const rows = [];
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
function heatCellStyle(value, min, max) {
  if (!(max > min)) return "";
  const alpha = (0.32 * (value - min)) / (max - min);
  return alpha < 0.02 ? "" : ` style="background:rgba(224,90,79,${alpha.toFixed(3)})"`;
}

function renderMetricsTable(names, steps) {
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
  const columns = ["ferrPeakUm", "ferrRmsUm", "overshootUm"];
  const bounds = {};
  for (const c of columns) {
    const values = rows.map((r) => r[c]);
    bounds[c] = { min: Math.min(...values), max: Math.max(...values) };
  }
  const heat = (c, r) => heatCellStyle(r[c], bounds[c].min, bounds[c].max);
  const stepColors = new Map();
  for (const r of rows) {
    if (!stepColors.has(r.step)) {
      stepColors.set(r.step, PALETTE[stepColors.size % PALETTE.length]);
    }
  }
  const body = rows
    .map((r, i) => {
      const swatch = `<span class="swatch" style="background:${runColor(r.run)}"></span>`;
      const stepColor = stepColors.get(r.step);
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

function sweptAxisKey(manifest) {
  if (!manifest || manifest.steps.length < 2) return null;
  const keys = Object.keys(manifest.steps[0].swept || {}).filter((k) =>
    manifest.steps.every((s) => typeof (s.swept || {})[k] === "number")
  );
  const varying = keys.filter(
    (k) => new Set(manifest.steps.map((s) => s.swept[k])).size > 1
  );
  if (!varying.length) return null;
  return varying.includes("speed") ? "speed" : varying[0];
}

function sweepMetricsSeries(names) {
  const series = [];
  for (const name of names) {
    const detail = state.details.get(name);
    if (!detail || !detail.results || !detail.manifest) continue;
    const key = sweptAxisKey(detail.manifest);
    if (!key) continue;
    const sweptByStep = new Map(detail.manifest.steps.map((s) => [s.name, s.swept[key]]));
    const perDrivePoints = new Map();
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
      const stepPoints = new Map();
      if (view === "per-motor") {
        for (const v of driveValues) stepPoints.set(v.drive, v);
      } else if (driveValues.length) {
        const fold = (f) =>
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
        perDrivePoints.get(drive).push({ x: sweptByStep.get(step.name), flagged, ...p });
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

function renderSweepMetricsChart(names) {
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
  const viewLabel = { agg: "worst-drive", avg: "avg", "per-motor": "per-motor" }[motorView()];
  title.textContent = `${viewLabel} metrics vs swept ${series[0].key} (µm)`;
  box.appendChild(title);
  const canvas = document.createElement("canvas");
  canvas.width = 860;
  canvas.height = 260;
  box.appendChild(canvas);
  const legend = document.createElement("div");
  legend.className = "legend";
  const traces = [];
  const marks = [];
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
  const sweepOpts = { xTitle: series[0].key };
  drawChart(canvas, traces, "µm", null, "", marks, sweepOpts);
  attachChartHover(canvas, traces, "µm", null, "", marks, sweepOpts);
  box.appendChild(legend);
  container.appendChild(box);
}

function driveRamp(count, idx) {
  return count > 1 ? (0.5 * idx) / (count - 1) : 0;
}

/// Per-drive PSDs are counts²/Hz on drives whose counts_per_mm may differ,
/// so averaging happens in µm²/Hz — each drive converted first, then the
/// power mean — and only then collapses to a tone amplitude.
function psdFerrUm2(step, runName, drive) {
  const umPerCount = 1000 / countsPerMm(runName, drive);
  return step.psd.per_drive[drive].map((p) => p * umPerCount * umPerCount);
}

function psdFerrTraces(names, plots, steps) {
  const traces = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName, j) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step || !step.psd) return;
      const driveNames = Object.keys(step.psd.per_drive);
      if (!driveNames.length) return;
      const style = traceStyle(names, steps, i, j);
      const pushTrace = (psdUm2, color, label) => {
        const clipped = clipToPsdBand(step.psd.freq_hz, psdUm2);
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
      const avgUm2 = new Array(step.psd.freq_hz.length).fill(0);
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

function psdAccelTraces(names, plots, steps) {
  const traces = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName, j) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step || !step.psd || !step.psd.accel) return;
      const style = traceStyle(names, steps, i, j);
      const clipped = clipToPsdBand(step.psd.accel.freq_hz, step.psd.accel.psd);
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

function fmtLinear(v) {
  if (v === 0) return "0";
  const a = Math.abs(v);
  return a >= 1000 || a < 0.01 ? v.toExponential(1) : v.toPrecision(3);
}

function drawPsdChart(canvas, traces, band, yTitle, hover, opts) {
  opts = opts || {};
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || canvas.width;
  const h = canvas.clientHeight || canvas.height;
  const backingW = Math.round(w * dpr);
  const backingH = Math.round(h * dpr);
  if (canvas.width !== backingW || canvas.height !== backingH) {
    canvas.width = backingW;
    canvas.height = backingH;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
  const pad = { l: 46, r: 8, t: 8, b: 22 };
  const EPS = 1e-6;
  const toV = (raw) => (opts.linear ? raw : Math.log10(Math.max(raw, EPS)));
  let fMin = Infinity, fMax = -Infinity, vMin = Infinity, vMax = -Infinity;
  for (const tr of traces) {
    for (let i = 0; i < tr.freq.length; i++) {
      const f = tr.freq[i];
      const v = toV(tr.y[i]);
      fMin = Math.min(fMin, f);
      fMax = Math.max(fMax, f);
      vMin = Math.min(vMin, v);
      vMax = Math.max(vMax, v);
    }
  }
  if (opts.zeroFloor) vMin = 0;
  if (opts.fixedY) {
    vMin = opts.fixedY.yMin;
    vMax = opts.fixedY.yMax;
  }
  if (!isFinite(fMin) || !isFinite(vMin)) return;
  if (vMin === vMax) { vMin -= 1; vMax += 1; }
  const x = (f) => pad.l + ((f - fMin) / (fMax - fMin || 1)) * (w - pad.l - pad.r);
  const yOfV = (v) => h - pad.b - ((v - vMin) / (vMax - vMin || 1)) * (h - pad.t - pad.b);
  const y = (raw) => yOfV(toV(raw));

  if (band) {
    const [blo, bhi] = band;
    const bandLo = Math.max(blo, fMin);
    const bandHi = Math.min(bhi, fMax);
    if (bandHi > bandLo) {
      ctx.fillStyle = "rgba(217, 164, 65, 0.10)";
      ctx.fillRect(x(bandLo), pad.t, x(bandHi) - x(bandLo), h - pad.t - pad.b);
    }
  }

  ctx.strokeStyle = "#29313a";
  ctx.fillStyle = "#8a97a3";
  ctx.font = "10px monospace";
  ctx.beginPath();
  for (let i = 0; i <= 4; i++) {
    const v = vMin + ((vMax - vMin) * i) / 4;
    const py = yOfV(v);
    ctx.moveTo(pad.l, py);
    ctx.lineTo(w - pad.r, py);
    ctx.fillText(opts.linear ? fmtLinear(v) : `1e${v.toFixed(1)}`, 2, py + 3);
  }
  for (let i = 0; i <= 4; i++) {
    const f = fMin + ((fMax - fMin) * i) / 4;
    const px = x(f);
    ctx.fillText(f.toFixed(0) + "Hz", px, h - 6);
  }
  ctx.stroke();

  if (opts.threshold != null) {
    ctx.strokeStyle = "#d9a441";
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    const py = y(opts.threshold);
    ctx.moveTo(pad.l, py);
    ctx.lineTo(w - pad.r, py);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  for (const tr of traces) {
    ctx.strokeStyle = tr.color;
    ctx.lineWidth = 1.25;
    ctx.setLineDash(tr.dashed ? [4, 3] : []);
    ctx.beginPath();
    for (let i = 0; i < tr.freq.length; i++) {
      const px = x(tr.freq[i]);
      const py = y(tr.y[i]);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();
  }
  ctx.setLineDash([]);

  if (opts.markers) {
    ctx.setLineDash([3, 3]);
    opts.markers.forEach((m, idx) => {
      if (m.freq < fMin || m.freq > fMax) return;
      const px = x(m.freq);
      ctx.strokeStyle = "#b388ff";
      ctx.beginPath();
      ctx.moveTo(px, pad.t);
      ctx.lineTo(px, h - pad.b);
      ctx.stroke();
      ctx.fillStyle = "#b388ff";
      ctx.fillText(m.label, px + 4, pad.t + 12 + (idx % 3) * 10);
    });
    ctx.setLineDash([]);
  }

  if (band) {
    const [blo, bhi] = band;
    traces.forEach((tr, idx) => {
      let bestI = -1, bestV = -Infinity;
      for (let i = 0; i < tr.freq.length; i++) {
        if (tr.freq[i] >= blo && tr.freq[i] < bhi && tr.y[i] > bestV) {
          bestV = tr.y[i];
          bestI = i;
        }
      }
      if (bestI < 0) return;
      const px = x(tr.freq[bestI]);
      const py = y(tr.y[bestI]);
      ctx.fillStyle = tr.color;
      ctx.beginPath();
      ctx.arc(px, py, 2.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillText(`${tr.freq[bestI].toFixed(0)}Hz`, px + 4, py - 4 - (idx % 3) * 10);
    });
  }

  ctx.fillStyle = "#8a97a3";
  ctx.fillText(yTitle, pad.l, 10);

  if (hover) {
    let best = null;
    for (const tr of traces) {
      for (let i = 0; i < tr.freq.length; i++) {
        const dx = x(tr.freq[i]) - hover.mx;
        const dy = y(tr.y[i]) - hover.my;
        const d = dx * dx + dy * dy;
        if (!best || d < best.d) best = { d, tr, i };
      }
    }
    if (best) {
      const px = x(best.tr.freq[best.i]);
      const py = y(best.tr.y[best.i]);
      ctx.strokeStyle = "#4a5560";
      ctx.beginPath();
      ctx.moveTo(px, pad.t);
      ctx.lineTo(px, h - pad.b);
      ctx.stroke();
      ctx.fillStyle = best.tr.color;
      ctx.beginPath();
      ctx.arc(px, py, 3, 0, Math.PI * 2);
      ctx.fill();
      const value = opts.linear
        ? fmtLinear(best.tr.y[best.i])
        : best.tr.y[best.i].toExponential(2);
      const text = `${best.tr.freq[best.i].toFixed(1)} Hz  ${value}  ${best.tr.label}`;
      const tw = ctx.measureText(text).width;
      const tx = Math.min(Math.max(px + 8, pad.l), w - pad.r - tw - 8);
      const ty = Math.max(py - 10, pad.t + 12);
      ctx.fillStyle = "#0d1117";
      ctx.fillRect(tx - 4, ty - 10, tw + 8, 14);
      ctx.strokeStyle = "#4a5560";
      ctx.strokeRect(tx - 4, ty - 10, tw + 8, 14);
      ctx.fillStyle = "#e6edf3";
      ctx.fillText(text, tx, ty);
    }
  }
}

/// Redraws with the cursor position on every move — the readout follows
/// the nearest sample, so exact peak frequencies are readable instead of
/// eyeballed off the axis.
function attachPsdHover(canvas, traces, band, yTitle, opts) {
  canvas.addEventListener("mousemove", (e) => {
    drawPsdChart(canvas, traces, band, yTitle, { mx: e.offsetX, my: e.offsetY }, opts);
  });
  canvas.addEventListener("mouseleave", () => drawPsdChart(canvas, traces, band, yTitle, null, opts));
}

function psdBox(title, traces, band, yTitle, opts) {
  const box = document.createElement("div");
  box.className = "chart-box";
  const head = document.createElement("h3");
  head.textContent = title;
  box.appendChild(head);
  const canvas = document.createElement("canvas");
  canvas.width = 860;
  canvas.height = 280;
  box.appendChild(canvas);
  drawPsdChart(canvas, traces, band, yTitle, null, opts);
  attachPsdHover(canvas, traces, band, yTitle, opts);
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

function renderPsdChart(names, plots, steps) {
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

function visibleStepNames(stepNames) {
  if (!state.stepFilter) return stepNames;
  const kept = stepNames.filter((s) => state.stepFilter.has(s));
  return kept.length ? kept : stepNames;
}

/// The one step filter drives every chart that splits by step (PSD, time
/// domain, metrics), so its chips render into every section that has a
/// container for them — otherwise a filter picked on one page silently
/// shapes another page's chart with no control in sight.
function renderStepChips(stepNames) {
  for (const id of ["psd-step-chips", "time-step-chips"]) {
    const container = el(id);
    if (!container) continue;
    const filter = state.stepFilter ? [...state.stepFilter] : null;
    if (payloadUnchanged(`step-chips-${id}`, { stepNames, filter })) continue;
    fillStepChips(container, stepNames);
  }
}

function fillStepChips(container, stepNames) {
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
      } else if (inFilter && state.stepFilter.size === 1) {
        state.stepFilter = null;
      } else {
        state.stepFilter = new Set([stepName]);
      }
      redrawCharts();
    });
    container.appendChild(chip);
  }
}

export { driveMoveSummary, settleCellHtml, torqueCellHtml, metricsDriveRow, foldDriveRows, metricsTableRows, heatCellStyle, renderMetricsTable, sweptAxisKey, sweepMetricsSeries, renderSweepMetricsChart, driveRamp, psdFerrUm2, psdFerrTraces, psdAccelTraces, fmtLinear, drawPsdChart, attachPsdHover, psdBox, renderPsdChart, visibleStepNames, renderStepChips, fillStepChips };
