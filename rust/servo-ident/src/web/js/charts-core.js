import { el, payloadUnchanged, runDataSig } from "./api.js";
import { driveRamp } from "./metrics.js";
import { runColor } from "./runs.js";
import { motorViewPerMotor } from "./shell.js";
import { PALETTE, PSD_MAX_FREQ_KEY, PSD_MAX_FREQ_CHOICES_HZ, PSD_MAX_FREQ_DEFAULT_HZ, state } from "./state.js";
import { timeSeriesPlot } from "./uplot-chart.js";

// --- chart drawing ------------------------------------------------------------

function pickSeries(runName, step) {
  if (motorViewPerMotor()) {
    const drives = Object.entries(step.drives);
    return drives.map(([drive, d], k) => ({
      y: d.ferr_counts.map((c) => c * (1000 / countsPerMm(runName, drive))),
      label: "ferr (µm)",
      suffix: ` (${drive})`,
      ramp: driveRamp(drives.length, k),
    }));
  }
  if (step.combined) {
    return [{ y: step.combined.on_ferr_mm, label: "on-axis ferr (mm)", suffix: "", ramp: 0 }];
  }
  const firstDrive = Object.values(step.drives)[0];
  return [
    {
      y: firstDrive ? firstDrive.ferr_counts : [],
      label: "ferr (counts)",
      suffix: "",
      ramp: 0,
    },
  ];
}

/// Renders at the device pixel ratio so lines stay vector-crisp on hidpi
/// displays: the backing store is sized to the CSS box × dpr and the
/// context scaled back, while all layout math stays in CSS pixels.
function drawChart(canvas, traces, yLabel, fixedY, xUnit) {
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
  let tMin = Infinity, tMax = -Infinity, yMin = Infinity, yMax = -Infinity;
  for (const tr of traces) {
    for (let i = 0; i < tr.t.length; i++) {
      if (tr.y[i] === null) continue;
      tMin = Math.min(tMin, tr.t[i]);
      tMax = Math.max(tMax, tr.t[i]);
      yMin = Math.min(yMin, tr.y[i]);
      yMax = Math.max(yMax, tr.y[i]);
    }
  }
  if (fixedY) {
    yMin = fixedY.yMin;
    yMax = fixedY.yMax;
  }
  if (!isFinite(tMin) || !isFinite(yMin)) return;
  if (yMin === yMax) { yMin -= 1; yMax += 1; }
  const x = (t) => pad.l + ((t - tMin) / (tMax - tMin || 1)) * (w - pad.l - pad.r);
  const y = (v) => h - pad.b - ((v - yMin) / (yMax - yMin || 1)) * (h - pad.t - pad.b);

  const fmtTick = (v, span) => (Math.abs(span) >= 20 ? v.toFixed(0) : v.toFixed(2));
  ctx.strokeStyle = "#29313a";
  ctx.fillStyle = "#8a97a3";
  ctx.font = "10px monospace";
  ctx.beginPath();
  for (let i = 0; i <= 4; i++) {
    const v = yMin + ((yMax - yMin) * i) / 4;
    const py = y(v);
    ctx.moveTo(pad.l, py);
    ctx.lineTo(w - pad.r, py);
    ctx.fillText(fmtTick(v, yMax - yMin), 2, py + 3);
  }
  for (let i = 0; i <= 4; i++) {
    const t = tMin + ((tMax - tMin) * i) / 4;
    const px = x(t);
    ctx.fillText(fmtTick(t, tMax - tMin) + (xUnit == null ? "s" : xUnit), px, h - 6);
  }
  ctx.stroke();

  for (const tr of traces) {
    ctx.strokeStyle = tr.color;
    ctx.lineWidth = 1.25;
    ctx.setLineDash(tr.dash || []);
    ctx.beginPath();
    let penDown = false;
    for (let i = 0; i < tr.t.length; i++) {
      if (tr.y[i] === null) {
        penDown = false;
        continue;
      }
      const px = x(tr.t[i]);
      const py = y(tr.y[i]);
      if (penDown) ctx.lineTo(px, py);
      else ctx.moveTo(px, py);
      penDown = true;
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.fillStyle = "#8a97a3";
  ctx.fillText(yLabel, pad.l, 10);
}

function drawTimeDomain(names, plots, steps) {
  const container = el("charts");
  if (!container) return;
  const sig = { runs: runDataSig(names), steps, perMotor: motorViewPerMotor() };
  if (payloadUnchanged("time-domain", sig)) return;
  container.innerHTML = "";
  if (names.length === 0) {
    container.innerHTML = '<p class="note">select runs above</p>';
    return;
  }
  for (const stepName of steps) {
    const box = document.createElement("div");
    box.className = "chart-box";
    const title = document.createElement("h3");
    title.textContent = stepName;
    box.appendChild(title);
    const plotHost = document.createElement("div");
    box.appendChild(plotHost);
    const legend = document.createElement("div");
    legend.className = "legend";

    const traces = [];
    let yLabel = "";
    plots.forEach((p, i) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step) return;
      for (const series of pickSeries(names[i], step)) {
        yLabel = series.label;
        const color = mixColor(runColor(names[i]), "#ffffff", series.ramp);
        traces.push({ t: step.t_s, y: series.y, color });
        const item = document.createElement("span");
        item.innerHTML =
          `<span class="swatch" style="background:${color}"></span>` +
          `${names[i]}${series.suffix}`;
        legend.appendChild(item);
      }
    });
    if (traces.length) {
      timeSeriesPlot(plotHost, {
        width: container.clientWidth || 860,
        height: 200,
        yLabel,
        traces,
      });
    }
    box.appendChild(legend);
    container.appendChild(box);
  }
}

// --- following-error PSD --------------------------------------------------

function newestSelectedRunName(names) {
  const selected = new Set(names);
  const found = state.runs.find((r) => selected.has(r.name));
  return found ? found.name : names[0];
}

/// The peak list needs one step to harvest from: the newest selected run's
/// recommended step when it is visible, else its last visible step.
function peakStep(names, plots, steps) {
  const newest = newestSelectedRunName(names);
  const plot = plots[names.indexOf(newest)];
  const present = plot
    ? steps.filter((s) => plot.steps.some((x) => x.name === s))
    : [];
  const detail = state.details.get(newest);
  const recommended = detail && detail.results && detail.results.verdict.recommended_step;
  const step =
    recommended && present.includes(recommended)
      ? recommended
      : present.length
        ? present[present.length - 1]
        : null;
  return { newest, step };
}

function mixColor(hex, targetHex, t) {
  const c = parseInt(hex.slice(1), 16);
  const g = parseInt(targetHex.slice(1), 16);
  const mix = (shift) => {
    const a = (c >> shift) & 0xff;
    const b = (g >> shift) & 0xff;
    return Math.round(a + (b - a) * t);
  };
  return `#${((mix(16) << 16) | (mix(8) << 8) | mix(0)).toString(16).padStart(6, "0")}`;
}

/// One run selected: each step gets its own palette color, rotated so the
/// first step is exactly the run's table-swatch color — the swatch and the
/// chart must never disagree, whatever color the run ended up holding.
/// Several runs: each run keeps its table-swatch hue and its steps ramp
/// toward white, so runs stay distinguishable and the step chips are the
/// clutter valve.
function traceStyle(names, steps, runIdx, stepIdx) {
  if (names.length === 1) {
    const base = runColor(names[0]);
    const baseIdx = PALETTE.indexOf(base);
    if (baseIdx < 0) throw new Error(`${base}: run color is not in the palette`);
    return {
      color: PALETTE[(baseIdx + stepIdx) % PALETTE.length],
      name: steps[stepIdx],
    };
  }
  const base = runColor(names[runIdx]);
  const ramp = steps.length > 1 ? (0.55 * stepIdx) / (steps.length - 1) : 0;
  const name =
    steps.length === 1 ? names[runIdx] : `${names[runIdx]} · ${steps[stepIdx]}`;
  return { color: mixColor(base, "#ffffff", ramp), name };
}

/// Drawing the full Nyquist span squishes the servo/mechanical modes into
/// the left quarter of the chart, so the user picks the band ceiling.
function psdMaxFreqHz() {
  const stored = Number(localStorage.getItem(PSD_MAX_FREQ_KEY));
  return PSD_MAX_FREQ_CHOICES_HZ.includes(stored)
    ? stored
    : PSD_MAX_FREQ_DEFAULT_HZ;
}

function clipToPsdBand(freq, y) {
  const maxHz = psdMaxFreqHz();
  let end = freq.length;
  while (end > 0 && freq[end - 1] > maxHz) end--;
  return { freq: freq.slice(0, end), y: y.slice(0, end) };
}

/// Welch PSD -> single-sided tone amplitude: a sinusoid of amplitude A puts
/// A²/2 of power into its bin's equivalent noise bandwidth, so
/// A = sqrt(2 · psd · ENBW) with ENBW = 1.5·Δf for the analyzer's Hann window.
const WELCH_HANN_ENBW_BINS = 1.5;

function psdToAmplitude(freq, psd) {
  if (freq.length < 2) throw new Error("psd grid too short for a bin width");
  const factor = Math.sqrt(2 * WELCH_HANN_ENBW_BINS * (freq[1] - freq[0]));
  return psd.map((p) => Math.sqrt(p) * factor);
}

function countsPerMm(runName, driveName) {
  const detail = state.details.get(runName);
  const motors = detail && detail.manifest ? detail.manifest.motors : [];
  const motor = motors.find((m) => m.name === driveName);
  if (!motor || !motor.counts_per_mm) {
    throw new Error(`${runName}: manifest has no counts_per_mm for ${driveName}`);
  }
  return motor.counts_per_mm;
}

export { pickSeries, drawChart, drawTimeDomain, newestSelectedRunName, peakStep, mixColor, traceStyle, psdMaxFreqHz, clipToPsdBand, WELCH_HANN_ENBW_BINS, psdToAmplitude, countsPerMm };
