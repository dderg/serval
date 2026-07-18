import { el, payloadUnchanged, runDataSig } from "./api";
import { driveRamp } from "./metrics";
import { runColor } from "./runs";
import { motorViewPerMotor } from "./shell";
import { PALETTE, PSD_MAX_FREQ_KEY, PSD_MAX_FREQ_CHOICES_HZ, PSD_MAX_FREQ_DEFAULT_HZ, state } from "./state";
import { timeSeriesPlot } from "./uplot-chart";
import type { TimeTrace } from "./uplot-chart";
import type { PlotSeries, PlotStep } from "./wire";

// --- chart drawing ------------------------------------------------------------

interface PickedSeries {
  y: (number | null)[];
  label: string;
  suffix: string;
  ramp: number;
}

function pickSeries(runName: string, step: PlotStep): PickedSeries[] {
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
function hidpiCanvasContext(canvas: HTMLCanvasElement): { ctx: CanvasRenderingContext2D; w: number; h: number } {
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
  if (!ctx) throw new Error("canvas has no 2d context");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

function drawTimeDomain(names: string[], plots: PlotSeries[], steps: string[]) {
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

    const traces: TimeTrace[] = [];
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

function newestSelectedRunName(names: string[]): string {
  const selected = new Set(names);
  const found = state.runs.find((r) => selected.has(r.name));
  return found ? found.name : names[0];
}

/// The peak list needs one step to harvest from: the newest selected run's
/// recommended step when it is visible, else its last visible step.
function peakStep(names: string[], plots: PlotSeries[], steps: string[]): { newest: string; step: string | null } {
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

function mixColor(hex: string, targetHex: string, t: number): string {
  const c = parseInt(hex.slice(1), 16);
  const g = parseInt(targetHex.slice(1), 16);
  const mix = (shift: number) => {
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
function traceStyle(names: string[], steps: string[], runIdx: number, stepIdx: number): { color: string; name: string } {
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

function clipToPsdBand(freq: number[], y: number[]): { freq: number[]; y: number[] } {
  const maxHz = psdMaxFreqHz();
  let end = freq.length;
  while (end > 0 && freq[end - 1] > maxHz) end--;
  return { freq: freq.slice(0, end), y: y.slice(0, end) };
}

/// Welch PSD -> single-sided tone amplitude: a sinusoid of amplitude A puts
/// A²/2 of power into its bin's equivalent noise bandwidth, so
/// A = sqrt(2 · psd · ENBW) with ENBW = 1.5·Δf for the analyzer's Hann window.
const WELCH_HANN_ENBW_BINS = 1.5;

function psdToAmplitude(freq: number[], psd: number[]): number[] {
  if (freq.length < 2) throw new Error("psd grid too short for a bin width");
  const factor = Math.sqrt(2 * WELCH_HANN_ENBW_BINS * (freq[1] - freq[0]));
  return psd.map((p) => Math.sqrt(p) * factor);
}

function countsPerMm(runName: string, driveName: string): number {
  const detail = state.details.get(runName);
  const motors = (detail && detail.manifest && detail.manifest.motors) || [];
  const motor = motors.find((m) => m.name === driveName);
  if (!motor || !motor.counts_per_mm) {
    throw new Error(`${runName}: manifest has no counts_per_mm for ${driveName}`);
  }
  return motor.counts_per_mm;
}

export { pickSeries, hidpiCanvasContext, drawTimeDomain, newestSelectedRunName, peakStep, mixColor, traceStyle, psdMaxFreqHz, clipToPsdBand, WELCH_HANN_ENBW_BINS, psdToAmplitude, countsPerMm };
