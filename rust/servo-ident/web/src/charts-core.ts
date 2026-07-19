import { html } from "htm/preact";
import { el, payloadUnchanged } from "./api";
import { detailData, runDataSig, runsData } from "./queries/runs";
import { driveRamp } from "./metrics";
import { runColor } from "./runs";
import { motorViewPerMotor, motorViewToggleHtml } from "./shell";
import { PALETTE, PSD_MAX_FREQ_KEY, PSD_MAX_FREQ_CHOICES_HZ, PSD_MAX_FREQ_DEFAULT_HZ, state } from "./state";
import { timeSeriesPlot } from "./uplot-chart";
import type { TimeTrace } from "./uplot-chart";
import type { PlotSeries, PlotStep } from "./api/runs";
import { loadFerrUnit, setFerrUnit } from "./units";
import type { FerrUnit } from "./units";

// --- chart drawing ------------------------------------------------------------

interface PickedSeries {
  y: (number | null)[];
  label: string;
  suffix: string;
  ramp: number;
}

function motorVisible(drive: string): boolean {
  return !state.motorFilter || state.motorFilter.has(drive);
}

function pickSeries(runName: string, step: PlotStep, unit: FerrUnit): PickedSeries[] {
  if (motorViewPerMotor()) {
    const drives = Object.entries(step.drives);
    const label = unit === "µm" ? "ferr (µm)" : "ferr (counts)";
    return drives
      .map(([drive, d], k) => ({
        y: unit === "µm" ? d.ferr_counts.map((c) => c * (1000 / countsPerMm(runName, drive))) : d.ferr_counts,
        label,
        suffix: ` (${drive})`,
        drive,
        ramp: driveRamp(drives.length, k),
      }))
      .filter((s) => motorVisible(s.drive));
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

function stepDrivePairs(names: string[], plots: PlotSeries[], steps: string[]): [string, string][] {
  const pairs: [string, string][] = [];
  plots.forEach((p, i) => {
    steps.forEach((stepName) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step) return;
      for (const drive of Object.keys(step.drives)) pairs.push([names[i], drive]);
    });
  });
  return pairs;
}

function ferrUnitAvailability(pairs: [string, string][]): { ok: boolean; missing: string[] } {
  const missing = [...new Set(pairs.filter(([run, drive]) => countsPerMmOrNull(run, drive) === null).map(([, d]) => d))];
  return { ok: pairs.length > 0 && missing.length === 0, missing };
}

function resolvedFerrUnit(availability: { ok: boolean }): FerrUnit {
  return loadFerrUnit() === "µm" && availability.ok ? "µm" : "counts";
}

function ferrUnitToggleHtml(idPrefix: string): string {
  return (
    `<span class="chips">` +
    `<button class="chip" id="${idPrefix}-unit-um" data-unit="µm">µm</button>` +
    `<button class="chip" id="${idPrefix}-unit-counts" data-unit="counts">counts</button>` +
    `</span>` +
    `<span class="note" id="${idPrefix}-unit-hint"></span>`
  );
}

function syncFerrUnitUi(idPrefix: string, availability: { ok: boolean; missing: string[] }) {
  const pref = loadFerrUnit();
  const umBtn = el<HTMLButtonElement>(`${idPrefix}-unit-um`);
  const countsBtn = el<HTMLButtonElement>(`${idPrefix}-unit-counts`);
  const hint = el(`${idPrefix}-unit-hint`);
  if (umBtn) {
    umBtn.disabled = !availability.ok;
    umBtn.classList.toggle("active", availability.ok && pref === "µm");
  }
  if (countsBtn) countsBtn.classList.toggle("active", !availability.ok || pref === "counts");
  if (hint) hint.textContent = availability.ok ? "" : `counts_per_mm missing for ${availability.missing.join(", ")}`;
}

function bindFerrUnitToggle(idPrefix: string, redraw: () => void) {
  const umBtn = el<HTMLButtonElement>(`${idPrefix}-unit-um`);
  const countsBtn = el<HTMLButtonElement>(`${idPrefix}-unit-counts`);
  if (umBtn) {
    umBtn.addEventListener("click", () => {
      if (umBtn.disabled) return;
      setFerrUnit("µm");
      redraw();
    });
  }
  if (countsBtn) {
    countsBtn.addEventListener("click", () => {
      setFerrUnit("counts");
      redraw();
    });
  }
}

function drawTimeDomain(names: string[], plots: PlotSeries[], steps: string[]) {
  const container = el("charts");
  if (!container) return;
  const availability = ferrUnitAvailability(stepDrivePairs(names, plots, steps));
  syncFerrUnitUi("time", availability);
  const unit = resolvedFerrUnit(availability);
  const motorFilter = state.motorFilter ? [...state.motorFilter] : null;
  const sig = { runs: runDataSig(names), steps, perMotor: motorViewPerMotor(), motorFilter, unit };
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
      for (const series of pickSeries(names[i], step, unit)) {
        yLabel = series.label;
        const color = mixColor(runColor(names[i]), "#ffffff", series.ramp);
        const hoverLabel = (names.length > 1 ? names[i] : "") + series.suffix;
        traces.push({ t: step.t_s, y: series.y, color, label: hoverLabel.trim() });
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
        hover: true,
      });
    }
    box.appendChild(legend);
    container.appendChild(box);
  }
}

// --- following-error PSD --------------------------------------------------

function newestSelectedRunName(names: string[]): string {
  const selected = new Set(names);
  const found = runsData().find((r) => selected.has(r.name));
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
  const detail = detailData(newest);
  const recommended = detail && detail.results && detail.results.verdict.recommended_step;
  const step =
    recommended && present.includes(recommended)
      ? recommended
      : present.length
        ? present[present.length - 1]
        : null;
  return { newest, step };
}

interface FilterChipItem {
  key: string;
  label: string;
  swatch?: string;
}

/// The one chip-selection grammar every filtered chart shares: an "all"
/// chip that clears the filter, plain click selects exactly one item
/// (clicking the lone selection clears back to all), shift+click grows or
/// shrinks a multi-selection, and a selection of none or of everything
/// normalizes to no filter.
function fillFilterChips(
  container: HTMLElement,
  allLabel: string,
  allTitle: string,
  noun: string,
  items: FilterChipItem[],
  getFilter: () => Set<string> | null,
  setFilter: (next: Set<string> | null) => void,
  onChange: () => void
) {
  container.innerHTML = "";
  const filter = getFilter();
  const all = document.createElement("button");
  all.className = "chip" + (filter ? "" : " active");
  all.textContent = allLabel;
  all.title = allTitle;
  all.addEventListener("click", () => {
    setFilter(null);
    onChange();
  });
  container.appendChild(all);
  const allKeys = items.map((i) => i.key);
  for (const item of items) {
    const chip = document.createElement("button");
    const inFilter = filter !== null && filter.has(item.key);
    chip.className = "chip" + (inFilter ? " active" : "");
    if (item.swatch) {
      chip.innerHTML = `<span class="swatch" style="background:${item.swatch}"></span>`;
    }
    chip.appendChild(document.createTextNode(item.label));
    chip.title = `click: only this ${noun} — shift+click: add/remove it`;
    chip.addEventListener("click", (ev) => {
      const cur = getFilter();
      if (ev.shiftKey) {
        const next = new Set(cur ?? allKeys);
        if (next.has(item.key)) next.delete(item.key);
        else next.add(item.key);
        setFilter(next.size === 0 || next.size === allKeys.length ? null : next);
      } else if (cur && cur.has(item.key) && cur.size === 1) {
        setFilter(null);
      } else {
        setFilter(new Set([item.key]));
      }
      onChange();
    });
    container.appendChild(chip);
  }
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

function countsPerMmOrNull(runName: string, driveName: string): number | null {
  const detail = detailData(runName);
  const motors = (detail && detail.manifest && detail.manifest.motors) || [];
  const motor = motors.find((m) => m.name === driveName);
  return motor && motor.counts_per_mm ? motor.counts_per_mm : null;
}

function countsPerMm(runName: string, driveName: string): number {
  const v = countsPerMmOrNull(runName, driveName);
  if (v === null) throw new Error(`${runName}: manifest has no counts_per_mm for ${driveName}`);
  return v;
}

function SectionHead({ title, tools }: { title: string; tools?: string }) {
  return html`<div class="section-head"><h2>${title}</h2></div>
    ${tools != null
      ? html`<div class="section-tools" dangerouslySetInnerHTML=${{ __html: tools }}></div>`
      : null}`;
}

function TimeDomainSection() {
  const tools =
    motorViewToggleHtml("combined") +
    ferrUnitToggleHtml("time") +
    `<div class="chips" id="time-motor-chips"></div>` +
    `<div class="chips" id="time-step-chips"></div>`;
  return html`<section class="time-section">
    <${SectionHead} title="time domain — following error" tools=${tools} />
    <div class="charts" id="charts"></div>
  </section>`;
}

function PathSection() {
  const tools = `<button id="path-fit">fit</button><span class="note" id="path-note"></span>`;
  return html`<section class="path-section" id="path-section" hidden>
    <${SectionHead} title="toolpath — commanded vs actual" tools=${tools} />
    <div class="chips" id="path-legend"></div>
    <div class="spatial-box"><canvas id="path-canvas"></canvas></div>
  </section>`;
}

export type { PickedSeries, FilterChipItem };
export {
  fillFilterChips,
  pickSeries,
  hidpiCanvasContext,
  drawTimeDomain,
  newestSelectedRunName,
  peakStep,
  mixColor,
  traceStyle,
  psdMaxFreqHz,
  clipToPsdBand,
  WELCH_HANN_ENBW_BINS,
  psdToAmplitude,
  countsPerMm,
  countsPerMmOrNull,
  stepDrivePairs,
  ferrUnitAvailability,
  resolvedFerrUnit,
  ferrUnitToggleHtml,
  syncFerrUnitUi,
  bindFerrUnitToggle,
  SectionHead,
  TimeDomainSection,
  PathSection,
};
