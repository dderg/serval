import { el, payloadUnchanged, runDataSig, ensurePlotSeries, pageRuns } from "./api";
import { drawTimeDomain, peakStep } from "./charts-core";
import { motorNames, cellRaw, renderDriveGroups } from "./drive";
import { renderFrfCharts, renderRingdownCharts } from "./dynamics";
import { renderMetricsTable, renderSweepMetricsChart, renderPsdChart, renderAccelPsdChart, visibleStepNames, renderStepChips, renderMotorChips } from "./metrics";
import { renderPathChart } from "./path-chart";
import { selectedRunNames } from "./runs";
import { currentPageDef } from "./shell";
import { RESONANCE_BAND_HZ, PEAK_MIN_SEPARATION_HZ, PEAK_LIST_SIZE, state } from "./state";
import { redrawStrain } from "./strain";
import type { DriveParam, PlotSeries } from "./wire";

// --- PSD peak list -----------------------------------------------------------

/// Greedy spaced peak-picking inside the resonance band: repeatedly take
/// the highest remaining bin at least PEAK_MIN_SEPARATION_HZ away from
/// every already-taken peak.
interface PsdPeak {
  freq: number;
  power: number;
}

function findPsdPeaks(freq: number[], psd: number[], band: [number, number], count: number): PsdPeak[] {
  const [blo, bhi] = band;
  const candidates: PsdPeak[] = [];
  for (let i = 0; i < freq.length; i++) {
    if (freq[i] >= blo && freq[i] < bhi) candidates.push({ freq: freq[i], power: psd[i] });
  }
  candidates.sort((a, b) => b.power - a.power);
  const peaks: PsdPeak[] = [];
  for (const c of candidates) {
    if (peaks.length >= count) break;
    if (peaks.every((p) => Math.abs(p.freq - c.freq) >= PEAK_MIN_SEPARATION_HZ)) {
      peaks.push(c);
    }
  }
  return peaks;
}

interface NotchSlot {
  n: number;
  freqParam: DriveParam;
  parked: boolean;
  current: number;
}

function notchSlotStates(): NotchSlot[] {
  const data = state.drive.data;
  if (!data) return [];
  const params = data.params;
  const slots: NotchSlot[] = [];
  for (let n = 1; n <= 5; n++) {
    const freqParam = params.find((p) => p.name === `notch_${n}_freq`);
    if (!freqParam) continue;
    const motors = motorNames(data.motors);
    const values = motors.map((m) => cellRaw(freqParam, m));
    const parked = values.every((v) => v === 8000);
    slots.push({ n, freqParam, parked, current: values[0] });
  }
  return slots;
}

function proposePeakIntoSlot(slot: NotchSlot, peakFreq: number) {
  const data = state.drive.data;
  if (!data) throw new Error("proposePeakIntoSlot without drive state");
  const raw = Math.round(peakFreq);
  const targets = motorNames(data.motors);
  const existing = { ...(state.drive.pending[slot.freqParam.name] || {}) };
  for (const m of targets) existing[m] = raw;
  state.drive.pending[slot.freqParam.name] = existing;
  renderDriveGroups();
}

function renderPeakList(names: string[], plots: PlotSeries[], steps: string[]) {
  const container = el("peak-list");
  if (!container) return;
  const slotSig = state.drive.data
    ? notchSlotStates().map((s) => [s.n, s.parked, s.current])
    : [];
  if (payloadUnchanged("peak-list", { runs: runDataSig(names), steps, slots: slotSig })) return;
  const runLabel = el("peaks-run");
  if (!names.length || !steps.length) {
    container.innerHTML = '<p class="note">select runs above</p>';
    if (runLabel) runLabel.textContent = "";
    return;
  }
  const picked = peakStep(names, plots, steps);
  const plot = plots[names.indexOf(picked.newest)];
  const step = plot && picked.step && plot.steps.find((s) => s.name === picked.step);
  if (!step || !step.psd) {
    container.innerHTML = '<p class="note">no PSD for this step</p>';
    if (runLabel) runLabel.textContent = "";
    return;
  }
  if (runLabel) runLabel.textContent = `${picked.newest} / ${picked.step}`;
  const driveNames = Object.keys(step.psd.per_drive);
  const peaks = findPsdPeaks(
    step.psd.freq_hz,
    step.psd.per_drive[driveNames[0]],
    RESONANCE_BAND_HZ,
    PEAK_LIST_SIZE
  );
  if (!peaks.length) {
    container.innerHTML = '<p class="note">no peaks in the 20–450 Hz band</p>';
    return;
  }
  const slots = state.drive.data ? notchSlotStates() : [];
  container.innerHTML = peaks
    .map((p) => {
      const buttons = slots
        .map((s) => {
          const label = s.parked ? `→ notch ${s.n}` : `→ notch ${s.n} (${s.current}Hz)`;
          const title = `set notch_${s.n}_freq to ${Math.round(p.freq)} on all motors (width/depth stay yours)`;
          return `<button class="peak-slot" data-slot="${s.n}" data-freq="${p.freq}" title="${title}">${label}</button>`;
        })
        .join("");
      return (
        `<div class="peak-row"><span class="peak-freq">${p.freq.toFixed(1)} Hz</span>` +
        `<span class="hint">${p.power.toExponential(1)} counts²/Hz</span>${buttons}</div>`
      );
    })
    .join("");
  container.querySelectorAll<HTMLButtonElement>("button.peak-slot").forEach((btn) => {
    btn.addEventListener("click", () => {
      const slot = notchSlotStates().find((s) => s.n === Number(btn.dataset.slot));
      if (slot) proposePeakIntoSlot(slot, Number(btn.dataset.freq));
    });
  });
}

/// Redraw the current page's chart sections from the run selection. Plot
/// series are cached per run mtime, so reselecting is cheap.
async function redrawCharts() {
  const def = currentPageDef();
  if (def.journal) return;
  if (def.strain) {
    await redrawStrain();
    return;
  }
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
  if (def.peaks) renderPeakList(okNames, plots, steps);
  if (def.charts && def.charts.includes("time")) drawTimeDomain(okNames, plots, steps);
}

export { findPsdPeaks, notchSlotStates, proposePeakIntoSlot, renderPeakList, redrawCharts };
