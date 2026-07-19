import { html } from "htm/preact";
import { useEffect, useRef } from "preact/hooks";
import { useQuery } from "@tanstack/preact-query";
import { el, mustEl } from "./api";
import { LiveSpatialSection } from "./spatial";
import { timeSeriesPlot } from "./uplot-chart";
import { formatAge } from "./drive";
import { runGcode } from "./moonraker";
import { PALETTE, LIVE_STATUS_POLL_MS, LIVE_TAIL_POLL_MS, LIVE_UNIT_KEY, state } from "./state";
import { notify, useStore } from "./store";
import type { LiveSeries } from "./state";
import type { FixedY, TimeSeriesPlot, TimeTrace } from "./uplot-chart";
import type { LiveStatusPayload, LiveTapPayload, LiveTapStreaming } from "./wire";
import { getLiveTap } from "./api/live";
import { liveStatusQuery } from "./queries/live";
import { driveStateData } from "./queries/drive";
import type { ComponentChildren } from "preact";

// --- live tap ------------------------------------------------------------------
//
// Streams from GET /api/live_tap the moment the page opens — the server
// relays the ethercat-rt telemetry tap, so viewing needs no capture, no
// file, and no G-code. The cursor handshake mirrors the run-file tail:
// the first poll attaches "now" and returns only a cursor, every later
// poll sends it back and gets just the new samples. A cycle_index jump
// (drops under backpressure, tap reconnect) becomes a null break in the
// series — the chart shows a gap, never stale data drawn as live. Each
// motor gets its own stacked chart over the slider's window, all on a
// shared y-scale so the noisy motor stands out.

const FREEZE_BUFFER_MAX_S = 180;


function freezeState(frozen: boolean) {
  state.live.frozen = frozen;
  if (frozen) {
    const last = state.live.t.length ? state.live.t[state.live.t.length - 1] : 0;
    state.live.freezeStartT = last - state.live.windowS;
    state.live.freezeEndT = last;
  } else {
    state.live.freezeStartT = null;
    state.live.freezeEndT = null;
    state.live.freezeTruncated = false;
    trimLiveWindow();
  }
}

function setFrozen(frozen: boolean) {
  freezeState(frozen);
}

function toggleLiveFreeze() {
  freezeState(!state.live.frozen);
  notify();
}

function isTypingTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) return false;
  return (
    target.tagName === "INPUT" ||
    target.tagName === "TEXTAREA" ||
    target.tagName === "SELECT" ||
    target.isContentEditable
  );
}

function loadLiveUnit(): "µm" | "counts" {
  return localStorage.getItem(LIVE_UNIT_KEY) === "counts" ? "counts" : "µm";
}


function ferrUnitAvailability(drives: string[]): { ok: boolean; missing: string[] } {
  const missing = drives.filter((d) => !state.live.countsPerMm[d]);
  return { ok: drives.length > 0 && missing.length === 0, missing };
}





/// Header badge on every tab: a slow, cursor-less poll returns only the
/// attach payload (cursor + timing, no samples), so it costs nothing beyond
/// keeping the tap session alive while the dashboard is open.
export async function pollRtHealth() {
  const badge = el("rt-health");
  if (!badge) return;
  try {
    const payload = await getLiveTap();
    const t = payload.status === "streaming" ? payload.timing : null;
    if (!t) {
      badge.textContent = "";
      badge.classList.remove("rt-health-bad");
      return;
    }
    badge.textContent = `RT skips ${t.skips} · late ${t.late_frames} · margin ${(-t.lateness_ns / 1000).toFixed(0)} µs`;
    badge.classList.toggle("rt-health-bad", t.skips > 0 || t.late_frames > 0);
  } catch {
    badge.textContent = "";
    badge.classList.remove("rt-health-bad");
  }
}


function appendTapSamples(payload: LiveTapStreaming) {
  state.live.cursor = payload.next_cycle;
  state.live.fsHz = payload.fs_hz;
  (payload.drive_names || []).forEach((name, i) => {
    state.live.countsPerMm[name] = (payload.counts_per_mm || [])[i];
  });
  const tapDrives = payload.drives || {};
  const drives = Object.keys(tapDrives);
  const n = drives.length ? tapDrives[drives[0]].ferr.length : 0;
  if (!n) return;
  if (payload.first_cycle == null || payload.stride == null) {
    throw new Error("live tap streaming samples missing first_cycle or stride");
  }
  const firstCycle = payload.first_cycle;
  if (state.live.cycle0 === null) state.live.cycle0 = firstCycle;
  const cycle0 = state.live.cycle0;
  for (const drive of drives) {
    if (!state.live.perDrive[drive]) {
      state.live.perDrive[drive] = {
        ferr: new Array(state.live.t.length).fill(null),
        torque: new Array(state.live.t.length).fill(null),
        target: new Array(state.live.t.length).fill(null),
        pos: new Array(state.live.t.length).fill(null),
      };
    }
  }
  const stride = payload.stride;
  const gapThreshold = stride * 3;
  for (let i = 0; i < n; i++) {
    const cycle = firstCycle + i * stride;
    if (state.live.lastCycle !== null && cycle - state.live.lastCycle > gapThreshold) {
      state.live.t.push((state.live.lastCycle + stride - cycle0) / payload.fs_hz);
      for (const drive of drives) {
        state.live.perDrive[drive].ferr.push(null);
        state.live.perDrive[drive].torque.push(null);
        state.live.perDrive[drive].target.push(null);
        state.live.perDrive[drive].pos.push(null);
      }
    }
    state.live.t.push((cycle - cycle0) / payload.fs_hz);
    for (const drive of drives) {
      state.live.perDrive[drive].ferr.push(tapDrives[drive].ferr[i]);
      state.live.perDrive[drive].torque.push(tapDrives[drive].torque[i] / 10);
      state.live.perDrive[drive].target.push(tapDrives[drive].target[i]);
      state.live.perDrive[drive].pos.push(tapDrives[drive].pos[i]);
    }
    state.live.lastCycle = cycle;
  }
  trimLiveWindow();
}

function trimLiveWindow() {
  if (!state.live.t.length) return;
  const last = state.live.t[state.live.t.length - 1];
  let cutoff = last - state.live.windowS;
  if (state.live.frozen && state.live.freezeStartT !== null) {
    cutoff = Math.min(cutoff, state.live.freezeStartT);
    const capCutoff = last - FREEZE_BUFFER_MAX_S;
    if (capCutoff > cutoff) {
      cutoff = capCutoff;
      state.live.freezeTruncated = true;
    }
  }
  let drop = 0;
  while (drop < state.live.t.length && state.live.t[drop] < cutoff) drop++;
  if (drop > 0) {
    state.live.t.splice(0, drop);
    for (const series of Object.values(state.live.perDrive)) {
      series.ferr.splice(0, drop);
      series.torque.splice(0, drop);
      series.target.splice(0, drop);
      series.pos.splice(0, drop);
    }
  }
}

/// slot0..slotN are the tap's honest names (the RT process never sees
/// klippy's motor names); drive_state.json's slots map recovers the
/// motor name when a dump has run.
function liveDriveLabel(tapName: string): string {
  const drive = driveStateData();
  const slots = drive && drive.slots;
  if (!slots) return tapName;
  const match = /^slot(\d+)$/.exec(tapName);
  if (!match) return tapName;
  const slot = Number(match[1]);
  for (const [motor, s] of Object.entries(slots)) {
    if (s === slot) return motor;
  }
  return tapName;
}




function ferrDisplayScale(
  drives: string[],
  unit: "µm" | "counts"
): { unit: "µm" | "counts"; scale: Record<string, number> | null } {
  if (unit === "counts" || !ferrUnitAvailability(drives).ok) return { unit: "counts", scale: null };
  return {
    unit: "µm",
    scale: Object.fromEntries(drives.map((d) => [d, 1000 / state.live.countsPerMm[d]])),
  };
}





const liveTapUi = { statusText: "connecting to the telemetry tap…", statusBad: false };

function formatLiveStatus(payload: LiveTapPayload): { text: string; bad: boolean } {
  if (payload.status !== "streaming") {
    return {
      text:
        payload.status === "unreachable"
          ? `telemetry tap unreachable — ${payload.reason}`
          : "connecting to the telemetry tap…",
      bad: false,
    };
  }
  const t = payload.timing;
  const health = t
    ? ` — skipped cycles ${t.skips} · late frames ${t.late_frames} · margin ${(-t.lateness_ns / 1000).toFixed(0)} µs`
    : "";
  return {
    text: `streaming at ${(payload.fs_hz / 1000).toFixed(1)} kHz${health}`,
    bad: !!t && (t.skips > 0 || t.late_frames > 0),
  };
}

function formatLiveFileStatus(status: LiveStatusPayload): string {
  if (!status.capture) return "nothing recorded yet";
  const cap = status.capture;
  if (cap.name == null || cap.size_bytes == null) return "capture status unavailable";
  const growing = cap.age_s != null && cap.age_s < 3;
  return growing
    ? `recording ${cap.name} — ${(cap.size_bytes / 1024).toFixed(0)} KiB`
    : `last: ${cap.name} (${cap.age_s == null ? "?" : formatAge(cap.age_s)} ago)`;
}

interface LiveChartGroupData {
  display: Record<string, (number | null)[]>;
  peaks: Record<string, number>;
  yMin: number;
  yMax: number;
}

function computeLiveChartGroup(
  drives: string[],
  channel: keyof LiveSeries,
  scale: Record<string, number> | null
): LiveChartGroupData | null {
  let yMin = Infinity;
  let yMax = -Infinity;
  const peaks: Record<string, number> = {};
  const display: Record<string, (number | null)[]> = {};
  for (const d of drives) {
    const k = scale ? scale[d] : 1;
    const values = state.live.perDrive[d][channel];
    display[d] = scale ? values.map((v) => (v === null ? null : v * k)) : values;
    let peak = 0;
    for (const v of display[d]) {
      if (v === null) continue;
      if (v < yMin) yMin = v;
      if (v > yMax) yMax = v;
      const mag = Math.abs(v);
      if (mag > peak) peak = mag;
    }
    peaks[d] = peak;
  }
  if (!isFinite(yMin)) return null;
  return { display, peaks, yMin, yMax };
}

function chooseLiveUnit(unit: "µm" | "counts") {
  localStorage.setItem(LIVE_UNIT_KEY, unit);
  notify();
}

async function pollLiveTapTick() {
  if (state.live.polling) return;
  state.live.polling = true;
  try {
    const payload = await getLiveTap(state.live.cursor === null ? undefined : state.live.cursor);
    const status = formatLiveStatus(payload);
    liveTapUi.statusText = status.text;
    liveTapUi.statusBad = status.bad;
    if (payload.status === "streaming") appendTapSamples(payload);
    notify();
  } catch (e) {
    liveTapUi.statusText = String(e);
    liveTapUi.statusBad = false;
    notify();
  } finally {
    state.live.polling = false;
  }
}

function useLiveTap() {
  useEffect(() => {
    state.live.cursor = null;
    state.live.cycle0 = null;
    state.live.lastCycle = null;
    state.live.t = [];
    state.live.perDrive = {};
    state.live.countsPerMm = {};
    state.live.frozen = false;
    state.live.freezeStartT = null;
    state.live.freezeEndT = null;
    state.live.freezeTruncated = false;
    liveTapUi.statusText = "connecting to the telemetry tap…";
    liveTapUi.statusBad = false;
    pollLiveTapTick();
    const id = setInterval(pollLiveTapTick, LIVE_TAIL_POLL_MS);
    return () => clearInterval(id);
  }, []);
}

function useFreezeKeyboard() {
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.code !== "Space" || e.repeat || isTypingTarget(e.target)) return;
      e.preventDefault();
      toggleLiveFreeze();
    };
    document.addEventListener("keydown", onKey);
    return () => document.removeEventListener("keydown", onKey);
  }, []);
}

interface LivePlotData {
  t: number[];
  y: (number | null)[];
}

interface ChartBoxProps {
  drive: string;
  idPrefix: string;
  color: string;
  yLabel: string;
  plotData: LivePlotData | null;
  fixedY: FixedY | null;
  peakText: string;
}

function ChartBox({ drive, idPrefix, color, yLabel, plotData, fixedY, peakText }: ChartBoxProps) {
  const hostRef = useRef<HTMLDivElement>(null);
  const plotRef = useRef<TimeSeriesPlot | null>(null);
  useEffect(() => {
    const host = hostRef.current;
    if (!host || state.live.frozen || !plotData) return;
    const trace: TimeTrace = { t: plotData.t, y: plotData.y, color };
    let plot = plotRef.current;
    if (plot && !plot.u.root.isConnected) {
      plot.u.destroy();
      plot = null;
      plotRef.current = null;
    }
    if (!plot) {
      plotRef.current = timeSeriesPlot(host, {
        width: host.parentElement?.clientWidth || 860,
        height: 130,
        yLabel,
        fixedY,
        traces: [trace],
      });
    } else {
      plot.setTraces([trace], fixedY);
    }
  });
  useEffect(
    () => () => {
      if (plotRef.current) {
        plotRef.current.u.destroy();
        plotRef.current = null;
      }
    },
    []
  );
  return html`<div class="chart-box"><h3><span class="swatch" style=${{ background: color }}></span><span id=${`${idPrefix}-name-${drive}`}>${liveDriveLabel(drive)}</span> <span class="note" id=${`${idPrefix}-peak-${drive}`}>${peakText}</span></h3><div id=${`${idPrefix}-plot-${drive}`} ref=${hostRef}></div></div>`;
}

interface LiveChartGroupProps {
  containerId: string;
  idPrefix: string;
  channel: keyof LiveSeries;
  yLabel: string;
  peakFmt: (peak: number) => string;
  scale: Record<string, number> | null;
  drives: string[];
  placeholder: ComponentChildren;
}

function LiveChartGroup({
  containerId,
  idPrefix,
  channel,
  yLabel,
  peakFmt,
  scale,
  drives,
  placeholder,
}: LiveChartGroupProps) {
  const group = drives.length ? computeLiveChartGroup(drives, channel, scale) : null;
  const boxes = drives.map((d, i) => {
    const color = PALETTE[i % PALETTE.length];
    const plotData = group ? { t: state.live.t, y: group.display[d] } : null;
    const fixedY = group ? { yMin: group.yMin, yMax: group.yMax } : null;
    const peakText = group ? peakFmt(group.peaks[d]) : "";
    return html`<${ChartBox}
      key=${d}
      drive=${d}
      idPrefix=${idPrefix}
      color=${color}
      yLabel=${yLabel}
      plotData=${plotData}
      fixedY=${fixedY}
      peakText=${peakText}
    />`;
  });
  return html`<div class="charts" id=${containerId}>${drives.length ? boxes : placeholder}</div>`;
}

function LiveFerrSection({ drives }: { drives: string[] }) {
  const availability = ferrUnitAvailability(drives);
  const pref = loadLiveUnit();
  const wanted = pref === "µm" && availability.ok ? "µm" : "counts";
  const ferr = ferrDisplayScale(drives, wanted);
  const peakFmt =
    ferr.unit === "µm"
      ? (p: number) => `peak |ferr| ${p.toFixed(1)} µm`
      : (p: number) => `peak |ferr| ${p} counts`;
  const onWindowInput = (e: Event) => {
    state.live.windowS = Number((e.target as HTMLInputElement).value);
    trimLiveWindow();
    notify();
  };
  return html`<section class="live-section">
    <div class="section-head"><h2>live following error — per motor</h2></div>
    <div class="section-tools">
      <span class="chips live-unit-chips"><button class=${availability.ok && pref === "µm" ? "chip active" : "chip"} id="live-unit-um" data-unit="µm" disabled=${!availability.ok} onClick=${() => availability.ok && chooseLiveUnit("µm")}>µm</button><button class=${!availability.ok || pref === "counts" ? "chip active" : "chip"} id="live-unit-counts" data-unit="counts" onClick=${() => chooseLiveUnit("counts")}>counts</button></span>
      <span class="note" id="live-unit-hint">${availability.ok ? "" : `counts_per_mm missing for ${availability.missing.join(", ")}`}</span>
      <label class="live-window">window <input type="range" id="live-window" min="1" max="30" step="1" value=${state.live.windowS} onInput=${onWindowInput} /><span id="live-window-value">${state.live.windowS} s</span></label>
      <span class=${liveTapUi.statusBad ? "note live-timing-bad" : "note"} id="live-status">${liveTapUi.statusText}</span>
    </div>
    <${LiveChartGroup}
      containerId="live-charts"
      idPrefix="live"
      channel="ferr"
      yLabel=${`ferr (${ferr.unit})`}
      peakFmt=${peakFmt}
      scale=${ferr.scale}
      drives=${drives}
      placeholder=${html`<p class="note">streams straight from the drives the moment the tap answers — no capture, no file</p>`}
    />
  </section>`;
}

function LiveTorqueSection({ drives }: { drives: string[] }) {
  return html`<section class="live-section">
    <div class="section-head"><h2>live actual torque — per motor</h2></div>
    <${LiveChartGroup}
      containerId="live-torque-charts"
      idPrefix="live-torque"
      channel="torque"
      yLabel="torque (% rated)"
      peakFmt=${(p: number) => `peak |torque| ${p.toFixed(1)}%`}
      scale=${null}
      drives=${drives}
      placeholder=${null}
    />
  </section>`;
}

function RecordPanel() {
  const { data, error } = useQuery({ ...liveStatusQuery(), refetchInterval: LIVE_STATUS_POLL_MS });
  const statusText = error ? String(error) : data ? formatLiveFileStatus(data) : "";
  const cmdRef = useRef<HTMLInputElement>(null);
  const record = () => {
    const line = cmdRef.current?.value.trim();
    if (line) runGcode([line], "live");
  };
  return html`<section class="sweep">
    <div class="section-head"><h2>record to file</h2><span class="note" id="live-file-status">${statusText}</span></div>
    <div class="row"><input ref=${cmdRef} type="text" id="live-start-command" defaultValue="SERVO_CAPTURE_START NAME=live AXIS=X" /><button id="live-start-btn" onClick=${record}>record</button><button id="live-stop-btn" onClick=${() => runGcode(["SERVO_CAPTURE_STOP"], "live")}>stop</button></div>
    <p class="note">viewing needs no recording. record when you want an analyzable .scap in the captures root; stop finalizes it.</p>
  </section>`;
}

function LivePage({ aside }: { aside?: ComponentChildren }) {
  useStore();
  useLiveTap();
  useFreezeKeyboard();
  const drives = state.live.t.length ? Object.keys(state.live.perDrive).sort() : [];
  return html`<div class="workspace">
    <main class="analysis">
      <${LiveSpatialSection}
        frozen=${state.live.frozen}
        freezeTruncated=${state.live.freezeTruncated}
        onToggleFreeze=${toggleLiveFreeze}
      />
      <${LiveFerrSection} drives=${drives} />
      <${LiveTorqueSection} drives=${drives} />
    </main>
    <aside class="controls">
      <${RecordPanel} />
      ${aside ?? null}
    </aside>
  </div>`;
}

export { appendTapSamples, trimLiveWindow, liveDriveLabel, setFrozen, ferrDisplayScale, ferrUnitAvailability, loadLiveUnit, FREEZE_BUFFER_MAX_S, LivePage, RecordPanel, formatLiveStatus, formatLiveFileStatus, computeLiveChartGroup };
