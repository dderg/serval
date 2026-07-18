import { api, el, mustEl, payloadUnchanged } from "./api";
import { timeSeriesPlot } from "./uplot-chart";
import { formatAge } from "./drive";
import { runGcode } from "./moonraker";
import { PALETTE, LIVE_STATUS_POLL_MS, LIVE_TAIL_POLL_MS, state } from "./state";
import type { LiveSeries } from "./state";
import type { FixedY, TimeSeriesPlot, TimeTrace } from "./uplot-chart";
import type { LiveStatus, LiveTapPayload } from "./wire";

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

function bindLiveEvents() {
  mustEl("live-start-btn").addEventListener("click", () => {
    const line = mustEl<HTMLInputElement>("live-start-command").value.trim();
    if (line) runGcode([line], "live");
  });
  mustEl("live-stop-btn").addEventListener("click", () => runGcode(["SERVO_CAPTURE_STOP"], "live"));
  const slider = mustEl<HTMLInputElement>("live-window");
  slider.addEventListener("input", () => {
    state.live.windowS = Number(slider.value);
    mustEl("live-window-value").textContent = `${state.live.windowS} s`;
    trimLiveWindow();
    drawLiveCharts();
  });
}

async function pollLiveFileStatus() {
  const label = el("live-file-status");
  if (!label) return;
  let status: LiveStatus;
  try {
    status = await api("/api/live");
  } catch (e) {
    label.textContent = String(e);
    return;
  }
  if (!status.capture) {
    label.textContent = "nothing recorded yet";
    return;
  }
  const cap = status.capture;
  const growing = cap.age_s !== null && cap.age_s < 3;
  label.textContent = growing
    ? `recording ${cap.name} — ${(cap.size_bytes / 1024).toFixed(0)} KiB`
    : `last: ${cap.name} (${cap.age_s === null ? "?" : formatAge(cap.age_s)} ago)`;
}

/// Header badge on every tab: a slow, cursor-less poll returns only the
/// attach payload (cursor + timing, no samples), so it costs nothing beyond
/// keeping the tap session alive while the dashboard is open.
export async function pollRtHealth() {
  const badge = el("rt-health");
  if (!badge) return;
  try {
    const payload: LiveTapPayload = await api("/api/live_tap");
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

async function pollLiveTap() {
  if (state.live.polling) return;
  state.live.polling = true;
  try {
    const query = state.live.cursor === null ? "" : `?since_cycle=${state.live.cursor}`;
    const payload: LiveTapPayload = await api(`/api/live_tap${query}`);
    const label = el("live-status");
    if (payload.status !== "streaming") {
      if (label) {
        label.textContent =
          payload.status === "unreachable"
            ? `telemetry tap unreachable — ${payload.reason}`
            : "connecting to the telemetry tap…";
      }
      return;
    }
    if (label) {
      const t = payload.timing;
      const health = t
        ? ` — skipped cycles ${t.skips} · late frames ${t.late_frames} · margin ${(-t.lateness_ns / 1000).toFixed(0)} µs`
        : "";
      label.textContent = `streaming at ${(payload.fs_hz / 1000).toFixed(1)} kHz${health}`;
      label.classList.toggle("live-timing-bad", !!t && (t.skips > 0 || t.late_frames > 0));
    }
    appendTapSamples(payload);
    drawLiveCharts();
  } catch (e) {
    const label = el("live-status");
    if (label) label.textContent = String(e);
  } finally {
    state.live.polling = false;
  }
}

function appendTapSamples(payload: LiveTapPayload) {
  state.live.cursor = payload.next_cycle;
  state.live.fsHz = payload.fs_hz;
  const tapDrives = payload.drives || {};
  const drives = Object.keys(tapDrives);
  const n = drives.length ? tapDrives[drives[0]].ferr.length : 0;
  if (!n) return;
  if (state.live.cycle0 === null) state.live.cycle0 = payload.first_cycle;
  const cycle0 = state.live.cycle0;
  for (const drive of drives) {
    if (!state.live.perDrive[drive]) {
      state.live.perDrive[drive] = {
        ferr: new Array(state.live.t.length).fill(null),
        torque: new Array(state.live.t.length).fill(null),
      };
    }
  }
  const stride = payload.stride;
  const gapThreshold = stride * 3;
  for (let i = 0; i < n; i++) {
    const cycle = payload.first_cycle + i * stride;
    if (state.live.lastCycle !== null && cycle - state.live.lastCycle > gapThreshold) {
      state.live.t.push((state.live.lastCycle + stride - cycle0) / payload.fs_hz);
      for (const drive of drives) {
        state.live.perDrive[drive].ferr.push(null);
        state.live.perDrive[drive].torque.push(null);
      }
    }
    state.live.t.push((cycle - cycle0) / payload.fs_hz);
    for (const drive of drives) {
      state.live.perDrive[drive].ferr.push(tapDrives[drive].ferr[i]);
      state.live.perDrive[drive].torque.push(tapDrives[drive].torque[i] / 10);
    }
    state.live.lastCycle = cycle;
  }
  trimLiveWindow();
}

function trimLiveWindow() {
  if (!state.live.t.length) return;
  const cutoff = state.live.t[state.live.t.length - 1] - state.live.windowS;
  let drop = 0;
  while (drop < state.live.t.length && state.live.t[drop] < cutoff) drop++;
  if (drop > 0) {
    state.live.t.splice(0, drop);
    for (const series of Object.values(state.live.perDrive)) {
      series.ferr.splice(0, drop);
      series.torque.splice(0, drop);
    }
  }
}

/// slot0..slotN are the tap's honest names (the RT process never sees
/// klippy's motor names); drive_state.json's slots map recovers the
/// motor name when a dump has run.
function liveDriveLabel(tapName: string): string {
  const slots = state.drive.data && state.drive.data.slots;
  if (!slots) return tapName;
  const match = /^slot(\d+)$/.exec(tapName);
  if (!match) return tapName;
  const slot = Number(match[1]);
  for (const [motor, s] of Object.entries(slots)) {
    if (s === slot) return motor;
  }
  return tapName;
}

function ensureLiveChartBoxes(containerId: string, idPrefix: string, drives: string[]): boolean {
  const container = el(containerId);
  if (!container) return false;
  if (!payloadUnchanged(`live-boxes-${containerId}`, drives)) {
    container.innerHTML = drives
      .map(
        (d, i) =>
          `<div class="chart-box">` +
          `<h3><span class="swatch" style="background:${PALETTE[i % PALETTE.length]}"></span>` +
          `<span id="${idPrefix}-name-${d}">${liveDriveLabel(d)}</span> ` +
          `<span class="note" id="${idPrefix}-peak-${d}"></span></h3>` +
          `<div id="${idPrefix}-plot-${d}"></div>` +
          `</div>`
      )
      .join("");
  }
  return true;
}

const livePlots = new Map<string, { plot: TimeSeriesPlot }>();

function livePlotFor(hostId: string, yLabel: string, trace: TimeTrace, fixedY: FixedY): TimeSeriesPlot | null {
  const host = el(hostId);
  if (!host) return null;
  const existing = livePlots.get(hostId);
  if (existing) {
    if (existing.plot.u.root.isConnected) {
      existing.plot.setTraces([trace], fixedY);
      return existing.plot;
    }
    existing.plot.u.destroy();
    livePlots.delete(hostId);
  }
  const plot = timeSeriesPlot(host, {
    width: host.parentElement?.clientWidth || 860,
    height: 130,
    yLabel,
    fixedY,
    traces: [trace],
  });
  livePlots.set(hostId, { plot });
  return plot;
}

function drawLiveChartGroup(
  containerId: string,
  idPrefix: string,
  drives: string[],
  channel: keyof LiveSeries,
  yLabel: string,
  peakFmt: (peak: number) => string
) {
  if (!ensureLiveChartBoxes(containerId, idPrefix, drives)) return;
  let yMin = Infinity;
  let yMax = -Infinity;
  const peaks: Record<string, number> = {};
  for (const d of drives) {
    let peak = 0;
    for (const v of state.live.perDrive[d][channel]) {
      if (v === null) continue;
      if (v < yMin) yMin = v;
      if (v > yMax) yMax = v;
      const mag = Math.abs(v);
      if (mag > peak) peak = mag;
    }
    peaks[d] = peak;
  }
  if (!isFinite(yMin)) return;
  drives.forEach((d, i) => {
    livePlotFor(
      `${idPrefix}-plot-${d}`,
      yLabel,
      {
        t: state.live.t,
        y: state.live.perDrive[d][channel],
        color: PALETTE[i % PALETTE.length],
      },
      { yMin, yMax }
    );
    const name = el(`${idPrefix}-name-${d}`);
    if (name) name.textContent = liveDriveLabel(d);
    const label = el(`${idPrefix}-peak-${d}`);
    if (label) label.textContent = peakFmt(peaks[d]);
  });
}

function drawLiveCharts() {
  if (!state.live.t.length) return;
  const drives = Object.keys(state.live.perDrive).sort();
  if (!drives.length) return;
  drawLiveChartGroup(
    "live-charts",
    "live",
    drives,
    "ferr",
    "ferr (counts)",
    (p) => `peak |ferr| ${p}`
  );
  drawLiveChartGroup(
    "live-torque-charts",
    "live-torque",
    drives,
    "torque",
    "torque (% rated)",
    (p) => `peak |torque| ${p.toFixed(1)}%`
  );
}

function startLivePolling() {
  state.live.cursor = null;
  state.live.cycle0 = null;
  state.live.lastCycle = null;
  state.live.t = [];
  state.live.perDrive = {};
  pollLiveFileStatus();
  pollLiveTap();
  state.live.timers = [
    setInterval(pollLiveFileStatus, LIVE_STATUS_POLL_MS),
    setInterval(pollLiveTap, LIVE_TAIL_POLL_MS),
  ];
}

function stopLivePolling() {
  for (const id of state.live.timers) clearInterval(id);
  state.live.timers = [];
}

export { bindLiveEvents, pollLiveFileStatus, pollLiveTap, appendTapSamples, trimLiveWindow, liveDriveLabel, ensureLiveChartBoxes, drawLiveChartGroup, drawLiveCharts, startLivePolling, stopLivePolling };
