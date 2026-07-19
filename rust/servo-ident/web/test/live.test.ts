import { beforeEach, expect, test } from "bun:test";
import { registerDom } from "./dom";
import { liveStatusQuery, liveStatusKey } from "../src/queries/live";
import { getLiveStatus } from "../src/api/live";
import type { LiveStatusPayload, LiveTapPayload } from "../src/wire";

registerDom();

const {
  trimLiveWindow,
  ferrDisplayScale,
  ferrUnitAvailability,
  setFrozen,
  FREEZE_BUFFER_MAX_S,
  formatLiveStatus,
  formatLiveFileStatus,
  computeLiveChartGroup,
} = await import(
  "../src/live"
);
const { liveDrawCount, state } = await import("../src/state");

function seedBuffer(seconds: number, hz = 10) {
  const n = seconds * hz + 1;
  state.live.t = Array.from({ length: n }, (_, i) => i / hz);
  state.live.perDrive = {
    slot0: {
      ferr: Array.from({ length: n }, (_, i) => i),
      torque: new Array(n).fill(0),
      target: new Array(n).fill(0),
      pos: new Array(n).fill(0),
    },
  };
}

beforeEach(() => {
  state.live.frozen = false;
  state.live.freezeStartT = null;
  state.live.freezeEndT = null;
  state.live.freezeTruncated = false;
  state.live.windowS = 10;
  state.live.countsPerMm = {};
  state.live.t = [];
  state.live.perDrive = {};
});

test("trimLiveWindow keeps only the slider window when live", () => {
  seedBuffer(30);
  trimLiveWindow();
  expect(state.live.t[0]).toBeGreaterThanOrEqual(30 - 10);
  expect(state.live.t.length).toBe(state.live.perDrive.slot0.ferr.length);
  expect(state.live.perDrive.slot0.ferr[0]).toBe(200);
});

test("trimLiveWindow honors a 1 s window", () => {
  seedBuffer(30);
  state.live.windowS = 1;
  trimLiveWindow();
  expect(state.live.t[0]).toBeGreaterThanOrEqual(29);
});

test("frozen buffer keeps the frozen window while new samples stream in", () => {
  seedBuffer(30);
  state.live.frozen = true;
  state.live.freezeStartT = 20;
  state.live.freezeEndT = 30;
  seedBuffer(60);
  trimLiveWindow();
  expect(state.live.t[0]).toBe(20);
  expect(state.live.freezeTruncated).toBe(false);
  expect(liveDrawCount()).toBe(state.live.t.findIndex((v) => v > 30));
});

test("frozen buffer is capped and truncation is flagged loudly", () => {
  state.live.frozen = true;
  state.live.freezeStartT = 0;
  state.live.freezeEndT = 10;
  seedBuffer(FREEZE_BUFFER_MAX_S + 60);
  trimLiveWindow();
  const span = state.live.t[state.live.t.length - 1] - state.live.t[0];
  expect(span).toBeLessThanOrEqual(FREEZE_BUFFER_MAX_S);
  expect(state.live.freezeTruncated).toBe(true);
});

test("setFrozen anchors the freeze window and unfreezing snaps back to live", () => {
  seedBuffer(30);
  setFrozen(true);
  expect(state.live.frozen).toBe(true);
  expect(state.live.freezeEndT).toBe(30);
  expect(state.live.freezeStartT).toBe(20);
  expect(liveDrawCount()).toBe(state.live.t.length);
  seedBuffer(60);
  expect(liveDrawCount()).toBeLessThan(state.live.t.length);
  setFrozen(false);
  expect(state.live.frozen).toBe(false);
  expect(state.live.freezeEndT).toBeNull();
  expect(state.live.t[0]).toBeGreaterThanOrEqual(60 - 10);
  expect(liveDrawCount()).toBe(state.live.t.length);
});

test("ferrDisplayScale converts counts to µm when every drive has counts_per_mm and µm is requested", () => {
  state.live.countsPerMm = { slot0: 1000, slot1: 500 };
  const { unit, scale } = ferrDisplayScale(["slot0", "slot1"], "µm");
  expect(unit).toBe("µm");
  expect(scale).toEqual({ slot0: 1, slot1: 2 });
});

test("ferrDisplayScale renders counts when counts mode is requested even with full coverage", () => {
  state.live.countsPerMm = { slot0: 1000, slot1: 500 };
  const { unit, scale } = ferrDisplayScale(["slot0", "slot1"], "counts");
  expect(unit).toBe("counts");
  expect(scale).toBeNull();
});

test("ferrDisplayScale falls back to counts when no counts_per_mm is known", () => {
  const { unit, scale } = ferrDisplayScale(["slot0", "slot1"], "µm");
  expect(unit).toBe("counts");
  expect(scale).toBeNull();
});

test("ferrDisplayScale falls back to counts on a mixed set even when µm is requested", () => {
  state.live.countsPerMm = { slot0: 1000 };
  const { unit, scale } = ferrDisplayScale(["slot0", "slot1"], "µm");
  expect(unit).toBe("counts");
  expect(scale).toBeNull();
});

test("ferrUnitAvailability is ok only when every drive has counts_per_mm", () => {
  state.live.countsPerMm = { slot0: 1000, slot1: 500 };
  expect(ferrUnitAvailability(["slot0", "slot1"])).toEqual({ ok: true, missing: [] });
});

test("ferrUnitAvailability reports the drives missing counts_per_mm", () => {
  state.live.countsPerMm = { slot0: 1000 };
  expect(ferrUnitAvailability(["slot0", "slot1"])).toEqual({ ok: false, missing: ["slot1"] });
});

test("ferrUnitAvailability is not ok with an empty drive set", () => {
  expect(ferrUnitAvailability([])).toEqual({ ok: false, missing: [] });
});

test("live status query config is colocated on the shared key and reuses the live transport", () => {
  const opts = liveStatusQuery();
  expect(opts.queryKey).toBe(liveStatusKey);
  expect(opts.queryKey).toEqual(["live", "status"]);
  expect(opts.queryFn).toBe(getLiveStatus);
  expect(opts.staleTime).toBe(0);
});

test("formatLiveStatus reports streaming rate with a healthy timing tail", () => {
  const payload = {
    status: "streaming",
    fs_hz: 20000,
    timing: { skips: 0, late_frames: 0, lateness_ns: -2000 },
  } as unknown as LiveTapPayload;
  expect(formatLiveStatus(payload)).toEqual({
    text: "streaming at 20.0 kHz — skipped cycles 0 · late frames 0 · margin 2 µs",
    bad: false,
  });
});

test("formatLiveStatus flags streaming as bad when cycles skip or frames run late", () => {
  const payload = {
    status: "streaming",
    fs_hz: 20000,
    timing: { skips: 3, late_frames: 0, lateness_ns: -2000 },
  } as unknown as LiveTapPayload;
  const status = formatLiveStatus(payload);
  expect(status.bad).toBe(true);
  expect(status.text).toContain("skipped cycles 3");
});

test("formatLiveStatus drops the timing tail when no timing is present", () => {
  const payload = { status: "streaming", fs_hz: 20000, timing: null } as unknown as LiveTapPayload;
  expect(formatLiveStatus(payload)).toEqual({ text: "streaming at 20.0 kHz", bad: false });
});

test("formatLiveStatus surfaces the reason when the tap is unreachable", () => {
  const payload = { status: "unreachable", reason: "no socket" } as unknown as LiveTapPayload;
  expect(formatLiveStatus(payload)).toEqual({
    text: "telemetry tap unreachable — no socket",
    bad: false,
  });
});

test("formatLiveStatus falls back to the connecting copy before the tap answers", () => {
  const payload = { status: "connecting" } as unknown as LiveTapPayload;
  expect(formatLiveStatus(payload)).toEqual({
    text: "connecting to the telemetry tap…",
    bad: false,
  });
});

test("computeLiveChartGroup returns raw values, peak magnitude, and shared y-extent", () => {
  state.live.perDrive = {
    slot0: { ferr: [1, -3, 2, null], torque: [], target: [], pos: [] },
  };
  const group = computeLiveChartGroup(["slot0"], "ferr", null);
  expect(group).toEqual({
    display: { slot0: [1, -3, 2, null] },
    peaks: { slot0: 3 },
    yMin: -3,
    yMax: 2,
  });
});

test("computeLiveChartGroup applies a per-drive scale to values, peaks, and extent", () => {
  state.live.perDrive = {
    slot0: { ferr: [1, -3, 2, null], torque: [], target: [], pos: [] },
  };
  const group = computeLiveChartGroup(["slot0"], "ferr", { slot0: 2 });
  expect(group).toEqual({
    display: { slot0: [2, -6, 4, null] },
    peaks: { slot0: 6 },
    yMin: -6,
    yMax: 4,
  });
});

test("computeLiveChartGroup returns null when no finite samples exist", () => {
  state.live.perDrive = {
    slot0: { ferr: [null, null], torque: [], target: [], pos: [] },
  };
  expect(computeLiveChartGroup(["slot0"], "ferr", null)).toBeNull();
});

test("formatLiveFileStatus reports nothing recorded until a capture exists", () => {
  expect(formatLiveFileStatus({ capture: null } as LiveStatusPayload)).toBe("nothing recorded yet");
});

test("formatLiveFileStatus shows a growing capture as recording with its size", () => {
  const status = { capture: { name: "live.scap", size_bytes: 2048, age_s: 1 } } as LiveStatusPayload;
  expect(formatLiveFileStatus(status)).toBe("recording live.scap — 2 KiB");
});

test("formatLiveFileStatus shows a stale capture as the last file with its age", () => {
  const status = { capture: { name: "live.scap", size_bytes: 2048, age_s: 5 } } as LiveStatusPayload;
  expect(formatLiveFileStatus(status)).toBe("last: live.scap (5s ago)");
});

test("formatLiveFileStatus reports unavailable when the capture name is missing", () => {
  const status = { capture: { name: null, size_bytes: 100, age_s: 1 } } as LiveStatusPayload;
  expect(formatLiveFileStatus(status)).toBe("capture status unavailable");
});
