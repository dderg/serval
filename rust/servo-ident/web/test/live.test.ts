import { beforeEach, expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { trimLiveWindow, ferrDisplayScale, setFrozen, FREEZE_BUFFER_MAX_S } = await import("../src/live");
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

test("ferrDisplayScale converts counts to µm when every drive has counts_per_mm", () => {
  state.live.countsPerMm = { slot0: 1000, slot1: 500 };
  const { unit, scale } = ferrDisplayScale(["slot0", "slot1"]);
  expect(unit).toBe("µm");
  expect(scale).toEqual({ slot0: 1, slot1: 2 });
});

test("ferrDisplayScale falls back to counts when no counts_per_mm is known", () => {
  const { unit, scale } = ferrDisplayScale(["slot0", "slot1"]);
  expect(unit).toBe("counts");
  expect(scale).toBeNull();
});

test("ferrDisplayScale refuses a mixed-unit chart", () => {
  state.live.countsPerMm = { slot0: 1000 };
  expect(() => ferrDisplayScale(["slot0", "slot1"])).toThrow("mixed-unit");
});
