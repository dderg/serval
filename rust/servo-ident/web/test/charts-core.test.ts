import { beforeEach, expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { countsPerMmOrNull, countsPerMm, ferrUnitAvailability, pickSeries } = await import(
  "../src/charts-core"
);
const { state, MOTOR_VIEW_KEY, LIVE_UNIT_KEY } = await import("../src/state");
const { setFerrUnit } = await import("../src/units");
import type { PlotStep } from "../src/wire";

function seedRun(name: string, motors: { name: string; counts_per_mm: number | null }[]) {
  state.details.set(name, {
    mtime_utc: "",
    has_results: false,
    manifest: { experiment: "x", steps: [], motors },
    results: null,
  });
}

beforeEach(() => {
  state.details.clear();
  localStorage.removeItem(MOTOR_VIEW_KEY);
  localStorage.removeItem(LIVE_UNIT_KEY);
});

test("countsPerMmOrNull returns the manifest value when present", () => {
  seedRun("run1", [{ name: "motor0", counts_per_mm: 400 }]);
  expect(countsPerMmOrNull("run1", "motor0")).toBe(400);
});

test("countsPerMmOrNull returns null when the manifest lacks the drive or the value", () => {
  seedRun("run1", [{ name: "motor0", counts_per_mm: null }]);
  expect(countsPerMmOrNull("run1", "motor0")).toBeNull();
  expect(countsPerMmOrNull("run1", "motor1")).toBeNull();
  expect(countsPerMmOrNull("missing-run", "motor0")).toBeNull();
});

test("countsPerMm still throws when counts_per_mm is unrecoverable", () => {
  seedRun("run1", [{ name: "motor0", counts_per_mm: null }]);
  expect(() => countsPerMm("run1", "motor0")).toThrow();
});

test("ferrUnitAvailability is ok only when every pair resolves counts_per_mm", () => {
  seedRun("run1", [
    { name: "motor0", counts_per_mm: 400 },
    { name: "motor1", counts_per_mm: null },
  ]);
  const okPairs: [string, string][] = [["run1", "motor0"]];
  expect(ferrUnitAvailability(okPairs)).toEqual({ ok: true, missing: [] });
  const mixedPairs: [string, string][] = [
    ["run1", "motor0"],
    ["run1", "motor1"],
  ];
  const result = ferrUnitAvailability(mixedPairs);
  expect(result.ok).toBe(false);
  expect(result.missing).toEqual(["motor1"]);
});

test("ferrUnitAvailability is not ok with no pairs", () => {
  expect(ferrUnitAvailability([])).toEqual({ ok: false, missing: [] });
});

function makeStep(): PlotStep {
  return {
    name: "step",
    fs_hz: 1000,
    stride: 1,
    t_s: [0, 1, 2],
    moving: [],
    drives: {
      motor0: { ferr_counts: [10, 20, 30], torque_per_mille: [0, 0, 0] },
    },
    combined: null,
    accel: null,
    differential: null,
    ringdown: null,
    path: null,
    psd: { freq_hz: [], per_drive: {}, accel: null },
  };
}

test("pickSeries converts to µm in per-motor view when counts_per_mm is available", () => {
  seedRun("run1", [{ name: "motor0", counts_per_mm: 400 }]);
  localStorage.setItem(MOTOR_VIEW_KEY, "per-motor");
  const series = pickSeries("run1", makeStep(), "µm");
  expect(series).toHaveLength(1);
  expect(series[0].label).toBe("ferr (µm)");
  expect(series[0].y[0]).toBeCloseTo(10 * (1000 / 400));
});

test("pickSeries returns raw counts when the unit is counts", () => {
  seedRun("run1", [{ name: "motor0", counts_per_mm: 400 }]);
  localStorage.setItem(MOTOR_VIEW_KEY, "per-motor");
  const series = pickSeries("run1", makeStep(), "counts");
  expect(series[0].label).toBe("ferr (counts)");
  expect(series[0].y).toEqual([10, 20, 30]);
});
