import { beforeEach, expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { uniformCountsPerMm, psdCartesianScaled, psdCartesianTraces } = await import(
  "../src/metrics"
);
const { state, PALETTE, PSD_MAX_FREQ_KEY } = await import("../src/state");
const { queryClient, queryKeys } = await import("../src/query-client");
import type { PlotSeries, PlotStep } from "../src/wire";

function seedRun(name: string, cpms: (number | null)[]) {
  queryClient.setQueryData(queryKeys.runDetail(name), {
    mtime_utc: "",
    has_results: false,
    manifest: {
      experiment: "x",
      steps: [],
      motors: cpms.map((cpm, i) => ({ name: `motor${i}`, counts_per_mm: cpm })),
    },
    results: null,
  });
}

beforeEach(() => {
  state.runColors.clear();
  state.runColors.set("run1", PALETTE[0]);
  localStorage.setItem(PSD_MAX_FREQ_KEY, "500");
});

test("uniformCountsPerMm returns the shared value and throws on a mix", () => {
  seedRun("run1", [3276.8, 3276.8]);
  expect(uniformCountsPerMm("run1")).toBe(3276.8);
  seedRun("run2", [3276.8, 1638.4]);
  expect(() => uniformCountsPerMm("run2")).toThrow("shared counts_per_mm");
  seedRun("run3", []);
  expect(() => uniformCountsPerMm("run3")).toThrow();
});

test("psdCartesianScaled converts mm²/Hz to µm²/Hz and counts²/Hz", () => {
  seedRun("run1", [1000, 1000]);
  expect(psdCartesianScaled([2e-6], "run1", "µm")).toEqual([2]);
  expect(psdCartesianScaled([2e-6], "run1", "counts")).toEqual([2]);
});

function cartesianStep(name: string): PlotStep {
  return {
    name,
    fs_hz: 4000,
    stride: 1,
    t_s: [],
    moving: [],
    drives: {},
    combined: null,
    accel: null,
    differential: null,
    ringdown: null,
    path: null,
    psd: {
      freq_hz: [0, 10, 20, 30],
      per_drive: {},
      cartesian: { x: [4e-6, 4e-6, 4e-6, 4e-6], y: [1e-6, 1e-6, 1e-6, 1e-6] },
      accel: null,
    },
  } as unknown as PlotStep;
}

test("psdCartesianTraces emits one trace per mode with unit scaling applied", () => {
  seedRun("run1", [3276.8]);
  const plots: PlotSeries[] = [{ version: 1, steps: [cartesianStep("s1")] } as PlotSeries];
  const traces = psdCartesianTraces(["run1"], plots, ["s1"], "µm");
  expect(traces.map((t) => t.label)).toEqual(["s1 (x)", "s1 (y)"]);
  expect(traces[0].y[0] / traces[1].y[0]).toBeCloseTo(2, 10);
});

test("psdCartesianTraces is empty without a cartesian psd", () => {
  const step = cartesianStep("s1");
  (step.psd as { cartesian: unknown }).cartesian = null;
  const plots: PlotSeries[] = [{ version: 1, steps: [step] } as PlotSeries];
  expect(psdCartesianTraces(["run1"], plots, ["s1"], "µm")).toEqual([]);
});
