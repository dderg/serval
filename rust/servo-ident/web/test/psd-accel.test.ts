import { beforeEach, expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { psdAccelTraces } = await import("../src/metrics");
const { state, PALETTE, PSD_MAX_FREQ_KEY } = await import("../src/state");
import type { PlotSeries, PlotStep } from "../src/wire";

const FREQ = [0, 10, 20, 30];

function accelStep(name: string): PlotStep {
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
      freq_hz: FREQ,
      per_drive: {},
      cartesian: null,
      accel: {
        freq_hz: FREQ,
        psd: [6, 6, 6, 6],
        psd_x: [1, 1, 1, 1],
        psd_y: [2, 2, 2, 2],
        psd_z: [3, 3, 3, 3],
      },
    },
  } as unknown as PlotStep;
}

beforeEach(() => {
  state.runColors.clear();
  state.runColors.set("run1", PALETTE[0]);
  localStorage.setItem(PSD_MAX_FREQ_KEY, "500");
});

test("psdAccelTraces emits total plus x/y/z per step, total emphasized", () => {
  const plots: PlotSeries[] = [{ version: 1, steps: [accelStep("s1")] } as PlotSeries];
  const traces = psdAccelTraces(["run1"], plots, ["s1"]);
  expect(traces.map((t) => t.label)).toEqual([
    "s1 (total)",
    "s1 (x)",
    "s1 (y)",
    "s1 (z)",
  ]);
  const [total, x, y, z] = traces;
  expect(total.width).toBeGreaterThan(1.25);
  expect(total.dashed).toBe(false);
  for (const axis of [x, y, z]) expect(axis.dashed).toBe(true);
  const amp2 = (v: number) => v * v;
  for (let b = 0; b < FREQ.length; b++) {
    expect(amp2(total.y[b])).toBeCloseTo(amp2(x.y[b]) + amp2(y.y[b]) + amp2(z.y[b]), 10);
  }
});

test("psdAccelTraces is empty when steps carry no accel psd", () => {
  const step = accelStep("s1");
  (step.psd as { accel: unknown }).accel = null;
  const plots: PlotSeries[] = [{ version: 1, steps: [step] } as PlotSeries];
  expect(psdAccelTraces(["run1"], plots, ["s1"])).toEqual([]);
});
