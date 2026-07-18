import { beforeAll, expect, test } from "bun:test";
import { registerDom } from "./dom";
import type { PlotRingdownSource } from "../src/generated/PlotRingdownSource";

registerDom();

const RUN = "synthetic_run";
const FS = 4000;

function syntheticSource(): PlotRingdownSource {
  const tail = (phase: number) =>
    Array.from({ length: 2048 }, (_, k) =>
      Math.exp((-30 * k) / FS) * Math.sin((2 * Math.PI * 120 * k) / FS + phase)
    );
  return {
    source: "accel",
    unit: "mm/s²",
    fs_hz: FS,
    modes: [],
    psd_freq_hz: [0, 100, 200],
    psd: [1e-6, 1e-3, 1e-6],
    tails: [
      { start_s: 0.0, value: tail(0) },
      { start_s: 1.0, value: tail(0.5) },
    ],
  };
}

let dynamics: typeof import("../src/dynamics");

beforeAll(async () => {
  const { state } = await import("../src/state");
  state.runColors.set(RUN, "#58a6ff");
  dynamics = await import("../src/dynamics");
});

test("brush selection survives a refresh and drives the per-tail PSD", () => {
  const inst = dynamics.createRingdownChart("ringdown", "accel");
  const entries = [{ name: RUN, src: syntheticSource() }];
  dynamics.updateRingdownChart(inst, entries);
  expect(inst.plot).not.toBeNull();
  expect(inst.selection).toBeNull();
  expect(inst.psdWrap.textContent).toContain("full dwell");

  const brushed: [number, number] = [50, 400];
  inst.selection = brushed;
  inst.renderPsd();
  expect(inst.psdWrap.textContent).toContain("per tail");

  dynamics.updateRingdownChart(inst, [{ name: RUN, src: syntheticSource() }]);
  expect(inst.selection).toEqual(brushed);
  expect(inst.psdWrap.textContent).toContain("50–400ms");
  expect(inst.psdWrap.textContent).toContain("per tail");
});

test("a selection past the new data extent is dropped on refresh", () => {
  const inst = dynamics.createRingdownChart("ringdown", "accel");
  dynamics.updateRingdownChart(inst, [{ name: RUN, src: syntheticSource() }]);
  inst.selection = [0, 10_000];
  dynamics.updateRingdownChart(inst, [{ name: RUN, src: syntheticSource() }]);
  expect(inst.selection).toBeNull();
  expect(inst.psdWrap.textContent).toContain("full dwell");
});

test("a selection too short for Welch falls back to the full-dwell PSD", () => {
  const inst = dynamics.createRingdownChart("ringdown", "accel");
  dynamics.updateRingdownChart(inst, [{ name: RUN, src: syntheticSource() }]);
  inst.selection = [0, 5];
  inst.renderPsd();
  expect(inst.psdWrap.textContent).toContain("full dwell");
});
