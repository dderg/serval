import { expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { pathTraces } = await import("../src/path-chart");
const { fitViewport } = await import("../src/path-view");
const { PALETTE } = await import("../src/state");
import type { PlotSeries, PlotStep } from "../src/wire";

function step(name: string, path: PlotStep["path"]): PlotStep {
  return { name, path } as PlotStep;
}

function series(steps: PlotStep[]): PlotSeries {
  return { version: 1, steps };
}

const PATH = {
  cmd_x_mm: [0, 1],
  cmd_y_mm: [0, 0],
  act_x_mm: [0, 0.9],
  act_y_mm: [0, 0.1],
};

test("pathTraces pairs a dashed commanded and solid actual trace per step", () => {
  const colors = new Map([["run1", "#123456"]]);
  const traces = pathTraces(["run1"], [series([step("s1", PATH)])], ["s1"], colors);
  expect(traces.length).toBe(2);
  expect(traces[0]).toMatchObject({
    xs: PATH.cmd_x_mm,
    ys: PATH.cmd_y_mm,
    color: "#123456",
    dash: [5, 3],
  });
  expect(traces[1]).toMatchObject({ xs: PATH.act_x_mm, ys: PATH.act_y_mm, color: "#123456" });
  expect(traces[1].dash).toBeUndefined();
  expect(traces[1].width).toBeGreaterThan(traces[0].width);
});

test("pathTraces respects the step filter and skips steps without a path", () => {
  const plots = [series([step("s1", PATH), step("s2", PATH), step("s3", null)])];
  const traces = pathTraces(["run1"], plots, ["s2", "s3"], new Map());
  expect(traces.length).toBe(2);
  expect(traces[0].xs).toBe(PATH.cmd_x_mm);
});

test("pathTraces falls back to the palette when a run has no assigned color", () => {
  const traces = pathTraces(["run1"], [series([step("s1", PATH)])], ["s1"], new Map());
  expect(traces[0].color).toBe(PALETTE[0]);
});

test("fitViewport frames any number of trace pairs", () => {
  const view = fitViewport([[0, 10], [0, 5], [null], [null], [-10], [0]], 1000, 500);
  expect(view).not.toBeNull();
  expect(view!.cx).toBe(0);
  expect(view!.cy).toBe(2.5);
  expect(view!.mmPerPx).toBeCloseTo(20 / 840, 10);
});
