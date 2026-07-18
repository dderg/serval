import { expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { pathTraces, stepFullPath } = await import("../src/path-chart");
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

const FULL_PATH = {
  cmd_x_mm: [0, 0.5, 1],
  cmd_y_mm: [0, 0, 0],
  act_x_mm: [0, 0.45, 0.9],
  act_y_mm: [0, 0.05, 0.1],
};

test("pathTraces prefers the full-resolution path for steps the payload covers", () => {
  const full = new Map([
    [
      "run1",
      {
        version: 1,
        steps: [{ name: "s1", n_records: 3, truncated: false, path: FULL_PATH }],
      },
    ],
  ]);
  const plots = [series([step("s1", PATH), step("s2", PATH)])];
  const traces = pathTraces(["run1"], plots, ["s1", "s2"], new Map(), full);
  expect(traces.length).toBe(4);
  expect(traces[0].xs).toBe(FULL_PATH.cmd_x_mm);
  expect(traces[1].xs).toBe(FULL_PATH.act_x_mm);
  expect(traces[2].xs).toBe(PATH.cmd_x_mm);
  expect(traces[3].xs).toBe(PATH.act_x_mm);
});

test("pathTraces keeps the preview when a run has no full-resolution payload", () => {
  const traces = pathTraces(["run1"], [series([step("s1", PATH)])], ["s1"], new Map(), new Map());
  expect(traces[0].xs).toBe(PATH.cmd_x_mm);
});

test("stepFullPath resolves per step name and misses cleanly", () => {
  const payload = {
    version: 1,
    steps: [{ name: "s1", n_records: 3, truncated: false, path: FULL_PATH }],
  };
  expect(stepFullPath(payload, "s1")).toBe(FULL_PATH);
  expect(stepFullPath(payload, "s2")).toBeNull();
  expect(stepFullPath(undefined, "s1")).toBeNull();
});
