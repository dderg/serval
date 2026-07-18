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

const { pathEntries } = await import("../src/path-chart");
const { mixColor } = await import("../src/charts-core");

test("pathEntries ramps step colors within a run and keeps single-step runs at base", () => {
  const colors = new Map([["run1", "#4fb3ff"]]);
  const plots = [series([step("s1", PATH), step("s2", PATH), step("s3", PATH)])];
  const entries = pathEntries(["run1"], plots, ["s1", "s2", "s3"], colors);
  expect(entries.length).toBe(6);
  expect(entries[0].trace.color).toBe("#4fb3ff");
  expect(entries[2].trace.color).toBe(mixColor("#4fb3ff", "#ffffff", 0.275));
  expect(entries[4].trace.color).toBe(mixColor("#4fb3ff", "#ffffff", 0.55));
  expect(entries[0].trace.color).not.toBe(entries[2].trace.color);
  expect(entries[0].kind).toBe("commanded");
  expect(entries[1].kind).toBe("actual");
  expect(entries[1].trace.color).toBe(entries[0].trace.color);
});

test("pathEntries labels steps alone for one run and run-qualified for several", () => {
  const plots = [series([step("s1", PATH)]), series([step("s1", PATH)])];
  const single = pathEntries(["run1"], [plots[0]], ["s1"], new Map());
  expect(single[0].label).toBe("s1");
  const multi = pathEntries(["run1", "run2"], plots, ["s1"], new Map());
  expect(multi[0].label).toBe("run1 · s1");
  expect(multi[2].label).toBe("run2 · s1");
});
