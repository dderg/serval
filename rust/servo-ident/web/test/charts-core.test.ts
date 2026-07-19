import { beforeEach, expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { countsPerMmOrNull, countsPerMm, ferrUnitAvailability, pickSeries } = await import(
  "../src/charts-core"
);
const { MOTOR_VIEW_KEY, LIVE_UNIT_KEY } = await import("../src/state");
const { setFerrUnit } = await import("../src/units");
const { queryClient, queryKeys } = await import("../src/query-client");
import type { PlotStep } from "../src/wire";

function seedRun(name: string, motors: { name: string; counts_per_mm: number | null }[]) {
  queryClient.setQueryData(queryKeys.runDetail(name), {
    mtime_utc: "",
    has_results: false,
    manifest: { experiment: "x", steps: [], motors },
    results: null,
  });
}

beforeEach(() => {
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

const { fillFilterChips } = await import("../src/charts-core");

function chipHarness() {
  const container = document.createElement("div");
  let filter: Set<string> | null = null;
  let changes = 0;
  const fill = () =>
    fillFilterChips(
      container,
      "all",
      "show everything",
      "item",
      [
        { key: "a", label: "a", swatch: "#4fb3ff" },
        { key: "b", label: "b" },
        { key: "c", label: "c" },
      ],
      () => filter,
      (next) => {
        filter = next;
      },
      () => {
        changes++;
        fill();
      }
    );
  fill();
  const chips = () => [...container.querySelectorAll("button")];
  const click = (label: string, shift = false) => {
    const chip = chips().find((c) => c.textContent === label);
    if (!chip) throw new Error(`no chip labeled ${label}`);
    chip.dispatchEvent(new MouseEvent("click", { shiftKey: shift }));
  };
  return { container, click, chips, getFilter: () => filter, getChanges: () => changes };
}

test("fillFilterChips: plain click is exclusive, clicking the lone selection clears", () => {
  const h = chipHarness();
  expect(h.chips()[0].classList.contains("active")).toBe(true);
  h.click("a");
  expect([...h.getFilter()!]).toEqual(["a"]);
  expect(h.chips()[0].classList.contains("active")).toBe(false);
  h.click("b");
  expect([...h.getFilter()!]).toEqual(["b"]);
  h.click("b");
  expect(h.getFilter()).toBeNull();
  expect(h.chips()[0].classList.contains("active")).toBe(true);
});

test("fillFilterChips: shift+click grows and shrinks, full or empty selection means all", () => {
  const h = chipHarness();
  h.click("a");
  h.click("b", true);
  expect([...h.getFilter()!].sort()).toEqual(["a", "b"]);
  h.click("c", true);
  expect(h.getFilter()).toBeNull();
  h.click("a", true);
  expect([...h.getFilter()!].sort()).toEqual(["b", "c"]);
  h.click("b", true);
  h.click("c", true);
  expect(h.getFilter()).toBeNull();
});

test("fillFilterChips: the all chip clears any selection and swatches render", () => {
  const h = chipHarness();
  h.click("a");
  h.click("all");
  expect(h.getFilter()).toBeNull();
  expect(h.getChanges()).toBe(2);
  const aChip = h.chips().find((c) => c.textContent === "a")!;
  expect(aChip.querySelector(".swatch")).not.toBeNull();
});
