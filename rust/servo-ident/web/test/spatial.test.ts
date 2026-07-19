import { expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { spatialCoeffs, projectRow, fitViewport, tickStepMm } = await import("../src/spatial");

const COREXY = {
  modes: ["x", "y"],
  axes: ["motor_a", "motor_b"],
  frame: [
    [0.5, -0.5],
    [0.5, 0.5],
  ],
};
const SLOTS = { motor_a: 0, motor_b: 1 };
const CPM = { slot0: 1000, slot1: 500 };

test("spatialCoeffs divides the frame columns by each slot's counts_per_mm", () => {
  const coeffs = spatialCoeffs(COREXY, SLOTS, CPM);
  expect(coeffs).toEqual({
    x: { slot0: 0.5 / 1000, slot1: -0.5 / 500 },
    y: { slot0: 0.5 / 1000, slot1: 0.5 / 500 },
  });
});

test("spatialCoeffs reports what is missing instead of guessing", () => {
  expect(spatialCoeffs(null, SLOTS, CPM)).toContain("SERVO_DUMP_TUNING");
  expect(spatialCoeffs({ ...COREXY, modes: ["x"] }, SLOTS, CPM)).toContain("needs servo x and y");
  expect(spatialCoeffs(COREXY, { motor_a: 0 }, CPM)).toContain("motor_b");
  expect(spatialCoeffs(COREXY, SLOTS, { slot0: 1000 })).toContain("slot1");
});

function series(target: (number | null)[], pos: (number | null)[]) {
  return { ferr: [], torque: [], target, pos };
}

test("projectRow maps corexy counts to cartesian mm and nulls out gaps", () => {
  const coeffs = spatialCoeffs(COREXY, SLOTS, { slot0: 1000, slot1: 1000 });
  if (typeof coeffs === "string") throw new Error(coeffs);
  const perDrive = {
    slot0: series([2000, null, 4000], [2000, 2000, 2000]),
    slot1: series([0, 0, 0], [0, 0, 0]),
  };
  // a = 2mm, b = 0mm -> x = (a - b)/2 = 1mm, y = (a + b)/2 = 1mm
  expect(projectRow(coeffs.x, perDrive, 3, "target")).toEqual([1, null, 2]);
  expect(projectRow(coeffs.y, perDrive, 3, "pos")).toEqual([1, 1, 1]);
});

test("projectRow refuses a sample when any contributing drive is absent", () => {
  const coeffs = spatialCoeffs(COREXY, SLOTS, CPM);
  if (typeof coeffs === "string") throw new Error(coeffs);
  expect(projectRow(coeffs.x, { slot0: series([1], [1]) }, 1, "target")).toEqual([null]);
});

test("fitViewport frames both paths with equal aspect and margin", () => {
  const view = fitViewport(
    [
      [0, 10],
      [0, 5],
      [null, null],
      [null, null],
    ],
    1000,
    500
  );
  expect(view).not.toBeNull();
  expect(view!.cx).toBe(5);
  expect(view!.cy).toBe(2.5);
  // spanX 10mm over 840 usable px beats spanY 5mm over 420 usable px (tie)
  expect(view!.mmPerPx).toBeCloseTo(10 / 840, 10);
});

test("fitViewport is null with no finite points", () => {
  expect(fitViewport([[null], [null], [null], [null]], 100, 100)).toBeNull();
});

test("tickStepMm picks 1-2-5 steps near the target spacing", () => {
  expect(tickStepMm(0.01)).toBe(1);
  expect(tickStepMm(0.05)).toBe(5);
  expect(tickStepMm(1)).toBe(100);
});

test("drawSpatialView renders from tap samples and reports the live deviation", async () => {
  const { drawSpatialView } = await import("../src/spatial");
  const { appendTapSamples } = await import("../src/live");
  const { queryClient } = await import("../src/queries/client");
  const { driveKey } = await import("../src/queries/drive");

  // `document` and the query cache are process-wide singletons shared with
  // every other test file bun runs in this process — mutating them here
  // without restoring leaks into whichever file happens to run next (order
  // is not alphabetical in CI), so a later file's own render can see this
  // stub instead of its real data. Always undo on the way out.
  const originalBody = document.body.innerHTML;
  const originalDriveState = queryClient.getQueryData(driveKey);
  try {
    document.body.innerHTML =
      `<button id="live-spatial-fit">fit</button>` +
      `<span id="live-spatial-note"></span>` +
      `<canvas id="live-spatial-canvas"></canvas>`;
    queryClient.setQueryData(driveKey, {
      spatial: COREXY,
      slots: SLOTS,
    });

    appendTapSamples({
      status: "streaming",
      fs_hz: 4000,
      cycle_ns: 250000,
      drive_names: ["slot0", "slot1"],
      counts_per_mm: [1000, 1000],
      first_cycle: 10,
      next_cycle: 12,
      stride: 1,
      timing: null,
      drives: {
        // commanded a=[0,1,2]mm b=0; actual b lags by 1mm on the last sample
        slot0: { ferr: [0, 0, 0], torque: [0, 0, 0], target: [0, 1000, 2000], pos: [0, 1000, 2000] },
        slot1: { ferr: [0, 0, 0], torque: [0, 0, 0], target: [0, 0, 0], pos: [0, 0, 1000] },
      },
    });
    drawSpatialView();
    const note = document.getElementById("live-spatial-note")!.textContent!;
    // dev = |(a-b)/2, (a+b)/2| gap = |(-0.5, 0.5)| mm ≈ 707 µm
    expect(note).toContain("dev 707 µm");
  } finally {
    document.body.innerHTML = originalBody;
    if (originalDriveState === undefined) {
      queryClient.removeQueries({ queryKey: driveKey });
    } else {
      queryClient.setQueryData(driveKey, originalDriveState);
    }
  }
});
