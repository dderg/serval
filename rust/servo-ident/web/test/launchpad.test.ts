import { expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { LAUNCHPAD_GROUPS, buildCommand, missingRequired } = await import("../src/launchpad");

function macro(name: string) {
  for (const g of LAUNCHPAD_GROUPS) {
    for (const m of g.macros) if (m.name === name) return m;
  }
  throw new Error(`no launchpad macro ${name}`);
}

test("buildCommand emits only the params the user filled, prefixed by the macro name", () => {
  const line = buildCommand(macro("SERVO_APPLY_GAINS"), { SPEED_GAIN: "300", INTEGRAL: "2500" });
  expect(line).toBe("SERVO_APPLY_GAINS SPEED_GAIN=300 INTEGRAL=2500");
});

test("buildCommand skips blank and whitespace-only values", () => {
  const line = buildCommand(macro("SERVO_MEASURE_TRACKING"), { AXIS: "Y", SPEED: "", ACCEL: "   ", NAME: "run1" });
  expect(line).toBe("SERVO_MEASURE_TRACKING AXIS=Y NAME=run1");
});

test("buildCommand keeps the spec param order, not the value insertion order", () => {
  const line = buildCommand(macro("SERVO_APPLY_GAINS"), { INTEGRAL: "2500", POS_GAIN: "400" });
  expect(line).toBe("SERVO_APPLY_GAINS POS_GAIN=400 INTEGRAL=2500");
});

test("buildCommand trims surrounding whitespace off each value", () => {
  const line = buildCommand(macro("SERVO_SET_INERTIA_RATIO"), { RATIO: "  120 " });
  expect(line).toBe("SERVO_SET_INERTIA_RATIO RATIO=120");
});

test("buildCommand with no values is just the bare macro name", () => {
  expect(buildCommand(macro("SERVO_FIT_DYNAMICS"), {})).toBe("SERVO_FIT_DYNAMICS");
});

test("missingRequired names every unfilled required param", () => {
  expect(missingRequired(macro("SERVO_STRAIN_COMP_TUNE"), {})).toEqual(["RUN"]);
  expect(missingRequired(macro("SERVO_STRAIN_COMP_TUNE"), { RUN: "strain_20260718" })).toEqual([]);
});

test("missingRequired treats a whitespace-only value as unfilled", () => {
  expect(missingRequired(macro("SERVO_SET_INERTIA_RATIO"), { RATIO: "  " })).toEqual(["RATIO"]);
});

test("missingRequired ignores optional params left blank", () => {
  expect(missingRequired(macro("SERVO_MEASURE_TRACKING"), {})).toEqual([]);
});
