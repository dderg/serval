import { expect, test } from "bun:test";
import { summarizeMotorValues, valuesAgree } from "../src/motor-values";
import type { MotorValueEntry } from "../src/motor-values";

function entry(motor: string, value: number, original = value): MotorValueEntry {
  return { motor: `motor_${motor}`, label: motor, value, original };
}

test("valuesAgree", () => {
  expect(valuesAgree([])).toBe(false);
  expect(valuesAgree([7])).toBe(true);
  expect(valuesAgree([7, 7, 7])).toBe(true);
  expect(valuesAgree([7, 8])).toBe(false);
});

test("agreeing motors collapse to one shared value with no pending edits", () => {
  const s = summarizeMotorValues([entry("x", 550), entry("y", 550)]);
  expect(s.agree).toBe(true);
  expect(s.sharedValue).toBe(550);
  expect(s.pendingMotors).toEqual([]);
  expect(s.spreadText).toBe("x=550 y=550");
});

test("disagreeing motors have no shared value and expose the spread", () => {
  const s = summarizeMotorValues([entry("x", 550), entry("y", 600)]);
  expect(s.agree).toBe(false);
  expect(s.sharedValue).toBeNull();
  expect(s.spreadText).toBe("x=550 y=600");
});

test("pending motors are the ones whose value differs from the drive reading", () => {
  const s = summarizeMotorValues([entry("x", 700, 550), entry("y", 550), entry("z", 700, 550)]);
  expect(s.pendingMotors).toEqual(["motor_x", "motor_z"]);
  expect(s.agree).toBe(false);
});

test("a set-all edit is pending on every motor yet still agrees", () => {
  const s = summarizeMotorValues([entry("x", 700, 550), entry("y", 700, 600)]);
  expect(s.agree).toBe(true);
  expect(s.sharedValue).toBe(700);
  expect(s.pendingMotors).toEqual(["motor_x", "motor_y"]);
});

test("no motors is a loud error", () => {
  expect(() => summarizeMotorValues([])).toThrow("no motors");
});
