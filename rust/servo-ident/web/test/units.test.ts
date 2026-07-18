import { beforeEach, expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { loadFerrUnit, setFerrUnit } = await import("../src/units");
const { LIVE_UNIT_KEY } = await import("../src/state");

beforeEach(() => {
  localStorage.removeItem(LIVE_UNIT_KEY);
});

test("loadFerrUnit defaults to µm", () => {
  expect(loadFerrUnit()).toBe("µm");
});

test("setFerrUnit persists under the live unit key so live and analysis share one preference", () => {
  setFerrUnit("counts");
  expect(localStorage.getItem(LIVE_UNIT_KEY)).toBe("counts");
  expect(loadFerrUnit()).toBe("counts");
  setFerrUnit("µm");
  expect(loadFerrUnit()).toBe("µm");
});

test("loadFerrUnit ignores garbage values and falls back to µm", () => {
  localStorage.setItem(LIVE_UNIT_KEY, "furlongs");
  expect(loadFerrUnit()).toBe("µm");
});
