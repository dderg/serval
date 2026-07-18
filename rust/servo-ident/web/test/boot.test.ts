import { afterAll, beforeAll, expect, test } from "bun:test";
import { registerDom, installFetchStub, indexHtmlBody, RUN_NAME } from "./dom";

registerDom();
const { unmatched } = installFetchStub();

const intervals: ReturnType<typeof setInterval>[] = [];
const realSetInterval = globalThis.setInterval;
(globalThis as Record<string, any>).setInterval = ((fn: TimerHandler, ms?: number) => {
  const id = realSetInterval(fn as () => void, ms ?? 0);
  intervals.push(id);
  return id;
}) as typeof setInterval;

const consoleErrors: unknown[][] = [];
const realConsoleError = console.error;
console.error = (...args: unknown[]) => {
  consoleErrors.push(args);
  realConsoleError(...args);
};

let boot: typeof import("../src/boot");

beforeAll(async () => {
  document.body.innerHTML = indexHtmlBody();
  boot = await import("../src/boot");
  await boot.tick();
  await new Promise((resolve) => setTimeout(resolve, 50));
  await boot.tick();
  await new Promise((resolve) => setTimeout(resolve, 50));
});

afterAll(() => {
  for (const id of intervals) clearInterval(id);
  console.error = realConsoleError;
});

test("boot renders the shell without errors", () => {
  expect(consoleErrors).toEqual([]);
  expect(unmatched).toEqual([]);
  expect(document.querySelectorAll("#page-tabs a.tab").length).toBeGreaterThan(0);
});

test("runs table lists the fixture run, selected", () => {
  const rows = [...document.querySelectorAll("#journal-body tr")];
  expect(rows.length).toBe(1);
  expect(rows[0].textContent).toContain("cal_attempt3");
  expect(rows[0].className).toContain("selected");
});

test("charts exist for the selected run", () => {
  expect(document.querySelectorAll(".uplot").length).toBeGreaterThan(0);
  expect(document.querySelectorAll("canvas").length).toBeGreaterThan(0);
});

test("a refresh tick with identical payloads causes no DOM churn", async () => {
  const canvasesBefore = [...document.querySelectorAll("canvas")];
  const chartBoxesBefore = [...document.querySelectorAll(".uplot")];
  await boot.tick();
  expect(consoleErrors).toEqual([]);
  const canvasesAfter = [...document.querySelectorAll("canvas")];
  const chartBoxesAfter = [...document.querySelectorAll(".uplot")];
  expect(canvasesAfter.length).toBe(canvasesBefore.length);
  canvasesAfter.forEach((c, i) => expect(c === canvasesBefore[i]).toBe(true));
  chartBoxesAfter.forEach((c, i) => expect(c === chartBoxesBefore[i]).toBe(true));
});

test("run detail state is keyed by the fixture run", async () => {
  const { state } = await import("../src/state");
  expect(state.runs.map((r) => r.name)).toEqual([RUN_NAME]);
  expect(state.details.has(RUN_NAME)).toBe(true);
  expect(state.plotSeries.has(RUN_NAME)).toBe(true);
});
