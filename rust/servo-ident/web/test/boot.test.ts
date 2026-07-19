import { afterAll, beforeAll, expect, test } from "bun:test";
import { registerDom, installFetchStub, indexHtmlBody, RUN_NAME } from "./dom";
import type * as ApiMod from "../src/api";
import type * as QueryMod from "../src/query-client";
import type * as RunsMod from "../src/runs";

registerDom();
const { unmatched } = installFetchStub();

const intervals: Timer[] = [];
const realSetInterval = globalThis.setInterval;
(globalThis as Record<string, unknown>).setInterval = ((fn: TimerHandler, ms?: number) => {
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

const raf = () => new Promise((resolve) => requestAnimationFrame(() => resolve(null)));
const macrotask = () =>
  new Promise((resolve) => {
    const channel = new MessageChannel();
    channel.port1.onmessage = () => resolve(null);
    channel.port2.postMessage(0);
  });

async function settle() {
  await raf();
  await macrotask();
}

let api: typeof ApiMod;
let query: typeof QueryMod;
let runs: typeof RunsMod;

async function loadRuns() {
  await settle();
  await query.queryClient.refetchQueries({ queryKey: query.queryKeys.runs, type: "all" });
  await settle();
}

beforeAll(async () => {
  document.body.innerHTML = indexHtmlBody();
  // The app modules read localStorage and the DOM at import time and boot runs
  // its bootstrap side effects on load, so they must import only after happy-dom
  // is registered and the fixture DOM + fetch stub exist. Static imports would
  // execute before this setup — the loading order is the whole point here.
  api = await import("../src/api");
  query = await import("../src/query-client");
  runs = await import("../src/runs");
  await import("../src/boot");
  await loadRuns();
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

test("an unchanged runs refetch causes no drive/journal churn and keeps active input text", async () => {
  const noteCell = document.querySelector<HTMLElement>("#journal-body td.run-note");
  expect(noteCell).not.toBeNull();
  noteCell!.click();
  await settle();
  const noteInput = document.querySelector<HTMLInputElement>("#journal-body input.run-note-input");
  expect(noteInput).not.toBeNull();
  noteInput!.value = "wip note";

  const driveBtn = document.querySelector("#drive-refresh-btn");
  const canvasesBefore = [...document.querySelectorAll("canvas")];
  const chartBoxesBefore = [...document.querySelectorAll(".uplot")];
  const rowsBefore = [...document.querySelectorAll("#journal-body tr")];

  await loadRuns();

  expect(consoleErrors).toEqual([]);
  expect(document.querySelector("#drive-refresh-btn") === driveBtn).toBe(true);
  const canvasesAfter = [...document.querySelectorAll("canvas")];
  const chartBoxesAfter = [...document.querySelectorAll(".uplot")];
  const rowsAfter = [...document.querySelectorAll("#journal-body tr")];
  expect(canvasesAfter.length).toBe(canvasesBefore.length);
  canvasesAfter.forEach((c, i) => expect(c === canvasesBefore[i]).toBe(true));
  chartBoxesAfter.forEach((c, i) => expect(c === chartBoxesBefore[i]).toBe(true));
  expect(rowsAfter.length).toBe(rowsBefore.length);
  rowsAfter.forEach((r, i) => expect(r === rowsBefore[i]).toBe(true));

  const noteInputAfter = document.querySelector<HTMLInputElement>("#journal-body input.run-note-input");
  expect(noteInputAfter === noteInput).toBe(true);
  expect(noteInputAfter!.value).toBe("wip note");

  noteInput!.dispatchEvent(new KeyboardEvent("keydown", { key: "Escape", bubbles: true }));
});

test("a newly appeared run shows up on the next query poll", async () => {
  const existing = api.runsData();
  const newRun = { ...existing[0], name: "cal_attempt4", has_results: false, verdict: null, note: null };
  const stubFetch = globalThis.fetch;
  const json = (body: string) =>
    new Response(body, { status: 200, headers: { "Content-Type": "application/json" } });
  globalThis.fetch = (async (input: RequestInfo | URL, init?: RequestInit) => {
    const url = String(input);
    const path = url.startsWith("http") ? new URL(url).pathname : url.split("?")[0];
    if (path === "/api/runs") return json(JSON.stringify([...existing, newRun]));
    if (path === `/api/runs/${newRun.name}/manifest`) return json("null");
    return stubFetch(input, init);
  }) as typeof fetch;
  try {
    await loadRuns();
    expect(consoleErrors).toEqual([]);
    expect(api.runsData().some((r) => r.name === newRun.name)).toBe(true);
    expect(document.querySelectorAll("#journal-body tr").length).toBe(2);
  } finally {
    globalThis.fetch = stubFetch;
    await loadRuns();
  }
  expect(document.querySelectorAll("#journal-body tr").length).toBe(1);
});

test("run data is cached in the query client, keyed by the fixture run", () => {
  expect(api.runsData().map((r) => r.name)).toEqual([RUN_NAME]);
  expect(api.detailData(RUN_NAME)).toBeDefined();
  expect(query.queryClient.getQueryData(query.queryKeys.plotSeries(RUN_NAME))).toBeDefined();
});

function runsQuery() {
  const q = query.queryClient.getQueryCache().find({ queryKey: query.queryKeys.runs });
  expect(q).toBeDefined();
  return q!;
}

test("run polling lives in exactly one global observer, not in the table component", () => {
  const pollers = runsQuery().observers.filter(
    (o) => o.options.refetchInterval === 5000 && o.options.refetchIntervalInBackground === false
  );
  expect(pollers.length).toBe(1);
});

test("startRunsPolling is idempotent — a second call adds no observer", () => {
  const before = runsQuery().getObserversCount();
  runs.startRunsPolling();
  runs.startRunsPolling();
  expect(runsQuery().getObserversCount()).toBe(before);
  const pollers = runsQuery().observers.filter((o) => o.options.refetchInterval === 5000);
  expect(pollers.length).toBe(1);
});

test("deleting a run drops every ['runs', name] cache, not just detail and plot", async () => {
  const cache = query.queryClient.getQueryCache();
  query.queryClient.setQueryData(query.queryKeys.runDetail(RUN_NAME), { probe: "detail" });
  query.queryClient.setQueryData(query.queryKeys.plotSeries(RUN_NAME), { probe: "plot" });
  query.queryClient.setQueryData(query.queryKeys.runPath(RUN_NAME), { probe: "path" });
  query.queryClient.setQueryData(query.queryKeys.strain(RUN_NAME), { probe: "strain" });
  query.queryClient.removeQueries({ queryKey: ["runs", RUN_NAME] });
  await settle();
  expect(cache.find({ queryKey: query.queryKeys.runDetail(RUN_NAME) })).toBeUndefined();
  expect(cache.find({ queryKey: query.queryKeys.plotSeries(RUN_NAME) })).toBeUndefined();
  expect(cache.find({ queryKey: query.queryKeys.runPath(RUN_NAME) })).toBeUndefined();
  expect(cache.find({ queryKey: query.queryKeys.strain(RUN_NAME) })).toBeUndefined();
  expect(cache.find({ queryKey: query.queryKeys.runs })).toBeDefined();
});
