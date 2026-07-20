import { afterAll, beforeAll, expect, test } from "bun:test";
import { registerDom, installFetchStub, installDomHarness, indexHtmlBody, settleDom, RUN_NAME } from "./dom";
import type * as ApiMod from "../src/api";
import type * as ClientMod from "../src/queries/client";
import type * as RunsQueryMod from "../src/queries/runs";
import type * as PathQueryMod from "../src/queries/path";
import type * as StrainQueryMod from "../src/queries/strain";
import type * as RunsMod from "../src/runs";

registerDom();
const { unmatched } = installFetchStub();
const { consoleErrors, cleanup } = installDomHarness();
const settle = () => settleDom();

let api: typeof ApiMod;
let client: typeof ClientMod;
let runsQ: typeof RunsQueryMod;
let pathQ: typeof PathQueryMod;
let strainQ: typeof StrainQueryMod;
let runs: typeof RunsMod;

async function loadRuns() {
  await settle();
  await client.queryClient.refetchQueries({ queryKey: runsQ.runKeys.all, type: "all" });
  await settle();
  await settle();
}

beforeAll(async () => {
  document.body.innerHTML = indexHtmlBody();
  // The app modules read localStorage and the DOM at import time and boot runs
  // its bootstrap side effects on load, so they must import only after happy-dom
  // is registered and the fixture DOM + fetch stub exist. Static imports would
  // execute before this setup — the loading order is the whole point here.
  api = await import("../src/api");
  client = await import("../src/queries/client");
  runsQ = await import("../src/queries/runs");
  pathQ = await import("../src/queries/path");
  strainQ = await import("../src/queries/strain");
  runs = await import("../src/runs");
  await import("../src/boot");
  await loadRuns();
});

afterAll(async () => {
  const { render } = await import("htm/preact");
  const app = document.getElementById("app");
  if (app) render(null, app);
  cleanup();
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
  const existing = runsQ.runsData();
  const newRun = { ...existing[0], name: "cal_attempt4", has_results: false, verdict: null, note: null };
  const stubFetch = globalThis.fetch;
  const json = (body: string) =>
    new Response(body, { status: 200, headers: { "Content-Type": "application/json" } });
  globalThis.fetch = (async (input: RequestInfo | URL, init?: RequestInit) => {
    const url = input instanceof Request ? input.url : String(input);
    const path = url.startsWith("http") ? new URL(url).pathname : url.split("?")[0];
    if (path === "/api/runs") return json(JSON.stringify([...existing, newRun]));
    if (path === `/api/runs/${newRun.name}/manifest`) return json("null");
    return stubFetch(input, init);
  }) as typeof fetch;
  try {
    await loadRuns();
    expect(consoleErrors).toEqual([]);
    expect(runsQ.runsData().some((r) => r.name === newRun.name)).toBe(true);
    expect(document.querySelectorAll("#journal-body tr").length).toBe(2);
  } finally {
    globalThis.fetch = stubFetch;
    await loadRuns();
  }
  expect(document.querySelectorAll("#journal-body tr").length).toBe(1);
});

test("run data is cached in the query client, keyed by the fixture run", () => {
  expect(runsQ.runsData().map((r) => r.name)).toEqual([RUN_NAME]);
  expect(runsQ.detailData(RUN_NAME)).toBeDefined();
  expect(client.queryClient.getQueryData(runsQ.runKeys.plot(RUN_NAME))).toBeDefined();
});
test("an unchanged runs refetch retries a missing detail", async () => {
  client.queryClient.removeQueries({ queryKey: runsQ.runKeys.detail(RUN_NAME) });
  expect(runsQ.detailData(RUN_NAME)).toBeUndefined();
  await loadRuns();
  expect(runsQ.detailData(RUN_NAME)).toBeDefined();
});


function runsQuery() {
  const q = client.queryClient.getQueryCache().find({ queryKey: runsQ.runKeys.all });
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
  const cache = client.queryClient.getQueryCache();
  client.queryClient.setQueryData(runsQ.runKeys.detail(RUN_NAME), { probe: "detail" });
  client.queryClient.setQueryData(runsQ.runKeys.plot(RUN_NAME), { probe: "plot" });
  client.queryClient.setQueryData(pathQ.runPathKey(RUN_NAME), { probe: "path" });
  client.queryClient.setQueryData(strainQ.strainKey(RUN_NAME), { probe: "strain" });
  client.queryClient.removeQueries({ queryKey: ["runs", RUN_NAME] });
  await settle();
  expect(cache.find({ queryKey: runsQ.runKeys.detail(RUN_NAME) })).toBeUndefined();
  expect(cache.find({ queryKey: runsQ.runKeys.plot(RUN_NAME) })).toBeUndefined();
  expect(cache.find({ queryKey: pathQ.runPathKey(RUN_NAME) })).toBeUndefined();
  expect(cache.find({ queryKey: strainQ.strainKey(RUN_NAME) })).toBeUndefined();
  expect(cache.find({ queryKey: runsQ.runKeys.all })).toBeDefined();
});

async function goto(page: string) {
  location.hash = `#/${page}`;
  window.dispatchEvent(new Event("hashchange"));
  await settle();
  await settle();
}

test("one App mount: hash routing swaps page components under a single persistent topbar", async () => {
  expect(document.querySelectorAll("#app").length).toBe(1);
  expect(document.querySelectorAll("header.topbar").length).toBe(1);
  expect(document.querySelectorAll("#page-root").length).toBe(1);
  const topbar = document.querySelector("header.topbar");
  const pageRoot = document.getElementById("page-root");

  await goto("journal");
  expect(document.querySelector(".journal-wrap")).not.toBeNull();
  expect(document.getElementById("metrics-table")).toBeNull();

  await goto("docs");
  expect(document.querySelector(".docs-section")).not.toBeNull();
  expect(document.getElementById("launchpad-body")).not.toBeNull();

  await goto("strain");
  expect(document.getElementById("strain-run-body")).not.toBeNull();

  await goto("live");
  expect(document.querySelector(".live-section")).not.toBeNull();

  await goto("tune");
  expect(document.querySelector(".metrics-section")).not.toBeNull();
  expect(document.getElementById("psd-charts")).not.toBeNull();

  expect(document.querySelector("header.topbar") === topbar).toBe(true);
  expect(document.getElementById("page-root") === pageRoot).toBe(true);
  expect(document.querySelectorAll("header.topbar").length).toBe(1);
  expect(consoleErrors).toEqual([]);
});

test("repeated route changes do not accumulate query observers or destroy session query data", async () => {
  const runData = client.queryClient.getQueryData(runsQ.runKeys.all);
  await goto("tune");
  const observersBefore = runsQuery().getObserversCount();
  for (let i = 0; i < 3; i++) {
    await goto("journal");
    await goto("tune");
  }
  expect(runsQuery().getObserversCount()).toBe(observersBefore);
  const pollers = runsQuery().observers.filter((o) => o.options.refetchInterval === 5000);
  expect(pollers.length).toBe(1);
  expect(document.querySelectorAll("header.topbar").length).toBe(1);
  expect(document.querySelectorAll("#moonraker-url").length).toBe(1);
  expect(client.queryClient.getQueryData(runsQ.runKeys.all)).toBe(runData);
  expect(consoleErrors).toEqual([]);
});
