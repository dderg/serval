import { afterAll, beforeAll, expect, test } from "bun:test";
import { registerDom, installFetchStub, installDomHarness, indexHtmlBody, nextFrame, settleDom, RUN_NAME } from "./dom";
import type * as ApiMod from "../src/api";
import type * as ClientMod from "../src/queries/client";
import type * as RunsQueryMod from "../src/queries/runs";
import type * as DriveQueryMod from "../src/queries/drive";
import type * as RunsMod from "../src/runs";
import type * as StoreMod from "../src/store";
import type { html as htmlTag, render as renderFn } from "htm/preact";
import type { state as stateValue } from "../src/state";
import type { VNode } from "preact";

registerDom();
const { unmatched } = installFetchStub();

// Record the run mutation endpoints (note/analyze/delete) the shared stub does
// not answer, and reply success so their wiring can be exercised. The delete
// path invalidates + refetches (the stub re-serves the fixture run), so the
// assertion is that the endpoint was hit, not that the row vanished for good.
const mutationCalls: string[] = [];
const baseFetch = globalThis.fetch;
globalThis.fetch = (async (input: RequestInfo | URL, init?: RequestInit) => {
  const req = input instanceof Request ? input : new Request(input, init);
  const path = new URL(req.url, "http://127.0.0.1/").pathname;
  const method = req.method.toUpperCase();
  if (method === "DELETE" && /^\/api\/runs\/[^/]+$/.test(path)) {
    mutationCalls.push(`DELETE ${path}`);
    return new Response("{}", { status: 200, headers: { "Content-Type": "application/json" } });
  }
  if (method === "POST" && /^\/api\/runs\/[^/]+\/(analyze|note)$/.test(path)) {
    mutationCalls.push(`POST ${path}`);
    return new Response(JSON.stringify({ note: "" }), {
      status: 200,
      headers: { "Content-Type": "application/json" },
    });
  }
  return baseFetch(req);
}) as typeof fetch;

const { consoleErrors, cleanup } = installDomHarness();

async function settle() {
  await settleDom();
  await nextFrame();
}

let html: typeof htmlTag;
let render: typeof renderFn;
let api: typeof ApiMod;
let client: typeof ClientMod;
let runsQ: typeof RunsQueryMod;
let driveQ: typeof DriveQueryMod;
let runs: typeof RunsMod;
let appState: typeof stateValue;
let store: typeof StoreMod;

async function loadRuns() {
  await settle();
  await client.queryClient.refetchQueries({ queryKey: runsQ.runKeys.all, type: "all" });
  await settle();
  await settle();
}

function pageRoot(): HTMLElement {
  const el = document.getElementById("page-root");
  if (!el) throw new Error("no #page-root");
  return el;
}

beforeAll(async () => {
  document.body.innerHTML = indexHtmlBody();
  document.body.insertAdjacentHTML("afterbegin", `<input type="text" id="moonraker-url">`);
  const host = document.createElement("div");
  host.id = "page-root";
  document.body.appendChild(host);
  // These app modules read localStorage/the DOM at import time, so they must be
  // imported only after happy-dom is registered and the fixture DOM + fetch stub
  // exist — the documented module-loading-boundary exception to static import.
  ({ html, render } = await import("htm/preact"));
  api = await import("../src/api");
  client = await import("../src/queries/client");
  runsQ = await import("../src/queries/runs");
  driveQ = await import("../src/queries/drive");
  runs = await import("../src/runs");
  appState = (await import("../src/state")).state;
  store = await import("../src/store");

  runs.startRunsPolling((r) => void runs.reconcileRuns(r));
  await driveQ.fetchDriveState();
  await loadRuns();
  render(html`<${client.QueryRoot}><${runs.TunePage} /><//>`, pageRoot());
  await settle();
  await settle();
});

afterAll(() => {
  render(null as unknown as VNode, pageRoot());
  cleanup();
  globalThis.fetch = baseFetch;
});

test("tune page renders the full body with the expected controls, ids and classes", () => {
  expect(consoleErrors).toEqual([]);
  expect(unmatched).toEqual([]);
  const root = pageRoot();
  expect(root.querySelector(".workspace")).not.toBeNull();
  expect(root.querySelector("main.analysis")).not.toBeNull();
  expect(root.querySelector("aside.controls")).not.toBeNull();
  for (const id of [
    "journal-body",
    "drive-panel",
    "drive-groups",
    "drive-apply-btn",
    "pending-preview",
    "metrics-table",
    "sweep-metrics-chart",
    "psd-charts",
    "psd-max-freq",
    "psd-step-chips",
    "charts",
    "time-step-chips",
    "accel-psd-section",
    "frf-section",
    "ringdown-section",
    "path-section",
    "path-canvas",
    "console-input",
    "launchpad-body",
    "sent-log",
  ]) {
    expect(document.getElementById(id), `#${id} present`).not.toBeNull();
  }
  expect(document.querySelectorAll("button.motor-view-btn").length).toBeGreaterThan(0);
  expect(document.getElementById("psd-unit-um")).not.toBeNull();
});

test("runs table lists the fixture run, auto-selected", () => {
  const rows = [...document.querySelectorAll("#journal-body tr")];
  expect(rows.length).toBe(1);
  expect(rows[0].textContent).toContain("cal_attempt3");
  expect(rows[0].className).toContain("selected");
});

test("chart sections mount uPlot and canvas engines into their hosts", () => {
  expect(document.querySelectorAll(".uplot").length).toBeGreaterThan(0);
  expect(document.querySelectorAll("canvas").length).toBeGreaterThan(0);
});

test("charts survive an unchanged runs refetch without page DOM replacement", async () => {
  const canvasesBefore = [...document.querySelectorAll("canvas")];
  const plotsBefore = [...document.querySelectorAll(".uplot")];
  const rowsBefore = [...document.querySelectorAll("#journal-body tr")];

  await loadRuns();

  expect(consoleErrors).toEqual([]);
  const canvasesAfter = [...document.querySelectorAll("canvas")];
  const plotsAfter = [...document.querySelectorAll(".uplot")];
  const rowsAfter = [...document.querySelectorAll("#journal-body tr")];
  expect(canvasesAfter.length).toBe(canvasesBefore.length);
  canvasesAfter.forEach((c, i) => expect(c === canvasesBefore[i]).toBe(true));
  expect(plotsAfter.length).toBe(plotsBefore.length);
  plotsAfter.forEach((p, i) => expect(p === plotsBefore[i]).toBe(true));
  expect(rowsAfter.length).toBe(rowsBefore.length);
  rowsAfter.forEach((r, i) => expect(r === rowsBefore[i]).toBe(true));
});

test("an in-progress note edit survives an unchanged runs refetch", async () => {
  const noteCell = document.querySelector<HTMLElement>("#journal-body td.run-note");
  expect(noteCell).not.toBeNull();
  noteCell!.click();
  await settle();
  const noteInput = document.querySelector<HTMLInputElement>("#journal-body input.run-note-input");
  expect(noteInput).not.toBeNull();
  noteInput!.value = "wip note";

  await loadRuns();

  const after = document.querySelector<HTMLInputElement>("#journal-body input.run-note-input");
  expect(after === noteInput).toBe(true);
  expect(after!.value).toBe("wip note");
  after!.dispatchEvent(new KeyboardEvent("keydown", { key: "Escape", bubbles: true }));
  await settle();
});

test("pinning a run keeps it selected and assigns a swatch color", async () => {
  const pin = document.querySelector<HTMLButtonElement>("#journal-body .pin-toggle");
  expect(pin).not.toBeNull();
  pin!.click();
  await settle();
  expect(appState.pinned.has(RUN_NAME)).toBe(true);
  expect(appState.selected.has(RUN_NAME)).toBe(true);
  const row = document.querySelector("#journal-body tr");
  expect(row!.querySelector(".swatch")).not.toBeNull();
  expect(() => runs.runColor(RUN_NAME)).not.toThrow();
  document.querySelector<HTMLButtonElement>("#journal-body .pin-toggle.pinned")!.click();
  await settle();
});

test("clicking a step filter chip narrows the step filter", async () => {
  const chips = [...document.querySelectorAll<HTMLButtonElement>("#psd-step-chips button.chip")];
  expect(chips.length).toBeGreaterThan(1);
  const stepChip = chips[chips.length - 1];
  const stepName = stepChip.textContent!.trim();
  stepChip.click();
  await settle();
  const filter = appState.stepFilter;
  expect(filter).not.toBeNull();
  expect(filter!.has(stepName)).toBe(true);
  document.querySelector<HTMLButtonElement>("#psd-step-chips button.chip")!.click();
  await settle();
});

test("active drive edits survive a drive_state refetch", async () => {
  const ds = driveQ.driveData();
  const param = ds.params[0];
  const motor = Object.keys(ds.motors)[0];
  const bumped = ds.motors[motor][param.c_code] + 100;
  appState.drive.pending = { [param.name]: { [motor]: bumped } };
  store.notify();
  await settle();

  const preview = document.getElementById("pending-preview")!;
  const applyBtn = document.getElementById("drive-apply-btn") as HTMLButtonElement;
  expect(preview.querySelectorAll(".pending-line").length).toBeGreaterThan(0);
  expect(applyBtn.disabled).toBe(false);

  await driveQ.fetchDriveState();
  await settle();

  const previewAfter = document.getElementById("pending-preview")!;
  const applyAfter = document.getElementById("drive-apply-btn") as HTMLButtonElement;
  expect(previewAfter.querySelectorAll(".pending-line").length).toBeGreaterThan(0);
  expect(applyAfter.disabled).toBe(false);
  expect(appState.drive.pending[param.name][motor]).toBe(bumped);

  appState.drive.pending = {};
  store.notify();
  await settle();
});

test("right-clicking a run opens the context menu with the run actions", async () => {
  const row = document.querySelector<HTMLElement>("#journal-body tr")!;
  row.dispatchEvent(
    new MouseEvent("contextmenu", { bubbles: true, cancelable: true, clientX: 40, clientY: 40 }),
  );
  await settle();
  const menu = document.querySelector(".context-menu");
  expect(menu).not.toBeNull();
  const labels = [...menu!.querySelectorAll("button")].map((b) => b.textContent!.trim());
  expect(labels).toContain("→ console");
  expect(labels).toContain("delete");
  expect(document.querySelectorAll(".context-menu").length).toBe(1);
});

test("the → console action prefills the console with the run command", async () => {
  const menu = document.querySelector(".context-menu")!;
  const consoleItem = [...menu.querySelectorAll<HTMLButtonElement>("button")].find(
    (b) => b.textContent!.trim() === "→ console",
  )!;
  consoleItem.click();
  await settle();
  const consoleInput = document.getElementById("console-input") as HTMLTextAreaElement;
  expect(consoleInput.value.length).toBeGreaterThan(0);
});

test("deleting a run through the context menu hits the delete endpoint", async () => {
  const row = document.querySelector<HTMLElement>("#journal-body tr")!;
  row.dispatchEvent(
    new MouseEvent("contextmenu", { bubbles: true, cancelable: true, clientX: 40, clientY: 40 }),
  );
  await settle();
  const before = mutationCalls.length;
  const del = [...document.querySelectorAll<HTMLButtonElement>(".context-menu button")].find(
    (b) => b.textContent!.trim() === "delete",
  )!;
  del.click();
  await settle();
  await settle();
  expect(mutationCalls).toContain(`DELETE /api/runs/${RUN_NAME}`);
  expect(mutationCalls.length).toBeGreaterThan(before);
});

test("analyze fires for a run without results", async () => {
  await loadRuns();
  const existing = client.queryClient.getQueryData(runsQ.runKeys.all) as Record<string, unknown>[];
  const fake = { ...existing[0], name: "cal_pending", has_results: false, note: null };
  client.queryClient.setQueryData(runsQ.runKeys.all, [...existing, fake]);
  store.notify();
  await settle();
  const before = mutationCalls.length;
  const analyzeBtn = [
    ...document.querySelectorAll<HTMLButtonElement>("#journal-body td.actions button"),
  ].find((b) => b.textContent!.trim() === "analyze")!;
  expect(analyzeBtn).not.toBeUndefined();
  analyzeBtn.click();
  await settle();
  await settle();
  expect(mutationCalls).toContain("POST /api/runs/cal_pending/analyze");
  expect(mutationCalls.length).toBeGreaterThan(before);
  client.queryClient.setQueryData(runsQ.runKeys.all, existing);
  store.notify();
  await settle();
});

test("journal page renders the unfiltered run journal with console and launchpad", async () => {
  render(null as unknown as VNode, pageRoot());
  await settle();
  appState.page = "journal";
  await loadRuns();
  render(html`<${client.QueryRoot}><${runs.JournalPage} /><//>`, pageRoot());
  await settle();
  await settle();
  const root = pageRoot();
  expect(root.querySelector(".workspace.single")).not.toBeNull();
  expect(root.querySelector(".journal-wrap")).not.toBeNull();
  expect(document.getElementById("journal-body")).not.toBeNull();
  expect([...document.querySelectorAll("#journal-body tr")].length).toBe(1);
  expect(document.getElementById("console-input")).not.toBeNull();
  expect(document.getElementById("launchpad-body")).not.toBeNull();
  expect(document.getElementById("psd-charts")).toBeNull();
  expect(document.getElementById("metrics-table")).toBeNull();
});
