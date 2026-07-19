import { beforeEach, expect, test } from "bun:test";
import { act } from "preact/test-utils";
import { registerDom } from "./dom";

registerDom();

// TanStack batches observer notifications on a real timer; a synchronous
// scheduler makes query-driven rerenders flush deterministically in the loop.
const { notifyManager } = await import("@tanstack/preact-query");
notifyManager.setScheduler((cb: () => void) => cb());

// Dynamic import is the module-loading boundary: registerDom() must install the
// DOM globals before these modules evaluate (matches every test in this dir).
const docs = await import("../src/docs");
const consoleMod = await import("../src/console");
const { queryClient, QueryRoot } = await import("../src/queries/client");
const { html, render } = await import("htm/preact");
const { notify } = await import("../src/store");
const { state, HELP_CACHE_KEY } = await import("../src/state");

const HELP: Record<string, string> = {
  SERVO_APPLY_GAINS:
    "Switch to manual tuning and write gains. Params AXIS=X|Y (X) POS_GAIN (400) SERVO_HELPER",
  SERVO_HELPER: "A helper. Params MODE=A|B (A)",
  G28: "home the printer",
};

function jsonResponse(body: unknown): Response {
  return new Response(JSON.stringify(body), { status: 200, headers: { "Content-Type": "application/json" } });
}

let gcodeHelp: Record<string, string> = {};
let gcodeHelpFails = false;
let gcodeHelpDefer: Promise<void> | null = null;

globalThis.fetch = (async (input: RequestInfo | URL) => {
  const url = input instanceof Request ? input.url : String(input);
  const path = url.startsWith("http") ? new URL(url).pathname : url.split("?")[0];
  if (path.endsWith("/printer/gcode/help")) {
    if (gcodeHelpDefer) await gcodeHelpDefer;
    return gcodeHelpFails ? new Response("down", { status: 503 }) : jsonResponse({ result: gcodeHelp });
  }
  if (path.endsWith("/server/info")) return jsonResponse({ result: { klippy_state: "ready" } });
  if (path.endsWith("/printer/gcode/script")) return new Response("ok", { status: 200 });
  if (path.endsWith("/server/gcode_store")) return jsonResponse({ result: { gcode_store: [] } });
  return new Response(`no route ${url}`, { status: 404 });
}) as typeof fetch;

function raf(): Promise<void> {
  const { promise, resolve } = Promise.withResolvers<void>();
  requestAnimationFrame(() => resolve());
  return promise;
}

function macrotask(): Promise<void> {
  const { promise, resolve } = Promise.withResolvers<void>();
  const channel = new MessageChannel();
  channel.port1.onmessage = () => resolve();
  channel.port2.postMessage(0);
  return promise;
}

async function settle() {
  for (let i = 0; i < 4; i++) {
    await raf();
    await macrotask();
  }
}

let docsHost: HTMLElement | null = null;

function renderDocs() {
  const host = document.getElementById("page-root")!;
  if (docsHost && docsHost !== host) render(null, docsHost);
  docsHost = host;
  render(html`<${QueryRoot}><${docs.DocsPage} /><//>`, host);
  notify();
}

async function mount() {
  renderDocs();
  await settle();
}

beforeEach(() => {
  localStorage.clear();
  queryClient.clear();
  gcodeHelp = { ...HELP };
  gcodeHelpFails = false;
  gcodeHelpDefer = null;
  location.hash = "";
  state.console.text = "";
  state.console.history = [];
  state.console.cursor = null;
  state.console.draft = "";
  state.console.search = null;
  document.body.innerHTML = `<input type="text" id="moonraker-url"><div id="page-root"></div>`;
});

test("splitMacroHelp separates prose from the Params tail", () => {
  expect(docs.splitMacroHelp("Do a thing. Params AXIS X")).toEqual({ prose: "Do a thing.", params: "AXIS X" });
  expect(docs.splitMacroHelp("just prose")).toEqual({ prose: "just prose", params: null });
});

test("parseParamsTail tokenizes params, choices, defaults, and prose runs", () => {
  expect(docs.parseParamsTail("AXIS=X|Y (X) as SERVO plus")).toEqual([
    { kind: "param", name: "AXIS", choices: "X|Y", dflt: "X" },
    { kind: "text", text: "as" },
    { kind: "param", name: "SERVO", choices: null, dflt: null },
    { kind: "text", text: "plus" },
  ]);
});

test("firstSentence stops at the first sentence break", () => {
  expect(docs.firstSentence("One. Two. Three")).toBe("One.");
  expect(docs.firstSentence("no period here")).toBe("no period here");
});

test("docsDeepLinkTarget reads and upper-cases the hash macro", () => {
  location.hash = "#/docs/servo_apply_gains";
  expect(docs.docsDeepLinkTarget()).toBe("SERVO_APPLY_GAINS");
  location.hash = "#/tune";
  expect(docs.docsDeepLinkTarget()).toBe(null);
});

test("longestCommonPrefix trims to the shared head", () => {
  expect(docs.longestCommonPrefix(["SERVO_ALL", "SERVO_APPLY", "SERVO_A"])).toBe("SERVO_A");
});

test("DocsPage renders the docs, console, and launchpad sections with their ids", async () => {
  await mount();
  expect(document.querySelector(".workspace.single main.analysis")).toBeTruthy();
  expect(document.querySelector("section.docs-section")).toBeTruthy();
  expect(document.getElementById("docs-list")).toBeTruthy();
  expect(document.querySelector("section.session")).toBeTruthy();
  expect(document.getElementById("console-input")).toBeTruthy();
  expect(document.querySelector("section.launchpad-panel")).toBeTruthy();
  expect(document.getElementById("launchpad-body")).toBeTruthy();
});

test("the macro list renders one details per SERVO_ command with chips and xrefs", async () => {
  await mount();
  const list = document.getElementById("docs-list")!;
  expect(list.querySelectorAll("details.macro-doc").length).toBe(2);
  expect([...list.querySelectorAll(".macro-name")].map((e) => e.textContent)).toEqual(
    expect.arrayContaining(["SERVO_APPLY_GAINS", "SERVO_HELPER"])
  );
  expect(document.getElementById("docs-status")!.textContent).toContain(
    "the running klippy's cmd_*_help strings, fetched"
  );
  const apply = document.getElementById("doc-SERVO_APPLY_GAINS")!;
  const xref = apply.querySelector("a.chip.param-chip.xref");
  expect(xref?.getAttribute("href")).toBe("#/docs/SERVO_HELPER");
  expect([...apply.querySelectorAll("span.chip.param-chip")].map((c) => c.textContent)).toEqual(
    expect.arrayContaining(["AXIS=X|Y (X)", "POS_GAIN (400)"])
  );
});

test("a #/docs/<macro> deep link opens exactly that macro's details", async () => {
  location.hash = "#/docs/servo_helper";
  await mount();
  expect((document.getElementById("doc-SERVO_HELPER") as HTMLDetailsElement).open).toBe(true);
  expect((document.getElementById("doc-SERVO_APPLY_GAINS") as HTMLDetailsElement).open).toBe(false);
});
test("changing docs deep links opens the new target without remounting", async () => {
  location.hash = "#/docs/servo_helper";
  await mount();
  const apply = document.getElementById("doc-SERVO_APPLY_GAINS") as HTMLDetailsElement;
  expect(apply.open).toBe(false);
  location.hash = "#/docs/servo_apply_gains";
  await act(async () => {
    window.dispatchEvent(new HashChangeEvent("hashchange"));
    await settle();
  });
  expect(apply.open).toBe(true);
});


test("a manual details toggle survives a store-driven rerender", async () => {
  await mount();
  const details = document.getElementById("doc-SERVO_HELPER") as HTMLDetailsElement;
  expect(details.open).toBe(false);
  details.open = true;
  notify();
  await settle();
  expect((document.getElementById("doc-SERVO_HELPER") as HTMLDetailsElement).open).toBe(true);
});

test("a cached copy with klippy unreachable shows a retry control and still lists macros", async () => {
  localStorage.setItem(
    HELP_CACHE_KEY,
    JSON.stringify({ fetched_utc: "2026-07-19T10:20:30Z", commands: { SERVO_HELPER: HELP.SERVO_HELPER } })
  );
  gcodeHelpFails = true;
  await mount();
  const status = document.getElementById("docs-status")!;
  expect(status.textContent).toContain("cached copy");
  expect(status.textContent).toContain("klippy unreachable");
  expect(document.getElementById("docs-retry")).toBeTruthy();
  expect(document.querySelectorAll("details.macro-doc").length).toBeGreaterThan(0);
});

test("while the first fetch is in flight the status reads as fetching", async () => {
  const gate = Promise.withResolvers<void>();
  gcodeHelpDefer = gate.promise;
  renderDocs();
  await raf();
  await macrotask();
  expect(document.getElementById("docs-status")!.textContent).toContain("fetching from klippy");
  gate.resolve();
  await settle();
  expect(document.getElementById("docs-status")!.textContent).toContain("fetched");
});

test("with no cache and klippy down the status offers retry and the list is empty", async () => {
  gcodeHelpFails = true;
  await mount();
  expect(document.getElementById("docs-retry")).toBeTruthy();
  expect(document.getElementById("docs-list")!.querySelector("p.note")?.textContent).toContain("no macro help yet");
});

test("selecting a launchpad macro shows its form, preview, and required guard", async () => {
  await mount();
  document.querySelector<HTMLButtonElement>('.lp-item[data-lp-macro="SERVO_SET_INERTIA_RATIO"]')!.click();
  await settle();
  expect(document.getElementById("launchpad-back")).toBeTruthy();
  expect(document.getElementById("launchpad-preview")!.textContent).toBe("SERVO_SET_INERTIA_RATIO");
  expect((document.getElementById("launchpad-run") as HTMLButtonElement).disabled).toBe(true);
  expect(document.getElementById("launchpad-missing")!.textContent).toContain("RATIO");

  const ratio = document.querySelector<HTMLInputElement>('[data-lp-param="RATIO"]')!;
  ratio.value = "120";
  ratio.dispatchEvent(new Event("input", { bubbles: true }));
  await settle();
  expect(document.getElementById("launchpad-preview")!.textContent).toBe("SERVO_SET_INERTIA_RATIO RATIO=120");
  expect((document.getElementById("launchpad-run") as HTMLButtonElement).disabled).toBe(false);
});

test("the launchpad copy button drops the built command into the console", async () => {
  await mount();
  document.querySelector<HTMLButtonElement>('.lp-item[data-lp-macro="SERVO_APPLY_GAINS"]')!.click();
  await settle();
  const posGain = document.querySelector<HTMLInputElement>('[data-lp-param="POS_GAIN"]')!;
  posGain.value = "400";
  posGain.dispatchEvent(new Event("input", { bubbles: true }));
  await settle();
  document.getElementById("launchpad-copy")!.click();
  await settle();
  expect(state.console.text).toBe("SERVO_APPLY_GAINS POS_GAIN=400");
  expect((document.getElementById("console-input") as HTMLTextAreaElement).value).toBe("SERVO_APPLY_GAINS POS_GAIN=400");
});

test("the back button returns to the macro list", async () => {
  await mount();
  document.querySelector<HTMLButtonElement>(".lp-item")!.click();
  await settle();
  document.getElementById("launchpad-back")!.click();
  await settle();
  expect(document.getElementById("launchpad-back")).toBeNull();
  expect(document.querySelectorAll(".lp-item").length).toBeGreaterThan(0);
});

test("a selected launchpad macro and its values persist across a remount", async () => {
  await mount();
  document.querySelector<HTMLButtonElement>('.lp-item[data-lp-macro="SERVO_SET_INERTIA_RATIO"]')!.click();
  await settle();
  const ratio = document.querySelector<HTMLInputElement>('[data-lp-param="RATIO"]')!;
  ratio.value = "120";
  ratio.dispatchEvent(new Event("input", { bubbles: true }));
  await settle();

  document.body.innerHTML = `<input type="text" id="moonraker-url"><div id="page-root"></div>`;
  await mount();
  expect(document.querySelector<HTMLInputElement>('[data-lp-param="RATIO"]')!.value).toBe("120");
});

test("tab completion completes a SERVO command name from the macro help", async () => {
  await mount();
  const input = document.getElementById("console-input") as HTMLTextAreaElement;
  input.value = "SERVO_APP";
  input.selectionStart = input.selectionEnd = input.value.length;
  docs.consoleTabComplete(input);
  expect(input.value).toBe("SERVO_APPLY_GAINS ");
});

test("tab completion offers a command's remaining params", async () => {
  await mount();
  const input = document.getElementById("console-input") as HTMLTextAreaElement;
  input.value = "SERVO_HELPER MO";
  input.selectionStart = input.selectionEnd = input.value.length;
  docs.consoleTabComplete(input);
  expect(input.value).toBe("SERVO_HELPER MODE=");
});

test("history stepping recalls prior submitted commands newest-first", async () => {
  await mount();
  state.console.history = ["G28", "SERVO_HELPER"];
  consoleMod.historyStep(-1);
  expect((document.getElementById("console-input") as HTMLTextAreaElement).value).toBe("SERVO_HELPER");
  consoleMod.historyStep(-1);
  expect((document.getElementById("console-input") as HTMLTextAreaElement).value).toBe("G28");
});

test("unmounting the docs page detaches the console input listeners", async () => {
  await mount();
  const stale = document.getElementById("console-input") as HTMLTextAreaElement;
  document.body.innerHTML = `<input type="text" id="moonraker-url"><div id="page-root"></div>`;
  await mount();
  state.console.text = "sentinel";
  stale.value = "changed after unmount";
  stale.dispatchEvent(new Event("input", { bubbles: true }));
  expect(state.console.text).toBe("sentinel");
});
