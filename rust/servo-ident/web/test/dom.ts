import { GlobalRegistrator } from "@happy-dom/global-registrator";
import { readFileSync } from "node:fs";
import { join } from "node:path";

// happy-dom's canvas has no 2d context, but uPlot draws unconditionally.
// This mock answers every ctx method with a no-op (measureText and
// gradients need real-shaped return values) so chart code runs to
// completion; pixel output is out of scope for these tests.
function mock2dContext(): CanvasRenderingContext2D {
  const noop = () => undefined;
  const target: Record<string | symbol, unknown> = {
    measureText: () => ({ width: 0 }),
    createLinearGradient: () => ({ addColorStop: noop }),
    getImageData: () => ({ data: new Uint8ClampedArray(4) }),
  };
  return new Proxy(target, {
    get(t, prop) {
      if (!(prop in t)) t[prop] = noop;
      return t[prop];
    },
    set(t, prop, value) {
      t[prop] = value;
      return true;
    },
  }) as unknown as CanvasRenderingContext2D;
}

function registerDom() {
  if (GlobalRegistrator.isRegistered) return;
  GlobalRegistrator.register({ url: "http://127.0.0.1/" });
  (globalThis as Record<string, any>).Path2D ??= class Path2D {
    addPath() {}
    moveTo() {}
    lineTo() {}
    rect() {}
    closePath() {}
    arc() {}
  };
  const proto = (globalThis as Record<string, any>).HTMLCanvasElement.prototype;
  const contexts = new WeakMap<object, CanvasRenderingContext2D>();
  proto.getContext = function () {
    if (!contexts.has(this)) contexts.set(this, mock2dContext());
    return contexts.get(this);
  };
}

function fixture(name: string): string {
  return readFileSync(join(import.meta.dir, "fixtures", `${name}.json`), "utf-8");
}

function fixtureJson<T>(name: string): T {
  return JSON.parse(fixture(name)) as T;
}

const RUN_NAME = (fixtureJson<{ name: string }[]>("runs"))[0].name;

/// fetch stub covering everything boot touches: the servo-cal API answers
/// from the captured demo fixtures, moonraker answers minimally healthy.
/// Anything else is a test bug and fails loudly.
function installFetchStub(): { unmatched: string[] } {
  const unmatched: string[] = [];
  const json = (body: string) =>
    new Response(body, { status: 200, headers: { "Content-Type": "application/json" } });
  const routes: [RegExp, () => Response][] = [
    [/^\/api\/runs$/, () => json(fixture("runs"))],
    [/^\/api\/drive_state$/, () => json(fixture("drive_state"))],
    [/^\/api\/live$/, () => json(fixture("live"))],
    [/^\/api\/live_tap$/, () => json(JSON.stringify({ status: "connecting" }))],
    [new RegExp(`^/api/runs/${RUN_NAME}/manifest$`), () => json(fixture("manifest"))],
    [new RegExp(`^/api/runs/${RUN_NAME}/results$`), () => json(fixture("results"))],
    [new RegExp(`^/api/runs/${RUN_NAME}/plot_series$`), () => json(fixture("plot_series"))],
    [/\/server\/info$/, () => json(JSON.stringify({ result: { klippy_state: "ready" } }))],
    [/\/printer\/gcode\/help$/, () => json(JSON.stringify({ result: {} }))],
  ];
  globalThis.fetch = (async (input: RequestInfo | URL) => {
    const url = String(input);
    const path = url.startsWith("http") ? new URL(url).pathname : url.split("?")[0];
    for (const [re, respond] of routes) {
      if (re.test(path)) return respond();
    }
    unmatched.push(url);
    return new Response(`no fixture for ${url}`, { status: 404 });
  }) as typeof fetch;
  return { unmatched };
}

function indexHtmlBody(): string {
  const html = readFileSync(join(import.meta.dir, "..", "index.html"), "utf-8");
  const body = /<body>([\s\S]*)<\/body>/.exec(html);
  if (!body) throw new Error("index.html has no <body>");
  return body[1].replace(/<script[\s\S]*?<\/script>/g, "");
}

export { registerDom, installFetchStub, indexHtmlBody, fixtureJson, RUN_NAME };
