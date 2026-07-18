import { expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { prepareTrace, pickLevel, drawTrace, nearestTrace } = await import("../src/path-view");
import type { PathTrace, Viewport } from "../src/path-view";

function countingCtx() {
  const counts = { moveTo: 0, lineTo: 0 };
  const noop = () => undefined;
  return {
    counts,
    ctx: new Proxy(
      {
        moveTo: () => counts.moveTo++,
        lineTo: () => counts.lineTo++,
      } as Record<string | symbol, unknown>,
      {
        get(t, prop) {
          if (!(prop in t)) t[prop] = noop;
          return t[prop];
        },
        set(t, prop, value) {
          t[prop] = value;
          return true;
        },
      }
    ) as unknown as CanvasRenderingContext2D,
  };
}

function spiralTrace(n: number): PathTrace {
  const xs: (number | null)[] = new Array(n);
  const ys: (number | null)[] = new Array(n);
  for (let i = 0; i < n; i++) {
    const a = (i / n) * 200 * Math.PI;
    const r = 1 + 99 * (i / n);
    xs[i] = r * Math.cos(a);
    ys[i] = r * Math.sin(a);
  }
  return { xs, ys, color: "#4fb3ff", width: 1 };
}

test("prepareTrace builds a geometric LOD pyramid down to the floor", () => {
  const p = prepareTrace(spiralTrace(500_000));
  expect(p.levels[0].xs.length).toBe(500_000);
  expect(p.levels[p.levels.length - 1].xs.length).toBeLessThanOrEqual(512);
  for (let i = 1; i < p.levels.length; i++) {
    expect(p.levels[i].xs.length).toBe(Math.ceil(p.levels[i - 1].xs.length / 4));
  }
  expect(p.bounds!.xMax).toBeCloseTo(100, 0);
});

test("prepareTrace caches by array identity", () => {
  const trace = spiralTrace(1000);
  expect(prepareTrace(trace)).toBe(prepareTrace({ ...trace }));
});

test("prepareTrace preserves null breaks as NaN in the base level", () => {
  const p = prepareTrace({ xs: [0, null, 2], ys: [0, null, 2], color: "#fff", width: 1 });
  expect(Number.isNaN(p.levels[0].xs[1])).toBe(true);
  expect(p.levels[0].xs[2]).toBe(2);
});

test("pickLevel selects coarse levels zoomed out and full detail zoomed in", () => {
  const p = prepareTrace(spiralTrace(500_000));
  const zoomedOut: Viewport = { cx: 0, cy: 0, mmPerPx: 0.25 };
  const zoomedIn: Viewport = { cx: 50, cy: 0, mmPerPx: 0.00005 };
  expect(pickLevel(p, zoomedOut).xs.length).toBeLessThan(10_000);
  expect(pickLevel(p, zoomedIn).xs.length).toBe(500_000);
});

test("drawTrace culls offscreen segments without breaking crossing segments", () => {
  const view: Viewport = { cx: 0, cy: 0, mmPerPx: 1 };
  const trace: PathTrace = {
    xs: [-5000, -4000, -3000, 5000, 4000],
    ys: [0, 0, 0, 0, 0],
    color: "#fff",
    width: 1,
  };
  const { ctx, counts } = countingCtx();
  drawTrace(ctx, view, 800, 600, trace);
  expect(counts.moveTo).toBe(1);
  expect(counts.lineTo).toBe(1);
});

test("drawTrace at full detail zoomed in touches only visible work", () => {
  const trace = spiralTrace(500_000);
  const view: Viewport = { cx: 50, cy: 0, mmPerPx: 0.0005 };
  const { ctx, counts } = countingCtx();
  drawTrace(ctx, view, 800, 600, trace);
  expect(counts.lineTo).toBeLessThan(5000);
});

test("nearestTrace finds the closest trace within reach and respects the cutoff", () => {
  const view: Viewport = { cx: 0, cy: 0, mmPerPx: 1 };
  const a: PathTrace = { xs: [-100, 100], ys: [10, 10], color: "#fff", width: 1 };
  const b: PathTrace = { xs: [-100, 100], ys: [-40, -40], color: "#fff", width: 1 };
  const hit = nearestTrace([a, b], view, 800, 600, 400, 296, 8);
  expect(hit).not.toBeNull();
  expect(hit!.traceIndex).toBe(0);
  expect(hit!.distPx).toBeCloseTo(6, 3);
  expect(nearestTrace([a, b], view, 800, 600, 400, 200, 8)).toBeNull();
});

function legacyDraw(ctx: CanvasRenderingContext2D, view: Viewport, w: number, h: number, trace: PathTrace) {
  ctx.beginPath();
  let penDown = false;
  let lastPx = NaN;
  let lastPy = NaN;
  for (let i = 0; i < trace.xs.length; i++) {
    const x = trace.xs[i];
    const y = trace.ys[i];
    if (x === null || y === null) {
      penDown = false;
      continue;
    }
    const px = (x - view.cx) / view.mmPerPx + w / 2;
    const py = h / 2 - (y - view.cy) / view.mmPerPx;
    if (penDown && Math.abs(px - lastPx) < 0.5 && Math.abs(py - lastPy) < 0.5) continue;
    if (penDown) ctx.lineTo(px, py);
    else ctx.moveTo(px, py);
    penDown = true;
    lastPx = px;
    lastPy = py;
  }
  ctx.stroke();
}

test("bench: 500k-point draw is far cheaper than the legacy full walk", () => {
  const trace = spiralTrace(500_000);
  prepareTrace(trace);
  const view: Viewport = { cx: 0, cy: 0, mmPerPx: 0.25 };
  const zoomedIn: Viewport = { cx: 50, cy: 0, mmPerPx: 0.001 };
  const frames = 20;

  const { ctx: legacyCtx, counts: legacyCounts } = countingCtx();
  const t0 = performance.now();
  for (let f = 0; f < frames; f++) legacyDraw(legacyCtx, view, 800, 600, trace);
  const legacyMs = (performance.now() - t0) / frames;

  const { ctx, counts } = countingCtx();
  const t1 = performance.now();
  for (let f = 0; f < frames; f++) drawTrace(ctx, view, 800, 600, trace);
  const fitMs = (performance.now() - t1) / frames;

  const t2 = performance.now();
  for (let f = 0; f < frames; f++) drawTrace(ctx, zoomedIn, 800, 600, trace);
  const zoomMs = (performance.now() - t2) / frames;

  console.log(
    `path draw bench (500k pts, per frame): legacy ${legacyMs.toFixed(2)}ms / ` +
      `${(legacyCounts.lineTo / frames).toFixed(0)} lineTo — lod-fit ${fitMs.toFixed(2)}ms / ` +
      `${(counts.lineTo / frames).toFixed(0)} lineTo — full-detail zoom ${zoomMs.toFixed(2)}ms`
  );
  expect(fitMs).toBeLessThan(legacyMs);
  expect(counts.lineTo / frames).toBeLessThan(legacyCounts.lineTo / frames / 10);
});
