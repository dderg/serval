import { hidpiCanvasContext } from "./charts-core";

// --- shared 2D path canvas ---------------------------------------------------
//
// Data-space (mm) viewport with the gesture set every path canvas shares:
// ctrl/meta+wheel zooms about the cursor, a plain wheel is a two-finger pan,
// drag pans, double-click or a fit button restores auto-fit. `createPathView`
// keeps one viewport per canvas owner (live spatial view, run path chart);
// callers hand it traces and layer their own markers/legend on the returned
// context.

interface Viewport {
  cx: number;
  cy: number;
  mmPerPx: number;
}

interface PathTrace {
  xs: (number | null)[];
  ys: (number | null)[];
  color: string;
  width: number;
  dash?: number[];
}

const BG_COLOR = "#0d1117";
const GRID_COLOR = "#29313a";
const LABEL_COLOR = "#8a97a3";
const FIT_MARGIN_FRAC = 0.08;
const MIN_SPAN_MM = 0.02;
const MM_PER_PX_LIMITS = [1e-5, 10] as const;
const TICK_TARGET_PX = 90;
const WHEEL_ZOOM_FACTOR_LIMITS = [0.1, 10] as const;
const WHEEL_MOUSEMOVE_SUPPRESS_MS = 80;
const LOD_STRIDE = 4;
const LOD_MIN_POINTS = 512;
const LOD_TARGET_POINTS_PER_PX = 3;
const CULL_MARGIN_PX = 8;

interface LodLevel {
  xs: Float64Array;
  ys: Float64Array;
}

interface TraceBounds {
  xMin: number;
  xMax: number;
  yMin: number;
  yMax: number;
}

interface PreparedTrace {
  levels: LodLevel[];
  bounds: TraceBounds | null;
}

function baseLevel(xs: (number | null)[], ys: (number | null)[]): LodLevel {
  const n = xs.length;
  const fx = new Float64Array(n);
  const fy = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const x = xs[i];
    const y = ys[i];
    if (x === null || y === null) {
      fx[i] = NaN;
      fy[i] = NaN;
    } else {
      fx[i] = x;
      fy[i] = y;
    }
  }
  return { xs: fx, ys: fy };
}

function decimateLevel(level: LodLevel): LodLevel {
  const n = level.xs.length;
  const out = Math.ceil(n / LOD_STRIDE);
  const fx = new Float64Array(out);
  const fy = new Float64Array(out);
  for (let b = 0; b < out; b++) {
    const start = b * LOD_STRIDE;
    const end = Math.min(start + LOD_STRIDE, n);
    let px = NaN;
    let py = NaN;
    for (let i = start; i < end; i++) {
      if (!Number.isNaN(level.xs[i])) {
        px = level.xs[i];
        py = level.ys[i];
        break;
      }
    }
    fx[b] = px;
    fy[b] = py;
  }
  return { xs: fx, ys: fy };
}

function levelBounds(level: LodLevel): TraceBounds | null {
  let xMin = Infinity;
  let xMax = -Infinity;
  let yMin = Infinity;
  let yMax = -Infinity;
  for (let i = 0; i < level.xs.length; i++) {
    const x = level.xs[i];
    if (Number.isNaN(x)) continue;
    const y = level.ys[i];
    if (x < xMin) xMin = x;
    if (x > xMax) xMax = x;
    if (y < yMin) yMin = y;
    if (y > yMax) yMax = y;
  }
  if (!isFinite(xMin) || !isFinite(yMin)) return null;
  return { xMin, xMax, yMin, yMax };
}

const preparedCache = new WeakMap<object, PreparedTrace>();

function prepareTrace(trace: PathTrace): PreparedTrace {
  const cached = preparedCache.get(trace.xs);
  if (cached) return cached;
  const base = baseLevel(trace.xs, trace.ys);
  const levels = [base];
  while (levels[levels.length - 1].xs.length > LOD_MIN_POINTS) {
    levels.push(decimateLevel(levels[levels.length - 1]));
  }
  const prepared = { levels, bounds: levelBounds(base) };
  preparedCache.set(trace.xs, prepared);
  return prepared;
}

function pickLevel(prepared: PreparedTrace, view: Viewport): LodLevel {
  const b = prepared.bounds;
  if (!b) return prepared.levels[0];
  const extentPx = Math.max(b.xMax - b.xMin, b.yMax - b.yMin) / view.mmPerPx;
  const budget = extentPx * LOD_TARGET_POINTS_PER_PX + LOD_MIN_POINTS;
  for (const level of prepared.levels) {
    if (level.xs.length <= budget) return level;
  }
  return prepared.levels[prepared.levels.length - 1];
}

function fitViewport(paths: (number | null)[][], w: number, h: number): Viewport | null {
  let xMin = Infinity;
  let xMax = -Infinity;
  let yMin = Infinity;
  let yMax = -Infinity;
  for (let p = 0; p + 1 < paths.length; p += 2) {
    const xs = paths[p];
    const ys = paths[p + 1];
    for (let i = 0; i < xs.length; i++) {
      const x = xs[i];
      const y = ys[i];
      if (x === null || y === null) continue;
      if (x < xMin) xMin = x;
      if (x > xMax) xMax = x;
      if (y < yMin) yMin = y;
      if (y > yMax) yMax = y;
    }
  }
  if (!isFinite(xMin) || !isFinite(yMin)) return null;
  return fitBounds(xMin, xMax, yMin, yMax, w, h);
}

function fitBounds(
  xMin: number,
  xMax: number,
  yMin: number,
  yMax: number,
  w: number,
  h: number
): Viewport {
  const spanX = Math.max(xMax - xMin, MIN_SPAN_MM);
  const spanY = Math.max(yMax - yMin, MIN_SPAN_MM);
  const mmPerPx = Math.max(
    spanX / (w * (1 - 2 * FIT_MARGIN_FRAC)),
    spanY / (h * (1 - 2 * FIT_MARGIN_FRAC))
  );
  return { cx: (xMin + xMax) / 2, cy: (yMin + yMax) / 2, mmPerPx };
}

function tickStepMm(mmPerPx: number): number {
  const raw = mmPerPx * TICK_TARGET_PX;
  const pow = Math.pow(10, Math.floor(Math.log10(raw)));
  for (const m of [1, 2, 5]) {
    if (m * pow >= raw) return m * pow;
  }
  return 10 * pow;
}

function drawGrid(ctx: CanvasRenderingContext2D, view: Viewport, w: number, h: number) {
  const step = tickStepMm(view.mmPerPx);
  const decimals = Math.max(0, -Math.floor(Math.log10(step)));
  const xLo = view.cx - (w / 2) * view.mmPerPx;
  const xHi = view.cx + (w / 2) * view.mmPerPx;
  const yLo = view.cy - (h / 2) * view.mmPerPx;
  const yHi = view.cy + (h / 2) * view.mmPerPx;
  ctx.strokeStyle = GRID_COLOR;
  ctx.fillStyle = LABEL_COLOR;
  ctx.font = "10px monospace";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let gx = Math.ceil(xLo / step) * step; gx <= xHi; gx += step) {
    const px = (gx - view.cx) / view.mmPerPx + w / 2;
    ctx.moveTo(px, 0);
    ctx.lineTo(px, h);
    ctx.fillText(gx.toFixed(decimals), px + 3, h - 4);
  }
  for (let gy = Math.ceil(yLo / step) * step; gy <= yHi; gy += step) {
    const py = h / 2 - (gy - view.cy) / view.mmPerPx;
    ctx.moveTo(0, py);
    ctx.lineTo(w, py);
    ctx.fillText(gy.toFixed(decimals), 3, py - 3);
  }
  ctx.stroke();
  ctx.fillText(`grid ${step >= 1 ? step.toFixed(0) : step} mm`, w - 90, h - 4);
}

function drawTrace(
  ctx: CanvasRenderingContext2D,
  view: Viewport,
  w: number,
  h: number,
  trace: PathTrace
) {
  const prepared = prepareTrace(trace);
  if (!prepared.bounds) return;
  const level = pickLevel(prepared, view);
  const { xs, ys } = level;
  const margin = CULL_MARGIN_PX * view.mmPerPx;
  const xLo = view.cx - (w / 2) * view.mmPerPx - margin;
  const xHi = view.cx + (w / 2) * view.mmPerPx + margin;
  const yLo = view.cy - (h / 2) * view.mmPerPx - margin;
  const yHi = view.cy + (h / 2) * view.mmPerPx + margin;
  ctx.strokeStyle = trace.color;
  ctx.lineWidth = trace.width;
  ctx.setLineDash(trace.dash ?? []);
  ctx.beginPath();
  let penDown = false;
  let prevX = NaN;
  let prevY = NaN;
  let lastPx = NaN;
  let lastPy = NaN;
  for (let i = 0; i < xs.length; i++) {
    const x = xs[i];
    const y = ys[i];
    if (Number.isNaN(x) || Number.isNaN(y)) {
      penDown = false;
      prevX = NaN;
      continue;
    }
    if (Number.isNaN(prevX)) {
      prevX = x;
      prevY = y;
      penDown = false;
      continue;
    }
    const segVisible =
      Math.max(prevX, x) >= xLo &&
      Math.min(prevX, x) <= xHi &&
      Math.max(prevY, y) >= yLo &&
      Math.min(prevY, y) <= yHi;
    if (!segVisible) {
      penDown = false;
      prevX = x;
      prevY = y;
      continue;
    }
    const px = (x - view.cx) / view.mmPerPx + w / 2;
    const py = h / 2 - (y - view.cy) / view.mmPerPx;
    if (!penDown) {
      const ppx = (prevX - view.cx) / view.mmPerPx + w / 2;
      const ppy = h / 2 - (prevY - view.cy) / view.mmPerPx;
      ctx.moveTo(ppx, ppy);
      penDown = true;
      lastPx = ppx;
      lastPy = ppy;
    }
    if (Math.abs(px - lastPx) >= 0.5 || Math.abs(py - lastPy) >= 0.5) {
      ctx.lineTo(px, py);
      lastPx = px;
      lastPy = py;
    }
    prevX = x;
    prevY = y;
  }
  ctx.stroke();
  ctx.setLineDash([]);
}

interface TraceHit {
  traceIndex: number;
  pointIndex: number;
  distPx: number;
  mmX: number;
  mmY: number;
}

function segmentClosest(px: number, py: number, ax: number, ay: number, bx: number, by: number) {
  const dx = bx - ax;
  const dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  const t = lenSq === 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / lenSq));
  const cx = ax + t * dx;
  const cy = ay + t * dy;
  return { distSq: (px - cx) * (px - cx) + (py - cy) * (py - cy), t };
}

function nearestTrace(
  traces: PathTrace[],
  view: Viewport,
  w: number,
  h: number,
  px: number,
  py: number,
  maxDistPx: number
): TraceHit | null {
  const cursorX = view.cx + (px - w / 2) * view.mmPerPx;
  const cursorY = view.cy - (py - h / 2) * view.mmPerPx;
  const reach = maxDistPx * view.mmPerPx;
  let best: TraceHit | null = null;
  let bestDistSq = maxDistPx * maxDistPx;
  traces.forEach((trace, traceIndex) => {
    const prepared = prepareTrace(trace);
    if (!prepared.bounds) return;
    const level = pickLevel(prepared, view);
    const { xs, ys } = level;
    const stride = LOD_STRIDE ** prepared.levels.indexOf(level);
    let prevX = NaN;
    let prevY = NaN;
    let prevI = -1;
    for (let i = 0; i < xs.length; i++) {
      const x = xs[i];
      const y = ys[i];
      if (Number.isNaN(x) || Number.isNaN(y)) {
        prevX = NaN;
        continue;
      }
      if (!Number.isNaN(prevX)) {
        const near =
          Math.max(prevX, x) >= cursorX - reach &&
          Math.min(prevX, x) <= cursorX + reach &&
          Math.max(prevY, y) >= cursorY - reach &&
          Math.min(prevY, y) <= cursorY + reach;
        if (near) {
          const ax = (prevX - view.cx) / view.mmPerPx + w / 2;
          const ay = h / 2 - (prevY - view.cy) / view.mmPerPx;
          const bx = (x - view.cx) / view.mmPerPx + w / 2;
          const by = h / 2 - (y - view.cy) / view.mmPerPx;
          const { distSq, t } = segmentClosest(px, py, ax, ay, bx, by);
          if (distSq < bestDistSq) {
            bestDistSq = distSq;
            const nearEnd = t < 0.5;
            best = {
              traceIndex,
              pointIndex: (nearEnd ? prevI : i) * stride,
              distPx: Math.sqrt(distSq),
              mmX: nearEnd ? prevX : x,
              mmY: nearEnd ? prevY : y,
            };
          }
        }
      }
      prevX = x;
      prevY = y;
      prevI = i;
    }
  });
  return best;
}

function blankCanvas(canvas: HTMLCanvasElement) {
  const { ctx, w, h } = hidpiCanvasContext(canvas);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = BG_COLOR;
  ctx.fillRect(0, 0, w, h);
}

interface RenderedView {
  ctx: CanvasRenderingContext2D;
  view: Viewport;
  w: number;
  h: number;
}

interface PathView {
  bind(canvas: HTMLCanvasElement, fitButton: HTMLElement | null, onRedraw: () => void): void;
  render(canvas: HTMLCanvasElement, traces: PathTrace[]): RenderedView | null;
  isManual(): boolean;
  gestureActive(): boolean;
  lastRendered(): RenderedView | null;
}

function createPathView(): PathView {
  let manualView: Viewport | null = null;
  let autoView: Viewport | null = null;
  let drag: { px: number; py: number } | null = null;
  let boundCanvas: HTMLCanvasElement | null = null;
  let boundFitButton: HTMLElement | null = null;
  let redraw: () => void = () => {};
  let redrawQueued = false;
  let wheelSuppressUntil = 0;
  let rendered: RenderedView | null = null;

  function activeView(): Viewport | null {
    return manualView ?? autoView;
  }

  function scheduleRedraw() {
    if (redrawQueued) return;
    redrawQueued = true;
    requestAnimationFrame(() => {
      redrawQueued = false;
      redraw();
    });
  }

  function canvasMm(canvas: HTMLCanvasElement, view: Viewport, e: MouseEvent): [number, number] {
    const rect = canvas.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    return [
      view.cx + (px - rect.width / 2) * view.mmPerPx,
      view.cy - (py - rect.height / 2) * view.mmPerPx,
    ];
  }

  function bindCanvas(canvas: HTMLCanvasElement) {
    canvas.addEventListener(
      "wheel",
      (e) => {
        const view = activeView();
        if (!view) return;
        e.preventDefault();
        wheelSuppressUntil = performance.now() + WHEEL_MOUSEMOVE_SUPPRESS_MS;
        if (e.ctrlKey || e.metaKey) {
          const [mmX, mmY] = canvasMm(canvas, view, e);
          const factor = Math.min(
            WHEEL_ZOOM_FACTOR_LIMITS[1],
            Math.max(WHEEL_ZOOM_FACTOR_LIMITS[0], Math.exp(e.deltaY * 0.01))
          );
          const mmPerPx = Math.min(
            MM_PER_PX_LIMITS[1],
            Math.max(MM_PER_PX_LIMITS[0], view.mmPerPx * factor)
          );
          const scale = mmPerPx / view.mmPerPx;
          manualView = {
            mmPerPx,
            cx: mmX + (view.cx - mmX) * scale,
            cy: mmY + (view.cy - mmY) * scale,
          };
        } else {
          manualView = {
            mmPerPx: view.mmPerPx,
            cx: view.cx + e.deltaX * view.mmPerPx,
            cy: view.cy - e.deltaY * view.mmPerPx,
          };
        }
        scheduleRedraw();
      },
      { passive: false }
    );
    canvas.addEventListener("pointerdown", (e) => {
      wheelSuppressUntil = 0;
      drag = { px: e.clientX, py: e.clientY };
      canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener("pointermove", (e) => {
      const view = activeView();
      if (!drag || !view) return;
      if (performance.now() < wheelSuppressUntil) return;
      manualView = {
        mmPerPx: view.mmPerPx,
        cx: view.cx - (e.clientX - drag.px) * view.mmPerPx,
        cy: view.cy + (e.clientY - drag.py) * view.mmPerPx,
      };
      drag = { px: e.clientX, py: e.clientY };
      scheduleRedraw();
    });
    canvas.addEventListener("pointerup", () => {
      drag = null;
    });
    canvas.addEventListener("dblclick", () => {
      manualView = null;
      scheduleRedraw();
    });
  }

  function bind(canvas: HTMLCanvasElement, fitButton: HTMLElement | null, onRedraw: () => void) {
    redraw = onRedraw;
    if (boundCanvas !== canvas) {
      boundCanvas = canvas;
      bindCanvas(canvas);
    }
    if (fitButton && boundFitButton !== fitButton) {
      boundFitButton = fitButton;
      fitButton.addEventListener("click", () => {
        manualView = null;
        scheduleRedraw();
      });
    }
  }

  function render(canvas: HTMLCanvasElement, traces: PathTrace[]): RenderedView | null {
    const { ctx, w, h } = hidpiCanvasContext(canvas);
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = BG_COLOR;
    ctx.fillRect(0, 0, w, h);
    if (manualView === null) {
      let xMin = Infinity;
      let xMax = -Infinity;
      let yMin = Infinity;
      let yMax = -Infinity;
      for (const t of traces) {
        const b = prepareTrace(t).bounds;
        if (!b) continue;
        if (b.xMin < xMin) xMin = b.xMin;
        if (b.xMax > xMax) xMax = b.xMax;
        if (b.yMin < yMin) yMin = b.yMin;
        if (b.yMax > yMax) yMax = b.yMax;
      }
      autoView = isFinite(xMin) && isFinite(yMin) ? fitBounds(xMin, xMax, yMin, yMax, w, h) : null;
    }
    const view = activeView();
    if (!view) {
      rendered = null;
      return null;
    }
    drawGrid(ctx, view, w, h);
    for (const t of traces) drawTrace(ctx, view, w, h, t);
    rendered = { ctx, view, w, h };
    return rendered;
  }

  return {
    bind,
    render,
    isManual: () => manualView !== null,
    gestureActive: () => drag !== null || performance.now() < wheelSuppressUntil,
    lastRendered: () => rendered,
  };
}

export {
  fitViewport,
  tickStepMm,
  blankCanvas,
  createPathView,
  prepareTrace,
  pickLevel,
  drawTrace,
  nearestTrace,
};
export type { Viewport, PathTrace, RenderedView, PathView, TraceHit, PreparedTrace, LodLevel };
