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
  ctx.strokeStyle = trace.color;
  ctx.lineWidth = trace.width;
  ctx.setLineDash(trace.dash ?? []);
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
  ctx.setLineDash([]);
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
      autoView = fitViewport(
        traces.flatMap((t) => [t.xs, t.ys]),
        w,
        h
      );
    }
    const view = activeView();
    if (!view) return null;
    drawGrid(ctx, view, w, h);
    for (const t of traces) drawTrace(ctx, view, w, h, t);
    return { ctx, view, w, h };
  }

  return { bind, render, isManual: () => manualView !== null };
}

export { fitViewport, tickStepMm, blankCanvas, createPathView };
export type { Viewport, PathTrace, RenderedView, PathView };
