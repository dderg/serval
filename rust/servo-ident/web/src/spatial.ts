import { el } from "./api";
import { hidpiCanvasContext } from "./charts-core";
import { liveDrawCount, state } from "./state";
import type { LiveSeries } from "./state";
import type { SpatialFrame } from "./wire";

// --- live spatial view -------------------------------------------------------
//
// Commanded vs actual toolhead path as the rotors see it: the tap's raw
// drive-frame target/pos counts, mapped to cartesian mm through the
// `spatial` frame SERVO_DUMP_TUNING writes into drive_state.json. The
// frame folds each motor's invert sign in and counts_per_mm rides on the
// tap payload, so the map is coeff = frame[mode][motor] / counts_per_mm
// per tap drive and everything else is a dot product per sample. The
// encoder zero is wherever power-on left it, so coordinates are relative —
// the shape (corner overshoot, ringing, lag) is the signal. Ctrl/meta+wheel
// zooms about the cursor, a plain wheel is a two-finger pan, drag pans,
// double-click or the fit button restores auto-fit; the time window is the
// live tab's shared slider.

interface SpatialCoeffs {
  x: Record<string, number>;
  y: Record<string, number>;
}

function spatialCoeffs(
  spatial: SpatialFrame | null | undefined,
  slots: Record<string, number> | null | undefined,
  countsPerMm: Record<string, number>
): SpatialCoeffs | string {
  if (!spatial || !slots) return "run SERVO_DUMP_TUNING to publish the kinematic frame";
  const xi = spatial.modes.indexOf("x");
  const yi = spatial.modes.indexOf("y");
  if (xi < 0 || yi < 0) {
    return `spatial frame covers only [${spatial.modes.join(", ")}] — the XY view needs servo x and y rails`;
  }
  const x: Record<string, number> = {};
  const y: Record<string, number> = {};
  for (let s = 0; s < spatial.axes.length; s++) {
    const motor = spatial.axes[s];
    const slot = slots[motor];
    if (slot === undefined) return `motor ${motor} has no tap slot — re-run SERVO_DUMP_TUNING`;
    const tap = `slot${slot}`;
    const cpm = countsPerMm[tap];
    if (!cpm) return `waiting for the tap header (no counts_per_mm for ${tap})`;
    x[tap] = spatial.frame[xi][s] / cpm;
    y[tap] = spatial.frame[yi][s] / cpm;
  }
  return { x, y };
}

function projectRow(
  row: Record<string, number>,
  perDrive: Record<string, LiveSeries>,
  n: number,
  channel: "target" | "pos"
): (number | null)[] {
  const taps = Object.keys(row);
  const out: (number | null)[] = new Array(n).fill(null);
  for (let i = 0; i < n; i++) {
    let v = 0;
    let complete = true;
    for (const tap of taps) {
      const raw = perDrive[tap]?.[channel][i];
      if (raw === null || raw === undefined) {
        complete = false;
        break;
      }
      v += row[tap] * raw;
    }
    if (complete) out[i] = v;
  }
  return out;
}

interface Viewport {
  cx: number;
  cy: number;
  mmPerPx: number;
}

const CMD_COLOR = "#4fb3ff";
const ACT_COLOR = "#e05a4f";
const GRID_COLOR = "#29313a";
const LABEL_COLOR = "#8a97a3";
const FIT_MARGIN_FRAC = 0.08;
const MIN_SPAN_MM = 0.02;
const MM_PER_PX_LIMITS = [1e-5, 10] as const;
const TICK_TARGET_PX = 90;

let manualView: Viewport | null = null;
let autoView: Viewport | null = null;
let drag: { px: number; py: number } | null = null;
let boundCanvas: HTMLCanvasElement | null = null;

function activeView(): Viewport | null {
  return manualView ?? autoView;
}

function fitViewport(paths: (number | null)[][], w: number, h: number): Viewport | null {
  let xMin = Infinity;
  let xMax = -Infinity;
  let yMin = Infinity;
  let yMax = -Infinity;
  const [xs1, ys1, xs2, ys2] = paths;
  for (const [xs, ys] of [
    [xs1, ys1],
    [xs2, ys2],
  ]) {
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

function drawPath(
  ctx: CanvasRenderingContext2D,
  view: Viewport,
  w: number,
  h: number,
  xs: (number | null)[],
  ys: (number | null)[],
  color: string
) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.25;
  ctx.beginPath();
  let penDown = false;
  let lastPx = NaN;
  let lastPy = NaN;
  for (let i = 0; i < xs.length; i++) {
    const x = xs[i];
    const y = ys[i];
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

function lastPoint(xs: (number | null)[], ys: (number | null)[]): [number, number] | null {
  for (let i = xs.length - 1; i >= 0; i--) {
    const x = xs[i];
    const y = ys[i];
    if (x !== null && y !== null) return [x, y];
  }
  return null;
}

function drawMarker(
  ctx: CanvasRenderingContext2D,
  view: Viewport,
  w: number,
  h: number,
  point: [number, number] | null,
  color: string,
  filled: boolean
) {
  if (!point) return;
  const px = (point[0] - view.cx) / view.mmPerPx + w / 2;
  const py = h / 2 - (point[1] - view.cy) / view.mmPerPx;
  ctx.beginPath();
  ctx.arc(px, py, filled ? 3 : 4, 0, 2 * Math.PI);
  if (filled) {
    ctx.fillStyle = color;
    ctx.fill();
  } else {
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }
}

function drawLegend(ctx: CanvasRenderingContext2D, w: number) {
  ctx.font = "11px monospace";
  ctx.fillStyle = CMD_COLOR;
  ctx.fillText("commanded", w - 170, 14);
  ctx.fillStyle = ACT_COLOR;
  ctx.fillText("actual", w - 68, 14);
}

function blankCanvas(canvas: HTMLCanvasElement) {
  const { ctx, w, h } = hidpiCanvasContext(canvas);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
}

function setNote(text: string) {
  const note = el("live-spatial-note");
  if (note) note.textContent = text;
}

function deviationText(paths: (number | null)[][]): string {
  const [cmdX, cmdY, actX, actY] = paths;
  for (let i = cmdX.length - 1; i >= 0; i--) {
    const cx = cmdX[i];
    const cy = cmdY[i];
    const ax = actX[i];
    const ay = actY[i];
    if (cx === null || cy === null || ax === null || ay === null) continue;
    const um = Math.hypot(ax - cx, ay - cy) * 1000;
    return `dev ${um.toFixed(0)} µm`;
  }
  return "";
}

function canvasMm(canvas: HTMLCanvasElement, view: Viewport, e: MouseEvent): [number, number] {
  const rect = canvas.getBoundingClientRect();
  const px = e.clientX - rect.left;
  const py = e.clientY - rect.top;
  return [view.cx + (px - rect.width / 2) * view.mmPerPx, view.cy - (py - rect.height / 2) * view.mmPerPx];
}

let redrawQueued = false;
let wheelSuppressUntil = 0;

function scheduleSpatialDraw() {
  if (redrawQueued) return;
  redrawQueued = true;
  requestAnimationFrame(() => {
    redrawQueued = false;
    drawSpatialView();
  });
}

const WHEEL_ZOOM_FACTOR_LIMITS = [0.1, 10] as const;
const WHEEL_MOUSEMOVE_SUPPRESS_MS = 80;

function bindSpatialEvents(canvas: HTMLCanvasElement) {
  if (boundCanvas === canvas) return;
  boundCanvas = canvas;
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
      scheduleSpatialDraw();
    },
    { passive: false }
  );
  canvas.addEventListener("pointerdown", (e) => {
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
    scheduleSpatialDraw();
  });
  canvas.addEventListener("pointerup", () => {
    drag = null;
  });
  canvas.addEventListener("dblclick", () => {
    manualView = null;
    scheduleSpatialDraw();
  });
  el("live-spatial-fit")?.addEventListener("click", () => {
    manualView = null;
    scheduleSpatialDraw();
  });
}

function drawSpatialView() {
  const canvas = el<HTMLCanvasElement>("live-spatial-canvas");
  if (!canvas) return;
  bindSpatialEvents(canvas);
  const coeffs = spatialCoeffs(
    state.drive.data?.spatial,
    state.drive.data?.slots,
    state.live.countsPerMm
  );
  if (typeof coeffs === "string") {
    blankCanvas(canvas);
    setNote(coeffs);
    return;
  }
  const n = liveDrawCount();
  const paths = [
    projectRow(coeffs.x, state.live.perDrive, n, "target"),
    projectRow(coeffs.y, state.live.perDrive, n, "target"),
    projectRow(coeffs.x, state.live.perDrive, n, "pos"),
    projectRow(coeffs.y, state.live.perDrive, n, "pos"),
  ];
  const { ctx, w, h } = hidpiCanvasContext(canvas);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
  autoView = fitViewport(paths, w, h);
  const view = activeView();
  if (!view) {
    setNote("waiting for samples…");
    return;
  }
  drawGrid(ctx, view, w, h);
  const [cmdX, cmdY, actX, actY] = paths;
  drawPath(ctx, view, w, h, cmdX, cmdY, CMD_COLOR);
  drawPath(ctx, view, w, h, actX, actY, ACT_COLOR);
  drawMarker(ctx, view, w, h, lastPoint(cmdX, cmdY), CMD_COLOR, false);
  drawMarker(ctx, view, w, h, lastPoint(actX, actY), ACT_COLOR, true);
  drawLegend(ctx, w);
  const zoomHint = manualView ? "" : " — ctrl+wheel zooms, scroll or drag pans, double-click refits";
  setNote(`${deviationText(paths)}${zoomHint}`);
}

export { spatialCoeffs, projectRow, fitViewport, tickStepMm, drawSpatialView };
export type { SpatialCoeffs, Viewport };
