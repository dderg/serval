import { el } from "./api";
import { blankCanvas, createPathView, fitViewport, tickStepMm } from "./path-view";
import type { Viewport } from "./path-view";
import { liveDrawCount, state } from "./state";
import type { LiveSeries } from "./state";
import { queryClient, queryKeys } from "./query-client";
import type { SpatialFrame, DriveState } from "./wire";

// --- live spatial view -------------------------------------------------------
//
// Commanded vs actual toolhead path as the rotors see it: the tap's raw
// drive-frame target/pos counts, mapped to cartesian mm through the
// `spatial` frame SERVO_DUMP_TUNING writes into drive_state.json. The
// frame folds each motor's invert sign in and counts_per_mm rides on the
// tap payload, so the map is coeff = frame[mode][motor] / counts_per_mm
// per tap drive and everything else is a dot product per sample. The
// encoder zero is wherever power-on left it, so coordinates are relative —
// the shape (corner overshoot, ringing, lag) is the signal. Viewport and
// gestures come from the shared path view (`path-view.ts`); the time
// window is the live tab's shared slider.

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

const CMD_COLOR = "#4fb3ff";
const ACT_COLOR = "#e05a4f";

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

const liveView = createPathView();

function drawSpatialView() {
  const canvas = el<HTMLCanvasElement>("live-spatial-canvas");
  if (!canvas) return;
  liveView.bind(canvas, el("live-spatial-fit"), drawSpatialView);
  const drive = queryClient.getQueryData<DriveState>(queryKeys.driveState);
  const coeffs = spatialCoeffs(drive?.spatial, drive?.slots, state.live.countsPerMm);
  if (typeof coeffs === "string") {
    blankCanvas(canvas);
    setNote(coeffs);
    return;
  }
  const n = liveDrawCount();
  const [cmdX, cmdY, actX, actY] = [
    projectRow(coeffs.x, state.live.perDrive, n, "target"),
    projectRow(coeffs.y, state.live.perDrive, n, "target"),
    projectRow(coeffs.x, state.live.perDrive, n, "pos"),
    projectRow(coeffs.y, state.live.perDrive, n, "pos"),
  ];
  const rendered = liveView.render(canvas, [
    { xs: cmdX, ys: cmdY, color: CMD_COLOR, width: 1.25 },
    { xs: actX, ys: actY, color: ACT_COLOR, width: 1.25 },
  ]);
  if (!rendered) {
    setNote("waiting for samples…");
    return;
  }
  const { ctx, view, w, h } = rendered;
  drawMarker(ctx, view, w, h, lastPoint(cmdX, cmdY), CMD_COLOR, false);
  drawMarker(ctx, view, w, h, lastPoint(actX, actY), ACT_COLOR, true);
  drawLegend(ctx, w);
  const zoomHint = liveView.isManual()
    ? ""
    : " — ctrl+wheel zooms, scroll or drag pans, double-click refits";
  setNote(`${deviationText([cmdX, cmdY, actX, actY])}${zoomHint}`);
}

export { spatialCoeffs, projectRow, fitViewport, tickStepMm, drawSpatialView };
export type { SpatialCoeffs, Viewport };
