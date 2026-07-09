// Shared 4-panel trajectory viewer: path + velocity/acceleration/jerk canvases
// with synced hover-scrub, zoom/pan, peak markers, impulse stems and a
// before/after variant flip. viewer.js (snapshot review) and playground.js
// (interactive gcode playground) each wrap one TrajectoryView with their own
// page chrome. Imports are module-relative so the same file works served by
// server.py and from a purely static host.
import init, { TrajectoryData } from "./wasm/snapshot_viewer.js";

// -- Colors ------------------------------------------------------------------
export const COLORS = {
  raw: "#555",
  line: "#4a9eff",
  arc: "#4ecb71",
  clothoid: "#f5a623",
  zero: "#4a9eff",
  constant: "#4ecb71",
  linear: "#f5a623",
  other: "#ff5252",
  cusp: "#e040fb",
  gap: "#ffd54a",
  kappa: "#4dd0e1",
  vx: "#4a9eff",
  vy: "#4ecb71",
  vz: "#ab7df8",
  ve: "#e874c8",
  tang: "#f5a623",
  cent: "#4dd0e1",
  scalar: "#ef5350",
  grid: "#22252b",
  axis: "#555",
  crosshair: "rgba(255,255,255,0.35)",
  marker: "#e0518a",
  impulse: "#ffd54a",
};

// -- Panel configuration -----------------------------------------------------
const PANELS = [
  { canvasId: "canvas-path", type: "path" },
  { canvasId: "canvas-vel", type: "vel" },
  { canvasId: "canvas-acc", type: "acc" },
  { canvasId: "canvas-jrk", type: "jrk" },
  { canvasId: "canvas-kappa", type: "kappa" },
];

// Tooltip wording for the derivative-discontinuity impulses drawn on the
// acc/jrk panels — a velocity step shows as an infinite accel spike, an
// accel step shows as an infinite jerk spike.
const IMPULSE_DESC = {
  acc: { label: "accel impulse", delta: "Δvel", unit: "mm/s", infOf: "∞ accel" },
  jrk: { label: "jerk impulse", delta: "Δaccel", unit: "mm/s²", infOf: "∞ jerk" },
};

export function initWasm() {
  return init();
}

// -- Nice tick spacing -------------------------------------------------------
function niceStep(range, targetTicks) {
  const rough = range / targetTicks;
  const mag = Math.pow(10, Math.floor(Math.log10(rough)));
  const norm = rough / mag;
  let nice;
  if (norm < 1.5) nice = 1;
  else if (norm < 3.5) nice = 2;
  else if (norm < 7.5) nice = 5;
  else nice = 10;
  return nice * mag;
}

// -- Helpers -----------------------------------------------------------------
export function formatNum(v) {
  // Past six digits a raw integer overflows the 56px axis gutter (jerk runs
  // into the millions), so switch to compact notation before it clips.
  if (Math.abs(v) >= 1e6) return v.toExponential(1);
  if (Math.abs(v) >= 1000) return v.toFixed(0);
  if (Math.abs(v) >= 1) return v.toFixed(1);
  if (Math.abs(v) >= 0.01) return v.toFixed(3);
  return v.toExponential(1);
}

function closestIndex(arr, val) {
  let lo = 0, hi = arr.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (arr[mid] < val) lo = mid + 1;
    else hi = mid;
  }
  if (lo > 0) {
    if (Math.abs(arr[lo - 1] - val) < Math.abs(arr[lo] - val)) lo--;
  }
  return lo;
}

function findPeaks(scalar) {
  if (scalar.length < 3) return [];
  let dataMax = 0;
  for (let i = 0; i < scalar.length; i++) {
    if (scalar[i] > dataMax) dataMax = scalar[i];
  }
  if (dataMax < 1e-9) return [];
  const threshold = dataMax * 0.3;
  const minGap = 10;
  const minProminence = dataMax * 0.03;
  const peaks = [];
  let lastPeak = -minGap;
  for (let i = 1; i < scalar.length - 1; i++) {
    if (scalar[i] < threshold) continue;
    if (i - lastPeak < minGap) continue;
    // Must be a local maximum: higher than both immediate neighbors
    if (scalar[i] <= scalar[i-1] || scalar[i] <= scalar[i+1]) continue;
    // Prominence: how much higher than the lowest point in nearby neighborhood
    const lo = Math.max(0, i - 3);
    const hi = Math.min(scalar.length - 1, i + 3);
    let surroundMin = scalar[i];
    for (let j = lo; j <= hi; j++) {
      if (scalar[j] < surroundMin) surroundMin = scalar[j];
    }
    if (scalar[i] - surroundMin < minProminence) continue;
    peaks.push(i);
    lastPeak = i;
  }
  return peaks;
}

// -- WASM array memoization --------------------------------------------------
// Every TrajectoryData getter copies its whole Vec into a fresh Float64Array
// across the WASM boundary, so calling them inside per-pointer-event hot paths
// re-marshalled the entire trajectory on every mouse move. Pull each array once
// per loaded variant and hand back the cached copy; keep the cheap index/scalar
// accessors as passthroughs so call sites stay `data.t()`.
const ARRAY_KEYS = [
  "raw_x", "raw_y", "kin_x", "kin_y", "t",
  "vx", "vy", "vz", "ve", "v_scalar",
  "ax", "ay", "az", "ae", "a_scalar",
  "jx", "jy", "jz", "je", "j_scalar",
  "kappa", "curvature_class",
  "a_tang", "a_cent", "j_tang", "j_cent",
  "jerk_impulse_t", "jerk_impulse_mag", "accel_impulse_t", "accel_impulse_mag",
];

function anyNonZero(arr) {
  for (let i = 0; i < arr.length; i++) if (arr[i] !== 0) return true;
  return false;
}

export function memoizeTrajectory(td) {
  const wrap = { free: () => td.free() };
  for (const k of ARRAY_KEYS) {
    const cached = td[k]();
    wrap[k] = () => cached;
  }
  wrap.traversal_time = () => td.traversal_time();
  wrap.point_count = () => td.point_count();
  return wrap;
}

export function trajectoryFromSnapshot(snapshotObj) {
  return memoizeTrajectory(new TrajectoryData(JSON.stringify(snapshotObj)));
}

// -- PanelRenderer -----------------------------------------------------------
class PanelRenderer {
  constructor(view, canvasId, type) {
    this.view = view;
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext("2d");
    // Static plot lives on the base canvas (redrawn only on bounds change); the
    // crosshair/marker are DOM elements moved by CSS transform, so a hover is a
    // compositor-only reposition — no per-frame canvas raster or texture upload.
    this.cursor = document.getElementById(`cursor-${type}`);
    this.cv = this.cursor.querySelector(".cv");
    this.ch = this.cursor.querySelector(".ch");
    this.cdot = this.cursor.querySelector(".cdot");
    this.type = type;
    this.margin = { top: 22, right: 14, bottom: 26, left: 56 };
    this._peaks = [];
    this._resize();
  }

  initObserver() {
    this._ro = new ResizeObserver(() => {
      this._resize();
      this.view.lastBoundsKey = "";
      this.view.scheduleFull();
    });
    this._ro.observe(this.canvas.parentElement);
  }

  _resize() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const newW = rect.width;
    const newH = rect.height;
    if (newW === this.w && newH === this.h) return;
    this.w = newW;
    this.h = newH;
    this.canvas.width = this.w * dpr;
    this.canvas.height = this.h * dpr;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this._applyMargins();
  }

  _applyMargins() {
    this.plotW = this.w - this.margin.left - this.margin.right;
    this.plotH = this.h - this.margin.top - this.margin.bottom;
    this.cv.style.height = this.plotH + "px";
    this.ch.style.width = this.plotW + "px";
  }

  get plotX0() { return this.margin.left; }
  get plotY0() { return this.margin.top; }

  toPixelX(dataVal, dataMin, dataMax) {
    return this.plotX0 + ((dataVal - dataMin) / (dataMax - dataMin)) * this.plotW;
  }
  toPixelY(dataVal, dataMin, dataMax) {
    return this.plotY0 + this.plotH - ((dataVal - dataMin) / (dataMax - dataMin)) * this.plotH;
  }
  toDataX(pixelX, dataMin, dataMax) {
    return dataMin + ((pixelX - this.plotX0) / this.plotW) * (dataMax - dataMin);
  }
  toDataY(pixelY, dataMin, dataMax) {
    return dataMax - ((pixelY - this.plotY0) / this.plotH) * (dataMax - dataMin);
  }

  _drawGrid(ctx, xMin, xMax, yMin, yMax) {
    const { plotX0, plotY0, plotW, plotH } = this;

    ctx.save();
    ctx.strokeStyle = COLORS.grid;
    ctx.lineWidth = 0.5;

    const xStep = niceStep(xMax - xMin, Math.max(2, Math.floor(plotW / 80)));
    const xStart = Math.ceil(xMin / xStep) * xStep;
    ctx.beginPath();
    for (let x = xStart; x <= xMax; x += xStep) {
      const px = this.toPixelX(x, xMin, xMax);
      ctx.moveTo(px, plotY0);
      ctx.lineTo(px, plotY0 + plotH);
    }
    ctx.stroke();

    const yStep = niceStep(yMax - yMin, Math.max(2, Math.floor(plotH / 50)));
    const yStart = Math.ceil(yMin / yStep) * yStep;
    ctx.beginPath();
    for (let y = yStart; y <= yMax; y += yStep) {
      const py = this.toPixelY(y, yMin, yMax);
      ctx.moveTo(plotX0, py);
      ctx.lineTo(plotX0 + plotW, py);
    }
    ctx.stroke();

    ctx.fillStyle = COLORS.axis;
    ctx.font = "10px 'Courier Prime', monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let x = xStart; x <= xMax; x += xStep) {
      const px = this.toPixelX(x, xMin, xMax);
      ctx.fillText(formatNum(x), px, plotY0 + plotH + 4);
    }
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let y = yStart; y <= yMax; y += yStep) {
      const py = this.toPixelY(y, yMin, yMax);
      ctx.fillText(formatNum(y), plotX0 - 4, py);
    }
    ctx.restore();
  }

  // -- Render path panel to buffer -------------------------------------------
  // Equalize bounds so 1 data unit = same number of pixels in X and Y
  _equalizePathBounds(xMin, xMax, yMin, yMax) {
    const xRange = xMax - xMin;
    const yRange = yMax - yMin;
    const dataAspect = xRange / (yRange || 1);
    const pixelAspect = this.plotW / (this.plotH || 1);
    let nxMin = xMin, nxMax = xMax, nyMin = yMin, nyMax = yMax;
    if (dataAspect < pixelAspect) {
      // Canvas wider than data — expand X
      const targetXRange = yRange * pixelAspect;
      const xMid = (xMin + xMax) / 2;
      nxMin = xMid - targetXRange / 2;
      nxMax = xMid + targetXRange / 2;
    } else if (dataAspect > pixelAspect) {
      // Canvas taller than data — expand Y
      const targetYRange = xRange / pixelAspect;
      const yMid = (yMin + yMax) / 2;
      nyMin = yMid - targetYRange / 2;
      nyMax = yMid + targetYRange / 2;
    }
    return { xMin: nxMin, xMax: nxMax, yMin: nyMin, yMax: nyMax };
  }

  renderPathBuffer(xMin, xMax, yMin, yMax) {
    const DATA = this.view.data;
    const eq = this._equalizePathBounds(xMin, xMax, yMin, yMax);
    xMin = eq.xMin; xMax = eq.xMax; yMin = eq.yMin; yMax = eq.yMax;
    this._eqPathBounds = eq; // store for compositePath

    const bctx = this.ctx;
    bctx.clearRect(0, 0, this.w, this.h);
    this._drawGrid(bctx, xMin, xMax, yMin, yMax);

    const rawX = DATA.raw_x(), rawY = DATA.raw_y();
    bctx.save();
    bctx.beginPath();
    bctx.strokeStyle = COLORS.raw;
    bctx.lineWidth = 0.8;
    for (let i = 0; i < rawX.length; i++) {
      const px = this.toPixelX(rawX[i], xMin, xMax);
      const py = this.toPixelY(rawY[i], yMin, yMax);
      i === 0 ? bctx.moveTo(px, py) : bctx.lineTo(px, py);
    }
    bctx.stroke();

    this._strokeCurvaturePath(bctx, DATA, xMin, xMax, yMin, yMax);

    if (rawX.length > 0) {
      bctx.beginPath();
      bctx.fillStyle = COLORS.scalar;
      bctx.arc(
        this.toPixelX(rawX[0], xMin, xMax),
        this.toPixelY(rawY[0], yMin, yMax),
        4, 0, Math.PI * 2
      );
      bctx.fill();
    }
    bctx.restore();
  }

  // The executed (post-lowered, post-shaper) toolhead path, colored by the
  // measured curvature-behavior class at each sample -- the same dense grid
  // driving every other panel, walked directly with no segment-boundary
  // matching needed. Cusp/Gap are point anomalies (a near-zero-speed
  // instant, a piece-domain mismatch), drawn as markers rather than folded
  // into the stroke color.
  _strokeCurvaturePath(bctx, DATA, xMin, xMax, yMin, yMax) {
    const kx = DATA.kin_x(), ky = DATA.kin_y();
    const cls = DATA.curvature_class();
    if (kx.length <= 1) return;
    const CLASS_COLOR = [COLORS.zero, COLORS.constant, COLORS.linear, COLORS.other];
    let curColor = null;
    bctx.lineWidth = 1.2;
    const markers = [];
    for (let i = 1; i < kx.length; i++) {
      const c = cls[i];
      if (c === 4 || c === 5) { markers.push(i); continue; }
      const color = CLASS_COLOR[c] || COLORS.zero;
      if (color !== curColor) {
        if (curColor !== null) bctx.stroke();
        bctx.beginPath();
        bctx.strokeStyle = color;
        bctx.moveTo(this.toPixelX(kx[i - 1], xMin, xMax), this.toPixelY(ky[i - 1], yMin, yMax));
        curColor = color;
      }
      bctx.lineTo(this.toPixelX(kx[i], xMin, xMax), this.toPixelY(ky[i], yMin, yMax));
    }
    if (curColor !== null) bctx.stroke();
    for (const i of markers) {
      bctx.beginPath();
      bctx.fillStyle = cls[i] === 4 ? COLORS.cusp : COLORS.gap;
      bctx.arc(this.toPixelX(kx[i], xMin, xMax), this.toPixelY(ky[i], yMin, yMax), 2.5, 0, Math.PI * 2);
      bctx.fill();
    }
  }

  _strokeSeries(bctx, t, valueAt, tMin, tMax, yMin, yMax) {
    const inView = (i) => i >= 0 && i < t.length && t[i] >= tMin && t[i] <= tMax;
    const touchesView = (i) => inView(i - 1) || inView(i) || inView(i + 1);
    bctx.beginPath();
    let started = false;
    for (let i = 0; i < t.length; i++) {
      if (!touchesView(i)) { started = false; continue; }
      const px = this.toPixelX(t[i], tMin, tMax);
      const py = this.toPixelY(valueAt(i), yMin, yMax);
      if (!started) { bctx.moveTo(px, py); started = true; }
      else bctx.lineTo(px, py);
    }
    bctx.stroke();
  }

  // -- Render time-series panel to buffer ------------------------------------
  // `series` is a list of { arr, color, label, yMax, dash?, scalar?, axis? }
  // lanes plotted as |value|, each against its own yMax. Entries with
  // axis:"right" (the E lane, whose magnitudes dwarf or vanish next to XY)
  // get a secondary right-hand axis; the `scalar` entry is the headline
  // magnitude trace (wider, carries the peak markers). Hidden entries are
  // filtered out by the caller.
  renderTimeBuffer(tMin, tMax, yMin, yMax, series, drawPeaks) {
    const DATA = this.view.data;
    const right = series.find(s => s.axis === "right");
    const wantRight = right ? 46 : 14;
    if (this.margin.right !== wantRight) {
      this.margin.right = wantRight;
      this._applyMargins();
    }

    const bctx = this.ctx;
    bctx.clearRect(0, 0, this.w, this.h);
    this._drawGrid(bctx, tMin, tMax, yMin, yMax);
    if (right) this._drawRightAxis(bctx, 0, right.yMax, right.color);

    const t = DATA.t();
    bctx.save();
    bctx.beginPath();
    bctx.rect(this.plotX0, this.plotY0, this.plotW, this.plotH);
    bctx.clip();

    let scalarEntry = null;
    for (const s of series) {
      if (s.scalar) { scalarEntry = s; continue; }
      bctx.strokeStyle = s.color;
      bctx.lineWidth = 1.0;
      bctx.setLineDash(s.dash || []);
      this._strokeSeries(bctx, t, (i) => Math.abs(s.arr[i]), tMin, tMax, yMin, s.yMax);
    }
    bctx.setLineDash([]);

    this._peaks = [];
    if (scalarEntry) {
      const scalar = scalarEntry.arr;
      bctx.strokeStyle = scalarEntry.color;
      bctx.lineWidth = 1.2;
      this._strokeSeries(bctx, t, (i) => Math.abs(scalar[i]), tMin, tMax, yMin, yMax);

      if (drawPeaks) {
        const peaks = findPeaks(scalar);
        for (const pi of peaks) {
          if (t[pi] < tMin || t[pi] > tMax) continue;
          const px = this.toPixelX(t[pi], tMin, tMax);
          const py = this.toPixelY(scalar[pi], yMin, yMax);
          bctx.beginPath();
          bctx.fillStyle = COLORS.marker;
          bctx.arc(px, py, 3, 0, Math.PI * 2);
          bctx.fill();
          this._peaks.push({ px, py, tVal: t[pi], val: scalar[pi], idx: pi });
        }
      }
    }

    bctx.restore();
  }

  _drawRightAxis(ctx, yMin, yMax, color) {
    const { plotX0, plotY0, plotW, plotH } = this;
    ctx.save();
    ctx.fillStyle = color;
    ctx.font = "10px 'Courier Prime', monospace";
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    const yStep = niceStep(yMax - yMin, Math.max(2, Math.floor(plotH / 50)));
    for (let y = Math.ceil(yMin / yStep) * yStep; y <= yMax; y += yStep) {
      const py = plotY0 + plotH - ((y - yMin) / (yMax - yMin)) * plotH;
      ctx.fillText(formatNum(y), plotX0 + plotW + 4, py);
    }
    ctx.restore();
  }

  // -- Find nearest peak within pixel radius ---------------------------------
  nearestPeak(mx, my, radius) {
    let best = null, bestDist = radius * radius;
    for (const p of this._peaks) {
      const dx = p.px - mx, dy = p.py - my;
      const d = dx * dx + dy * dy;
      if (d < bestDist) { bestDist = d; best = p; }
    }
    return best;
  }

  // -- Derivative impulses (accel/velocity discontinuities) ------------------
  // A step in acceleration (or velocity) is an infinite, zero-width jerk (or
  // accel) spike the analytic per-piece derivative can't plot. Draw each as a
  // stem whose height encodes |Δvalue| relative to the largest impulse on
  // this panel; the exact value shows on hover.
  drawImpulses(times, mags, tMin, tMax, maxMag) {
    this._impulses = [];
    const bctx = this.ctx;
    bctx.save();
    bctx.beginPath();
    bctx.rect(this.plotX0, this.plotY0, this.plotW, this.plotH);
    bctx.clip();
    bctx.strokeStyle = COLORS.impulse;
    bctx.fillStyle = COLORS.impulse;
    bctx.lineWidth = 1.5;
    const base = this.plotY0 + this.plotH;
    for (let i = 0; i < times.length; i++) {
      const tb = times[i];
      if (tb < tMin || tb > tMax) continue;
      const px = this.toPixelX(tb, tMin, tMax);
      const frac = maxMag > 0 ? Math.min(1, mags[i] / maxMag) : 1;
      const top = this.plotY0 + this.plotH * (1 - 0.92 * frac);
      bctx.beginPath();
      bctx.moveTo(px, base);
      bctx.lineTo(px, top);
      bctx.stroke();
      bctx.beginPath();
      bctx.moveTo(px, top);
      bctx.lineTo(px - 4, top + 8);
      bctx.lineTo(px + 4, top + 8);
      bctx.closePath();
      bctx.fill();
      this._impulses.push({ px, tVal: tb, mag: mags[i] });
    }
    bctx.restore();
  }

  nearestImpulse(mx, radius) {
    if (!this._impulses) return null;
    let best = null, bd = radius;
    for (const im of this._impulses) {
      const d = Math.abs(im.px - mx);
      if (d < bd) { bd = d; best = im; }
    }
    return best;
  }

  // -- Move the DOM cursor; the base plot is never touched -------------------
  composite(tVal, tMin, tMax, showCursor) {
    if (showCursor && tVal != null && this.type !== "path") {
      const px = this.toPixelX(tVal, tMin, tMax);
      if (px >= this.plotX0 && px <= this.plotX0 + this.plotW) {
        this.cv.style.transform = `translate(${px}px,${this.plotY0}px)`;
        this.cursor.classList.add("on");
        return;
      }
    }
    this.cursor.classList.remove("on");
  }

  compositePath(idx) {
    if (idx == null) { this.cursor.classList.remove("on"); return; }
    const DATA = this.view.data;
    const kx = DATA.kin_x(), ky = DATA.kin_y();
    if (idx >= kx.length) { this.cursor.classList.remove("on"); return; }
    const pb = this._eqPathBounds || this.view.pathView;
    const px = this.toPixelX(kx[idx], pb.xMin, pb.xMax);
    const py = this.toPixelY(ky[idx], pb.yMin, pb.yMax);

    this.cv.style.transform = `translate(${px}px,${this.plotY0}px)`;
    this.ch.style.transform = `translate(${this.plotX0}px,${py}px)`;
    this.cdot.style.transform = `translate(${px}px,${py}px)`;
    this.ch.style.display = "block";
    this.cdot.style.display = "block";
    this.cursor.classList.add("on");
  }
}

// -- Compute bounds ----------------------------------------------------------
function computeDataBounds(data) {
  const rx = data.raw_x(), ry = data.raw_y();
  const kx = data.kin_x(), ky = data.kin_y();
  let xMin = Infinity, xMax = -Infinity, yMin = Infinity, yMax = -Infinity;
  for (let i = 0; i < rx.length; i++) {
    if (rx[i] < xMin) xMin = rx[i]; if (rx[i] > xMax) xMax = rx[i];
    if (ry[i] < yMin) yMin = ry[i]; if (ry[i] > yMax) yMax = ry[i];
  }
  for (let i = 0; i < kx.length; i++) {
    if (kx[i] < xMin) xMin = kx[i]; if (kx[i] > xMax) xMax = kx[i];
    if (ky[i] < yMin) yMin = ky[i]; if (ky[i] > yMax) yMax = ky[i];
  }
  const padX = Math.max((xMax - xMin) * 0.08, 2);
  const padY = Math.max((yMax - yMin) * 0.08, 2);
  return { xMin: xMin - padX, xMax: xMax + padX, yMin: yMin - padY, yMax: yMax + padY };
}

function computeTimeBounds(data) {
  const t = data.t();
  const tMax = t.length > 0 ? t[t.length - 1] : 1;
  return { tMin: 0, tMax: tMax * 1.02 };
}

// -- TrajectoryView ----------------------------------------------------------
// Owns the four panels, the shared view/hover state and the before/after data
// pair. Page chrome hooks in via `onChanged`, fired after every data or
// variant change so the page can sync its own meta/buttons.
export class TrajectoryView {
  constructor({ hiddenSeriesKey = "snapshotViewer.hiddenSeries" } = {}) {
    // Per-panel hidden series, toggled by clicking legend chips; persisted so
    // a preferred view (e.g. XY-scalar-only) survives reloads and case
    // switches.
    this.hiddenSeriesKey = hiddenSeriesKey;
    this.hiddenSeries = { vel: new Set(), acc: new Set(), jrk: new Set() };
    try {
      const saved = JSON.parse(localStorage.getItem(hiddenSeriesKey)) || {};
      for (const type of Object.keys(this.hiddenSeries)) {
        for (const label of saved[type] || []) this.hiddenSeries[type].add(label);
      }
    } catch (e) { /* corrupt storage — start with everything visible */ }

    this.timeView = { tMin: 0, tMax: 0 };
    this.pathView = { xMin: 0, xMax: 0, yMin: 0, yMax: 0 };
    this.defaultTimeView = { tMin: 0, tMax: 0 };
    this.defaultPathView = { xMin: 0, xMax: 0, yMin: 0, yMax: 0 };

    this.data = null;
    this.dataAfter = null; // current trajectory
    this.dataBefore = null; // comparison trajectory, null when nothing to compare
    this.variant = "after"; // which of the two `data` currently points at
    this.lastBoundsKey = "";
    this.hoverIdx = null;
    this.hoverTime = null; // time the marker sits at; drives the graph crosshair
    // What the cursor is anchored to, so a before/after flip keeps it under the
    // pointer. "time": graphs own the cursor (fixed time, path marker re-derives).
    // "path": the path owns it (fixed xy, the time it is reached re-derives).
    this.hoverMode = null; // "time" | "path" | null
    this.hoverXY = null; // { x, y } cursor position when hoverMode === "path"
    this.showPeaks = false;
    this.wheelTimer = null; // suppresses mousemove during trackpad gestures
    this.tooltipEl = document.getElementById("tooltip");
    this.readoutEl = document.getElementById("readout");
    this.onChanged = null;

    // Pointer, wheel and drag events fire faster than the display refreshes
    // (high-poll mice, streaming wheel deltas). Coalesce every redraw into a
    // single requestAnimationFrame so we draw at most once per displayed frame.
    this._frameId = 0;
    this._pendingFullRender = false;

    this.renderers = PANELS.map(p => new PanelRenderer(this, p.canvasId, p.type));
    this.renderers.forEach(r => r.initObserver());
    this._setupPathInteraction();
    for (let i = 1; i < this.renderers.length; i++) {
      this._setupTimeInteraction(i);
    }
    this._setupLegendToggles();
  }

  _saveHiddenSeries() {
    const out = {};
    for (const [type, set] of Object.entries(this.hiddenSeries)) out[type] = [...set];
    localStorage.setItem(this.hiddenSeriesKey, JSON.stringify(out));
  }

  _setupLegendToggles() {
    for (const type of Object.keys(this.hiddenSeries)) {
      const el = document.querySelector(`#panel-${type} .panel-label`);
      el.addEventListener("click", (e) => {
        const label = e.target.dataset && e.target.dataset.series;
        if (!label) return;
        const set = this.hiddenSeries[type];
        if (set.has(label)) set.delete(label);
        else set.add(label);
        this._saveHiddenSeries();
        this.lastBoundsKey = "";
        this.scheduleFull();
      });
    }
  }

  // Suppress mousemove during trackpad wheel gestures (macOS collision)
  _suppressMousemove() {
    clearTimeout(this.wheelTimer);
    this.wheelTimer = setTimeout(() => { this.wheelTimer = null; }, 80);
  }

  _nearestPathPoint(dataX, dataY) {
    const kx = this.data.kin_x(), ky = this.data.kin_y();
    let best = 0, bd = Infinity;
    for (let i = 0; i < kx.length; i++) {
      const dx = kx[i] - dataX, dy = ky[i] - dataY;
      const d = dx * dx + dy * dy;
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  }

  // Per-class sample counts on the current path, e.g. "412 zero, 88
  // constant, 40 linear, 6 other, 2 cusp".
  curvatureSummary() {
    const NAMES = ["zero", "constant", "linear", "other", "cusp", "gap"];
    const cls = this.data.curvature_class();
    const counts = {};
    for (let i = 0; i < cls.length; i++) {
      const name = NAMES[cls[i]] || "other";
      counts[name] = (counts[name] || 0) + 1;
    }
    return Object.keys(counts).sort().map(k => `${counts[k]} ${k}`).join(", ");
  }

  hasBefore() { return this.dataBefore != null; }

  // -- Readout ---------------------------------------------------------------
  _updateReadout(idx) {
    const el = this.readoutEl;
    if (idx == null) {
      el.innerHTML = '<span class="g">hover a panel to scrub</span>';
      return;
    }
    const t = this.data.t()[idx];
    const kx = this.data.kin_x()[idx], ky = this.data.kin_y()[idx];
    const v = this.data.v_scalar()[idx];
    const a = this.data.a_scalar()[idx];
    const j = this.data.j_scalar()[idx];
    const kappa = this.data.kappa()[idx];
    const aTang = this.data.a_tang()[idx];
    const aCent = this.data.a_cent()[idx];
    const vz = this.data.vz(), ve = this.data.ve();

    let extra = "";
    if (anyNonZero(vz)) extra += `<span class="g">vZ=${formatNum(vz[idx])}</span>`;
    if (anyNonZero(ve)) extra += `<span class="g">vE=${formatNum(ve[idx])}</span>`;

    el.innerHTML =
      `<span class="g">t=${formatNum(t)}s</span>` +
      `<span class="v">X=${formatNum(kx)}</span>` +
      `<span class="p">Y=${formatNum(ky)}</span>` +
      `<span class="g">|v|=${formatNum(v)}</span>` +
      extra +
      `<span class="r">a=${formatNum(a)}</span>` +
      `<span class="g">a∥=${formatNum(aTang)}</span>` +
      `<span class="g">a⊥=${formatNum(aCent)}</span>` +
      `<span class="g">j=${formatNum(j)}</span>` +
      `<span class="g">κ=${formatNum(kappa)}</span>`;
  }

  // -- Synced hover ------------------------------------------------------------
  _syncHover(idx) {
    this.hoverIdx = idx;
    const t = this.data.t();
    const dataT = this.hoverTime != null ? this.hoverTime : idx != null ? t[idx] : null;
    const { tMin, tMax } = this.timeView;

    // Composite all time panels (fast — just blit + cursor)
    for (let i = 1; i < this.renderers.length; i++) {
      this.renderers[i].composite(dataT, tMin, tMax, idx != null);
    }

    // Composite path with marker
    this.renderers[0].compositePath(idx);

    this._updateReadout(idx);
  }

  scheduleFull() {
    this._pendingFullRender = true;
    if (!this._frameId) this._frameId = requestAnimationFrame(() => this._runFrame());
  }

  scheduleHover() {
    if (!this._frameId) this._frameId = requestAnimationFrame(() => this._runFrame());
  }

  _runFrame() {
    this._frameId = 0;
    if (this._pendingFullRender) {
      // Keep the request alive until data exists: a frame that fired between
      // a ResizeObserver's canvas clear and the snapshot arriving must not
      // swallow the repaint, or the cleared panels stay blank forever.
      if (!this.data) return;
      this._pendingFullRender = false;
      this.renderAll();
    } else {
      this._compositeHover();
    }
  }

  // Blit buffers + cursor for the current hover state, without rebuilding buffers.
  _compositeHover() {
    if (!this.data) return;
    const { tMin, tMax } = this.timeView;
    if (this.hoverIdx != null) {
      this._syncHover(this.hoverIdx);
    } else {
      for (let i = 1; i < this.renderers.length; i++) {
        this.renderers[i].composite(null, tMin, tMax, false);
      }
      this.renderers[0].compositePath(null);
    }
  }

  // Lanes for one derivative panel: X/Y always, Z/E only when the case moves
  // them, the tangent/normal projection of the XY vector (dashed), and the
  // |XY| magnitude last so it draws on top. E rides its own right-hand axis.
  // Hidden state comes from the per-panel legend toggles.
  _panelSeries(type, xKey, yKey, zKey, eKey, tangKey, centKey, scalarKey) {
    const DATA = this.data;
    const series = [
      { key: xKey, color: COLORS.vx, label: "|X|" },
      { key: yKey, color: COLORS.vy, label: "|Y|" },
    ];
    if (anyNonZero(DATA[zKey]())) series.push({ key: zKey, color: COLORS.vz, label: "|Z|" });
    if (anyNonZero(DATA[eKey]())) series.push({ key: eKey, color: COLORS.ve, label: "|E|", axis: "right" });
    if (tangKey) {
      series.push({ key: tangKey, color: COLORS.tang, dash: [5, 3], label: "∥" });
      series.push({ key: centKey, color: COLORS.cent, dash: [5, 3], label: "⊥" });
    }
    series.push({ key: scalarKey, color: COLORS.scalar, label: "|XY|", scalar: true });
    for (const s of series) {
      s.arr = DATA[s.key]();
      s.hidden = this.hiddenSeries[type].has(s.label);
    }
    return series;
  }

  _updateLegend(type, title, series) {
    const el = document.querySelector(`#panel-${type} .panel-label`);
    const entries = series.map(s =>
      `<span class="lg${s.hidden ? " off" : ""}" data-series="${s.label}"` +
      ` style="color:${s.color}">${s.label}</span>`
    );
    el.innerHTML = `${title}&ensp;${entries.join(" ")}`;
  }

  // -- Render all buffers (only when bounds change) ----------------------------
  renderAll() {
    if (!this.data) return;
    const { tMin, tMax } = this.timeView;
    const { xMin, xMax, yMin, yMax } = this.pathView;

    // Scale across both variants so blinking before/after keeps a fixed axis —
    // a peak that shrinks must visibly drop, not get renormalized to the top.
    // Only visible series count, so hiding a dominant lane rescales the rest;
    // right-axis (E) series get their own independent scale.
    function visibleMaxOf(data, key) {
      const dt = data.t();
      const arr = data[key]();
      let m = 0;
      for (let i = 0; i < dt.length; i++) {
        const a = Math.abs(arr[i]);
        if (dt[i] >= tMin && dt[i] <= tMax && a > m) m = a;
      }
      return m;
    }
    const visibleMax = (keys) => {
      let m = 0;
      for (const key of keys) {
        m = Math.max(m, visibleMaxOf(this.dataAfter, key));
        if (this.dataBefore) m = Math.max(m, visibleMaxOf(this.dataBefore, key));
      }
      return m * 1.15 || 1;
    };
    function scaleSeries(series) {
      const shown = series.filter(s => !s.hidden);
      const axisKeys = (axis) => shown.filter(s => s.axis === axis).map(s => s.key);
      const leftMax = visibleMax(axisKeys(undefined));
      const rightMax = visibleMax(axisKeys("right"));
      for (const s of shown) s.yMax = s.axis === "right" ? rightMax : leftMax;
      return { shown, leftMax };
    }

    const velSeries = this._panelSeries("vel", "vx", "vy", "vz", "ve", null, null, "v_scalar");
    const accSeries = this._panelSeries("acc", "ax", "ay", "az", "ae", "a_tang", "a_cent", "a_scalar");
    const jrkSeries = this._panelSeries("jrk", "jx", "jy", "jz", "je", "j_tang", "j_cent", "j_scalar");
    const kappaSeries = [{ key: "kappa", color: COLORS.kappa, label: "κ", scalar: true, hidden: false }];
    kappaSeries[0].arr = this.data.kappa();
    const vel = scaleSeries(velSeries);
    const acc = scaleSeries(accSeries);
    const jrk = scaleSeries(jrkSeries);
    const kappa = scaleSeries(kappaSeries);

    const hiddenKey = Object.entries(this.hiddenSeries)
      .map(([k, v]) => `${k}:${[...v].join("+")}`)
      .join(";");
    const key = `${tMin},${tMax},${xMin},${xMax},${yMin},${yMax},` +
      `${vel.leftMax},${acc.leftMax},${jrk.leftMax},${kappa.leftMax},${hiddenKey},${this.variant}`;
    if (key !== this.lastBoundsKey) {
      this.lastBoundsKey = key;

      this._updateLegend("vel", "Velocity", velSeries);
      this._updateLegend("acc", "Acceleration", accSeries);
      this._updateLegend("jrk", "Jerk", jrkSeries);
      this._updateLegend("kappa", "Curvature", kappaSeries);

      this.renderers[0].renderPathBuffer(xMin, xMax, yMin, yMax);
      this.renderers[1].renderTimeBuffer(tMin, tMax, 0, vel.leftMax, vel.shown, this.showPeaks);
      this.renderers[2].renderTimeBuffer(tMin, tMax, 0, acc.leftMax, acc.shown, this.showPeaks);
      this.renderers[3].renderTimeBuffer(tMin, tMax, 0, jrk.leftMax, jrk.shown, this.showPeaks);
      this.renderers[4].renderTimeBuffer(tMin, tMax, 0, kappa.leftMax, kappa.shown, this.showPeaks);

      const DATA = this.data;
      const drawImpulsesOn = (renderer, times, mags) => {
        let max = 0;
        for (let i = 0; i < mags.length; i++) if (mags[i] > max) max = mags[i];
        renderer.drawImpulses(times, mags, tMin, tMax, max);
      };
      drawImpulsesOn(this.renderers[2], DATA.accel_impulse_t(), DATA.accel_impulse_mag());
      drawImpulsesOn(this.renderers[3], DATA.jerk_impulse_t(), DATA.jerk_impulse_mag());
    }

    // Composite all (always — cursor may have moved)
    this._compositeHover();
  }

  // -- Interaction (time panels) -----------------------------------------------
  _setupTimeInteraction(panelIdx) {
    const view = this;
    const r = this.renderers[panelIdx];
    const canvas = r.canvas;
    let dragging = false;
    let dragStartX = 0, dragStartTMin = 0, dragStartTMax = 0;

    canvas.addEventListener("mousemove", (e) => {
      if (view.wheelTimer || !view.data) return;
      const rect = r.canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;

      if (dragging) {
        const dx = e.clientX - dragStartX;
        const dtPerPx = (dragStartTMax - dragStartTMin) / r.plotW;
        const dt = -dx * dtPerPx;
        view.timeView.tMin = dragStartTMin + dt;
        view.timeView.tMax = dragStartTMax + dt;
        view.lastBoundsKey = "";
        view.scheduleFull();
      } else {
        const dataT = r.toDataX(mx, view.timeView.tMin, view.timeView.tMax);
        view.hoverMode = "time";
        view.hoverTime = dataT;
        view.hoverIdx = closestIndex(view.data.t(), dataT);
        view.scheduleHover();

        // A derivative impulse (accel/velocity discontinuity) takes precedence
        // over a peak.
        const impulse = r.nearestImpulse(mx, 6);
        const peak = r.nearestPeak(mx, my, 12);
        const tooltipEl = view.tooltipEl;
        if (impulse) {
          const desc = IMPULSE_DESC[r.type];
          tooltipEl.style.display = "block";
          tooltipEl.style.left = (e.clientX + 14) + "px";
          tooltipEl.style.top = (e.clientY - 10) + "px";
          tooltipEl.textContent =
            `${desc.label} at t=${formatNum(impulse.tVal)}s\n` +
            `${desc.delta}=${formatNum(impulse.mag)} ${desc.unit} (${desc.infOf})`;
        } else if (peak) {
          tooltipEl.style.display = "block";
          tooltipEl.style.left = (e.clientX + 14) + "px";
          tooltipEl.style.top = (e.clientY - 10) + "px";
          tooltipEl.textContent = `peak at t=${formatNum(peak.tVal)}s\nvalue=${formatNum(peak.val)}`;
        } else {
          tooltipEl.style.display = "none";
        }
      }
    });

    canvas.addEventListener("mouseleave", () => {
      view.hoverIdx = null;
      view.hoverTime = null;
      view.hoverMode = null;
      view.hoverXY = null;
      view.tooltipEl.style.display = "none";
      view.scheduleHover();
    });

    canvas.addEventListener("mousedown", (e) => {
      dragging = true;
      dragStartX = e.clientX;
      dragStartTMin = view.timeView.tMin;
      dragStartTMax = view.timeView.tMax;
      canvas.style.cursor = "grabbing";
    });

    window.addEventListener("mouseup", () => {
      if (dragging) { dragging = false; canvas.style.cursor = ""; }
    });

    canvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      view._suppressMousemove();
      const rect = r.canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const tAtCursor = r.toDataX(mx, view.timeView.tMin, view.timeView.tMax);

      if (e.ctrlKey || e.metaKey) {
        const factor = Math.max(0.1, Math.min(10, Math.exp(e.deltaY * 0.01)));
        view.timeView.tMin = tAtCursor - (tAtCursor - view.timeView.tMin) * factor;
        view.timeView.tMax = tAtCursor + (view.timeView.tMax - tAtCursor) * factor;
      } else {
        const dtPerPx = (view.timeView.tMax - view.timeView.tMin) / r.plotW;
        const dx = e.deltaX !== 0 ? e.deltaX : e.deltaY;
        const dt = dx * dtPerPx;
        view.timeView.tMin += dt;
        view.timeView.tMax += dt;
      }
      view.lastBoundsKey = "";
      view.scheduleFull();
    }, { passive: false });
  }

  // -- Interaction (path panel) ------------------------------------------------
  _setupPathInteraction() {
    const view = this;
    const r = this.renderers[0];
    const canvas = r.canvas;
    let dragging = false;
    let dragStartX = 0, dragStartY = 0, dragStartPV = null;

    canvas.addEventListener("mousemove", (e) => {
      if (view.wheelTimer || !view.data) return;
      const rect = r.canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;

      if (dragging) {
        const dx = e.clientX - dragStartX;
        const dy = e.clientY - dragStartY;
        const dppx = (dragStartPV.xMax - dragStartPV.xMin) / r.plotW;
        const dppy = (dragStartPV.yMax - dragStartPV.yMin) / r.plotH;
        view.pathView.xMin = dragStartPV.xMin - dx * dppx;
        view.pathView.xMax = dragStartPV.xMax - dx * dppx;
        view.pathView.yMin = dragStartPV.yMin + dy * dppy;
        view.pathView.yMax = dragStartPV.yMax + dy * dppy;
        view.lastBoundsKey = "";
        view.scheduleFull();
      } else {
        const pb = r._eqPathBounds || view.pathView;
        const dataX = r.toDataX(mx, pb.xMin, pb.xMax);
        const dataY = r.toDataY(my, pb.yMin, pb.yMax);
        view.hoverIdx = view._nearestPathPoint(dataX, dataY);
        view.hoverMode = "path";
        view.hoverXY = { x: dataX, y: dataY };
        view.hoverTime = view.data.t()[view.hoverIdx];
        view.scheduleHover();
      }
    });

    canvas.addEventListener("mouseleave", () => {
      view.hoverIdx = null;
      view.hoverTime = null;
      view.hoverMode = null;
      view.hoverXY = null;
      view.scheduleHover();
    });

    canvas.addEventListener("mousedown", (e) => {
      dragging = true;
      dragStartX = e.clientX;
      dragStartY = e.clientY;
      dragStartPV = { ...(r._eqPathBounds || view.pathView) };
      canvas.style.cursor = "grabbing";
    });

    window.addEventListener("mouseup", () => {
      if (dragging) { dragging = false; canvas.style.cursor = ""; }
    });

    canvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      view._suppressMousemove();
      const pb = r._eqPathBounds || view.pathView;
      const rect = r.canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      const dataX = r.toDataX(mx, pb.xMin, pb.xMax);
      const dataY = r.toDataY(my, pb.yMin, pb.yMax);
      if (e.ctrlKey || e.metaKey) {
        const factor = Math.max(0.1, Math.min(10, Math.exp(e.deltaY * 0.01)));
        view.pathView.xMin = dataX - (dataX - pb.xMin) * factor;
        view.pathView.xMax = dataX + (pb.xMax - dataX) * factor;
        view.pathView.yMin = dataY - (dataY - pb.yMin) * factor;
        view.pathView.yMax = dataY + (pb.yMax - dataY) * factor;
      } else {
        const dppx = (pb.xMax - pb.xMin) / r.plotW;
        const dppy = (pb.yMax - pb.yMin) / r.plotH;
        view.pathView.xMin = pb.xMin + e.deltaX * dppx;
        view.pathView.xMax = pb.xMax + e.deltaX * dppx;
        view.pathView.yMin = pb.yMin - e.deltaY * dppy;
        view.pathView.yMax = pb.yMax - e.deltaY * dppy;
      }
      view.lastBoundsKey = "";
      view.scheduleFull();
    }, { passive: false });
  }

  // -- Variant (before/after) --------------------------------------------------
  // Swap the active dataset without touching pathView/timeView, so the user's
  // zoom and pan are preserved across the flip, and whatever the cursor is
  // anchored to (a time on the graphs, a point on the path) stays under it.
  setVariant(which) {
    if (which === this.variant) return;
    const next = which === "before" ? this.dataBefore : this.dataAfter;
    if (next == null) return;

    this.variant = which;
    this.data = next;
    if (this.hoverMode === "path" && this.hoverXY != null) {
      // Keep the marker on the same spot of the path; the time it is reached
      // there differs between variants, so re-derive it (and the crosshair).
      this.hoverIdx = this._nearestPathPoint(this.hoverXY.x, this.hoverXY.y);
      this.hoverTime = this.data.t()[this.hoverIdx];
    } else if (this.hoverMode === "time" && this.hoverTime != null) {
      // Keep the crosshair at the same time; the path marker re-derives.
      this.hoverIdx = closestIndex(this.data.t(), this.hoverTime);
    } else {
      this.hoverIdx = null;
    }

    this.onChanged?.();
    this.lastBoundsKey = "";
    this.renderAll();
  }

  toggleVariant() {
    if (this.dataBefore == null) return;
    this.setVariant(this.variant === "after" ? "before" : "after");
  }

  // -- Load a data pair --------------------------------------------------------
  // `after` and `before` are raw snapshot JSON objects; `before` may be null.
  // `keepView` preserves zoom/pan (playground re-plans on every config tweak;
  // resetting the view each keystroke would fight the user).
  setData(after, before, { keepView = false } = {}) {
    if (this.dataAfter && typeof this.dataAfter.free === "function") this.dataAfter.free();
    if (this.dataBefore && typeof this.dataBefore.free === "function") this.dataBefore.free();
    this.dataAfter = trajectoryFromSnapshot(after);
    this.dataBefore = before ? trajectoryFromSnapshot(before) : null;
    // Under keepView (playground live re-plan) a new plan must not kick the
    // user out of the comparison view: stay on "before" while it exists. A
    // fresh load (snapshot review case switch) always starts on "after".
    if (!keepView || this.variant !== "before" || this.dataBefore == null) {
      this.variant = "after";
    }
    this.data = this.variant === "before" ? this.dataBefore : this.dataAfter;

    const pb = computeDataBounds(this.data);
    Object.assign(this.defaultPathView, pb);
    const tb = computeTimeBounds(this.data);
    Object.assign(this.defaultTimeView, tb);
    if (!keepView) {
      Object.assign(this.pathView, pb);
      Object.assign(this.timeView, tb);
      this.hoverIdx = null;
      this.hoverTime = null;
      this.hoverMode = null;
      this.hoverXY = null;
    }

    this.onChanged?.();
    this.lastBoundsKey = "";
    this.renderAll();
    // Also queue a frame: layout may still be settling, and a ResizeObserver
    // firing after the synchronous paint clears the canvas bitmaps. The
    // queued render re-reads sizes after this task's layout flush, so the
    // panels cannot end up sized-but-blank.
    this.scheduleFull();
  }

  // Re-anchor the comparison snapshot without touching the current one — the
  // playground's "pin" action.
  setBaseline(before) {
    if (this.dataBefore && typeof this.dataBefore.free === "function") this.dataBefore.free();
    this.dataBefore = before ? trajectoryFromSnapshot(before) : null;
    if (this.variant === "before" && this.dataBefore == null) {
      this.variant = "after";
      this.data = this.dataAfter;
    }
    this.onChanged?.();
    this.lastBoundsKey = "";
    this.renderAll();
  }

  resetZoom() {
    Object.assign(this.pathView, this.defaultPathView);
    Object.assign(this.timeView, this.defaultTimeView);
    this.lastBoundsKey = "";
    this.hoverIdx = null;
    this.hoverTime = null;
    this.hoverMode = null;
    this.hoverXY = null;
    this.renderAll();
  }

  setShowPeaks(on) {
    this.showPeaks = on;
    this.lastBoundsKey = "";
    this.renderAll();
  }
}

// -- Resizable path/graphs split ---------------------------------------------
export function setupSplitter(storageKey) {
  const panels = document.querySelector(".panels");
  const splitter = document.getElementById("splitter");

  const saved = parseFloat(localStorage.getItem(storageKey));
  if (saved > 0 && saved < 1) {
    panels.style.setProperty("--path-w", (saved * 100) + "%");
  }

  let dragging = false;
  splitter.addEventListener("mousedown", (e) => {
    dragging = true;
    document.body.style.cursor = "col-resize";
    document.body.style.userSelect = "none";
    e.preventDefault();
  });
  window.addEventListener("mousemove", (e) => {
    if (!dragging) return;
    const rect = panels.getBoundingClientRect();
    const w = Math.max(220, Math.min(rect.width - 260, e.clientX - rect.left));
    panels.style.setProperty("--path-w", (w / rect.width * 100) + "%");
  });
  window.addEventListener("mouseup", () => {
    if (!dragging) return;
    dragging = false;
    document.body.style.cursor = "";
    document.body.style.userSelect = "";
    const frac = parseFloat(panels.style.getPropertyValue("--path-w")) / 100;
    if (frac > 0) localStorage.setItem(storageKey, frac.toFixed(4));
  });
}
