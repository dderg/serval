import init, { TrajectoryData } from "/static/wasm/snapshot_viewer.js";

const params = new URLSearchParams(window.location.search);
let currentCase = params.get("case");
let caseList = []; // [{ name, ... }] — the switchable set, from /api/cases

// -- Colors ------------------------------------------------------------------
const COLORS = {
  raw: "#555",
  line: "#4a9eff",
  arc: "#4ecb71",
  clothoid: "#f5a623",
  vx: "#4a9eff",
  vy: "#4ecb71",
  scalar: "#ef5350",
  grid: "#22252b",
  axis: "#555",
  crosshair: "rgba(255,255,255,0.35)",
  marker: "#e0518a",
};

// -- Panel configuration -----------------------------------------------------
const PANELS = [
  { canvasId: "canvas-path", type: "path" },
  { canvasId: "canvas-vel", type: "vel", scalarKey: "v_scalar", compXKey: "vx", compYKey: "vy" },
  { canvasId: "canvas-acc", type: "acc", scalarKey: "a_scalar", compXKey: "ax", compYKey: "ay" },
  { canvasId: "canvas-jrk", type: "jrk", scalarKey: "j_scalar", compXKey: "jx", compYKey: "jy" },
];

// -- View state --------------------------------------------------------------
let timeView = { tMin: 0, tMax: 0 };
let pathView = { xMin: 0, xMax: 0, yMin: 0, yMax: 0 };
const defaultTimeView = { tMin: 0, tMax: 0 };
const defaultPathView = { xMin: 0, xMax: 0, yMin: 0, yMax: 0 };

// -- Data refs ---------------------------------------------------------------
let DATA = null;
let dataAfter = null; // current trajectory
let dataBefore = null; // committed baseline, null when there is nothing to compare
let variant = "after"; // which of the two DATA currently points at
let renderers = [];
let lastBoundsKey = "";
let hoverIdx = null;
let hoverTime = null; // time the marker sits at; drives the graph crosshair
// What the cursor is anchored to, so a before/after flip keeps it under the
// pointer. "time": graphs own the cursor (fixed time, path marker re-derives).
// "path": the path owns it (fixed xy, the time it is reached re-derives).
let hoverMode = null; // "time" | "path" | null
let hoverXY = null; // { x, y } cursor position when hoverMode === "path"
let showPeaks = false;
let wheelTimer = null; // suppresses mousemove during trackpad gestures
const tooltipEl = document.getElementById("tooltip");

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
// Suppress mousemove during trackpad wheel gestures (macOS collision)
function suppressMousemove() {
  clearTimeout(wheelTimer);
  wheelTimer = setTimeout(() => { wheelTimer = null; }, 80);
}

function formatNum(v) {
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

function nearestPathPoint(dataX, dataY) {
  const kx = DATA.kin_x(), ky = DATA.kin_y();
  let best = 0, bd = Infinity;
  for (let i = 0; i < kx.length; i++) {
    const dx = kx[i] - dataX, dy = ky[i] - dataY;
    const d = dx * dx + dy * dy;
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

function findPeaks(scalar, yMax) {
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

// -- PanelRenderer -----------------------------------------------------------
class PanelRenderer {
  constructor(canvasId, type) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext("2d");
    this.type = type;
    this.margin = { top: 22, right: 14, bottom: 26, left: 56 };
    this._buf = null;
    this._peaks = [];
    this._resize();
  }

  initObserver() {
    this._ro = new ResizeObserver(() => {
      this._resize();
      lastBoundsKey = "";
      renderAll();
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
    this.plotW = this.w - this.margin.left - this.margin.right;
    this.plotH = this.h - this.margin.top - this.margin.bottom;
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

  _ensureBuf() {
    if (!this._buf) this._buf = document.createElement("canvas");
    const dpr = window.devicePixelRatio || 1;
    if (this._buf.width !== this.canvas.width || this._buf.height !== this.canvas.height) {
      this._buf.width = this.canvas.width;
      this._buf.height = this.canvas.height;
    }
    const bctx = this._buf.getContext("2d");
    bctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return bctx;
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
    const eq = this._equalizePathBounds(xMin, xMax, yMin, yMax);
    xMin = eq.xMin; xMax = eq.xMax; yMin = eq.yMin; yMax = eq.yMax;
    this._eqPathBounds = eq; // store for compositePath

    const bctx = this._ensureBuf();
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

    const segCount = DATA.segment_count();
    for (let i = 0; i < segCount; i++) {
      const typ = DATA.segment_type(i);
      const d = DATA.segment_data(i);
      const color = COLORS[typ] || COLORS.line;
      bctx.beginPath();
      bctx.strokeStyle = color;
      bctx.lineWidth = 1.2;
      if (typ === "line") {
        bctx.moveTo(this.toPixelX(d[0], xMin, xMax), this.toPixelY(d[1], yMin, yMax));
        bctx.lineTo(this.toPixelX(d[2], xMin, xMax), this.toPixelY(d[3], yMin, yMax));
      } else {
        for (let j = 0; j < d.length; j += 2) {
          const px = this.toPixelX(d[j], xMin, xMax);
          const py = this.toPixelY(d[j + 1], yMin, yMax);
          j === 0 ? bctx.moveTo(px, py) : bctx.lineTo(px, py);
        }
      }
      bctx.stroke();
    }

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

  // -- Render time-series panel to buffer ------------------------------------
  renderTimeBuffer(tMin, tMax, yMin, yMax, compX, compY, scalar, drawPeaks) {
    const bctx = this._ensureBuf();
    bctx.clearRect(0, 0, this.w, this.h);
    this._drawGrid(bctx, tMin, tMax, yMin, yMax);

    const t = DATA.t();
    bctx.save();
    bctx.beginPath();
    bctx.rect(this.plotX0, this.plotY0, this.plotW, this.plotH);
    bctx.clip();

    // |X|
    bctx.strokeStyle = COLORS.vx;
    bctx.lineWidth = 0.6;
    bctx.beginPath();
    let started = false;
    for (let i = 0; i < t.length; i++) {
      if (t[i] < tMin || t[i] > tMax) continue;
      const px = this.toPixelX(t[i], tMin, tMax);
      const py = this.toPixelY(Math.abs(compX[i]), yMin, yMax);
      if (!started) { bctx.moveTo(px, py); started = true; }
      else bctx.lineTo(px, py);
    }
    bctx.stroke();

    // |Y|
    bctx.strokeStyle = COLORS.vy;
    bctx.lineWidth = 0.6;
    bctx.beginPath();
    started = false;
    for (let i = 0; i < t.length; i++) {
      if (t[i] < tMin || t[i] > tMax) continue;
      const px = this.toPixelX(t[i], tMin, tMax);
      const py = this.toPixelY(Math.abs(compY[i]), yMin, yMax);
      if (!started) { bctx.moveTo(px, py); started = true; }
      else bctx.lineTo(px, py);
    }
    bctx.stroke();

    // scalar
    bctx.strokeStyle = COLORS.scalar;
    bctx.lineWidth = 0.8;
    bctx.beginPath();
    started = false;
    for (let i = 0; i < t.length; i++) {
      if (t[i] < tMin || t[i] > tMax) continue;
      const px = this.toPixelX(t[i], tMin, tMax);
      const py = this.toPixelY(scalar[i], yMin, yMax);
      if (!started) { bctx.moveTo(px, py); started = true; }
      else bctx.lineTo(px, py);
    }
    bctx.stroke();

    // Peak markers
    this._peaks = [];
    if (drawPeaks) {
      const peaks = findPeaks(scalar, yMax);
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

    bctx.restore();
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

  // -- Fast composite: blit buffer + cursor overlay --------------------------
  composite(tVal, tMin, tMax, showCursor) {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.w, this.h);
    if (this._buf) {
      ctx.drawImage(this._buf, 0, 0, this.w, this.h);
    }
    if (showCursor && tVal != null && this.type !== "path") {
      const px = this.toPixelX(tVal, tMin, tMax);
      if (px >= this.plotX0 && px <= this.plotX0 + this.plotW) {
        ctx.save();
        ctx.strokeStyle = COLORS.crosshair;
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.moveTo(px, this.plotY0);
        ctx.lineTo(px, this.plotY0 + this.plotH);
        ctx.stroke();
        ctx.restore();
      }
    }
  }

  compositePath(idx) {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.w, this.h);
    if (this._buf) {
      ctx.drawImage(this._buf, 0, 0, this.w, this.h);
    }
    if (idx == null) return;
    const kx = DATA.kin_x(), ky = DATA.kin_y();
    if (idx >= kx.length) return;
    const pb = this._eqPathBounds || pathView;
    const px = this.toPixelX(kx[idx], pb.xMin, pb.xMax);
    const py = this.toPixelY(ky[idx], pb.yMin, pb.yMax);

    // Crosshair lines on path
    ctx.save();
    ctx.strokeStyle = COLORS.crosshair;
    ctx.lineWidth = 0.7;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(px, this.plotY0);
    ctx.lineTo(px, this.plotY0 + this.plotH);
    ctx.moveTo(this.plotX0, py);
    ctx.lineTo(this.plotX0 + this.plotW, py);
    ctx.stroke();
    ctx.restore();

    // Marker dot
    ctx.save();
    ctx.beginPath();
    ctx.fillStyle = COLORS.marker;
    ctx.strokeStyle = "#14161a";
    ctx.lineWidth = 2;
    ctx.arc(px, py, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
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

// Per-type fitted-segment counts, e.g. "3 arc, 5 line" — mirrors the PNG title.
function segmentSummary() {
  const counts = {};
  const n = DATA.segment_count();
  for (let i = 0; i < n; i++) {
    const typ = DATA.segment_type(i);
    counts[typ] = (counts[typ] || 0) + 1;
  }
  return Object.keys(counts).sort().map(k => `${counts[k]} ${k}`).join(", ");
}

// -- Readout -----------------------------------------------------------------
function updateReadout(idx) {
  const el = document.getElementById("readout");
  if (idx == null) {
    el.innerHTML = '<span class="g">hover a panel to scrub</span>';
    return;
  }
  const t = DATA.t()[idx];
  const kx = DATA.kin_x()[idx], ky = DATA.kin_y()[idx];
  const v = DATA.v_scalar()[idx];
  const a = DATA.a_scalar()[idx];
  const j = DATA.j_scalar()[idx];

  el.innerHTML =
    `<span class="g">t=${formatNum(t)}s</span>` +
    `<span class="v">X=${formatNum(kx)}</span>` +
    `<span class="p">Y=${formatNum(ky)}</span>` +
    `<span class="g">|v|=${formatNum(v)}</span>` +
    `<span class="r">a=${formatNum(a)}</span>` +
    `<span class="g">j=${formatNum(j)}</span>`;
}

// -- Synced hover ------------------------------------------------------------
function syncHover(idx) {
  hoverIdx = idx;
  const t = DATA.t();
  const dataT = hoverTime != null ? hoverTime : idx != null ? t[idx] : null;
  const { tMin, tMax } = timeView;

  // Composite all time panels (fast — just blit + cursor)
  for (let i = 1; i < renderers.length; i++) {
    renderers[i].composite(dataT, tMin, tMax, idx != null);
  }

  // Composite path with marker
  renderers[0].compositePath(idx);

  updateReadout(idx);
}

// -- Render all buffers (only when bounds change) ----------------------------
function renderAll() {
  if (!DATA) return;
  const { tMin, tMax } = timeView;
  const { xMin, xMax, yMin, yMax } = pathView;

  const vScalar = DATA.v_scalar();
  const aScalar = DATA.a_scalar();
  const jScalar = DATA.j_scalar();

  // Scale across both variants so blinking before/after keeps a fixed axis —
  // a peak that shrinks must visibly drop, not get renormalized to the top.
  function visibleMaxOf(data, scalarKey) {
    const dt = data.t();
    const arr = data[scalarKey]();
    let m = 0;
    for (let i = 0; i < dt.length; i++) {
      if (dt[i] >= tMin && dt[i] <= tMax && arr[i] > m) m = arr[i];
    }
    return m;
  }
  function visibleMax(scalarKey) {
    let m = visibleMaxOf(dataAfter, scalarKey);
    if (dataBefore) m = Math.max(m, visibleMaxOf(dataBefore, scalarKey));
    return m * 1.15 || 1;
  }
  const vYMax = visibleMax("v_scalar");
  const aYMax = visibleMax("a_scalar");
  const jYMax = visibleMax("j_scalar");

  const key = `${tMin},${tMax},${xMin},${xMax},${yMin},${yMax},${vYMax},${aYMax},${jYMax}`;
  if (key !== lastBoundsKey) {
    lastBoundsKey = key;

    renderers[0].renderPathBuffer(xMin, xMax, yMin, yMax);
    renderers[1].renderTimeBuffer(tMin, tMax, 0, vYMax, DATA.vx(), DATA.vy(), vScalar, showPeaks);
    renderers[2].renderTimeBuffer(tMin, tMax, 0, aYMax, DATA.ax(), DATA.ay(), aScalar, showPeaks);
    renderers[3].renderTimeBuffer(tMin, tMax, 0, jYMax, DATA.jx(), DATA.jy(), jScalar, showPeaks);
  }

  // Composite all (always — cursor may have moved)
  if (hoverIdx != null) {
    syncHover(hoverIdx);
  } else {
    for (let i = 1; i < renderers.length; i++) {
      renderers[i].composite(null, tMin, tMax, false);
    }
    renderers[0].compositePath(null);
  }
}

// -- Interaction (time panels) -----------------------------------------------
function setupTimeInteraction(panelIdx) {
  const canvas = renderers[panelIdx].canvas;
  let dragging = false;
  let dragStartX = 0, dragStartTMin = 0, dragStartTMax = 0;

  canvas.addEventListener("mousemove", (e) => {
    if (wheelTimer) return;
    const r = renderers[panelIdx];
    const rect = r.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    if (dragging) {
      const dx = e.clientX - dragStartX;
      const dtPerPx = (dragStartTMax - dragStartTMin) / r.plotW;
      const dt = -dx * dtPerPx;
      timeView.tMin = dragStartTMin + dt;
      timeView.tMax = dragStartTMax + dt;
      lastBoundsKey = "";
      renderAll();
    } else {
      const dataT = r.toDataX(mx, timeView.tMin, timeView.tMax);
      hoverMode = "time";
      hoverTime = dataT;
      const idx = closestIndex(DATA.t(), dataT);
      syncHover(idx);

      // Check for nearby peak
      const peak = r.nearestPeak(mx, my, 12);
      if (peak) {
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
    hoverIdx = null;
    hoverTime = null;
    hoverMode = null;
    hoverXY = null;
    tooltipEl.style.display = "none";
    renderAll();
  });

  canvas.addEventListener("mousedown", (e) => {
    dragging = true;
    dragStartX = e.clientX;
    dragStartTMin = timeView.tMin;
    dragStartTMax = timeView.tMax;
    canvas.style.cursor = "grabbing";
  });

  window.addEventListener("mouseup", () => {
    if (dragging) { dragging = false; canvas.style.cursor = ""; }
  });

  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    suppressMousemove();
    const r = renderers[panelIdx];
    const rect = r.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const tAtCursor = r.toDataX(mx, timeView.tMin, timeView.tMax);

    if (e.ctrlKey || e.metaKey) {
      const factor = Math.max(0.1, Math.min(10, Math.exp(e.deltaY * 0.01)));
      timeView.tMin = tAtCursor - (tAtCursor - timeView.tMin) * factor;
      timeView.tMax = tAtCursor + (timeView.tMax - tAtCursor) * factor;
    } else {
      const dtPerPx = (timeView.tMax - timeView.tMin) / r.plotW;
      const dx = e.deltaX !== 0 ? e.deltaX : e.deltaY;
      const dt = dx * dtPerPx;
      timeView.tMin += dt;
      timeView.tMax += dt;
    }
    lastBoundsKey = "";
    renderAll();
  }, { passive: false });
}

// -- Interaction (path panel) ------------------------------------------------
function setupPathInteraction() {
  const canvas = renderers[0].canvas;
  let dragging = false;
  let dragStartX = 0, dragStartY = 0, dragStartPV = null;

  canvas.addEventListener("mousemove", (e) => {
    if (wheelTimer) return;
    const r = renderers[0];
    const rect = r.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    if (dragging) {
      const dx = e.clientX - dragStartX;
      const dy = e.clientY - dragStartY;
      const dppx = (dragStartPV.xMax - dragStartPV.xMin) / r.plotW;
      const dppy = (dragStartPV.yMax - dragStartPV.yMin) / r.plotH;
      pathView.xMin = dragStartPV.xMin - dx * dppx;
      pathView.xMax = dragStartPV.xMax - dx * dppx;
      pathView.yMin = dragStartPV.yMin + dy * dppy;
      pathView.yMax = dragStartPV.yMax + dy * dppy;
      lastBoundsKey = "";
      renderAll();
    } else {
      const pb = r._eqPathBounds || pathView;
      const dataX = r.toDataX(mx, pb.xMin, pb.xMax);
      const dataY = r.toDataY(my, pb.yMin, pb.yMax);
      const idx = nearestPathPoint(dataX, dataY);
      hoverMode = "path";
      hoverXY = { x: dataX, y: dataY };
      hoverTime = DATA.t()[idx];
      syncHover(idx);
    }
  });

  canvas.addEventListener("mouseleave", () => {
    hoverIdx = null;
    hoverTime = null;
    hoverMode = null;
    hoverXY = null;
    renderAll();
  });

  canvas.addEventListener("mousedown", (e) => {
    dragging = true;
    dragStartX = e.clientX;
    dragStartY = e.clientY;
    dragStartPV = { ...(renderers[0]._eqPathBounds || pathView) };
    canvas.style.cursor = "grabbing";
  });

  window.addEventListener("mouseup", () => {
    if (dragging) { dragging = false; canvas.style.cursor = ""; }
  });

  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    suppressMousemove();
    const r = renderers[0];
    const pb = r._eqPathBounds || pathView;
    const rect = r.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    const dataX = r.toDataX(mx, pb.xMin, pb.xMax);
    const dataY = r.toDataY(my, pb.yMin, pb.yMax);
    if (e.ctrlKey || e.metaKey) {
      const factor = Math.max(0.1, Math.min(10, Math.exp(e.deltaY * 0.01)));
      pathView.xMin = dataX - (dataX - pb.xMin) * factor;
      pathView.xMax = dataX + (pb.xMax - dataX) * factor;
      pathView.yMin = dataY - (dataY - pb.yMin) * factor;
      pathView.yMax = dataY + (pb.yMax - dataY) * factor;
    } else {
      const dppx = (pb.xMax - pb.xMin) / r.plotW;
      const dppy = (pb.yMax - pb.yMin) / r.plotH;
      pathView.xMin = pb.xMin + e.deltaX * dppx;
      pathView.xMax = pb.xMax + e.deltaX * dppx;
      pathView.yMin = pb.yMin - e.deltaY * dppy;
      pathView.yMax = pb.yMax - e.deltaY * dppy;
    }
    lastBoundsKey = "";
    renderAll();
  }, { passive: false });
}

// -- Case switching ----------------------------------------------------------
function caseIndex() {
  return caseList.findIndex(c => c.name === currentCase);
}

function syncCaseControls() {
  const sel = document.getElementById("case-select");
  if (sel.value !== currentCase) sel.value = currentCase;
  const i = caseIndex();
  document.getElementById("case-prev").disabled = i <= 0;
  document.getElementById("case-next").disabled = i < 0 || i >= caseList.length - 1;
}

function stepCase(dir) {
  const i = caseIndex();
  if (i < 0) return;
  const next = i + dir;
  if (next < 0 || next >= caseList.length) return;
  loadCase(caseList[next].name);
}

async function loadCaseList() {
  let review = [];
  try {
    const data = await fetch("/api/cases").then(r => r.json());
    review = data.review || [];
  } catch (e) { /* offline / no server scan — fall back to single case */ }
  caseList = review;
  if (currentCase && !caseList.some(c => c.name === currentCase)) {
    caseList.push({ name: currentCase });
  }
  if (!currentCase && caseList.length > 0) currentCase = caseList[0].name;

  const sel = document.getElementById("case-select");
  sel.innerHTML = "";
  for (const c of caseList) {
    const opt = document.createElement("option");
    opt.value = c.name;
    opt.textContent = c.name;
    sel.appendChild(opt);
  }
  sel.addEventListener("change", () => loadCase(sel.value));
  document.getElementById("case-prev").addEventListener("click", () => stepCase(-1));
  document.getElementById("case-next").addEventListener("click", () => stepCase(1));
}

// -- Variant (before/after) --------------------------------------------------
function updateMeta() {
  document.getElementById("meta").textContent =
    `t=${DATA.traversal_time().toFixed(3)}s  ` +
    `[${segmentSummary()}]  ` +
    `${DATA.blended_corners()} blended, ${DATA.chain_fits()} chains, ` +
    `${DATA.point_count()} pts`;
}

function syncVariantControls() {
  const btn = document.getElementById("toggle-variant");
  const hasBefore = dataBefore != null;
  btn.disabled = !hasBefore;
  btn.classList.toggle("after", hasBefore && variant === "after");
  btn.classList.toggle("before", hasBefore && variant === "before");
  if (!hasBefore) {
    btn.textContent = "After";
    btn.title = "No baseline to compare against";
  } else {
    btn.textContent = variant === "before" ? "Before" : "After";
    btn.title = "Compare before/after (space)";
  }
}

// Swap the active dataset without touching pathView/timeView, so the user's
// zoom and pan are preserved across the flip, and whatever the cursor is
// anchored to (a time on the graphs, a point on the path) stays under it.
function setVariant(which) {
  if (which === variant) return;
  const next = which === "before" ? dataBefore : dataAfter;
  if (next == null) return;

  variant = which;
  DATA = next;
  if (hoverMode === "path" && hoverXY != null) {
    // Keep the marker on the same spot of the path; the time it is reached
    // there differs between variants, so re-derive it (and the crosshair).
    hoverIdx = nearestPathPoint(hoverXY.x, hoverXY.y);
    hoverTime = DATA.t()[hoverIdx];
  } else if (hoverMode === "time" && hoverTime != null) {
    // Keep the crosshair at the same time; the path marker re-derives.
    hoverIdx = closestIndex(DATA.t(), hoverTime);
  } else {
    hoverIdx = null;
  }

  updateMeta();
  syncVariantControls();
  lastBoundsKey = "";
  renderAll();
}

function toggleVariant() {
  if (dataBefore == null) return;
  setVariant(variant === "after" ? "before" : "after");
}

async function fetchSnapshot(name, which) {
  const resp = await fetch(
    `/snapshot-data/${encodeURIComponent(name)}?which=${which}`
  );
  if (!resp.ok) return null;
  return resp.json();
}

// -- Load a single case into the graphs --------------------------------------
async function loadCase(name) {
  currentCase = name;
  const url = new URL(window.location);
  url.searchParams.set("case", name);
  history.replaceState(null, "", url);
  syncCaseControls();
  document.title = `Snapshot — ${name}`;

  const after = await fetchSnapshot(name, "after");
  if (after == null) {
    document.getElementById("meta").textContent = "Error: failed to load case";
    return;
  }
  const before = await fetchSnapshot(name, "before");

  if (dataAfter && typeof dataAfter.free === "function") dataAfter.free();
  if (dataBefore && typeof dataBefore.free === "function") dataBefore.free();
  dataAfter = new TrajectoryData(JSON.stringify(after));
  dataBefore = before ? new TrajectoryData(JSON.stringify(before)) : null;
  variant = "after";
  DATA = dataAfter;

  updateMeta();
  syncVariantControls();

  const pb = computeDataBounds(DATA);
  Object.assign(defaultPathView, pb);
  Object.assign(pathView, pb);

  const tb = computeTimeBounds(DATA);
  Object.assign(defaultTimeView, tb);
  Object.assign(timeView, tb);

  hoverIdx = null;
  hoverTime = null;
  hoverMode = null;
  hoverXY = null;
  lastBoundsKey = "";
  renderAll();
}

// -- Resizable path/graphs split ---------------------------------------------
const SPLIT_KEY = "snapshotViewer.pathSplit";

function setupSplitter() {
  const panels = document.querySelector(".panels");
  const splitter = document.getElementById("splitter");

  const saved = parseFloat(localStorage.getItem(SPLIT_KEY));
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
    if (frac > 0) localStorage.setItem(SPLIT_KEY, frac.toFixed(4));
  });
}

// -- PNG popup ---------------------------------------------------------------
function openPng() {
  if (!currentCase) return;
  const scroll = document.getElementById("png-scroll");
  scroll.innerHTML = "";
  const img = new Image();
  img.src = `/img/${encodeURIComponent(currentCase)}/after.png?t=${Date.now()}`;
  scroll.appendChild(img);
  scroll.scrollTop = 0;
  document.getElementById("png-overlay").classList.add("open");
}

function closePng() {
  document.getElementById("png-overlay").classList.remove("open");
}

function pngOpen() {
  return document.getElementById("png-overlay").classList.contains("open");
}

// -- Init --------------------------------------------------------------------
async function main() {
  await init();
  await loadCaseList();

  if (!currentCase) {
    document.getElementById("meta").textContent = "No case specified — add ?case=name to URL";
    return;
  }

  renderers = PANELS.map(p => new PanelRenderer(p.canvasId, p.type));
  renderers.forEach(r => r.initObserver());

  setupPathInteraction();
  for (let i = 1; i < renderers.length; i++) {
    setupTimeInteraction(i);
  }
  setupSplitter();

  document.getElementById("reset-zoom").addEventListener("click", () => {
    Object.assign(pathView, defaultPathView);
    Object.assign(timeView, defaultTimeView);
    lastBoundsKey = "";
    hoverIdx = null;
    hoverTime = null;
    hoverMode = null;
    hoverXY = null;
    renderAll();
  });

  document.getElementById("toggle-peaks").addEventListener("click", (e) => {
    showPeaks = !showPeaks;
    e.target.classList.toggle("active", showPeaks);
    lastBoundsKey = "";
    renderAll();
  });

  document.getElementById("toggle-variant").addEventListener("click", toggleVariant);
  document.getElementById("open-png").addEventListener("click", openPng);
  document.getElementById("png-overlay").addEventListener("click", closePng);

  document.addEventListener("keydown", (e) => {
    if (pngOpen()) {
      if (e.key === "Escape") closePng();
      return;
    }
    if (e.key === "ArrowLeft") stepCase(-1);
    else if (e.key === "ArrowRight") stepCase(1);
    else if (e.key === " " || e.key === "b" || e.key === "B") {
      e.preventDefault();
      toggleVariant();
    }
  });

  await loadCase(currentCase);
}

main();
