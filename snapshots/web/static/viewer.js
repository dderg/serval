import init, { TrajectoryData } from "/static/wasm/snapshot_viewer.js";

const params = new URLSearchParams(window.location.search);
const caseName = params.get("case");

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
  crosshair: "rgba(255,255,255,0.25)",
  highlight: "#fff",
};

// -- Panel configuration -----------------------------------------------------
const PANELS = [
  { canvasId: "canvas-path", type: "path" },
  { canvasId: "canvas-vel", type: "vel" },
  { canvasId: "canvas-acc", type: "acc" },
  { canvasId: "canvas-jrk", type: "jrk" },
];

// -- View state --------------------------------------------------------------
let timeView = { tMin: 0, tMax: 0 };
let pathView = { xMin: 0, xMax: 0, yMin: 0, yMax: 0 };
const defaultTimeView = { tMin: 0, tMax: 0 };
const defaultPathView = { xMin: 0, xMax: 0, yMin: 0, yMax: 0 };

// -- Data refs ----------------------------------------------------------------
let DATA = null; // TrajectoryData
let renderers = [];

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

// -- PanelRenderer -----------------------------------------------------------
class PanelRenderer {
  constructor(canvasId, type) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext("2d");
    this.type = type;
    this.margin = { top: 22, right: 14, bottom: 26, left: 56 };
    this._resize();
  }

  initObserver() {
    this._ro = new ResizeObserver(() => {
      this._resize();
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

  // Data → pixel
  toPixelX(dataVal, dataMin, dataMax) {
    return this.plotX0 + ((dataVal - dataMin) / (dataMax - dataMin)) * this.plotW;
  }
  toPixelY(dataVal, dataMin, dataMax) {
    return this.plotY0 + this.plotH - ((dataVal - dataMin) / (dataMax - dataMin)) * this.plotH;
  }

  // Pixel → data
  toDataX(pixelX, dataMin, dataMax) {
    return dataMin + ((pixelX - this.plotX0) / this.plotW) * (dataMax - dataMin);
  }
  toDataY(pixelY, dataMin, dataMax) {
    return dataMax - ((pixelY - this.plotY0) / this.plotH) * (dataMax - dataMin);
  }

  _drawGrid(xMin, xMax, yMin, yMax) {
    const ctx = this.ctx;
    const { plotX0, plotY0, plotW, plotH } = this;

    ctx.save();
    ctx.strokeStyle = COLORS.grid;
    ctx.lineWidth = 0.5;

    // X grid
    const xStep = niceStep(xMax - xMin, Math.max(2, Math.floor(plotW / 80)));
    const xStart = Math.ceil(xMin / xStep) * xStep;
    ctx.beginPath();
    for (let x = xStart; x <= xMax; x += xStep) {
      const px = this.toPixelX(x, xMin, xMax);
      ctx.moveTo(px, plotY0);
      ctx.lineTo(px, plotY0 + plotH);
    }
    ctx.stroke();

    // Y grid
    const yStep = niceStep(yMax - yMin, Math.max(2, Math.floor(plotH / 50)));
    const yStart = Math.ceil(yMin / yStep) * yStep;
    ctx.beginPath();
    for (let y = yStart; y <= yMax; y += yStep) {
      const py = this.toPixelY(y, yMin, yMax);
      ctx.moveTo(plotX0, py);
      ctx.lineTo(plotX0 + plotW, py);
    }
    ctx.stroke();

    // Axis labels
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

  clear() {
    this.ctx.clearRect(0, 0, this.w, this.h);
  }

  renderPath(xMin, xMax, yMin, yMax) {
    this.clear();
    const ctx = this.ctx;
    this._drawGrid(xMin, xMax, yMin, yMax);

    // Raw waypoints
    const rawX = DATA.raw_x();
    const rawY = DATA.raw_y();
    ctx.save();
    ctx.beginPath();
    ctx.strokeStyle = COLORS.raw;
    ctx.lineWidth = 0.8;
    for (let i = 0; i < rawX.length; i++) {
      const px = this.toPixelX(rawX[i], xMin, xMax);
      const py = this.toPixelY(rawY[i], yMin, yMax);
      i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
    }
    ctx.stroke();

    // Fitted segments
    const segCount = DATA.segment_count();
    for (let i = 0; i < segCount; i++) {
      const typ = DATA.segment_type(i);
      const d = DATA.segment_data(i);
      const color = COLORS[typ] || COLORS.line;
      ctx.beginPath();
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.2;
      if (typ === "line") {
        ctx.moveTo(this.toPixelX(d[0], xMin, xMax), this.toPixelY(d[1], yMin, yMax));
        ctx.lineTo(this.toPixelX(d[2], xMin, xMax), this.toPixelY(d[3], yMin, yMax));
      } else {
        for (let j = 0; j < d.length; j += 2) {
          const px = this.toPixelX(d[j], xMin, xMax);
          const py = this.toPixelY(d[j + 1], yMin, yMax);
          j === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
        }
      }
      ctx.stroke();
    }

    // Start dot
    if (rawX.length > 0) {
      ctx.beginPath();
      ctx.fillStyle = COLORS.scalar;
      ctx.arc(
        this.toPixelX(rawX[0], xMin, xMax),
        this.toPixelY(rawY[0], yMin, yMax),
        4, 0, Math.PI * 2
      );
      ctx.fill();
    }
    ctx.restore();
  }

  renderTimeSeries(tMin, tMax, yMin, yMax, yLabel, compX, compY, scalar) {
    this.clear();
    const ctx = this.ctx;
    this._drawGrid(tMin, tMax, yMin, yMax);

    const t = DATA.t();
    ctx.save();

    // Clip to plot area
    ctx.beginPath();
    ctx.rect(this.plotX0, this.plotY0, this.plotW, this.plotH);
    ctx.clip();

    // |X|
    ctx.strokeStyle = COLORS.vx;
    ctx.lineWidth = 0.6;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < t.length; i++) {
      if (t[i] < tMin || t[i] > tMax) continue;
      const px = this.toPixelX(t[i], tMin, tMax);
      const py = this.toPixelY(Math.abs(compX[i]), yMin, yMax);
      if (!started) { ctx.moveTo(px, py); started = true; }
      else ctx.lineTo(px, py);
    }
    ctx.stroke();

    // |Y|
    ctx.strokeStyle = COLORS.vy;
    ctx.lineWidth = 0.6;
    ctx.beginPath();
    started = false;
    for (let i = 0; i < t.length; i++) {
      if (t[i] < tMin || t[i] > tMax) continue;
      const px = this.toPixelX(t[i], tMin, tMax);
      const py = this.toPixelY(Math.abs(compY[i]), yMin, yMax);
      if (!started) { ctx.moveTo(px, py); started = true; }
      else ctx.lineTo(px, py);
    }
    ctx.stroke();

    // scalar
    ctx.strokeStyle = COLORS.scalar;
    ctx.lineWidth = 0.8;
    ctx.beginPath();
    started = false;
    for (let i = 0; i < t.length; i++) {
      if (t[i] < tMin || t[i] > tMax) continue;
      const px = this.toPixelX(t[i], tMin, tMax);
      const py = this.toPixelY(scalar[i], yMin, yMax);
      if (!started) { ctx.moveTo(px, py); started = true; }
      else ctx.lineTo(px, py);
    }
    ctx.stroke();

    ctx.restore();
  }

  drawCrosshair(dataT, tMin, tMax) {
    const px = this.toPixelX(dataT, tMin, tMax);
    if (px < this.plotX0 || px > this.plotX0 + this.plotW) return;
    const ctx = this.ctx;
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

// -- Helpers -----------------------------------------------------------------
function formatNum(v) {
  if (Math.abs(v) >= 1000) return v.toFixed(0);
  if (Math.abs(v) >= 1) return v.toFixed(1);
  if (Math.abs(v) >= 0.01) return v.toFixed(3);
  return v.toExponential(1);
}

function lerp(a, b, t) { return a + (b - a) * t; }

// Find index in sorted array closest to value
function closestIndex(arr, val) {
  let lo = 0, hi = arr.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (arr[mid] < val) lo = mid + 1;
    else hi = mid;
  }
  if (lo > 0) {
    const dLo = Math.abs(arr[lo] - val);
    const dHi = Math.abs(arr[lo - 1] - val);
    if (dHi < dLo) lo--;
  }
  return lo;
}

function computeDataBounds(data) {
  const rx = data.raw_x(), ry = data.raw_y();
  const kx = data.kin_x(), ky = data.kin_y();
  let xMin = Infinity, xMax = -Infinity, yMin = Infinity, yMax = -Infinity;
  const update = (arr, setter) => {
    for (let i = 0; i < arr.length; i++) {
      setter(arr[i]);
    }
  };
  update(rx, v => { if (v < xMin) xMin = v; if (v > xMax) xMax = v; });
  update(ry, v => { if (v < yMin) yMin = v; if (v > yMax) yMax = v; });
  update(kx, v => { if (v < xMin) xMin = v; if (v > xMax) xMax = v; });
  update(ky, v => { if (v < yMin) yMin = v; if (v > yMax) yMax = v; });
  // Add padding
  const padX = Math.max((xMax - xMin) * 0.08, 2);
  const padY = Math.max((yMax - yMin) * 0.08, 2);
  return { xMin: xMin - padX, xMax: xMax + padX, yMin: yMin - padY, yMax: yMax + padY };
}

function computeTimeBounds(data) {
  const t = data.t();
  const tMax = t.length > 0 ? t[t.length - 1] : 1;
  return { tMin: 0, tMax: tMax * 1.02 };
}

function computeYBounds(data, scalarArr) {
  let yMax = 0;
  for (let i = 0; i < scalarArr.length; i++) {
    if (scalarArr[i] > yMax) yMax = scalarArr[i];
  }
  return { yMin: 0, yMax: yMax * 1.15 || 1 };
}

function renderAll() {
  const { tMin, tMax } = timeView;
  const { xMin, xMax, yMin, yMax } = pathView;

  // Path panel
  renderers[0].clear();
  renderers[0].renderPath(xMin, xMax, yMin, yMax);

  // Time panels
  const t = DATA.t();
  const vx = DATA.vx(), vy = DATA.vy(), vScalar = DATA.v_scalar();
  const axD = DATA.ax(), ay = DATA.ay(), aScalar = DATA.a_scalar();
  const jx = DATA.jx(), jy = DATA.jy(), jScalar = DATA.j_scalar();

  // Compute Y bounds from visible range
  function visibleMax(arr) {
    let m = 0;
    for (let i = 0; i < t.length; i++) {
      if (t[i] >= tMin && t[i] <= tMax && arr[i] > m) m = arr[i];
    }
    return m * 1.15 || 1;
  }
  const vYMax = visibleMax(vScalar);
  const aYMax = visibleMax(aScalar);
  const jYMax = visibleMax(jScalar);

  renderers[1].clear();
  renderers[1].renderTimeSeries(tMin, tMax, 0, vYMax, "mm/s", vx, vy, vScalar);
  renderers[2].clear();
  renderers[2].renderTimeSeries(tMin, tMax, 0, aYMax, "mm/s²", axD, ay, aScalar);
  renderers[3].clear();
  renderers[3].renderTimeSeries(tMin, tMax, 0, jYMax, "mm/s³", jx, jy, jScalar);
}

function drawCrosshairs(dataT) {
  const { tMin, tMax } = timeView;
  // Redraw panels then overlay crosshair
  renderAll();
  for (const r of renderers) {
    r.drawCrosshair(dataT, tMin, tMax);
  }
  // Highlight on path
  highlightPathPoint(dataT);
}

function highlightPathPoint(dataT) {
  const t = DATA.t();
  const kx = DATA.kin_x(), ky = DATA.kin_y();
  const idx = closestIndex(t, dataT);
  if (idx >= kx.length) return;
  const r = renderers[0];
  const ctx = r.ctx;
  const px = r.toPixelX(kx[idx], pathView.xMin, pathView.xMax);
  const py = r.toPixelY(ky[idx], pathView.yMin, pathView.yMax);
  ctx.save();
  ctx.beginPath();
  ctx.fillStyle = COLORS.highlight;
  ctx.strokeStyle = COLORS.scalar;
  ctx.lineWidth = 2;
  ctx.arc(px, py, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

// -- Tooltip -----------------------------------------------------------------
const tooltipEl = document.getElementById("tooltip");

function showTooltip(e, dataT) {
  const t = DATA.t();
  const idx = closestIndex(t, dataT);
  const vx = DATA.vx()[idx];
  const vy = DATA.vy()[idx];
  const v = DATA.v_scalar()[idx];
  const a = DATA.a_scalar()[idx];
  const j = DATA.j_scalar()[idx];
  const kx = DATA.kin_x()[idx];
  const ky = DATA.kin_y()[idx];

  tooltipEl.style.display = "block";
  tooltipEl.style.left = (e.clientX + 14) + "px";
  tooltipEl.style.top = (e.clientY - 10) + "px";
  tooltipEl.textContent =
    `t=${formatNum(dataT)}s\n` +
    `X=${formatNum(kx)} Y=${formatNum(ky)}\n` +
    `v=${formatNum(v)} mm/s\n` +
    `|vx|=${formatNum(Math.abs(vx))} |vy|=${formatNum(Math.abs(vy))}\n` +
    `a=${formatNum(a)} mm/s²\n` +
    `j=${formatNum(j)} mm/s³`;
}

function hideTooltip() {
  tooltipEl.style.display = "none";
}

// -- Interaction (time panels) -----------------------------------------------
function setupTimeInteraction(panelIdx) {
  const canvas = renderers[panelIdx].canvas;
  let dragging = false;
  let dragStartX = 0;
  let dragStartTMin = 0;
  let dragStartTMax = 0;

  canvas.addEventListener("mousemove", (e) => {
    const r = renderers[panelIdx];
    const rect = r.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const dataT = r.toDataX(mx, timeView.tMin, timeView.tMax);

    if (dragging) {
      const dx = e.clientX - dragStartX;
      const dtPerPx = (dragStartTMax - dragStartTMin) / r.plotW;
      const dt = -dx * dtPerPx;
      timeView.tMin = dragStartTMin + dt;
      timeView.tMax = dragStartTMax + dt;
      renderAll();
    } else {
      drawCrosshairs(dataT);
    }
    showTooltip(e, dataT);
  });

  canvas.addEventListener("mouseleave", () => {
    hideTooltip();
    if (!dragging) renderAll();
  });

  canvas.addEventListener("mousedown", (e) => {
    dragging = true;
    dragStartX = e.clientX;
    dragStartTMin = timeView.tMin;
    dragStartTMax = timeView.tMax;
    canvas.style.cursor = "grabbing";
  });

  window.addEventListener("mouseup", () => {
    if (dragging) {
      dragging = false;
      canvas.style.cursor = "";
    }
  });

  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    const r = renderers[panelIdx];
    const rect = r.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const tAtCursor = r.toDataX(mx, timeView.tMin, timeView.tMax);
    const factor = e.deltaY > 0 ? 1.12 : 1 / 1.12;
    timeView.tMin = tAtCursor - (tAtCursor - timeView.tMin) * factor;
    timeView.tMax = tAtCursor + (timeView.tMax - tAtCursor) * factor;
    renderAll();
  }, { passive: false });
}

// -- Interaction (path panel) ------------------------------------------------
function setupPathInteraction() {
  const canvas = renderers[0].canvas;
  let dragging = false;
  let dragStartX = 0, dragStartY = 0;
  let dragStartPV = null;

  canvas.addEventListener("mousemove", (e) => {
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
      renderAll();
    }

    // Show coordinates in tooltip
    const dataX = r.toDataX(mx, pathView.xMin, pathView.xMax);
    const dataY = r.toDataY(my, pathView.yMin, pathView.yMax);
    tooltipEl.style.display = "block";
    tooltipEl.style.left = (e.clientX + 14) + "px";
    tooltipEl.style.top = (e.clientY - 10) + "px";
    tooltipEl.textContent = `X=${formatNum(dataX)} Y=${formatNum(dataY)}`;
  });

  canvas.addEventListener("mouseleave", () => {
    hideTooltip();
    if (!dragging) renderAll();
  });

  canvas.addEventListener("mousedown", (e) => {
    dragging = true;
    dragStartX = e.clientX;
    dragStartY = e.clientY;
    dragStartPV = { ...pathView };
    canvas.style.cursor = "grabbing";
  });

  window.addEventListener("mouseup", () => {
    if (dragging) {
      dragging = false;
      canvas.style.cursor = "";
    }
  });

  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    const r = renderers[0];
    const rect = r.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    const dataX = r.toDataX(mx, pathView.xMin, pathView.xMax);
    const dataY = r.toDataY(my, pathView.yMin, pathView.yMax);
    const factor = e.deltaY > 0 ? 1.12 : 1 / 1.12;
    pathView.xMin = dataX - (dataX - pathView.xMin) * factor;
    pathView.xMax = dataX + (pathView.xMax - dataX) * factor;
    pathView.yMin = dataY - (dataY - pathView.yMin) * factor;
    pathView.yMax = dataY + (pathView.yMax - dataY) * factor;
    renderAll();
  }, { passive: false });
}

// -- Init --------------------------------------------------------------------
async function main() {
  if (!caseName) {
    document.getElementById("case-name").textContent = "No case specified — add ?case=name to URL";
    return;
  }

  await init();

  const resp = await fetch(`/snapshot-data/${encodeURIComponent(caseName)}`);
  if (!resp.ok) {
    document.getElementById("case-name").textContent = `Error: ${resp.statusText}`;
    return;
  }
  const snapshot = await resp.json();
  DATA = new TrajectoryData(JSON.stringify(snapshot));

  document.getElementById("case-name").textContent = caseName;
  document.getElementById("meta").textContent =
    `t=${DATA.traversal_time().toFixed(3)}s  ` +
    `${DATA.blended_corners()} blended, ${DATA.chain_fits()} chains, ` +
    `${DATA.point_count()} pts`;

  // Initialize renderers
  renderers = PANELS.map(p => new PanelRenderer(p.canvasId, p.type));
  renderers.forEach(r => r.initObserver());

  // Compute default views
  const pb = computeDataBounds(DATA);
  Object.assign(defaultPathView, pb);
  Object.assign(pathView, pb);

  const tb = computeTimeBounds(DATA);
  Object.assign(defaultTimeView, tb);
  Object.assign(timeView, tb);

  // Wire interactions
  setupPathInteraction();
  for (let i = 1; i < renderers.length; i++) {
    setupTimeInteraction(i);
  }

  // Reset zoom
  document.getElementById("reset-zoom").addEventListener("click", () => {
    Object.assign(pathView, defaultPathView);
    Object.assign(timeView, defaultTimeView);
    renderAll();
  });

  renderAll();
}

main();
