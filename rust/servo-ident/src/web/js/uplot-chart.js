import uPlot from "../vendor/uPlot-1.6.32.esm.js";

const THEME = {
  bg: "#0d1117",
  grid: "#29313a",
  axisText: "#8a97a3",
  cursor: "#4a5560",
  readoutText: "#e6edf3",
  font: "10px monospace",
  readoutFont: "11px monospace",
};

const UPLOT_CSS_HREF = "/vendor/uPlot-1.6.32.min.css";

function ensureUplotCss() {
  if (document.querySelector(`link[href="${UPLOT_CSS_HREF}"]`)) return;
  const link = document.createElement("link");
  link.rel = "stylesheet";
  link.href = UPLOT_CSS_HREF;
  document.head.appendChild(link);
}

/// uPlot.join with the default NULL_RETAIN mode: explicit nulls in a trace
/// stay nulls (rendered as gaps), while alignment artifacts from merging
/// per-trace x grids become undefined (rendered as connected).
function joinTraces(traces) {
  return uPlot.join(traces.map((tr) => [tr.t, tr.y]));
}

function fmtTick(v, span) {
  return Math.abs(span) >= 20 ? v.toFixed(0) : v.toFixed(2);
}

function axisTickValues(unit) {
  return (u, vals) => {
    const span = vals.length > 1 ? vals[vals.length - 1] - vals[0] : 0;
    return vals.map((v) => fmtTick(v, span) + unit);
  };
}

function themedAxis(extra) {
  return {
    stroke: THEME.axisText,
    grid: { stroke: THEME.grid, width: 1 },
    ticks: { stroke: THEME.grid, width: 1 },
    font: THEME.font,
    ...extra,
  };
}

function marksPlugin(marks) {
  return {
    hooks: {
      draw: (u) => {
        const xMin = u.scales.x.min;
        const xMax = u.scales.x.max;
        u.ctx.save();
        u.ctx.lineWidth = 1;
        u.ctx.setLineDash([4, 4]);
        for (const m of marks) {
          if (m.x < xMin || m.x > xMax) continue;
          const px = u.valToPos(m.x, "x", true);
          u.ctx.strokeStyle = m.color;
          u.ctx.beginPath();
          u.ctx.moveTo(px, u.bbox.top);
          u.ctx.lineTo(px, u.bbox.top + u.bbox.height);
          u.ctx.stroke();
        }
        u.ctx.restore();
      },
    },
  };
}

/// The metrics-vs-gain readout: uPlot's x-cursor picks one index across all
/// series, but this chart wants the single nearest point by 2D pixel
/// distance — the hovered point, for that run/metric — so the plugin snaps
/// itself and draws the vertical line, dot, and value box.
function nearestPointPlugin({ yLabel, xUnit, xTitle, traces }) {
  let hovered = null;
  const fmtVal = (v) => (Math.abs(v) >= 1000 ? v.toFixed(0) : v.toFixed(1));
  return {
    opts: (u, opts) => ({ ...opts, cursor: { ...opts.cursor, show: true, x: false, y: false, points: { show: false } } }),
    hooks: {
      setCursor: (u) => {
        const { left, top } = u.cursor;
        let best = null;
        if (left != null && left >= 0 && top != null && top >= 0) {
          const xs = u.data[0];
          for (let si = 1; si < u.data.length; si++) {
            const ys = u.data[si];
            for (let i = 0; i < xs.length; i++) {
              if (ys[i] == null) continue;
              const dx = u.valToPos(xs[i], "x") - left;
              const dy = u.valToPos(ys[i], "y") - top;
              const d = dx * dx + dy * dy;
              if (!best || d < best.d) best = { d, si, i };
            }
          }
        }
        hovered = best;
        u.redraw(false);
      },
      draw: (u) => {
        if (!hovered) return;
        const { si, i } = hovered;
        const dpr = uPlot.pxRatio || window.devicePixelRatio || 1;
        const xVal = u.data[0][i];
        const yVal = u.data[si][i];
        const px = u.valToPos(xVal, "x", true);
        const py = u.valToPos(yVal, "y", true);
        const ctx = u.ctx;
        const trace = traces[si - 1];
        ctx.save();
        ctx.strokeStyle = THEME.cursor;
        ctx.lineWidth = dpr;
        ctx.beginPath();
        ctx.moveTo(px, u.bbox.top);
        ctx.lineTo(px, u.bbox.top + u.bbox.height);
        ctx.stroke();
        ctx.fillStyle = trace.color;
        ctx.beginPath();
        ctx.arc(px, py, 3 * dpr, 0, 2 * Math.PI);
        ctx.fill();
        const swept = xTitle ? `${xTitle} = ` : "";
        const lab = trace.label != null ? trace.label : "";
        const span = (u.scales.x.max ?? 0) - (u.scales.x.min ?? 0);
        const text = `${swept}${fmtTick(xVal, span)}${xUnit}  ${fmtVal(yVal)} ${yLabel}${lab ? "  " + lab : ""}`;
        ctx.font = THEME.readoutFont.replace("11px", `${11 * dpr}px`);
        const tw = ctx.measureText(text).width;
        const tx = Math.min(
          Math.max(px + 8 * dpr, u.bbox.left),
          u.bbox.left + u.bbox.width - tw - 8 * dpr
        );
        const ty = Math.max(py - 10 * dpr, u.bbox.top + 12 * dpr);
        ctx.fillStyle = THEME.bg;
        ctx.fillRect(tx - 4 * dpr, ty - 10 * dpr, tw + 8 * dpr, 14 * dpr);
        ctx.strokeStyle = THEME.cursor;
        ctx.strokeRect(tx - 4 * dpr, ty - 10 * dpr, tw + 8 * dpr, 14 * dpr);
        ctx.fillStyle = THEME.readoutText;
        ctx.fillText(text, tx, ty);
        ctx.restore();
      },
    },
  };
}

function traceSeries(tr) {
  return {
    label: tr.label != null ? tr.label : "",
    stroke: tr.color,
    width: 1.25,
    dash: tr.dash && tr.dash.length ? tr.dash : undefined,
    points: tr.points
      ? { show: true, size: 6, fill: tr.color, stroke: tr.color }
      : { show: false },
    spanGaps: false,
  };
}

/// Thin themed wrapper: builds one dark uPlot from drawChart-style traces
/// ({t, y, color, dash?, points?, label?}) and returns {u, setTraces} so
/// live charts can stream new data into a persistent instance.
function timeSeriesPlot(target, opts) {
  ensureUplotCss();
  const { width, height, yLabel, marks = [], hover = false } = opts;
  const xUnit = opts.xUnit == null ? "s" : opts.xUnit;
  let fixedY = opts.fixedY || null;
  let traces = opts.traces;

  const plugins = [];
  if (marks.length) plugins.push(marksPlugin(marks));
  if (hover) plugins.push(nearestPointPlugin({ yLabel, xUnit, xTitle: opts.xTitle, traces }));

  const uOpts = {
    width,
    height,
    pxAlign: false,
    cursor: { show: false },
    legend: { show: false },
    scales: {
      x: { time: false },
      y: {
        range: (u, dataMin, dataMax) => {
          const lo = fixedY ? fixedY.yMin : dataMin;
          const hi = fixedY ? fixedY.yMax : dataMax;
          if (lo == null || hi == null) return [0, 1];
          return lo === hi ? [lo - 1, hi + 1] : [lo, hi];
        },
      },
    },
    axes: [
      themedAxis({ values: axisTickValues(xUnit), size: 24 }),
      themedAxis({ label: yLabel, labelFont: THEME.font, labelGap: 2, size: 44 }),
    ],
    series: [{}, ...traces.map(traceSeries)],
    plugins,
  };

  const u = new uPlot(uOpts, joinTraces(traces), target);
  u.root.style.background = THEME.bg;

  return {
    u,
    setTraces(nextTraces, nextFixedY) {
      if (nextTraces.length !== traces.length) {
        throw new Error(
          `uplot setTraces: trace count changed ${traces.length} -> ${nextTraces.length}`
        );
      }
      traces = nextTraces;
      fixedY = nextFixedY || null;
      u.setData(joinTraces(traces));
    },
  };
}

export { timeSeriesPlot, THEME };
