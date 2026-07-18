import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";

const THEME = {
  bg: "#0d1117",
  grid: "#29313a",
  axisText: "#8a97a3",
  cursor: "#4a5560",
  readoutText: "#e6edf3",
  font: "10px monospace",
  readoutFont: "11px monospace",
};

interface TimeTrace {
  t: number[];
  y: (number | null)[];
  color: string;
  dash?: number[];
  points?: boolean;
  label?: string;
}

interface PsdTrace {
  freq: number[];
  y: number[];
  color: string;
  dashed: boolean;
  label?: string;
  run?: string;
}

interface Mark {
  x: number;
  color: string;
}

interface FreqMarker {
  freq: number;
  label: string;
}

interface FixedY {
  yMin: number;
  yMax: number;
}

interface Brush {
  minSpan: number;
  onSelect: (sel: [number, number] | null) => void;
}

interface TimeSeriesOpts {
  width: number;
  height: number;
  yLabel: string;
  traces: TimeTrace[];
  marks?: Mark[];
  hover?: boolean;
  brush?: Brush | null;
  xUnit?: string;
  xTitle?: string;
  fixedY?: FixedY | null;
}

interface TimeSeriesPlot {
  u: uPlot;
  setTraces(nextTraces: TimeTrace[], nextFixedY?: FixedY | null): void;
  setBrush(sel: [number, number] | null): void;
}

interface PsdPlotOpts {
  width: number;
  height: number;
  traces: PsdTrace[];
  band?: [number, number] | null;
  yTitle: string;
  linear?: boolean;
  zeroFloor?: boolean;
  fixedY?: FixedY | null;
  threshold?: number | null;
  markers?: FreqMarker[] | null;
  formatValue: (v: number) => string;
}

/// uPlot.join with the default NULL_RETAIN mode: explicit nulls in a trace
/// stay nulls (rendered as gaps), while alignment artifacts from merging
/// per-trace x grids become undefined (rendered as connected).
function joinTraces(traces: { t: number[]; y: (number | null)[] }[]): uPlot.AlignedData {
  return uPlot.join(traces.map((tr) => [tr.t, tr.y] as uPlot.AlignedData));
}

function fmtTick(v: number, span: number): string {
  return Math.abs(span) >= 20 ? v.toFixed(0) : v.toFixed(2);
}

function axisTickValues(unit: string) {
  return (u: uPlot, vals: number[]) => {
    const span = vals.length > 1 ? vals[vals.length - 1] - vals[0] : 0;
    return vals.map((v) => fmtTick(v, span) + unit);
  };
}

function themedAxis(extra: uPlot.Axis): uPlot.Axis {
  return {
    stroke: THEME.axisText,
    grid: { stroke: THEME.grid, width: 1 },
    ticks: { stroke: THEME.grid, width: 1 },
    font: THEME.font,
    ...extra,
  };
}

function xSpan(u: uPlot): [number, number] {
  const { min, max } = u.scales.x;
  if (min == null || max == null) throw new Error("uplot x scale has no range at draw time");
  return [min, max];
}

function marksPlugin(marks: Mark[]): uPlot.Plugin {
  return {
    hooks: {
      draw: (u) => {
        const [xMin, xMax] = xSpan(u);
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

interface NearestPointOpts {
  yLabel?: string;
  xUnit?: string;
  xTitle?: string;
  traces: { color: string; label?: string }[];
  formatText?: (xVal: number, yVal: number, trace: { color: string; label?: string }) => string;
}

/// The metrics-vs-gain readout: uPlot's x-cursor picks one index across all
/// series, but this chart wants the single nearest point by 2D pixel
/// distance — the hovered point, for that run/metric — so the plugin snaps
/// itself and draws the vertical line, dot, and value box.
function nearestPointPlugin({ yLabel, xUnit, xTitle, traces, formatText }: NearestPointOpts): uPlot.Plugin {
  let hovered: { d: number; si: number; i: number } | null = null;
  const fmtVal = (v: number) => (Math.abs(v) >= 1000 ? v.toFixed(0) : v.toFixed(1));
  return {
    opts: (u, opts) => ({ ...opts, cursor: { ...opts.cursor, show: true, x: false, y: false, points: { show: false } } }),
    hooks: {
      setCursor: (u) => {
        const { left, top } = u.cursor;
        let best: { d: number; si: number; i: number } | null = null;
        if (left != null && left >= 0 && top != null && top >= 0) {
          const xs = u.data[0];
          for (let si = 1; si < u.data.length; si++) {
            const ys = u.data[si];
            for (let i = 0; i < xs.length; i++) {
              const xv = xs[i];
              const yv = ys[i];
              if (xv == null || yv == null) continue;
              const dx = u.valToPos(xv, "x") - left;
              const dy = u.valToPos(yv, "y") - top;
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
        if (xVal == null || yVal == null) return;
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
        let text;
        if (formatText) {
          text = formatText(xVal, yVal, trace);
        } else {
          const swept = xTitle ? `${xTitle} = ` : "";
          const lab = trace.label != null ? trace.label : "";
          const span = (u.scales.x.max ?? 0) - (u.scales.x.min ?? 0);
          text = `${swept}${fmtTick(xVal, span)}${xUnit}  ${fmtVal(yVal)} ${yLabel}${lab ? "  " + lab : ""}`;
        }
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

function traceSeries(tr: TimeTrace): uPlot.Series {
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
function timeSeriesPlot(target: HTMLElement, opts: TimeSeriesOpts): TimeSeriesPlot {
  const { width, height, yLabel, marks = [], hover = false, brush = null } = opts;
  const xUnit = opts.xUnit == null ? "s" : opts.xUnit;
  let fixedY = opts.fixedY || null;
  let traces = opts.traces;

  const plugins: uPlot.Plugin[] = [];
  if (marks.length) plugins.push(marksPlugin(marks));
  if (hover) plugins.push(nearestPointPlugin({ yLabel, xUnit, xTitle: opts.xTitle, traces }));

  const uOpts: uPlot.Options = {
    width,
    height,
    pxAlign: false,
    cursor: brush
      ? { points: { show: false }, drag: { x: true, y: false, setScale: false } }
      : { show: false },
    hooks: brush
      ? {
          setSelect: [
            (u: uPlot) => {
              const lo = u.posToVal(u.select.left, "x");
              const hi = u.posToVal(u.select.left + u.select.width, "x");
              if (hi - lo < brush.minSpan) {
                u.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false);
                brush.onSelect(null);
              } else {
                brush.onSelect([lo, hi]);
              }
            },
          ],
        }
      : {},
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
    setBrush(sel) {
      if (!sel) {
        u.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false);
        return;
      }
      const left = u.valToPos(sel[0], "x");
      const width = u.valToPos(sel[1], "x") - left;
      const dpr = uPlot.pxRatio || window.devicePixelRatio || 1;
      u.setSelect({ left, top: 0, width, height: u.bbox.height / dpr }, false);
    },
  };
}

const PSD_LOG_FLOOR = 1e-6;

function psdBandPlugin(band: [number, number], traces: PsdTrace[], plotY: (v: number) => number): uPlot.Plugin {
  return {
    hooks: {
      drawClear: (u) => {
        const [xMin, xMax] = xSpan(u);
        const lo = Math.max(band[0], xMin);
        const hi = Math.min(band[1], xMax);
        if (hi <= lo) return;
        const x0 = u.valToPos(lo, "x", true);
        const x1 = u.valToPos(hi, "x", true);
        u.ctx.save();
        u.ctx.fillStyle = "rgba(217, 164, 65, 0.10)";
        u.ctx.fillRect(x0, u.bbox.top, x1 - x0, u.bbox.height);
        u.ctx.restore();
      },
      draw: (u) => {
        const dpr = uPlot.pxRatio || window.devicePixelRatio || 1;
        u.ctx.save();
        u.ctx.font = THEME.font.replace("10px", `${10 * dpr}px`);
        traces.forEach((tr, idx) => {
          let bestI = -1;
          let bestV = -Infinity;
          for (let i = 0; i < tr.freq.length; i++) {
            if (tr.freq[i] >= band[0] && tr.freq[i] < band[1] && tr.y[i] > bestV) {
              bestV = tr.y[i];
              bestI = i;
            }
          }
          if (bestI < 0) return;
          const px = u.valToPos(tr.freq[bestI], "x", true);
          const py = u.valToPos(plotY(tr.y[bestI]), "y", true);
          u.ctx.fillStyle = tr.color;
          u.ctx.beginPath();
          u.ctx.arc(px, py, 2.5 * dpr, 0, Math.PI * 2);
          u.ctx.fill();
          u.ctx.fillText(
            `${tr.freq[bestI].toFixed(0)}Hz`,
            px + 4 * dpr,
            py - (4 + (idx % 3) * 10) * dpr
          );
        });
        u.ctx.restore();
      },
    },
  };
}

function thresholdPlugin(threshold: number, plotY: (v: number) => number): uPlot.Plugin {
  return {
    hooks: {
      draw: (u) => {
        const dpr = uPlot.pxRatio || window.devicePixelRatio || 1;
        const py = u.valToPos(plotY(threshold), "y", true);
        u.ctx.save();
        u.ctx.strokeStyle = "#d9a441";
        u.ctx.lineWidth = dpr;
        u.ctx.setLineDash([4 * dpr, 3 * dpr]);
        u.ctx.beginPath();
        u.ctx.moveTo(u.bbox.left, py);
        u.ctx.lineTo(u.bbox.left + u.bbox.width, py);
        u.ctx.stroke();
        u.ctx.restore();
      },
    },
  };
}

function freqMarkersPlugin(markers: FreqMarker[]): uPlot.Plugin {
  return {
    hooks: {
      draw: (u) => {
        const dpr = uPlot.pxRatio || window.devicePixelRatio || 1;
        const [xMin, xMax] = xSpan(u);
        u.ctx.save();
        u.ctx.font = THEME.font.replace("10px", `${10 * dpr}px`);
        u.ctx.setLineDash([3 * dpr, 3 * dpr]);
        u.ctx.lineWidth = dpr;
        markers.forEach((m, idx) => {
          if (m.freq < xMin || m.freq > xMax) return;
          const px = u.valToPos(m.freq, "x", true);
          u.ctx.strokeStyle = "#b388ff";
          u.ctx.beginPath();
          u.ctx.moveTo(px, u.bbox.top);
          u.ctx.lineTo(px, u.bbox.top + u.bbox.height);
          u.ctx.stroke();
          u.ctx.fillStyle = "#b388ff";
          u.ctx.fillText(m.label, px + 4 * dpr, u.bbox.top + (12 + (idx % 3) * 10) * dpr);
        });
        u.ctx.restore();
      },
    },
  };
}

/// Frequency-domain sibling of timeSeriesPlot: takes PSD-style traces
/// ({freq, y, color, dashed, label}) and builds a uPlot with a true
/// log-10 y scale (uPlot distr:3) unless `linear`, plus the shared PSD
/// furniture — band shading with per-trace peak dots, a threshold line,
/// staggered vertical mode markers, and the nearest-point hover readout.
function psdPlot(target: HTMLElement, opts: PsdPlotOpts): uPlot {
  const { width, height, traces, band, yTitle, linear, zeroFloor, fixedY, threshold, markers, formatValue } = opts;
  const plotY = linear ? (v: number) => v : (v: number) => Math.max(v, PSD_LOG_FLOOR);

  const plugins: uPlot.Plugin[] = [];
  if (band) plugins.push(psdBandPlugin(band, traces, plotY));
  if (threshold != null) plugins.push(thresholdPlugin(threshold, plotY));
  if (markers && markers.length) plugins.push(freqMarkersPlugin(markers));
  plugins.push(
    nearestPointPlugin({
      traces,
      formatText: (xVal, yVal, trace) =>
        `${xVal.toFixed(1)} Hz  ${formatValue(yVal)}  ${trace.label}`,
    })
  );

  const yScale: uPlot.Scale = linear
    ? {
        range: (u, dataMin, dataMax) => {
          let lo = zeroFloor ? 0 : dataMin;
          let hi = dataMax;
          if (fixedY) {
            lo = fixedY.yMin;
            hi = fixedY.yMax;
          }
          if (lo == null || hi == null) return [0, 1];
          return lo === hi ? [lo - 1, hi + 1] : [lo, hi];
        },
      }
    : { distr: 3 };

  const u = new uPlot(
    {
      width,
      height,
      pxAlign: false,
      cursor: { show: true, x: false, y: false, points: { show: false } },
      legend: { show: false },
      scales: { x: { time: false }, y: yScale },
      axes: [
        themedAxis({ values: (u2, vals) => vals.map((f) => f.toFixed(0) + "Hz"), size: 24 }),
        themedAxis({
          label: yTitle,
          labelFont: THEME.font,
          labelGap: 2,
          size: 52,
          values: (u2, vals) => vals.map(formatValue),
        }),
      ],
      series: [
        {},
        ...traces.map((tr) => ({
          label: tr.label,
          stroke: tr.color,
          width: 1.25,
          dash: tr.dashed ? [4, 3] : undefined,
          points: { show: false },
          spanGaps: false,
        })),
      ],
      plugins,
    },
    joinTraces(traces.map((tr) => ({ t: tr.freq, y: tr.y.map(plotY) }))),
    target
  );
  u.root.style.background = THEME.bg;
  return u;
}

export type { TimeTrace, PsdTrace, Mark, FreqMarker, FixedY, Brush, TimeSeriesOpts, TimeSeriesPlot, PsdPlotOpts };
export { timeSeriesPlot, psdPlot, THEME };
