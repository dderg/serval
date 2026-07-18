import { el } from "./api.js";
import { drawChart, mixColor } from "./charts-core.js";
import { psdBox, visibleStepNames } from "./metrics.js";
import { runColor, refresh } from "./runs.js";
import { RINGDOWN_PSD_PLOT_MAX_HZ, state } from "./state.js";

// --- differential belt FRF (dynamics page) -----------------------------------

const FRF_BOXES = [
  { key: "mag_db", title: "magnitude", yTitle: "|H| (dB)" },
  { key: "phase_deg", title: "phase", yTitle: "phase (deg)" },
  { key: "coherence", title: "coherence", yTitle: "coherence" },
  { key: "torque_db", title: "torque FRF", yTitle: "torque (dB)" },
];

function differentialSeries(step) {
  const d = step.differential;
  if (!d) return null;
  const n = d.freq_hz.length;
  for (const spec of FRF_BOXES) {
    const arr = d[spec.key];
    if (!Array.isArray(arr) || arr.length !== n) {
      throw new Error(
        `${step.name}: differential.${spec.key} length ${Array.isArray(arr) ? arr.length : "missing"} != freq_hz length ${n}`
      );
    }
  }
  return d;
}

function frfTraces(names, plots, stepName, key) {
  const traces = [];
  plots.forEach((p, i) => {
    const step = p.steps.find((s) => s.name === stepName);
    const d = step && differentialSeries(step);
    if (!d) return;
    traces.push({
      freq: d.freq_hz,
      y: d[key],
      color: runColor(names[i]),
      dashed: false,
      label: names[i],
      run: names[i],
    });
  });
  return traces;
}

function frfModeMarkers(modes) {
  return modes.map((m) => ({
    freq: m.freq_hz,
    label: `${m.freq_hz.toFixed(1)} Hz${m.damping == null ? "" : ` ζ=${m.damping.toFixed(3)}`}`,
  }));
}

function frfModeTableHtml(modes) {
  if (!modes.length) return '<p class="note">no modes detected</p>';
  const rows = modes
    .map(
      (m) =>
        `<tr><td>${m.freq_hz.toFixed(1)} Hz</td><td>${m.gain_db.toFixed(1)} dB</td>` +
        `<td>${m.damping == null ? "—" : m.damping.toFixed(3)}</td><td>${m.coherence.toFixed(2)}</td></tr>`
    )
    .join("");
  return (
    `<table class="mode-table"><thead><tr>` +
    `<th>freq</th><th>|H|</th><th>damping</th><th>coherence</th>` +
    `</tr></thead><tbody>${rows}</tbody></table>`
  );
}

function differentialResultStep(runName, stepName) {
  const detail = state.details.get(runName);
  const step =
    detail && detail.results && detail.results.steps.find((s) => s.name === stepName);
  return (step && step.differential) || null;
}

/// The newest selected run with a differential step drives the mode markers,
/// the coherence threshold, and the mode table; every selected run's traces
/// overlay on the four shared-x boxes.
function renderFrfCharts(names, plots) {
  const section = el("frf-section");
  if (!section) return;
  const container = el("frf-charts");
  const modesEl = el("frf-modes");
  const meta = el("frf-meta");
  container.innerHTML = "";
  modesEl.innerHTML = "";
  meta.textContent = "";
  const stepNames = [
    ...new Set(plots.flatMap((p) => p.steps.filter((s) => s.differential).map((s) => s.name))),
  ];
  if (!stepNames.length) {
    section.hidden = true;
    return;
  }
  section.hidden = false;
  const metaParts = [];
  for (const stepName of stepNames) {
    let ref = null;
    let refName = null;
    for (let i = 0; i < plots.length; i++) {
      const step = plots[i].steps.find((s) => s.name === stepName);
      const d = step && differentialSeries(step);
      if (d) {
        ref = d;
        refName = names[i];
        break;
      }
    }
    for (const spec of FRF_BOXES) {
      const opts = { linear: true };
      if (spec.key === "mag_db") opts.markers = frfModeMarkers(ref.modes);
      if (spec.key === "coherence") {
        opts.fixedY = { yMin: 0, yMax: 1.05 };
        opts.threshold = ref.coherence_min;
      }
      container.appendChild(
        psdBox(
          `${stepName} — ${spec.title}`,
          frfTraces(names, plots, stepName, spec.key),
          null,
          spec.yTitle,
          opts
        )
      );
    }
    const result = differentialResultStep(refName, stepName);
    const label = result
      ? `${result.pair.join(" vs ")} — ${result.segments} Welch segments`
      : refName;
    modesEl.innerHTML += `<h3>${stepName} modes — ${label}</h3>${frfModeTableHtml(ref.modes)}`;
    metaParts.push(label);
  }
  meta.textContent = metaParts.join(" · ");
}

// --- ring-down after stop (dynamics page) -------------------------------------

function ringdownModeTableHtml(sources) {
  const rows = sources
    .flatMap((src) =>
      src.modes.length
        ? src.modes.map(
            (m) =>
              `<tr><td>${src.source}</td><td>${m.freq_hz.toFixed(1)} Hz</td>` +
              `<td>${m.zeta.toFixed(3)}</td>` +
              `<td>${m.zeta_lo.toFixed(3)}–${m.zeta_hi.toFixed(3)}</td>` +
              `<td>${m.disp_um.toFixed(2)} µm</td>` +
              `<td>${m.tails}</td><td>${m.r2.toFixed(2)}</td></tr>`
          )
        : [
            `<tr><td>${src.source}</td>` +
              `<td colspan="6" class="note">no ring above the noise floor</td></tr>`,
          ]
    )
    .join("");
  return (
    `<table class="mode-table"><thead><tr>` +
    `<th>source</th><th>freq</th><th>ζ</th><th>ζ spread</th>` +
    `<th>residual</th><th>tails</th><th>r²</th>` +
    `</tr></thead><tbody>${rows}</tbody></table>`
  );
}

function fftPow2Js(re, im) {
  const n = re.length;
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      [re[i], re[j]] = [re[j], re[i]];
      [im[i], im[j]] = [im[j], im[i]];
    }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = (-2 * Math.PI) / len;
    const wRe = Math.cos(ang), wIm = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let curRe = 1, curIm = 0;
      for (let k = 0; k < len / 2; k++) {
        const uRe = re[i + k], uIm = im[i + k];
        const vRe = re[i + k + len / 2] * curRe - im[i + k + len / 2] * curIm;
        const vIm = re[i + k + len / 2] * curIm + im[i + k + len / 2] * curRe;
        re[i + k] = uRe + vRe;
        im[i + k] = uIm + vIm;
        re[i + k + len / 2] = uRe - vRe;
        im[i + k + len / 2] = uIm - vIm;
        const nextRe = curRe * wRe - curIm * wIm;
        curIm = curRe * wIm + curIm * wRe;
        curRe = nextRe;
      }
    }
  }
}

// Mirrors the analyzer's welch_psd (Hann, pow2 nperseg ≤ 1024, 50% overlap,
// per-segment mean removal, one-sided) so a full-tail selection reproduces
// the server-computed PSD. Returns null when the selection is too short.
function welchPsdJs(x, fs) {
  let nperseg = 1;
  const cap = Math.min(x.length, 1024);
  while (nperseg * 2 <= cap) nperseg *= 2;
  if (nperseg < 64) return null;
  const step = nperseg / 2;
  const win = [];
  let winSqSum = 0;
  for (let k = 0; k < nperseg; k++) {
    const w = 0.5 - 0.5 * Math.cos((2 * Math.PI * k) / (nperseg - 1));
    win.push(w);
    winSqSum += w * w;
  }
  const scale = 1 / (fs * winSqSum);
  const bins = nperseg / 2 + 1;
  const acc = new Array(bins).fill(0);
  let count = 0;
  for (let start = 0; start + nperseg <= x.length; start += step) {
    const seg = x.slice(start, start + nperseg);
    const mean = seg.reduce((a, b) => a + b, 0) / nperseg;
    const re = seg.map((v, k) => (v - mean) * win[k]);
    const im = new Array(nperseg).fill(0);
    fftPow2Js(re, im);
    for (let b = 0; b < bins; b++) acc[b] += (re[b] * re[b] + im[b] * im[b]) * scale;
    count++;
  }
  const psd = acc.map((a) => a / count);
  for (let b = 1; b < bins - 1; b++) psd[b] *= 2;
  const freqs = Array.from({ length: bins }, (_, i) => (i * fs) / nperseg);
  return { freqs, psd };
}

function ringdownTailColor(name, k, count) {
  return mixColor(runColor(name), "#ffffff", (0.5 * k) / Math.max(1, count - 1));
}

// Brush spans survive the periodic refresh() re-render: keyed by
// step|source, re-applied when the chart DOM is rebuilt.
const ringdownSelections = new Map();

/// Tail chart with a drag-to-select brush: the selected time span is shaded
/// and reported through onSelect (in ms, or null when cleared by a click),
/// which drives the PSD-of-selection chart next to it.
function ringdownTailBox(stepName, source, runEntries, onSelect) {
  const selKey = `${stepName}|${source.source}`;
  const traces = [];
  const legend = [];
  for (const { name, src } of runEntries) {
    src.tails.forEach((tail, k) => {
      const t = tail.value.map((_, i) => (i / src.fs_hz) * 1000);
      traces.push({ t, y: tail.value, color: ringdownTailColor(name, k, src.tails.length) });
    });
    legend.push({ color: runColor(name), label: `${name} (${src.tails.length} tails)` });
  }
  const ref = runEntries[0].src;
  if (ref.modes.length && traces.length) {
    const dominant = ref.modes.reduce((a, b) => (b.amp > a.amp ? b : a));
    const sigma = dominant.zeta * 2 * Math.PI * dominant.freq_hz;
    const tEnv = traces[0].t;
    const env = tEnv.map((tMs) => dominant.amp * Math.exp((-sigma * (tMs - dominant.fit_start_ms)) / 1000));
    traces.push({ t: tEnv, y: env, color: "#8a97a3", dash: [5, 3] });
    traces.push({ t: tEnv, y: env.map((v) => -v), color: "#8a97a3", dash: [5, 3] });
    legend.push({
      color: "#8a97a3",
      label:
        `dominant mode ${dominant.freq_hz.toFixed(1)} Hz ` +
        `ζ=${dominant.zeta.toFixed(3)} — envelope fit`,
    });
  }
  const box = document.createElement("div");
  box.className = "chart-box";
  const title = document.createElement("h3");
  title.textContent = `${stepName} — ${source.source} tails (${source.unit}) — drag to select a span for the PSD`;
  box.appendChild(title);
  const canvas = document.createElement("canvas");
  canvas.width = 860;
  canvas.height = 220;
  box.appendChild(canvas);

  const tMax = traces.reduce((m, tr) => Math.max(m, tr.t[tr.t.length - 1] || 0), 0);
  const pad = { l: 46, r: 8 };
  const pxToMs = (px) => {
    const w = canvas.clientWidth || canvas.width;
    return ((px - pad.l) / (w - pad.l - pad.r)) * tMax;
  };
  const msToPx = (ms) => {
    const w = canvas.clientWidth || canvas.width;
    return pad.l + (ms / tMax) * (w - pad.l - pad.r);
  };
  const redraw = (sel) => {
    drawChart(canvas, traces, source.unit, null, "ms");
    if (sel) {
      const ctx = canvas.getContext("2d");
      const h = canvas.clientHeight || canvas.height;
      ctx.fillStyle = "rgba(88, 166, 255, 0.15)";
      const x0 = msToPx(sel[0]), x1 = msToPx(sel[1]);
      ctx.fillRect(Math.min(x0, x1), 8, Math.abs(x1 - x0), h - 30);
    }
  };
  let dragFrom = null;
  let selection = ringdownSelections.get(selKey) || null;
  if (selection && selection[1] > tMax) selection = null;
  redraw(selection);
  canvas.addEventListener("mousedown", (e) => {
    dragFrom = pxToMs(e.offsetX);
  });
  canvas.addEventListener("mousemove", (e) => {
    if (dragFrom == null) return;
    selection = [dragFrom, pxToMs(e.offsetX)];
    redraw(selection);
  });
  const finish = (e) => {
    if (dragFrom == null) return;
    const from = dragFrom;
    const to = pxToMs(e.offsetX);
    dragFrom = null;
    const lo = Math.max(0, Math.min(from, to));
    const hi = Math.min(tMax, Math.max(from, to));
    if (hi - lo < 2) {
      selection = null;
      ringdownSelections.delete(selKey);
      redraw(null);
      onSelect(null);
      return;
    }
    selection = [lo, hi];
    ringdownSelections.set(selKey, selection);
    redraw(selection);
    onSelect(selection);
  };
  canvas.addEventListener("mouseup", finish);
  canvas.addEventListener("mouseleave", (e) => {
    if (dragFrom != null) finish(e);
  });

  const legendEl = document.createElement("div");
  legendEl.className = "legend";
  for (const item of legend) {
    const span = document.createElement("span");
    span.innerHTML = `<span class="swatch" style="background:${item.color}"></span>${item.label}`;
    legendEl.appendChild(span);
  }
  box.appendChild(legendEl);
  return box;
}

/// PSD traces for a brushed span of the tails: one trace per tail (per
/// stroke), computed client-side with the same Welch settings as the
/// analyzer. Returns null when the span holds too few samples.
function ringdownSelectionPsdTraces(runEntries, selMs) {
  const traces = [];
  for (const { name, src } of runEntries) {
    const i0 = Math.max(0, Math.floor((selMs[0] / 1000) * src.fs_hz));
    const i1 = Math.min(
      Math.min(...src.tails.map((t) => t.value.length)),
      Math.ceil((selMs[1] / 1000) * src.fs_hz)
    );
    for (let k = 0; k < src.tails.length; k++) {
      const out = welchPsdJs(src.tails[k].value.slice(i0, i1), src.fs_hz);
      if (!out) return null;
      const cut = out.freqs.filter((f) => f <= RINGDOWN_PSD_PLOT_MAX_HZ).length;
      traces.push({
        freq: out.freqs.slice(0, cut),
        y: out.psd.slice(0, cut),
        color: ringdownTailColor(name, k, src.tails.length),
        dashed: false,
        label: `${name} tail ${k + 1}`,
        run: name,
      });
    }
  }
  return traces;
}

function ringdownModeMarkers(modes) {
  return modes.map((m) => ({
    freq: m.freq_hz,
    label: `${m.freq_hz.toFixed(1)} Hz ζ=${m.zeta.toFixed(3)}`,
  }));
}

/// Every selected run's tails overlay per source; the newest selected run
/// with a ringdown step drives the envelope fit, the PSD mode markers and
/// the mode table. Only sources the analyzer marked as headline (tails
/// present in the plot payload) get charts — every source lands in the
/// table.
function renderRingdownCharts(names, plots) {
  const section = el("ringdown-section");
  if (!section) return;
  const container = el("ringdown-charts");
  const modesEl = el("ringdown-modes");
  const meta = el("ringdown-meta");
  container.innerHTML = "";
  modesEl.innerHTML = "";
  meta.textContent = "";
  const stepNames = [
    ...new Set(plots.flatMap((p) => p.steps.filter((s) => s.ringdown).map((s) => s.name))),
  ];
  if (!stepNames.length) {
    section.hidden = true;
    return;
  }
  section.hidden = false;
  const metaParts = [];
  for (const stepName of visibleStepNames(stepNames)) {
    const perSource = new Map();
    plots.forEach((p, i) => {
      const step = p.steps.find((s) => s.name === stepName);
      if (!step || !step.ringdown) return;
      for (const src of step.ringdown.sources) {
        if (!src.tails.length) continue;
        if (!perSource.has(src.source)) perSource.set(src.source, []);
        perSource.get(src.source).push({ name: names[i], src });
      }
    });
    let ref = null;
    for (const p of plots) {
      const step = p.steps.find((s) => s.name === stepName);
      if (step && step.ringdown) {
        ref = step.ringdown;
        break;
      }
    }
    for (const [sourceName, runEntries] of perSource) {
      const fullTraces = runEntries.map(({ name, src }) => {
        const cut = src.psd_freq_hz.filter((f) => f <= RINGDOWN_PSD_PLOT_MAX_HZ).length;
        return {
          freq: src.psd_freq_hz.slice(0, cut),
          y: src.psd.slice(0, cut),
          color: runColor(name),
          dashed: false,
          label: name,
          run: name,
        };
      });
      const unit = runEntries[0].src.unit;
      const markers = { markers: ringdownModeMarkers(runEntries[0].src.modes) };
      const psdWrap = document.createElement("div");
      const renderPsd = (selMs) => {
        psdWrap.innerHTML = "";
        if (selMs) {
          const selTraces = ringdownSelectionPsdTraces(runEntries, selMs);
          if (selTraces) {
            psdWrap.appendChild(
              psdBox(
                `${stepName} — ${sourceName} PSD of ${selMs[0].toFixed(0)}–${selMs[1].toFixed(0)}ms, per tail`,
                selTraces,
                null,
                `${unit} PSD`,
                markers
              )
            );
            return;
          }
        }
        psdWrap.appendChild(
          psdBox(`${stepName} — ${sourceName} tail PSD (full dwell, tail average)`, fullTraces, null, `${unit} PSD`, markers)
        );
      };
      container.appendChild(ringdownTailBox(stepName, runEntries[0].src, runEntries, renderPsd));
      container.appendChild(psdWrap);
      renderPsd(ringdownSelections.get(`${stepName}|${sourceName}`) || null);
    }
    modesEl.innerHTML += `<h3>${stepName} modes</h3>${ringdownModeTableHtml(ref.sources)}`;
    metaParts.push(stepName);
  }
  meta.textContent = metaParts.join(" · ");
}

export { FRF_BOXES, differentialSeries, frfTraces, frfModeMarkers, frfModeTableHtml, differentialResultStep, renderFrfCharts, ringdownModeTableHtml, fftPow2Js, welchPsdJs, ringdownTailColor, ringdownSelections, ringdownTailBox, ringdownSelectionPsdTraces, ringdownModeMarkers, renderRingdownCharts };
