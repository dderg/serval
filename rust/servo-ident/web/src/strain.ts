import { api, el, mustEl, pageRuns, shortTime } from "./api";
import { hidpiCanvasContext, mixColor } from "./charts-core";
import { timeSeriesPlot } from "./uplot-chart";
import { loadRerunForm } from "./drive";
import { currentPageDef, controlsSectionsHtml, sectionHeadHtml } from "./shell";
import { PALETTE, state } from "./state";
import type { PageDef } from "./state";
import type { StrainData, StrainField, StrainLine, StrokePlan } from "./wire";

// --- strain map (strain page) -------------------------------------------------
//
// Renders GET /api/runs/<name>/strain: per raster line, the elastic
// (direction-symmetric) differential belt torque binned along the sweep.
// All four heatmaps (belt × sweep orientation) draw in BED coordinates —
// horizontal = bed x, vertical = bed y increasing upward, square aspect —
// on one symmetric diverging scale, so a feature at some bed position
// lines up across every panel. X-sweep lines are horizontal bands at
// their swept y; Y-sweep lines are vertical bands at their swept x.

const STRAIN_NEUTRAL = "#1f2630";
const STRAIN_NEG = "#4fb3ff";
const STRAIN_POS = "#e05a4f";
const STRAIN_HEAT_PAD = { l: 40, r: 8, t: 16, b: 26 };
const STRAIN_HEAT_CANVAS_W = 430;
const STRAIN_LINE_SPACING_FALLBACK_MM = 20;

function strainShellHtml(def: PageDef): string {
  return (
    `<div class="workspace">` +
    `<main class="analysis">` +
    `<section class="runs-section">` +
    sectionHeadHtml(
      "strain runs",
      `<span class="note">strain_map — click to map, shift+click a second run to diff (matching dimensions)</span>`
    ) +
    `<div class="table-wrap runs-wrap"><table><thead><tr>` +
    `<th>time</th><th>tag</th><th></th>` +
    `</tr></thead><tbody id="strain-run-body"></tbody></table></div>` +
    `</section>` +
    `<section>` +
    sectionHeadHtml(
      "strain map",
      `<button class="strain-field-btn" data-field="elastic">elastic</button>` +
        `<button class="strain-field-btn" data-field="friction" ` +
        `title="the direction-dependent half: (forward - backward)/2 — what a position-keyed offset cannot cancel">friction</button>` +
        `<span class="note" id="strain-summary"></span>`
    ) +
    `<div id="strain-heatmaps" class="strain-grid"></div>` +
    `<div id="strain-scale"></div>` +
    `</section>` +
    `<section>` +
    sectionHeadHtml(
      "per-line elastic profiles",
      `<span class="note">one polyline per raster line, in bed coordinates</span>`
    ) +
    `<div class="charts" id="strain-profiles"></div>` +
    `</section>` +
    `<section>` +
    sectionHeadHtml(
      "per-line DC offset — mean elastic",
      `<span class="note">a line-to-line offset is trapped preload, not local strain</span>`
    ) +
    `<div class="strain-grid" id="strain-dc"></div>` +
    `</section>` +
    `</main>` +
    `<aside class="controls">${controlsSectionsHtml(def)}</aside>` +
    `</div>`
  );
}

async function ensureStrain(name: string): Promise<StrainData> {
  const run = state.runs.find((r) => r.name === name);
  const cached = state.strain.cache.get(name);
  if (cached && run && cached.mtime_utc === run.mtime_utc) return cached.data;
  const data: StrainData = await api(`/api/runs/${encodeURIComponent(name)}/strain`);
  state.strain.cache.set(name, { mtime_utc: run ? run.mtime_utc : null, data });
  return data;
}

function runTag(name: string): string {
  const r = state.runs.find((x) => x.name === name);
  return r ? `${r.tag}${r.axis ? " " + r.axis : ""}` : name;
}

/// Builds per-belt elastic/friction diff arrays (a − b); a null on either
/// side propagates, since an unbinned cell has no meaning to subtract.
function buildDiffLine(base: StrainLine, cmp: StrainLine): StrainLine {
  return {
    name: base.name,
    swept: base.swept,
    bin_centers: base.bin_centers,
    belts: base.belts.map((bb, bi) => {
      const cb = cmp.belts[bi];
      return {
        pair: bb.pair,
        elastic: cb ? pointwiseDiff(bb.elastic, cb.elastic) : nulls(bb.elastic.length),
        friction: cb ? pointwiseDiff(bb.friction, cb.friction) : nulls(bb.friction.length),
      };
    }),
  };
}

function pointwiseDiff(a: (number | null)[], b: (number | null)[]): (number | null)[] {
  const out: (number | null)[] = new Array(a.length);
  for (let i = 0; i < a.length; i++) {
    const av = a[i];
    const bv = b[i];
    out[i] = av === null || bv === null ? null : av - bv;
  }
  return out;
}

function nulls(n: number): null[] {
  return new Array(n).fill(null);
}

/// Compression of a strain map's geometry: line names (which encode
/// orientation + swept coordinate), per-line bin centers, and belt pairs.
/// Two maps with an identical signature can be subtracted cell-by-cell.
function strainSignature(data: StrainData): string {
  return JSON.stringify({
    l: data.lines.map((l) => ({ n: l.name, b: l.bin_centers, p: l.belts.map((x) => x.pair) })),
  });
}

function renderStrainRuns() {
  const tbody = el("strain-run-body");
  if (!tbody) return;
  const runs = pageRuns(currentPageDef());
  if (!runs.some((r) => r.name === state.strain.selected)) {
    state.strain.selected = runs.length ? runs[0].name : null;
  }
  const known = new Set(runs.map((r) => r.name));
  for (const name of [...state.strain.compare]) {
    if (!known.has(name)) state.strain.compare.delete(name);
  }
  tbody.innerHTML = "";
  for (const run of runs) {
    const tr = document.createElement("tr");
    tr.classList.add("selectable");
    if (run.name === state.strain.selected) tr.classList.add("selected");
    if (state.strain.compare.has(run.name)) tr.classList.add("compare");
    tr.addEventListener("click", (ev: MouseEvent) => {
      if (ev.shiftKey) {
        if (run.name === state.strain.selected) return;
        if (state.strain.compare.has(run.name)) state.strain.compare.delete(run.name);
        else state.strain.compare.add(run.name);
      } else {
        state.strain.selected = run.name;
        state.strain.compare.clear();
      }
      redrawStrain();
    });
    const timeTd = document.createElement("td");
    timeTd.textContent = shortTime(run.mtime_utc);
    timeTd.title = `${run.name} — ${run.mtime_utc}`;
    tr.appendChild(timeTd);
    const tagTd = document.createElement("td");
    tagTd.textContent = runTag(run.name);
    tr.appendChild(tagTd);
    const actionTd = document.createElement("td");
    actionTd.className = "actions";
    const prefillBtn = document.createElement("button");
    prefillBtn.textContent = "→ console";
    prefillBtn.title = "prefill the console with this run's command";
    prefillBtn.disabled = !state.details.get(run.name)?.manifest;
    prefillBtn.addEventListener("click", (e: MouseEvent) => {
      e.stopPropagation();
      loadRerunForm(run.name);
    });
    actionTd.appendChild(prefillBtn);
    tr.appendChild(actionTd);
    tbody.appendChild(tr);
  }
}

function strainColor(t: number): string {
  const clamped = Math.max(-1, Math.min(1, t));
  return clamped < 0
    ? mixColor(STRAIN_NEUTRAL, STRAIN_NEG, -clamped)
    : mixColor(STRAIN_NEUTRAL, STRAIN_POS, clamped);
}

function sweptEntry(line: StrainLine): [string, number] {
  const entries = Object.entries(line.swept || {});
  return entries.length ? (entries[0] as [string, number]) : ["?", 0];
}

function strainLineLabel(line: StrainLine): string {
  const [key, value] = sweptEntry(line);
  return `${key}=${Number(value).toFixed(0)}`;
}

/// Raster lines grouped by sweep orientation and ordered by the swept
/// coordinate, so heatmap bands lay out like the bed does.
interface StrainGroup {
  orientation: string;
  title: string;
  lines: StrainLine[];
}

function strainGroups(data: StrainData): StrainGroup[] {
  const bySwept = (a: StrainLine, b: StrainLine) => sweptEntry(a)[1] - sweptEntry(b)[1];
  const group = (orientation: string, prefix: string, title: string): StrainGroup => ({
    orientation,
    title,
    lines: data.lines.filter((l) => l.name.startsWith(prefix)).sort(bySwept),
  });
  return [group("x", "xline", "X sweep"), group("y", "yline", "Y sweep")].filter(
    (g) => g.lines.length
  );
}

/// Bed-frame geometry: the sweep coordinate is shifted to start at 0, so
/// its bed origin is the manifest's stroke_plan start; when the plan is
/// unavailable it is recovered from the run itself — each orientation's
/// sweep starts where the OTHER orientation's raster lines sit, so their
/// minimum swept value is the origin. Band thickness is the raster pitch.
interface StrainGeo {
  x0: number;
  y0: number;
  bandHalf: number;
  xlo: number;
  xhi: number;
  ylo: number;
  yhi: number;
}

function strainBedGeometry(groups: StrainGroup[], plan: StrokePlan): StrainGeo {
  const sweptOf = (orientation: string) => {
    const g = groups.find((x) => x.orientation === orientation);
    return g ? g.lines.map((l) => sweptEntry(l)[1]) : [];
  };
  const minGap = (vals: number[]) => {
    const s = [...vals].sort((a, b) => a - b);
    let gap = Infinity;
    for (let i = 1; i < s.length; i++) gap = Math.min(gap, s[i] - s[i - 1]);
    return isFinite(gap) && gap > 0 ? gap : STRAIN_LINE_SPACING_FALLBACK_MM;
  };
  const xBands = sweptOf("y");
  const yBands = sweptOf("x");
  const spacing = plan.line_spacing || Math.min(minGap(xBands), minGap(yBands));
  const bandHalf = spacing / 2;
  const x0 = plan.x_start != null ? plan.x_start : xBands.length ? Math.min(...xBands) : 0;
  const y0 = plan.y_start != null ? plan.y_start : yBands.length ? Math.min(...yBands) : 0;
  const xs: number[] = [];
  const ys: number[] = [];
  for (const g of groups) {
    for (const line of g.lines) {
      const half = lineBinWidth(line) / 2;
      const c = line.bin_centers;
      const swept = sweptEntry(line)[1];
      const sweepOrigin = g.orientation === "x" ? x0 : y0;
      const along = [sweepOrigin + c[0] - half, sweepOrigin + c[c.length - 1] + half];
      const across = [swept - bandHalf, swept + bandHalf];
      if (g.orientation === "x") {
        xs.push(...along);
        ys.push(...across);
      } else {
        ys.push(...along);
        xs.push(...across);
      }
    }
  }
  const xlo = Math.min(...xs);
  const ylo = Math.min(...ys);
  return {
    x0,
    y0,
    bandHalf,
    xlo,
    xhi: Math.max(Math.max(...xs), xlo + 1),
    ylo,
    yhi: Math.max(Math.max(...ys), ylo + 1),
  };
}

function strainStats(data: StrainData): { maxElastic: number; maxFriction: number; meanFriction: number } {
  let maxElastic = 0;
  let maxFriction = 0;
  let fricSum = 0;
  let fricN = 0;
  for (const line of data.lines) {
    for (const belt of line.belts) {
      for (const v of belt.elastic) {
        if (v !== null) maxElastic = Math.max(maxElastic, Math.abs(v));
      }
      for (const v of belt.friction) {
        if (v !== null) {
          maxFriction = Math.max(maxFriction, Math.abs(v));
          fricSum += Math.abs(v);
          fricN++;
        }
      }
    }
  }
  return { maxElastic, maxFriction, meanFriction: fricN ? fricSum / fricN : 0 };
}

function lineBinWidth(line: StrainLine): number {
  const c = line.bin_centers;
  return c.length > 1 ? c[1] - c[0] : 2 * c[0];
}

function drawStrainHeatmap(canvas: HTMLCanvasElement, group: StrainGroup, beltIdx: number, vmax: number, geo: StrainGeo) {
  const { ctx, w, h } = hidpiCanvasContext(canvas);
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
  const pad = STRAIN_HEAT_PAD;
  const px = (mm: number) => pad.l + ((mm - geo.xlo) / (geo.xhi - geo.xlo)) * (w - pad.l - pad.r);
  const py = (mm: number) => h - pad.b - ((mm - geo.ylo) / (geo.yhi - geo.ylo)) * (h - pad.t - pad.b);

  ctx.font = "10px monospace";
  for (const line of group.lines) {
    const swept = sweptEntry(line)[1];
    const half = lineBinWidth(line) / 2;
    const sweepOrigin = group.orientation === "x" ? geo.x0 : geo.y0;
    line.bin_centers.forEach((center, b) => {
      const v = line.belts[beltIdx][state.strain.field][b];
      if (v === null) return;
      ctx.fillStyle = strainColor(v / vmax);
      const lo = sweepOrigin + center - half;
      const hi = sweepOrigin + center + half;
      if (group.orientation === "x") {
        const top = py(swept + geo.bandHalf);
        ctx.fillRect(px(lo), top + 0.5, px(hi) - px(lo), py(swept - geo.bandHalf) - top - 1);
      } else {
        const left = px(swept - geo.bandHalf);
        ctx.fillRect(left + 0.5, py(hi), px(swept + geo.bandHalf) - left - 1, py(lo) - py(hi));
      }
    });
  }

  ctx.fillStyle = "#8a97a3";
  for (let i = 0; i <= 4; i++) {
    const xmm = geo.xlo + ((geo.xhi - geo.xlo) * i) / 4;
    ctx.fillText(xmm.toFixed(0), Math.min(px(xmm) - 6, w - 20), h - pad.b + 12);
    const ymm = geo.ylo + ((geo.yhi - geo.ylo) * i) / 4;
    ctx.fillText(ymm.toFixed(0), 2, Math.max(py(ymm) + 3, pad.t + 8));
  }
  ctx.fillText("bed x (mm)", pad.l + (w - pad.l - pad.r) / 2 - 30, h - 4);
  ctx.fillText("bed y (mm)", pad.l, 11);
}

function strainHeatmapBox(title: string, group: StrainGroup, beltIdx: number, vmax: number, geo: StrainGeo): HTMLDivElement {
  const box = document.createElement("div");
  box.className = "chart-box";
  const head = document.createElement("h3");
  head.textContent = title;
  box.appendChild(head);
  const canvas = document.createElement("canvas");
  const pad = STRAIN_HEAT_PAD;
  const plotW = STRAIN_HEAT_CANVAS_W - pad.l - pad.r;
  const plotH = plotW * ((geo.yhi - geo.ylo) / (geo.xhi - geo.xlo));
  canvas.width = STRAIN_HEAT_CANVAS_W;
  canvas.height = Math.round(pad.t + pad.b + plotH);
  box.appendChild(canvas);
  drawStrainHeatmap(canvas, group, beltIdx, vmax, geo);
  return box;
}

function strainScaleHtml(vmax: number): string {
  const stops: string[] = [];
  for (let i = 0; i <= 8; i++) stops.push(strainColor(i / 4 - 1));
  const what =
    state.strain.field === "friction"
      ? "friction (direction-dependent) differential torque"
      : "elastic differential torque";
  return (
    `<div class="strain-scale"><span>−${vmax.toFixed(1)}%</span>` +
    `<span class="bar" style="background:linear-gradient(90deg,${stops.join(",")})"></span>` +
    `<span>+${vmax.toFixed(1)}%</span>` +
    `<span class="hint">${what}, % rated — null bins stay dark</span></div>`
  );
}

function strainProfileBox(title: string, beltIdx: number, group: StrainGroup, vmax: number, geo: StrainGeo): HTMLDivElement {
  const box = document.createElement("div");
  box.className = "chart-box";
  const head = document.createElement("h3");
  head.textContent = title;
  box.appendChild(head);
  const plotHost = document.createElement("div");
  box.appendChild(plotHost);
  const lines = group.lines;
  const sweepOrigin = group.orientation === "x" ? geo.x0 : geo.y0;
  const ramp = (i: number) =>
    mixColor(
      PALETTE[beltIdx % PALETTE.length],
      "#ffffff",
      lines.length > 1 ? (0.65 * i) / (lines.length - 1) : 0
    );
  const traces = lines.map((line, i) => ({
    t: line.bin_centers.map((c) => sweepOrigin + c),
    y: line.belts[beltIdx][state.strain.field],
    color: ramp(i),
  }));
  timeSeriesPlot(plotHost, {
    width: 860,
    height: 300,
    yLabel: `${state.strain.field} (%) vs bed ${group.orientation}`,
    traces,
    fixedY: { yMin: -vmax, yMax: vmax },
    xUnit: "mm",
  });
  const legend = document.createElement("div");
  legend.className = "legend";
  lines.forEach((line, i) => {
    const item = document.createElement("span");
    item.innerHTML = `<span class="swatch" style="background:${ramp(i)}"></span>${line.name}`;
    legend.appendChild(item);
  });
  box.appendChild(legend);
  return box;
}

function meanElastic(line: StrainLine, beltIdx: number): number | null {
  const kept = line.belts[beltIdx].elastic.filter((v): v is number => v !== null);
  if (!kept.length) return null;
  return kept.reduce((a, b) => a + b, 0) / kept.length;
}

function drawStrainDcBars(canvas: HTMLCanvasElement, labels: string[], values: (number | null)[]) {
  const { ctx, w, h } = hidpiCanvasContext(canvas);
  ctx.fillStyle = "#0d1117";
  ctx.fillRect(0, 0, w, h);
  const pad = { l: 46, r: 8, t: 8, b: 40 };
  const vmax = Math.max(1e-6, ...values.filter((v): v is number => v !== null).map(Math.abs));
  const y = (v: number) => pad.t + ((vmax - v) / (2 * vmax)) * (h - pad.t - pad.b);
  const slot = (w - pad.l - pad.r) / labels.length;

  ctx.font = "10px monospace";
  ctx.fillStyle = "#8a97a3";
  for (const v of [-vmax, 0, vmax]) {
    ctx.fillText(v.toFixed(1), 2, y(v) + 3);
  }
  ctx.strokeStyle = "#29313a";
  ctx.beginPath();
  ctx.moveTo(pad.l, y(0));
  ctx.lineTo(w - pad.r, y(0));
  ctx.stroke();

  values.forEach((v, i) => {
    const cx = pad.l + (i + 0.5) * slot;
    if (v !== null) {
      ctx.fillStyle = strainColor(v / vmax);
      const top = Math.min(y(0), y(v));
      ctx.fillRect(cx - slot * 0.35, top, slot * 0.7, Math.max(1, Math.abs(y(v) - y(0))));
    }
    ctx.save();
    ctx.translate(cx + 3, h - pad.b + 10);
    ctx.rotate(-Math.PI / 4);
    ctx.fillStyle = "#8a97a3";
    ctx.textAlign = "right";
    ctx.fillText(labels[i], 0, 0);
    ctx.restore();
  });
  ctx.fillStyle = "#8a97a3";
  ctx.textAlign = "left";
  ctx.fillText("mean elastic (%)", pad.l, 10);
}

function strainDcBox(title: string, beltIdx: number, lines: StrainLine[]): HTMLDivElement {
  const box = document.createElement("div");
  box.className = "chart-box";
  const head = document.createElement("h3");
  head.textContent = title;
  box.appendChild(head);
  const canvas = document.createElement("canvas");
  canvas.width = 430;
  canvas.height = 210;
  box.appendChild(canvas);
  drawStrainDcBars(
    canvas,
    lines.map(strainLineLabel),
    lines.map((line) => meanElastic(line, beltIdx))
  );
  return box;
}

/// Shared geometry for a strain map: ordered groups (sweep orientation),
/// bed-frame extents, and the belt pair names. Diffing two maps reuses the
/// selected run's geometry since both must match its signature to compare.
function strainGeometry(data: StrainData): { groups: StrainGroup[]; geo: StrainGeo; pairs: string[] } {
  const groups = strainGroups(data);
  const detail = state.strain.selected ? state.details.get(state.strain.selected) : undefined;
  const plan = (detail && detail.manifest && detail.manifest.stroke_plan) || {};
  const geo = strainBedGeometry(groups, plan);
  const pairs = data.lines[0].belts.map((b) => b.pair);
  return { groups, geo, pairs };
}

function renderStrainCharts(data: StrainData) {
  const heatmaps = mustEl("strain-heatmaps");
  const profiles = mustEl("strain-profiles");
  const dc = mustEl("strain-dc");
  const summary = mustEl("strain-summary");
  heatmaps.innerHTML = "";
  profiles.innerHTML = "";
  dc.innerHTML = "";
  if (!data.lines.length) {
    summary.textContent = "run has no lines";
    mustEl("strain-scale").innerHTML = "";
    return;
  }
  const stats = strainStats(data);
  const vmax = Math.max(
    1e-6,
    state.strain.field === "friction" ? stats.maxFriction : stats.maxElastic
  );
  summary.textContent =
    `max |elastic| ${stats.maxElastic.toFixed(1)}% · ` +
    `mean |friction| ${stats.meanFriction.toFixed(1)}%`;
  document.querySelectorAll<HTMLButtonElement>("button.strain-field-btn").forEach((btn) => {
    btn.disabled = btn.dataset.field === state.strain.field;
  });
  mustEl("strain-scale").innerHTML = strainScaleHtml(vmax);
  const { groups, geo, pairs } = strainGeometry(data);
  pairs.forEach((pair, beltIdx) => {
    for (const group of groups) {
      const title = `${pair} — ${group.title}`;
      heatmaps.appendChild(strainHeatmapBox(title, group, beltIdx, vmax, geo));
      profiles.appendChild(strainProfileBox(title, beltIdx, group, vmax, geo));
      dc.appendChild(strainDcBox(title, beltIdx, group.lines));
    }
  });
}

/// Appends a diverging diff heatmap (selected − compare) per belt×orientation
/// for every compare run whose strain signature matches the selected run's.
/// A mismatched run is named in the summary instead of drawn — subtracting
/// cells only means something when both maps bin the bed identically.
async function renderStrainDiffs(selectedData: StrainData) {
  const heatmaps = el("strain-heatmaps");
  if (!heatmaps || !state.strain.compare.size) return;
  if (!selectedData || !selectedData.lines.length) return;
  const baseName = state.strain.selected;
  if (baseName === null) return;
  const base = strainGeometry(selectedData);
  const baseSig = strainSignature(selectedData);
  const baseTag = runTag(baseName);
  const field = state.strain.field;
  const skips: string[] = [];
  for (const name of [...state.strain.compare]) {
    let cmp: StrainData;
    try {
      cmp = await ensureStrain(name);
    } catch (e) {
      skips.push(`${runTag(name)}: ${String(e)}`);
      continue;
    }
    if (state.strain.selected !== baseName || !el("strain-heatmaps")) return;
    if (!cmp || !cmp.lines.length || strainSignature(cmp) !== baseSig) {
      skips.push(`${runTag(name)}: dimensions differ`);
      continue;
    }
    const cmpGroups = strainGroups(cmp);
    const aligned: StrainGroup[] = [];
    let mismatch = false;
    for (const bg of base.groups) {
      const cg = cmpGroups.find((g) => g.orientation === bg.orientation);
      if (!cg || cg.lines.length !== bg.lines.length) {
        mismatch = true;
        break;
      }
      aligned.push(cg);
    }
    if (mismatch || aligned.length !== base.groups.length || state.strain.selected !== baseName) {
      skips.push(`${runTag(name)}: layout mismatch`);
      continue;
    }
    const cmpTag = runTag(name);
    let vmaxDiff = 1e-6;
    for (let gi = 0; gi < base.groups.length; gi++) {
      const bg = base.groups[gi];
      const cg = aligned[gi];
      for (let li = 0; li < bg.lines.length; li++) {
        for (let bi = 0; bi < bg.lines[li].belts.length; bi++) {
          for (const v of pointwiseDiff(bg.lines[li].belts[bi][field], cg.lines[li].belts[bi][field])) {
            if (v !== null) vmaxDiff = Math.max(vmaxDiff, Math.abs(v));
          }
        }
      }
    }
    base.pairs.forEach((pair, beltIdx) => {
      if (state.strain.selected !== baseName || !el("strain-heatmaps")) return;
      for (let gi = 0; gi < base.groups.length; gi++) {
        const bg = base.groups[gi];
        const cg = aligned[gi];
        const diffLines = bg.lines.map((bl, li) => buildDiffLine(bl, cg.lines[li]));
        const diffGroup = { orientation: bg.orientation, title: bg.title, lines: diffLines };
        const title = `Δ ${baseTag} − ${cmpTag} · ${pair} — ${bg.title}`;
        heatmaps.appendChild(strainHeatmapBox(title, diffGroup, beltIdx, vmaxDiff, base.geo));
      }
    });
    if (state.strain.selected === baseName && el("strain-heatmaps")) {
      const scaleEl = document.createElement("div");
      scaleEl.className = "strain-scale";
      scaleEl.style.gridColumn = "1 / -1";
      const stops: string[] = [];
      for (let i = 0; i <= 8; i++) stops.push(strainColor(i / 4 - 1));
      scaleEl.innerHTML =
        `<span>−${vmaxDiff.toFixed(1)}%</span>` +
        `<span class="bar" style="background:linear-gradient(90deg,${stops.join(",")})"></span>` +
        `<span>+${vmaxDiff.toFixed(1)}%</span>` +
        `<span class="hint">Δ scale for ${cmpTag} — ${field}, null bins stay dark</span>`;
      heatmaps.appendChild(scaleEl);
    }
  }
  if (skips.length) {
    const note = el("strain-summary");
    if (note) note.textContent += `  · skipped: ${skips.join("; ")}`;
  }
}

async function redrawStrain() {
  renderStrainRuns();
  if (!el("strain-heatmaps")) return;
  const summary = mustEl("strain-summary");
  const name = state.strain.selected;
  if (!name) {
    summary.textContent = "no strain_map runs yet — run SERVO_STRAIN_MAP first";
    mustEl("strain-heatmaps").innerHTML = "";
    mustEl("strain-scale").innerHTML = "";
    mustEl("strain-profiles").innerHTML = "";
    mustEl("strain-dc").innerHTML = "";
    return;
  }
  let data: StrainData;
  try {
    data = await ensureStrain(name);
  } catch (e) {
    summary.textContent = String(e);
    return;
  }
  if (state.strain.selected !== name || !el("strain-heatmaps")) return;
  renderStrainCharts(data);
  await renderStrainDiffs(data);
}

export { STRAIN_NEUTRAL, STRAIN_NEG, STRAIN_POS, STRAIN_HEAT_PAD, STRAIN_HEAT_CANVAS_W, STRAIN_LINE_SPACING_FALLBACK_MM, strainShellHtml, ensureStrain, runTag, buildDiffLine, pointwiseDiff, nulls, strainSignature, renderStrainRuns, strainColor, sweptEntry, strainLineLabel, strainGroups, strainBedGeometry, strainStats, lineBinWidth, drawStrainHeatmap, strainHeatmapBox, strainScaleHtml, strainProfileBox, meanElastic, drawStrainDcBars, strainDcBox, strainGeometry, renderStrainCharts, renderStrainDiffs, redrawStrain };
