import { html } from "htm/preact";
import { useEffect, useRef } from "preact/hooks";
import { useQuery, useQueries } from "@tanstack/preact-query";
import { shortTime } from "./api";
import { detailData, pageRuns, runData } from "./queries/runs";
import { ensureStrain, strainKey } from "./queries/strain";
import { hidpiCanvasContext, mixColor } from "./charts-core";
import { timeSeriesPlot } from "./uplot-chart";
import { loadRerunForm } from "./drive";
import { ConsolePanel } from "./console";
import { LaunchpadPad } from "./launchpad";
import { applyAccordionState } from "./shell";
import { PALETTE, state } from "./state";
import { notify, useStore } from "./store";
import type { PageDef } from "./state";
import type { StrokePlan, StrainField } from "./wire";
import type { StrainMap as StrainData, RunSummary } from "./api/runs";
import type { VNode } from "preact";

type StrainLine = StrainData["lines"][number];

// --- strain map (strain page) -------------------------------------------------
//
// Renders getRunStrain: per raster line, the elastic
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


function runTag(name: string): string {
  const r = runData(name);
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




function strainColor(t: number): string {
  const clamped = Math.max(-1, Math.min(1, t));
  return clamped < 0
    ? mixColor(STRAIN_NEUTRAL, STRAIN_NEG, -clamped)
    : mixColor(STRAIN_NEUTRAL, STRAIN_POS, clamped);
}

function sweptEntry(line: StrainLine): [string, number] {
  const swept = line.swept;
  if (!swept || typeof swept !== "object") return ["?", 0];
  const entries = Object.entries(swept);
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


/// Shared geometry for a strain map: ordered groups (sweep orientation),
/// bed-frame extents, and the belt pair names. Diffing two maps reuses the
/// selected run's geometry since both must match its signature to compare.
function strainGeometry(data: StrainData): { groups: StrainGroup[]; geo: StrainGeo; pairs: string[] } {
  const groups = strainGroups(data);
  const detail = state.strain.selected ? detailData(state.strain.selected) : undefined;
  const plan = (detail && detail.manifest && detail.manifest.stroke_plan) || {};
  const geo = strainBedGeometry(groups, plan);
  const pairs = data.lines[0].belts.map((b) => b.pair);
  return { groups, geo, pairs };
}


function reconcileStrainSelection(runs: RunSummary[]): boolean {
  let changed = false;
  if (!runs.some((r) => r.name === state.strain.selected)) {
    const next = runs.length ? runs[0].name : null;
    if (state.strain.selected !== next) {
      state.strain.selected = next;
      changed = true;
    }
  }
  const known = new Set(runs.map((r) => r.name));
  for (const name of [...state.strain.compare]) {
    if (!known.has(name)) {
      state.strain.compare.delete(name);
      changed = true;
    }
  }
  return changed;
}

function setStrainField(field: StrainField) {
  state.strain.field = field;
  notify();
}

function onStrainRowClick(name: string, ev: MouseEvent) {
  if (ev.shiftKey) {
    if (name === state.strain.selected) return;
    if (state.strain.compare.has(name)) state.strain.compare.delete(name);
    else state.strain.compare.add(name);
  } else {
    state.strain.selected = name;
    state.strain.compare.clear();
  }
  notify();
}

interface StrainDiffBox {
  key: string;
  title: string;
  group: StrainGroup;
  beltIdx: number;
  vmax: number;
  geo: StrainGeo;
}

interface StrainDiffScale {
  key: string;
  cmpTag: string;
  vmax: number;
}

type StrainDiffNode =
  | ({ kind: "heat" } & StrainDiffBox)
  | ({ kind: "scale" } & StrainDiffScale);

interface StrainCompareResult {
  name: string;
  data: StrainData | null;
  error: unknown;
}

function computeStrainDiffs(
  selectedData: StrainData,
  compareResults: StrainCompareResult[]
): { nodes: StrainDiffNode[]; skips: string[] } {
  const nodes: StrainDiffNode[] = [];
  const skips: string[] = [];
  if (!selectedData.lines.length) return { nodes, skips };
  const baseName = state.strain.selected;
  if (baseName === null) return { nodes, skips };
  const base = strainGeometry(selectedData);
  const baseSig = strainSignature(selectedData);
  const baseTag = runTag(baseName);
  const field = state.strain.field;
  for (const { name, data, error } of compareResults) {
    if (error) {
      skips.push(`${runTag(name)}: ${String(error)}`);
      continue;
    }
    const cmp = data;
    if (!cmp) continue;
    if (!cmp.lines.length || strainSignature(cmp) !== baseSig) {
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
    if (mismatch || aligned.length !== base.groups.length) {
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
      for (let gi = 0; gi < base.groups.length; gi++) {
        const bg = base.groups[gi];
        const cg = aligned[gi];
        const diffLines = bg.lines.map((bl, li) => buildDiffLine(bl, cg.lines[li]));
        const diffGroup: StrainGroup = { orientation: bg.orientation, title: bg.title, lines: diffLines };
        nodes.push({
          kind: "heat",
          key: `diff-${cmpTag}-${pair}-${bg.orientation}`,
          title: `Δ ${baseTag} − ${cmpTag} · ${pair} — ${bg.title}`,
          group: diffGroup,
          beltIdx,
          vmax: vmaxDiff,
          geo: base.geo,
        });
      }
    });
    nodes.push({ kind: "scale", key: `diff-scale-${cmpTag}`, cmpTag, vmax: vmaxDiff });
  }
  return { nodes, skips };
}

function HeatmapCanvas({ title, group, beltIdx, vmax, geo }: StrainDiffBox) {
  const ref = useRef<HTMLCanvasElement>(null);
  const field = state.strain.field;
  useEffect(() => {
    const canvas = ref.current;
    if (canvas) drawStrainHeatmap(canvas, group, beltIdx, vmax, geo);
  }, [group, beltIdx, vmax, geo, field]);
  const pad = STRAIN_HEAT_PAD;
  const plotW = STRAIN_HEAT_CANVAS_W - pad.l - pad.r;
  const plotH = plotW * ((geo.yhi - geo.ylo) / (geo.xhi - geo.xlo));
  const height = Math.round(pad.t + pad.b + plotH);
  return html`<div class="chart-box">
    <h3>${title}</h3>
    <canvas ref=${ref} width=${STRAIN_HEAT_CANVAS_W} height=${height}></canvas>
  </div>`;
}

function DcBarsCanvas({ title, labels, values }: { title: string; labels: string[]; values: (number | null)[] }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const canvas = ref.current;
    if (canvas) drawStrainDcBars(canvas, labels, values);
  }, [labels, values]);
  return html`<div class="chart-box">
    <h3>${title}</h3>
    <canvas ref=${ref} width="430" height="210"></canvas>
  </div>`;
}

function profileRamp(beltIdx: number, count: number, i: number): string {
  return mixColor(
    PALETTE[beltIdx % PALETTE.length],
    "#ffffff",
    count > 1 ? (0.65 * i) / (count - 1) : 0
  );
}

function ProfileChart({ title, beltIdx, group, vmax, geo }: StrainDiffBox) {
  const hostRef = useRef<HTMLDivElement>(null);
  const field = state.strain.field;
  const lines = group.lines;
  const sweepOrigin = group.orientation === "x" ? geo.x0 : geo.y0;
  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    const traces = lines.map((line, i) => ({
      t: line.bin_centers.map((c) => sweepOrigin + c),
      y: line.belts[beltIdx][field],
      color: profileRamp(beltIdx, lines.length, i),
    }));
    const plot = timeSeriesPlot(host, {
      width: 860,
      height: 300,
      yLabel: `${field} (%) vs bed ${group.orientation}`,
      traces,
      fixedY: { yMin: -vmax, yMax: vmax },
      xUnit: "mm",
    });
    return () => plot.u.destroy();
  }, [group, beltIdx, vmax, geo, field]);
  return html`<div class="chart-box">
    <h3>${title}</h3>
    <div ref=${hostRef}></div>
    <div class="legend">
      ${lines.map(
        (line, i) => html`<span key=${line.name}
          ><span class="swatch" style=${`background:${profileRamp(beltIdx, lines.length, i)}`}></span>${line.name}</span
        >`
      )}
    </div>
  </div>`;
}

function ScaleBar({ vmax }: { vmax: number }) {
  const stops: string[] = [];
  for (let i = 0; i <= 8; i++) stops.push(strainColor(i / 4 - 1));
  const what =
    state.strain.field === "friction"
      ? "friction (direction-dependent) differential torque"
      : "elastic differential torque";
  return html`<div class="strain-scale">
    <span>−${vmax.toFixed(1)}%</span>
    <span class="bar" style=${`background:linear-gradient(90deg,${stops.join(",")})`}></span>
    <span>+${vmax.toFixed(1)}%</span>
    <span class="hint">${what}, % rated — null bins stay dark</span>
  </div>`;
}

function DiffScaleBar({ cmpTag, vmax }: { cmpTag: string; vmax: number }) {
  const stops: string[] = [];
  for (let i = 0; i <= 8; i++) stops.push(strainColor(i / 4 - 1));
  return html`<div class="strain-scale" style="grid-column:1 / -1">
    <span>−${vmax.toFixed(1)}%</span>
    <span class="bar" style=${`background:linear-gradient(90deg,${stops.join(",")})`}></span>
    <span>+${vmax.toFixed(1)}%</span>
    <span class="hint">Δ scale for ${cmpTag} — ${state.strain.field}, null bins stay dark</span>
  </div>`;
}

function StrainRunsTable({ def }: { def: PageDef }) {
  useStore();
  const runs = pageRuns(def);
  return html`<tbody id="strain-run-body">
    ${runs.map((run) => {
      const cls = ["selectable"];
      if (run.name === state.strain.selected) cls.push("selected");
      if (state.strain.compare.has(run.name)) cls.push("compare");
      const manifest = detailData(run.name)?.manifest;
      return html`<tr key=${run.name} class=${cls.join(" ")} onClick=${(ev: MouseEvent) => onStrainRowClick(run.name, ev)}>
        <td title=${`${run.name} — ${run.mtime_utc}`}>${shortTime(run.mtime_utc)}</td>
        <td>${runTag(run.name)}</td>
        <td class="actions">
          <button
            title="prefill the console with this run's command"
            disabled=${!manifest}
            onClick=${(e: MouseEvent) => {
              e.stopPropagation();
              loadRerunForm(run.name);
            }}
          >
            → console
          </button>
        </td>
      </tr>`;
    })}
  </tbody>`;
}

function StrainRunsSection({ def }: { def: PageDef }) {
  return html`<section class="runs-section">
    <div class="section-head"><h2>strain runs</h2></div>
    <div class="section-tools">
      <span class="note">strain_map — click to map, shift+click a second run to diff (matching dimensions)</span>
    </div>
    <div class="table-wrap runs-wrap">
      <table>
        <thead>
          <tr><th>time</th><th>tag</th><th></th></tr>
        </thead>
        <${StrainRunsTable} def=${def} />
      </table>
    </div>
  </section>`;
}

function StrainMapSection({ name, data, error }: { name: string | null; data: StrainData | null; error: unknown }) {
  useStore();
  const field = state.strain.field;
  const compareNames = [...state.strain.compare];
  const compareQueries = useQueries({
    queries: compareNames.map((cn) => ({
      queryKey: [...strainKey(cn), "view"],
      queryFn: () => ensureStrain(cn),
      enabled: data != null && data.lines.length > 0,
    })),
  });

  let summaryText = "";
  const heatNodes: VNode[] = [];
  let scaleNode: VNode | null = null;
  if (name == null) {
    summaryText = "no strain_map runs yet — run SERVO_STRAIN_MAP first";
  } else if (error && !data) {
    summaryText = String(error);
  } else if (data && !data.lines.length) {
    summaryText = "run has no lines";
  } else if (data) {
    const stats = strainStats(data);
    const vmax = Math.max(1e-6, field === "friction" ? stats.maxFriction : stats.maxElastic);
    summaryText =
      `max |elastic| ${stats.maxElastic.toFixed(1)}% · ` +
      `mean |friction| ${stats.meanFriction.toFixed(1)}%`;
    const { groups, geo, pairs } = strainGeometry(data);
    pairs.forEach((pair, beltIdx) => {
      for (const group of groups) {
        heatNodes.push(
          html`<${HeatmapCanvas}
            key=${`m-${pair}-${group.orientation}`}
            title=${`${pair} — ${group.title}`}
            group=${group}
            beltIdx=${beltIdx}
            vmax=${vmax}
            geo=${geo}
          />`
        );
      }
    });
    scaleNode = html`<${ScaleBar} vmax=${vmax} />`;
    const compareResults: StrainCompareResult[] = compareNames.map((cn, i) => ({
      name: cn,
      data: (compareQueries[i]?.data as StrainData | undefined) ?? null,
      error: compareQueries[i]?.error ?? null,
    }));
    const { nodes, skips } = computeStrainDiffs(data, compareResults);
    for (const node of nodes) {
      if (node.kind === "heat") {
        heatNodes.push(
          html`<${HeatmapCanvas}
            key=${node.key}
            title=${node.title}
            group=${node.group}
            beltIdx=${node.beltIdx}
            vmax=${node.vmax}
            geo=${node.geo}
          />`
        );
      } else {
        heatNodes.push(html`<${DiffScaleBar} key=${node.key} cmpTag=${node.cmpTag} vmax=${node.vmax} />`);
      }
    }
    if (skips.length) summaryText += `  · skipped: ${skips.join("; ")}`;
  }

  return html`<section>
    <div class="section-head"><h2>strain map</h2></div>
    <div class="section-tools">
      <button class="strain-field-btn" data-field="elastic" disabled=${field === "elastic"} onClick=${() => setStrainField("elastic")}>
        elastic
      </button>
      <button
        class="strain-field-btn"
        data-field="friction"
        title="the direction-dependent half: (forward - backward)/2 — what a position-keyed offset cannot cancel"
        disabled=${field === "friction"}
        onClick=${() => setStrainField("friction")}
      >
        friction
      </button>
      <span class="note" id="strain-summary">${summaryText}</span>
    </div>
    <div id="strain-heatmaps" class="strain-grid">${heatNodes}</div>
    <div id="strain-scale">${scaleNode}</div>
  </section>`;
}

function StrainProfilesSection({ data }: { data: StrainData | null }) {
  useStore();
  const field = state.strain.field;
  const boxes: VNode[] = [];
  if (data && data.lines.length) {
    const stats = strainStats(data);
    const vmax = Math.max(1e-6, field === "friction" ? stats.maxFriction : stats.maxElastic);
    const { groups, geo, pairs } = strainGeometry(data);
    pairs.forEach((pair, beltIdx) => {
      for (const group of groups) {
        boxes.push(
          html`<${ProfileChart}
            key=${`p-${pair}-${group.orientation}`}
            title=${`${pair} — ${group.title}`}
            group=${group}
            beltIdx=${beltIdx}
            vmax=${vmax}
            geo=${geo}
          />`
        );
      }
    });
  }
  return html`<section>
    <div class="section-head"><h2>per-line elastic profiles</h2></div>
    <div class="section-tools"><span class="note">one polyline per raster line, in bed coordinates</span></div>
    <div class="charts" id="strain-profiles">${boxes}</div>
  </section>`;
}

function StrainDcSection({ data }: { data: StrainData | null }) {
  useStore();
  const boxes: VNode[] = [];
  if (data && data.lines.length) {
    const { groups, pairs } = strainGeometry(data);
    pairs.forEach((pair, beltIdx) => {
      for (const group of groups) {
        boxes.push(
          html`<${DcBarsCanvas}
            key=${`d-${pair}-${group.orientation}`}
            title=${`${pair} — ${group.title}`}
            labels=${group.lines.map(strainLineLabel)}
            values=${group.lines.map((line) => meanElastic(line, beltIdx))}
          />`
        );
      }
    });
  }
  return html`<section>
    <div class="section-head"><h2>per-line DC offset — mean elastic</h2></div>
    <div class="section-tools"><span class="note">a line-to-line offset is trapped preload, not local strain</span></div>
    <div class="strain-grid" id="strain-dc">${boxes}</div>
  </section>`;
}

function StrainMain({ def }: { def: PageDef }) {
  useStore();
  const runs = pageRuns(def);
  const namesSig = runs.map((r) => r.name).join("|");
  useEffect(() => {
    if (reconcileStrainSelection(runs)) notify();
  }, [namesSig]);
  const name = state.strain.selected;
  const selected = useQuery({
    queryKey: [...strainKey(name ?? ""), "view"],
    queryFn: () => ensureStrain(name as string),
    enabled: name != null,
  });
  const data = name != null ? ((selected.data as StrainData | undefined) ?? null) : null;
  const error = name != null ? selected.error : null;
  return html`<main class="analysis">
    <${StrainRunsSection} def=${def} />
    <${StrainMapSection} name=${name} data=${data} error=${error} />
    <${StrainProfilesSection} data=${data} />
    <${StrainDcSection} data=${data} />
  </main>`;
}

function StrainPage({ def }: { def: PageDef }) {
  useEffect(() => {
    applyAccordionState();
  }, []);
  return html`<div class="workspace">
    <${StrainMain} def=${def} />
    <aside class="controls">
      <${ConsolePanel} templates=${def.templates} />
      <${LaunchpadPad} />
    </aside>
  </div>`;
}


export { STRAIN_NEUTRAL, STRAIN_NEG, STRAIN_POS, STRAIN_HEAT_PAD, STRAIN_HEAT_CANVAS_W, STRAIN_LINE_SPACING_FALLBACK_MM, runTag, buildDiffLine, pointwiseDiff, nulls, strainSignature, strainColor, sweptEntry, strainLineLabel, strainGroups, strainBedGeometry, strainStats, lineBinWidth, drawStrainHeatmap, meanElastic, drawStrainDcBars, strainGeometry, reconcileStrainSelection, computeStrainDiffs, StrainPage };
