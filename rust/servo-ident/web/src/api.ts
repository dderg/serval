import { state } from "./state";
import type { Manifest, ManifestAmbient, NotchStateValue, PlotSeries, RunSummary } from "./wire";
import type { PageDef } from "./state";

async function api(path: string, opts?: RequestInit): Promise<any> {
  // TODO(bindings): callers cast this; typed endpoint wrappers arrive with
  // the generated wire bindings.
  const resp = await fetch(path, opts);
  const text = await resp.text();
  if (!resp.ok) {
    throw new Error(`${path}: HTTP ${resp.status}: ${text}`);
  }
  return text.length ? JSON.parse(text) : null;
}

function el<T extends HTMLElement = HTMLElement>(id: string): T | null {
  return document.getElementById(id) as T | null;
}

/// Same lookup for elements the current page is guaranteed to have —
/// a missing id is a template bug, so it throws instead of returning null.
function mustEl<T extends HTMLElement = HTMLElement>(id: string): T {
  const found = el<T>(id);
  if (!found) throw new Error(`#${id}: element missing from the page`);
  return found;
}

// --- render gating ----------------------------------------------------------
//
// Every periodic render short-circuits through payloadUnchanged: a section
// whose inputs serialize to the same signature as its last render keeps its
// DOM (and any in-progress interaction) untouched. renderPage wipes the DOM
// wholesale, so it calls resetRenderState to drop every signature and let
// registered hooks discard DOM-bound caches.

const renderSigs = new Map<string, string>();
const renderResetHooks: (() => void)[] = [];

function payloadUnchanged(key: string, payload: unknown): boolean {
  const sig = JSON.stringify(payload);
  if (renderSigs.get(key) === sig) return true;
  renderSigs.set(key, sig);
  return false;
}

function onRenderReset(hook: () => void): void {
  renderResetHooks.push(hook);
}

function resetRenderState(): void {
  renderSigs.clear();
  for (const hook of renderResetHooks) hook();
}

function runDataSig(names: string[]) {
  return names.map((n) => {
    const run = state.runs.find((r) => r.name === n);
    return [n, run ? run.mtime_utc : null, state.runColors.get(n) || null];
  });
}

// --- run data ---------------------------------------------------------------

function detailIsFresh(name: string, run: RunSummary): boolean {
  const cached = state.details.get(name);
  if (!cached) return false;
  return cached.mtime_utc === run.mtime_utc && cached.has_results === run.has_results;
}

async function ensureDetail(run: RunSummary): Promise<void> {
  if (detailIsFresh(run.name, run)) return;
  const manifest = await api(`/api/runs/${encodeURIComponent(run.name)}/manifest`);
  const results = run.has_results
    ? await api(`/api/runs/${encodeURIComponent(run.name)}/results`)
    : null;
  state.details.set(run.name, {
    mtime_utc: run.mtime_utc,
    has_results: run.has_results,
    manifest,
    results,
  });
}

async function ensurePlotSeries(name: string): Promise<PlotSeries> {
  const run = state.runs.find((r) => r.name === name);
  const cached = state.plotSeries.get(name);
  if (cached && run && cached.mtime_utc === run.mtime_utc) return cached.data;
  const data: PlotSeries = await api(`/api/runs/${encodeURIComponent(name)}/plot_series`);
  state.plotSeries.set(name, { mtime_utc: run ? run.mtime_utc : null, data });
  return data;
}

type FlatAmbient = Record<string, Record<string, number | string | NotchStateValue>>;

function journalParams(manifest: Manifest | null): NonNullable<ManifestAmbient["journal_params"]> {
  return (manifest && manifest.ambient && manifest.ambient.journal_params) || {};
}

function ambientNotches(manifest: Manifest | null): ManifestAmbient["notches"] {
  return (manifest && manifest.ambient && manifest.ambient.notches) || null;
}

function flatAmbient(manifest: Manifest | null, includeNotches: boolean): FlatAmbient {
  const flat: FlatAmbient = {};
  for (const [motor, addrs] of Object.entries(journalParams(manifest))) {
    flat[motor] = { ...addrs };
  }
  if (!includeNotches) return flat;
  for (const [motor, notchState] of Object.entries(ambientNotches(manifest) || {})) {
    const dst = (flat[motor] = flat[motor] || {});
    for (const [key, value] of Object.entries(notchState)) {
      if (value && typeof value === "object") {
        for (const [field, v] of Object.entries(value)) dst[`${key}.${field}`] = v;
      } else {
        dst[`notch_${key}`] = value;
      }
    }
  }
  return flat;
}

function motorCount(journal: FlatAmbient): number {
  return Object.keys(journal).length;
}

function ambientDiff(prevManifest: Manifest | null, curManifest: Manifest | null): string {
  // Notch state only diffs when both runs recorded it - a null/legacy
  // previous manifest would otherwise flood the column with ?->value
  // lines for every notch field.
  const bothNotches = !!(ambientNotches(prevManifest) && ambientNotches(curManifest));
  const prev = flatAmbient(prevManifest, bothNotches);
  const cur = flatAmbient(curManifest, bothNotches);
  const multiMotor = motorCount(prev) > 1 || motorCount(cur) > 1;
  const parts: string[] = [];
  const motors = new Set([...Object.keys(prev), ...Object.keys(cur)]);
  for (const motor of [...motors].sort()) {
    const prevAddrs = prev[motor] || {};
    const curAddrs = cur[motor] || {};
    const addrs = new Set([...Object.keys(prevAddrs), ...Object.keys(curAddrs)]);
    for (const addr of [...addrs].sort()) {
      const before = prevAddrs[addr];
      const after = curAddrs[addr];
      if (before === after) continue;
      const label = multiMotor ? `${motor}.${addr}` : addr;
      const beforeText = before === undefined ? "?" : before;
      const afterText = after === undefined ? "?" : after;
      parts.push(`${label}: ${beforeText}→${afterText}`);
    }
  }
  return parts.join(", ");
}

function pageRuns(def: PageDef): RunSummary[] {
  const experiments = def.experiments;
  if (!experiments) return state.runs;
  return state.runs.filter((r) => experiments.includes(r.experiment));
}

function shortTime(mtimeUtc: string): string {
  const m = /T(\d{2}:\d{2}:\d{2})/.exec(mtimeUtc);
  return m ? m[1] : mtimeUtc;
}

export { api, el, mustEl, payloadUnchanged, onRenderReset, resetRenderState, runDataSig, detailIsFresh, ensureDetail, ensurePlotSeries, journalParams, ambientNotches, flatAmbient, motorCount, ambientDiff, pageRuns, shortTime };
