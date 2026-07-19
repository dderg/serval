import type { ManifestAmbient, NotchStateValue } from "./wire";
import type { Manifest } from "./api/runs";

function el<T extends HTMLElement = HTMLElement>(id: string): T | null {
  return document.getElementById(id) as T | null;
}

function mustEl<T extends HTMLElement = HTMLElement>(id: string): T {
  const found = el<T>(id);
  if (!found) throw new Error(`#${id}: element missing from the page`);
  return found;
}

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

function shortTime(mtimeUtc: string): string {
  const m = /T(\d{2}:\d{2}:\d{2})/.exec(mtimeUtc);
  return m ? m[1] : mtimeUtc;
}

export { el, mustEl, payloadUnchanged, onRenderReset, resetRenderState, journalParams, ambientNotches, flatAmbient, motorCount, ambientDiff, shortTime };
