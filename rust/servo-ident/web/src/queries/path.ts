import { getRunPath } from "../api/runs";
import type { RunPath } from "../api/runs";
import { queryClient } from "./client";
import { runData, runKeys } from "./runs";

interface PathCacheEntry {
  mtime_utc: string | null;
  data: RunPath | null;
  error: string | null;
}

const runPathKey = (name: string) => [...runKeys.run(name), "path"] as const;

const pendingFetches = new Set<string>();

function freshFullEntry(name: string): PathCacheEntry | null {
  const run = runData(name);
  const cached = queryClient.getQueryData<PathCacheEntry>(runPathKey(name));
  if (cached && run && cached.mtime_utc === run.mtime_utc) return cached;
  return null;
}

function readyFullPaths(names: string[]): Map<string, RunPath> {
  const map = new Map<string, RunPath>();
  for (const name of names) {
    const entry = freshFullEntry(name);
    if (entry && entry.data) map.set(name, entry.data);
  }
  return map;
}

function ensureFullPaths(names: string[], onSettled: () => void) {
  for (const name of names) {
    if (freshFullEntry(name) || pendingFetches.has(name)) continue;
    pendingFetches.add(name);
    const run = runData(name);
    const mtime = run ? run.mtime_utc : null;
    getRunPath(name)
      .then((data) => {
        queryClient.setQueryData<PathCacheEntry>(runPathKey(name), {
          mtime_utc: mtime,
          data,
          error: null,
        });
      })
      .catch((e) => {
        queryClient.setQueryData<PathCacheEntry>(runPathKey(name), {
          mtime_utc: mtime,
          data: null,
          error: String(e),
        });
      })
      .finally(() => {
        pendingFetches.delete(name);
        onSettled();
      });
  }
}

function isPathPending(name: string): boolean {
  return pendingFetches.has(name);
}

function freshPathError(name: string): string | null {
  const entry = freshFullEntry(name);
  return entry ? entry.error : null;
}

export { runPathKey, readyFullPaths, ensureFullPaths, isPathPending, freshPathError };
export type { PathCacheEntry };
