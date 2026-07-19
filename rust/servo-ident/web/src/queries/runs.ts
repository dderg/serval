import { useMutation, QueryObserver } from "@tanstack/preact-query";
import { queryClient } from "./client";
import { state } from "../state";
import type { PageDef } from "../state";
import {
  listRuns,
  getRunManifest,
  getRunResults,
  getRunPlotSeries,
  postRunNote,
  postRunAnalyze,
  deleteRun as apiDeleteRun,
} from "../api/runs";
import type { Manifest, PlotSeries, Results, RunSummary } from "../api/runs";

const RUNS_POLL_MS = 5000;

export const runKeys = {
  all: ["runs"] as const,
  run: (name: string) => ["runs", name] as const,
  detail: (name: string) => ["runs", name, "detail"] as const,
  plot: (name: string) => ["runs", name, "plot"] as const,
};

export interface RunDetail {
  mtime_utc: string;
  has_results: boolean;
  manifest: Manifest | null;
  results: Results | null;
}

export function runsData(): RunSummary[] {
  return queryClient.getQueryData<RunSummary[]>(runKeys.all) ?? [];
}

export function runData(name: string): RunSummary | undefined {
  return runsData().find((r) => r.name === name);
}

export function runDataSig(names: string[]) {
  return names.map((n) => {
    const run = runData(n);
    return [n, run ? run.mtime_utc : null, state.runColors.get(n) || null];
  });
}

export function detailData(name: string): RunDetail | undefined {
  return queryClient.getQueryData<RunDetail>(runKeys.detail(name));
}

export async function ensureDetail(run: RunSummary): Promise<RunDetail> {
  const cached = detailData(run.name);
  if (cached && cached.mtime_utc === run.mtime_utc && cached.has_results === run.has_results) {
    return cached;
  }
  const manifest = await getRunManifest(run.name);
  const results = run.has_results ? await getRunResults(run.name) : null;
  const detail: RunDetail = {
    mtime_utc: run.mtime_utc,
    has_results: run.has_results,
    manifest,
    results,
  };
  queryClient.setQueryData(runKeys.detail(run.name), detail);
  return detail;
}

export async function ensurePlotSeries(name: string): Promise<PlotSeries> {
  const run = runData(name);
  const cached = queryClient.getQueryData<{ mtime_utc: string | null; data: PlotSeries }>(
    runKeys.plot(name),
  );
  if (cached && run && cached.mtime_utc === run.mtime_utc) return cached.data;
  const data = await getRunPlotSeries(name);
  queryClient.setQueryData(runKeys.plot(name), { mtime_utc: run ? run.mtime_utc : null, data });
  return data;
}

export function pageRuns(def: PageDef): RunSummary[] {
  const experiments = def.experiments;
  if (!experiments) return runsData();
  return runsData().filter((r) => experiments.includes(r.experiment));
}

function runsUnchanged(prev: RunSummary[], next: RunSummary[]): boolean {
  return (
    next.length === prev.length &&
    next.every((run, i) => {
      const current = prev[i];
      const rv = run.verdict ?? null;
      const cv = current.verdict ?? null;
      const verdictsMatch =
        rv === cv ||
        (rv !== null &&
          cv !== null &&
          rv.recommended_step === cv.recommended_step &&
          rv.reason === cv.reason &&
          rv.flags.length === cv.flags.length &&
          rv.flags.every((flag, j) => flag === cv.flags[j]));
      return (
        run.name === current.name &&
        run.mtime_utc === current.mtime_utc &&
        run.experiment === current.experiment &&
        run.tag === current.tag &&
        run.axis === current.axis &&
        run.has_results === current.has_results &&
        verdictsMatch &&
        run.note === current.note
      );
    })
  );
}

async function fetchRuns(): Promise<RunSummary[]> {
  const prev = runsData();
  let runs: RunSummary[];
  try {
    runs = await listRuns();
  } catch (e) {
    console.error(e);
    throw e;
  }
  if (runsUnchanged(prev, runs)) return prev;
  await Promise.all(runs.map((r) => ensureDetail(r).catch((e) => console.error(e))));
  return runs;
}

export function runsQuery() {
  return { queryKey: runKeys.all, queryFn: fetchRuns };
}

let runsObserver: QueryObserver<RunSummary[]> | null = null;

export function startRunsPolling(onData?: (runs: RunSummary[]) => void) {
  if (runsObserver) return;
  runsObserver = new QueryObserver<RunSummary[]>(queryClient, {
    ...runsQuery(),
    refetchInterval: RUNS_POLL_MS,
    refetchIntervalInBackground: false,
  });
  let lastData: RunSummary[] | undefined;
  runsObserver.subscribe((result) => {
    if (onData && result.isSuccess && result.data && result.data !== lastData) {
      lastData = result.data;
      onData(result.data);
    }
  });
}

export function useSaveNote() {
  return useMutation(
    {
      mutationFn: ({ name, text }: { name: string; text: string }) => postRunNote(name, text),
      onMutate: ({ name, text }: { name: string; text: string }) => {
        const optimistic = text.trim() || null;
        const previous = queryClient.getQueryData<RunSummary[]>(runKeys.all);
        queryClient.setQueryData<RunSummary[]>(runKeys.all, (runs) =>
          runs ? runs.map((r) => (r.name === name ? { ...r, note: optimistic } : r)) : runs,
        );
        return { previous };
      },
      onSuccess: (saved, { name }) => {
        queryClient.setQueryData<RunSummary[]>(runKeys.all, (runs) =>
          runs ? runs.map((r) => (r.name === name ? { ...r, note: saved.note || null } : r)) : runs,
        );
      },
      onError: (e, _vars, ctx) => {
        console.error(e);
        alert(`saving note failed: ${e instanceof Error ? e.message : e}`);
        if (ctx?.previous) queryClient.setQueryData(runKeys.all, ctx.previous);
        void queryClient.invalidateQueries({ queryKey: runKeys.all });
      },
    },
    queryClient,
  );
}

export function useDeleteRun() {
  return useMutation(
    {
      mutationFn: (name: string) => apiDeleteRun(name),
      onSuccess: async (_data, name) => {
        queryClient.removeQueries({ queryKey: runKeys.run(name) });
        const remaining = runsData().filter((r) => r.name !== name);
        queryClient.setQueryData(runKeys.all, remaining);
        await queryClient.invalidateQueries({ queryKey: runKeys.all });
      },
      onError: (e, name) => {
        console.error(e);
        alert(`deleting ${name} failed: ${e instanceof Error ? e.message : e}`);
      },
    },
    queryClient,
  );
}

export function useAnalyzeRun() {
  return useMutation(
    {
      mutationFn: (name: string) => postRunAnalyze(name),
      onSuccess: () => queryClient.invalidateQueries({ queryKey: runKeys.all }),
    },
    queryClient,
  );
}
