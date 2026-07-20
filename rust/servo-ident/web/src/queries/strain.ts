import { getRunStrain } from "../api/runs";
import type { StrainMap } from "../api/runs";
import { queryClient } from "./client";
import { runData, runKeys } from "./runs";

export const strainKey = (name: string) => [...runKeys.run(name), "strain"] as const;
export const strainViewKey = (name: string) =>
  [...strainKey(name), "view", runData(name)?.mtime_utc ?? null] as const;

interface StrainCacheEntry {
  mtime_utc: string | null;
  data: StrainMap;
}

export async function ensureStrain(name: string): Promise<StrainMap> {
  const run = runData(name);
  const cached = queryClient.getQueryData<StrainCacheEntry>(strainKey(name));
  if (cached && run && cached.mtime_utc === run.mtime_utc) return cached.data;
  const data = await getRunStrain(name);
  queryClient.setQueryData<StrainCacheEntry>(strainKey(name), {
    mtime_utc: run ? run.mtime_utc : null,
    data,
  });
  return data;
}
