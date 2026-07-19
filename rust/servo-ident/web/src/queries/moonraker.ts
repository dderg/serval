import { getGcodeHelp, getServerInfo } from "../api/moonraker";
import type { ServerInfo } from "../api/moonraker";
import { HELP_CACHE_KEY } from "../state";
import { queryClient } from "./client";

export interface MacroHelp {
  commands: Record<string, string>;
  fetchedUtc: string | null;
  cached: boolean;
}

export const moonrakerKeys = {
  macroHelp: (base: string) => ["moonraker", base, "macro-help"] as const,
  health: (base: string) => ["moonraker", base, "health"] as const,
};

function cachedMacroHelp(): MacroHelp | undefined {
  const raw = localStorage.getItem(HELP_CACHE_KEY);
  if (!raw) return undefined;
  let parsed;
  try {
    parsed = JSON.parse(raw);
  } catch (e) {
    return undefined;
  }
  if (!parsed || typeof parsed.commands !== "object" || parsed.commands === null) return undefined;
  return { commands: parsed.commands, fetchedUtc: parsed.fetched_utc || null, cached: true };
}

async function fetchMacroHelpData(base: string): Promise<MacroHelp> {
  const all = await getGcodeHelp(base);
  const commands: Record<string, string> = {};
  for (const [name, text] of Object.entries(all)) {
    if (name.startsWith("SERVO_")) commands[name] = text;
  }
  const fetchedUtc = new Date().toISOString();
  localStorage.setItem(HELP_CACHE_KEY, JSON.stringify({ fetched_utc: fetchedUtc, commands }));
  return { commands, fetchedUtc, cached: false };
}

export function macroHelpOptions(base: string) {
  return {
    queryKey: moonrakerKeys.macroHelp(base),
    queryFn: () => fetchMacroHelpData(base),
    initialData: cachedMacroHelp,
    staleTime: 0,
  };
}

export function macroHelpData(base: string): MacroHelp | undefined {
  return queryClient.getQueryData<MacroHelp>(moonrakerKeys.macroHelp(base));
}

export function macroHelpView(base: string): { data: MacroHelp | undefined; pending: boolean; error: string | null } {
  const st = queryClient.getQueryState<MacroHelp>(moonrakerKeys.macroHelp(base));
  return {
    data: st?.data,
    pending: st?.fetchStatus === "fetching",
    error: st?.error ? String(st.error) : null,
  };
}

export function macroHelpNeedsFetch(base: string): boolean {
  const data = macroHelpData(base);
  return !data || data.cached;
}

export async function fetchMacroHelp(base: string) {
  await queryClient.fetchQuery(macroHelpOptions(base)).catch(() => {});
}

export function loadCachedMacroHelp(base: string) {
  if (queryClient.getQueryData(moonrakerKeys.macroHelp(base))) return;
  const cached = cachedMacroHelp();
  if (cached) queryClient.setQueryData(moonrakerKeys.macroHelp(base), cached);
}

export function invalidateMacroHelp(base: string) {
  queryClient.invalidateQueries({ queryKey: moonrakerKeys.macroHelp(base) });
}

export function moonrakerHealthCache(base: string): ServerInfo | undefined {
  return queryClient.getQueryData<ServerInfo>(moonrakerKeys.health(base));
}

export function fetchMoonrakerHealth(base: string): Promise<ServerInfo> {
  return queryClient.fetchQuery({
    queryKey: moonrakerKeys.health(base),
    queryFn: () => getServerInfo(base),
    staleTime: 0,
  });
}
