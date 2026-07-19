export interface ServerInfo {
  klippyState: string;
}

export interface GcodeStoreEntry {
  message: string;
  time: number;
  type: string;
}

export interface CommandOutcome {
  ok: boolean;
  status: number;
}

export interface ScriptOutcome extends CommandOutcome {
  text: string;
}

export async function getServerInfo(base: string): Promise<ServerInfo> {
  const resp = await fetch(`${base}/server/info`);
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
  const info = (await resp.json()).result;
  return { klippyState: info.klippy_state || "unknown" };
}

export async function getGcodeHelp(base: string): Promise<Record<string, string>> {
  const resp = await fetch(`${base}/printer/gcode/help`);
  if (!resp.ok) throw new Error(`gcode/help HTTP ${resp.status}`);
  return (await resp.json()).result as Record<string, string>;
}

export async function getGcodeStore(base: string, count: number): Promise<GcodeStoreEntry[]> {
  const resp = await fetch(`${base}/server/gcode_store?count=${count}`);
  if (!resp.ok) throw new Error(`gcode_store HTTP ${resp.status}`);
  return (await resp.json()).result.gcode_store as GcodeStoreEntry[];
}

export async function postEmergencyStop(base: string): Promise<CommandOutcome> {
  const resp = await fetch(`${base}/printer/emergency_stop`, { method: "POST" });
  return { ok: resp.ok, status: resp.status };
}

export async function postGcodeScript(base: string, script: string): Promise<ScriptOutcome> {
  const resp = await fetch(`${base}/printer/gcode/script?script=${encodeURIComponent(script)}`, { method: "POST" });
  const text = await resp.text();
  return { ok: resp.ok, status: resp.status, text };
}
