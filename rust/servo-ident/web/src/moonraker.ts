import { el, mustEl, shortTime } from "./api";
import { setConsoleValue } from "./console";
import { fetchMacroHelp } from "./docs";
import { state } from "./state";
import type { SentEntry } from "./state";
import { getGcodeStore, postEmergencyStop, postGcodeScript } from "./api/moonraker";
import { moonrakerHealthCache, fetchMoonrakerHealth, invalidateMacroHelp } from "./queries/moonraker";

// --- moonraker plumbing + session log ---------------------------------------

function moonrakerUrl(): string {
  return mustEl<HTMLInputElement>("moonraker-url").value.replace(/\/+$/, "");
}

/// Every button on every page posts G-code through Moonraker, so a broken
/// URL or missing cors_domains entry silently kills the whole dashboard.
/// This badge in the topbar turns that failure mode into words.
async function pollMoonrakerHealth() {
  const badge = el("moonraker-health");
  if (!badge) return;
  const base = moonrakerUrl();
  const prev = moonrakerHealthCache(base);
  try {
    const health = await fetchMoonrakerHealth(base);
    badge.className = "mr-health ok";
    badge.textContent = `klippy ${health.klippyState}`;
    if (health.klippyState === "ready" && prev?.klippyState !== "ready") {
      invalidateMacroHelp(base);
      fetchMacroHelp();
    }
  } catch (e) {
    badge.className = "mr-health err";
    badge.textContent = "moonraker unreachable — bad URL, moonraker down, or origin missing from cors_domains";
  }
}

/// One click, no confirmation: an accidental stop costs a FIRMWARE_RESTART,
/// a confirm dialog in a real emergency costs the machine.
async function emergencyStop() {
  const entry: SentEntry = { time: new Date().toISOString(), label: "e-stop", lines: ["emergency_stop"], results: [] };
  try {
    const r = await postEmergencyStop(moonrakerUrl());
    entry.results.push({ ok: r.ok, status: r.status });
  } catch (e) {
    entry.results.push({ ok: false, status: 0 });
  }
  state.sentLog.push(entry);
  renderSentLog();
  pollMoonrakerHealth();
}

function escapeHtml(s: string): string {
  return s
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function sentEntryHtml(entry: SentEntry): string {
  const ok = entry.results.length > 0 && entry.results.every((r) => r.ok);
  return (
    `<div class="sent-entry">` +
    `<div class="sent-head">${shortTime(entry.time)} — ${entry.label} — ` +
    `<span class="${ok ? "status-ok" : "status-err"}">${ok ? "ok" : "error"}</span></div>` +
    entry.lines
      .map((l, i) => {
        const r = entry.results[i];
        const suffix = r && !r.ok ? ` <span class="status-err">HTTP ${r.status}</span>` : "";
        const responses = ((entry.responses && entry.responses[i]) || [])
          .map((m: string) => {
            const cls = m.startsWith("!!") ? "resp-line resp-err" : "resp-line";
            return `<div class="${cls}">${escapeHtml(m)}</div>`;
          })
          .join("");
        return (
          `<div class="sent-line" data-line="${escapeHtml(l)}" ` +
          `title="click to insert into the console">${escapeHtml(l)}${suffix}</div>${responses}`
        );
      })
      .join("") +
    `</div>`
  );
}

function renderSentLog() {
  const container = el("sent-log");
  if (!container) return;
  container.innerHTML = state.sentLog.length
    ? state.sentLog.map(sentEntryHtml).join("")
    : '<p class="note">nothing sent yet</p>';
  container.onclick = (ev: MouseEvent) => {
    const target = ev.target as HTMLElement;
    const line = target.closest<HTMLElement>(".sent-line");
    if (line) setConsoleValue(line.dataset.line ?? "", true);
  };
  container.scrollTop = container.scrollHeight;
}

/// Timestamps in Moonraker's gcode store are server clock, so diffing
/// against its own latest entry needs no client/server clock agreement.
async function latestGcodeStoreTime(base: string): Promise<number> {
  const store = await getGcodeStore(base, 1);
  return store.length ? store[store.length - 1].time : 0;
}

async function fetchGcodeResponses(base: string, sinceTime: number): Promise<string[]> {
  const store = await getGcodeStore(base, 500);
  return store
    .filter((e) => e.type === "response" && e.time > sinceTime)
    .map((e) => e.message);
}

/// Sends `lines` (already-built gcode) through the shared Moonraker
/// plumbing — the grid's Apply and the console land in the same session
/// log, which survives page switches. `/printer/gcode/script` blocks
/// until the command finishes, and klippy's respond_info output only
/// travels the websocket — so each line's responses are harvested from
/// `/server/gcode_store` afterwards and echoed under the sent line.
async function runGcode(lines: string[], label: string) {
  const base = moonrakerUrl();
  const statusEl = el("run-status");
  if (statusEl) statusEl.textContent = "";
  const entry: SentEntry = { time: new Date().toISOString(), label, lines: [], results: [], responses: [] };
  state.sentLog.push(entry);
  for (const line of lines) {
    entry.lines.push(line);
    let sentAt: number | null = null;
    try {
      sentAt = await latestGcodeStoreTime(base);
    } catch (e) {
      console.error(e);
    }
    let ok = false;
    try {
      const r = await postGcodeScript(base, line);
      if (!r.ok && statusEl) {
        statusEl.innerHTML += `<div class="status-err">${line} -> HTTP ${r.status} ${r.text.slice(0, 200)}</div>`;
      }
      ok = r.ok;
      entry.results.push({ ok: r.ok, status: r.status });
    } catch (e) {
      if (statusEl) statusEl.innerHTML += `<div class="status-err">${line} -> ${e}</div>`;
      entry.results.push({ ok: false, status: 0 });
    }
    let responses: string[] = [];
    if (sentAt !== null) {
      try {
        responses = await fetchGcodeResponses(base, sentAt);
      } catch (e) {
        console.error(e);
      }
    }
    entry.responses!.push(responses);
    renderSentLog();
    if (!ok) break;
  }
  renderSentLog();
}

export { moonrakerUrl, pollMoonrakerHealth, emergencyStop, escapeHtml, sentEntryHtml, renderSentLog, latestGcodeStoreTime, fetchGcodeResponses, runGcode };
