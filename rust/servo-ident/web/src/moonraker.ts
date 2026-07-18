import { el, shortTime } from "./api";
import { setConsoleValue } from "./console";
import { fetchMacroHelp } from "./docs";
import { state } from "./state";

// --- moonraker plumbing + session log ---------------------------------------

function moonrakerUrl() {
  return el("moonraker-url").value.replace(/\/+$/, "");
}

/// Every button on every page posts G-code through Moonraker, so a broken
/// URL or missing cors_domains entry silently kills the whole dashboard.
/// This badge in the topbar turns that failure mode into words.
async function pollMoonrakerHealth() {
  const badge = el("moonraker-health");
  if (!badge) return;
  try {
    const resp = await fetch(`${moonrakerUrl()}/server/info`);
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const info = (await resp.json()).result;
    badge.className = "mr-health ok";
    badge.textContent = `klippy ${info.klippy_state || "unknown"}`;
    const ks = info.klippy_state || "unknown";
    if (ks === "ready" && state.help.klippyState !== "ready") fetchMacroHelp();
    state.help.klippyState = ks;
  } catch (e) {
    badge.className = "mr-health err";
    badge.textContent = "moonraker unreachable — bad URL, moonraker down, or origin missing from cors_domains";
  }
}

/// One click, no confirmation: an accidental stop costs a FIRMWARE_RESTART,
/// a confirm dialog in a real emergency costs the machine.
async function emergencyStop() {
  const entry = { time: new Date().toISOString(), label: "e-stop", lines: ["emergency_stop"], results: [] };
  try {
    const resp = await fetch(`${moonrakerUrl()}/printer/emergency_stop`, { method: "POST" });
    entry.results.push({ ok: resp.ok, status: resp.status });
  } catch (e) {
    entry.results.push({ ok: false, status: 0 });
  }
  state.sentLog.push(entry);
  renderSentLog();
  pollMoonrakerHealth();
}

function escapeHtml(s) {
  return s
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function sentEntryHtml(entry) {
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
          .map((m) => {
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
  container.onclick = (ev) => {
    const line = ev.target.closest(".sent-line");
    if (line) setConsoleValue(line.dataset.line, true);
  };
  container.scrollTop = container.scrollHeight;
}

/// Timestamps in Moonraker's gcode store are server clock, so diffing
/// against its own latest entry needs no client/server clock agreement.
async function latestGcodeStoreTime(base) {
  const resp = await fetch(`${base}/server/gcode_store?count=1`);
  if (!resp.ok) throw new Error(`gcode_store HTTP ${resp.status}`);
  const store = (await resp.json()).result.gcode_store;
  return store.length ? store[store.length - 1].time : 0;
}

async function fetchGcodeResponses(base, sinceTime) {
  const resp = await fetch(`${base}/server/gcode_store?count=500`);
  if (!resp.ok) throw new Error(`gcode_store HTTP ${resp.status}`);
  const store = (await resp.json()).result.gcode_store;
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
async function runGcode(lines, label) {
  const base = moonrakerUrl();
  const statusEl = el("run-status");
  if (statusEl) statusEl.textContent = "";
  const entry = { time: new Date().toISOString(), label, lines: [], results: [], responses: [] };
  state.sentLog.push(entry);
  for (const line of lines) {
    const url = `${base}/printer/gcode/script?script=${encodeURIComponent(line)}`;
    entry.lines.push(line);
    let sentAt = null;
    try {
      sentAt = await latestGcodeStoreTime(base);
    } catch (e) {
      console.error(e);
    }
    let ok = false;
    try {
      const resp = await fetch(url, { method: "POST" });
      const text = await resp.text();
      if (!resp.ok && statusEl) {
        statusEl.innerHTML += `<div class="status-err">${line} -> HTTP ${resp.status} ${text.slice(0, 200)}</div>`;
      }
      ok = resp.ok;
      entry.results.push({ ok: resp.ok, status: resp.status });
    } catch (e) {
      if (statusEl) statusEl.innerHTML += `<div class="status-err">${line} -> ${e}</div>`;
      entry.results.push({ ok: false, status: 0 });
    }
    let responses = [];
    if (sentAt !== null) {
      try {
        responses = await fetchGcodeResponses(base, sentAt);
      } catch (e) {
        console.error(e);
      }
    }
    entry.responses.push(responses);
    renderSentLog();
    if (!ok) break;
  }
  renderSentLog();
}

export { moonrakerUrl, pollMoonrakerHealth, emergencyStop, escapeHtml, sentEntryHtml, renderSentLog, latestGcodeStoreTime, fetchGcodeResponses, runGcode };
