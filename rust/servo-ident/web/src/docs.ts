import { el, payloadUnchanged, shortTime } from "./api";
import { loadConsoleHistory, setConsoleValue } from "./console";
import { moonrakerUrl, escapeHtml } from "./moonraker";
import { consoleSectionHtml } from "./shell";
import { HELP_CACHE_KEY, state } from "./state";

// --- macro docs -----------------------------------------------------------------

/// The macro documentation IS klippy's cmd_*_help strings, fetched from the
/// running instance over Moonraker's /printer/gcode/help — so it can never
/// drift from the code that executes the command. localStorage keeps the
/// last good copy readable while klippy is down, which is exactly when a
/// failed run sends you looking for the docs.
async function fetchMacroHelp() {
  const h = state.help;
  if (h.pending) return;
  h.pending = true;
  try {
    const resp = await fetch(`${moonrakerUrl()}/printer/gcode/help`);
    if (!resp.ok) throw new Error(`gcode/help HTTP ${resp.status}`);
    const all = (await resp.json()).result;
    const commands = {};
    for (const [name, text] of Object.entries(all)) {
      if (name.startsWith("SERVO_")) commands[name] = text;
    }
    h.commands = commands;
    h.fetchedUtc = new Date().toISOString();
    h.cached = false;
    h.error = null;
    localStorage.setItem(
      HELP_CACHE_KEY,
      JSON.stringify({ fetched_utc: h.fetchedUtc, commands })
    );
  } catch (e) {
    h.error = String(e);
    if (!h.commands) loadCachedMacroHelp();
  } finally {
    h.pending = false;
  }
  renderDocsList();
  renderConsoleHelp();
}

/// The catch only forgives corrupt localStorage JSON, same contract as
/// loadConsoleHistory.
function loadCachedMacroHelp() {
  const raw = localStorage.getItem(HELP_CACHE_KEY);
  if (!raw) return;
  let parsed;
  try {
    parsed = JSON.parse(raw);
  } catch (e) {
    return;
  }
  if (!parsed || typeof parsed.commands !== "object" || parsed.commands === null) return;
  state.help.commands = parsed.commands;
  state.help.fetchedUtc = parsed.fetched_utc || null;
  state.help.cached = true;
}

/// Every cmd_*_help string ends in a "Params NAME (default) ..." tail — the
/// one convention this rendering leans on. A string without the marker just
/// renders as prose.
function splitMacroHelp(text) {
  const m = /\bParams\b/.exec(text);
  if (!m) return { prose: text.trim(), params: null };
  return {
    prose: text.slice(0, m.index).trim(),
    params: text.slice(m.index + m[0].length).trim(),
  };
}

/// Tokenizes a Params tail into param chips and plain-text runs. UPPERCASE
/// words are params (an optional =A|B suffix lists choices), a following
/// (...) group is that param's default, anything else — "as
/// SERVO_MEASURE_INERTIA plus" — stays literal text.
function parseParamsTail(tail) {
  const items = [];
  const tokens = tail.split(/\s+/).filter((t) => t.length);
  let i = 0;
  while (i < tokens.length) {
    const tok = tokens[i];
    const clean = tok.replace(/[.,;]$/, "");
    const eq = clean.indexOf("=");
    const name = eq < 0 ? clean : clean.slice(0, eq);
    if (/^[A-Z][A-Z0-9_]*$/.test(name)) {
      items.push({
        kind: "param",
        name,
        choices: eq < 0 ? null : clean.slice(eq + 1),
        dflt: null,
      });
      i++;
      continue;
    }
    if (tok.startsWith("(")) {
      let group = tok;
      while (!group.endsWith(")") && i + 1 < tokens.length) {
        i++;
        group += ` ${tokens[i]}`;
      }
      i++;
      const dflt = group.replace(/^\(/, "").replace(/\)$/, "");
      const last = items[items.length - 1];
      if (last && last.kind === "param" && last.dflt === null) last.dflt = dflt;
      else items.push({ kind: "text", text: group });
      continue;
    }
    const last = items[items.length - 1];
    if (last && last.kind === "text") last.text += ` ${tok}`;
    else items.push({ kind: "text", text: tok });
    i++;
  }
  return items;
}

function paramChipsHtml(items) {
  const known = state.help.commands || {};
  return items
    .map((it) => {
      if (it.kind === "text") {
        return `<span class="param-text">${escapeHtml(it.text)}</span>`;
      }
      let label = escapeHtml(it.name);
      if (it.choices) label += `<span class="param-extra">=${escapeHtml(it.choices)}</span>`;
      if (it.dflt) label += ` <span class="param-extra">(${escapeHtml(it.dflt)})</span>`;
      if (known[it.name]) {
        return `<a class="chip param-chip xref" href="#/docs/${it.name}">${label}</a>`;
      }
      return `<span class="chip param-chip">${label}</span>`;
    })
    .join("");
}

function docsShellHtml() {
  return (
    `<div class="workspace single">` +
    `<main class="analysis">` +
    `<section class="docs-section">` +
    `<div class="section-head"><h2>calibration macros</h2>` +
    `<span class="note" id="docs-status"></span></div>` +
    `<div id="docs-list"></div>` +
    `</section>` +
    consoleSectionHtml({}) +
    `</main></div>`
  );
}

function docsDeepLinkTarget() {
  const m = /^#\/docs\/([A-Za-z0-9_]+)/.exec(location.hash || "");
  return m ? m[1].toUpperCase() : null;
}

function firstSentence(prose) {
  const cut = prose.indexOf(". ");
  return cut < 0 ? prose : prose.slice(0, cut + 1);
}

function macroDocHtml(name, text, open) {
  const { prose, params } = splitMacroHelp(text);
  const items = params ? parseParamsTail(params) : [];
  return (
    `<details class="macro-doc" id="doc-${escapeHtml(name)}"${open ? " open" : ""}>` +
    `<summary><span class="macro-name">${escapeHtml(name)}</span>` +
    `<span class="hint" title="${escapeHtml(firstSentence(prose))}">` +
    `${escapeHtml(firstSentence(prose))}</span></summary>` +
    `<div class="macro-body">` +
    `<p class="macro-prose">${escapeHtml(prose)}</p>` +
    (items.length
      ? `<div class="chips param-chips">${paramChipsHtml(items)}</div>`
      : "") +
    `</div></details>`
  );
}

function renderDocsList() {
  const list = el("docs-list");
  if (!list) return;
  const h = state.help;
  const status = el("docs-status");
  if (status) {
    if (h.commands && !h.cached) {
      status.textContent =
        `the running klippy's cmd_*_help strings, fetched ${shortTime(h.fetchedUtc)}`;
    } else if (h.commands) {
      status.innerHTML =
        `cached copy${h.fetchedUtc ? ` from ${shortTime(h.fetchedUtc)}` : ""} — ` +
        `klippy unreachable <button id="docs-retry">retry</button>`;
    } else if (h.pending) {
      status.textContent = "fetching from klippy…";
    } else {
      status.innerHTML =
        `${escapeHtml(h.error || "not fetched yet")} <button id="docs-retry">retry</button>`;
    }
  }
  if (!h.commands) {
    list.innerHTML = `<p class="note">no macro help yet — is klippy up and the moonraker URL right?</p>`;
  } else {
    const target = docsDeepLinkTarget();
    if (!payloadUnchanged("docs-list", { commands: h.commands, target })) {
      const firstRender = !list.dataset.rendered;
      list.innerHTML = Object.entries(h.commands)
        .map(([name, text]) => macroDocHtml(name, text, name === target))
        .join("");
      list.dataset.rendered = "1";
      if (firstRender && target && h.commands[target]) {
        const entry = el(`doc-${target}`);
        if (entry) entry.scrollIntoView({ block: "start" });
      }
    }
  }
  const retry = el("docs-retry");
  if (retry) retry.addEventListener("click", fetchMacroHelp);
}

function consoleCaretLine(input) {
  const caret = input.selectionStart;
  const text = input.value;
  const start = text.lastIndexOf("\n", caret - 1) + 1;
  let end = text.indexOf("\n", caret);
  if (end < 0) end = text.length;
  return { line: text.slice(start, end), start, caretInLine: caret - start };
}

function lineCommand(line) {
  return (line.trim().split(/\s+/)[0] || "").toUpperCase();
}

function macroParamNames(cmdName) {
  const known = state.help.commands || {};
  const text = known[cmdName];
  if (!text) return null;
  const { params } = splitMacroHelp(text);
  if (!params) return [];
  return parseParamsTail(params)
    .filter((it) => it.kind === "param" && !known[it.name])
    .map((it) => it.name);
}

/// What tab completion would complete at the current caret: SERVO_* command
/// names for the line's first word, otherwise the command's param names not
/// already given on the line. A token with "=" is a value — nothing to
/// complete there.
function consoleCompletion(input) {
  const none: any = { candidates: [] };
  const h = state.help;
  if (!h.commands) return none;
  const { line, start, caretInLine } = consoleCaretLine(input);
  const tokenStart = line.lastIndexOf(" ", caretInLine - 1) + 1;
  const token = line.slice(tokenStart, caretInLine);
  if (token.includes("=")) return none;
  const up = token.toUpperCase();
  const common = { lineStart: start, tokenStart, tokenLen: token.length };
  if (!line.slice(0, tokenStart).trim().length) {
    if (!up.length) return none;
    return {
      ...common,
      candidates: Object.keys(h.commands).filter((n) => n.startsWith(up)),
      suffix: " ",
    };
  }
  const names = macroParamNames(lineCommand(line));
  if (!names) return none;
  const taken = new Set(
    Array.from(line.matchAll(/([A-Za-z][A-Za-z0-9_]*)=/g), (m) => m[1].toUpperCase())
  );
  return {
    ...common,
    candidates: names.filter((n) => n.startsWith(up) && !taken.has(n)),
    suffix: "=",
  };
}

function longestCommonPrefix(names) {
  let prefix = names[0];
  for (const n of names.slice(1)) {
    while (!n.startsWith(prefix)) prefix = prefix.slice(0, -1);
  }
  return prefix;
}

function consoleTabComplete(input) {
  const c = consoleCompletion(input);
  if (!c.candidates.length) return;
  const replacement =
    c.candidates.length === 1
      ? c.candidates[0] + c.suffix
      : longestCommonPrefix(c.candidates);
  const from = c.lineStart + c.tokenStart;
  const text = input.value;
  setConsoleValue(
    text.slice(0, from) + replacement + text.slice(from + c.tokenLen),
    true
  );
  input.selectionStart = input.selectionEnd = from + replacement.length;
  renderConsoleHelp();
}

/// The terminal-style help under the prompt: one dim description line and a
/// usage line of the command's params. The param whose value the caret is in
/// is highlighted; while a param name is being typed, every candidate the
/// prefix still matches is highlighted.
function renderConsoleHelp() {
  const box = el("console-help");
  const input = el("console-input");
  if (!box || !input) return;
  const { line, caretInLine } = consoleCaretLine(input);
  const first = lineCommand(line);
  if (!first.startsWith("SERVO")) {
    box.innerHTML = "";
    return;
  }
  const h = state.help;
  if (!h.commands) {
    if (!h.pending && !h.error) fetchMacroHelp();
    box.innerHTML = `<div class="hint">${
      h.error ? `macro help unavailable — ${escapeHtml(h.error)}` : "fetching macro help…"
    }</div>`;
    return;
  }
  const helpText = h.commands[first];
  if (!helpText) {
    const matches = Object.keys(h.commands).filter((n) => n.startsWith(first));
    box.innerHTML = matches.length
      ? `<div class="console-help-cands">${matches.map(escapeHtml).join("  ")}</div>`
      : "";
    return;
  }
  const tokenStart = line.lastIndexOf(" ", caretInLine - 1) + 1;
  let tokenEnd = line.indexOf(" ", caretInLine);
  if (tokenEnd < 0) tokenEnd = line.length;
  const caretToken = line.slice(tokenStart, tokenEnd);
  const onFirstWord = !line.slice(0, tokenStart).trim().length;
  const activeName = !onFirstWord && caretToken.includes("=")
    ? caretToken.split("=")[0].toUpperCase()
    : null;
  const typedPrefix = !onFirstWord && !caretToken.includes("=")
    ? line.slice(tokenStart, caretInLine).toUpperCase()
    : "";
  const { prose, params } = splitMacroHelp(helpText);
  const items = params ? parseParamsTail(params) : [];
  const usage = items
    .map((it) => {
      if (it.kind === "text") return `<span class="dim">${escapeHtml(it.text)}</span>`;
      let cls = "p";
      if (it.name === activeName) cls += " active";
      else if (typedPrefix.length && it.name.startsWith(typedPrefix)) cls += " match";
      let s = `<span class="${cls}">${escapeHtml(it.name)}`;
      if (it.choices) s += `<span class="dim">=${escapeHtml(it.choices)}</span>`;
      if (it.dflt) s += `<span class="dim">(${escapeHtml(it.dflt)})</span>`;
      return `${s}</span>`;
    })
    .join(" ");
  box.innerHTML =
    `<div class="console-help-desc"><a href="#/docs/${first}" ` +
    `title="open in the docs tab">${first}</a>` +
    `<span class="dim"> — ${escapeHtml(prose)}</span>` +
    (h.cached ? `<span class="hint"> (cached — klippy unreachable)</span>` : "") +
    `</div>` +
    (usage ? `<div class="console-help-usage">${usage}</div>` : "");
}

export { fetchMacroHelp, loadCachedMacroHelp, splitMacroHelp, parseParamsTail, paramChipsHtml, docsShellHtml, docsDeepLinkTarget, firstSentence, macroDocHtml, renderDocsList, consoleCaretLine, lineCommand, macroParamNames, consoleCompletion, longestCommonPrefix, consoleTabComplete, renderConsoleHelp };
