import { el } from "./api";
import { consoleTabComplete, renderConsoleHelp } from "./docs";
import { runGcode } from "./moonraker";
import { CONSOLE_HISTORY_KEY, CONSOLE_HISTORY_MAX, state } from "./state";

// --- console ------------------------------------------------------------------

function pushConsoleHistory(entry: string) {
  const hist = state.console.history;
  if (hist[hist.length - 1] !== entry) hist.push(entry);
  if (hist.length > CONSOLE_HISTORY_MAX) hist.splice(0, hist.length - CONSOLE_HISTORY_MAX);
  localStorage.setItem(CONSOLE_HISTORY_KEY, JSON.stringify(hist));
}

function bindConsole() {
  const input = el<HTMLTextAreaElement>("console-input");
  if (!input) return;
  input.value = state.console.text;
  autosizeConsole(input);
  input.addEventListener("input", () => {
    state.console.text = input.value;
    autosizeConsole(input);
    renderConsoleHelp();
  });
  input.addEventListener("keyup", renderConsoleHelp);
  input.addEventListener("click", renderConsoleHelp);
  input.addEventListener("keydown", consoleKeydown);
  input.addEventListener("blur", () => exitConsoleSearch(true));
  renderConsoleHelp();
}

function autosizeConsole(input: HTMLTextAreaElement) {
  input.style.height = "auto";
  input.style.height = `${input.scrollHeight}px`;
}

function setConsoleValue(text: string, focus: boolean) {
  state.console.text = text;
  const input = el<HTMLTextAreaElement>("console-input");
  if (!input) return;
  input.value = text;
  input.selectionStart = input.selectionEnd = text.length;
  autosizeConsole(input);
  if (focus) input.focus();
  renderConsoleHelp();
}

function caretOnFirstLine(input: HTMLTextAreaElement) {
  return input.value.lastIndexOf("\n", input.selectionStart - 1) === -1;
}

function caretOnLastLine(input: HTMLTextAreaElement) {
  return input.value.indexOf("\n", input.selectionEnd) === -1;
}

function consoleKeydown(ev: KeyboardEvent) {
  const input = ev.target as HTMLTextAreaElement;
  const c = state.console;
  if (c.search) {
    consoleSearchKeydown(ev, input);
    return;
  }
  if (ev.key === "Enter" && !ev.shiftKey) {
    ev.preventDefault();
    submitConsole();
    return;
  }
  if (ev.key === "Tab" && !ev.shiftKey && !ev.ctrlKey && !ev.altKey) {
    ev.preventDefault();
    consoleTabComplete(input);
    return;
  }
  if (ev.ctrlKey && ev.key === "r") {
    ev.preventDefault();
    c.search = { query: "", pos: c.history.length - 1, saved: input.value, failed: false };
    renderConsoleSearch();
    return;
  }
  const back = (ev.ctrlKey && ev.key === "p") || (ev.key === "ArrowUp" && caretOnFirstLine(input));
  const fwd = (ev.ctrlKey && ev.key === "n") || (ev.key === "ArrowDown" && caretOnLastLine(input));
  if (back || fwd) {
    ev.preventDefault();
    historyStep(back ? -1 : 1);
    return;
  }
  if (ev.ctrlKey && ev.key === "c" && input.selectionStart === input.selectionEnd) {
    ev.preventDefault();
    c.cursor = null;
    setConsoleValue("", true);
  }
}

function historyStep(dir: number) {
  const c = state.console;
  if (!c.history.length) return;
  if (c.cursor === null) {
    if (dir > 0) return;
    c.draft = c.text;
    c.cursor = c.history.length;
  }
  const next = c.cursor + dir;
  if (next < 0) return;
  if (next >= c.history.length) {
    c.cursor = null;
    setConsoleValue(c.draft, true);
    return;
  }
  c.cursor = next;
  setConsoleValue(c.history[next], true);
}

function consoleSearchKeydown(ev: KeyboardEvent, input: HTMLTextAreaElement) {
  const s = state.console.search;
  if (!s) throw new Error("console search keydown without an active search");
  if (ev.ctrlKey && ev.key === "r") {
    ev.preventDefault();
    searchHistory(s.pos - 1);
    return;
  }
  if (ev.key === "Escape" || (ev.ctrlKey && ev.key === "g")) {
    ev.preventDefault();
    exitConsoleSearch(false);
    return;
  }
  if (ev.key === "Enter" && !ev.shiftKey) {
    ev.preventDefault();
    exitConsoleSearch(true);
    submitConsole();
    return;
  }
  if (ev.key === "Backspace") {
    ev.preventDefault();
    s.query = s.query.slice(0, -1);
    searchHistory(state.console.history.length - 1);
    return;
  }
  if (ev.key.length === 1 && !ev.ctrlKey && !ev.metaKey && !ev.altKey) {
    ev.preventDefault();
    s.query += ev.key;
    searchHistory(s.pos);
    return;
  }
  if (ev.key !== "Shift" && ev.key !== "CapsLock") exitConsoleSearch(true);
}

function searchHistory(fromIdx: number) {
  const s = state.console.search;
  if (!s) throw new Error("searchHistory without an active search");
  const hist = state.console.history;
  if (!s.query) {
    s.pos = hist.length - 1;
    s.failed = false;
    renderConsoleSearch();
    return;
  }
  let idx = Math.min(fromIdx, hist.length - 1);
  while (idx >= 0 && !hist[idx].includes(s.query)) idx--;
  s.failed = idx < 0;
  if (idx >= 0) {
    s.pos = idx;
    setConsoleValue(hist[idx], true);
  }
  renderConsoleSearch();
}

function exitConsoleSearch(keep: boolean) {
  const c = state.console;
  if (!c.search) return;
  const saved = c.search.saved;
  c.search = null;
  if (!keep) setConsoleValue(saved, true);
  renderConsoleSearch();
}

function renderConsoleSearch() {
  const box = el("console-search");
  if (!box) return;
  const s = state.console.search;
  box.textContent = s
    ? `(reverse-i-search) '${s.query}'${s.failed ? " — no match" : ""}`
    : "";
}

async function submitConsole() {
  const raw = state.console.text.trim();
  const lines = raw
    .split("\n")
    .map((l) => l.trim())
    .filter((l) => l.length && !l.startsWith(";"));
  if (!lines.length) return;
  pushConsoleHistory(raw);
  state.console.cursor = null;
  state.console.draft = "";
  setConsoleValue("", true);
  await runGcode(lines, "console");
}

export { pushConsoleHistory, bindConsole, autosizeConsole, setConsoleValue, caretOnFirstLine, caretOnLastLine, consoleKeydown, historyStep, consoleSearchKeydown, searchHistory, exitConsoleSearch, renderConsoleSearch, submitConsole };
