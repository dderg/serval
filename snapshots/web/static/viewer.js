// Snapshot review page: case navigation, before/after against the committed
// baseline, and Accept. The four panels themselves live in TrajectoryView.
import { TrajectoryView, initWasm, setupSplitter } from "./trajectory-view.js";

const params = new URLSearchParams(window.location.search);
let currentCase = params.get("case");
let caseList = []; // [{ name, status, has_before }] — switchable set, from /api/cases
let readOnly = false; // baselines mode serves a read-only gallery — no accept
const acceptedNames = new Set(); // cases accepted this session, for button state
let view = null;

// -- Case switching ----------------------------------------------------------
function caseIndex() {
  return caseList.findIndex(c => c.name === currentCase);
}

function currentEntry() {
  return caseList.find(c => c.name === currentCase) || null;
}

function syncCaseControls() {
  const sel = document.getElementById("case-select");
  if (sel.value !== currentCase) sel.value = currentCase;
  const i = caseIndex();
  document.getElementById("case-prev").disabled = i <= 0;
  document.getElementById("case-next").disabled = i < 0 || i >= caseList.length - 1;
  syncAcceptControl();
}

// The review list only ever holds changed/new cases, so an entry with a
// non-exact status is exactly a case that can be written as the new baseline.
function syncAcceptControl() {
  const btn = document.getElementById("accept");
  if (readOnly) { btn.hidden = true; return; }
  btn.hidden = false;
  const entry = currentEntry();
  const accepted = acceptedNames.has(currentCase);
  const reviewable = !accepted && entry != null
    && entry.status && entry.status !== "exact";
  btn.disabled = !reviewable;
  btn.textContent = accepted ? "Accepted ✓" : "Accept";
}

function stepCase(dir) {
  const i = caseIndex();
  if (i < 0) return;
  const next = i + dir;
  if (next < 0 || next >= caseList.length) return;
  loadCase(caseList[next].name);
}

function rebuildCaseSelect() {
  const sel = document.getElementById("case-select");
  sel.innerHTML = "";
  // <group>/<cfg>/<gcode>: one <optgroup> per test group, option text
  // "<cfg>/<gcode>" so the collapsed select still shows which config the case
  // ran under. Keyed by group so entries group correctly whatever order
  // caseList arrives in.
  const optgroups = new Map();
  for (const c of caseList) {
    const first = c.name.indexOf("/");
    const group = first > 0 ? c.name.substring(0, first) : "";
    const label = first >= 0 ? c.name.substring(first + 1) : c.name;
    let parent = sel;
    if (group) {
      parent = optgroups.get(group);
      if (!parent) {
        parent = document.createElement("optgroup");
        parent.label = group;
        sel.appendChild(parent);
        optgroups.set(group, parent);
      }
    }
    const opt = document.createElement("option");
    opt.value = c.name;
    opt.textContent = label;
    parent.appendChild(opt);
  }
}

async function loadCaseList() {
  let review = [];
  try {
    const data = await fetch("/api/cases").then(r => r.json());
    review = data.review || [];
    readOnly = Boolean(data.read_only);
  } catch (e) { /* offline / no server scan — fall back to single case */ }
  caseList = review;
  if (currentCase && !caseList.some(c => c.name === currentCase)) {
    caseList.push({ name: currentCase });
  }
  if (!currentCase && caseList.length > 0) currentCase = caseList[0].name;

  rebuildCaseSelect();
  document.getElementById("case-select").addEventListener("change", (e) => loadCase(e.target.value));
  document.getElementById("case-prev").addEventListener("click", () => stepCase(-1));
  document.getElementById("case-next").addEventListener("click", () => stepCase(1));
  syncCaseControls();
}

// -- Variant (before/after) --------------------------------------------------
function updateMeta() {
  document.getElementById("meta").textContent =
    `t=${view.data.traversal_time().toFixed(3)}s  ` +
    `[${view.segmentSummary()}]  ` +
    `${view.data.point_count()} pts`;
}

function syncVariantControls() {
  const btn = document.getElementById("toggle-variant");
  const hasBefore = view.hasBefore();
  btn.disabled = !hasBefore;
  btn.classList.toggle("after", hasBefore && view.variant === "after");
  btn.classList.toggle("before", hasBefore && view.variant === "before");
  if (!hasBefore) {
    btn.textContent = "After";
    btn.title = "No baseline to compare against";
  } else {
    btn.textContent = view.variant === "before" ? "Before" : "After";
    btn.title = "Compare before/after (space)";
  }
}

async function fetchSnapshot(name, which) {
  const resp = await fetch(
    `/snapshot-data/${encodeURIComponent(name)}?which=${which}`
  );
  if (!resp.ok) return null;
  return resp.json();
}

// -- Load a single case into the graphs --------------------------------------
async function loadCase(name) {
  currentCase = name;
  const url = new URL(window.location);
  url.searchParams.set("case", name);
  history.replaceState(null, "", url);
  syncCaseControls();
  document.title = `Snapshot — ${name}`;
  document.getElementById("case-path").textContent = name.replace(/\//g, " / ");

  const after = await fetchSnapshot(name, "after");
  if (after == null) {
    document.getElementById("meta").textContent = "Error: failed to load case";
    return;
  }
  // The baselines gallery has no prior to compare against; skip the fetch
  // instead of collecting a 404 per case load.
  const before = readOnly ? null : await fetchSnapshot(name, "before");
  view.setData(after, before);
}

// -- PNG popup ---------------------------------------------------------------
function openPng() {
  if (!currentCase) return;
  const scroll = document.getElementById("png-scroll");
  scroll.innerHTML = "";
  const img = new Image();
  img.src = `/img/${encodeURIComponent(currentCase)}/after.png?t=${Date.now()}`;
  scroll.appendChild(img);
  scroll.scrollTop = 0;
  document.getElementById("png-overlay").classList.add("open");
}

function closePng() {
  document.getElementById("png-overlay").classList.remove("open");
}

function pngOpen() {
  return document.getElementById("png-overlay").classList.contains("open");
}

// -- Accept ------------------------------------------------------------------
function showAcceptDone() {
  document.body.innerHTML =
    '<p style="padding:40px;font-size:15px;color:#9aa3b2;' +
    'font-family:system-ui,sans-serif">All snapshots accepted — review ' +
    "complete. You can close this tab and return to the terminal.</p>";
}

// After accepting, jump to the next case still needing review, preferring the
// one that followed the accepted case in the prior ordering.
function pickNextReviewable(remaining, fromName) {
  const names = new Set(remaining.map(c => c.name));
  const i = caseList.findIndex(c => c.name === fromName);
  for (let k = i + 1; k < caseList.length; k++) {
    if (names.has(caseList[k].name)) return caseList[k].name;
  }
  for (let k = i - 1; k >= 0; k--) {
    if (names.has(caseList[k].name)) return caseList[k].name;
  }
  return remaining.length > 0 ? remaining[0].name : null;
}

async function acceptCurrent() {
  if (readOnly || !currentCase) return;
  const entry = currentEntry();
  if (entry == null || !entry.status || entry.status === "exact") return;
  if (acceptedNames.has(currentCase)) return;
  const acceptedCase = currentCase;

  const btn = document.getElementById("accept");
  btn.disabled = true;
  btn.textContent = "Accepting…";

  let data;
  try {
    data = await fetch("/api/accept", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ names: [acceptedCase] }),
    }).then(r => r.json());
  } catch (e) {
    btn.textContent = "Accept failed";
    return;
  }

  acceptedNames.add(acceptedCase);
  const remaining = data.review || [];
  if (remaining.length === 0) {
    showAcceptDone();
    return;
  }
  const stayedOnAccepted = currentCase === acceptedCase;
  const nextName = pickNextReviewable(remaining, acceptedCase);
  caseList = remaining.slice();
  rebuildCaseSelect();
  if (stayedOnAccepted && nextName) loadCase(nextName);
  else syncCaseControls();
}

// -- Init --------------------------------------------------------------------
async function main() {
  await initWasm();

  view = new TrajectoryView();
  view.onChanged = () => {
    updateMeta();
    syncVariantControls();
  };
  setupSplitter("snapshotViewer.pathSplit");

  document.getElementById("reset-zoom").addEventListener("click", () => view.resetZoom());

  document.getElementById("toggle-peaks").addEventListener("click", (e) => {
    e.target.classList.toggle("active", !view.showPeaks);
    view.setShowPeaks(!view.showPeaks);
  });

  document.getElementById("toggle-fitted-path").addEventListener("click", (e) => {
    view.setShowFittedPath(!view.showFittedPath);
    e.target.textContent = view.showFittedPath ? "Fitted" : "Shaped";
    e.target.classList.toggle("active", view.showFittedPath);
  });

  document.getElementById("toggle-variant").addEventListener("click", () => view.toggleVariant());
  document.getElementById("open-png").addEventListener("click", openPng);
  document.getElementById("png-overlay").addEventListener("click", closePng);
  document.getElementById("accept").addEventListener("click", acceptCurrent);

  document.addEventListener("keydown", (e) => {
    if (pngOpen()) {
      if (e.key === "Escape") closePng();
      return;
    }
    if (e.key === "ArrowLeft") stepCase(-1);
    else if (e.key === "ArrowRight") stepCase(1);
    else if (e.key === " " || e.key === "b" || e.key === "B") {
      e.preventDefault();
      view.toggleVariant();
    }
    else if (e.key === "a" || e.key === "A") acceptCurrent();
  });

  // Paint the requested case before the /api/cases scan (which re-runs the
  // planner over every case) so the graphs come up immediately; the dropdown
  // fills in once the list arrives. With no ?case, the list picks the first.
  if (currentCase) {
    await loadCase(currentCase);
    await loadCaseList();
  } else {
    await loadCaseList();
    if (!currentCase) {
      document.getElementById("meta").textContent = "No case specified — add ?case=name to URL";
      return;
    }
    await loadCase(currentCase);
  }
}

main();
