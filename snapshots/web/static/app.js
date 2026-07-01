"use strict";

const casesEl = document.getElementById("cases");
const summaryEl = document.getElementById("summary");
const bannerEl = document.getElementById("banner");
const acceptAllBtn = document.getElementById("accept-all");
const titleEl = document.getElementById("title");

async function load() {
  const data = await fetch("/api/cases").then((r) => r.json());
  render(data);
}

function render(data) {
  const readOnly = Boolean(data.read_only);
  titleEl.textContent = data.title || "Motion snapshot review";
  acceptAllBtn.hidden = readOnly;
  bannerEl.hidden = !data.error;
  if (data.error) bannerEl.textContent = data.error;

  const review = data.review || [];
  const bits = [];
  if (readOnly) {
    bits.push(`${data.baseline_count || review.length} baselines`);
  } else {
    if (data.exact) bits.push(`${data.exact} match baseline`);
    bits.push(`${review.length} to review`);
  }
  summaryEl.textContent = bits.join(" · ");
  acceptAllBtn.disabled = readOnly || review.length === 0;

  casesEl.innerHTML = "";
  if (review.length === 0 && !data.error) {
    const div = document.createElement("div");
    div.className = "empty";
    div.textContent = readOnly
      ? "No baselines found."
      : "Nothing to review — every case matches its baseline.";
    casesEl.appendChild(div);
    return;
  }
  // Group by leading path (everything before the last /), so a
  // <group>/<cfg>/<gcode> name sections under its <group>/<cfg> config.
  const groups = new Map();
  for (const c of review) {
    const slash = c.name.lastIndexOf("/");
    const group = slash > 0 ? c.name.substring(0, slash).replace(/_/g, " ") : "Other";
    if (!groups.has(group)) groups.set(group, []);
    groups.get(group).push(c);
  }
  for (const [group, items] of groups) {
    const section = document.createElement("div");
    section.className = "section";
    const header = document.createElement("h2");
    header.className = "section-title";
    header.textContent = group.charAt(0).toUpperCase() + group.slice(1);
    section.appendChild(header);
    const grid = document.createElement("div");
    grid.className = "section-grid";
    for (const c of items) grid.appendChild(card(c));
    section.appendChild(grid);
    casesEl.appendChild(section);
  }
}

function viewerUrl(name) {
  return `/viewer.html?case=${encodeURIComponent(name)}`;
}

function card(c) {
  const el = document.createElement("section");
  el.className = "card";
  el.onclick = () => { window.location.href = viewerUrl(c.name); };

  const imgWrap = document.createElement("div");
  imgWrap.className = "card-img";
  const canvas = document.createElement("canvas");
  canvas.width = 400;
  canvas.height = 300;
  canvas.className = "card-canvas";
  imgWrap.appendChild(canvas);
  renderPathPreview(canvas, c.name);

  if (c.has_before) {
    const badge = document.createElement("span");
    badge.className = "diff-badge";
    badge.textContent = "DIFF";
    imgWrap.appendChild(badge);
  }

  el.appendChild(imgWrap);

  const foot = document.createElement("div");
  foot.className = "card-foot";
  const slash = c.name.lastIndexOf("/");
  const shortName = slash >= 0 ? c.name.substring(slash + 1) : c.name;
  foot.innerHTML =
    `<span class="case-name" title="${c.name}">${shortName}</span>` +
    `<span class="badge ${c.status}">${c.status}</span>`;
  el.appendChild(foot);

  return el;
}

// -- Path preview (dashboard thumbnails) ------------------------------------
async function renderPathPreview(canvas, name) {
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#0d0f12";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  try {
    const resp = await fetch(`/snapshot-data/${encodeURIComponent(name)}`);
    if (!resp.ok) return;
    const snap = await resp.json();

    const kx = snap.kin_x, ky = snap.kin_y;
    if (!kx || !ky || kx.length < 2) return;

    // Compute bounds
    let xMin = Infinity, xMax = -Infinity, yMin = Infinity, yMax = -Infinity;
    for (let i = 0; i < kx.length; i++) {
      if (kx[i] < xMin) xMin = kx[i]; if (kx[i] > xMax) xMax = kx[i];
      if (ky[i] < yMin) yMin = ky[i]; if (ky[i] > yMax) yMax = ky[i];
    }
    // Equal aspect ratio — match canvas pixel aspect
    const xR = xMax - xMin || 1, yR = yMax - yMin || 1;
    const dataAspect = xR / yR;
    const canvasAspect = canvas.width / canvas.height;
    let xMid = (xMin + xMax) / 2, yMid = (yMin + yMax) / 2;
    if (dataAspect < canvasAspect) {
      const targetX = yR * canvasAspect;
      xMin = xMid - targetX / 2; xMax = xMid + targetX / 2;
    } else {
      const targetY = xR / canvasAspect;
      yMin = yMid - targetY / 2; yMax = yMid + targetY / 2;
    }
    const pad = Math.max(xMax - xMin, yMax - yMin) * 0.08;
    xMin -= pad; xMax += pad; yMin -= pad; yMax += pad;

    const W = canvas.width, H = canvas.height;
    const toX = (v) => ((v - xMin) / (xMax - xMin)) * W;
    const toY = (v) => H - ((v - yMin) / (yMax - yMin)) * H;

    // Draw fitted segments
    const colors = { line: "#4a9eff", arc: "#4ecb71", clothoid: "#f5a623" };
    for (const seg of (snap.fitted_segments || [])) {
      ctx.beginPath();
      ctx.strokeStyle = colors[seg.type] || "#4a9eff";
      ctx.lineWidth = 1.2;
      if (seg.type === "line") {
        ctx.moveTo(toX(seg.x0), toY(seg.y0));
        ctx.lineTo(toX(seg.x1), toY(seg.y1));
      } else if (seg.x && seg.y) {
        for (let j = 0; j < seg.x.length; j++) {
          j === 0 ? ctx.moveTo(toX(seg.x[j]), toY(seg.y[j]))
                   : ctx.lineTo(toX(seg.x[j]), toY(seg.y[j]));
        }
      }
      ctx.stroke();
    }

    // Draw raw path (thin gray)
    const rx = snap.raw_x, ry = snap.raw_y;
    if (rx && ry) {
      ctx.beginPath();
      ctx.strokeStyle = "#333";
      ctx.lineWidth = 0.6;
      for (let i = 0; i < rx.length; i++) {
        i === 0 ? ctx.moveTo(toX(rx[i]), toY(ry[i]))
                 : ctx.lineTo(toX(rx[i]), toY(ry[i]));
      }
      ctx.stroke();
    }

    // Start dot
    ctx.beginPath();
    ctx.fillStyle = "#ef5350";
    ctx.arc(toX(kx[0]), toY(ky[0]), 3, 0, Math.PI * 2);
    ctx.fill();
  } catch (e) { /* ignore fetch errors */ }
}

async function postAccept(payload) {
  const data = await fetch("/api/accept", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  }).then((r) => r.json());
  if ((data.review || []).length === 0 && !data.error) {
    document.body.innerHTML =
      '<p class="empty">All snapshots accepted — review complete. ' +
      "You can close this tab and return to the terminal.</p>";
  } else {
    render(data);
  }
}

acceptAllBtn.onclick = () => postAccept({ all: true });

load();
