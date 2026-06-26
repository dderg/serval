"use strict";

const casesEl = document.getElementById("cases");
const summaryEl = document.getElementById("summary");
const bannerEl = document.getElementById("banner");
const acceptAllBtn = document.getElementById("accept-all");
const titleEl = document.getElementById("title");

let cacheBust = Date.now();
let allCases = [];
let modalIdx = -1;

async function load() {
  const data = await fetch("/api/cases").then((r) => r.json());
  cacheBust = Date.now();
  render(data);
}

function render(data) {
  const readOnly = Boolean(data.read_only);
  titleEl.textContent = data.title || "Motion snapshot review";
  acceptAllBtn.hidden = readOnly;
  bannerEl.hidden = !data.error;
  if (data.error) bannerEl.textContent = data.error;

  const review = data.review || [];
  allCases = review;
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
  // Group by prefix (part before /)
  const groups = new Map();
  for (let i = 0; i < review.length; i++) {
    const c = review[i];
    const slash = c.name.indexOf("/");
    const group = slash > 0 ? c.name.substring(0, slash).replace(/_/g, " ") : "Other";
    if (!groups.has(group)) groups.set(group, []);
    groups.get(group).push({ c, i });
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
    for (const { c, i } of items) grid.appendChild(card(c, i));
    section.appendChild(grid);
    casesEl.appendChild(section);
  }
}

function imgUrl(name, which) {
  return `/img/${encodeURIComponent(name)}/${which}.png?t=${cacheBust}`;
}

function card(c, idx) {
  const el = document.createElement("section");
  el.className = "card";
  el.onclick = () => openModal(idx);

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
  const slash = c.name.indexOf("/");
  const shortName = slash > 0 ? c.name.substring(slash + 1) : c.name;
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

// -- Modal -------------------------------------------------------------------
let modalZoom = 1;
let modalPanX = 0, modalPanY = 0;
let modalDragging = false, modalDragStartX = 0, modalDragStartY = 0;
let modalImgEl = null; // cached reference

function openModal(idx) {
  modalIdx = idx;
  modalZoom = 1;
  modalPanX = 0;
  modalPanY = 0;
  const c = allCases[idx];
  if (!c) return;

  let overlay = document.getElementById("modal-overlay");
  if (!overlay) {
    overlay = document.createElement("div");
    overlay.id = "modal-overlay";
    overlay.innerHTML = `
      <div class="modal-backdrop" id="modal-backdrop"></div>
      <div class="modal">
        <div class="modal-topbar">
          <span class="modal-name" id="modal-name"></span>
          <span class="modal-status" id="modal-status"></span>
          <div class="modal-zoom-btns">
            <button id="modal-zoom-out" title="Zoom out">−</button>
            <span id="modal-zoom-level">100%</span>
            <button id="modal-zoom-in" title="Zoom in">+</button>
            <button id="modal-zoom-reset" title="Reset zoom">Reset</button>
          </div>
          <a class="modal-interactive-btn" id="modal-interactive" target="_blank" rel="noreferrer">
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M15 3h6v6M9 21H3v-6M21 3l-7 7M3 21l7-7"/></svg>
            Interactive Viewer
          </a>
          <button class="modal-close" id="modal-close">&times;</button>
        </div>
        <button class="modal-arrow modal-prev" id="modal-prev">&#8249;</button>
        <button class="modal-arrow modal-next" id="modal-next">&#8250;</button>
        <div class="modal-img-wrap" id="modal-img-wrap"></div>
      </div>
    `;
    document.body.appendChild(overlay);

    document.getElementById("modal-backdrop").onclick = closeModal;
    document.getElementById("modal-close").onclick = closeModal;
    document.getElementById("modal-prev").onclick = () => navigateModal(-1);
    document.getElementById("modal-next").onclick = () => navigateModal(1);
    document.getElementById("modal-zoom-in").onclick = () => modalZoomBy(1.25);
    document.getElementById("modal-zoom-out").onclick = () => modalZoomBy(0.8);
    document.getElementById("modal-zoom-reset").onclick = modalZoomReset;

    // Wheel: pinch zoom, two-finger pan, scroll zoom
    let modalWheelTimer = null;
    const imgWrap = document.getElementById("modal-img-wrap");
    imgWrap.addEventListener("wheel", (e) => {
      e.preventDefault();
      // Suppress mousemove during gesture
      clearTimeout(modalWheelTimer);
      modalWheelTimer = setTimeout(() => { modalWheelTimer = null; }, 80);

      if (e.ctrlKey || e.metaKey) {
        // Pinch zoom: pinch in = zoom in, pinch out = zoom out
        const factor = Math.exp(-e.deltaY * 0.01);
        modalZoomBy(factor);
      } else if (e.deltaX !== 0) {
        // Two-finger scroll → pan (content follows fingers)
        modalPanX -= e.deltaX;
        modalApplyTransform();
      } else {
        // Scroll wheel → zoom
        const factor = e.deltaY > 0 ? 0.92 : 1 / 0.92;
        modalZoomBy(factor);
      }
    }, { passive: false });

    // Drag to pan (mouse or single-finger touch)
    imgWrap.addEventListener("mousedown", (e) => {
      modalDragging = true;
      modalDragStartX = e.clientX - modalPanX;
      modalDragStartY = e.clientY - modalPanY;
      imgWrap.style.cursor = "grabbing";
    });
    window.addEventListener("mousemove", (e) => {
      if (!modalDragging) return;
      modalPanX = e.clientX - modalDragStartX;
      modalPanY = e.clientY - modalDragStartY;
      modalApplyTransform();
    });
    window.addEventListener("mouseup", () => {
      modalDragging = false;
      const w = document.getElementById("modal-img-wrap");
      if (w) w.style.cursor = "";
    });
  }

  renderModalContent(c);
  overlay.classList.add("open");
  document.body.style.overflow = "hidden";
}

function modalZoomBy(factor) {
  modalZoom = Math.max(0.25, Math.min(5, modalZoom * factor));
  if (modalZoom <= 1) { modalPanX = 0; modalPanY = 0; }
  document.getElementById("modal-zoom-level").textContent = Math.round(modalZoom * 100) + "%";
  modalApplyTransform();
}

function modalZoomReset() {
  modalZoom = 1;
  modalPanX = 0;
  modalPanY = 0;
  document.getElementById("modal-zoom-level").textContent = "100%";
  modalApplyTransform();
}

function modalApplyTransform() {
  if (modalImgEl) modalImgEl.style.transform = `translate(${modalPanX}px, ${modalPanY}px) scale(${modalZoom})`;
}

function renderModalContent(c) {
  const wrap = document.getElementById("modal-img-wrap");
  wrap.innerHTML = "";
  modalZoom = 1;
  modalPanX = 0;
  modalPanY = 0;
  modalImgEl = null;
  document.getElementById("modal-zoom-level").textContent = "100%";

  if (c.has_before) {
    wrap.appendChild(buildCompare(c.name));
    modalImgEl = wrap.querySelector(".modal-compare");
  } else {
    const img = new Image();
    img.src = imgUrl(c.name, "after");
    img.className = "modal-img";
    wrap.appendChild(img);
    modalImgEl = img;
  }

  document.getElementById("modal-name").textContent = c.name;
  const badge = document.getElementById("modal-status");
  badge.textContent = c.status;
  badge.className = `modal-status badge ${c.status}`;

  const viewerUrl = `/viewer.html?case=${encodeURIComponent(c.name)}`;
  document.getElementById("modal-interactive").href = viewerUrl;

  document.getElementById("modal-prev").style.display = modalIdx > 0 ? "" : "none";
  document.getElementById("modal-next").style.display = modalIdx < allCases.length - 1 ? "" : "none";
}

function navigateModal(dir) {
  const newIdx = modalIdx + dir;
  if (newIdx < 0 || newIdx >= allCases.length) return;
  modalIdx = newIdx;
  renderModalContent(allCases[modalIdx]);
}

function closeModal() {
  const overlay = document.getElementById("modal-overlay");
  if (overlay) overlay.classList.remove("open");
  document.body.style.overflow = "";
  modalIdx = -1;
}

function buildCompare(name) {
  const wrap = document.createElement("div");
  wrap.className = "modal-compare";

  const before = new Image();
  before.src = imgUrl(name, "before");
  before.className = "modal-img";

  const afterWrap = document.createElement("div");
  afterWrap.className = "modal-after-wrap";
  const after = new Image();
  after.src = imgUrl(name, "after");
  after.className = "modal-img";
  afterWrap.appendChild(after);

  const range = document.createElement("input");
  range.type = "range";
  range.className = "modal-range";
  range.min = 0;
  range.max = 100;
  range.value = 50;

  const setPos = () => {
    afterWrap.style.width = range.value + "%";
    after.style.setProperty("--cw", wrap.clientWidth + "px");
    after.style.width = wrap.clientWidth + "px";
  };
  range.addEventListener("input", setPos);
  before.addEventListener("load", setPos);
  after.addEventListener("load", setPos);
  window.addEventListener("resize", setPos);

  const tagL = document.createElement("span");
  tagL.className = "modal-tag left";
  tagL.textContent = "before";
  const tagR = document.createElement("span");
  tagR.className = "modal-tag right";
  tagR.textContent = "after";

  wrap.append(before, afterWrap, range, tagL, tagR);
  return wrap;
}

// Keyboard
document.addEventListener("keydown", (e) => {
  if (modalIdx < 0) return;
  if (e.key === "ArrowLeft") navigateModal(-1);
  else if (e.key === "ArrowRight") navigateModal(1);
  else if (e.key === "Escape") closeModal();
  else if (e.key === "+" || e.key === "=") modalZoomBy(1.25);
  else if (e.key === "-") modalZoomBy(0.8);
  else if (e.key === "0") modalZoomReset();
});

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
