"use strict";

const casesEl = document.getElementById("cases");
const summaryEl = document.getElementById("summary");
const bannerEl = document.getElementById("banner");
const acceptAllBtn = document.getElementById("accept-all");
const titleEl = document.getElementById("title");

let cacheBust = Date.now();

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
  for (const c of review) casesEl.appendChild(card(c, readOnly));
}

function imgUrl(name, which) {
  return `/img/${encodeURIComponent(name)}/${which}.png?t=${cacheBust}`;
}

function card(c, readOnly) {
  const el = document.createElement("section");
  el.className = "card";

  const head = document.createElement("div");
  head.className = "card-head";
  head.innerHTML =
    `<span class="case-name">${c.name}</span>` +
    `<span class="badge ${c.status}">${c.status}</span>`;
  el.appendChild(head);

  const actions = document.createElement("div");
  actions.className = "card-actions";

  const viewerUrl = `/viewer.html?case=${encodeURIComponent(c.name)}`;
  const interactiveBtn = document.createElement("a");
  interactiveBtn.className = "btn btn-primary";
  interactiveBtn.href = viewerUrl;
  interactiveBtn.target = "_blank";
  interactiveBtn.rel = "noreferrer";
  interactiveBtn.textContent = "Interactive Viewer";
  actions.appendChild(interactiveBtn);

  const pngBtn = document.createElement("button");
  pngBtn.className = "btn";
  pngBtn.textContent = "View PNG";
  pngBtn.onclick = () => togglePng(el, c, readOnly, pngBtn);
  actions.appendChild(pngBtn);

  el.appendChild(actions);

  const imgSlot = document.createElement("div");
  imgSlot.className = "img-slot";
  imgSlot.hidden = true;
  el.appendChild(imgSlot);

  return el;
}

function togglePng(cardEl, c, readOnly, btn) {
  const slot = cardEl.querySelector(".img-slot");
  if (!slot.hidden) {
    slot.hidden = true;
    slot.innerHTML = "";
    btn.textContent = "View PNG";
    return;
  }
  slot.hidden = false;
  btn.textContent = "Hide PNG";

  if (c.has_before) {
    slot.appendChild(compare(c.name));
  } else {
    const img = new Image();
    img.src = imgUrl(c.name, readOnly ? "after" : "after");
    img.className = "png-img";
    slot.appendChild(img);
  }
}

function compare(name) {
  const wrap = document.createElement("div");
  wrap.className = "compare";

  const before = new Image();
  before.src = imgUrl(name, "before");

  const afterWrap = document.createElement("div");
  afterWrap.className = "after-wrap";
  const after = new Image();
  after.src = imgUrl(name, "after");
  afterWrap.appendChild(after);

  const range = document.createElement("input");
  range.type = "range";
  range.className = "range";
  range.min = 0;
  range.max = 100;
  range.value = 50;

  const setPos = () => {
    const pct = range.value;
    afterWrap.style.width = pct + "%";
    after.style.setProperty("--cw", wrap.clientWidth + "px");
    after.style.width = wrap.clientWidth + "px";
  };
  range.addEventListener("input", setPos);
  before.addEventListener("load", setPos);
  after.addEventListener("load", setPos);
  window.addEventListener("resize", setPos);

  wrap.append(
    before,
    afterWrap,
    range,
    tag("left", "before (baseline)"),
    tag("right", "after (current)"),
  );
  return wrap;
}

function tag(side, text) {
  const t = document.createElement("span");
  t.className = "tag " + side;
  t.textContent = text;
  return t;
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
