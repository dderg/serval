"use strict";

const casesEl = document.getElementById("cases");
const summaryEl = document.getElementById("summary");
const bannerEl = document.getElementById("banner");
const acceptAllBtn = document.getElementById("accept-all");

// Bump on every (re)load so accepted/changed images are re-fetched, not cached.
let cacheBust = Date.now();

async function load() {
  const data = await fetch("/api/cases").then((r) => r.json());
  cacheBust = Date.now();
  render(data);
}

function render(data) {
  bannerEl.hidden = !data.error;
  if (data.error) bannerEl.textContent = data.error;

  const review = data.review || [];
  const bits = [];
  if (data.exact) bits.push(`${data.exact} match baseline`);
  bits.push(`${review.length} to review`);
  summaryEl.textContent = bits.join(" · ");
  acceptAllBtn.disabled = review.length === 0;

  casesEl.innerHTML = "";
  if (review.length === 0 && !data.error) {
    const div = document.createElement("div");
    div.className = "empty";
    div.textContent = "Nothing to review — every case matches its baseline.";
    casesEl.appendChild(div);
    return;
  }
  for (const c of review) casesEl.appendChild(card(c));
}

function imgUrl(name, which) {
  return `/img/${encodeURIComponent(name)}/${which}.png?t=${cacheBust}`;
}

function card(c) {
  const el = document.createElement("section");
  el.className = "card";

  const head = document.createElement("div");
  head.className = "card-head";
  head.innerHTML =
    `<span class="case-name">${c.name}</span>` +
    `<span class="badge ${c.status}">${c.status}</span>`;
  el.appendChild(head);

  el.appendChild(c.has_before ? compare(c.name) : single(c.name));

  const foot = document.createElement("div");
  foot.className = "card-foot";
  if (c.has_before) {
    foot.appendChild(link("open before", imgUrl(c.name, "before")));
  }
  foot.appendChild(link("open after", imgUrl(c.name, "after")));
  el.appendChild(foot);
  return el;
}

function single(name) {
  const wrap = document.createElement("div");
  wrap.className = "single";
  const img = new Image();
  img.src = imgUrl(name, "after");
  wrap.appendChild(img);
  return wrap;
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
    // Keep the clipped "after" image the full card width so it lines up.
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

function link(text, href) {
  const a = document.createElement("a");
  a.textContent = text;
  a.href = href;
  a.target = "_blank";
  a.rel = "noreferrer";
  return a;
}

async function postAccept(payload) {
  const data = await fetch("/api/accept", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  }).then((r) => r.json());
  // When nothing is left to review the server shuts itself down; show the
  // completion note instead of re-rendering an empty list.
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
