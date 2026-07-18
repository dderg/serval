import { el } from "./api";
import { setConsoleValue } from "./console";
import { escapeHtml, renderSentLog, runGcode } from "./moonraker";
import { consoleSectionHtml } from "./shell";

// --- launchpad: a friendly form pad for the calibration macros --------------

type LpType = "int" | "float" | "string" | "enum" | "list";

interface LpParam {
  name: string;
  type: LpType;
  required?: boolean;
  dflt?: string;
  unit?: string;
  choices?: string[];
  hint?: string;
}

interface LpMacro {
  name: string;
  blurb: string;
  params: LpParam[];
}

interface LpGroup {
  label: string;
  macros: LpMacro[];
}

const AXIS_XY: LpParam = { name: "AXIS", type: "enum", choices: ["X", "Y"], dflt: "X" };
const AXIS_XYAB: LpParam = {
  name: "AXIS",
  type: "enum",
  choices: ["X", "Y", "A", "B"],
  dflt: "X",
  hint: "A/B are CoreXY diagonals",
};
const APPLY: LpParam = { name: "APPLY", type: "enum", choices: ["0", "1"], dflt: "0", hint: "1 writes the result" };
const SPEED: LpParam = { name: "SPEED", type: "float", dflt: "100", unit: "mm/s" };
const ACCEL: LpParam = { name: "ACCEL", type: "float", dflt: "3000", unit: "mm/s²" };
const ITERATIONS: LpParam = { name: "ITERATIONS", type: "int", dflt: "2", hint: "min 1" };
const DWELL_MS: LpParam = { name: "DWELL_MS", type: "int", dflt: "config", unit: "ms" };
const START: LpParam = { name: "START", type: "float", dflt: "config", unit: "mm" };
const END: LpParam = { name: "END", type: "float", dflt: "config", unit: "mm" };
const SERVO: LpParam = { name: "SERVO", type: "list", hint: "comma list — overrides AXIS" };
const SERVOS: LpParam = { name: "SERVOS", type: "list", hint: "CoreXY only, comma list" };
const ACCEL_CHIP: LpParam = { name: "ACCEL_CHIP", type: "string", dflt: "config" };
const TORQUE_NM: LpParam = { name: "TORQUE_NM", type: "float", dflt: "config", unit: "N·m" };
const INERTIA_KGM2: LpParam = { name: "INERTIA_KGM2", type: "float", dflt: "config", unit: "kg·m²" };
const XY_BOUNDS: LpParam[] = [
  { name: "X_START", type: "float", dflt: "config", unit: "mm" },
  { name: "X_END", type: "float", dflt: "config", unit: "mm" },
  { name: "Y_START", type: "float", dflt: "config", unit: "mm" },
  { name: "Y_END", type: "float", dflt: "config", unit: "mm" },
];
const GRID: LpParam[] = [
  { name: "ACCELS", type: "list", dflt: "config", unit: "mm/s²", hint: "comma list" },
  { name: "SPEEDS", type: "list", dflt: "config", unit: "mm/s", hint: "comma list" },
  ITERATIONS,
  DWELL_MS,
];

const LAUNCHPAD_GROUPS: LpGroup[] = [
  {
    label: "gains",
    macros: [
      {
        name: "SERVO_AUTOTUNE",
        blurb: "packaged tuning sequence — baseline, inertia, gains, dynamics, verify",
        params: [
          AXIS_XY,
          APPLY,
          { ...TORQUE_NM, hint: "required when APPLY=1" },
          { ...INERTIA_KGM2, hint: "required when APPLY=1" },
          { name: "SPEED_GAINS", type: "list", unit: "0.1 Hz", hint: "comma list for the sweep stage" },
          { ...DWELL_MS },
        ],
      },
      {
        name: "SERVO_APPLY_GAINS",
        blurb: "switch to manual tuning and write gain set 1",
        params: [
          AXIS_XY,
          SERVO,
          { name: "POS_GAIN", type: "int", dflt: "400", unit: "0.1 rad/s" },
          { name: "SPEED_GAIN", type: "int", dflt: "250", unit: "0.1 Hz" },
          { name: "INTEGRAL", type: "int", dflt: "3184", unit: "0.01 ms" },
        ],
      },
      {
        name: "SERVO_CALIBRATE_GAINS",
        blurb: "speed-gain sweep with capture; APPLY=1 writes the winner",
        params: [
          AXIS_XY,
          SERVO,
          { name: "SPEED_GAINS", type: "list", dflt: "500,650,800,1000", unit: "0.1 Hz", hint: "100..12500 each" },
          { name: "BASE_SPEED_GAIN", type: "int", unit: "0.1 Hz", hint: "pins non-swept servos; needs SERVO=" },
          START,
          END,
          SPEED,
          ACCEL,
          ITERATIONS,
          DWELL_MS,
          { name: "TAG", type: "string", dflt: "cal" },
          ACCEL_CHIP,
          APPLY,
        ],
      },
      {
        name: "SERVO_REFINE_GAIN",
        blurb: "1-D sensitivity sweep of one gain around its current value",
        params: [
          { name: "PARAM", type: "enum", required: true, choices: ["position", "speed", "integral"] },
          AXIS_XY,
          SERVO,
          { name: "VALUES", type: "list", hint: "explicit list — overrides SPAN/STEPS" },
          { name: "SPAN", type: "float", dflt: "0.3", hint: "0<span<1 fraction of current" },
          { name: "STEPS", type: "int", dflt: "5", hint: "min 2" },
          { name: "CURRENT", type: "int", hint: "default reads the drive" },
          START,
          END,
          SPEED,
          ACCEL,
          ITERATIONS,
          DWELL_MS,
          { name: "TAG", type: "string", dflt: "refine" },
          APPLY,
        ],
      },
      {
        name: "SERVO_GAIN_LADDER",
        blurb: "climb a gain by STEP until analysis flags trouble, then revert to SAFE",
        params: [
          { name: "SAFE", type: "int", required: true, hint: "device gain units" },
          { name: "START", type: "int", required: true, hint: "first climb value (gain units)" },
          { name: "MAX", type: "int", required: true, hint: "≥ START" },
          { name: "STEP", type: "int", dflt: "50", hint: "> 0" },
          { name: "PARAM", type: "enum", choices: ["position", "speed", "integral"], hint: "default climbs speed gain" },
          AXIS_XY,
          SERVO,
          SPEED,
          ACCEL,
          ITERATIONS,
          DWELL_MS,
          { name: "TAG", type: "string", dflt: "ladder" },
        ],
      },
    ],
  },
  {
    label: "dynamics & inertia",
    macros: [
      {
        name: "SERVO_MEASURE_INERTIA",
        blurb: "excitation grid for the inertia/friction fit",
        params: [
          AXIS_XY,
          { name: "NAME", type: "string", dflt: "ident" },
          SERVOS,
          ...XY_BOUNDS,
          START,
          END,
          ...GRID,
        ],
      },
      {
        name: "SERVO_FIT_DYNAMICS",
        blurb: "fit axis dynamics for torque feedforward; writes the profile",
        params: [
          AXIS_XY,
          { name: "NAME", type: "string", dflt: "ident" },
          { name: "DRIVE", type: "string", hint: "required on a multi-drive cartesian axis" },
          SERVOS,
          ...XY_BOUNDS,
          START,
          END,
          ...GRID,
          { ...TORQUE_NM, hint: "pair with INERTIA_KGM2 for the C00.06 pick" },
          INERTIA_KGM2,
        ],
      },
      {
        name: "SERVO_REFINE_DYNAMICS",
        blurb: "golden-section refine of one dynamics term on the running endpoint",
        params: [
          { name: "TERM", type: "enum", dflt: "MASS", choices: ["MASS", "VISCOUS", "COULOMB", "DIRECTION_SPLIT"] },
          AXIS_XY,
          SERVOS,
          { name: "PROFILE", type: "string", hint: "path; defaults to the node's profile" },
          { name: "LO", type: "float", dflt: "0.7", hint: "bracket must contain the baseline" },
          { name: "HI", type: "float", dflt: "1.3" },
          { name: "TOL", type: "float", dflt: "0.02" },
          { name: "MAX_EVALS", type: "int", dflt: "10", hint: "min 3" },
          ...XY_BOUNDS,
          START,
          END,
          ...GRID,
          { name: "TAG", type: "string", dflt: "refdyn" },
          { name: "NAME", type: "string", dflt: "refined_<term>" },
        ],
      },
      {
        name: "SERVO_CALIBRATE_INERTIA_RATIO",
        blurb: "identify the load inertia and print the recommended C00.06",
        params: [
          AXIS_XY,
          { name: "NAME", type: "string", dflt: "inertia" },
          { name: "DRIVE", type: "string", hint: "required on a multi-drive cartesian axis" },
          SERVOS,
          ...XY_BOUNDS,
          START,
          END,
          ...GRID,
          { ...TORQUE_NM, hint: "required unless rated_torque_nm is configured" },
          { ...INERTIA_KGM2, hint: "required unless rotor_inertia_kgm2 is configured" },
        ],
      },
      {
        name: "SERVO_SET_INERTIA_RATIO",
        blurb: "write C00.06 load inertia ratio in percent",
        params: [
          { name: "RATIO", type: "int", required: true, unit: "%", hint: "0..12000" },
          { name: "SERVO", type: "string", hint: "required unless exactly one servo is configured" },
        ],
      },
      {
        name: "SERVO_SWEEP_INERTIA",
        blurb: "empirical C00.06 ratio sweep with capture (measure only)",
        params: [
          { name: "RATIOS", type: "list", dflt: "40,70,100,130", unit: "%", hint: "0..12000 each" },
          AXIS_XY,
          SERVO,
          START,
          END,
          SPEED,
          ACCEL,
          ITERATIONS,
          DWELL_MS,
          { name: "TAG", type: "string", dflt: "inertia" },
        ],
      },
    ],
  },
  {
    label: "resonance & accel",
    macros: [
      {
        name: "SERVO_MEASURE_RINGDOWN",
        blurb: "short strokes into a full stop; fits the post-stop free decay",
        params: [
          AXIS_XYAB,
          { name: "SPEEDS", type: "list", dflt: "config", unit: "mm/s", hint: "comma list" },
          { name: "ACCEL", type: "float", dflt: "max_accel", unit: "mm/s²" },
          { name: "ITERATIONS", type: "int", dflt: "3" },
          { name: "DWELL_MS", type: "int", dflt: "≥1500", unit: "ms" },
          { name: "CRUISE_MS", type: "int", dflt: "200", unit: "ms" },
          { name: "TAG", type: "string", dflt: "ringdown" },
          ACCEL_CHIP,
        ],
      },
      {
        name: "SERVO_MEASURE_DIFFERENTIAL",
        blurb: "anti-phase chirp on one AWD belt pair — the differential FRF",
        params: [
          { name: "BELT", type: "enum", dflt: "A", choices: ["A", "B"] },
          { name: "FREQ_START", type: "float", dflt: "20", unit: "Hz" },
          { name: "FREQ_END", type: "float", dflt: "250", unit: "Hz", hint: "≤ 2000" },
          { name: "AMPLITUDE", type: "float", dflt: "0.05", unit: "mm", hint: "≤ 0.5" },
          { name: "HZ_PER_SEC", type: "float", dflt: "5", unit: "Hz/s" },
          { name: "DURATION", type: "float", dflt: "auto", unit: "s", hint: "≤ 300; 0 = from band" },
          { name: "RAMP", type: "float", dflt: "auto", unit: "s" },
          DWELL_MS,
          { name: "NAME", type: "string", dflt: "diff" },
        ],
      },
      {
        name: "SERVO_DIFF_DAMPER",
        blurb: "arm or disarm the differential belt-pair damper (GAIN=0 disarms)",
        params: [
          { name: "GAIN", type: "float", required: true, unit: "0.1% torque per mm/s", hint: "0 disarms" },
          { name: "BELT", type: "enum", dflt: "AB", choices: ["A", "B", "AB"] },
          { name: "CLAMP", type: "float", dflt: "50", unit: "×0.1% torque", hint: "≤ 300" },
          { name: "LPF_HZ", type: "float", dflt: "300", unit: "Hz" },
          { name: "LEAD_US", type: "float", dflt: "0", unit: "µs", hint: "0..5000" },
        ],
      },
      {
        name: "SERVO_SWEEP_ACCEL",
        blurb: "accel sweep to find the max non-saturating acceleration",
        params: [
          { name: "ACCELS", type: "list", required: true, unit: "mm/s²", hint: "comma list" },
          AXIS_XYAB,
          SPEED,
          START,
          END,
          ITERATIONS,
          DWELL_MS,
          { name: "TAG", type: "string", dflt: "accel" },
        ],
      },
    ],
  },
  {
    label: "strain",
    macros: [
      {
        name: "SERVO_MEASURE_STRAIN_MAP",
        blurb: "raster the bed with slow strokes — CoreXY only",
        params: [
          { name: "SPEED", type: "float", dflt: "50", unit: "mm/s" },
          { name: "ACCEL", type: "float", dflt: "1000", unit: "mm/s²" },
          { name: "LINE_SPACING", type: "float", dflt: "10", unit: "mm" },
          ...XY_BOUNDS,
          DWELL_MS,
          { name: "TAG", type: "string", dflt: "strain" },
          { name: "SYNC", type: "enum", dflt: "1", choices: ["0", "1"], hint: "1 zeroes preload first" },
        ],
      },
      {
        name: "SERVO_MEASURE_STRAIN_RESPONSE",
        blurb: "measure the belt stiffness matrix in the rolling regime",
        params: [
          { name: "SPEED", type: "float", dflt: "50", unit: "mm/s" },
          { name: "ACCEL", type: "float", dflt: "1000", unit: "mm/s²" },
          { name: "STEP_UM", type: "float", dflt: "50", unit: "µm" },
          { name: "SETTLE", type: "float", dflt: "0.8", unit: "s" },
          { name: "Y", type: "float", dflt: "area center", unit: "mm" },
          { name: "X_START", type: "float", dflt: "config", unit: "mm" },
          { name: "X_END", type: "float", dflt: "config", unit: "mm" },
          DWELL_MS,
          { name: "TAG", type: "string", dflt: "strainresp" },
          { name: "SYNC", type: "enum", dflt: "1", choices: ["0", "1"] },
        ],
      },
      {
        name: "SERVO_STRAIN_COMP_TUNE",
        blurb: "converge the strain map's stiffness matrix against reality",
        params: [
          { name: "RUN", type: "string", required: true, hint: "baseline raster run dir" },
          { name: "SPACING", type: "float", unit: "mm" },
          { name: "TOL", type: "float", dflt: "0.05", hint: "0<tol<0.5" },
          { name: "MAX_ITERS", type: "int", dflt: "5", hint: "min 1" },
          { name: "X", type: "float", dflt: "map zero", unit: "mm" },
          { name: "Y", type: "float", dflt: "map zero", unit: "mm" },
          { name: "SPEED", type: "float", dflt: "50", unit: "mm/s" },
          { name: "ACCEL", type: "float", dflt: "1000", unit: "mm/s²" },
          { name: "SETTLE", type: "float", dflt: "0.8", unit: "s" },
          DWELL_MS,
          { name: "TAG", type: "string", dflt: "straintune" },
          { name: "SYNC", type: "enum", dflt: "1", choices: ["0", "1"] },
        ],
      },
    ],
  },
  {
    label: "tracking",
    macros: [
      {
        name: "SERVO_MEASURE_TRACKING",
        blurb: "single accel/speed stroke with capture — the before/after check",
        params: [
          AXIS_XYAB,
          { name: "NAME", type: "string", dflt: "track" },
          ...XY_BOUNDS,
          START,
          END,
          { name: "SPEED", type: "float", dflt: "100", unit: "mm/s" },
          { name: "ACCEL", type: "float", dflt: "3000", unit: "mm/s²" },
          { name: "ITERATIONS", type: "int", dflt: "3" },
          DWELL_MS,
        ],
      },
    ],
  },
];

const MACROS: Record<string, LpMacro> = Object.fromEntries(
  LAUNCHPAD_GROUPS.flatMap((g) => g.macros.map((m) => [m.name, m]))
);

const VALUES_KEY = "servoCalLaunchpadValues";
const SELECTED_KEY = "servoCalLaunchpadSelected";

function buildCommand(macro: LpMacro, values: Record<string, string>): string {
  const parts = [macro.name];
  for (const p of macro.params) {
    const raw = (values[p.name] ?? "").trim();
    if (raw.length) parts.push(`${p.name}=${raw}`);
  }
  return parts.join(" ");
}

function missingRequired(macro: LpMacro, values: Record<string, string>): string[] {
  return macro.params
    .filter((p) => p.required && !(values[p.name] ?? "").trim().length)
    .map((p) => p.name);
}

function loadValues(): Record<string, Record<string, string>> {
  const raw = localStorage.getItem(VALUES_KEY);
  if (!raw) return {};
  try {
    const parsed = JSON.parse(raw);
    return parsed && typeof parsed === "object" ? parsed : {};
  } catch {
    return {};
  }
}

function saveValues(all: Record<string, Record<string, string>>) {
  localStorage.setItem(VALUES_KEY, JSON.stringify(all));
}

function loadSelected(): string | null {
  const name = localStorage.getItem(SELECTED_KEY);
  return name && MACROS[name] ? name : null;
}

function paramFieldHtml(p: LpParam, value: string): string {
  const meta = [
    p.dflt ? `default ${escapeHtml(p.dflt)}` : "",
    p.unit ? escapeHtml(p.unit) : "",
    p.hint ? escapeHtml(p.hint) : "",
  ]
    .filter(Boolean)
    .join(" · ");
  const req = p.required ? `<span class="lp-req" title="required">*</span>` : "";
  let control: string;
  if (p.type === "enum" && p.choices) {
    const blank = p.required ? "" : `<option value=""></option>`;
    const opts = p.choices
      .map((c) => `<option value="${escapeHtml(c)}"${c === value ? " selected" : ""}>${escapeHtml(c)}</option>`)
      .join("");
    control = `<select class="cell-input" data-lp-param="${p.name}">${blank}${opts}</select>`;
  } else {
    const placeholder = p.dflt ?? (p.required ? "required" : "");
    control =
      `<input type="text" class="cell-input" data-lp-param="${p.name}" ` +
      `value="${escapeHtml(value)}" placeholder="${escapeHtml(placeholder)}">`;
  }
  return (
    `<div class="lp-field">` +
    `<label>${escapeHtml(p.name)}${req}</label>` +
    control +
    (meta ? `<span class="lp-meta">${meta}</span>` : "") +
    `</div>`
  );
}

function formHtml(macro: LpMacro, values: Record<string, string>): string {
  return (
    `<div class="section-head"><h2>${escapeHtml(macro.name)}</h2>` +
    `<span class="note">${escapeHtml(macro.blurb)}</span></div>` +
    `<div class="lp-fields">${macro.params.map((p) => paramFieldHtml(p, values[p.name] ?? "")).join("")}</div>` +
    `<div class="lp-preview-row">` +
    `<code class="lp-preview" id="launchpad-preview"></code>` +
    `<div class="lp-actions">` +
    `<button id="launchpad-copy" title="drop this line into the console">to console</button>` +
    `<button id="launchpad-run" title="send this line over moonraker">run</button>` +
    `</div></div>` +
    `<div class="lp-missing note" id="launchpad-missing"></div>`
  );
}

function cardsHtml(selected: string | null): string {
  return LAUNCHPAD_GROUPS.map(
    (g) =>
      `<div class="lp-group"><h3>${escapeHtml(g.label)}</h3><div class="lp-cards">` +
      g.macros
        .map(
          (m) =>
            `<button class="lp-card${m.name === selected ? " active" : ""}" data-lp-macro="${m.name}">` +
            `<span class="lp-card-name">${escapeHtml(m.name)}</span>` +
            `<span class="lp-card-blurb">${escapeHtml(m.blurb)}</span></button>`
        )
        .join("") +
      `</div></div>`
  ).join("");
}

function launchpadShellHtml(): string {
  return (
    `<div class="workspace">` +
    `<main class="analysis">` +
    `<section class="launchpad-section">` +
    `<div class="section-head"><h2>calibration launchpad</h2>` +
    `<span class="note">pick a macro, fill the form, preview the exact g-code, run it</span></div>` +
    `<div id="launchpad-cards">${cardsHtml(loadSelected())}</div>` +
    `</section>` +
    `</main>` +
    `<aside class="controls">` +
    `<section class="launchpad-form" id="launchpad-form">` +
    `<p class="note">select a macro on the left to build its command</p>` +
    `</section>` +
    consoleSectionHtml({}) +
    `</aside>` +
    `</div>`
  );
}

function readFormValues(): Record<string, string> {
  const values: Record<string, string> = {};
  document.querySelectorAll<HTMLInputElement | HTMLSelectElement>("#launchpad-form [data-lp-param]").forEach((f) => {
    const name = f.dataset.lpParam;
    if (name) values[name] = f.value;
  });
  return values;
}

let selectedMacro: string | null = null;

function persistCurrent(values: Record<string, string>) {
  if (!selectedMacro) return;
  const all = loadValues();
  all[selectedMacro] = values;
  saveValues(all);
}

function updatePreview() {
  const macro = selectedMacro ? MACROS[selectedMacro] : null;
  if (!macro) return;
  const values = readFormValues();
  const preview = el("launchpad-preview");
  const line = buildCommand(macro, values);
  if (preview) preview.textContent = line;
  const missing = missingRequired(macro, values);
  const missingEl = el("launchpad-missing");
  if (missingEl) {
    missingEl.textContent = missing.length ? `fill required: ${missing.join(", ")}` : "";
  }
  const run = el<HTMLButtonElement>("launchpad-run");
  if (run) run.disabled = missing.length > 0;
  persistCurrent(values);
}

function selectMacro(name: string) {
  if (!MACROS[name]) throw new Error(`launchpad: unknown macro ${name}`);
  selectedMacro = name;
  localStorage.setItem(SELECTED_KEY, name);
  const form = el("launchpad-form");
  if (form) form.innerHTML = formHtml(MACROS[name], loadValues()[name] ?? {});
  document.querySelectorAll<HTMLElement>(".lp-card").forEach((c) => {
    c.classList.toggle("active", c.dataset.lpMacro === name);
  });
  updatePreview();
}

function bindLaunchpad() {
  const cards = el("launchpad-cards");
  if (cards) {
    cards.addEventListener("click", (ev) => {
      const card = (ev.target as HTMLElement).closest<HTMLElement>(".lp-card");
      if (card && card.dataset.lpMacro) selectMacro(card.dataset.lpMacro);
    });
  }
  const form = el("launchpad-form");
  if (form) {
    form.addEventListener("input", updatePreview);
    form.addEventListener("change", updatePreview);
    form.addEventListener("click", (ev) => {
      const target = ev.target as HTMLElement;
      if (target.id === "launchpad-copy") onCopy();
      if (target.id === "launchpad-run") onRun();
    });
  }
  selectedMacro = loadSelected();
  if (selectedMacro) selectMacro(selectedMacro);
}

function onCopy() {
  const line = el("launchpad-preview")?.textContent ?? "";
  if (line) setConsoleValue(line, true);
}

async function onRun() {
  const macro = selectedMacro ? MACROS[selectedMacro] : null;
  if (!macro) return;
  const values = readFormValues();
  if (missingRequired(macro, values).length) return;
  await runGcode([buildCommand(macro, values)], "launchpad");
  renderSentLog();
}

export { LAUNCHPAD_GROUPS, buildCommand, missingRequired, launchpadShellHtml, bindLaunchpad };
