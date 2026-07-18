import { html, render } from "../vendor/htm-preact-standalone-3.1.1.mjs";
import { api, el } from "./api.js";
import { setConsoleValue } from "./console.js";
import { runGcode } from "./moonraker.js";
import { refresh } from "./runs.js";
import { currentPageDef } from "./shell.js";
import { state } from "./state.js";
import { notify, useStore } from "./store.js";

// --- drive tuning grid --------------------------------------------------------
//
// Renders purely from GET /api/drive_state (servo_tuning.PANEL_PARAMS shape,
// docs/rewrite/servo-tuning-profiles.md) as a param × motor grid: one column
// per motor plus an "all" setter, so a 4-motor bench never needs the same
// value typed four times and every edit's motor scope is visible. Each page
// shows only its own param groups. Every cell shows and takes the RAW
// register value exactly as stored on the drive — the unit label names the
// LSB (e.g. "0.1 Hz") instead of the UI converting, so what you type is
// what SERVO_TUNE writes. Pure helpers first — autofill derivation,
// changed-cell diffing, SERVO_TUNE line building (always with an explicit
// MOTORS= list) — the logic a Rust test asserts is present and exercisable
// without a browser; the preact components that render the grid follow.
// Preact owns #drive-panel: renderDriveGroups() mounts once per page build,
// then every state mutation just notifies the store.

const GROUP_ORDER = ["gains", "filters", "notch", "speed_observer", "disturbance_observer", "load"];
const OTHER_GROUP = "other";
const AUTOFILL_SOURCE_PARAM = "speed_gain";
const DRIVE_REFRESH_POLL_MS = 1000;
const DRIVE_REFRESH_TIMEOUT_MS = 15000;

function deriveGainPositionFromSpeed(speedGainRaw) {
  return Math.round(speedGainRaw * 1.6);
}

function deriveGainIntegralFromSpeed(speedGainRaw) {
  return Math.round(1250000 / speedGainRaw);
}

const AUTOFILL_FORMULAS = {
  gain_position_from_speed: deriveGainPositionFromSpeed,
  gain_integral_from_speed: deriveGainIntegralFromSpeed,
};

function paramGroupSection(param) {
  return GROUP_ORDER.includes(param.group) ? param.group : OTHER_GROUP;
}

function groupParams(params) {
  const sections = new Map([...GROUP_ORDER, OTHER_GROUP].map((g) => [g, []]));
  for (const p of params) sections.get(paramGroupSection(p)).push(p);
  return sections;
}

function motorNames(motors) {
  return Object.keys(motors).sort();
}

function motorRawValues(motors, cCode) {
  return motorNames(motors).map((m) => motors[m][cCode]);
}

function valuesAgree(values) {
  return values.length > 0 && values.every((v) => v === values[0]);
}

function pinnedEntries(configPins, cCode) {
  const out = {};
  for (const motor of Object.keys(configPins || {}).sort()) {
    const pins = configPins[motor] || {};
    if (Object.prototype.hasOwnProperty.call(pins, cCode)) out[motor] = pins[cCode];
  }
  return out;
}

/// Effective raw value of one grid cell: the session's pending edit if any,
/// else the drive's reading from the last dump.
function cellRaw(param, motor) {
  const pend = state.drive.pending[param.name];
  if (pend && pend[motor] !== undefined) return pend[motor];
  return state.drive.data.motors[motor][param.c_code];
}

/// Which cells differ from the drive_state's per-motor readings, given this
/// session's pending edits. `pending[name]` is always a `{motor: raw}` map —
/// the "all" column just writes every motor at once.
function diffChangedParams(params, motors, pending) {
  const changed = [];
  for (const p of params) {
    const pend = pending[p.name];
    if (pend === undefined) continue;
    const cells = [];
    for (const motor of Object.keys(pend).sort()) {
      if (motors[motor][p.c_code] !== pend[motor]) cells.push({ motor, value: pend[motor] });
    }
    if (cells.length) changed.push({ name: p.name, cells });
  }
  return changed;
}

/// One SERVO_TUNE line per (param, value), motors grouped — the MOTORS= list
/// is always explicit so the log and the preview state exactly which drives
/// a write targets.
function buildServoTuneCommands(changed) {
  const lines = [];
  for (const c of changed) {
    const byValue = new Map();
    for (const { motor, value } of c.cells) {
      if (!byValue.has(value)) byValue.set(value, []);
      byValue.get(value).push(motor);
    }
    for (const [value, motorList] of byValue) {
      lines.push(`SERVO_TUNE PARAM=${c.name} VALUE=${value} MOTORS=${motorList.join(",")}`);
    }
  }
  return lines;
}

function paramByName(name) {
  return state.drive.data.params.find((p) => p.name === name);
}

/// speed_gain's effective per-motor raws — the input every autofill formula
/// maps over.
function currentSpeedGainByMotor() {
  const speedParam = paramByName(AUTOFILL_SOURCE_PARAM);
  const out = {};
  for (const m of motorNames(state.drive.data.motors)) {
    out[m] = cellRaw(speedParam, m);
  }
  return out;
}

/// speed_gain changed: push derived per-motor values into every autofill
/// target the user hasn't dirtied (edited directly) this session.
function propagateAutofill() {
  const speedByMotor = currentSpeedGainByMotor();
  for (const param of state.drive.data.params) {
    const formula = AUTOFILL_FORMULAS[param.autofill];
    if (!formula || state.drive.dirty.has(param.name)) continue;
    state.drive.pending[param.name] = Object.fromEntries(
      Object.entries(speedByMotor).map(([m, v]) => [m, formula(v)])
    );
  }
}

function rederiveAutofillTarget(name) {
  const formula = AUTOFILL_FORMULAS[paramByName(name).autofill];
  if (!formula) return;
  state.drive.pending[name] = Object.fromEntries(
    Object.entries(currentSpeedGainByMotor()).map(([m, v]) => [m, formula(v)])
  );
}

function formatAge(ageS) {
  if (ageS < 60) return `${ageS.toFixed(0)}s`;
  const m = Math.floor(ageS / 60);
  const s = Math.round(ageS % 60);
  return `${m}m${s}s`;
}

function currentDriveAgeS() {
  if (!state.drive.data) return null;
  return state.drive.data.age_s + (Date.now() - state.drive.fetchedAtMs) / 1000;
}

/// The refresh button must render even with no drive state at all —
/// SERVO_DUMP_TUNING is what creates drive_state.json in the first place,
/// so hiding the button behind loaded data would deadlock a fresh bench.
/// Rebuilt only once so the 1 s age ticker doesn't wipe the refresh
/// status text mid-dump.
function renderDriveBanner() {
  const banner = el("drive-state-banner");
  if (!el("drive-refresh-btn")) {
    banner.innerHTML =
      `<span class="note" id="drive-age"></span> ` +
      `<button id="drive-refresh-btn" title="SERVO_DUMP_TUNING and re-read">refresh</button>` +
      `<span id="drive-refresh-status" class="note"></span>`;
    el("drive-refresh-btn").addEventListener("click", refreshDriveState);
  }
  el("drive-age").textContent = state.drive.data
    ? `drive state ${formatAge(currentDriveAgeS())} old`
    : "no drive state yet — press refresh to read the drives";
}

function shortMotorLabel(motor) {
  return motor.replace(/^motor_/, "");
}

function stageCellEdit(param, motorSel, rawText) {
  const raw = parseInt(rawText, 10);
  if (Number.isNaN(raw)) return;
  const targets = motorSel === "*" ? motorNames(state.drive.data.motors) : [motorSel];
  const existing = { ...(state.drive.pending[param.name] || {}) };
  for (const m of targets) existing[m] = raw;
  state.drive.pending[param.name] = existing;
  if (param.name === AUTOFILL_SOURCE_PARAM) {
    propagateAutofill();
  } else if (param.autofill) {
    state.drive.dirty.add(param.name);
  }
  renderDriveGroups();
}

function OptionList({ param }) {
  return Object.entries(param.options).map(
    ([v, label]) => html`<option key=${v} value=${v}>${v}: ${label}</option>`
  );
}

function CellInput({ param, motor }) {
  const raw = cellRaw(param, motor);
  const original = state.drive.data.motors[motor][param.c_code];
  const cls = ["cell-input"];
  if (raw !== original) cls.push("pending");
  const others = motorNames(state.drive.data.motors)
    .filter((m) => m !== motor)
    .map((m) => cellRaw(param, m));
  if (others.some((v) => v !== raw)) cls.push("drift");
  const title = `${motor} — raw ${raw}${raw !== original ? ` (drive has ${original})` : ""}`;
  const onChange = (e) => stageCellEdit(param, motor, e.target.value);
  if (param.options) {
    return html`<select class=${cls.join(" ")} title=${title} value=${String(raw)} onChange=${onChange}>
      <${OptionList} param=${param} />
    </select>`;
  }
  return html`<input type="number" step="1" class=${cls.join(" ")} value=${raw} title=${title} onChange=${onChange} />`;
}

function AllInput({ param }) {
  const motors = motorNames(state.drive.data.motors);
  const values = motors.map((m) => cellRaw(param, m));
  const agree = valuesAgree(values);
  const cls = ["cell-input", "all"];
  if (motors.some((m) => cellRaw(param, m) !== state.drive.data.motors[m][param.c_code])) {
    cls.push("pending");
  }
  const title = agree
    ? "set all motors"
    : `set all motors — currently ${motors.map((m, i) => `${shortMotorLabel(m)}=${values[i]}`).join(" ")}`;
  const onChange = (e) => stageCellEdit(param, "*", e.target.value);
  if (param.options) {
    return html`<select class=${cls.join(" ")} title=${title} value=${agree ? String(values[0]) : ""} onChange=${onChange}>
      <option value="" disabled>${agree ? "" : "mixed"}</option>
      <${OptionList} param=${param} />
    </select>`;
  }
  return html`<input
    type="number"
    step="1"
    class=${cls.join(" ")}
    value=${agree ? values[0] : ""}
    placeholder=${agree ? "" : "mixed"}
    title=${title}
    onChange=${onChange}
  />`;
}

function ParamLabel({ param, section }) {
  const pins = pinnedEntries(state.drive.data.config_pins, param.c_code);
  const pinnedNames = Object.keys(pins);
  return html`<span title=${`${param.description} (${param.c_code})`}>${param.name}</span>${" "}
    ${param.unit ? html`<span class="unit">${param.unit}</span>` : null}
    ${pinnedNames.length
      ? html`<span
          class="badge pin"
          title=${`pinned in config — a restart re-applies ${[...new Set(pinnedNames.map((m) => pins[m]))].join("/")}`}
        >pin</span>`
      : null}
    ${section === OTHER_GROUP ? html`<span class="hint">(${param.group})</span>` : null}
    ${state.drive.dirty.has(param.name)
      ? html`<a
          href="#"
          class="rederive"
          title="restore the autofill link"
          onClick=${(e) => {
            e.preventDefault();
            state.drive.dirty.delete(param.name);
            rederiveAutofillTarget(param.name);
            renderDriveGroups();
          }}
        >re-derive</a>`
      : null}`;
}

/// Adaptive-notch recipe (A6-EC manual 7.10): reset the notch parameters,
/// hand notches 1-2 to the drive, or take them back (0 keeps whatever the
/// drive last wrote). Each button only STAGES adaptive_notch_mode for all
/// motors — the write happens through the apply button like any grid edit.
const NOTCH_QUICK_ACTIONS = [
  { label: "reset notch params", value: 3 },
  { label: "1 adaptive", value: 1 },
  { label: "2 adaptive", value: 2 },
  { label: "disable adaptive", value: 0 },
];

function stageAdaptiveNotchMode(value) {
  const staged = { ...(state.drive.pending.adaptive_notch_mode || {}) };
  for (const m of motorNames(state.drive.data.motors)) {
    staged[m] = value;
  }
  state.drive.pending.adaptive_notch_mode = staged;
  renderDriveGroups();
}

function NotchQuickActions() {
  return html`<details
    class="adaptive-actions"
    open=${state.drive.adaptiveOpen}
    onToggle=${(e) => {
      state.drive.adaptiveOpen = e.target.open;
    }}
  >
    <summary>adaptive notch recipes</summary>
    <div class="quick-actions">
      ${NOTCH_QUICK_ACTIONS.map(
        (a) => html`<button
          key=${a.value}
          class="quick-action"
          title=${`stages adaptive_notch_mode=${a.value} for all motors — nothing is written until apply`}
          onClick=${() => stageAdaptiveNotchMode(a.value)}
        >${a.label}</button>`
      )}
    </div>
    <p class="hint">stages adaptive_notch_mode — review in the pending list, then apply</p>
  </details>`;
}

const NOTCH_ROW_KINDS = ["freq", "width", "depth"];

function notchMatrix(params) {
  const byKey = new Map();
  const nums = new Set();
  const leftover = [];
  for (const p of params) {
    const m = /^notch_(\d+)_(freq|width|depth)$/.exec(p.name);
    if (m) {
      nums.add(Number(m[1]));
      byKey.set(`${m[1]}:${m[2]}`, p);
    } else {
      leftover.push(p);
    }
  }
  return { nums: [...nums].sort((a, b) => a - b), byKey, leftover };
}

/// The compact notch view: one column per notch, freq/width/depth rows, one
/// input per cell that stages the value for every motor (notches are
/// per-axis physics — on corexy every motor sees the same belt, so
/// per-motor notch tables are noise; the per-motor toggle remains for
/// drives that genuinely disagree).
function NotchCompactGrid({ nums, byKey }) {
  return html`<table class="param-grid notch-grid">
    <thead>
      <tr>
        <th class="param-col"></th>
        ${nums.map((n) => html`<th key=${n}>notch ${n}</th>`)}
      </tr>
    </thead>
    <tbody>
      ${NOTCH_ROW_KINDS.map((kind) => {
        const first = byKey.get(`${nums[0]}:${kind}`);
        return html`<tr key=${kind}>
          <td class="param-col">
            ${kind}${first && first.unit ? html` <span class="unit">${first.unit}</span>` : null}
          </td>
          ${nums.map((n) => {
            const p = byKey.get(`${n}:${kind}`);
            return html`<td key=${n}>${p ? html`<${AllInput} param=${p} />` : null}</td>`;
          })}
        </tr>`;
      })}
    </tbody>
  </table>`;
}

function PerMotorTable({ params, group, motors }) {
  return html`<table class="param-grid">
    <thead>
      <tr>
        <th class="param-col"></th>
        ${motors.map((m) => html`<th key=${m} title=${m}>${shortMotorLabel(m)}</th>`)}
        <th class="all-col">all</th>
      </tr>
    </thead>
    <tbody>
      ${params.map(
        (p) => html`<tr key=${p.name} data-param=${p.name}>
          <td class="param-col"><${ParamLabel} param=${p} section=${group} /></td>
          ${motors.map((m) => html`<td key=${m}><${CellInput} param=${p} motor=${m} /></td>`)}
          <td class="all-col"><${AllInput} param=${p} /></td>
        </tr>`
      )}
    </tbody>
  </table>`;
}

function NotchViewToggle({ label }) {
  return html` <a
    href="#"
    class="notch-view-toggle hint"
    onClick=${(e) => {
      e.preventDefault();
      state.drive.notchPerMotor = !state.drive.notchPerMotor;
      renderDriveGroups();
    }}
  >${label}</a>`;
}

function DriveGroups() {
  const def = currentPageDef();
  const motors = motorNames(state.drive.data.motors);
  const sections = groupParams(state.drive.data.params);
  const groups = [];
  for (const [group, params] of sections) {
    if (!params.length) continue;
    if (def.groups && group !== OTHER_GROUP && !def.groups.includes(group)) continue;
    if (group === "notch" && !state.drive.notchPerMotor) {
      const { nums, byKey, leftover } = notchMatrix(params);
      groups.push(html`<div key=${group} class="param-group">
        <h3>notch<${NotchViewToggle} label="per-motor view" /></h3>
        <${NotchCompactGrid} nums=${nums} byKey=${byKey} />
        ${leftover.length
          ? html`<${PerMotorTable} params=${leftover} group=${group} motors=${motors} />`
          : null}
        <${NotchQuickActions} />
      </div>`);
      continue;
    }
    groups.push(html`<div key=${group} class="param-group">
      <h3>
        ${group.replace(/_/g, " ")}${group === "notch"
          ? html`<${NotchViewToggle} label="compact view" />`
          : null}
      </h3>
      <${PerMotorTable} params=${params} group=${group} motors=${motors} />
      ${group === "notch" ? html`<${NotchQuickActions} />` : null}
    </div>`);
  }
  return groups;
}

function DrivePanel() {
  useStore();
  const data = state.drive.data;
  const changed = data ? diffChangedParams(data.params, data.motors, state.drive.pending) : [];
  const lines = buildServoTuneCommands(changed);
  return html`<div id="drive-groups">
      ${data
        ? html`<${DriveGroups} />`
        : html`<p class="note">
            no drive state yet — press refresh in the top bar to read every mapped parameter off
            the drives (SERVO_DUMP_TUNING)
          </p>`}
    </div>
    <div id="pending-preview" class="pending-preview">
      ${lines.map((l) => html`<div key=${l} class="pending-line">${l}</div>`)}
    </div>
    <div class="row">
      <button id="drive-apply-btn" disabled=${lines.length === 0} onClick=${applyDriveChanges}>
        apply
      </button>
      <span class="note" id="drive-changed-count">
        ${data ? (lines.length ? `${lines.length} write(s) pending` : "no changes pending") : ""}
      </span>
    </div>`;
}

let mountedDrivePanel = null;

function renderDriveGroups() {
  const container = el("drive-panel");
  if (container && mountedDrivePanel !== container) {
    if (mountedDrivePanel) render(null, mountedDrivePanel);
    mountedDrivePanel = container;
    render(html`<${DrivePanel} />`, container);
  }
  notify();
}

async function loadDriveState() {
  try {
    const data = await api("/api/drive_state");
    state.drive.data = data;
    state.drive.fetchedAtMs = Date.now();
  } catch (e) {
    state.drive.data = null;
    console.error(e);
  }
  state.drive.pending = {};
  state.drive.dirty = new Set();
  renderDriveBanner();
  renderDriveGroups();
}

async function refreshDriveState() {
  const statusEl = el("drive-refresh-status");
  const priorAge = state.drive.data ? currentDriveAgeS() : Infinity;
  if (statusEl) statusEl.textContent = " dumping…";
  await runGcode(["SERVO_DUMP_TUNING"], "refresh");
  const deadline = Date.now() + DRIVE_REFRESH_TIMEOUT_MS;
  while (Date.now() < deadline) {
    await new Promise((resolve) => setTimeout(resolve, DRIVE_REFRESH_POLL_MS));
    let data;
    try {
      data = await api("/api/drive_state");
    } catch (e) {
      continue;
    }
    if (data.age_s < priorAge) {
      state.drive.data = data;
      state.drive.fetchedAtMs = Date.now();
      state.drive.pending = {};
      state.drive.dirty = new Set();
      renderDriveBanner();
      renderDriveGroups();
      return;
    }
  }
  const late = el("drive-refresh-status");
  if (late) late.textContent = " refresh timed out — drive_state.json never got newer";
}

/// Apply sends the previewed SERVO_TUNE batch, then reloads
/// drive_state.json — SERVO_TUNE readback-verifies each write and patches
/// the file in place, so re-reading it is enough; the full
/// SERVO_DUMP_TUNING drive re-read stays behind the refresh button.
async function applyDriveChanges() {
  const changed = diffChangedParams(state.drive.data.params, state.drive.data.motors, state.drive.pending);
  const lines = buildServoTuneCommands(changed);
  if (!lines.length) return;
  await runGcode(lines, "apply");
  await loadDriveState();
}

// --- sweep re-run ---------------------------------------------------------

/// The stroke's SPEED/ACCEL must ride along on a re-run: they shape the
/// excitation, so a "same sweep" at the command defaults is not the same
/// sweep and its results are not comparable to the original.
function strokeSuffix(manifest, includeAccel) {
  const plan = manifest.stroke_plan || {};
  let suffix = "";
  if (plan.speed != null) suffix += ` SPEED=${plan.speed}`;
  if (includeAccel && plan.accel != null) suffix += ` ACCEL=${plan.accel}`;
  return suffix;
}

/// Old manifests predate the recorded `command` field; rebuilding from the
/// manifest is a lossy fallback that only knows the parameters listed here.
function reconstructCommand(manifest) {
  if (manifest.command) return manifest.command;
  const tag = manifest.tag || "cal";
  const axis = manifest.axis || "X";
  const iterations = (manifest.stroke_plan && manifest.stroke_plan.iterations) || 1;
  const sweptKeys = (manifest.steps || []).map((s) => Object.keys(s.swept || {}));
  const commonKeys = sweptKeys.reduce((a, b) => a.filter((k) => b.includes(k)), sweptKeys[0] || []);
  const common = `AXIS=${axis} ITERATIONS=${iterations} TAG=${tag}`;

  switch (manifest.experiment) {
    case "gain_sweep": {
      const values = manifest.steps.map((s) => s.swept.speed).join(",");
      return `SERVO_CALIBRATE_GAINS SPEED_GAINS=${values} ${common}${strokeSuffix(manifest, true)}`;
    }
    case "gain_ladder": {
      const speeds = manifest.steps.map((s) => s.swept.speed);
      const safe = speeds[0];
      const start = speeds.length > 1 ? speeds[1] : safe;
      const step = speeds.length > 2 ? speeds[2] - speeds[1] : 50;
      const max = speeds[speeds.length - 1];
      return `SERVO_GAIN_LADDER SAFE=${safe} START=${start} STEP=${step} MAX=${max} ${common}${strokeSuffix(manifest, true)}`;
    }
    case "refine_sweep": {
      const param = commonKeys.length === 1 ? commonKeys[0] : "speed";
      const values = manifest.steps.map((s) => s.swept[param]).join(",");
      return `SERVO_REFINE_GAIN PARAM=${param} VALUES=${values} ${common}${strokeSuffix(manifest, true)}`;
    }
    case "inertia_sweep": {
      const values = manifest.steps.map((s) => s.swept.ratio ?? Object.values(s.swept)[0]).join(",");
      return `SERVO_SWEEP_INERTIA RATIOS=${values} ${common}${strokeSuffix(manifest, true)}`;
    }
    case "accel_sweep": {
      const values = manifest.steps.map((s) => s.swept.accel ?? Object.values(s.swept)[0]).join(",");
      return `SERVO_SWEEP_ACCEL ACCELS=${values} ${common}${strokeSuffix(manifest, false)}`;
    }
    case "strain_map": {
      const plan = manifest.stroke_plan;
      return (
        `SERVO_MEASURE_STRAIN_MAP LINE_SPACING=${plan.line_spacing} SPEED=${plan.speed} ` +
        `ACCEL=${plan.accel} X_START=${plan.x_start} X_END=${plan.x_end} ` +
        `Y_START=${plan.y_start} Y_END=${plan.y_end} DWELL_MS=${plan.dwell_ms} ` +
        `${plan.zero_sync ? "" : "SYNC=0 "}TAG=${tag}`
      );
    }
    case "differential": {
      const plan = manifest.stroke_plan;
      return (
        `SERVO_MEASURE_DIFFERENTIAL BELT=${plan.belt} FREQ_START=${plan.freq_start} ` +
        `FREQ_END=${plan.freq_end} AMPLITUDE=${plan.amplitude} DURATION=${plan.duration} ` +
        `RAMP=${plan.ramp} DWELL_MS=${plan.dwell_ms} NAME=${tag}`
      );
    }
    case "ringdown": {
      const plan = manifest.stroke_plan;
      const cruise = plan.cruise_ms == null ? "" : `CRUISE_MS=${plan.cruise_ms} `;
      return (
        `SERVO_MEASURE_RINGDOWN SPEEDS=${(plan.speeds || []).join(",")} ` +
        `ACCEL=${plan.accel} DWELL_MS=${plan.dwell_ms} ${cruise}${common}`
      );
    }
    default:
      return `; ${manifest.experiment} has no known reconstruction — edit by hand`;
  }
}

function loadRerunForm(name) {
  const detail = state.details.get(name);
  if (!detail || !detail.manifest) return;
  const label = el("form-run-name");
  if (label) label.textContent = `from ${name}`;
  setConsoleValue(reconstructCommand(detail.manifest), false);
}

export { GROUP_ORDER, OTHER_GROUP, AUTOFILL_SOURCE_PARAM, DRIVE_REFRESH_POLL_MS, DRIVE_REFRESH_TIMEOUT_MS, deriveGainPositionFromSpeed, deriveGainIntegralFromSpeed, AUTOFILL_FORMULAS, paramGroupSection, groupParams, motorNames, motorRawValues, valuesAgree, pinnedEntries, cellRaw, diffChangedParams, buildServoTuneCommands, paramByName, currentSpeedGainByMotor, propagateAutofill, rederiveAutofillTarget, formatAge, currentDriveAgeS, renderDriveBanner, shortMotorLabel, stageCellEdit, NOTCH_QUICK_ACTIONS, NOTCH_ROW_KINDS, notchMatrix, renderDriveGroups, loadDriveState, refreshDriveState, applyDriveChanges, strokeSuffix, reconstructCommand, loadRerunForm };
