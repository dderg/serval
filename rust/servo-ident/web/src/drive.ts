import { html, render } from "htm/preact/standalone";
import { api, el, mustEl } from "./api";
import { MotorValues, valuesAgree } from "./motor-values";
import type { MotorValueEntry } from "./motor-values";
import { setConsoleValue } from "./console";
import { runGcode } from "./moonraker";
import { refresh } from "./runs";
import { currentPageDef } from "./shell";
import { state } from "./state";
import { notify, useStore } from "./store";
import type { PendingEdits } from "./state";
import type { DriveParam, DriveState, Manifest, StrokePlan } from "./wire";
import type { VNode } from "preact";

// --- drive tuning grid --------------------------------------------------------
//
// Renders purely from GET /api/drive_state (servo_tuning.PANEL_PARAMS shape,
// docs/rewrite/servo-tuning-profiles.md) as compact labeled fields, a
// fixed-column grid per group, whose value widget is
// the shared MotorValues component (motor-values.ts): one collapsed set-all
// field when motors agree, an expandable per-motor spread when they don't,
// so a 4-motor bench never needs the same value typed four times and every
// edit's motor scope is visible. Each page
// shows only its own param groups. Every cell shows and takes the RAW
// register value exactly as stored on the drive — the unit label names the
// LSB (e.g. "0.1 Hz") instead of the UI converting, so what you type is
// what SERVO_TUNE writes. Pure helpers first —
// changed-cell diffing, SERVO_TUNE line building (always with an explicit
// MOTORS= list) — the logic a Rust test asserts is present and exercisable
// without a browser; the preact components that render the grid follow.
// Preact owns #drive-panel: renderDriveGroups() mounts once per page build,
// then every state mutation just notifies the store.

const GROUP_ORDER = ["gains", "filters", "notch", "speed_observer", "disturbance_observer"];
const OTHER_GROUP = "other";
const RETIRED_PARAMS = new Set(["gain_mode", "stiffness_level", "adaptive_notch_mode", "inertia_ratio"]);
const DRIVE_REFRESH_POLL_MS = 1000;
const DRIVE_REFRESH_TIMEOUT_MS = 15000;

function paramGroupSection(param: DriveParam): string {
  return GROUP_ORDER.includes(param.group) ? param.group : OTHER_GROUP;
}

function groupParams(params: DriveParam[]): Map<string, DriveParam[]> {
  const sections = new Map<string, DriveParam[]>([...GROUP_ORDER, OTHER_GROUP].map((g) => [g, []]));
  for (const p of params) {
    if (RETIRED_PARAMS.has(p.name)) continue;
    sections.get(paramGroupSection(p))!.push(p);
  }
  return sections;
}

function motorNames(motors: DriveState["motors"]): string[] {
  return Object.keys(motors).sort();
}

function motorRawValues(motors: DriveState["motors"], cCode: string): number[] {
  return motorNames(motors).map((m) => motors[m][cCode]);
}

function pinnedEntries(configPins: DriveState["config_pins"], cCode: string): Record<string, number | string> {
  const out: Record<string, number | string> = {};
  for (const motor of Object.keys(configPins || {}).sort()) {
    const pins = (configPins && configPins[motor]) || {};
    if (Object.prototype.hasOwnProperty.call(pins, cCode)) out[motor] = pins[cCode];
  }
  return out;
}

/// Effective raw value of one grid cell: the session's pending edit if any,
/// else the drive's reading from the last dump.
function driveData(): DriveState {
  const data = state.drive.data;
  if (!data) throw new Error("drive state not loaded");
  return data;
}

function cellRaw(param: DriveParam, motor: string): number {
  const pend = state.drive.pending[param.name];
  if (pend && pend[motor] !== undefined) return pend[motor];
  return driveData().motors[motor][param.c_code];
}

/// Which cells differ from the drive_state's per-motor readings, given this
/// session's pending edits. `pending[name]` is always a `{motor: raw}` map —
/// the "all" column just writes every motor at once.
interface ChangedCell {
  motor: string;
  value: number;
}

interface ChangedParam {
  name: string;
  cells: ChangedCell[];
}

function diffChangedParams(params: DriveParam[], motors: DriveState["motors"], pending: PendingEdits): ChangedParam[] {
  const changed: ChangedParam[] = [];
  for (const p of params) {
    const pend = pending[p.name];
    if (pend === undefined) continue;
    const cells: ChangedCell[] = [];
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
function buildServoTuneCommands(changed: ChangedParam[]): string[] {
  const lines: string[] = [];
  for (const c of changed) {
    const byValue = new Map<number, string[]>();
    for (const { motor, value } of c.cells) {
      if (!byValue.has(value)) byValue.set(value, []);
      byValue.get(value)!.push(motor);
    }
    for (const [value, motorList] of byValue) {
      lines.push(`SERVO_TUNE PARAM=${c.name} VALUE=${value} MOTORS=${motorList.join(",")}`);
    }
  }
  return lines;
}

function paramByName(name: string): DriveParam {
  const param = driveData().params.find((p) => p.name === name);
  if (!param) throw new Error(`${name}: unknown drive param`);
  return param;
}

function formatAge(ageS: number): string {
  if (ageS < 60) return `${ageS.toFixed(0)}s`;
  const m = Math.floor(ageS / 60);
  const s = Math.round(ageS % 60);
  return `${m}m${s}s`;
}

function currentDriveAgeS(): number | null {
  if (!state.drive.data || state.drive.fetchedAtMs === null) return null;
  return state.drive.data.age_s + (Date.now() - state.drive.fetchedAtMs) / 1000;
}

/// The refresh button must render even with no drive state at all —
/// SERVO_DUMP_TUNING is what creates drive_state.json in the first place,
/// so hiding the button behind loaded data would deadlock a fresh bench.
/// Rebuilt only once so the 1 s age ticker doesn't wipe the refresh
/// status text mid-dump.
function renderDriveBanner() {
  const banner = mustEl("drive-state-banner");
  if (!el("drive-refresh-btn")) {
    banner.innerHTML =
      `<span class="note" id="drive-age"></span> ` +
      `<button id="drive-refresh-btn" title="SERVO_DUMP_TUNING and re-read">refresh</button>` +
      `<span id="drive-refresh-status" class="note"></span>`;
    mustEl("drive-refresh-btn").addEventListener("click", refreshDriveState);
  }
  const ageS = currentDriveAgeS();
  mustEl("drive-age").textContent = ageS !== null
    ? `drive state ${formatAge(ageS)} old`
    : "no drive state yet — press refresh to read the drives";
}

function shortMotorLabel(motor: string): string {
  return motor.replace(/^motor_/, "");
}

function stageCellEdit(param: DriveParam, motorSel: string, rawText: string) {
  const raw = parseInt(rawText, 10);
  if (Number.isNaN(raw)) return;
  const targets = motorSel === "*" ? motorNames(driveData().motors) : [motorSel];
  const existing = { ...(state.drive.pending[param.name] || {}) };
  for (const m of targets) existing[m] = raw;
  state.drive.pending[param.name] = existing;
  renderDriveGroups();
}

function paramMotorEntries(param: DriveParam): MotorValueEntry[] {
  return motorNames(driveData().motors).map((motor) => ({
    motor,
    label: shortMotorLabel(motor),
    value: cellRaw(param, motor),
    original: driveData().motors[motor][param.c_code],
  }));
}

function ParamMotorValues({ param }: { param: DriveParam }) {
  return html`<${MotorValues}
    entries=${paramMotorEntries(param)}
    options=${param.options}
    expanded=${state.drive.expandedParams.has(param.name)}
    onToggleExpanded=${(open: boolean) => {
      if (open) state.drive.expandedParams.add(param.name);
      else state.drive.expandedParams.delete(param.name);
      renderDriveGroups();
    }}
    onStage=${(motorSel: string, text: string) => stageCellEdit(param, motorSel, text)}
  />`;
}

function displayParamName(param: DriveParam, section: string): string {
  if (section === OTHER_GROUP) return param.name;
  const groupTokens = new Set(section.split("_").flatMap((t) => [t, t.replace(/s$/, "")]));
  const kept = param.name.split("_").filter((t) => !groupTokens.has(t));
  return kept.length ? kept.join("_") : param.name;
}

function ParamLabel({ param, section }: { param: DriveParam; section: string }) {
  const pins = pinnedEntries(driveData().config_pins, param.c_code);
  const pinnedNames = Object.keys(pins);
  return html`<span class="param-name" title=${`${param.name} — ${param.description} (${param.c_code})`}
      >${displayParamName(param, section)}</span
    >
    ${param.unit ? html`<span class="unit">${param.unit}</span>` : null}
    ${pinnedNames.length
      ? html`<span
          class="badge pin"
          title=${`pinned in config — a restart re-applies ${[...new Set(pinnedNames.map((m) => pins[m]))].join("/")}`}
        >pin</span>`
      : null}
    ${section === OTHER_GROUP ? html`<span class="hint">(${param.group})</span>` : null}`;
}

const NOTCH_ROW_KINDS = ["freq", "width", "depth"];

function notchMatrix(params: DriveParam[]): { nums: number[]; byKey: Map<string, DriveParam>; leftover: DriveParam[] } {
  const byKey = new Map<string, DriveParam>();
  const nums = new Set<number>();
  const leftover: DriveParam[] = [];
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

/// One column per notch, freq/width/depth rows; each cell is the collapsed
/// per-motor component (notches are per-axis physics — on corexy every
/// motor sees the same belt), expandable in place when drives disagree.
function NotchGrid({ nums, byKey }: { nums: number[]; byKey: Map<string, DriveParam> }) {
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
            return html`<td key=${n}>${p ? html`<${ParamMotorValues} param=${p} />` : null}</td>`;
          })}
        </tr>`;
      })}
    </tbody>
  </table>`;
}

/// Column count adapts to the group's field count so no row ever ends in a
/// lone orphan: 1–3 params sit N-across, 4 params go 2×2 (4-across when the
/// container is wide, via CSS), larger groups pick the divisor that keeps
/// the last row at least half full.
function paramLineColumns(count: number): number {
  if (count <= 3) return count;
  if (count % 4 === 0) return 4;
  if (count % 3 === 0) return 3;
  return count % 3 === 1 ? 4 : 3;
}

/// One compact labeled field per param on a per-group uniform grid so every
/// field is the same width whether or not its row is full; expanding a
/// field's per-motor spread pops it onto its own full-width row
/// (grid-column span) while the rest of the grid stays compact.
function ParamLine({ params, group }: { params: DriveParam[]; group: string }) {
  return html`<div class=${`param-line cols-${paramLineColumns(params.length)}`}>
    ${params.map(
      (p) => html`<div
        key=${p.name}
        data-param=${p.name}
        class=${state.drive.expandedParams.has(p.name) ? "param-field expanded" : "param-field"}
      >
        <div class="param-field-label"><${ParamLabel} param=${p} section=${group} /></div>
        <${ParamMotorValues} param=${p} />
      </div>`
    )}
  </div>`;
}

function DriveGroups() {
  const def = currentPageDef();
  const sections = groupParams(driveData().params);
  const groups: VNode[] = [];
  for (const [group, params] of sections) {
    if (!params.length) continue;
    if (def.groups && group !== OTHER_GROUP && !def.groups.includes(group)) continue;
    if (group === "notch") {
      const { nums, byKey, leftover } = notchMatrix(params);
      groups.push(html`<div key=${group} class="param-group">
        <h3>notch</h3>
        <${NotchGrid} nums=${nums} byKey=${byKey} />
        ${leftover.length ? html`<${ParamLine} params=${leftover} group=${group} />` : null}
      </div>`);
      continue;
    }
    groups.push(html`<div key=${group} class="param-group">
      <h3>${group.replace(/_/g, " ")}</h3>
      <${ParamLine} params=${params} group=${group} />
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

let mountedDrivePanel: HTMLElement | null = null;

function renderDriveGroups() {
  const container = el("drive-panel");
  if (container && mountedDrivePanel !== container) {
    if (mountedDrivePanel) render(null as unknown as VNode, mountedDrivePanel);
    mountedDrivePanel = container;
    render(html`<${DrivePanel} />`, container);
  }
  notify();
}

async function loadDriveState() {
  try {
    const data: DriveState = await api("/api/drive_state");
    state.drive.data = data;
    state.drive.fetchedAtMs = Date.now();
  } catch (e) {
    state.drive.data = null;
    console.error(e);
  }
  state.drive.pending = {};
  renderDriveBanner();
  renderDriveGroups();
}

async function refreshDriveState() {
  const statusEl = el("drive-refresh-status");
  const priorAge = currentDriveAgeS() ?? Infinity;
  if (statusEl) statusEl.textContent = " dumping…";
  await runGcode(["SERVO_DUMP_TUNING"], "refresh");
  const deadline = Date.now() + DRIVE_REFRESH_TIMEOUT_MS;
  while (Date.now() < deadline) {
    await new Promise((resolve) => setTimeout(resolve, DRIVE_REFRESH_POLL_MS));
    let data: DriveState;
    try {
      data = await api("/api/drive_state");
    } catch (e) {
      continue;
    }
    if (data.age_s < priorAge) {
      state.drive.data = data;
      state.drive.fetchedAtMs = Date.now();
      state.drive.pending = {};
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
  const data = driveData();
  const changed = diffChangedParams(data.params, data.motors, state.drive.pending);
  const lines = buildServoTuneCommands(changed);
  if (!lines.length) return;
  await runGcode(lines, "apply");
  await loadDriveState();
}

// --- sweep re-run ---------------------------------------------------------

/// The stroke's SPEED/ACCEL must ride along on a re-run: they shape the
/// excitation, so a "same sweep" at the command defaults is not the same
/// sweep and its results are not comparable to the original.
function strokeSuffix(manifest: Manifest, includeAccel: boolean): string {
  const plan = manifest.stroke_plan || {};
  let suffix = "";
  if (plan.speed != null) suffix += ` SPEED=${plan.speed}`;
  if (includeAccel && plan.accel != null) suffix += ` ACCEL=${plan.accel}`;
  return suffix;
}

function requiredStrokePlan(manifest: Manifest): StrokePlan {
  if (!manifest.stroke_plan) {
    throw new Error(`${manifest.experiment}: manifest has no stroke_plan to rebuild the command from`);
  }
  return manifest.stroke_plan;
}

/// Old manifests predate the recorded `command` field; rebuilding from the
/// manifest is a lossy fallback that only knows the parameters listed here.
function reconstructCommand(manifest: Manifest): string {
  if (manifest.command) return manifest.command;
  const tag = manifest.tag || "cal";
  const axis = manifest.axis || "X";
  const iterations = (manifest.stroke_plan && manifest.stroke_plan.iterations) || 1;
  const common = `AXIS=${axis} ITERATIONS=${iterations} TAG=${tag}`;

  switch (manifest.experiment) {
    case "gain_sweep": {
      const values = manifest.steps.map((s) => (s.swept || {}).speed).join(",");
      return `SERVO_CALIBRATE_GAINS SPEED_GAINS=${values} ${common}${strokeSuffix(manifest, true)}`;
    }
    case "inertia_sweep": {
      const values = manifest.steps.map((s) => (s.swept || {}).ratio ?? Object.values(s.swept || {})[0]).join(",");
      return `SERVO_SWEEP_INERTIA RATIOS=${values} ${common}${strokeSuffix(manifest, true)}`;
    }
    case "accel_sweep": {
      const values = manifest.steps.map((s) => (s.swept || {}).accel ?? Object.values(s.swept || {})[0]).join(",");
      return `SERVO_SWEEP_ACCEL ACCELS=${values} ${common}${strokeSuffix(manifest, false)}`;
    }
    case "strain_map": {
      const plan = requiredStrokePlan(manifest);
      return (
        `SERVO_MEASURE_STRAIN_MAP LINE_SPACING=${plan.line_spacing} SPEED=${plan.speed} ` +
        `ACCEL=${plan.accel} X_START=${plan.x_start} X_END=${plan.x_end} ` +
        `Y_START=${plan.y_start} Y_END=${plan.y_end} DWELL_MS=${plan.dwell_ms} ` +
        `${plan.zero_sync ? "" : "SYNC=0 "}TAG=${tag}`
      );
    }
    case "differential": {
      const plan = requiredStrokePlan(manifest);
      return (
        `SERVO_MEASURE_DIFFERENTIAL BELT=${plan.belt} FREQ_START=${plan.freq_start} ` +
        `FREQ_END=${plan.freq_end} AMPLITUDE=${plan.amplitude} DURATION=${plan.duration} ` +
        `RAMP=${plan.ramp} DWELL_MS=${plan.dwell_ms} NAME=${tag}`
      );
    }
    case "ringdown": {
      const plan = requiredStrokePlan(manifest);
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

function loadRerunForm(name: string) {
  const detail = state.details.get(name);
  if (!detail || !detail.manifest) return;
  const label = el("form-run-name");
  if (label) label.textContent = `from ${name}`;
  setConsoleValue(reconstructCommand(detail.manifest), false);
}

export { GROUP_ORDER, OTHER_GROUP, RETIRED_PARAMS, DRIVE_REFRESH_POLL_MS, DRIVE_REFRESH_TIMEOUT_MS, paramGroupSection, groupParams, motorNames, motorRawValues, valuesAgree, pinnedEntries, cellRaw, diffChangedParams, buildServoTuneCommands, paramByName, formatAge, currentDriveAgeS, renderDriveBanner, shortMotorLabel, stageCellEdit, NOTCH_ROW_KINDS, notchMatrix, renderDriveGroups, loadDriveState, refreshDriveState, applyDriveChanges, displayParamName, strokeSuffix, reconstructCommand, loadRerunForm };
