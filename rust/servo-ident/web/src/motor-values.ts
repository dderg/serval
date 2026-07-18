import { html } from "htm/preact/standalone";

// --- per-motor value component ---------------------------------------------
//
// The one widget every per-motor drive parameter renders through. Collapsed
// it shows a single "all motors" value; when the motors disagree the
// collapsed field reads "mixed" and the toggle carries the per-motor spread.
// Expanding shows one field per motor plus the set-all field. Editable mode
// stages raw register values through `onStage("*"| motor, text)`; omitting
// `onStage` renders the same layout read-only.

interface MotorValueEntry {
  motor: string;
  label: string;
  value: number;
  original: number;
}

interface MotorValuesSummary {
  agree: boolean;
  sharedValue: number | null;
  pendingMotors: string[];
  spreadText: string;
}

function valuesAgree(values: (number | string)[]): boolean {
  return values.length > 0 && values.every((v) => v === values[0]);
}

function summarizeMotorValues(entries: MotorValueEntry[]): MotorValuesSummary {
  if (!entries.length) throw new Error("summarizeMotorValues: no motors");
  const agree = valuesAgree(entries.map((e) => e.value));
  return {
    agree,
    sharedValue: agree ? entries[0].value : null,
    pendingMotors: entries.filter((e) => e.value !== e.original).map((e) => e.motor),
    spreadText: entries.map((e) => `${e.label}=${e.value}`).join(" "),
  };
}

type StageFn = (motorSel: string, rawText: string) => void;

interface ValueFieldProps {
  cls: string[];
  title: string;
  value: number | null;
  placeholder: string;
  options: Record<string, string> | null | undefined;
  onCommit: ((rawText: string) => void) | null;
}

function ValueField({ cls, title, value, placeholder, options, onCommit }: ValueFieldProps) {
  if (!onCommit) {
    return html`<span class=${[...cls, "readonly"].join(" ")} title=${title}>
      ${value === null ? placeholder : options ? (options[String(value)] ?? value) : value}
    </span>`;
  }
  const onChange = (e: Event) => onCommit((e.target as HTMLInputElement).value);
  if (options) {
    return html`<select class=${cls.join(" ")} title=${title} value=${value === null ? "" : String(value)} onChange=${onChange}>
      <option value="" disabled>${placeholder}</option>
      ${Object.entries(options).map(([v, label]) => html`<option key=${v} value=${v}>${v}: ${label}</option>`)}
    </select>`;
  }
  return html`<input
    type="number"
    step="1"
    class=${cls.join(" ")}
    value=${value === null ? "" : value}
    placeholder=${placeholder}
    title=${title}
    onChange=${onChange}
  />`;
}

interface MotorValuesProps {
  entries: MotorValueEntry[];
  options?: Record<string, string> | null;
  expanded: boolean;
  onToggleExpanded: (expanded: boolean) => void;
  onStage?: StageFn | null;
}

function ExpandedMotorFields({ entries, options, onStage }: { entries: MotorValueEntry[]; options: MotorValuesProps["options"]; onStage: StageFn | null }) {
  const summary = summarizeMotorValues(entries);
  return entries.map((e) => {
    const cls = ["cell-input"];
    if (e.value !== e.original) cls.push("pending");
    if (!summary.agree) cls.push("drift");
    const title = `${e.motor} — raw ${e.value}${e.value !== e.original ? ` (drive has ${e.original})` : ""}`;
    return html`<span key=${e.motor} class="motor-field">
      <label>${e.label}</label>
      <${ValueField}
        cls=${cls}
        title=${title}
        value=${e.value}
        placeholder=""
        options=${options}
        onCommit=${onStage ? (text: string) => onStage(e.motor, text) : null}
      />
    </span>`;
  });
}

function MotorValues({ entries, options, expanded, onToggleExpanded, onStage }: MotorValuesProps) {
  const summary = summarizeMotorValues(entries);
  const toggle = (e: MouseEvent) => {
    e.preventDefault();
    onToggleExpanded(!expanded);
  };
  const toggleCls = ["mv-toggle"];
  if (!summary.agree) toggleCls.push("mixed");
  const toggleTitle = expanded
    ? "collapse to one value for all motors"
    : summary.agree
      ? "show per-motor values"
      : `motors disagree — ${summary.spreadText}; click to edit per motor`;
  const toggleText = expanded ? "×" : summary.agree ? "⋯" : "≠";
  const toggleBtn = html`<button class=${toggleCls.join(" ")} title=${toggleTitle} onClick=${toggle}>
    ${toggleText}
  </button>`;

  if (expanded) {
    const allCls = ["cell-input", "all"];
    if (summary.pendingMotors.length) allCls.push("pending");
    return html`<span class="motor-values expanded">
      <${ExpandedMotorFields} entries=${entries} options=${options} onStage=${onStage ?? null} />
      ${onStage
        ? html`<span class="motor-field all-field">
            <label>all</label>
            <${ValueField}
              cls=${allCls}
              title="set all motors"
              value=${summary.sharedValue}
              placeholder="mixed"
              options=${options}
              onCommit=${(text: string) => onStage("*", text)}
            />
          </span>`
        : null}
      ${toggleBtn}
    </span>`;
  }

  const cls = ["cell-input", "all"];
  if (summary.pendingMotors.length) cls.push("pending");
  const title = summary.agree
    ? "all motors"
    : `set all motors — currently ${summary.spreadText}`;
  return html`<span class="motor-values collapsed">
    <${ValueField}
      cls=${cls}
      title=${title}
      value=${summary.sharedValue}
      placeholder="mixed"
      options=${options}
      onCommit=${onStage ? (text: string) => onStage("*", text) : null}
    />
    ${toggleBtn}
  </span>`;
}

export type { MotorValueEntry, MotorValuesSummary, MotorValuesProps };
export { valuesAgree, summarizeMotorValues, MotorValues };
