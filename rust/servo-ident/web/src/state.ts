import type { StrainField } from "./wire";

const MOONRAKER_KEY = "servoCalMoonrakerUrl";
const CONSOLE_HISTORY_KEY = "servoCalConsoleHistory";
const HELP_CACHE_KEY = "servoCalGcodeHelp";
const CONSOLE_HISTORY_MAX = 500;
const PALETTE = ["#4fb3ff", "#e05a4f", "#4caf50", "#d9a441", "#b388ff", "#4fd8c4"];
const RESONANCE_BAND_HZ: [number, number] = [20, 450];
const RINGDOWN_PSD_PLOT_MAX_HZ = 500;
const PSD_MAX_FREQ_KEY = "servoCalPsdMaxFreqHz";
const MOTOR_VIEW_KEY = "servoCalMotorView";
const LIVE_UNIT_KEY = "servoCalLiveUnit";
const PSD_MAX_FREQ_CHOICES_HZ = [250, 500, 750, 1000, 1500];
const PSD_MAX_FREQ_DEFAULT_HZ = 750;
const INITIAL_SELECTED_RUNS = 1;

/// Lives here (not console.ts) because `state` runs it at module init —
/// pulling it from console.ts would make the state → console import cycle
/// hit console's TDZ under plain ESM evaluation.
/// The catch only forgives corrupt localStorage JSON — anything else (a
/// mistyped key, a TDZ const) must surface, not quietly reset the history.
function loadConsoleHistory() {
  const raw = localStorage.getItem(CONSOLE_HISTORY_KEY) || "[]";
  let parsed;
  try {
    parsed = JSON.parse(raw);
  } catch (e) {
    return [];
  }
  return Array.isArray(parsed) ? parsed.filter((l): l is string => typeof l === "string") : [];
}

interface PageTemplate {
  label: string;
  command: string;
  title: string;
}

interface PageDef {
  label: string;
  groups?: string[];
  experiments?: string[] | null;
  charts?: string[];
  intro?: string;
  metrics?: boolean;
  sweepChart?: boolean;
  templates?: PageTemplate[];
  strain?: boolean;
  live?: boolean;
  journal?: boolean;
  docs?: boolean;
}

const PAGE_DEFS: Record<string, PageDef> = {
  tune: {
    label: "tune",
    groups: ["gains", "notch", "filters", "speed_observer", "disturbance_observer", "load"],
    experiments: [
      "gain_sweep",
      "tracking",
      "inertia_grid",
      "differential",
      "ringdown",
    ],
    charts: ["psd", "time", "path", "frf", "ringdown"],
    intro:
      "one tuning loop: raise gains until resonance or torque rail, notch what " +
      "the PSD shows, identify the load, and judge disturbance rejection in the " +
      "time domain — fold the sections you don't need",
    metrics: true,
    sweepChart: true,
    templates: [
      {
        label: "tracking…",
        command: "SERVO_MEASURE_TRACKING AXIS=X SPEED=100 ACCEL=3000 ITERATIONS=3",
        title:
          "single stroke run with capture — the before/after check for any tuning " +
          "change; per-drive overshoot/settle land in the tracking metrics table",
      },
      {
        label: "fit…",
        command: "SERVO_FIT_DYNAMICS",
        title:
          "strokes the axis, fits inertia/friction per drive, prints the recommended " +
          "inertia ratio and writes the feedforward profile",
      },
      {
        label: "ringdown…",
        command: "SERVO_MEASURE_RINGDOWN AXIS=X SPEEDS=100,250,400 ITERATIONS=3",
        title:
          "high-accel strokes into a full stop — fits the post-stop free decay " +
          "for per-mode frequency and damping ratio; the closed-loop transient " +
          "a drive's adaptive filters can't compensate the way they fight a chirp",
      },
    ],
  },
  strain: {
    label: "strain",
    strain: true,
    experiments: ["strain_map"],
    intro: "map differential belt torque across the bed — elastic strain and friction",
    templates: [
      {
        label: "map…",
        command: "SERVO_MEASURE_STRAIN_MAP LINE_SPACING=20 SPEED=50 ACCEL=1000 TAG=strain",
        title:
          "raster the bed with slow strokes — parks at the region center and runs " +
          "SERVO_SYNC first so every map shares a preload zero; omit X/Y_START/END " +
          "to cover the whole probed region",
      },
    ],
  },
  live: {
    label: "live",
    live: true,
    intro: "following error streamed off the growing capture file",
  },
  journal: {
    label: "journal",
    journal: true,
  },
  docs: {
    label: "docs",
    docs: true,
  },
};
const DEFAULT_PAGE = "tune";
const LIVE_STATUS_POLL_MS = 1000;
const LIVE_TAIL_POLL_MS = 400;
const MOONRAKER_HEALTH_POLL_MS = 5000;
const RT_HEALTH_POLL_MS = 2000;

interface ConsoleSearch {
  query: string;
  pos: number;
  saved: string;
  failed: boolean;
}

interface ConsoleState {
  text: string;
  history: string[];
  cursor: number | null;
  draft: string;
  search: ConsoleSearch | null;
}

type PendingEdits = Record<string, Record<string, number>>;

interface DrivePanelState {
  pending: PendingEdits;
  expandedParams: Set<string>;
}

interface LiveSeries {
  ferr: (number | null)[];
  torque: (number | null)[];
  target: (number | null)[];
  pos: (number | null)[];
}

interface LiveState {
  cursor: number | null;
  fsHz: number | null;
  cycle0: number | null;
  lastCycle: number | null;
  t: number[];
  perDrive: Record<string, LiveSeries>;
  countsPerMm: Record<string, number>;
  windowS: number;
  timers: ReturnType<typeof setInterval>[];
  polling: boolean;
  frozen: boolean;
  freezeStartT: number | null;
  freezeEndT: number | null;
  freezeTruncated: boolean;
}

function liveDrawCount(): number {
  const t = state.live.t;
  if (!state.live.frozen || state.live.freezeEndT === null) return t.length;
  let n = t.length;
  while (n > 0 && t[n - 1] > state.live.freezeEndT) n--;
  return n;
}

interface StrainPageState {
  selected: string | null;
  compare: Set<string>;
  field: StrainField;
}

interface SentEntry {
  time: string;
  label: string;
  lines: string[];
  results: { ok: boolean; status: number }[];
  responses?: string[][];
}

interface AppState {
  page: string;
  selected: Set<string>;
  pinned: Set<string>;
  runColors: Map<string, string>;
  autoSelected: boolean;
  stepFilter: Set<string> | null;
  motorFilter: Set<string> | null;
  accelAxisFilter: Set<string> | null;
  console: ConsoleState;
  drive: DrivePanelState;
  live: LiveState;
  strain: StrainPageState;
  sentLog: SentEntry[];
}

const state: AppState = {
  page: DEFAULT_PAGE,
  selected: new Set(),
  pinned: new Set(),
  runColors: new Map(),
  autoSelected: false,
  stepFilter: null,
  motorFilter: null,
  accelAxisFilter: null,
  console: {
    text: "",
    history: loadConsoleHistory(),
    cursor: null,
    draft: "",
    search: null,
  },
  drive: {
    pending: {},
    expandedParams: new Set(),
  },
  live: {
    cursor: null,
    fsHz: null,
    cycle0: null,
    lastCycle: null,
    t: [],
    perDrive: {},
    countsPerMm: {},
    windowS: 10,
    timers: [],
    polling: false,
    frozen: false,
    freezeStartT: null,
    freezeEndT: null,
    freezeTruncated: false,
  },
  strain: {
    selected: null,
    compare: new Set(),
    field: "elastic",
  },
  sentLog: [],
};

export type { PageDef, PageTemplate, ConsoleSearch, SentEntry, LiveSeries, PendingEdits };
export { MOONRAKER_KEY, CONSOLE_HISTORY_KEY, HELP_CACHE_KEY, CONSOLE_HISTORY_MAX, PALETTE, RESONANCE_BAND_HZ, RINGDOWN_PSD_PLOT_MAX_HZ, PSD_MAX_FREQ_KEY, MOTOR_VIEW_KEY, LIVE_UNIT_KEY, PSD_MAX_FREQ_CHOICES_HZ, PSD_MAX_FREQ_DEFAULT_HZ, INITIAL_SELECTED_RUNS, PAGE_DEFS, DEFAULT_PAGE, LIVE_STATUS_POLL_MS, LIVE_TAIL_POLL_MS, MOONRAKER_HEALTH_POLL_MS, RT_HEALTH_POLL_MS, loadConsoleHistory, liveDrawCount, state };
