import type {
  DriveState,
  PlotSeries,
  RunDetail,
  RunPath,
  RunSummary,
  StrainData,
  StrainField,
} from "./wire";

const REFRESH_MS = 5000;
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

interface PathFullEntry {
  mtime_utc: string | null;
  data: RunPath | null;
  error: string | null;
}

type PendingEdits = Record<string, Record<string, number>>;

interface DrivePanelState {
  data: DriveState | null;
  fetchedAtMs: number | null;
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
  cache: Map<string, { mtime_utc: string | null; data: StrainData }>;
  field: StrainField;
}

interface SentEntry {
  time: string;
  label: string;
  lines: string[];
  results: { ok: boolean; status: number }[];
  responses?: string[][];
}

interface HelpState {
  commands: Record<string, string> | null;
  fetchedUtc: string | null;
  cached: boolean;
  error: string | null;
  pending: boolean;
  klippyState: string | null;
}

interface AppState {
  page: string;
  runs: RunSummary[];
  details: Map<string, RunDetail>;
  plotSeries: Map<string, { mtime_utc: string | null; data: PlotSeries }>;
  pathFull: Map<string, PathFullEntry>;
  selected: Set<string>;
  pinned: Set<string>;
  runColors: Map<string, string>;
  autoSelected: boolean;
  stepFilter: Set<string> | null;
  motorFilter: Set<string> | null;
  console: ConsoleState;
  drive: DrivePanelState;
  live: LiveState;
  strain: StrainPageState;
  sentLog: SentEntry[];
  help: HelpState;
}

const state: AppState = {
  page: DEFAULT_PAGE,
  runs: [],
  details: new Map(), // name -> {mtime_utc, has_results, manifest, results}
  plotSeries: new Map(), // name -> {mtime_utc, data}
  pathFull: new Map(), // name -> {mtime_utc, data, error} from /api/runs/<name>/path
  selected: new Set(),
  pinned: new Set(), // runs that stay selected when a plain click switches runs
  runColors: new Map(), // run name -> palette color, kept while the run stays selected
  autoSelected: false,
  stepFilter: null, // null = every step; otherwise a Set of visible step names
  motorFilter: null, // null = every motor; only consulted in per-motor view
  console: {
    text: "", // current input line, survives page switches
    history: loadConsoleHistory(),
    cursor: null, // history index while navigating; null = editing a fresh line
    draft: "", // the fresh line stashed when history navigation starts
    search: null, // {query, pos, saved, failed} while ctrl+r reverse search is live
  },
  drive: {
    data: null, // last /api/drive_state response (params, motors, config_pins, age_s)
    fetchedAtMs: null, // Date.now() when data was fetched, for a client-ticking age display
    pending: {}, // param name -> {motor: raw} — edits not yet applied
    expandedParams: new Set(), // param names whose MotorValues cell shows per-motor fields
  },
  live: {
    cursor: null, // last next_cycle from /api/live_tap; null = attach now
    fsHz: null,
    cycle0: null, // first streamed cycle_index — the chart's t=0
    lastCycle: null, // cycle_index of the last kept sample, for gap breaks
    t: [], // seconds since stream start, one per kept point
    perDrive: {}, // tap drive name -> {ferr, torque, target, pos} arrays (null = gap break)
    countsPerMm: {}, // tap drive name -> counts_per_mm from the tap header
    windowS: 10, // seconds kept and drawn, set by the slider
    timers: [], // interval ids cleared on page switch
    polling: false,
    frozen: false,
    freezeStartT: null, // window start at freeze time; trim never drops past it
    freezeEndT: null, // last sample time at freeze; frozen draws stop here
    freezeTruncated: false, // set when the frozen buffer hit its cap and lost samples
  },
  strain: {
    selected: null, // run name shown on the strain page; auto-picks the newest
    compare: new Set(), // extra run names diffed against `selected` when dimensions match
    cache: new Map(), // name -> {mtime_utc, data} from /api/runs/<name>/strain
    field: "elastic", // which half to chart: elastic (fwd+back)/2 or friction (fwd-back)/2
  },
  sentLog: [], // {time, label, lines, results} — every G-code batch sent this session
  help: {
    commands: null, // SERVO_* name -> cmd_*_help string, straight from klippy
    fetchedUtc: null,
    cached: false, // true when `commands` came from localStorage, not a live fetch
    error: null,
    pending: false,
    klippyState: null, // last /server/info klippy_state, to refetch after a RESTART
  },
};

export type { PageDef, PageTemplate, ConsoleSearch, SentEntry, LiveSeries, PendingEdits, PathFullEntry };
export { REFRESH_MS, MOONRAKER_KEY, CONSOLE_HISTORY_KEY, HELP_CACHE_KEY, CONSOLE_HISTORY_MAX, PALETTE, RESONANCE_BAND_HZ, RINGDOWN_PSD_PLOT_MAX_HZ, PSD_MAX_FREQ_KEY, MOTOR_VIEW_KEY, LIVE_UNIT_KEY, PSD_MAX_FREQ_CHOICES_HZ, PSD_MAX_FREQ_DEFAULT_HZ, INITIAL_SELECTED_RUNS, PAGE_DEFS, DEFAULT_PAGE, LIVE_STATUS_POLL_MS, LIVE_TAIL_POLL_MS, MOONRAKER_HEALTH_POLL_MS, RT_HEALTH_POLL_MS, loadConsoleHistory, liveDrawCount, state };
