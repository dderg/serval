import { api } from "./api";
import { loadConsoleHistory } from "./console";

const REFRESH_MS = 5000;
const MOONRAKER_KEY = "servoCalMoonrakerUrl";
const CONSOLE_HISTORY_KEY = "servoCalConsoleHistory";
const HELP_CACHE_KEY = "servoCalGcodeHelp";
const CONSOLE_HISTORY_MAX = 500;
const PALETTE = ["#4fb3ff", "#e05a4f", "#4caf50", "#d9a441", "#b388ff", "#4fd8c4"];
const RESONANCE_BAND_HZ = [20, 450];
const RINGDOWN_PSD_PLOT_MAX_HZ = 500;
const PSD_MAX_FREQ_KEY = "servoCalPsdMaxFreqHz";
const MOTOR_VIEW_KEY = "servoCalMotorView";
const PSD_MAX_FREQ_CHOICES_HZ = [250, 500, 750, 1000, 1500];
const PSD_MAX_FREQ_DEFAULT_HZ = 750;
const INITIAL_SELECTED_RUNS = 1;
const PEAK_MIN_SEPARATION_HZ = 15;
const PEAK_LIST_SIZE = 3;

// Each page serves one calibration activity with only the tools that
// activity needs (docs/plans/servo-calibration-automation.md, second demo
// review): the interleaved tuning loop is navigation between pages, not
// scrolling within one.
const PAGE_DEFS = {
  gains: {
    // gains and notches are one tuning loop, not two — the resonances the
    // PSD shows are what keep gains from going higher, so the gains and notch
    // grids, the peak list, and the metrics-vs-gain chart share one page.
    label: "gains",
    groups: ["gains", "notch"],
    experiments: ["gain_sweep", "refine_sweep", "gain_ladder", "tracking"],
    charts: ["psd"],
    intro:
      "find the highest speed gain without resonance or torque rail, then " +
      "notch out whatever resonance the PSD shows so gains can go higher",
    metrics: true,
    sweepChart: true,
    peaks: true,
    templates: [
      {
        label: "ladder…",
        command: "SERVO_GAIN_LADDER SAFE=550 START=700 STEP=50 MAX=900 AXIS=X ITERATIONS=1",
        title: "climb from START by STEP until a rung flags, then revert to SAFE",
      },
      {
        label: "tracking…",
        command: "SERVO_MEASURE_TRACKING AXIS=X SPEED=100 ACCEL=3000 ITERATIONS=3",
        title:
          "single stroke run with capture — the before/after check for any tuning " +
          "change; per-drive overshoot/settle land in the tracking metrics table",
      },
    ],
  },
  observers: {
    label: "observers",
    groups: ["filters", "speed_observer", "disturbance_observer"],
    experiments: null,
    charts: ["time"],
    intro: "disturbance rejection and filtering — judge in the time domain",
  },
  dynamics: {
    label: "dynamics",
    groups: ["load"],
    experiments: ["tracking", "inertia_grid", "differential", "ringdown"],
    charts: ["frf", "ringdown"],
    metrics: true,
    intro: "identify the load, then let feedforward carry it",
    templates: [
      {
        label: "fit…",
        command: "SERVO_FIT_DYNAMICS",
        title:
          "strokes the axis, fits inertia/friction per drive, prints the recommended " +
          "inertia ratio and writes the feedforward profile",
      },
      {
        label: "tracking…",
        command: "SERVO_MEASURE_TRACKING AXIS=X SPEED=100 ACCEL=3000 ITERATIONS=3",
        title:
          "single stroke run with capture — the before/after check for any tuning " +
          "change; per-drive overshoot/settle land in the tracking metrics table",
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
const DEFAULT_PAGE = "gains";
const LIVE_STATUS_POLL_MS = 1000;
const LIVE_TAIL_POLL_MS = 400;
const MOONRAKER_HEALTH_POLL_MS = 5000;

const state: any = {
  page: DEFAULT_PAGE,
  runs: [],
  details: new Map(), // name -> {mtime_utc, has_results, manifest, results}
  plotSeries: new Map(), // name -> {mtime_utc, data}
  selected: new Set(),
  pinned: new Set(), // runs that stay selected when a plain click switches runs
  runColors: new Map(), // run name -> palette color, kept while the run stays selected
  autoSelected: false,
  stepFilter: null, // null = every step; otherwise a Set of visible step names
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
    dirty: new Set(), // autofill-target param names the user has edited directly this session
    notchPerMotor: false, // compact one-value-per-notch grid unless toggled
    adaptiveOpen: false, // the adaptive-recipes fold survives re-renders
  },
  live: {
    cursor: null, // last next_cycle from /api/live_tap; null = attach now
    fsHz: null,
    cycle0: null, // first streamed cycle_index — the chart's t=0
    lastCycle: null, // cycle_index of the last kept sample, for gap breaks
    t: [], // seconds since stream start, one per kept point
    perDrive: {}, // tap drive name -> {ferr, torque} arrays (null = gap break)
    windowS: 10, // seconds kept and drawn, set by the slider
    timers: [], // interval ids cleared on page switch
    polling: false,
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

export { REFRESH_MS, MOONRAKER_KEY, CONSOLE_HISTORY_KEY, HELP_CACHE_KEY, CONSOLE_HISTORY_MAX, PALETTE, RESONANCE_BAND_HZ, RINGDOWN_PSD_PLOT_MAX_HZ, PSD_MAX_FREQ_KEY, MOTOR_VIEW_KEY, PSD_MAX_FREQ_CHOICES_HZ, PSD_MAX_FREQ_DEFAULT_HZ, INITIAL_SELECTED_RUNS, PEAK_MIN_SEPARATION_HZ, PEAK_LIST_SIZE, PAGE_DEFS, DEFAULT_PAGE, LIVE_STATUS_POLL_MS, LIVE_TAIL_POLL_MS, MOONRAKER_HEALTH_POLL_MS, state };
