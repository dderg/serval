// Interim hand-written shapes of the server payloads. TODO(bindings): the
// next phase replaces this whole file with TypeScript generated from the
// Rust wire structs (results.rs / serve.rs schemars schemas).

interface RunSummary {
  name: string;
  mtime_utc: string;
  has_results: boolean;
  experiment: string;
  tag: string;
  axis: string | null;
  note: string | null;
}

interface ManifestMotor {
  name: string;
  counts_per_mm: number | null;
}

interface StrokePlan {
  speed?: number | null;
  accel?: number | null;
  iterations?: number | null;
  line_spacing?: number | null;
  x_start?: number | null;
  x_end?: number | null;
  y_start?: number | null;
  y_end?: number | null;
  dwell_ms?: number | null;
  zero_sync?: boolean | null;
  belt?: string | null;
  freq_start?: number | null;
  freq_end?: number | null;
  amplitude?: number | null;
  duration?: number | null;
  ramp?: number | null;
  cruise_ms?: number | null;
  speeds?: number[] | null;
}

interface ManifestStep {
  name: string;
  swept: Record<string, number> | null;
}

type NotchStateValue = Record<string, number | string> | number | string;

interface ManifestAmbient {
  journal_params?: Record<string, Record<string, number | string>> | null;
  notches?: Record<string, Record<string, NotchStateValue>> | null;
}

interface Manifest {
  experiment: string;
  command?: string | null;
  tag?: string | null;
  axis?: string | null;
  stroke_plan?: StrokePlan | null;
  steps: ManifestStep[];
  motors?: ManifestMotor[] | null;
  ambient?: ManifestAmbient | null;
}

interface MoveMetrics {
  ferr_peak: number;
  ferr_rms: number;
  overshoot: number;
  settle_ms: number | null;
  settle_window_truncated: boolean;
}

interface TorqueMetrics {
  peak_pct_rated: number;
  rail_detected: boolean;
  rail_pct_moving: number;
  rail_ms: number;
  longest_burst_ms: number;
}

interface DriveMetrics {
  moves: MoveMetrics[];
  torque: TorqueMetrics;
}

interface ResultDrive {
  metrics: DriveMetrics;
}

interface DifferentialResult {
  pair: string[];
  segments: number;
}

interface ResultStep {
  name: string;
  flags: string[];
  drives: Record<string, ResultDrive>;
  differential?: DifferentialResult | null;
}

interface Results {
  verdict: { recommended_step: string | null };
  steps: ResultStep[];
}

interface RunDetail {
  mtime_utc: string;
  has_results: boolean;
  manifest: Manifest | null;
  results: Results | null;
}

interface PsdData {
  freq_hz: number[];
  per_drive: Record<string, number[]>;
  accel?: { freq_hz: number[]; psd: number[] } | null;
}

interface FrfMode {
  freq_hz: number;
  gain_db: number;
  damping: number | null;
  coherence: number;
}

interface DifferentialPlot {
  freq_hz: number[];
  mag_db: number[];
  phase_deg: number[];
  coherence: number[];
  torque_db: number[];
  modes: FrfMode[];
  coherence_min: number;
}

interface RingdownMode {
  freq_hz: number;
  zeta: number;
  zeta_lo: number;
  zeta_hi: number;
  disp_um: number;
  tails: number;
  r2: number;
  amp: number;
  fit_start_ms: number;
}

interface RingdownSource {
  source: string;
  unit: string;
  fs_hz: number;
  tails: { value: number[] }[];
  modes: RingdownMode[];
  psd_freq_hz: number[];
  psd: number[];
}

interface RingdownPlot {
  sources: RingdownSource[];
}

interface PlotStep {
  name: string;
  t_s: number[];
  drives: Record<string, { ferr_counts: number[] }>;
  combined?: { on_ferr_mm: number[] } | null;
  psd?: PsdData | null;
  differential?: DifferentialPlot | null;
  ringdown?: RingdownPlot | null;
}

interface PlotSeries {
  steps: PlotStep[];
}

interface DriveParam {
  name: string;
  c_code: string;
  group: string;
  description: string;
  unit?: string | null;
  autofill?: string | null;
  options?: Record<string, string> | null;
}

interface DriveState {
  age_s: number;
  params: DriveParam[];
  motors: Record<string, Record<string, number>>;
  config_pins: Record<string, Record<string, number | string>> | null;
  slots?: Record<string, number> | null;
}

interface LiveCapture {
  name: string;
  age_s: number | null;
  size_bytes: number;
}

interface LiveStatus {
  capture: LiveCapture | null;
}

interface LiveTapPayload {
  status: string;
  reason?: string | null;
  fs_hz: number;
  first_cycle: number;
  next_cycle: number;
  stride: number;
  drives?: Record<string, { ferr: number[]; torque: number[] }> | null;
}

type StrainField = "elastic" | "friction";

interface StrainBelt {
  pair: string;
  elastic: (number | null)[];
  friction: (number | null)[];
}

interface StrainLine {
  name: string;
  swept: Record<string, number> | null;
  bin_centers: number[];
  belts: StrainBelt[];
}

interface StrainData {
  lines: StrainLine[];
}

interface NotePayload {
  note: string | null;
}

export type {
  RunSummary,
  ManifestMotor,
  StrokePlan,
  ManifestStep,
  NotchStateValue,
  ManifestAmbient,
  Manifest,
  MoveMetrics,
  TorqueMetrics,
  DriveMetrics,
  ResultDrive,
  DifferentialResult,
  ResultStep,
  Results,
  RunDetail,
  PsdData,
  FrfMode,
  DifferentialPlot,
  RingdownMode,
  RingdownSource,
  RingdownPlot,
  PlotStep,
  PlotSeries,
  DriveParam,
  DriveState,
  LiveCapture,
  LiveStatus,
  LiveTapPayload,
  StrainField,
  StrainBelt,
  StrainLine,
  StrainData,
  NotePayload,
};
