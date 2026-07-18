// Server payload types. Everything the Rust side derives `JsonSchema` on is
// generated into ./generated/ by ts-rs (freshness-guarded by the
// `ts_bindings` cargo test); the shapes below are only the payloads Rust
// reads loosely (manifest passthrough, live tap, strain) and thin
// server-side compositions like the drive_state `age_s` field.

import type { DriveStatePayload } from "./generated/DriveStatePayload";
import type { Results } from "./generated/Results";

export type { DifferentialMode, DifferentialMode as FrfMode } from "./generated/DifferentialMode";
export type { DifferentialResult } from "./generated/DifferentialResult";
export type { DriveResult, DriveResult as ResultDrive } from "./generated/DriveResult";
export type { DriveStateParam, DriveStateParam as DriveParam } from "./generated/DriveStateParam";
export type { LiveCapture } from "./generated/LiveCapture";
export type { LiveStatus } from "./generated/LiveStatus";
export type { Metrics, Metrics as DriveMetrics } from "./generated/Metrics";
export type { Move, Move as MoveMetrics } from "./generated/Move";
export type { NoteResponse, NoteResponse as NotePayload } from "./generated/NoteResponse";
export type { PlotDifferential, PlotDifferential as DifferentialPlot } from "./generated/PlotDifferential";
export type { PlotPsd, PlotPsd as PsdData } from "./generated/PlotPsd";
export type { PlotRingdown, PlotRingdown as RingdownPlot } from "./generated/PlotRingdown";
export type {
  PlotRingdownSource,
  PlotRingdownSource as RingdownSource,
} from "./generated/PlotRingdownSource";
export type { PlotSeries } from "./generated/PlotSeries";
export type { PlotStep } from "./generated/PlotStep";
export type { Results } from "./generated/Results";
export type { RingdownMode } from "./generated/RingdownMode";
export type { RunSummary } from "./generated/RunSummary";
export type { StepResult, StepResult as ResultStep } from "./generated/StepResult";
export type { TorqueSummary, TorqueSummary as TorqueMetrics } from "./generated/TorqueSummary";
export type { VerdictSummary } from "./generated/VerdictSummary";

// `handle_drive_state` serves `DriveStatePayload` from disk plus a
// server-computed `age_s`.
export type DriveState = DriveStatePayload & { age_s: number };

// --- Payloads Rust does not model with schemars (manifest is a raw file
// passthrough; live tap and strain are hand-built JSON) ---

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

interface RunDetail {
  mtime_utc: string;
  has_results: boolean;
  manifest: Manifest | null;
  results: Results | null;
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

export type {
  ManifestMotor,
  StrokePlan,
  ManifestStep,
  NotchStateValue,
  ManifestAmbient,
  Manifest,
  RunDetail,
  LiveTapPayload,
  StrainField,
  StrainBelt,
  StrainLine,
  StrainData,
};
