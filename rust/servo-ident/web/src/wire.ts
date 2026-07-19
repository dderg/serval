// Server payload types. Everything the Rust side derives `JsonSchema` on is
// generated into ./generated/ by ts-rs (freshness-guarded by the
// `ts_bindings` cargo test); the shapes below are only the payloads Rust
// reads loosely (strain) and thin server-side compositions like the
// drive_state `age_s` field.

import type { DriveStatePayload } from "./generated/DriveStatePayload";
import type { Results } from "./generated/Results";
import type { components, paths } from "./api/openapi.generated";

export type { DifferentialMode, DifferentialMode as FrfMode } from "./generated/DifferentialMode";
export type { DifferentialResult } from "./generated/DifferentialResult";
export type { DriveResult, DriveResult as ResultDrive } from "./generated/DriveResult";
export type { DriveStateParam, DriveStateParam as DriveParam } from "./generated/DriveStateParam";
export type { LiveCapture } from "./generated/LiveCapture";
export type { LiveStatus } from "./generated/LiveStatus";
export type { SpatialFrame } from "./generated/SpatialFrame";
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
export type { PlotPath } from "./generated/PlotPath";
export type { PlotSeries } from "./generated/PlotSeries";
export type { PlotStep } from "./generated/PlotStep";
export type { Results } from "./generated/Results";
export type { RingdownMode } from "./generated/RingdownMode";
export type { RunPath } from "./generated/RunPath";
export type { RunPathStep } from "./generated/RunPathStep";
export type { RunSummary } from "./generated/RunSummary";
export type { StepResult, StepResult as ResultStep } from "./generated/StepResult";
export type { TorqueSummary, TorqueSummary as TorqueMetrics } from "./generated/TorqueSummary";
export type { VerdictSummary } from "./generated/VerdictSummary";
export type { StrainBelt } from "./generated/StrainBelt";
export type { StrainLine } from "./generated/StrainLine";
export type { StrainMap, StrainMap as StrainData } from "./generated/StrainMap";

// `handle_drive_state` serves `DriveStatePayload` from disk plus a
// server-computed `age_s`.
export type DriveState = DriveStatePayload & { age_s: number };

// --- Manifest family: aliases into the generated OpenAPI `Manifest` contract
// (schema-only Rust types in `openapi.rs`) ---

type ManifestMotor = components["schemas"]["ManifestMotor"];
type StrokePlan = components["schemas"]["StrokePlan"];
type ManifestStep = components["schemas"]["ManifestStep"];
type NotchStateValue = components["schemas"]["NotchStateValue"];
type ManifestAmbient = components["schemas"]["ManifestAmbient"];
type Manifest = components["schemas"]["Manifest"];

interface RunDetail {
  mtime_utc: string;
  has_results: boolean;
  manifest: Manifest | null;
  results: Results | null;
}

type LiveTapPayload =
  paths["/api/live_tap"]["get"]["responses"][200]["content"]["application/json"];

type LiveTapStreaming = Extract<LiveTapPayload, { status: "streaming" }>;

type LiveStatusPayload =
  paths["/api/live"]["get"]["responses"][200]["content"]["application/json"];

type StrainField = "elastic" | "friction";

export type {
  ManifestMotor,
  StrokePlan,
  ManifestStep,
  NotchStateValue,
  ManifestAmbient,
  Manifest,
  RunDetail,
  LiveTapPayload,
  LiveTapStreaming,
  LiveStatusPayload,
  StrainField,
};
