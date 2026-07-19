import type { components, paths } from "./api/openapi.generated";

type Schema = components["schemas"];
type DifferentialMode = Schema["DifferentialMode"];
type FrfMode = DifferentialMode;
type DifferentialResult = Schema["DifferentialResult"];
type DriveResult = Schema["DriveResult"];
type ResultDrive = DriveResult;
type DriveStateParam = Schema["DriveStateParam"];
type DriveParam = DriveStateParam;
type LiveCapture = Schema["LiveCapture"];
type LiveStatus = Schema["LiveStatus"];
type SpatialFrame = Schema["SpatialFrame"];
type Metrics = Schema["Metrics"];
type DriveMetrics = Metrics;
type Move = Schema["Move"];
type MoveMetrics = Move;
type NoteResponse = Schema["NoteResponse"];
type NotePayload = NoteResponse;
type PlotDifferential = Schema["PlotDifferential"];
type DifferentialPlot = PlotDifferential;
type PlotPsd = Schema["PlotPsd"];
type PsdData = PlotPsd;
type PlotRingdown = Schema["PlotRingdown"];
type RingdownPlot = PlotRingdown;
type PlotRingdownSource = Schema["PlotRingdownSource"];
type RingdownSource = PlotRingdownSource;
type PlotPath = Schema["PlotPath"];
type PlotSeries = Schema["PlotSeries"];
type PlotStep = Schema["PlotStep"];
type Results = Schema["Results"];
type RingdownMode = Schema["RingdownMode"];
type RunPath = Schema["RunPath"];
type RunPathStep = Schema["RunPathStep"];
type RunSummary = Schema["RunSummary"];
type StepResult = Schema["StepResult"];
type ResultStep = StepResult;
type TorqueSummary = Schema["TorqueSummary"];
type TorqueMetrics = TorqueSummary;
type VerdictSummary = Schema["VerdictSummary"];
type StrainBelt = Schema["StrainBelt"];
type StrainLine = Schema["StrainLine"];
type StrainMap = Schema["StrainMap"];
type StrainData = StrainMap;
type DriveStatePayload = Schema["DriveStatePayload"];
type DriveState = DriveStatePayload & { age_s: number };

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
  DifferentialMode,
  FrfMode,
  DifferentialResult,
  DriveResult,
  ResultDrive,
  DriveStateParam,
  DriveParam,
  LiveCapture,
  LiveStatus,
  SpatialFrame,
  Metrics,
  DriveMetrics,
  Move,
  MoveMetrics,
  NoteResponse,
  NotePayload,
  PlotDifferential,
  DifferentialPlot,
  PlotPsd,
  PsdData,
  PlotRingdown,
  RingdownPlot,
  PlotRingdownSource,
  RingdownSource,
  PlotPath,
  PlotSeries,
  PlotStep,
  Results,
  RingdownMode,
  RunPath,
  RunPathStep,
  RunSummary,
  StepResult,
  ResultStep,
  TorqueSummary,
  TorqueMetrics,
  VerdictSummary,
  StrainBelt,
  StrainLine,
  StrainMap,
  StrainData,
  DriveStatePayload,
  DriveState,
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
