import { client, unwrap } from "./client";

export async function listRuns() {
  return unwrap(await client.GET("/api/runs"));
}

export async function getRunManifest(name: string) {
  return unwrap(await client.GET("/api/runs/{name}/manifest", { params: { path: { name } } }));
}

export async function getRunResults(name: string) {
  return unwrap(await client.GET("/api/runs/{name}/results", { params: { path: { name } } }));
}

export async function getRunPlotSeries(name: string) {
  return unwrap(await client.GET("/api/runs/{name}/plot_series", { params: { path: { name } } }));
}

export async function getRunPath(name: string) {
  return unwrap(await client.GET("/api/runs/{name}/path", { params: { path: { name } } }));
}

export async function getRunStrain(name: string) {
  return unwrap(await client.GET("/api/runs/{name}/strain", { params: { path: { name } } }));
}

export async function postRunAnalyze(name: string) {
  return unwrap(await client.POST("/api/runs/{name}/analyze", { params: { path: { name } } }));
}

export async function postRunNote(name: string, note: string) {
  return unwrap(
    await client.POST("/api/runs/{name}/note", {
      params: { path: { name } },
      body: { note },
    }),
  );
}

export async function deleteRun(name: string) {
  return unwrap(await client.DELETE("/api/runs/{name}", { params: { path: { name } } }));
}

export type RunSummary = Awaited<ReturnType<typeof listRuns>>[number];
export type Manifest = Awaited<ReturnType<typeof getRunManifest>>;
export type Results = Awaited<ReturnType<typeof getRunResults>>;
export type StepResult = Results["steps"][number];
export type DriveResult = StepResult["drives"][string];
export type DifferentialResult = NonNullable<StepResult["differential"]>;
export type PlotSeries = Awaited<ReturnType<typeof getRunPlotSeries>>;
export type PlotStep = PlotSeries["steps"][number];
export type RunPath = Awaited<ReturnType<typeof getRunPath>>;
export type StrainMap = Awaited<ReturnType<typeof getRunStrain>>;
export type NoteResponse = Awaited<ReturnType<typeof postRunNote>>;
export type DeleteResponse = Awaited<ReturnType<typeof deleteRun>>;
