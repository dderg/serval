import { useQuery } from "@tanstack/preact-query";
import { getDriveState } from "../api/drive";
import { queryClient } from "./client";
import type { DriveState } from "../wire";

export const driveKey = ["drive", "state"] as const;

export function driveStateOptions() {
  return { queryKey: driveKey, queryFn: getDriveState };
}

export function useDriveState() {
  return useQuery(driveStateOptions());
}

export function driveStateData(): DriveState | undefined {
  return queryClient.getQueryData<DriveState>(driveKey);
}

export function driveData(): DriveState {
  const data = driveStateData();
  if (!data) throw new Error("drive state not loaded");
  return data;
}

export function currentDriveAgeS(): number | null {
  const data = driveStateData();
  const updatedAt = queryClient.getQueryState(driveKey)?.dataUpdatedAt;
  if (!data || !updatedAt) return null;
  return data.age_s + (Date.now() - updatedAt) / 1000;
}

export function fetchDriveState(): Promise<DriveState> {
  return queryClient.fetchQuery(driveStateOptions());
}
