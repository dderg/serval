import { client, unwrap } from "./client";
import type { DriveState } from "../wire";

export async function getDriveState(): Promise<DriveState> {
  return unwrap(await client.GET("/api/drive_state")) as DriveState;
}
