import { client, unwrap } from "./client";

export async function getLiveStatus() {
  return unwrap(await client.GET("/api/live"));
}

export async function getLiveTap(sinceCycle?: number) {
  return unwrap(
    await client.GET("/api/live_tap", {
      params: { query: { since_cycle: sinceCycle } },
    }),
  );
}
