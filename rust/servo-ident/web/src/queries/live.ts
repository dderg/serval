import { getLiveStatus } from "../api/live";

export const liveStatusKey = ["live", "status"] as const;

export function liveStatusQuery() {
  return { queryKey: liveStatusKey, queryFn: getLiveStatus, staleTime: 0 };
}
