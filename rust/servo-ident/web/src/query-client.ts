import { html } from "htm/preact";
import type { ComponentChildren } from "preact";
import { QueryClient, QueryClientProvider } from "@tanstack/preact-query";

export const queryClient = new QueryClient({
  defaultOptions: {
    queries: { retry: false, gcTime: Infinity },
    mutations: { retry: false },
  },
});

export const queryKeys = {
  runs: ["runs"] as const,
  runDetail: (name: string) => ["runs", name, "detail"] as const,
  plotSeries: (name: string) => ["runs", name, "plot"] as const,
  runPath: (name: string) => ["runs", name, "path"] as const,
  strain: (name: string) => ["runs", name, "strain"] as const,
  driveState: ["drive", "state"] as const,
  macroHelp: (base: string) => ["moonraker", base, "macro-help"] as const,
  moonrakerHealth: (base: string) => ["moonraker", base, "health"] as const,
  liveStatus: ["live", "status"] as const,
} as const;

export function QueryRoot({ children }: { children: ComponentChildren }) {
  return html`<${QueryClientProvider} client=${queryClient}>${children}<//>`;
}
