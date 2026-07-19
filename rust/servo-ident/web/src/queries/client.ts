import { html } from "htm/preact";
import type { ComponentChildren } from "preact";
import { QueryClient, QueryClientProvider } from "@tanstack/preact-query";

export const queryClient = new QueryClient({
  defaultOptions: {
    queries: { retry: false, gcTime: Infinity },
    mutations: { retry: false },
  },
});

export function QueryRoot({ children }: { children: ComponentChildren }) {
  return html`<${QueryClientProvider} client=${queryClient}>${children}<//>`;
}
