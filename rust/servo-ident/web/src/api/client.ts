import createClient from "openapi-fetch";

import type { paths } from "./openapi.generated";

export const client = createClient<paths>({
  baseUrl: "",
  fetch: (input: Request) => globalThis.fetch(input),
});

export function unwrap<T>(result: {
  data?: T;
  error?: unknown;
  response: Response;
}): T {
  if (result.error !== undefined) {
    throw new Error(
      `${result.response.status} ${result.response.url}: ${JSON.stringify(result.error)}`,
    );
  }
  if (result.data === undefined) {
    throw new Error(
      `${result.response.status} ${result.response.url}: response had no data`,
    );
  }
  return result.data;
}
