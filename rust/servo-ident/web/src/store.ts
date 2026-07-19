import { useLayoutEffect, useReducer } from "preact/hooks";

const listeners = new Set<() => void>();

function notify() {
  for (const listener of [...listeners]) listener();
}

function subscribe(listener: () => void) {
  listeners.add(listener);
  return () => listeners.delete(listener);
}

function useStore() {
  const [, bump] = useReducer((n) => n + 1, 0);
  useLayoutEffect(() => subscribe(() => bump(undefined)), []);
}

export { notify, subscribe, useStore };
