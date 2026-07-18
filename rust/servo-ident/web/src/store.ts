import { useEffect, useReducer } from "htm/preact/standalone";

// Minimal subscribe store bridging the mutable `state` object to preact:
// mutate state, call notify(), and every mounted component that called
// useStore() re-renders from the fresh state.

const listeners = new Set<() => void>();

function notify() {
  for (const listener of [...listeners]) listener();
}

function subscribe(listener) {
  listeners.add(listener);
  return () => listeners.delete(listener);
}

function useStore() {
  const [, bump] = useReducer((n) => n + 1, 0);
  useEffect(() => subscribe(() => bump(undefined)), []);
}

export { notify, subscribe, useStore };
