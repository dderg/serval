import { useEffect, useReducer } from "../vendor/htm-preact-standalone-3.1.1.mjs";

// Minimal subscribe store bridging the mutable `state` object to preact:
// mutate state, call notify(), and every mounted component that called
// useStore() re-renders from the fresh state.

const listeners = new Set();

function notify() {
  for (const listener of [...listeners]) listener();
}

function subscribe(listener) {
  listeners.add(listener);
  return () => listeners.delete(listener);
}

function useStore() {
  const [, bump] = useReducer((n) => n + 1, 0);
  useEffect(() => subscribe(() => bump()), []);
}

export { notify, subscribe, useStore };
