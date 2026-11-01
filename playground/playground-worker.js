// Runs the motion-playground WASM (the real fitter → planner → lowerer →
// shaper pipeline) off the main thread so typing in the gcode box never
// blocks the UI. Messages in: { seq, gcode, config }; out: zero or more
// { seq, snapshot, partial: true } as the trajectory grows, then
// { seq, snapshot, planMs } — or { seq, error }.
import init, { plan_streaming } from "./wasm-playground/motion_playground.js";

const ready = init();

const PARTIAL_POST_INTERVAL_MS = 80;

self.onmessage = async (e) => {
  const { seq, gcode, config } = e.data;
  try {
    await ready;
    const t0 = performance.now();
    let lastPartialAt = -Infinity;
    const json = plan_streaming(gcode, JSON.stringify(config), (partialJson) => {
      const now = performance.now();
      if (now - lastPartialAt < PARTIAL_POST_INTERVAL_MS) return;
      lastPartialAt = now;
      self.postMessage({ seq, snapshot: JSON.parse(partialJson), partial: true });
    });
    const planMs = performance.now() - t0;
    self.postMessage({ seq, snapshot: JSON.parse(json), planMs });
  } catch (err) {
    self.postMessage({ seq, error: typeof err === "string" ? err : String(err?.message ?? err) });
  }
};
