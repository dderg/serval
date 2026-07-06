// Runs the motion-playground WASM (the real fitter → planner → lowerer →
// shaper pipeline) off the main thread so typing in the gcode box never
// blocks the UI. Messages in: { seq, gcode, config }; out: { seq, snapshot,
// planMs } or { seq, error }.
import init, { plan } from "./wasm-playground/motion_playground.js";

const ready = init();

self.onmessage = async (e) => {
  const { seq, gcode, config } = e.data;
  try {
    await ready;
    const t0 = performance.now();
    const json = plan(gcode, JSON.stringify(config));
    const planMs = performance.now() - t0;
    self.postMessage({ seq, snapshot: JSON.parse(json), planMs });
  } catch (err) {
    self.postMessage({ seq, error: typeof err === "string" ? err : String(err?.message ?? err) });
  }
};
