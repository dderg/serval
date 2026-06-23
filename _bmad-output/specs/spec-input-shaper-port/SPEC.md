---
id: SPEC-input-shaper-port
companions: [code-map.md]
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only — consult them only if you need narrative rationale or prose color this contract intentionally omits.

# Input Shaping on the new (post-TOPP) motion path

## Why

Input shaping (resonance compensation) was left off the live `temporal::multi` (Consolini-Locatelli) path when TOPP was removed — the same gap that just got closed for Pressure Advance ([[spec-pressure-advance-port]]). The shaper half of the post-processor rail is already built and tested: the `SmoothZv`/`SmoothMzv` types, the kernel builders, the `CompiledChain.kernel` field, the config parse, and the live `update_post_processor` retune path all exist. But the live emit path (`lower_move` → `Sampler::axis_state`) consumes **only** the PA `gain` field — it never reads `kernel`, and spatial axes pass through with no post-processing at all. So a configured `smooth_zv` does nothing on the live path; X/Y resonance is uncompensated. The convolution math itself is alive only in the dead `emit_shaped.rs` path. This is an **opportunity to capture**: wire the existing, validated shaper kernel onto the live emit path the way PA's gain was wired. The defining difference from PA — and the real work — is that shaping is **non-local convolution** (a ±half-support window plus cross-batch history), where PA was a purely local same-`t` correction. The anchor every trade-off resolves against: shaping is applied **after** limits and never feeds them, so it never makes the planner pick a slower algorithm; the trajectory-time it adds is the intrinsic smear of convolution, which is the whole point of shaping, not a throughput regression.

## Capabilities

- id: CAP-1
  intent: System applies the configured input-shaper kernel as a convolution on the spatial trajectory in the unified post-processing stage (CAP-5), after the toolhead trajectory is planned.
  success: A move planned with a `smooth_zv` post-processor on an axis emits a shaped trajectory for that axis equal (to fit tolerance) to the kernel convolution of the same move's unshaped trajectory — matched against the `ShapedSignal` convolution primitive as oracle; the same move with no shaper post-processor emits the unshaped trajectory unchanged. End-to-end test on the live emit path asserts both.

- id: CAP-2
  intent: Operator configures a shaper via `[post_processor]` (`type: smooth_zv` / `smooth_mzv`, `frequency_hz`) on an `[axis] post_processors:` list and retunes it live with `SET_POST_PROCESSOR NAME=<shaper> FREQUENCY_HZ=<value>`.
  success: A live `SET_POST_PROCESSOR` changes `frequency_hz` and the new value applies only to plans produced after the command (reusing the PA-proven `update_post_processor` → `set_axis_chains` swap path); held output committed before the swap is unchanged. Non-finite or non-positive `frequency_hz` is rejected loudly at both config build and runtime, matching PA's `k` validation.

- id: CAP-3
  intent: Shaping convolution windows that cross a streaming batch boundary use both carried per-axis history and retained forward lookahead so the shaped trajectory has no seam.
  success: A move sequence planned as two streamed batches emits the same shaped trajectory (to fit tolerance) as the identical sequence planned in a single batch, and velocity is continuous across the batch seam. Samples near the live commit frontier are held until the future base signal required by the shaper half-support is planned; internal batch seams never use clamp-at-edge padding. If a convolution window needs past history or future lookahead that the streaming state cannot supply, the planner fails loudly rather than truncating the kernel or padding with zeros.

- id: CAP-4
  intent: The shaper model is selected by the post-processor `type`, dispatched through the unified `AxisPostProcessor` seam (CAP-5), so additional smooth shapers can be added without touching the live convolution plumbing.
  success: `smooth_zv` and `smooth_mzv` ship; the seam is demonstrable such that adding another smooth shaper requires only a new `AxisPostProcessor` implementer plus its `type` string and kernel builder — no change to the unified stage, the base-signal evaluator, or the streaming-history seam. Per-axis composition and application order follow the `[axis] post_processors:` declaration order, type-agnostic, exactly as for PA.

- id: CAP-5
  intent: PA and input shaping execute through a single unified post-processor interface — an `AxisPostProcessor::eval(base, t)` over a `BaseSignal` evaluator of the unshaped axis — run by one post-lowering stage; PA is migrated off the baked-in `Sampler` `with_pa`/`follower_gains` mechanism onto this interface.
  success: The `with_pa` flag and `follower_gains` array are gone from `Sampler`; `lower_move` emits only the unshaped base, and a single stage applies each axis's ordered chain. PA re-homed this way emits byte-identical output to the pre-refactor live path for a PA-only plan: zero-support stages reuse the incoming fit grid with no adaptive re-refinement, and PA reads the analytic base evaluator's 1st/2nd derivative rather than a re-differentiated fitted NURBS. Adding shaping touches only a new chain member, not the stage. A PA chain and a shaper chain dispatch through the identical `eval` call.

## Constraints

- Shaping is applied **after** limit computation and **never** feeds back into limit calculation — the same rail as PA, and the same behavior as mainline's post-process ("smooth") shaper. The unshaped base signal is what drives limits and the fit grid.
- **Input shaping modifies the spatial trajectory by design.** The PA "XYZ byte-identical" guarantee does NOT apply to shaped axes and is not a goal — convolution legitimately reshapes and lengthens motion. The byte-identity that still must hold is: an axis with *no* kernel configured is emitted unchanged.
- Convolution must reuse the existing, validated kernel math verbatim — `build_smooth_zv_kernel` / `build_smooth_mzv_kernel`, the `SMOOTH_ZV_T_SM_PER_HZ` / `SMOOTH_MZV_T_SM_PER_HZ` constants, and the `ShapedSignal` convolution primitive. These are **rescued** from the dead stack (see removal constraint below), not rewritten. The only new thing is live wiring and the unified interface, not shaper math.
- The unshaped `BaseSignal` is the analytic evaluator derived from the velocity phases and geometry/follower demand. Local zero-support stages, including PA, must evaluate this incoming signal on the already-selected grid and must not trigger a separate adaptive refit; this preserves the current PA knot placement and byte-identity guarantee.
- An axis carries an **ordered chain** of post-processors applied in `[axis] post_processors:` declaration order; a shaper and a PA may coexist on one axis. PA and shaping are not special-cased relative to each other. (This replaces the current `CompiledChain { kernel, gain }` two-field flattening, which hardcodes exactly one-kernel-plus-one-gain.)
- **Post-processors are axis-agnostic.** Any processor type attaches to any axis that names it; there is no extruder special-case and no "shapeable vs non-shapeable" axis class. A shaper on a follower axis is valid and shapes that follower's signal; PA on a spatial axis is valid. The only per-axis difference is how the `BaseSignal` is evaluated (spatial geometry for `axis < 3`, follower `ratio·s + start` for followers) — not whether a given processor is permitted. The unified stage must not reject or branch on processor-type-vs-axis-class.
- Cross-batch convolution is two-sided. The live streaming state must carry per-axis history tails and retain enough not-yet-committed forward base signal for the shaper window. Clamp-at-edge behavior is valid only at the absolute print start/end, never at an internal batch seam. Fail loudly if a window needs unavailable history or lookahead rather than silently shortening the kernel, padding with zeros, or clamping at a seam.
- Fail loudly per project rule: unexpected planner state (missing history, degenerate fit input, non-finite shaped sample) raises a clear error rather than silently recovering or padding.
- Throughput is non-negotiable: shaping changes neither the planner algorithm nor the computed limits. The only trajectory-time effect permitted is the intrinsic convolution smear.
- **The new live post-processor stage must not depend on any dead-pipeline code, and the dead code is marked unmistakably as unused — it is NOT deleted in this work.** The old emission pipeline (`motion-engine::planner.rs` `PlannerHandle`, `trajectory::streaming`, `trajectory::emit_shaped`, `trajectory::beta`, and the post-processor code they alone consume — `apply_derivative_gain`, `CompiledChain.kernel`, `PlanAction::Kernel`) is retired wholesale in a separate maintainer pass. Here, mark it dead **by expression** — relocate it under a clearly-named `legacy`/`dead` module and/or attach `#[deprecated]` — so its superseded status is obvious at a glance and the live path's independence from it is enforced by the compiler, not by convention. `ShapedSignal`, the kernel builders, and the `SMOOTH_*_T_SM_PER_HZ` constants are shared primitives (consumed by both the dead emit and the new live stage) and stay where they are.

## Non-goals

- Resonance **measurement / calibration** — `TEST_RESONANCES`, `SHAPER_CALIBRATE`, accelerometer ingestion. This spec applies an already-chosen shaper; choosing the frequency is out of scope.
- Classic discrete-impulse shapers (ZV/ZVD/EI as impulse trains). Only the smooth convolution-kernel family that already exists on the rail rides this path; new shapers must be smooth-kernel.
- The legacy `[input_shaper]` config section — it stays rejected; configuration is `[post_processor]` only.
- Reviving OR deleting the dead `emit_shaped.rs` / `temporal::topp` pipeline. The live path gets its own convolution and does not touch the dead stack; this work only **marks** that stack unused (see the marking constraint). Retiring it wholesale is a separate maintainer pass, out of scope here.
- Any plan-time or limit-time shaping coupling. Shaping never constrains velocity, acceleration, or jerk.

## Success signal

A representative print with a configured `smooth_zv` (or `smooth_mzv`) post-processor on X/Y runs on the live path: the emitted X/Y trajectory is the resonance-compensated convolution of the planned motion (matching the validated dead-path shaper output to tolerance), `SET_POST_PROCESSOR FREQUENCY_HZ=` retunes only future plans, the shaped motion is continuous across streaming batch boundaries, and the full Rust suite is green.

## Assumptions

- The shaper kernel math is real, correct, and tested (`build_smooth_*_kernel`, `ShapedSignal`, the `T_SM_PER_HZ` constants); the remaining work is wiring it onto the live emit path through the unified interface, not building the convolution — mirroring PA's `apply_derivative_gain`-is-already-correct assumption.
- `ShapedSignal` currently takes a `ScalarNurbs`; live shaping may need an evaluator/closure-taking entry point or an equivalent sampled-analytic adapter so the convolution can consume the analytic `BaseSignal` without forcing PA onto a re-differentiated NURBS representation.
- The config parse and live `update_post_processor` rail already compile `smooth_zv`/`smooth_mzv` and drive the live `set_axis_chains` swap, so CAP-2 is largely confirmation plus closing the `frequency_hz` fail-loud validation gap. The compiled per-axis representation changes from `CompiledChain { kernel, gain }` to an ordered chain (CAP-5), so this rail is re-pointed, not rebuilt.
