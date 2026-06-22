---
title: 'Input shaper port to live post-processing path'
type: 'feature'
created: '2026-06-22T00:00:00+02:00'
status: 'done'
baseline_commit: 'bfdf06e2b5cb10b5b4a8d8e412c30d08731605a6'
context:
  - '{project-root}/_bmad-output/specs/spec-input-shaper-port/SPEC.md'
  - '{project-root}/_bmad-output/specs/spec-input-shaper-port/code-map.md'
---

<frozen-after-approval reason="human-owned intent - do not modify unless human renegotiates">

## Intent

**Problem:** Configured `smooth_zv` and `smooth_mzv` post-processors are compiled and live-retunable, but the live `StreamState -> lower_move -> Sampler` path only consumes PA gain; spatial axes bypass post-processing and `CompiledChain.kernel` is not read. X/Y resonance compensation is therefore absent on the post-TOPP motion path.

**Approach:** Replace the gain-only lowering hook with an ordered axis post-processor chain that evaluates an analytic unshaped `BaseSignal`, then applies PA and smooth shaper stages through one interface after limit computation. Reuse the existing smooth kernel builders and `ShapedSignal` convolution math, carry enough past history and future lookahead to avoid seams, and mark the superseded emit stack as legacy without deleting it.

## Boundaries & Constraints

**Always:** Shaping runs after velocity/limit planning and never feeds limit computation. Empty chains emit the unshaped base unchanged. PA remains exact through analytic base value/derivative/second-derivative evaluation, and zero-support stages reuse the incoming fit grid with no adaptive re-refinement. Smooth shapers reuse existing kernel math and fail loudly on missing history/lookahead, non-finite samples, or invalid frequency. Post-processors are axis-agnostic and apply in `[axis] post_processors:` declaration order.

**Ask First:** Any change that alters planner limits, weakens throughput/trajectory optimality, deletes the legacy planner/streaming/emit stack, changes config syntax, or introduces a cheaper shaping algorithm instead of the existing convolution primitive.

**Never:** Do not revive TOPP, legacy `[input_shaper]`, resonance calibration, or classic impulse shaper types. Do not silently pad missing convolution history or future lookahead with zeros, and do not clamp at internal batch seams. Do not special-case shapers to spatial axes or PA to follower axes.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|----------------------------|----------------|
| Unconfigured axis | Axis chain is empty | Emitted NURBS is byte-identical to current unshaped output | None |
| PA-only axis | Axis chain has `linear_pressure_advance` | Output matches current PA live path, including XYZ identity when PA is only on follower | None |
| Smooth shaper axis | Axis chain has `smooth_zv` or `smooth_mzv` | Emitted trajectory matches `ShapedSignal` convolution of the same base signal to fit tolerance | Non-finite shaped sample returns a clear error |
| Backward streaming seam | Shaper window reaches into already committed history | Two-batch output matches one-batch output to fit tolerance and velocity is continuous at the seam | Missing history returns a clear error |
| Forward live edge | Shaper window reaches into not-yet-committed future base | Commit holds back affected shaped samples until the future base exists; internal seams never clamp | Missing lookahead returns a clear error |
| Live retune | `SET_POST_PROCESSOR FREQUENCY_HZ=<value>` during streaming | Already committed output is unchanged; future plans use the new kernel | Non-finite or `<= 0` frequency is rejected without mutating the instance |

</frozen-after-approval>

## Code Map

- `rust/trajectory/src/post_processor.rs` -- Replace `CompiledChain { kernel, gain }`/`PlanAction` flattening with ordered post-processor chain members; add fail-loud frequency validation in `set_param`; keep `apply_derivative_gain` marked legacy.
- `rust/trajectory/src/shaper.rs` -- Expose/reuse the existing `ShapedSignal` convolution math through an evaluator/closure-taking path or equivalent sampled-analytic adapter for live post-processing.
- `rust/motion-engine/src/lowering.rs` -- Make `Sampler` provide the analytic unshaped base signal and remove `with_pa`/`follower_gains`; zero-support stages evaluate on the incoming fit grid.
- `rust/motion-engine/src/stream.rs` -- Apply ordered per-axis post-processing at commit time with cross-batch history, forward lookahead, live-edge hold-back, and fail-loud seam checks.
- `rust/motion-engine/src/config.rs` -- Keep `[post_processor]` syntax and live `set_param`/compile flow pointed at the new ordered chain representation.
- `rust/motion-engine/src/lowering/tests.rs`, `rust/motion-engine/src/stream/tests.rs`, `rust/trajectory/src/post_processor/tests.rs` -- Cover PA parity, smooth-shaper oracle matching, live retune validation, and streaming seam equivalence.
- `rust/trajectory/src/lib.rs`, `rust/trajectory/src/beta.rs`, `rust/trajectory/src/streaming/`, `rust/trajectory/src/emit_shaped.rs`, `rust/motion-engine/src/planner.rs` -- Mark old pipeline as legacy/deprecated without making the live path depend on it.

## Tasks & Acceptance

**Execution:**
- [x] `rust/trajectory/src/post_processor.rs` -- Introduce an ordered chain representation capable of PA and smooth-shaper stages in declaration order -- removes the one-kernel-plus-one-gain flattening that loses composition semantics.
- [x] `rust/trajectory/src/post_processor.rs` -- Reject non-finite or non-positive shaper `frequency_hz` at runtime and preserve the existing valid value after a rejected update -- closes the fail-loud gap; `config.rs` already delegates and build-time validation already exists.
- [x] `rust/motion-engine/src/lowering.rs` -- Refactor `Sampler` into the exact base evaluator for spatial and follower axes, including value, derivative, and second derivative -- gives PA and shapers one axis-agnostic input contract.
- [x] `rust/motion-engine/src/lowering.rs` and `rust/motion-engine/src/stream.rs` -- Ensure zero-support processors reuse the incoming fit grid with no adaptive re-refinement and read the analytic base instead of a re-differentiated fitted NURBS -- preserves PA byte-identity.
- [x] `rust/motion-engine/src/stream.rs` -- Add the post-lowering fold over axis chains, with per-axis history, retained future lookahead, live-edge hold-back, and explicit errors for unavailable convolution windows -- enables live shaping without limit feedback.
- [x] `rust/trajectory/src/shaper.rs` or a sibling module -- Reuse `ShapedSignal` kernel math through an evaluator/closure-compatible path rather than reimplementing convolution or forcing the first-stage base through NURBS -- keeps validated math intact.
- [x] Legacy emit stack files -- Mark superseded planner/streaming/emit/NURBS post-processing code as legacy or deprecated, but do not delete it -- makes the boundary obvious until the maintainer removes the full old stack.
- [x] Test files -- Add focused Rust unit/integration coverage for the matrix scenarios and keep tests separate from production modules -- proves behavior and fail-loud paths.

**Acceptance Criteria:**
- Given an empty post-processor chain, when a move is emitted through the live path, then every axis matches the previous unshaped output.
- Given a PA-only follower chain, when the same move is emitted before and after the refactor, then the emitted output is byte-identical to the current PA live behavior because the incoming fit grid is reused and PA reads the analytic base.
- Given a smooth shaper on any axis, when the live path emits the move, then the shaped output matches the `ShapedSignal` oracle for the unshaped base to fit tolerance.
- Given a shaped move sequence split across two streaming commits, when compared with the same sequence committed as one batch, then position and velocity match at the seam to tolerance for both past-history and future-lookahead sides of the shaper window.
- Given shaped samples near the live commit frontier, when the required future base is not yet planned, then the stream holds those samples back rather than clamping or padding at an internal seam.
- Given `SET_POST_PROCESSOR FREQUENCY_HZ` with `NaN`, infinity, zero, or negative value, when the update is applied, then it returns the configured error and the previous kernel remains active.

## Design Notes

The first `BaseSignal` is analytic, built from velocity phases plus spatial geometry or follower demand. PA has zero support and evaluates locally as `value + k * deriv` and `deriv + k * deriv2` on the existing grid; it does not trigger a fresh adaptive fit. Nonzero-support shapers use the rescued convolution math over a two-sided window, then refit their result for later stages. This keeps processor behavior axis-agnostic: spatial and follower axes differ only in base evaluation, not in which processor types are allowed.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p trajectory -p motion-engine -E 'test(post_processor) | test(lowering) | test(stream)'` -- expected: targeted Rust tests pass.
- `cd rust && cargo nextest run -p trajectory -p motion-engine` -- expected: affected crates pass.
- `./scripts/ci.sh rust-clippy` -- expected: no warnings promoted to errors.
- `./scripts/ci.sh rust-fmt` -- expected: formatting is clean.

## Suggested Review Order

**Chain Model**

- Ordered stages preserve declared PA/shaper composition while legacy fields stay marked.
  [`post_processor.rs:38`](../../../rust/trajectory/src/post_processor.rs#L38)

- Runtime and compile-time validation close the direct-construction frequency gap.
  [`post_processor.rs:98`](../../../rust/trajectory/src/post_processor.rs#L98)

**Base Evaluation**

- Analytic base state keeps PA exact before any nonzero-support stage.
  [`lowering.rs:129`](../../../rust/motion-engine/src/lowering.rs#L129)

- Zero-support stages apply on the incoming grid until the first shaper.
  [`lowering.rs:162`](../../../rust/motion-engine/src/lowering.rs#L162)

**Live Shaping**

- Commit flow applies the post-lowering fold and stores committed base history.
  [`stream.rs:240`](../../../rust/motion-engine/src/stream.rs#L240)

- Forward support holds back live-edge output until required future base exists.
  [`stream.rs:286`](../../../rust/motion-engine/src/stream.rs#L286)

- Axis-chain folding validates ragged axes and reports fail-loud errors.
  [`stream.rs:339`](../../../rust/motion-engine/src/stream.rs#L339)

- Shaper window checks distinguish absolute stream edges from internal seams.
  [`stream.rs:367`](../../../rust/motion-engine/src/stream.rs#L367)

- Binary-search segment lookup tolerates tiny contiguous-segment gaps.
  [`stream.rs:425`](../../../rust/motion-engine/src/stream.rs#L425)

- Shaped refit rejects empty templates and samples derivatives inside domain.
  [`stream.rs:468`](../../../rust/motion-engine/src/stream.rs#L468)

- Trailing PA after a shaper uses a local primitive, not legacy emit code.
  [`stream.rs:522`](../../../rust/motion-engine/src/stream.rs#L522)

**Convolution Reuse**

- Closure-backed construction lets live shaping read the analytic base directly.
  [`shaper.rs:46`](../../../rust/trajectory/src/shaper.rs#L46)

**Coverage**

- Stream tests cover oracle matching, hold-back release, idle gaps, and seams.
  [`stream/tests.rs:146`](../../../rust/motion-engine/src/stream/tests.rs#L146)

- Post-processor tests cover order, duplicate rejection, and validation paths.
  [`post_processor/tests.rs:30`](../../../rust/trajectory/src/post_processor/tests.rs#L30)
