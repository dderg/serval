---
title: 'Incremental Stream Planning'
type: 'refactor'
created: '2026-06-22'
status: 'done'
baseline_commit: 'c25061e581df9d66868c4a045faa583331e91a7c'
context:
  - '{project-root}/_bmad-output/specs/spec-incremental-planning/SPEC.md'
  - '{project-root}/_bmad-output/specs/spec-incremental-planning/finality-barrier.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** `StreamState::commit` re-fits, re-plans, and re-lowers the whole uncommitted buffer every batch (`rust/motion-engine/src/stream.rs:218-363`). The `keep_secs=0.5s` holdback keeps a fat tail in the buffer that gets re-planned-to-rest each call, so `pipe_plan` grows with buffer depth and spikes to 217 ms (avg 27 ms) re-deriving a 60+-move tail. One such stall drains the MCU buffer and a piece lands in the MCU's past → `-308 PieceStartInPast` crash (EtherCAT 1 kHz trips first).

**Approach:** Commit every move up to the last **finality barrier** as soon as it is final, and never build the trailing brake-to-rest during steady streaming. The barrier is the **reconvergence point** of the backward velocity sweep — the last seam whose velocity sits at `min(v_forward, ceiling)` rather than being dragged below it by the buffer's fictional terminal `v=0`. Committing up to the barrier evicts everything but the open tail (≈ one braking distance), so the buffer — and therefore each fit/plan/lower — stays bounded by open-tail length, flat in total depth. The brake-to-rest becomes a flush-only artifact: built once at true end-of-stream, or on a producer-stall watermark.

## Boundaries & Constraints

**Always:**
- Output-equivalent: committed trajectory must equal a full re-plan within ε=`1e-6` on the iterative (disk-ODE) velocity stage; deterministic stages (fit, lower) exact-equal. Throughput is never traded for cheaper planning.
- The barrier is found structurally by the backward sweep's reconvergence, never by a kinematic estimate — arcs/clothoids handled exactly. The committable boundary is the latest **clean seam** (zero-curvature, per `is_clean_seam`) at-or-before the barrier; the warm-start's scalar entry_v handoff stays valid.
- The buffer-terminal `v=0` is an artifact and must never be selected as a barrier (it is below its ceiling, on the way down). A real reversal stop (cap=0, at its ceiling) may.
- `committed_head_len` / `fit_chain_with_head_restore` front-edge window-invariance stays intact (orthogonal to the barrier — front edge vs trailing ramp).
- Fail loudly: a `debug_assert` that no already-committed seam is ever later revised. A watermark-triggered brake-to-rest that cannot fit the remaining locked lead raises a **distinct, self-identifying error** (not a generic fault) so a downstream crash is unambiguously traced to "solve-time constant too short."

**Ask First:**
- Adding any fit/plan/lower **cache** of committed intermediates — the eviction-based design needs none; if reuse seems required, the barrier logic is likely wrong. HALT.
- Moving input shaping / pressure advance into the solver/limits path (would break barrier exactness, needing a shaper-window setback).

**Never:**
- Raising `lead_secs`, ring size, or buffer horizons (masks the stall).
- Changing optimization quality or the planner algorithms (fitter, velocity sweep, lowering) — this is a compute-cost restructuring; the trajectory is unchanged.
- Retaining the fixed-time `keep_secs` heuristic as the commit gate (the barrier supersedes it).
- Incremental coverage of `arc_fit` run reconstructions (off by default; out of scope, deferred-work.md:69).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Steady stream | Deep buffer, moves appended at far end | Commit up to barrier; buffer left = open tail; no brake-to-rest built | N/A |
| Append after commit | Locked prefix committed, arbitrary moves appended | Every locked seam unchanged (pos, time, velocity) | N/A |
| True end-of-stream (flush) | Producer done, `commit(force=true)` | Brake-to-rest built once, decel to rest | N/A |
| Producer-stall watermark | Locked lead drops to `t_brake(v_barrier)+solve_const+margin` | Force-drain to rest with ≥ margin lead; if a move arrives in-window, discard provisional brake, resume locked commits | N/A |
| Watermark fired too late | Remaining locked lead < solve-time constant | Distinct `BrakeToRestShortfall` error, attributable | Raise, do not pad |
| Committed-seam revision | A later solve would lower an already-committed seam | `debug_assert` trips (unreachable under proof) | Fail loud |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/velocity.rs:116-368` -- `plan_velocity_warm_start`; boundary-velocity array built by forward pass (`:255-269`) then backward pass from terminal `v=0` (`:270-279`). Barrier = last seam the backward pass left unchanged. Returns `VelocityProfile` (`:64-67`).
- `rust/geometry/src/disk.rs:46-51,279-292` -- curvature cap `limit_speed(κ,a)`; `eval_profile` takes `min(f.v,b.v)` — the reconvergence locus.
- `rust/motion-engine/src/stream.rs:289-363` -- commit selection (`keep_secs`/`is_clean_seam` today); `StreamConfig.keep_secs:19-23`; persistent seam state `:92-107`.
- `rust/motion-engine/src/stream_planner.rs:599-735` -- batch driver; watermark `remaining=(t_committed+LEAD-SAFETY_MARGIN)-esc` (`:612-618`); idle-drain `commit(true)` (`:622-642`). `LEAD=0.25` (`anchor.rs`), `SAFETY_MARGIN=0.25`.
- `rust/host-rt/.../pump.rs:880-936` -- `-308` origin: `arrival_lead_ticks<0` → `transit_diag_alert`. (Validation target, not edited.)
- `rust/motion-engine/src/stream/tests.rs` -- helpers `cfg/cfg_bench/line/line_bench` (`:6-59`); regression guards to keep green.
- `rust/motion-engine/Cargo.toml` -- dev-deps; `proptest` not yet present.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/velocity.rs` -- During the backward pass, record the highest seam index left unchanged (backward-reach ≥ forward value); expose it as `barrier: usize` on `VelocityProfile`. Also expose `v_barrier` (that seam's velocity) for watermark sizing. No change to the produced velocities. *(Done. Also guarded `pin_rest_anchor` to genuine rest anchors so a warm-start entry's deceleration is not rejected — see Change Log.)*
- [x] `rust/motion-engine/src/stream.rs` -- Replace `keep_secs`-bounded selection with `commit_count` = latest `is_clean_seam` index ≤ `profile.barrier`, **held back by `brake_to_rest_setback`** so committed bodies are terminal-independent (Change Log). Remove `keep_secs` from `StreamConfig`/`StreamState`. Fail-loud `debug_assert` that the commit never passes the barrier. Force path unchanged = whole buffer to rest.
- [x] `rust/motion-engine/src/stream.rs` -- Add `StreamError::BrakeToRestShortfall { lead_remaining, solve_const }`; `commit_stall_brake` raises it when the caller-supplied remaining locked lead is below the solve-time constant. Add jerk-limited `jerk_limited_brake_time` helper for watermark sizing only.
- [x] `rust/motion-engine/src/stream_planner.rs` -- Driver sizes the producer-stall watermark as `stall_brake_time() + STALL_SOLVE_CONST + STALL_MARGIN`, triggers the force-drain there via `commit_stall_brake`, resumes normal commits when a move arrives in-window, and passes remaining lead in so the shortfall error can fire.
- [x] `rust/motion-engine/Cargo.toml` -- Add `proptest` to `[dev-dependencies]` (`"1"`).
- [x] `rust/motion-engine/src/stream/tests.rs` -- Added: (a) **property** `locked_prefix_is_invariant_under_append` (proptest, append-invariance — positions exact, times within ε); (b) **differential** `committed_segments_match_a_full_replan` (voron perimeter + dense infill arc — committed segments byte-identical to a full re-plan's leading segments); (c) **deep-buffer cost** `open_tail_stays_bounded_as_buffer_depth_grows` (sweep 50→500, retained open tail bounded, flat in depth); (d) **negative** `stall_brake_shortfall_is_attributable_and_fails_loud`. Plus geometry unit tests for the barrier and the warm-start guard.

**Acceptance Criteria:**
- Given a deep-buffer stream swept from a few to several hundred moves, when committing per batch, then per-commit `pipe_plan` time is bounded by open-tail length and flat in total depth (no 217 ms growth).
- Given a committed locked prefix, when arbitrary further moves are appended, then every locked seam's position, time, and velocity are unchanged and the buffer-terminal rest is never the barrier.
- Given a fully-streamed print with no producer stall, when it runs to flush, then the brake-to-rest solve is invoked exactly once (at end); injecting a mid-print stall invokes it once more, triggered with ≥ braking+margin lead, with no late dispatch.
- Given the existing stream regression suite (`cold_run_infill_streams_without_overcommit`, `head_trim_preserves_position_and_extrusion_continuity`, continuity-commit suite), when run, then all stay green — no `OverCommitted`, no head-trim continuity break.

## Spec Change Log

- **Finding (impl): the seam-reconvergence barrier alone is not output-equivalent — a brake-distance setback is required.** The companion locates the barrier as the last seam at `min(v_forward, ceiling)`. That guarantees seam *velocity* finality, but the lowering reconstructs each move's velocity *body* against its run terminal, so the move ending at the reconvergence seam has its interior shaped by the buffer's tentative rest. Empirically this made the incrementally-committed trajectory measurably slower than a full re-plan (≈10% with a 1-move look-ahead; ≈0.6% even with deep look-ahead) and made the committed prefix change when a move was appended — both forbidden by the throughput constraint. **Amended (`stream.rs`):** the commit boundary is held back from the buffer's tentative terminal by `brake_to_rest_setback` — a safe over-estimate (`v_peak · t_brake`) of the jerk-limited stopping distance from the buffer's peak feedrate — so every committed body is a function of geometry alone, never the fiction. This is **not** the rejected fixed-time `keep_secs` heuristic: it is a velocity-aware, structural distance derived from the same `t_brake` the companion already uses to "bound how many moves stay open." With it, committed segments are byte-identical to a full re-plan (positions exact; seam times within the iterative velocity ε), and appends leave the locked prefix invariant (positions exact; a single barrier-segment time within tens of µs). **KEEP:** the velocity `barrier` is still computed and still bounds the commit (the setback only ever tightens it); `v_barrier` still sizes the producer-stall watermark.

- **Finding (impl):** With the reconvergence barrier, a commit can land at a velocity-final seam whose brake-to-rest, when flushed over a short residual, is geometrically feasible (passes `OverCommitted`) but steep. The velocity reconstruction's `pin_rest_anchor` rejected this — it force-zeroed the acceleration of *every* run-start/run-end anchor, including the warm-start entry (`v>0`), raising `RestAnchorAccel` (surfaced by `cold_run_infill_streams_without_overcommit`). **Amended (`geometry/src/velocity.rs`):** `pin_rest_anchor` now applies only to genuine rest anchors (boundary `v ≈ 0`); a warm-start entry at `v>0` keeps its real (decelerating) acceleration. This avoids the known-bad state where the deferred brake-to-rest from a non-cruise committed velocity is rejected as a corrupt rest point. It is a reconstruction-artifact guard, not an optimization-quality change — and it makes an incremental warm-start entry *match* a full re-plan's interior point (never pinned), strengthening output-equivalence. **KEEP:** the guard lives at the call site, so the `pin_rest_anchor` unit tests still validate the function in isolation and stay green.

## Design Notes

The cost win is **eviction, not caching**: committing up to the barrier shrinks the buffer to the open tail (≈ one braking distance from the frontier), so the next fit/plan/lower is inherently cheap. Do **not** build a fit/plan/lower cache — the structural proof (finality-barrier.md) is what guarantees the committed seams are final, so there is nothing to reuse or compare. The only runtime guard is the fail-loud `debug_assert`.

Barrier detection rides on data the sweep already computes — the backward pass at `velocity.rs:270-279` already lowers `v[j]`; the barrier is simply the highest `j` it left untouched. `v_barrier` is free (the locked solve ends there) and feeds the watermark via the straight-line `t_brake` (a safe over-estimate — curvature only shortens the stop, so the trigger fires slightly early, which is safe).

## Verification

**Commands:**
- `cargo nextest run -p motion-engine` -- expected: new property/differential/deep-buffer/negative tests pass; all existing stream regressions green.
- `cargo nextest run -p geometry -E 'test(velocity)'` -- expected: barrier exposure leaves produced velocities unchanged.
- `cargo test --doc -p geometry -p motion-engine` -- expected: green if doc examples touched.
- `./scripts/ci.sh quick` -- expected: fully green (ruff, rust-test, rust-clippy `-D warnings`, rust-fmt, watchdog-canary) before PR.

**Manual checks:**
- EtherCAT bench (CAP-4, outside CI): a representative dense G-code completes with no `-308 PieceStartInPast` and no `transit_diag` negative-arrival-lead attributable to a `pipe_plan` spike; dispatch frontier stays ahead of wall-time throughout. Use `query-logs` / `mcu-diagnostics` skills.

## Suggested Review Order

**The finality barrier (the core mechanism)**

- Entry point — the reconvergence barrier: highest seam the backward sweep left at its forward/ceiling value; `v_barrier` feeds the watermark.
  [`velocity.rs:296`](../../rust/geometry/src/velocity.rs#L296)
- The barrier's contract, exposed on the velocity profile (seam index == committable move count).
  [`velocity.rs:77`](../../rust/geometry/src/velocity.rs#L77)
- Warm-start guard: only genuine rest anchors (`v≈0`) are pinned, so a deferred brake's steep entry is not rejected as a corrupt rest point.
  [`velocity.rs:355`](../../rust/geometry/src/velocity.rs#L355)

**Committing up to the barrier (incremental, output-equivalent)**

- Commit selection: latest clean seam ≤ barrier, held back by the brake-distance setback; fail-loud `debug_assert` fences the barrier.
  [`stream.rs:335`](../../rust/motion-engine/src/stream.rs#L335)
- The setback — why the seam-reconvergence barrier alone is not body-final, and how `v·t_brake` makes committed bodies terminal-independent.
  [`stream.rs:582`](../../rust/motion-engine/src/stream.rs#L582)

**The deferred brake-to-rest (flush-only)**

- `commit_stall_brake`: materialize the brake-to-rest on a producer stall; fail loud and attributable if the locked lead is below the solve budget.
  [`stream.rs:421`](../../rust/motion-engine/src/stream.rs#L421)
- The self-identifying shortfall error — traces a late dispatch to "solve-time constant too short," never a generic fault.
  [`stream.rs:45`](../../rust/motion-engine/src/stream.rs#L45)
- Jerk-limited `t_brake`, used only to size the watermark (a safe over-estimate), never to locate the barrier.
  [`stream.rs:557`](../../rust/motion-engine/src/stream.rs#L557)
- Driver: size the stall watermark from `v_barrier`, trigger the drain there, pass remaining lead in for the shortfall guard.
  [`stream_planner.rs:615`](../../rust/motion-engine/src/stream_planner.rs#L615)

**Tests (the proof was implemented faithfully)**

- Output-equivalence: committed segments byte-identical to a full re-plan's leading segments (voron perimeter + dense infill arc).
  [`stream/tests.rs:479`](../../rust/motion-engine/src/stream/tests.rs#L479)
- Append-invariance (proptest): locked-prefix positions exact, seam times within the iterative ε.
  [`stream/tests.rs:608`](../../rust/motion-engine/src/stream/tests.rs#L608)
- Flat-in-depth cost: the retained open tail stays bounded as buffer depth sweeps 50→500.
  [`stream/tests.rs:524`](../../rust/motion-engine/src/stream/tests.rs#L524)
- Negative: `BrakeToRestShortfall` fires, attributable, when the locked lead is below the solve budget.
  [`stream/tests.rs:555`](../../rust/motion-engine/src/stream/tests.rs#L555)
