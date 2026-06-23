---
id: SPEC-incremental-planning
companions:
  - finality-barrier.md
sources:
  - ../../implementation-artifacts/investigations/piece-start-in-past-clock-rebase-investigation.md
  - ../../implementation-artifacts/deferred-work.md
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only — consult them only if you need narrative rationale or prose color this contract intentionally omits.

# Incremental Stream Planning

## Why

A **pain to solve**, with a throughput **mandate** behind it. Today `StreamState::commit` re-fits, re-plans, and re-lowers the *entire* uncommitted move buffer on every commit batch (`rust/motion-engine/src/stream.rs:218-362`); nothing fitted or planned is cached across calls. Cost is O(buffer depth) per commit. The investigation `piece-start-in-past-clock-rebase-investigation.md` confirmed this is the live root cause of the `-308 PieceStartInPast` crashes on the EtherCAT bench: `pipe_plan` spikes to **217 ms** (avg 27 ms) re-planning a 60+-move tail, while fit and lower stay in microseconds. The planner spends its real-time budget re-deriving geometry it already solved instead of advancing the dispatch frontier, so the delivered lead stays chronically thin; one 100–217 ms stall then drains the MCU buffer and a piece arrives in the MCU's past → fault → print crash. EtherCAT (1 kHz, zero slack) trips first; the USB board follows.

The fix is structural, not a cache-and-compare. Streaming only ever *appends* moves at the far end, and the velocity sweep is append-invariant except for one path: a downstream constraint pulling an earlier seam's velocity *down*. So every acceleration (pinned by the past), every cruise and corner peak (at its ceiling), and every brake into an *already-buffered* corner is **final the moment it is in the buffer**. The *only* non-final region is the trailing brake-to-rest dictated by the buffer's fictional terminal stop — and that fiction is load-bearing only at the genuine end of the stream. So we commit all final material incrementally and never even build the brake-to-rest until a real flush. This is the deferred-work.md (line 68) fix, sharpened: not "re-plan the tail cheaper" but "don't build the throwaway tail at all."

## Capabilities

- id: CAP-1
  intent: The planner commits every move up to the last finality barrier as soon as it is final, reusing the locked prefix's fit/plan/lower results instead of re-deriving them, and terminates the locked solve at the barrier's own ceiling velocity (cruise speed or corner cap, never an assumed rest). The barrier is found by running the backward velocity sweep only from the frontier back to where it reconverges with the forward/ceiling profile — never over the whole buffer.
  success: A synthetic deep-buffer stream (depth swept from a few moves to several hundred) shows per-commit `pipe_plan` time bounded by the open-tail (reconvergence) length, flat in total buffer depth — replacing today's linear growth to 217 ms.

- id: CAP-2
  intent: The finality boundary is determined structurally — a seam is final if its velocity is set by the past (acceleration), by its own ceiling (cruise / curvature cap / real reversal stop), or by an already-buffered downstream corner; only the trailing brake-to-rest bound by the buffer's tentative terminal is left open.
  success: A property test commits a stream, appends arbitrary further moves, and asserts every seam in the locked prefix is unchanged (position, time, velocity) regardless of what was appended. The buffer-terminal rest is never selected as final.

- id: CAP-3
  intent: The brake-to-rest deceleration tail is materialized only on flush — true end-of-stream, or a producer-stall low-watermark — never recomputed during steady streaming; the watermark triggers the solve with enough locked lead (braking time + solve time + margin) that it always completes before its first piece must be dispatched.
  success: Over a fully-streamed print with no producer stalls, the brake-to-rest solve is invoked exactly once (at end). Injecting a mid-print producer stall invokes it once more, triggered with ≥ braking+margin of locked lead remaining, producing a smooth decelerate-to-stop with no late dispatch and no `-308`. A move arriving inside the watermark window discards the provisional brake and resumes locked commits.

- id: CAP-4
  intent: On the EtherCAT bench, a representative dense print runs to completion without the planner-stall-induced late delivery that the full re-plan caused.
  success: A bench run of a representative dense G-code completes with no `-308 PieceStartInPast` fault and no `transit_diag` negative-arrival-lead event attributable to a `pipe_plan` spike; the dispatch frontier stays ahead of wall-time throughout.

## Constraints

- **Finality is proven, not compared.** Correctness rests on two structural facts the implementation must hold: streaming is append-only (no geometry is ever inserted between existing moves), and the velocity sweep is monotone (the backward pass only lowers velocity). The forward sweep and the prefix fit depend only on geometry at-or-behind a seam. See `finality-barrier.md`.
- **The buffer-terminal `v=0` is an artifact, not a stop.** It exists only because the queue ends and rises the moment the next move arrives. It is never committed as final and never drives a deceleration during steady streaming — only a real ceiling-touch (cruise / curvature cap / genuine reversal) or an actual flush does.
- **The locked solve needs no terminal-rest assumption.** It ends at the last barrier pinned to that barrier's ceiling velocity, so committing the prefix never requires building the brake-to-rest. The brake-to-rest is a flush-only artifact.
- **Output-equivalence is binding.** The committed trajectory must be identical to what a full re-plan produces — the non-negotiable throughput constraint forbids trading trajectory quality for cheaper planning. The structural proof is what guarantees this; an offline differential test (incremental vs full re-plan over cold_run infill and voron perimeter) checks the proof was implemented faithfully.
- **Fail loudly.** If any later commit would revise an already-committed seam, raise a clear error / `debug_assert` — unreachable under the proof, so reaching it means the barrier logic is wrong and we want to know immediately. Likewise if a watermark-triggered brake-to-rest cannot fit in the remaining locked lead (a real shortfall, not to be padded over).
- **The backward coupling is the velocity solver's alone.** The barrier, the open tail, and the deferred brake are velocity-solver concepts. The fitter is δ-local and forward — its one front-edge non-determinism is already made window-invariant by `committed_head_len` / `fit_chain_with_head_restore`, which incremental prefix reuse must keep intact. The fitter increments over the new moves plus its own local reach, separately and more simply than the solver.
- **The barrier respects real geometry for free.** Because the barrier is placed by reconvergence of the actual backward sweep (not a kinematic estimate), arcs and clothoids are handled exactly; curvature can only force a lower speed into a stop, so it only ever shortens the open tail. The jerk-limited braking closed form (`t_brake = v/a + a/j`, or `2·√(v/j)` below `a²/j`) is used only to size the flush-trigger watermark and to bound how many moves stay open — never to locate the barrier.
- **Existing stream regression tests stay green** — `cold_run_infill_streams_without_overcommit`, `head_trim_preserves_position_and_extrusion_continuity`, the continuity-commit suite (`rust/motion-engine/src/stream/tests.rs`). No `OverCommitted`, no head-trim continuity break.

## Non-goals

- **Not raising `lead_secs`, ring size, or buffer horizons.** These mask the stall, make the defect harder to catch, and are explicitly rejected as a fix direction.
- **Not changing optimization quality or the planner's algorithms** (fitter, SLP velocity relaxation, lowering). This is a compute-cost restructuring; the produced trajectory is unchanged.
- **Not retaining the fixed-time `keep_secs` heuristic as the commit gate.** The structural barrier supersedes it; the held-back region becomes exactly the flush-only brake-to-rest tail, not a fixed 0.5 s margin.
- **Not the clock double-writer fix.** That root cause (Path A / serialhdl contaminating the router anchor) is already resolved and separate.
- **Not incremental coverage of arc-fit run reconstructions.** `arc_fit` is off by default; mid-run continuity under incremental caching follows the existing deferred gap (deferred-work.md line 69) and is out of scope until arc-fit ships in production.

## Success signal

A dense print that previously crashed with `-308` on the EtherCAT bench now runs start to finish, and the `pipe_plan` trace is flat as the buffer deepens — no 100–217 ms spikes — because the planner commits final material incrementally and never builds a brake-to-rest it is about to discard. Across the whole print the deceleration solve runs once, at the end; the differential test confirms the total trajectory is identical to a full re-plan.

## Assumptions

- The per-commit cost target is "flat in buffer depth / bounded by open-tail length," not strictly O(1) per commit.
- "Identical" in CAP-2/CAP-4 means within the planner's existing numeric tolerance where the math is iterative (velocity SLP) and exact where it is deterministic (fit, lower); pinned precisely as an Open Question.
- **Input shaping and pressure advance are post-solver per-axis post-processors** (as in sota-motion; not yet in this branch) — applied to already-planned motion, with no effect on limits. The velocity barrier is therefore exact with no shaper-window setback. If shaping were ever moved into the solver/limits path, this exactness would break and a setback margin would be required.

## Open Questions

- The flush-trigger watermark uses `v_barrier` (free, tight) for the real trigger and the section max feedrate as the conservative bound on how many moves stay open — confirm the solve-time budget term (a fixed margin vs a measured planner solve-time) added on top.
- The exact equivalence tolerance for the differential test: exact-equal for the deterministic stages and what ε for the iterative velocity stage?
