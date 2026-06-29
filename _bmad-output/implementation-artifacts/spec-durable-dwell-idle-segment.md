---
title: 'Durable dwell as a committed idle segment'
type: 'bugfix'
created: '2026-06-29'
status: 'done'
baseline_commit: '448413f41fbec4b17544a568d2f91fce40a9165e'
context: ['{project-root}/_bmad-output/brainstorming/brainstorming-session-2026-06-28-homing-dwell.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** `planner.dwell(N)` records a pause as a soft `t_committed` bump (`advance_time`, stream.rs:235) with no dispatched cargo. Any timeline reset erases it: idle re-anchor (stream_planner.rs:575), `HomeDrip` (stream_planner.rs:638-640), and host grounding (motion.py:636-642). So a dwell that is the last action before idle silently vanishes — G4, stepper-enable settle, sensor settles, and homing current-change settle all start the next motion with no real pause. Confirmed on the Trident bench (homing X had zero settle).

**Approach:** Represent the dwell as a **first-class committed idle segment** — a zero-motion `ShapedSegment` (empty/zero axes, no `PieceEntry` pushed) spanning `[T, T+N]`, reusing the engine's existing zero-motion segment support — emitted, committed, and dispatched like a move. The drain frontier must traverse it, so re-anchor/grounding can no longer erase it (durable by construction, the same way real moves are durable). Idle stops being "absence of work" and becomes drainable cargo; the underrun detector needs no new exemptions.

**Resolved (was Ask-First):** (1) the hold is a **pure time-occupier** (zero-motion segment, no motor drive), not constant-position NURBS — reusing existing zero-motion support, which also shrinks the v=0 audit to a *verification* that the existing support already covers the dwell path. (2) **No new wire format / MCU message** — zero-motion segment support already exists; a zero-axis segment occupies time and pushes no pieces (`dispatch_committed` already guards on `n_ax > 0`).

## Boundaries & Constraints

**Always:**
- The idle segment is half-open `[T, T+N)` committed cargo the drain frontier traverses; it advances the **successor start-time only**, never the **producer-delivery deadline**.
- Lateness stays judged on the true drain cursor: `PieceStartInPast` compares a piece's `intended_start` against the real drain cursor / `t_committed`, **never** a floored `max(esc, idle_floor)`. A real producer stall whose gap ≤ N must still raise at T+N.
- Fail loud (CLAUDE.md): the first uncovered tick (drain frontier past committed coverage with producer live and no committed successor) raises — do not pad. A dwell against an already-grounded timeline raises rather than silently advancing the clock.
- Every consumer that assumes non-zero velocity/displacement gets an audited zero-motion path; fail loud on any unhandled one (do not emit a silently-wrong profile).

**Ask First:**
- If the existing zero-motion segment support turns out NOT to flow through `commit` → `dispatch_committed` → the drain-frontier/coverage accounting (i.e. a zero-axis segment occupies planner time but the drain frontier does not actually traverse it) — that would reopen the segment-vs-scalar question; HALT and resurface before coding around it.

**Never:**
- Do not keep `dwell()` as a bare `advance_time` clock bump.
- Do not add a separate host-side `barrier()`/round-trip for the dwell case — the homing TMC write is already clock-scheduled (`set_register(.., print_time)`), so settle composes as write@T + idle[T,T+N] + home@T+N with no drain. (A host `wait_moves()` fence remains only for device→host data dependencies like endstop position — out of scope here.)
- Do not introduce a scalar `idle_floor` watermark with per-fault-site suppression guards (rejected: out-of-band state the host coverage oracle cannot observe without an MCU; rots as fault sites are added).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected | Error Handling |
|----------|--------------|----------|----------------|
| G4 mid-stream | move → dwell(N) → move | idle seg `[T,T+N)` committed; successor starts at T+N | N/A |
| Dwell before idle | dwell(N), no successor yet | idle seg committed; engine idle through T+N, then underrun rules apply | raise at first uncovered tick > T+N |
| Real stall, gap ≤ N | dwell(N); producer never delivers | drain traverses idle to T+N, no successor → **RAISE** PieceStartInPast @ T+N | raise (not swallowed) |
| Late move @ exactly T+N | successor committed before drain reaches T+N | proceed; starts at T+N | N/A |
| Late move committed after drain passed T+N | intended_start < drain cursor | **RAISE** | raise |
| Re-anchor / HomeDrip over a committed idle seg | reset fires while idle `[T,T+N)` committed | idle segment **survives**; no gap introduced | raise if a reset would drop committed coverage |
| Back-to-back dwell | dwell(N1) then dwell(N2) | single contiguous idle `[T, T+N1+N2)` | N/A |
| Zero / sub-tick dwell | N < one tick | defined no-op or 1-tick segment (pick, assert) | never a negative interval/gap |
| Homing current settle | write@T + dwell(N) + home_axis_start | home anchored ≥ T+N; current settles in real MCU time | raise if home would anchor < T+N |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/stream.rs:235` `advance_time` / `:247` `advance_idle` / `:259` `restart_idle_timeline` — today's scalar idle. Replace the dwell path with emission of a committed idle `ShapedSegment`.
- `trajectory/src/lib.rs:18` `ShapedSegment { axes, t_start, t_end, motor_mask, .. }` — the hold is a constant-position (or empty-axes) segment over `[T, T+N]`.
- `rust/motion-engine/src/stream.rs:324` `commit` — the idle segment flows through commit like a move (drain frontier traverses it).
- `rust/motion-engine/src/stream_planner.rs:623-636` `Dwell` handler — commit + dispatch an idle segment instead of `advance_time`.
- `rust/motion-engine/src/stream_planner.rs:575` reanchor & `:638-640` `HomeDrip` — must traverse/carry committed idle, not reset past it.
- `klippy/motion.py:636-642` `_ground_pending_end_time_after_engine_drain` — must not lower below committed idle; fail loud on a start behind the idle frontier.
- `rust/motion-engine/src/stream.rs` brake-to-rest / thin-lead / coverage paths — v=0 audit surface.

## Tasks & Acceptance

**Execution:**
- [ ] `rust/motion-engine/src/stream.rs` -- Add a `StreamState` method that emits a zero-motion `ShapedSegment` (`[T, T+N]`, no axes/pieces) as committed cargo, reusing the existing zero-motion support; confirm it flows `commit` (:324) → `dispatch_committed` → drain frontier. -- Idle becomes drainable work.
- [ ] `rust/motion-engine/src/stream_planner.rs:623-636` -- `Dwell` handler commits+dispatches the idle segment; drop the `advance_time` clock bump for the dwell path. -- Durable by construction.
- [ ] `rust/motion-engine/src/stream_planner.rs:575,638-640` -- reanchor and `HomeDrip` traverse/preserve committed idle instead of resetting past it; fail loud if a reset would drop committed coverage. -- Kills the erasure.
- [ ] `klippy/motion.py:636-642` -- grounding clamps no lower than the committed idle frontier; raise on a start behind it. -- Host side honors the segment.
- [ ] `rust/motion-engine/src/stream.rs` (brake-to-rest, thin-lead, coverage) -- VERIFY the existing zero-motion support already handles these against the hold segment; add explicit handling + fail-loud only on any path the existing support misses. -- De-risked v=0 surface.
- [ ] Tests (Rust `stream_planner`/`stream`, host `klippy/test/test_motion.py`) -- the coverage-and-raise oracle + the matrix edge cases. -- Gate.

**Acceptance Criteria:**
- Given a G4/dwell(N) as the last action before idle, when re-anchor / grounding / HomeDrip fire, then a committed idle interval ≥ N print-clock seconds survives between the prior motion's end and the next motion's start.
- Given a producer stall whose gap ≤ N, when the drain frontier reaches T+N with no committed successor, then `PieceStartInPast` raises (the dwell does not swallow the underrun).
- Given homing current-change, when `_set_homing_current` writes at T and dwells N, then `home_axis_start` anchors at ≥ T+N and the current settles in real MCU time.
- Given the host coverage-and-raise oracle over the committed stream, then `(committed_motion ∪ committed_idle)` covers `[stream_start, drain_frontier)` with no gap, and the first uncovered tick (if any) coincides exactly with a raised `PieceStartInPast` or end-of-stream brake.

## Design Notes

The single gating oracle (host-side, no MCU): commit-frontier coverage-and-raise consistency — gap-without-raise = swallow (fail), raise-without-gap = false scream (fail). It is *complete* under the committed-segment model because idle is in-band committed truth; a scalar watermark would be out-of-band and unobservable in CI — that is why the segment representation wins for an engine whose MCU path isn't in CI.

Bench-only (instrument, don't hide): physical stepper hold across `[T,T+N)`; TMC current electrically settled before the post-floor move; emit `floor_end − actual_delivery_time` as an event to mine the real producer-jitter margin.

Full rationale, the segment-vs-scalar decision (2–1), and the 7 chaos cases: see the linked brainstorming session doc.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p motion-engine` -- expected: green incl. new dwell/coverage tests.
- `./scripts/ci.sh py` -- expected: green incl. `test_motion.py` dwell durability + grounding tests.
- `./scripts/ci.sh rust-clippy && ./scripts/ci.sh rust-fmt && ./scripts/ci.sh rust-mcu-h7 && ./scripts/ci.sh rust-mcu-f4 && ./scripts/ci.sh rust-mcu-g0` -- expected: clean (touches motion-engine/trajectory MCU code).

**Manual (bench):** Trident — G4 P500 mid-print shows a real pause; homing X shows the current-change settle; `query-logs` shows `floor_end − actual_delivery_time` margins.

## Suggested Review Order

**The mechanism (the fix)**

- Entry point: dwell emits a durable held-position segment instead of a soft clock bump.
  [`stream.rs:247`](../../rust/motion-engine/src/stream.rs#L247)

- The held curve — constant monomial coeffs `[pos,0,0,0]` over `[T,T+N]`.
  [`stream.rs:795`](../../rust/motion-engine/src/stream.rs#L795)

- `Dwell` arm: commit the buffer, push the idle onto the batch, dispatch it; the `advance_time` bump is gone.
  [`stream_planner.rs:623`](../../rust/motion-engine/src/stream_planner.rs#L623)

**Host fail-loud guard**

- Grounding refuses to drop committed coverage (the durable-idle swallow).
  [`motion.py:641`](../../klippy/motion.py#L641)

**Durability + invariants (tests)**

- Idle survives a re-anchor byte-identical.
  [`stream_planner/tests.rs:279`](../../rust/motion-engine/src/stream_planner/tests.rs#L279)

- Idle survives a HomeDrip, stays ahead of the homing move (AC#3).
  [`stream_planner/tests.rs:388`](../../rust/motion-engine/src/stream_planner/tests.rs#L388)

- Sub-tick positive dwell emits an exact-duration segment, never collapsed.
  [`stream/tests.rs:979`](../../rust/motion-engine/src/stream/tests.rs#L979)

## Sign-off (accepted)

- AC#2 and the raise-half of AC#4 (stall gap ≤ N → `PieceStartInPast`) are enforced **MCU-side** (`runtime/motion_core.rs:127`), outside CI. Accepted as bench-only; swallow-prevention is upheld structurally host-side (no `idle_floor`; lateness on the true drain cursor).
- AC#3's explicit "raise if home anchors < T+N" is covered-by-construction (monotonic dispatch anchor + `wait_moves`) plus the `dwell_idle_survives_a_home_drip` durability test, rather than a redundant host raise.
