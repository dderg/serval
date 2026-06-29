---
stepsCompleted: [1, 2, 3]
status: complete
inputDocuments: ['_bmad-output/implementation-artifacts/investigations/homing-current-not-applied-investigation.md']
session_topic: 'Redesign planner.dwell() so a dwell is a durable, real pause for ALL consumers (G4, stall/sensor settle, homing current)'
session_goals: 'Diverge on mechanisms for a robust general-purpose dwell primitive that survives drain / idle-re-anchor / home-re-anchor; converge on an approach worth specifying'
selected_approach: 'ai-recommended'
techniques_used: ['First Principles Thinking', 'Morphological Analysis', 'Solution Matrix']
ideas_generated: []
context_file: ''
---

# Brainstorming Session Results

**Facilitator:** dderg
**Date:** 2026-06-28

## Session Overview

**Topic:** Redesign `planner.dwell()` so a dwell is a durable, real pause for ALL consumers — not just the homing-current case where it was noticed.

**Goals:** Diverge on mechanisms for a robust general-purpose dwell primitive that survives drain / idle-re-anchor / home-re-anchor, then converge on an approach worth turning into a spec.

### Problem Context (confirmed)

`planner.dwell()` expresses a pause as `state.advance_time(duration_s)` (stream.rs:235) — a soft bump of the planner's committed-time clock. That bump is durable only while the stream keeps flowing; it is silently erased whenever the timeline is grounded or re-anchored:
- `wait_moves()` → `_ground_pending_end_time_after_engine_drain` (motion.py:636-642) lowers `_mcu_pending_end_time`.
- idle re-anchor: `reanchor = state.is_empty() && esc > t_committed` (stream_planner.rs:575).
- `HomeDrip`: `state.reset(.., 0.0)` + `sync_instant = None/Instant::now()` (stream_planner.rs:638-640).

So dwell fails precisely when it is the **last action before the machine goes idle** — which is most consumers (settle-then-measure, settle-then-home, enable-then-move).

**Consumers (blast radius of any change):** G4 (`cmd_G4`), stepper_enable `DISABLE_STALL_TIME` (×6), sensor settles (`ldc1612`, `angle`, `resonance_tester`, eddy probe), `gcode.py:471`, homing current change.

### Session Setup

_(facilitator approach + technique selection pending)_

## Technique Selection

**Approach:** AI-Recommended Techniques

**Sequence:**
- **Phase 1 — First Principles Thinking:** redefine what a "dwell" is on this machine, below the `advance_time()` assumption.
- **Phase 2 — Morphological Analysis:** enumerate design axes (where the pause lives / what it waits on / what survives re-anchor / who triggers) and combine into candidate mechanisms.
- **Phase 3 — Solution Matrix:** score finalists against durability, MCU-clock correctness, throughput, blast radius, risk.

## Idea Generation

### Phase 1 — First Principles Thinking

**Bedrock established:**
- **#1 — What a dwell is:** an interval where the *motion system* is deliberately idle (toolhead stationary, steppers holding) while async housekeeping (PID, heaters, fans, host reactor) keeps running. Motion stops; the rest of the machine doesn't.
- **#2 — Flush vs idle are separable:** `planner.dwell()` already does two ops back-to-back — `flush(prior motion → dispatched)` then `insert-idle(N)`. The flush is durable (real segments dispatched); the idle is vapor (a soft clock bump). Only the second is broken.
- **Rejected framing:** "which clock (host / MCU-print / MCU-confirmed)" is the wrong first question. A dwell is a *happens-before barrier*, and the bug is representational — idle stored as mutable clock state any re-anchor may renegotiate — not a choice of clock.

### Phase 2/3 — Roundtable (party-mode: Winston, Amelia, Dr. Quinn, Murat)

**The system contradiction (Dr. Quinn, TRIZ):** the same gap of committed time is *stale time* to the throughput optimizer (re-anchor it away = win) and *intentional time* to the dwell (must elapse). One mechanism, two readers, opposite goals. Dwell idle is indistinguishable from accidental idle, so it gets eaten. Resolution: stop making dwell an *attribute of the clock*; make it an *object in the queue* — then it's no longer "empty" time the optimizer may reclaim.

## Converged Design

### Decision 1 — Two primitives, partitioned by one mechanical test
- **The test:** *can the side-effect be pre-timestamped onto the MCU clock?*
  - **Yes →** it joins the timeline; the settle is just a durable idle interval. **No barrier.** (Homing TMC current-change qualifies: `set_register("IHOLD_IRUN", val, print_time)` is already clock-scheduled. Winston conceded his "one true (iii) consumer" — it was never off-timeline.)
  - **No, and its runtime output conditions a later host command →** a host-side `wait_moves()`/drain barrier. This is a *data-dependency fence* (e.g. endstop trigger position flowing host-ward), orthogonal to dwell — NOT a competing dwell mode.

### Decision 2 — Durable dwell = first-class committed idle SEGMENT (not a scalar watermark)
- **Verdict:** (A) idle-as-real-segment, 2–1 (Winston + Murat over Amelia's scalar `idle_floor`).
- **Why A wins:** durability + fail-loudly become *structural*, not discipline-maintained. Idle stops being "absence of work" (which N fault sites must each be taught to exempt) and becomes drainable cargo the cursor traverses → the underrun detector needs **zero exemptions**, single-sourced. The segment lives in the ledger, so it survives re-anchor/grounding *for the same reason real moves do*.
- **Decisive for THIS engine:** MCU path isn't in CI. Murat's coverage-and-raise oracle can observe a swallow under (A) (idle is in-band committed truth) but **cannot** under (B) (scalar floor is out-of-band host intention — a floor-vs-true-drain divergence the gate can't see without an MCU).
- **Amelia's dissent → build constraint:** no `v=0` segment type exists today. An idle (zero-velocity/zero-displacement) segment is a new edge case for every consumer that assumes motion (junction-deviation, brake-to-rest, thin-lead drain, the drain-frontier walk). Each gets an audited idle path; fail-loud on any missed.

### Decision 3 — Deadline-vs-start-time (UNANIMOUS, the swallow-prevention invariant)
- The floor extends the **successor start-time ONLY**, never the **producer-delivery deadline**.
- Lateness rule: `PieceStartInPast` compares producer-stamped `intended_start` vs the **true drain cursor `esc`** — never the floored `max(esc, idle_floor)`. A real stall whose gap ≤ N still raises at T+N; the dwell buys queue depth, not slack.

### Fail-loud boundary — invariant + chaos cases (Murat)
- **Invariant:** every host-idle interval must be covered tick-for-tick by a commit-frontier entry (motion or idle); the first uncovered tick is a producer underrun and MUST raise. Half-open `[T, T+N)` with a hard right edge.
- **Gating oracle:** commit-frontier coverage-and-raise consistency — `(committed_motion ∪ committed_idle)` covers `[stream_start, drain_frontier)` with no gap, AND the first uncovered tick coincides exactly with a raised `PieceStartInPast` (or end-of-stream brake). Gap-without-raise = swallow (fail); raise-without-gap = false scream (fail). Pure host introspection, no MCU.
- **7 chaos cases (verdict / host assertion):** (1) healthy dwell + successor@T+N → proceed; (2) stall gap≤N hiding behind dwell → RAISE @T+N (swallow probe); (3a) late move @exactly T+N committed before drain → proceed; (3b) same committed after drain passed T+N → RAISE; (4) dwell→end-of-stream idle→brake→tardy past-start move → brake then RAISE, not re-stitched; (5) back-to-back G4 → single merged floor `[T, T+N1+N2)`; (6) reanchor/HomeDrip overlapping a committed floor → floor survives byte-identical; (7) zero/sub-tick dwell → defined no-op or 1-tick floor, never negative/gap. Cases 5/6/7 are the erasure-site regression surface.
- **Bench-only blind spots (instrument, don't hide):** physical stepper hold during the floor; TMC current *electrically* settled before the post-floor move; real producer-jitter tail — emit `floor_end − actual_delivery_time` as an event to mine the margin (don't assume N is enough).

### Code surface (today's erasure sites to convert/guard)
- `rust/motion-engine/src/stream.rs` — `advance_time` (235): source of the idle; emit/commit the idle segment here instead of a bare clock slide.
- `rust/motion-engine/src/stream_planner.rs:575` — idle re-anchor must traverse, not discard, committed idle.
- `rust/motion-engine/src/stream_planner.rs:638-640` — `HomeDrip` reset must carry the committed idle forward, not zero to now.
- `rust/motion-engine/src/stream_planner.rs:623-636` — `Dwell` handler: commit an idle segment, not `advance_time`.
- `klippy/motion.py:636-642` — host grounding must not lower below committed idle; fail-loud on a start behind the idle frontier.

### Homing settle composition (the original symptom, now derived)
`set_register(IHOLD_IRUN)@T` (clock-scheduled) → committed idle `[T, T+N]` → `home_axis_start` anchored at ≥ T+N. No barrier, no host round-trip. The HomeDrip re-anchor honoring committed idle is the fix.

**Status:** design converged. Session complete.
