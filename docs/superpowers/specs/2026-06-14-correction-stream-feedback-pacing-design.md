# Correction Stream — Feedback-Paced on Shared Ring Machinery

Date: 2026-06-14
Status: design, pending review
Builds on: `2026-06-12-per-motor-correction-moves-design.md` (the
`PushCorrectionPieces` wire + MCU mechanism) and
`2026-06-14-correction-stream-sequences-design.md` (the host sequence builder).
Supersedes the host-side **pacing** introduced there (the wall-clock refill loop
in `stream_correction_entries`).

## Why

The correction stream re-implements the main ring's job — stream cubic-Bézier
pieces to a depth-bounded MCU ring, refilling as it drains — but does it with a
bespoke, inferior mechanism. The main ring paces refill on real MCU **drain
feedback** (`DrainSync`, fed by the heartbeat's `retired_counts`); the correction
stream instead *guesses* from a host wall-clock model (`piece_end_host` +
margin). On a fast buzz (many short pieces, chunk size 15 ≈ ring depth 16, a
one-slot refill window), that guess overcommits the ring and the MCU rejects
the push with `-309 = KALICO_ERR_RING_FULL` (`engine.rs:commit_correction`,
`new_head - retired > ring_depth`). Observed on the Trident: `buzz_move(25)`
aborts the sync with `-309`.

It is two implementations of one idea, and the worse one is the one in
production for corrections. This unifies the **behavior** onto the main ring's
feedback-paced machinery while keeping the two streams **stored separately**.

## Non-goals / decisions

- **Seam: share host pacing + drain feedback only — not the MCU evaluator or
  the wire.** The correction ring keeps its own per-axis evaluation
  (`dispatch_correction::tick_correction`, already a separate path at
  `engine.rs:416`), its `correction_active()` gate, single-motor stepping, and
  the `PushCorrectionPieces` message. The step hot path (`get_position_and_velocity`
  / dispatch) is untouched. (Rejected: folding corrections into the main ring
  with a per-piece motor-filter — it taxes the sacred step eval and spends the
  `PieceEntry._reserved` ABI word, across the whole ~1984-slot main ring, on a
  calibration-only flag.)
- **Share behavior, separate data.** Reuse the *algorithms* (host feedback-paced
  streaming, `DrainSync`, and the already-shared `RingDescriptor` bookkeeping);
  keep separate ring instances, separate stream-state structs (the correction
  ring already carries its own `correction_motor_idx` / `correction_armed` /
  `correction_p_prev`), a separate storage region, and a separate `DrainSync`
  instance. Nothing merges into one ring or one piece struct.
- **No printing-hot-path cost.** Drain feedback rides the existing periodic
  heartbeat, not the step ISR. The correction evaluator already runs per-tick
  today, gated by `correction_active()`. This change adds no per-step work.

## Design

### 1. Host pacing — reuse the feedback streamer

Delete the wall-clock refill loop in `stream_correction_entries`
(`bridge.rs:3805-3848`: the `chunk_release_times` / `piece_end_host` /
`std::thread::sleep` polling). Drive the correction stream through the same
feedback-paced refill the main ring uses (`pump.rs` `AxisQueue`/`schedule` +
`DrainSync`): push the next `PushCorrectionPieces` chunk **only while
`DrainSync` reports `room()`**, refill as `retired` advances. Because the host
only sends into known-free slots, it **cannot** overcommit — `-309` is
eliminated by construction, not by a tighter guess.

The shared streaming logic is generic over the message type (`PushPieces` vs
`PushCorrectionPieces`) and the ring/queue identity. The correction stream
instantiates it with its own queue, its own `DrainSync` instance, and the
`PushCorrectionPieces` encoder; the main ring is unchanged.

The stream still picks its start once (`now + CORRECTION_LEAD_SECS`); only the
*refill* moves from wall-clock to feedback.

### 2. Drain feedback (B) — correction `retired` on the heartbeat, gated

The MCU heartbeat already reports the main ring's per-axis `retired_counts`
(`engine.rs:495` → `bridge.rs:2645-2649` → `DrainSync.set_retired`). Extend the
heartbeat to also carry the **correction ring's** per-axis `retired`. The host
feeds it into a **separate `DrainSync` instance** for corrections, so the main
and correction `retired` for the same `(mcu, axis)` never collide.

**Cadence gate (load-bearing).** The heartbeat's adaptive "report faster when a
ring is near-empty" signal must consider the correction ring **only when
`correction_active()`**. During a print, corrections are inactive, so the
(empty, idle) correction ring contributes nothing to the cadence decision and
the print-time heartbeat rate is unchanged. During a buzz,
`correction_active()` is true, so the draining correction ring drives the
cadence up exactly when tight feedback is needed. This is the one new condition
on the MCU heartbeat and is the only print-time-live surface this change adds
to.

### 3. Depth (C)

Raise `CORRECTION_RING_DEPTH` from 16 to **64** (`stepping_state.rs:14`).
~64 × 32 B = 2 KB per axis, comfortably inside the shared ~1984-slot
`PieceEntry` budget (`engine.rs:170`). Rationale: with feedback pacing, the ring
must hold more than one heartbeat-feedback interval's worth of pieces to stay
gapless on a fast buzz; 64 gives headroom over the old 16's one-slot window.
The exact value is pinned in the plan against the measured heartbeat interval.

### 4. Non-regression guard

With no correction active, the main ring's `pump` / `DrainSync` / heartbeat
behavior — including cadence — must be byte-identical to today. The correction
path plugs in additively (its own queue, `DrainSync` instance, message, and the
`correction_active`-gated cadence term). A test asserts the main-ring pacing and
the idle-correction-ring heartbeat cadence are unchanged.

### Unchanged (kept lean / hot-path-safe)

`PieceEntry` (no new field, `_reserved` stays free), the MCU correction
evaluator (`tick_correction`), the `correction_active()` gate, single-motor
stepping, and the `PushCorrectionPieces` wire message.

## Validation / fail-loud

- The feedback streamer never sends beyond `DrainSync.room()`; a computed
  overcommit is a hard internal error, not a silent drop.
- A correction stream whose drain stalls (MCU not retiring) surfaces as a
  hard timeout/error after a bounded wait — same posture as the main ring.
- Unchanged correction validations (axis-idle, motor bound, no stale start).

## Testing

- **`pump` / streamer unit tests:** the shared feedback-paced refill, driven by
  a fake `DrainSync`, never exceeds `room()` for the correction queue; a long
  piece list streams to completion as `retired` advances; refill stops when
  drain stalls.
- **Heartbeat cadence unit test:** an idle (empty, `!correction_active`)
  correction ring does not change the heartbeat cadence vs main-ring-only; an
  active draining correction ring does raise it.
- **`DrainSync` (correction instance):** `sent`/`retired`/`baseline` track
  across multiple back-to-back streams (the buzz then nudge sequence).
- **runtime/MCU:** the correction ring's `retired` is reported in the heartbeat
  and matches the evaluator's drain.
- **kalico-sim:** a `buzz_move(25)`-length sequence streams to completion with
  no `RING_FULL`; only the target motor toggles; axis position unchanged.
- **Bench (later, with explicit go-ahead):** the buzz runs gapless, the partner
  motor stays put, and a sync run converges.

## Risks

- **Genericizing `pump` without disturbing the main ring.** The refactor must be
  additive; the non-regression guard (§4) is the primary defense. If `pump` is
  too entangled with main-ring specifics to share cleanly, fall back to a thin
  shared helper extracted from it rather than parameterizing the whole scheduler.
- **Heartbeat cadence math.** The exact MCU cadence change is pinned in the plan
  against the real cadence logic; the gate must be verified to leave the
  print-time path measurably unchanged.
