# Correction Streams — Multi-Segment Sequences & Self-Sequencing

Date: 2026-06-14
Status: design, pending review
Builds on: `2026-06-12-per-motor-correction-moves-design.md` (the
`PushCorrectionPieces` wire contract and MCU mechanism). This spec adds the
host-side mechanics a sequence-of-moves consumer needs, without changing that
contract.

## Why

The per-motor correction path exists and works for the z_tilt consumer: one
`adjust_motor(mcu, axis, motor, delta, speed, accel)` call plans a single
0→delta trapezoid and waits it out. The next consumer — AWD motor sync
(motors-sync) — drives a motor very differently:

- a **buzz**: ~25 fading oscillations on one motor to excite the belt, then
  measure;
- **repeated single-microstep nudges** during a measurement sweep;
- a final **correction nudge** to remove the offset.

Expressed through today's `adjust_motor`, each swing/nudge is an independent
call: it starts at `now + 0.15 s` lead, blocks on an MCU ack, and the caller
must wait its full duration before the next. A 50-segment buzz becomes 50
serialized point-to-point moves with ~7.5 s of pure lead and a hard stop
between every swing — not a shake. And the caller ends up doing print-time
bookkeeping (`get_last_move_time`, dwell math) to avoid overlap, which means
the plugin is reaching into our scheduling internals.

Both problems are host-side. The wire contract and MCU evaluator already do
the right thing; what's missing is a host that can express a **contiguous
multi-segment correction** and own its scheduling.

## Non-goals / decisions

- **Do not unify the correction and main pipelines.** On the MCU, corrections
  are already a separate stream gated by a cheap `correction_active()` flag,
  single-motor, with their own ring and scratch position — they never sum
  with live axis motion (consumers correct while the axis is idle). Merging
  would force a per-step branch in the step ISR to ask "whole axis or one
  motor?" — paying hot-path cost, on the code CLAUDE.md says never to slow,
  for a calibration feature. Rejected.
- **No new wire fields, no MCU evaluator change.** A contiguous sequence is
  just more `PieceEntry`s in the existing relative frame on the existing
  stream. The MCU already evaluates streamed pieces and gates them to one
  motor.
- **No planner for corrections.** Speed/accel stay caller-supplied and
  trusted (clamped to the axis machine max as a safety floor/ceiling). These
  are slow microstep nudges; the SOTA planner is neither needed nor wanted
  here.
- **No velocity-blending across joints (yet).** The buzz's joints are
  direction reversals (velocity zero by construction), so gapless trapezoids
  already produce a clean oscillation. Blending across non-reversing joints
  is a future refinement, not needed by any current consumer.

## Design

Two host-side additions in `rust/motion-bridge`, plus finishing the
already-designed streaming on the bridge.

### 1. Multi-segment profile builder

Generalize `correction::plan_correction_profile` from "one trapezoid for one
delta" to "a contiguous piece sequence for a list of relative segments":

```
plan_correction_sequence(segments: &[f64], speed, accel) -> Vec<ProfilePiece>
```

Each segment is one trapezoid (the existing `push_quadratic`/`push_linear`
builder), emitted **end-to-end with no time gap** — segment k+1's first piece
starts exactly where segment k's last piece ends. Sub-`epsilon` segments are
dropped. The single-delta case (`segments = [delta]`) is byte-identical to
today's output, so the z_tilt consumer is unaffected.

**Load-bearing invariant: zero gap between segments within a stream.** A buzz
with any pause between swings is not a buzz — the excitation breaks. This is
the whole reason the sequence is submitted as one stream rather than N calls,
and it is asserted directly in tests (contiguous piece times) and on the
bench (no audible/measurable inter-swing pause).

The buzz and the nudge-sweep are both just `segments` lists the consumer
hands in; the builder has no buzz-specific knowledge.

### 2. Stream submission that owns scheduling

A bridge entry point that submits a whole sequence as **one correction
stream**:

```
submit_correction(mcu, axis, motor, segments, speed, accel) -> duration_secs
```

- Picks the stream start once (`now + lead`), not per segment.
- Pushes pieces up to the MCU correction-ring capacity, then **refills as the
  ring drains** — the streaming the 06-12 spec already specified ("streamed
  like the main ring so move length is not capped by ring depth") and that
  the contract already carries via `start_slot`/`new_head`. Today's
  `adjust_motor` pushes every chunk up front in one synchronous burst, which
  only works because z_tilt moves fit the ring; a 50-segment buzz does not.
  This is the one piece of real new plumbing: drive refill off the ring head
  the same way the main piece ring is fed.
- Returns the total duration. The caller waits that out (a legitimate
  "wait for the move to finish", like `wait_moves`) — but never computes
  per-segment start times. No scheduling internals cross the boundary.

`adjust_motor` becomes the one-segment wrapper over `submit_correction`,
preserving the existing `MOTOR_ADJUST`/z_tilt behavior.

### What the consumer sees

motors-sync (and any future caller) gets: enable/disable a motor (exists),
and "run this sequence of relative moves on this motor at this speed/accel,
tell me when it's done." It never sees a clock, a ring, or a lead time.

## Validation / fail-loud

Unchanged from the 06-12 contract — axis-idle, motor bound, no stale
`start_time`, one stream at a time (with same-stream refill as the streaming
exception). The refill loop must surface a ring-overflow or a refill-behind
(host fell behind the drain) as a hard structured error, not silent drops —
same posture as the main ring.

## Testing

- `motion-bridge` unit tests (separate file): `plan_correction_sequence`
  produces contiguous pieces (segment k+1 start == segment k end, no gap);
  single-segment output equals the current `plan_correction_profile`;
  sub-epsilon segments dropped; an oscillation list yields alternating-sign
  pieces meeting at zero-velocity turning points.
- `runtime`: streaming refill of a sequence longer than the correction ring
  drains correctly and gates to the one motor (extends the 06-12 scratch /
  single-motor tests).
- kalico-sim: push a multi-segment sequence to an idle axis; assert only the
  target stepper toggles, the axis tracker is unchanged, and the pieces are
  contiguous in time; assert hard error if the host refill falls behind.
- Bench: a long buzz on one belt motor runs as one continuous shake (no
  inter-swing pauses); the partner motor stays put; reported axis position
  unchanged.
```
