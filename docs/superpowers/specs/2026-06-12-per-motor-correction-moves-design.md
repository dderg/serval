# Per-Motor Correction Moves — MCU/Host Contract

Date: 2026-06-12
Status: approved design, pre-implementation
Scope: the wire contract and MCU-side mechanism for moving one motor of a
multi-motor axis independently. Consumers (z_tilt, quad_gantry_level, AWD
motor sync) sit on top and are sketched here only to validate the contract.

## Problem

Several features need to move one motor of an axis relative to its siblings:

- `Z_TILT_ADJUST` / `QUAD_GANTRY_LEVEL`: after probing, each Z motor must move
  by its own delta (possibly several mm) to level the bed/gantry. Both are
  currently stubbed (`klippy/extras/z_tilt.py` `adjust_steppers()` raises
  "not yet implemented").
- AWD motor sync: measure and remove mechanical offset between two motors
  driving the same axis, at standstill.

The rewrite's motion pipeline is strictly per-axis: one piece ring of cubic
Bézier `PieceEntry`s per axis (`runtime/src/piece_ring.rs`), one step queue
per axis with a single direction bit (`runtime/src/step_queue.rs`), and every
stepper bound to an axis slot receives the same output. Same-axis sync is
by-construction — a property we keep. There is currently no wire-level way to
address an individual motor.

Motor types differ in positioning resolution: pulse-mode steppers move in
whole microsteps; phase-stepped motors (TMC XDIRECT coil drive) position at
sine-LUT resolution; servos take continuous setpoints. The contract must be
uniform across all three.

## Core decision

**Correction moves are regular pieces, routed to one motor.**

The host plans the adjustment move with the full planner — same trapezoid
generation, same discretization, same 32-byte `PieceEntry` Bézier format —
and pushes it over a new message that names a single motor. The MCU evaluates
the pieces with the polynomial evaluator it already runs and applies the
output to that one stepper. The MCU never computes velocity or acceleration;
it never learns a second way to move things.

Consequences that fall out of this:

- **No quantization semantics to define.** Pulse-mode steppers step a
  correction move exactly the way they step any move (nearest-microstep at
  each sample). Phase/servo apply it at their native resolution. Probe-driven
  consumers iterate (adjust → re-probe), so sub-microstep residue is
  re-measured and self-corrects; no host-side residual ledger.
- **No range limit.** A 10 mm correction is just a longer piece sequence.
- **No new message fields for motion shape.** Speed/accel live in the
  consumer's config (e.g. z_tilt's `speed`), expressed through the planned
  pieces, not through the wire.

### The detached frame ("move and forget")

Correction pieces are expressed in a **relative frame starting at 0**: a
10 mm adjustment is a piece sequence from position 0 to 10. The MCU evaluates
them into a scratch position that exists only for the duration of the batch.
The per-axis position tracker — and everything the host believes about axis
position — is never touched. The motor physically ends up somewhere else;
that is the entire point of leveling. Nothing is rebased and nothing is
tracked: the move is not part of the axis's story.

## Wire contract

New request message (and response), mirroring `PushPieces`
(`rust/kalico-protocol/src/messages.rs:175`):

```rust
pub struct PushCorrectionPieces {
    pub axis_idx: u8,      // 0-3, same slot numbering as PushPieces
    pub motor_idx: u8,     // index into the axis's bound steppers (0-3)
    pub piece_count: u8,
    pub start_slot: u16,
    pub new_head: u32,
    pub pieces_bytes: Vec<u8>,   // piece_count × 32-byte PieceEntry, frame-relative
}

pub struct PushCorrectionPiecesResponse {
    pub result: i32,             // 0 or negative error code
    pub arrival_clock: u64,
}
```

`PieceEntry` payload format and codec are shared with `PushPieces` byte-for-
byte (`start_time` in axis-MCU clock ticks, `coeffs: [f32; 4]` in mm —
relative frame, `duration` in seconds). `PushPieces` itself is untouched: no
new fields, no decode branches on the hot path.

A separate message (rather than a `motor_mask` field on `PushPieces`) was
chosen deliberately:

- The print-critical message stays byte-for-byte stable; the cold feature
  adds zero bytes and zero branches to the hot path.
- The two coordinate frames (absolute axis vs detached relative) are
  distinguished by message type, structurally — there is no field whose
  wrong value silently swaps frames.
- A mask cannot express the real need anyway (different motors need
  different deltas), so its apparent generality is fake; valid values would
  collapse to "all bits" or "one bit".

### Validation — all hard protocol errors (fail loudly)

On receiving `PushCorrectionPieces`, the MCU rejects with a distinct error
code when:

1. The axis has pending or active normal pieces (axis not idle).
2. A homing move is active on the axis.
3. Any correction stream is already active on this MCU, on any axis
   (initial policy: one correction stream at a time; streaming more pieces
   for the *same* active `(axis, motor)` stream is the one exception — that
   is how long moves arrive; see Future relaxations).
4. `motor_idx` is not a bound stepper on this axis.
5. `start_time` of the front piece is in the past (same rule as normal
   pieces — no padding, no advancing).

The inverse door is guarded too: **`PushPieces` arriving while a correction
stream is active is a hard error.** Both rejections emit structured log
events (`kalico_log_emit`).

### Completion

Deterministic, no completion event: the host planned the pieces, knows the
total duration, and waits past `start_time + Σ duration` before proceeding
(z_tilt: before re-probing). The MCU emits a structured log event when a
correction stream drains, for diagnostics and test assertions — observable,
not contractual.

## MCU-side mechanism

A small dedicated correction ring per axis (separate from the main piece
ring; depth modest, streamed like the main ring so move length is not capped
by ring depth), plus a scratch evaluation state:
`(active motor_idx, scratch last-position, current piece cursor)`.

Per motor type, the scratch position is applied the way that type already
applies position:

- **Pulse mode**: each sample, evaluate the correction polynomial → scratch
  position → step delta vs scratch last-position (same nearest-microstep
  rounding as normal dispatch) → pulses gated to the target stepper's
  step/dir GPIO only. Each bound stepper has its own `struct stepper`
  step/dir pins (`src/stepper.c` `runtime_motor_steppers[][]`), so
  single-motor gating is a routing decision, not new hardware handling. The
  per-axis step queue and axis position tracker are not involved. At stream
  end the scratch state is discarded.
- **Phase mode**: each sample, scratch position (mm → microsteps) is written
  into the target stepper's existing `phase_offset_microsteps`
  (`runtime/src/stepping_state.rs`); the polynomial is the ramp profile. At
  stream end the final value simply remains folded into the offset — the
  motor's coils stay where the move left them. `phase_offset_target` is set
  to match so the existing ramp logic (`dispatch_stepper.rs:254`) does not
  ramp it back.
- **Servo (future)**: scratch position added to the setpoint, folded at
  stream end. Same shape; no contract change.

C/Rust boundary: per `docs/kalico-rewrite/mcu-c-rust-boundary.md`, the
correction ring placement follows the same ownership rules as the existing
piece ring (C owns shared-memory placement; Rust owns the evaluation).

## Host side (consumers — sketch)

Bridge API: `adjust_motor(mcu, axis, motor_idx, delta_mm, speed, accel)` —
plans a trapezoid, discretizes to pieces in the relative frame, streams
`PushCorrectionPieces`, returns the completion host-time. The caller must
have quiesced the toolhead (flush + wait for axis idle) first; the MCU
enforces it regardless.

- **z_tilt / QGL**: probe → solve per-motor deltas (normalized by the
  consumer so the reference motor's delta is zero or deltas are zero-mean —
  consumer policy, invisible to the contract) → `adjust_motor` per motor,
  sequentially → wait → re-probe until converged. This replaces the
  `adjust_steppers()` stub.
- **AWD sync**: measure inter-motor offset (method out of scope here) →
  single `adjust_motor` on the lagging motor at standstill.

## Future relaxations (policy changes, zero contract change)

- Concurrent correction streams on different motors (parallel QGL
  adjustment) — lift validation rule 3, add per-stream scratch state.
- Live correction during motion for phase-stepped/servo motors (continuous
  AWD sync under feedback) — lift validation rule 1 for those motor types.
- `FORCE_MOVE` / `manual_stepper` — an offset move of one motor *is* a
  correction move; can be layered on this message.

## Testing

- `kalico-protocol`: encode/decode round-trip unit tests for the new
  messages (separate test file, per repo convention).
- `runtime`: unit tests for validation rejections (busy axis, bad motor_idx,
  overlap, stale start_time) and for scratch evaluation producing
  single-motor step output in pulse mode and folded offsets in phase mode.
- End-to-end: kalico-sim scenario — push correction pieces to an idle axis,
  assert only the target stepper's pin toggles and the axis tracker is
  unchanged; assert hard errors when the axis is moving.
- Bench, manual: a debug command ships with the bridge API, before any
  probe consumer —

  ```
  MOTOR_ADJUST AXIS=Z MOTOR=1 DELTA=2.0 [SPEED=5] [ACCEL=100]
  ```

  thin wrapper over `adjust_motor`; enables the motors if needed, requires
  an idle (not necessarily homed) axis. Bench script on the Trident
  (3-motor Z): `MOTOR_ADJUST ... DELTA=2.0` → exactly one leadscrew turns,
  the other two stay still, Mainsail's reported Z position does not change;
  `DELTA=-2.0` returns it. Error paths: issue it mid-move → hard error;
  bad MOTOR index → hard error. Cross-check via query-logs that the
  correction-stream start/drain events fired and only the target stepper
  was driven.
- Bench: Trident `Z_TILT_ADJUST` convergence once the probe consumer lands.
