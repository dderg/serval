# Per-Motor Correction Moves Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the spec in `docs/superpowers/specs/2026-06-12-per-motor-correction-moves-design.md` — host-planned Bézier "correction pieces" delivered over a new `PushCorrectionPieces` message and evaluated by the MCU against a single stepper of a multi-stepper axis, plus the `MOTOR_ADJUST` debug command and the z_tilt/QGL consumer.

**Architecture:** A small per-axis correction ring (depth 16) sits next to the main piece ring inside the existing `rt_storage` piece arena. The TIM5 tick evaluates the correction stream with the same `get_position_and_velocity` machinery in a detached relative frame; pulse mode routes the resulting steps to one stepper via a `stepper_sel` byte carried in the (currently padded) `StepEntry`, phase mode drives the target stepper's existing `phase_offset_target`. The host plans the trapezoid as exact cubic Bernstein pieces in `motion-bridge` and streams them on the control channel.

**Tech Stack:** Rust (`kalico-protocol`, `runtime`, `kalico-c-api`, `motion-bridge`), C (`src/kalico_dispatch.c`, `src/stepper.c`), Python (klippy).

**Conventions that apply to every task:**
- Run Rust tests with `cargo nextest run` from `rust/` (never bare `cargo test`); scope with `-p <crate>` or `-E 'test(<name>)'`.
- Unit tests live in a separate file from the tested code (existing pattern: `mod tests;` at the bottom pointing to `tests.rs` in a subdirectory or sibling file — follow whichever the touched crate already uses).
- No explanatory comments; encode rationale in names/asserts. TODO markers allowed.
- Fail loudly: all rejections are distinct error codes, never silent recovery.
- The schema hash changes in Task 1 — after this branch, **both bench MCUs must be reflashed together** (H7 from `.config.h7.bak`, F446 from `.config.f446.test`, `make clean` between builds, build on the Pi after push/pull — never scp binaries).
- `cargo fmt --all --check` from `rust/` is the last step before any push.

---

### Task 1: Protocol messages — `PushCorrectionPieces` / response

**Files:**
- Modify: `rust/kalico-protocol/schema_def.rs` (append after the `PushPiecesResponse` entry)
- Modify: `rust/kalico-protocol/src/messages.rs`
- Test: `rust/kalico-protocol/tests/` (alongside the existing round-trip tests; check `rust/kalico-protocol/tests/` and `src/messages/tests.rs` for where PushPieces round-trips live and add next to them)

- [ ] **Step 1: Write the failing round-trip test**

Locate the existing `PushPieces` encode/decode round-trip test (`grep -rn "PushPieces" rust/kalico-protocol --include="*test*"`) and add in the same file:

```rust
#[test]
fn push_correction_pieces_roundtrip() {
    let msg = PushCorrectionPieces {
        axis_idx: 2,
        motor_idx: 1,
        piece_count: 2,
        start_slot: 5,
        new_head: 7,
        pieces_bytes: vec![0xAB; 64],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    let decoded = PushCorrectionPieces::decode(&buf).unwrap();
    assert_eq!(decoded, msg);
}

#[test]
fn push_correction_pieces_response_roundtrip() {
    let msg = PushCorrectionPiecesResponse {
        result: -31,
        arrival_clock: 0x1122_3344_5566_7788,
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    let decoded = PushCorrectionPiecesResponse::decode(&buf).unwrap();
    assert_eq!(decoded, msg);
}

#[test]
fn push_correction_pieces_rejects_short_body() {
    let msg = PushCorrectionPieces {
        axis_idx: 2,
        motor_idx: 1,
        piece_count: 3,
        start_slot: 0,
        new_head: 3,
        pieces_bytes: vec![0; 64],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert!(PushCorrectionPieces::decode(&buf).is_err());
}
```

(If the existing tests use a `decode(&buf)` helper vs `Decode::decode_from(&mut Cursor...)`, match that idiom exactly.)

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p kalico-protocol -E 'test(push_correction)'`
Expected: compile error — `PushCorrectionPieces` not found.

- [ ] **Step 3: Implement**

In `rust/kalico-protocol/src/messages.rs`:

Add to `MessageKind` (enum, `from_u16`, keeping ascending order — 0x0062/0x0063 sit between `PushPiecesResponse` and `StartCapture`):

```rust
    PushCorrectionPieces = 0x0062,
    PushCorrectionPiecesResponse = 0x0063,
```

```rust
            0x0062 => Self::PushCorrectionPieces,
            0x0063 => Self::PushCorrectionPiecesResponse,
```

Add the structs, mirroring `PushPieces`'s codec exactly (including the `checked_mul(32)` length validation):

```rust
#[derive(Debug, Clone, PartialEq)]
pub struct PushCorrectionPieces {
    pub axis_idx: u8,
    pub motor_idx: u8,
    pub piece_count: u8,
    pub start_slot: u16,
    pub new_head: u32,
    pub pieces_bytes: Vec<u8>,
}

impl Encode for PushCorrectionPieces {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.axis_idx);
        put_u8(out, self.motor_idx);
        put_u8(out, self.piece_count);
        put_u16(out, self.start_slot);
        put_u32(out, self.new_head);
        out.extend_from_slice(&self.pieces_bytes);
    }
}

impl Decode for PushCorrectionPieces {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let axis_idx = get_u8(c)?;
        let motor_idx = get_u8(c)?;
        let piece_count = get_u8(c)?;
        let start_slot = get_u16(c)?;
        let new_head = get_u32(c)?;
        let pieces_len = (piece_count as usize).checked_mul(32).ok_or(
            DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(piece_count),
                available: c.remaining(),
            },
        )?;
        if pieces_len > c.remaining() {
            return Err(DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(piece_count),
                available: c.remaining(),
            });
        }
        let mut pieces_bytes = vec![0u8; pieces_len];
        for b in &mut pieces_bytes {
            *b = get_u8(c)?;
        }
        Ok(Self { axis_idx, motor_idx, piece_count, start_slot, new_head, pieces_bytes })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PushCorrectionPiecesResponse {
    pub result: i32,
    pub arrival_clock: u64,
}

impl Encode for PushCorrectionPiecesResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u64(out, self.arrival_clock);
    }
}

impl Decode for PushCorrectionPiecesResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            arrival_clock: get_u64(c)?,
        })
    }
}
```

In `rust/kalico-protocol/schema_def.rs`, append to `SCHEMA_MESSAGES` in ascending tag order (after the 0x0061 entry):

```rust
    SchemaMessage {
        type_tag: 0x0062,
        name: "PushCorrectionPieces",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "axis_idx", ty: "u8" },
            SchemaField { name: "motor_idx", ty: "u8" },
            SchemaField { name: "piece_count", ty: "u8" },
            SchemaField { name: "start_slot", ty: "u16" },
            SchemaField { name: "new_head", ty: "u32" },
            SchemaField { name: "pieces_bytes", ty: "array<u8>" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0063,
        name: "PushCorrectionPiecesResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "arrival_clock", ty: "u64" },
        ],
    },
```

The build script regenerates `src/kalico_protocol_schema.h` (new `KALICO_MSG_PUSH_CORRECTION_PIECES` / `..._RESPONSE` defines + new `KALICO_SCHEMA_HASH`). There is a schema-hash-change integration test (`rust/kalico-protocol/tests/schema_hash_change.rs`) — read it and update its expected hash/fixture per its own instructions.

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo nextest run -p kalico-protocol`
Expected: all pass, including the schema-hash test after its fixture update. Verify `git diff src/kalico_protocol_schema.h` shows the two new defines and a changed hash.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-protocol src/kalico_protocol_schema.h
git commit -m "feat(protocol): PushCorrectionPieces message pair"
```

---

### Task 2: Runtime — correction ring state and validated commit

**Files:**
- Modify: `rust/runtime/src/stepping_state.rs`
- Modify: `rust/runtime/src/engine.rs`
- Modify: `rust/runtime/src/error.rs` (one new code)
- Test: `rust/runtime/src/engine/tests.rs` (or wherever `mod tests` for engine.rs resolves — `grep -n "mod tests" rust/runtime/src/engine.rs`)

- [ ] **Step 1: Write the failing tests**

```rust
fn engine_with_z_axis(mode: StepMode) -> (Engine, Vec<PieceEntry>) {
    let mut engine = Engine::default();
    let storage = vec![
        PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 };
        runtime::state::TOTAL_RING_PIECES
    ];
    let bindings = [
        StepperBindingRust { stepper_oid: 10, tmc_cs_oid: TMC_CS_OID_NONE, _pad: [0; 2] },
        StepperBindingRust { stepper_oid: 11, tmc_cs_oid: TMC_CS_OID_NONE, _pad: [0; 2] },
        StepperBindingRust { stepper_oid: 12, tmc_cs_oid: TMC_CS_OID_NONE, _pad: [0; 2] },
    ];
    let rc = engine.configure_axis(2, mode, 0.00125, 64, &bindings, runtime::state::TOTAL_RING_PIECES);
    assert_eq!(rc, KALICO_OK);
    (engine, storage)
}

fn one_piece(start_time: u64) -> PieceEntry {
    PieceEntry { start_time, coeffs: [0.0, 0.5, 1.0, 1.5], duration: 0.5, _reserved: 0 }
}

#[test]
fn configure_axis_allocates_correction_ring() {
    let (engine, _) = engine_with_z_axis(StepMode::Pulse);
    let axis = engine.stepping_axes[2].as_ref().unwrap();
    assert_eq!(axis.correction_ring.ring_depth, CORRECTION_RING_DEPTH);
    assert!(axis.correction_ring.ring_offset >= axis.ring.ring_offset + axis.ring.ring_depth);
}

#[test]
fn commit_correction_rejects_bad_motor_idx() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    assert_eq!(
        engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage),
        KALICO_OK
    );
    assert_eq!(engine.commit_correction(2, 3, 1), KALICO_ERR_INVALID_ARG);
}

#[test]
fn commit_correction_rejects_busy_axis() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    assert_eq!(engine.push_pieces(2, &[one_piece(1000)], &mut storage), KALICO_OK);
    engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 1), KALICO_ERR_MOTION_IN_PROGRESS);
}

#[test]
fn commit_correction_rejects_second_stream_other_motor() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 1), KALICO_OK);
    engine.write_correction_piece(2, 1, 0, one_piece(2000), &mut storage);
    assert_eq!(engine.commit_correction(2, 2, 2), KALICO_ERR_MOTION_IN_PROGRESS);
}

#[test]
fn commit_correction_allows_streaming_same_motor() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 1), KALICO_OK);
    engine.write_correction_piece(2, 1, 0, one_piece(2000), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 2), KALICO_OK);
}

#[test]
fn normal_commit_rejected_while_correction_active() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 1), KALICO_OK);
    assert_eq!(engine.guard_normal_commit(2), KALICO_ERR_CORRECTION_IN_PROGRESS);
}
```

Adjust the `engine_with_z_axis` helper to whatever helper idiom the existing engine tests already use (imports, `Engine::default()` availability).

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p runtime -E 'test(correction)'`
Expected: compile errors — `correction_ring`, `commit_correction` etc. not found.

- [ ] **Step 3: Implement**

`rust/runtime/src/error.rs` — append after `KALICO_ERR_PHASE_MOTOR_UNMAPPED` (next free in the -3xx block; also add the matching `FaultCode` variant + `name()` arm following the file's pattern):

```rust
/// Normal PushPieces commit attempted while a correction stream is active
/// on the axis, or a correction commit attempted while one is already
/// active elsewhere. Detail: `((axis_idx & 0xFF) << 16) | motor_idx`.
pub const KALICO_ERR_CORRECTION_IN_PROGRESS: i32 = -314;
```

(Reuse `KALICO_ERR_MOTION_IN_PROGRESS` (-31) for "axis has pending/active normal pieces"; the new -314 is the inverse door and the stream-overlap case.)

`rust/runtime/src/stepping_state.rs`:

```rust
pub const CORRECTION_RING_DEPTH: usize = 16;
pub const CORRECTION_MOTOR_NONE: u8 = 0xFF;
```

Extend `AxisState` (and `new_unconfigured`, keeping `const fn`):

```rust
    pub correction_ring: RingDescriptor,
    pub correction_armed: Option<ArmedPiece>,
    pub correction_motor_idx: u8,
    pub correction_last_step_count: i32,
    pub correction_p_prev: f32,
```

```rust
            correction_ring: RingDescriptor::new_unconfigured(),
            correction_armed: None,
            correction_motor_idx: CORRECTION_MOTOR_NONE,
            correction_last_step_count: 0,
            correction_p_prev: 0.0,
```

```rust
impl AxisState {
    pub fn correction_active(&self) -> bool {
        !self.correction_ring.is_empty() || self.correction_armed.is_some()
    }
}
```

Also extend `reset_isr_cache()` to clear `correction_armed = None; correction_last_step_count = 0; correction_p_prev = 0.0;` and reset `correction_motor_idx = CORRECTION_MOTOR_NONE;`, and make `Engine::discard_pending` / `runtime_force_idle` / `seed_position` drain `correction_ring` the same way they drain `ring`.

`rust/runtime/src/engine.rs` — in `configure_axis`, after the main ring allocation, allocate the correction ring from the same cursor (the budget check covers both):

```rust
        if self.ring_alloc_cursor + ring_depth + crate::stepping_state::CORRECTION_RING_DEPTH
            > total_ring_pieces
        {
            return KALICO_ERR_RING_FULL;
        }

        let offset = self.ring_alloc_cursor;
        self.ring_alloc_cursor += ring_depth;
        let correction_offset = self.ring_alloc_cursor;
        self.ring_alloc_cursor += crate::stepping_state::CORRECTION_RING_DEPTH;
```

and after `axis.ring = ...`:

```rust
        axis.correction_ring = crate::piece_ring::RingDescriptor::new(
            correction_offset,
            crate::stepping_state::CORRECTION_RING_DEPTH,
        );
```

New engine methods:

```rust
    pub fn write_correction_piece(
        &mut self,
        axis_idx: u8,
        start_slot: u16,
        index: u8,
        entry: PieceEntry,
        storage: &mut [PieceEntry],
    ) -> i32 {
        let Some(axis) = self
            .stepping_axes
            .get_mut(axis_idx as usize)
            .and_then(|s| s.as_mut())
        else {
            return KALICO_ERR_INVALID_ARG;
        };
        if !axis.correction_ring.is_configured() {
            return KALICO_ERR_INVALID_ARG;
        }
        let slot = (start_slot as usize + index as usize) % axis.correction_ring.ring_depth;
        axis.correction_ring.write_slot(storage, slot, entry);
        KALICO_OK
    }

    pub fn commit_correction(&mut self, axis_idx: u8, motor_idx: u8, new_head: u32) -> i32 {
        use crate::stepping_state::CORRECTION_MOTOR_NONE;
        let any_other_active = self.stepping_axes.iter().enumerate().any(|(i, a)| {
            i != axis_idx as usize && a.as_ref().map_or(false, |ax| ax.correction_active())
        });
        let Some(axis) = self
            .stepping_axes
            .get_mut(axis_idx as usize)
            .and_then(|s| s.as_mut())
        else {
            return KALICO_ERR_INVALID_ARG;
        };
        if !axis.correction_ring.is_configured() {
            return KALICO_ERR_INVALID_ARG;
        }
        if (motor_idx as usize) >= axis.steppers.len() {
            return KALICO_ERR_INVALID_ARG;
        }
        if !axis.ring.is_empty() || axis.armed.is_some() {
            return crate::error::KALICO_ERR_MOTION_IN_PROGRESS;
        }
        if any_other_active {
            return crate::error::KALICO_ERR_CORRECTION_IN_PROGRESS;
        }
        if axis.correction_active() && axis.correction_motor_idx != motor_idx {
            return crate::error::KALICO_ERR_CORRECTION_IN_PROGRESS;
        }
        if !axis.correction_active() {
            axis.correction_motor_idx = motor_idx;
            axis.correction_last_step_count = 0;
            axis.correction_p_prev = 0.0;
        }
        match axis.correction_ring.commit_head(new_head) {
            crate::piece_ring::CommitOutcome::Applied
            | crate::piece_ring::CommitOutcome::Stale => KALICO_OK,
            crate::piece_ring::CommitOutcome::Overcommit => KALICO_ERR_RING_FULL,
        }
    }

    pub fn guard_normal_commit(&self, axis_idx: u8) -> i32 {
        let active = self
            .stepping_axes
            .get(axis_idx as usize)
            .and_then(|s| s.as_ref())
            .map_or(false, |ax| ax.correction_active());
        if active {
            crate::error::KALICO_ERR_CORRECTION_IN_PROGRESS
        } else {
            KALICO_OK
        }
    }
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo nextest run -p runtime`
Expected: new tests pass, no regressions (existing engine tests must still pass with the larger per-axis allocation — if any existing test configures rings that now exceed `TOTAL_RING_PIECES`, shrink its `ring_depth`, not the budget check).

- [ ] **Step 5: Commit**

```bash
git add rust/runtime
git commit -m "feat(runtime): per-axis correction ring with validated commit"
```

---

### Task 3: Runtime — `StepEntry.stepper_sel` and single-stepper pulse routing

**Files:**
- Modify: `rust/runtime/src/step_queue.rs`
- Modify: `rust/runtime/src/per_axis_timer.rs`
- Modify: `src/stepper.c` (`runtime_emit_step_pulses`) and its prototype (find it: `grep -rn "runtime_emit_step_pulses" src/`)
- Test: existing step_queue / per_axis_timer test files (find via `grep -rn "mod tests" rust/runtime/src/step_queue.rs rust/runtime/src/per_axis_timer.rs`)

- [ ] **Step 1: Write the failing test**

In the step_queue test file:

```rust
#[test]
fn step_entry_carries_stepper_sel() {
    let entry = StepEntry { cycle_abs: 100, dir: 1, stepper_sel: 3, _pad: [0; 2] };
    assert_eq!(core::mem::size_of::<StepEntry>(), 8);
    assert_eq!(entry.stepper_sel, 3);
}
```

In the per_axis_timer test file, locate the existing test that pushes an entry and asserts `runtime_emit_step_pulses` was invoked via the test hook (`grep -n "emit" rust/runtime/src/per_axis_timer*`), and extend the recorded-call assertion to include the sel value (extend the test hook signature in the same change).

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p runtime -E 'test(stepper_sel)'`
Expected: compile error — no field `stepper_sel`.

- [ ] **Step 3: Implement**

`rust/runtime/src/step_queue.rs` — `StepEntry` becomes (size stays 8, C mirror unchanged in size):

```rust
#[repr(C)]
pub struct StepEntry {
    pub cycle_abs: u32,
    pub dir: i8,
    pub stepper_sel: u8,
    pub _pad: [u8; 2],
}
```

`stepper_sel` semantics: `0` = all steppers of the motor (every existing producer already writes zeroed pad bytes — update the literal initializers `_pad: [0; 3]` → `stepper_sel: 0, _pad: [0; 2]` at every construction site; `grep -rn "_pad: \[0; 3\]" rust/`), `n` = only stepper `n-1`.

`rust/runtime/src/per_axis_timer.rs` — change the extern and the call:

```rust
unsafe extern "C" {
    fn runtime_emit_step_pulses(motor_idx: u8, n_steps: i32, stepper_sel: u8);
}
```

```rust
unsafe { runtime_emit_step_pulses(axis_idx as u8, i32::from(entry.dir), entry.stepper_sel) };
```

(Mirror the same change in the host/test hook variant of `runtime_emit_step_pulses` that the test build links — find it next to the existing test hooks in this file or `test_hooks`.)

`src/stepper.c` — `runtime_emit_step_pulses` gains the parameter; sel==0 keeps the existing all-steppers loops; nonzero selects one stepper. Shape of the change (apply inside the existing function at `src/stepper.c:395-448`, keeping the existing pulse-timing and both-edge logic identical — only the stepper iteration bounds change):

```c
void runtime_emit_step_pulses(uint8_t motor_idx, int32_t n_steps, uint8_t stepper_sel)
{
    ...
    uint8_t cnt = runtime_motor_stepper_count[motor_idx];
    uint8_t j_begin = 0, j_end = cnt;
    if (stepper_sel != 0) {
        if (stepper_sel > cnt)
            shutdown("correction stepper_sel out of range");
        j_begin = stepper_sel - 1;
        j_end = stepper_sel;
    }
    ...
    /* every existing `for (j = 0; j < cnt; j++)` over dir-writes and
       step-pin toggles becomes `for (j = j_begin; j < j_end; j++)` */
```

Direction handling: when `stepper_sel != 0`, write the dir pin for the selected stepper only and invalidate the cached motor direction (`runtime_motor_last_dir[motor_idx] = -1;`) so the next all-steppers pulse re-asserts direction on every stepper.

Update the C prototype declaration wherever it is declared (`grep -rn "runtime_emit_step_pulses" src/`).

- [ ] **Step 4: Run tests and compile firmware**

Run: `cd rust && cargo nextest run -p runtime`
Expected: PASS.
Run: `make clean && make -j$(sysctl -n hw.ncpu)` (with whatever `.config` is currently active) to confirm the C side compiles.
Expected: clean build.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime src/stepper.c $(git diff --name-only src/)
git commit -m "feat(runtime): stepper_sel routing for single-stepper pulse emission"
```

---

### Task 4: Runtime — correction stream evaluation in the tick

**Files:**
- Create: `rust/runtime/src/dispatch_correction.rs` (+ `mod dispatch_correction;` in `rust/runtime/src/lib.rs`)
- Modify: `rust/runtime/src/engine.rs` (`tick`)
- Modify: `rust/runtime/src/log_codes.rs`
- Test: `rust/runtime/src/dispatch_correction/tests.rs`

- [ ] **Step 1: Write the failing tests**

The existing engine/dispatch tests show how to drive `tick` with `test_install_step_queues` and fake storage — model the harness on them (find: `grep -rln "test_install_step_queues" rust/runtime/src`). Tests to write:

```rust
#[test]
fn pulse_correction_steps_only_selected_stepper() {
    // configure pulse Z axis with 3 steppers, commit a correction stream for
    // motor_idx=1 moving 0 -> 0.0125mm (10 microsteps at 0.00125) over 0.01s,
    // tick through the stream, then drain the Z step queue and assert:
    //   - every popped StepEntry has stepper_sel == 2
    //   - total steps == 10, dir == 1
    //   - axis.last_step_count (main tracker) unchanged
    //   - steppers[1].position_count advanced by 10; steppers[0]/[2] unchanged
}

#[test]
fn pulse_correction_stream_end_resets_relative_frame() {
    // after the stream drains: correction_active() == false,
    // correction_last_step_count == 0, correction_motor_idx == CORRECTION_MOTOR_NONE,
    // and a second committed stream for the same axis works from scratch
}

#[test]
fn phase_correction_moves_only_selected_offset_target() {
    // configure phase Z axis (tmc_cs_oid set on all steppers), commit a
    // correction stream for motor_idx=0, tick through it, assert
    // steppers[0].phase_offset_target advanced by round(delta/microstep)
    // and steppers[1..].phase_offset_target unchanged; main p_prev unchanged.
}

#[test]
fn correction_does_not_mark_axis_position() {
    // p_prev and last_step_count equal their pre-stream values after drain
}
```

Write them concretely against the harness idiom you find; the four assertions above are the contract.

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p runtime -E 'test(correction)'`
Expected: FAIL (no correction evaluation happens; queues stay empty).

- [ ] **Step 3: Implement**

`rust/runtime/src/log_codes.rs`: add two events in `SUBSYSTEM_MOTION`, taking the next two unused event ids in that subsystem (the table is append-only/wire-stable — do not renumber anything):

```rust
(SUBSYSTEM_MOTION, EVENT_MOTION_CORRECTION_START) => (
    "motion.correction_start",
    "correction stream start axis={arg0} motor={arg1}",
),
(SUBSYSTEM_MOTION, EVENT_MOTION_CORRECTION_DRAINED) => (
    "motion.correction_drained",
    "correction stream drained axis={arg0} steps={arg1}",
),
```

`rust/runtime/src/dispatch_correction.rs`:

```rust
use core::sync::atomic::Ordering;

use crate::piece_ring::PieceEntry;
use crate::state::SharedState;
use crate::step_queue::{StepEntry, StepQueue, peek as queue_peek, push as queue_push};
use crate::stepping_state::{AxisState, CORRECTION_MOTOR_NONE, StepMode};
use crate::sub_sample_timing::{StepTimeInputs, StepTimingResult, compute_step_times};

#[allow(clippy::too_many_arguments)]
pub fn tick_correction(
    axis_idx: usize,
    axis: &mut AxisState,
    queue_ptr: *mut StepQueue,
    shared: &SharedState,
    storage: &mut [PieceEntry],
    now: u64,
    sample_period_cycles: u32,
    sample_period_sec: f32,
    sample_start_cycles: u32,
    cycles_per_second: f32,
    fault: &impl crate::fault_sink::FaultSink,
) -> bool {
    if !axis.correction_active() {
        return false;
    }
    let motor_idx = axis.correction_motor_idx;
    let eval = crate::motion_core::get_position_and_velocity(
        &mut axis.correction_armed,
        &mut axis.correction_ring,
        storage,
        now,
        sample_period_cycles,
        cycles_per_second,
        axis_idx,
        fault,
    );
    let Some((c_end, _v_end)) = eval else {
        if !axis.correction_active() {
            finish_stream(axis_idx, axis, shared);
        }
        return true;
    };
    let c_prev = axis.correction_p_prev;
    axis.correction_p_prev = c_end;

    match axis.mode.load(Ordering::Acquire) {
        m if m == StepMode::Pulse as u8 => emit_correction_steps(
            axis_idx,
            axis,
            queue_ptr,
            shared,
            c_end,
            c_prev,
            sample_period_sec,
            sample_start_cycles,
            cycles_per_second,
            motor_idx,
        ),
        m if m == StepMode::Phase as u8 => advance_phase_target(axis, c_end, motor_idx),
        other => crate::fault_helpers::raise_unknown_step_mode(shared, axis_idx, other),
    }
    true
}
```

`emit_correction_steps` is `dispatch_pulse` (`dispatch_stepper.rs:103-228`) with four differences — write it as a sibling, do not try to parameterize `dispatch_pulse` itself:
- tracks `axis.correction_last_step_count` instead of `axis.last_step_count`;
- builds `StepEntry { cycle_abs, dir, stepper_sel: motor_idx + 1, _pad: [0; 2] }`;
- on success updates **only** `axis.steppers[motor_idx].position_count` (checked_add, `raise_position_count_overflow` on overflow) — never the other steppers;
- the steps-per-sample overflow and queue-overflow fault paths are kept verbatim (same fault helpers).

`advance_phase_target`:

```rust
fn advance_phase_target(axis: &mut AxisState, c_end: f32, motor_idx: u8) {
    let microstep_distance = axis.microstep_distance;
    if !microstep_distance.is_finite() || microstep_distance == 0.0 {
        return;
    }
    #[allow(clippy::cast_possible_truncation)]
    let scratch_steps = libm::roundf(c_end / microstep_distance) as i32;
    let delta = scratch_steps.wrapping_sub(axis.correction_last_step_count);
    if delta == 0 {
        return;
    }
    axis.correction_last_step_count = scratch_steps;
    let Some(stepper) = axis.steppers.get(motor_idx as usize) else {
        return;
    };
    let new_target = stepper
        .phase_offset_target
        .load(Ordering::Acquire)
        .wrapping_add(delta);
    stepper.phase_offset_target.store(new_target, Ordering::Release);
}
```

(The existing `ramp_phase_offset` + `idle_phase_slew_pending` machinery in `dispatch_phase`/`tick` then materializes the offset into coil writes; `max_phase_offset_ramp_per_sample` only needs to exceed the per-sample microstep delta — at z_tilt speeds that is < 1 microstep/sample, and a lagging ramp still converges after the stream because the slew-pending path keeps dispatching.)

`finish_stream`:

```rust
fn finish_stream(axis_idx: usize, axis: &mut AxisState, shared: &SharedState) {
    crate::log_emit_helper(/* match the emit idiom used by fault_helpers.rs:
        level=info, SUBSYSTEM_MOTION, EVENT_MOTION_CORRECTION_DRAINED,
        code=axis_idx as u16, arg0=axis_idx as u32,
        arg1=axis.correction_last_step_count.unsigned_abs() */);
    axis.correction_motor_idx = CORRECTION_MOTOR_NONE;
    axis.correction_last_step_count = 0;
    axis.correction_p_prev = 0.0;
}
```

(Read `rust/runtime/src/fault_helpers.rs` for the exact `kalico_log_emit` call shape and copy it; emit `EVENT_MOTION_CORRECTION_START` analogously from `Engine::commit_correction` on the inactive→active transition.)

`rust/runtime/src/engine.rs` `tick`: at the top of the per-axis loop body (before the normal `get_position_and_velocity` block), call:

```rust
            let corr_active = {
                let Some(axis) = self.stepping_axes.get_mut(i).and_then(|s| s.as_mut()) else {
                    continue;
                };
                let fault = SharedFaultSink { shared };
                crate::dispatch_correction::tick_correction(
                    i,
                    axis,
                    get_queue(i),
                    shared,
                    storage,
                    now,
                    self.sample_period_cycles,
                    sample_period_sec,
                    now_lo,
                    self.cycles_per_second,
                    &fault,
                )
            };
            if corr_active {
                active = true;
            }
```

(Gate the `#[cfg(feature = "motion-module-stepper")]` items the same way the existing block does; in the non-stepper cfg, pass a null queue and let pulse emission be unreachable.)

`tick_correction` returning `true` must make the axis count as active even when the main ring is idle so phase-mode slew keeps running — the existing `idle_phase_slew_pending` path already handles the dispatch side.

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo nextest run -p runtime`
Expected: all correction tests pass; full runtime suite green.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime
git commit -m "feat(runtime): correction stream evaluation in the sample tick"
```

---

### Task 5: FFI + C dispatch — receive `PushCorrectionPieces` on the MCU

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `src/kalico_dispatch.c`
- Test: `rust/kalico-c-api/tests/correction_pieces.rs` (new, modeled on `rust/kalico-c-api/tests/write_piece.rs`)

- [ ] **Step 1: Write the failing FFI test**

Read `rust/kalico-c-api/tests/write_piece.rs` and write `correction_pieces.rs` in its idiom:

```rust
#[test]
fn write_and_commit_correction_roundtrip() {
    // init runtime, configure axis 2 (3 steppers) exactly as write_piece.rs does,
    // serialize one PieceEntry to 32 bytes,
    // kalico_runtime_write_correction_piece(rt, 2, 0, 0, ptr) == KALICO_OK,
    // kalico_runtime_commit_correction(rt, 2, 1, 1) == KALICO_OK
}

#[test]
fn commit_correction_rejects_when_axis_busy() {
    // push + commit one normal piece first, then expect
    // kalico_runtime_commit_correction(...) == KALICO_ERR_MOTION_IN_PROGRESS (-31)
}

#[test]
fn normal_commit_rejects_when_correction_active() {
    // commit a correction stream, then kalico_runtime_commit_head(rt, 2, 1)
    // == KALICO_ERR_CORRECTION_IN_PROGRESS (-314)
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p kalico-c-api -E 'test(correction)'`
Expected: compile error — FFI symbols missing.

- [ ] **Step 3: Implement**

`rust/kalico-c-api/src/runtime_ffi.rs` — two new externs, copied structurally from `kalico_runtime_write_piece` / `kalico_runtime_commit_head` (same null/init guards, same `§11.2 foreground-only` SAFETY discipline, same `read_unaligned`):

```rust
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_write_correction_piece(
        rt: *mut KalicoRuntime,
        axis_idx: u8,
        start_slot: u16,
        index: u8,
        piece_ptr: *const u8,
    ) -> i32 {
        // guards as in kalico_runtime_write_piece, then:
        //   let entry = read_unaligned(piece_ptr as *const PieceEntry);
        //   (*isr_ptr).engine.write_correction_piece(axis_idx, start_slot, index, entry, storage)
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_commit_correction(
        rt: *mut KalicoRuntime,
        axis_idx: u8,
        motor_idx: u8,
        new_head: u32,
    ) -> i32 {
        // guards as in kalico_runtime_commit_head (incl. pieces_gated() -> STREAM_HALTED), then:
        //   (*isr_ptr).engine.commit_correction(axis_idx, motor_idx, new_head)
    }
```

(Write the bodies fully by transcribing the two existing functions and swapping the engine calls — the guard prologue is identical.)

Also in `kalico_runtime_commit_head`, before the `commit_head` match, add the inverse-door guard:

```rust
            let guard = (*isr_ptr).engine.guard_normal_commit(axis_idx);
            if guard != KALICO_OK {
                return guard;
            }
```

`src/kalico_dispatch.c` — new handler wired into the `switch (kind)` in `kalico_dispatch_frame`:

```c
    case KALICO_MSG_PUSH_CORRECTION_PIECES:
        handle_push_correction_pieces(correlation_id, body, body_len);
        return;
```

```c
// PushCorrectionPieces body: axis_idx u8 | motor_idx u8 | piece_count u8
// | start_slot u16_le | new_head u32_le | piece_count * 32 bytes.
#define CORRECTION_HEADER_LEN 9u

extern int32_t kalico_runtime_write_correction_piece(
    void *rt, uint8_t axis_idx, uint16_t start_slot, uint8_t index,
    const uint8_t *piece);
extern int32_t kalico_runtime_commit_correction(
    void *rt, uint8_t axis_idx, uint8_t motor_idx, uint32_t new_head);

static void
send_push_correction_pieces_response(uint32_t correlation_id, int32_t result,
                                     uint64_t arrival_clock)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 4 + 8];
    encode_message_header(payload, KALICO_MSG_PUSH_CORRECTION_PIECES_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)(result & 0xFF);
    b[1] = (uint8_t)((result >> 8) & 0xFF);
    b[2] = (uint8_t)((result >> 16) & 0xFF);
    b[3] = (uint8_t)((result >> 24) & 0xFF);
    for (int i = 0; i < 8; i++)
        b[4 + i] = (uint8_t)((arrival_clock >> (8 * i)) & 0xFF);
    kalico_transport_send_frame(KALICO_CHANNEL_CONTROL, payload, sizeof(payload));
}

static void
handle_push_correction_pieces(uint32_t correlation_id, const uint8_t *body,
                              uint16_t body_len)
{
    uint32_t clk_lo = timer_read_time();
    uint32_t clk_hi = stats_send_time_high + (clk_lo < stats_send_time);
    uint64_t arrival_clock = ((uint64_t)clk_hi << 32) | (uint64_t)clk_lo;

    if (!runtime_handle) {
        send_push_correction_pieces_response(correlation_id,
                                             KALICO_ERR_NOT_INIT, 0);
        return;
    }
    if (body_len < CORRECTION_HEADER_LEN) {
        send_push_correction_pieces_response(correlation_id,
                                             KALICO_ERR_INVALID_CURVE, 0);
        return;
    }
    uint8_t axis_idx = body[0];
    uint8_t motor_idx = body[1];
    uint8_t piece_count = body[2];
    uint16_t start_slot = (uint16_t)body[3] | ((uint16_t)body[4] << 8);
    uint32_t new_head = (uint32_t)body[5] | ((uint32_t)body[6] << 8)
                      | ((uint32_t)body[7] << 16) | ((uint32_t)body[8] << 24);
    if (body_len != CORRECTION_HEADER_LEN + (uint16_t)piece_count * 32u) {
        send_push_correction_pieces_response(correlation_id,
                                             KALICO_ERR_INVALID_CURVE, 0);
        return;
    }
    int32_t rc = 0;
    for (uint8_t i = 0; i < piece_count && rc == 0; i++)
        rc = kalico_runtime_write_correction_piece(
            runtime_handle, axis_idx, start_slot, i,
            &body[CORRECTION_HEADER_LEN + (uint32_t)i * 32u]);
    if (rc == 0) {
        irqstatus_t flag = irq_save();
        rc = kalico_runtime_commit_correction(runtime_handle, axis_idx,
                                              motor_idx, new_head);
        irq_restore(flag);
    }
    send_push_correction_pieces_response(correlation_id, rc, arrival_clock);
}
```

(`irq_save` around the commit because the validation reads ring/armed state the TIM5 ISR mutates — same pattern as `handle_stop`.)

- [ ] **Step 4: Run tests and compile firmware**

Run: `cd rust && cargo nextest run -p kalico-c-api`
Expected: PASS.
Run: `make clean && make -j$(sysctl -n hw.ncpu)`
Expected: clean build.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-c-api src/kalico_dispatch.c
git commit -m "feat(mcu): receive PushCorrectionPieces on the control channel"
```

---

### Task 6: Host bridge — correction move planner (pure)

**Files:**
- Create: `rust/motion-bridge/src/correction.rs` (+ `mod correction;` in `rust/motion-bridge/src/lib.rs` or `main` module file — match where `enqueue` is declared)
- Test: `rust/motion-bridge/src/correction/tests.rs`

- [ ] **Step 1: Write the failing tests**

```rust
use super::*;

#[test]
fn trapezoid_reaches_delta_exactly() {
    let pieces = plan_correction_profile(10.0, 5.0, 100.0).unwrap();
    let last = pieces.last().unwrap();
    assert!((last.coeffs[3] - 10.0).abs() < 1e-9);
    assert!((pieces.iter().map(|p| p.duration).sum::<f64>()
        - profile_duration(10.0, 5.0, 100.0).unwrap()).abs() < 1e-9);
}

#[test]
fn profile_is_continuous_in_position_and_velocity() {
    let pieces = plan_correction_profile(-3.7, 8.0, 500.0).unwrap();
    for w in pieces.windows(2) {
        let end_p = w[0].coeffs[3];
        let start_p = w[1].coeffs[0];
        assert!((end_p - start_p).abs() < 1e-9);
        let end_v = 3.0 * (w[0].coeffs[3] - w[0].coeffs[2]) / w[0].duration;
        let start_v = 3.0 * (w[1].coeffs[1] - w[1].coeffs[0]) / w[1].duration;
        assert!((end_v - start_v).abs() < 1e-6);
    }
}

#[test]
fn short_move_uses_triangular_profile() {
    let pieces = plan_correction_profile(0.01, 50.0, 100.0).unwrap();
    let peak_v = pieces
        .iter()
        .map(|p| (3.0 * (p.coeffs[1] - p.coeffs[0]) / p.duration).abs())
        .fold(0.0_f64, f64::max);
    assert!(peak_v < 50.0);
    assert!((pieces.last().unwrap().coeffs[3] - 0.01).abs() < 1e-9);
}

#[test]
fn rejects_nonpositive_speed_accel_and_zero_delta() {
    assert!(plan_correction_profile(1.0, 0.0, 100.0).is_err());
    assert!(plan_correction_profile(1.0, 5.0, -1.0).is_err());
    assert!(plan_correction_profile(0.0, 5.0, 100.0).is_err());
}

#[test]
fn no_piece_exceeds_max_piece_duration() {
    let pieces = plan_correction_profile(50.0, 2.0, 100.0).unwrap();
    for p in &pieces {
        assert!(p.duration <= MAX_CORRECTION_PIECE_SECS + 1e-9);
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(correction)'`
Expected: compile error — module missing.

- [ ] **Step 3: Implement**

```rust
pub const MAX_CORRECTION_PIECE_SECS: f64 = 0.25;

#[derive(Debug, Clone, Copy)]
pub struct ProfilePiece {
    pub coeffs: [f64; 4],
    pub duration: f64,
}

pub fn profile_duration(delta_mm: f64, speed: f64, accel: f64) -> Result<f64, String> {
    let d = delta_mm.abs();
    if !(d > 0.0) || !(speed > 0.0) || !(accel > 0.0) {
        return Err(format!(
            "correction profile needs delta!=0, speed>0, accel>0; got {delta_mm} {speed} {accel}"
        ));
    }
    let v = speed.min((d * accel).sqrt());
    let t_ramp = v / accel;
    let d_cruise = d - v * t_ramp;
    Ok(2.0 * t_ramp + d_cruise / v)
}

pub fn plan_correction_profile(
    delta_mm: f64,
    speed: f64,
    accel: f64,
) -> Result<Vec<ProfilePiece>, String> {
    profile_duration(delta_mm, speed, accel)?;
    let sign = delta_mm.signum();
    let d = delta_mm.abs();
    let v = speed.min((d * accel).sqrt());
    let t_ramp = v / accel;
    let d_ramp = 0.5 * accel * t_ramp * t_ramp;
    let d_cruise = d - 2.0 * d_ramp;

    let mut out = Vec::new();
    push_quadratic(&mut out, 0.0, 0.0, accel, t_ramp, sign);
    if d_cruise > 1e-12 {
        push_linear(&mut out, d_ramp, v, d_cruise / v, sign);
    }
    push_quadratic(&mut out, d_ramp + d_cruise, v, -accel, t_ramp, sign);
    Ok(out
        .into_iter()
        .flat_map(|p| subdivide(p, MAX_CORRECTION_PIECE_SECS))
        .collect())
}
```

`push_quadratic` (position `p(t) = p0 + v0 t + ½ a t²` on `[0, T]` as exact cubic Bernstein — `b0 = p0`, `b1 = p0 + v0 T/3`, `b2 = p0 + 2 v0 T/3 + a T²/3`, `b3 = p0 + v0 T + ½ a T²`, each scaled by `sign`), `push_linear` (`b_i = p0 + v T i/3`, scaled), and `subdivide` (de Casteljau split at `t = max_secs/duration` repeatedly; verify against `subdivide_bernstein` in `enqueue.rs` — if that function is reusable on `[f64; 4]`, call it instead of writing a new one).

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(correction)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge
git commit -m "feat(bridge): pure trapezoid planner for correction profiles"
```

---

### Task 7: Host bridge — `adjust_motor` pyo3 entry point

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (new method on `PyMotionBridge`)
- Modify: `rust/motion-bridge/src/correction.rs` (message-chunking helper + tests)

- [ ] **Step 1: Write the failing chunking test**

In `correction/tests.rs`:

```rust
#[test]
fn chunking_respects_frame_budget_and_ring_depth() {
    let pieces: Vec<ProfilePiece> = plan_correction_profile(50.0, 2.0, 100.0).unwrap();
    let entries: Vec<kalico_protocol_pieces::PieceEntry> = to_piece_entries(&pieces, |secs| {
        (secs * 1e6) as u64
    }, 0.0);
    let msgs = chunk_correction_messages(2, 1, &entries);
    let mut expected_head: u32 = 0;
    for m in &msgs {
        assert!(m.piece_count as usize <= MAX_CORRECTION_PIECES_PER_MSG);
        assert_eq!(m.pieces_bytes.len(), m.piece_count as usize * 32);
        expected_head += u32::from(m.piece_count);
        assert_eq!(m.new_head, expected_head);
    }
    assert_eq!(expected_head as usize, entries.len());
}
```

(Use the actual `PieceEntry` import path the crate already uses — `runtime::piece_ring::PieceEntry` per `enqueue.rs`.)

- [ ] **Step 2: Run to verify failure, then implement**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(chunking)'` → compile error.

In `correction.rs`:

```rust
pub const MAX_CORRECTION_PIECES_PER_MSG: usize = 15;

pub fn to_piece_entries(
    pieces: &[ProfilePiece],
    project: impl Fn(f64) -> u64,
    start_host_secs: f64,
) -> Vec<runtime::piece_ring::PieceEntry> {
    let mut t = start_host_secs;
    pieces
        .iter()
        .map(|p| {
            let entry = runtime::piece_ring::PieceEntry {
                start_time: project(t),
                coeffs: [
                    p.coeffs[0] as f32,
                    p.coeffs[1] as f32,
                    p.coeffs[2] as f32,
                    p.coeffs[3] as f32,
                ],
                duration: p.duration as f32,
                _reserved: 0,
            };
            t += p.duration;
            entry
        })
        .collect()
}

pub fn chunk_correction_messages(
    axis_idx: u8,
    motor_idx: u8,
    entries: &[runtime::piece_ring::PieceEntry],
) -> Vec<kalico_protocol::messages::PushCorrectionPieces> {
    let mut out = Vec::new();
    let mut head: u32 = 0;
    for chunk in entries.chunks(MAX_CORRECTION_PIECES_PER_MSG) {
        let start_slot = (head % runtime::stepping_state::CORRECTION_RING_DEPTH as u32) as u16;
        let mut pieces_bytes = Vec::with_capacity(chunk.len() * 32);
        for e in chunk {
            pieces_bytes.extend_from_slice(&e.to_le_bytes());
        }
        head += chunk.len() as u32;
        out.push(kalico_protocol::messages::PushCorrectionPieces {
            axis_idx,
            motor_idx,
            piece_count: chunk.len() as u8,
            start_slot,
            new_head: head,
            pieces_bytes,
        });
    }
    out
}
```

(`MAX_CORRECTION_PIECES_PER_MSG = 15`: demux buffer 512 − envelope 4 − crc 2 − msg header 7 − body header 9 = 490; 490/32 = 15. Add a const assert mirroring that arithmetic. Note 15 < `CORRECTION_RING_DEPTH` 16, so each chunk fits the ring; the per-chunk send-then-ack flow below means the MCU drains while later chunks arrive — but to stay simple and correct, chunks are sent sequentially and each waits for its ack; if a `KALICO_ERR_RING_FULL` comes back because the stream outpaces consumption, that is a real error to surface, not retry.)

In `bridge.rs`, next to `motion_state_at_clock`, add a `#[pyo3]` method (transcribe the transport/lookup idiom from `WireSink::call_push_pieces` in `pump.rs:736-827` and the `get_host_io` pattern from `register_phase_motor`):

```rust
    #[pyo3(signature = (mcu_id, axis_idx, motor_idx, delta_mm, speed, accel, host_now))]
    fn adjust_motor(
        &self,
        mcu_id: u32,
        axis_idx: u8,
        motor_idx: u8,
        delta_mm: f64,
        speed: f64,
        accel: f64,
        host_now: f64,
    ) -> PyResult<f64> {
        const CORRECTION_LEAD_SECS: f64 = 0.15;
        let profile = crate::correction::plan_correction_profile(delta_mm, speed, accel)
            .map_err(PyRuntimeError::new_err)?;
        let duration = crate::correction::profile_duration(delta_mm, speed, accel)
            .map_err(PyRuntimeError::new_err)?;
        let start_secs = host_now + CORRECTION_LEAD_SECS;
        let handle = crate::types::mcu_handle_from_raw(mcu_id);
        let entries = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            crate::correction::to_piece_entries(
                &profile,
                |secs| router.host_time_to_mcu_clock(handle, secs).unwrap_or(0),
                start_secs,
            )
        };
        if entries.iter().any(|e| e.start_time == 0) {
            return Err(PyRuntimeError::new_err(format!(
                "adjust_motor: clock unsynced for mcu {mcu_id}"
            )));
        }
        let io = self.get_host_io(mcu_id).ok_or_else(|| {
            PyRuntimeError::new_err(
                "adjust_motor: serial transport not attached for this MCU \
                 (EtherCAT correction moves are not supported yet)",
            )
        })?;
        for msg in crate::correction::chunk_correction_messages(axis_idx, motor_idx, &entries) {
            let mut body = Vec::with_capacity(9 + msg.pieces_bytes.len());
            kalico_protocol::codec::Encode::encode(&msg, &mut body);
            let (_kind, resp) = io
                .kalico_call_on_channel(
                    kalico_protocol::KALICO_CHANNEL_CONTROL,
                    kalico_protocol::MessageKind::PushCorrectionPieces,
                    body,
                    std::time::Duration::from_secs(2),
                )
                .map_err(|e| PyRuntimeError::new_err(format!("adjust_motor send: {e:?}")))?;
            use kalico_protocol::codec::Decode as _;
            let r = kalico_protocol::messages::PushCorrectionPiecesResponse::decode(&resp)
                .map_err(|e| PyRuntimeError::new_err(format!("adjust_motor decode: {e:?}")))?;
            if r.result != 0 {
                return Err(PyRuntimeError::new_err(format!(
                    "adjust_motor rejected by MCU: error {}",
                    r.result
                )));
            }
        }
        Ok(start_secs + duration - host_now)
    }
```

Adapt the exact lock-field names (`self.router`, `self.get_host_io`, `decode(&resp)` vs `decode_from`) to what `bridge.rs`/`pump.rs` actually expose — all three are demonstrated in the existing code cited above.

- [ ] **Step 3: Run tests + build the native module**

Run: `cd rust && cargo nextest run -p motion-bridge && cargo build -p motion-bridge`
Expected: PASS, clean build.

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge
git commit -m "feat(bridge): adjust_motor one-shot correction send path"
```

---

### Task 8: Klippy — name resolution, wrapper, and `MOTOR_ADJUST`

**Files:**
- Modify: `klippy/motion_toolhead.py` (motor-binding lookup; the slot-collection loop is at ~lines 862-897)
- Modify: `klippy/motion_bridge.py` (`MotionBridgeWrapper.adjust_motor`)
- Create: `klippy/extras/motor_adjust.py`

No Python test harness exists for these modules in this repo's rewrite path; verification is Task 10 (sim) and the bench. Keep this task to mechanical wiring.

- [ ] **Step 1: Implement the binding lookup**

In `motion_toolhead.py`, where the per-slot stepper lists are assembled (primary first, AWD partners in name order), record a name→binding map and expose it:

```python
        # inside the slot-collection loop, once per bound stepper:
        self._motor_bindings[stepper.get_name()] = (mcu_id, slot_idx, motor_idx)
```

```python
    def get_motor_binding(self, stepper_name):
        binding = self._motor_bindings.get(stepper_name)
        if binding is None:
            raise self.printer.config_error(
                "Unknown motor '%s'; bound motors: %s"
                % (stepper_name, ", ".join(sorted(self._motor_bindings)))
            )
        return binding
```

(`motor_idx` is the index within the slot's stepper list — the same order `kalico_configure_axis` receives, which is what the MCU's `axis.steppers` indexing means. `mcu_id` is the raw id used elsewhere when calling bridge methods — match the existing call sites.)

- [ ] **Step 2: Implement the wrapper**

In `motion_bridge.py`, next to `home_axis_start`:

```python
    def adjust_motor(self, mcu_id, axis_idx, motor_idx, delta_mm, speed, accel, host_now):
        return self._bridge.adjust_motor(
            mcu_id, axis_idx, motor_idx, delta_mm, speed, accel, host_now
        )
```

- [ ] **Step 3: Implement the command**

`klippy/extras/motor_adjust.py`:

```python
class MotorAdjust:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "MOTOR_ADJUST",
            self.cmd_MOTOR_ADJUST,
            desc=self.cmd_MOTOR_ADJUST_help,
        )

    cmd_MOTOR_ADJUST_help = (
        "Move a single motor of a multi-motor axis by DELTA mm without"
        " changing the commanded axis position"
    )

    def adjust(self, stepper_name, delta_mm, speed, accel):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.wait_moves()
        mcu_id, axis_idx, motor_idx = toolhead.get_motor_binding(stepper_name)
        bridge = toolhead.get_bridge()
        reactor = self.printer.get_reactor()
        host_now = reactor.monotonic()
        duration = bridge.adjust_motor(
            mcu_id, axis_idx, motor_idx, delta_mm, speed, accel, host_now
        )
        deadline = host_now + duration + 0.05
        while reactor.monotonic() < deadline:
            reactor.pause(reactor.monotonic() + 0.01)

    def cmd_MOTOR_ADJUST(self, gcmd):
        stepper_name = gcmd.get("MOTOR")
        delta_mm = gcmd.get_float("DELTA")
        speed = gcmd.get_float("SPEED", 5.0, above=0.0)
        accel = gcmd.get_float("ACCEL", 100.0, above=0.0)
        self.adjust(stepper_name, delta_mm, speed, accel)
        gcmd.respond_info(
            "motor %s adjusted by %.6f mm" % (stepper_name, delta_mm)
        )


def load_config(config):
    return MotorAdjust(config)
```

Adapt `toolhead.get_bridge()` / how other extras reach the bridge wrapper to the actual accessor (`grep -n "def get_bridge\|self.bridge" klippy/motion_toolhead.py`); if `wait_moves` doesn't guarantee MCU-side ring drain, follow it with the same `motion_drain_poll` loop `motion_toolhead.py:552-606` uses. Motor enable: `wait_moves` plus the `stepper_enable` activity hooks cover the enabled-during-session case; for adjust-before-any-move, look up `stepper_enable` and call its enable for the named stepper's line before sending (see `klippy/extras/stepper_enable.py:29-78`).

- [ ] **Step 4: Syntax check**

Run: `python3 -m py_compile klippy/extras/motor_adjust.py klippy/motion_bridge.py klippy/motion_toolhead.py`
Expected: no output.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/motor_adjust.py klippy/motion_bridge.py klippy/motion_toolhead.py
git commit -m "feat(klippy): MOTOR_ADJUST command over the correction-move path"
```

---

### Task 9: Klippy — implement `ZAdjustHelper.adjust_steppers` (z_tilt + QGL)

**Files:**
- Modify: `klippy/extras/z_tilt.py:37-46` (the stub; QGL shares this helper)

- [ ] **Step 1: Implement**

Replace the body of `ZAdjustHelper.adjust_steppers` (keep the existing respond_info listing):

```python
    def adjust_steppers(self, adjustments, speed):
        gcode = self.printer.lookup_object("gcode")
        stepstrs = [
            "%s = %.6f" % (s.get_name(), a)
            for s, a in zip(self.z_steppers, adjustments)
        ]
        gcode.respond_info("Making the following Z adjustments:\n%s"
                           % ("\n".join(stepstrs),))
        adjuster = self.printer.load_object(self.config, "motor_adjust")
        accel = self.printer.lookup_object("toolhead").get_max_axis_accel(2)
        for stepper, adjustment in zip(self.z_steppers, adjustments):
            if abs(adjustment) < 1e-6:
                continue
            adjuster.adjust(stepper.get_name(), adjustment, speed, accel)
```

Notes for the implementer:
- Check the sign convention the z_tilt solver produces (read `ZAdjustStatusHelper`/the calling code in the same file: `adjustments` are per-stepper position corrections; mainline applied them via per-stepper moves with the same sign). If the first probe iteration diverges on the bench, the sign flips here — the retry/tolerance loop already in z_tilt makes this observable immediately.
- `self.config` must be stashed in `__init__` if not already; `get_max_axis_accel` — use whatever accessor motion_toolhead exposes for the Z accel limit, or fall back to a `motor_adjust`-style default of 100.0 if none exists.
- QGL (`quad_gantry_level.py`) calls this same helper; no change needed there.

- [ ] **Step 2: Syntax check + commit**

Run: `python3 -m py_compile klippy/extras/z_tilt.py`

```bash
git add klippy/extras/z_tilt.py
git commit -m "feat(klippy): per-motor Z adjustment via correction moves"
```

---

### Task 10: End-to-end simulation check

**Files:** none (verification task)

- [ ] **Step 1: Run the simulator scenario**

Use the `kalico-sim` skill (Docker-based simulator) with a 3-Z-stepper cartesian config:
1. Boot, home nothing, run `MOTOR_ADJUST MOTOR=stepper_z1 DELTA=2.0` → expect success response and `motion.correction_start` / `motion.correction_drained` events in `events/*.jsonl` (query via the `query-logs` skill).
2. Verify the reported toolhead Z position is unchanged after the command.
3. Issue `MOTOR_ADJUST` while a long move is in flight → expect a gcode error carrying MCU error -31.
4. `MOTOR_ADJUST MOTOR=stepper_q DELTA=1` → expect the unknown-motor error listing valid names.

- [ ] **Step 2: Full suite + fmt gate**

Run: `cd rust && cargo nextest run && cargo test --doc && cargo fmt --all --check`
Expected: all green.

- [ ] **Step 3: Commit any fixes, then stop**

Bench validation (Trident, 3-motor Z) comes only after all of the above passes, per the exhaust-analysis-before-bench rule, and every flash follows commit → push → pull → compile on the Pi → flash both MCUs (schema hash changed!) with `make clean` between the H7 and F446 builds.

---

## Self-review notes (resolved inline)

- Spec coverage: wire message (T1, T5), validation incl. inverse door (T2, T5), pulse routing (T3, T4), phase folding via `phase_offset_target` (T4), detached frame & reset (T2, T4), log events (T4), host API (T6, T7), MOTOR_ADJUST by stepper name (T8), z_tilt/QGL consumer (T9), sim + error-path tests (T10). Caps-reservation: `configure_axis` charges both rings against the same `total_piece_memory` budget the host already sizes rings from, so the host's ring-depth computation in `motion_toolhead.py` must leave headroom — **T2 Step 3 addendum:** find where the host computes per-axis `ring_depth` from `RuntimeCapsResponse.total_piece_memory` (`grep -rn "total_piece_memory" rust/ klippy/`) and subtract `CORRECTION_RING_DEPTH × axis_count × 32` bytes before dividing; add a bridge-side test pinning that arithmetic.
- Type consistency: `commit_correction(axis_idx: u8, motor_idx: u8, new_head: u32)` is used identically in T2 (engine), T5 (FFI + C extern). `stepper_sel = motor_idx + 1` (T3 producer in T4's `emit_correction_steps`, consumer in T3's C change). `PushCorrectionPieces` field order matches T1 codec, T5 C parser, T7 chunker.
- The plan deliberately avoids touching `PushPieces`, the pieces-channel streaming sink, and `dispatch_pulse` — per the spec's hot-path isolation decision.
