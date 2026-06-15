# Unify Corrections into the Motion Pipeline — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a per-motor correction a normal masked piece on the main motion ring dispatched by the async pump, and delete the entire bespoke correction path.

**Architecture:** A new `motor_mask: u8` on `PieceEntry` (in the spare `_reserved` word) drives the MCU evaluator: `mask==0` = normal full-axis move (all motors step+count, axis `p_prev` advances); a single-bit mask = overlay (that motor steps+counts, `p_prev` untouched); multi-bit = loud fault. Corrections enter via a new planner `submit_axis_overlay` that feeds the same pump as moves, so they're async/non-blocking. Then the correction ring, wire message, streamer, drain, and heartbeat plumbing are deleted.

**Tech Stack:** Rust (`runtime` no_std MCU engine, `motion-bridge` PyO3, `kalico-protocol`), C (`src/*.c` MCU dispatch), Python (klippy host). Tests: `cargo nextest run` from `rust/`; `python -m pytest`.

**Reference spec:** `docs/superpowers/specs/2026-06-15-unify-corrections-into-motion-pipeline-design.md`

**Decided:** single-bit masks only; `mask` with >1 bit set is a hard fault (YAGNI).

**Build/test commands:**
- Rust: `cd rust && cargo nextest run -p runtime` / `-p motion-bridge`; clippy `cargo clippy -p <crate> -- -D warnings`; fmt `cargo fmt --all`.
- MCU build sanity: `./scripts/ci.sh rust-mcu-h7` (cross-compile gate).
- Python: `python -m pytest test/test_toolhead_shim.py -v`.
- Full gate: `./scripts/ci.sh quick` then `./scripts/ci.sh py`.

**Execution note:** Phases are ordered so the tree builds and tests pass at every commit. Phase A adds the mask mechanism (old correction path still live, unused mask defaults to 0 = no behavior change). Phase B routes corrections through the pump. Phase C deletes the old path only after B works. Do NOT flash a bench until Phase C's gate is green.

---

## File Structure

- `rust/runtime/src/piece_ring.rs` — `PieceEntry` gains `motor_mask: u8`; `to_le_bytes`/`from_le_bytes`/doc-examples updated. Add `fn stepper_sel_from_mask`.
- `rust/runtime/src/dispatch_stepper.rs` — `dispatch_pulse` takes `motor_mask`, derives `stepper_sel`, scopes `commit_position_count`; fault on multi-bit.
- `rust/runtime/src/engine.rs` — tick reads the active piece's `motor_mask`, passes it to `dispatch_pulse`, gates the `p_prev`/`v_prev` write on `mask==0`.
- `rust/motion-bridge/src/planner.rs` — `PlannerMsg::AxisOverlay` + `submit_axis_overlay`.
- `rust/motion-bridge/src/bridge.rs` — `submit_correction_sequence`/`adjust_motor` become async (build pieces → `submit_axis_overlay`), no `start_print_time`.
- `klippy/motion.py` — `_stream_correction_on_timeline` reduced to a plain async submit; band-aids removed.
- `klippy/extras/{motors_sync,motor_adjust,force_move}.py` — callers wait via `wait_moves()`.
- **Deletions (Phase C):** `dispatch_correction.rs`, `correction.rs`, `stream_correction_entries`, `PushCorrectionPieces[Response]`, `correction_ring`/`correction_drain`/`CORRECTION_RING_DEPTH`, heartbeat `correction_retired` plumbing.

---

# Phase A — The masked-piece mechanism (MCU engine)

## Task A1: Add `motor_mask` to `PieceEntry`

**Files:**
- Modify: `rust/runtime/src/piece_ring.rs:308-318` (struct), `:355-366` (`to_le_bytes`), `from_le_bytes`, doc examples at `:183-186,302,349`.

- [ ] **Step 1: Write the failing test**

Append to the test module in `rust/runtime/src/piece_ring.rs` (find `#[cfg(test)] mod tests` or the inline tests; if none, add `#[cfg(test)] mod mask_tests { use super::*; ... }`):

```rust
#[test]
fn motor_mask_round_trips_at_byte_28() {
    let p = PieceEntry { start_time: 7, coeffs: [1.0, 2.0, 3.0, 4.0], duration: 0.5, motor_mask: 0b0000_0100, _reserved: [0; 3] };
    let b = p.to_le_bytes();
    assert_eq!(b[28], 0b0000_0100);
    assert_eq!(&b[29..32], &[0u8; 3]);
    let r = PieceEntry::from_le_bytes(&b);
    assert_eq!(r.motor_mask, 0b0000_0100);
    assert_eq!(r.start_time, 7);
}
```

- [ ] **Step 2: Run it (expect FAIL — field doesn't exist)**

Run: `cd rust && cargo nextest run -p runtime -E 'test(motor_mask_round_trips)'`
Expected: compile error (no `motor_mask` field).

- [ ] **Step 3: Change the struct**

In `rust/runtime/src/piece_ring.rs`, replace the `_reserved: u32` field:

```rust
#[repr(C, align(8))]
pub struct PieceEntry {
    pub start_time: u64,
    pub coeffs: [f32; 4],
    pub duration: f32,
    /// Bit i set => motor i of this axis runs this piece. `0` => all motors
    /// (a normal full-axis move that advances `p_prev`). Non-zero => overlay:
    /// only those motors step+count, `p_prev` is not advanced.
    pub motor_mask: u8,
    #[allow(clippy::pub_underscore_fields)]
    pub _reserved: [u8; 3],
}
```

The `const _: () = { assert!(size_of == 32); assert!(align == 8); }` block stays unchanged (size/align identical: u8 + [u8;3] occupies the same 4 bytes at offset 28).

- [ ] **Step 4: Update `to_le_bytes` byte 28-31**

In `to_le_bytes` replace the `_reserved` line:

```rust
        b[28] = self.motor_mask;
        b[29..32].copy_from_slice(&self._reserved);
        b
```

- [ ] **Step 5: Update `from_le_bytes`**

Find `from_le_bytes` (it currently reads `_reserved` as `u32::from_le_bytes(b[28..32])`). Replace its tail so it reads:

```rust
            motor_mask: b[28],
            _reserved: [b[29], b[30], b[31]],
```

(The `_reserved` bytes are not validated — drop any `assert _reserved == 0` if present; the mask byte is meaningful now.)

- [ ] **Step 6: Fix every `PieceEntry { ... }` literal**

Compile to find them: `cd rust && cargo build -p runtime 2>&1 | grep -n "missing field\|_reserved"`. Every literal `PieceEntry { ..., _reserved: 0 }` becomes `..., motor_mask: 0, _reserved: [0; 3]`. Update the doc-comment examples at `:183-186`, `:302`, `:349` likewise. (`motion-bridge` `correction::to_piece_entries` and `enqueue.rs` construct `PieceEntry` too — fix those in their own compiles in later tasks; for now `-p runtime` must build.)

- [ ] **Step 7: Run tests + clippy**

Run: `cd rust && cargo nextest run -p runtime -E 'test(motor_mask_round_trips)' && cargo clippy -p runtime -- -D warnings`
Expected: PASS, no warnings.

- [ ] **Step 8: Commit**

```bash
git add rust/runtime/src/piece_ring.rs
git commit -m "feat(runtime): add motor_mask to PieceEntry (in the reserved word)"
```

---

## Task A2: `stepper_sel_from_mask` helper (single-bit + fail-loud)

**Files:**
- Modify: `rust/runtime/src/piece_ring.rs` (add helper near `PieceEntry`)
- Reference: `rust/runtime/src/step_queue.rs:35` (`STEPPER_SEL_ALL: u8 = 0`)

- [ ] **Step 1: Write the failing test**

Append to the piece_ring test module:

```rust
#[test]
fn stepper_sel_from_mask_cases() {
    assert_eq!(stepper_sel_from_mask(0), Ok(0));            // all
    assert_eq!(stepper_sel_from_mask(0b0000_0001), Ok(1));  // motor 0 -> sel 1
    assert_eq!(stepper_sel_from_mask(0b0000_1000), Ok(4));  // motor 3 -> sel 4
    assert_eq!(stepper_sel_from_mask(0b1000_0000), Ok(8));  // motor 7 -> sel 8
    assert!(stepper_sel_from_mask(0b0000_0011).is_err());   // multi-bit -> fault
}
```

- [ ] **Step 2: Run it (expect FAIL — fn missing)**

Run: `cd rust && cargo nextest run -p runtime -E 'test(stepper_sel_from_mask_cases)'`
Expected: compile error.

- [ ] **Step 3: Implement**

Add to `rust/runtime/src/piece_ring.rs`:

```rust
/// Maps a single-motor `motor_mask` to a `StepEntry.stepper_sel`.
/// `0` => `STEPPER_SEL_ALL` (0). Exactly one bit set => `bit_index + 1`.
/// More than one bit set is rejected (overlays target one motor; YAGNI).
#[inline]
pub fn stepper_sel_from_mask(mask: u8) -> Result<u8, ()> {
    if mask == 0 {
        return Ok(0);
    }
    if mask.count_ones() != 1 {
        return Err(());
    }
    Ok(mask.trailing_zeros() as u8 + 1)
}
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo nextest run -p runtime -E 'test(stepper_sel_from_mask_cases)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/piece_ring.rs
git commit -m "feat(runtime): stepper_sel_from_mask helper (single-bit, fail-loud)"
```

---

## Task A3: Scope position-count commit to masked motors

**Files:**
- Modify: `rust/runtime/src/dispatch_stepper.rs:290-312` (`commit_position_count`)

**Read first:** `commit_position_count` (loops `axis.steppers`, advances each `position_count`) and `dispatch_correction.rs:199-218` `commit_motor_position_count` (advances one). This task generalizes the former to honor a mask.

- [ ] **Step 1: Write the failing test**

Add a runtime unit test (in `dispatch_stepper.rs`'s test module, or a new `dispatch_stepper/tests.rs` if it uses one — match the file's existing test convention). The test constructs an `AxisConfig` with 2 steppers and asserts:
- `commit_position_count_masked(axis, .., 0, delta)` advances BOTH steppers' `position_count` by `delta`.
- `commit_position_count_masked(axis, .., 0b10, delta)` advances ONLY stepper index 1.

```rust
#[test]
fn commit_masked_scopes_position_count() {
    let (axis, shared) = test_axis_with_two_steppers(); // build per this file's existing test helpers
    commit_position_count_masked(&axis, 0, &shared, 0, 5);
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 5);
    assert_eq!(axis.steppers[1].position_count.load(Ordering::Acquire), 5);
    commit_position_count_masked(&axis, 0, &shared, 0b10, 3);
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 5);   // unchanged
    assert_eq!(axis.steppers[1].position_count.load(Ordering::Acquire), 8);
}
```

If the file has no test scaffolding for `AxisConfig`, instead unit-test the mask→indices selection in isolation and verify the full behavior in the A4 integration test. Report which you chose.

- [ ] **Step 2: Run it (expect FAIL)**

Run: `cd rust && cargo nextest run -p runtime -E 'test(commit_masked_scopes_position_count)'`
Expected: FAIL (fn missing).

- [ ] **Step 3: Implement the masked commit**

Add alongside `commit_position_count`:

```rust
pub(crate) fn commit_position_count_masked(
    axis: &AxisConfig,
    axis_idx: usize,
    shared: &SharedState,
    motor_mask: u8,
    delta: i32,
) {
    if delta == 0 {
        return;
    }
    if shared.step_modes.get(axis_idx).map_or(false, |m| {
        m.load(Ordering::Acquire) == crate::state::StepMode::Modulated as u8
    }) {
        return;
    }
    for (i, stepper) in axis.steppers.iter().enumerate() {
        if motor_mask != 0 && (motor_mask & (1u8 << i)) == 0 {
            continue;
        }
        let prev = stepper.position_count.load(Ordering::Acquire);
        let Some(next) = prev.checked_add(delta) else {
            raise_position_count_overflow(shared, axis_idx);
            return;
        };
        stepper.position_count.store(next, Ordering::Release);
    }
}
```

Leave the existing `commit_position_count` in place for now (A4 switches `dispatch_pulse` to the masked version; the old one is removed in Phase C if unused).

- [ ] **Step 4: Run tests + clippy**

Run: `cd rust && cargo nextest run -p runtime -E 'test(commit_masked)' && cargo clippy -p runtime -- -D warnings`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/dispatch_stepper.rs
git commit -m "feat(runtime): mask-scoped position-count commit"
```

---

## Task A4: Thread `motor_mask` through `dispatch_pulse` (step-sel + commit scope)

**Files:**
- Modify: `rust/runtime/src/dispatch_stepper.rs:154-288` (`dispatch_pulse`: add `motor_mask: u8` param; line 258 `stepper_sel`; line 287 commit)
- Modify: caller of `dispatch_pulse` in `rust/runtime/src/engine.rs` (around the `dispatch_axis`/tick path, ~line 472)

**Read first:** the full `dispatch_pulse` (154-288) and the engine tick that calls it (`engine.rs:416-483`).

- [ ] **Step 1: Write the failing test**

Add an engine-level test that pushes one normal piece (`motor_mask: 0`) and one overlay piece (`motor_mask: 0b10`) and asserts step emission targets all motors vs only stepper 1. Match the runtime's existing engine test harness (look in `rust/runtime/src/engine` tests or `motion_core` tests for how pieces are pushed and steps observed). If a step-capture harness exists (e.g., a mock `StepQueue`), assert the `stepper_sel` of emitted `StepEntry`s: `0` (ALL) for the normal piece, `2` (motor 1) for the overlay. Name it `dispatch_pulse_honors_motor_mask`.

(If no engine-level step-capture harness exists, assert via `position_count`: after a normal piece both steppers' counts move; after an overlay only the masked one moves — reusing A3's helper through the real tick.)

- [ ] **Step 2: Run it (expect FAIL)**

Run: `cd rust && cargo nextest run -p runtime -E 'test(dispatch_pulse_honors_motor_mask)'`
Expected: FAIL.

- [ ] **Step 3: Add `motor_mask` param + derive `stepper_sel` + masked commit**

In `dispatch_pulse` signature add `motor_mask: u8,` (after `axis_idx`/`axis`). Near the top (after the `microstep_distance` finite check), derive the selector once and fault on multi-bit:

```rust
    let stepper_sel = match crate::piece_ring::stepper_sel_from_mask(motor_mask) {
        Ok(sel) => sel,
        Err(()) => {
            raise_multi_motor_mask(shared, axis_idx, motor_mask);
            return;
        }
    };
```

Replace line 258 `stepper_sel: crate::step_queue::STEPPER_SEL_ALL,` with `stepper_sel,`. Replace the two `commit_position_count(axis, axis_idx, shared, <delta>)` calls (the overflow-partial at ~268 and the final at ~287) with `commit_position_count_masked(axis, axis_idx, shared, motor_mask, <delta>)`.

Add a fault raiser in the same module as the other `raise_*` (mirror `raise_steps_per_sample_exceeded`): `raise_multi_motor_mask(shared, axis_idx, mask)` — define a new error code `KALICO_ERR_MULTI_MOTOR_MASK` in `rust/runtime/src/error.rs` (next free code near -140..-145; e.g. `-143`) and a corresponding `raise_*` in `rust/runtime/src/fault.rs`/wherever `raise_steps_per_sample_exceeded` lives. Follow that function's exact pattern.

- [ ] **Step 4: Pass the active piece's mask from the engine tick**

In `engine.rs` where `dispatch_pulse`/`dispatch_axis` is invoked (around 472), read the active piece's mask. The active piece is `axis.armed` after `get_position_and_velocity`; pass `axis.armed.motor_mask` (confirm the field is reachable — if `get_position_and_velocity` consumes/rotates the armed piece, capture the mask from the piece BEFORE evaluation via `axis.ring.peek(storage).map(|p| p.motor_mask).unwrap_or(0)`). Thread that `u8` into `dispatch_pulse`.

- [ ] **Step 5: Run tests + clippy + MCU build**

Run: `cd rust && cargo nextest run -p runtime -E 'test(dispatch_pulse_honors_motor_mask)' && cargo clippy -p runtime -- -D warnings`
Then: `./scripts/ci.sh rust-mcu-h7` (confirms the no_std MCU target still links).
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/dispatch_stepper.rs rust/runtime/src/engine.rs rust/runtime/src/error.rs rust/runtime/src/fault.rs
git commit -m "feat(runtime): dispatch_pulse honors motor_mask (step-sel, commit scope, multi-bit fault)"
```

---

## Task A5: Gate `p_prev`/`v_prev` on `mask == 0`

**Files:**
- Modify: `rust/runtime/src/engine.rs:449-454` (the `axis.p_prev = p_end; axis.v_prev = v_end;` write)

- [ ] **Step 1: Write the failing test**

Add an engine test `overlay_piece_does_not_advance_p_prev`: push a normal piece, record `motor_state(axis).0` (the `p_prev`); push an overlay piece (`motor_mask != 0`) of nonzero displacement; assert `p_prev` is UNCHANGED after the overlay, while the masked stepper's `position_count` DID change. Use the same harness as A4. (Position-of-record read via the engine's `motor_state(i)` → `(p_prev, v_prev)`.)

- [ ] **Step 2: Run it (expect FAIL — overlay currently advances p_prev)**

Run: `cd rust && cargo nextest run -p runtime -E 'test(overlay_piece_does_not_advance_p_prev)'`
Expected: FAIL.

- [ ] **Step 3: Gate the accumulator write**

At `engine.rs:449-454`, wrap the `p_prev`/`v_prev` update so it only runs for full-axis pieces. Capture the active mask (same source as A4 Step 4) into a local `active_mask` in that scope, then:

```rust
                Some((p_end, v_end)) => {
                    active = true;
                    let p_sample_start = axis.p_prev;
                    if active_mask == 0 {
                        axis.p_prev = p_end;
                        axis.v_prev = v_end;
                    }
                    (p_end, v_end, p_sample_start)
                }
```

Note `p_sample_start` still reads the pre-update `p_prev` so `dispatch_pulse`'s step math is unaffected for overlays (the overlay's steps come from `last_step_count` deltas, which the masked commit handles).

- [ ] **Step 4: Run tests + clippy + MCU build**

Run: `cd rust && cargo nextest run -p runtime -E 'test(overlay_piece_does_not_advance_p_prev) + test(dispatch_pulse_honors) + test(motor_mask)' && cargo clippy -p runtime -- -D warnings && ./scripts/ci.sh rust-mcu-h7`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/engine.rs
git commit -m "feat(runtime): overlay pieces leave the axis p_prev accumulator untouched"
```

---

## Task A6: Overlay step-frame register (free-running, no reset)

**Why:** A4 made masked pieces diff their steps against the per-axis
`last_step_count` and write it back — but an overlay is a *different curve* on
*one* motor; perturbing the axis frame would make the next full-axis move
mis-step the overlaid motor (correcting the offset back out). Overlays need their
own per-stepper "previous curve sample" register, free-running, never reset. A5's
test currently hides this by hand-fitting the overlay coeffs to the axis frame.

**Files:**
- Modify: `rust/runtime/src/stepping_state.rs` (`StepperRef`: add `overlay_step_frame`)
- Modify: `rust/runtime/src/dispatch_stepper.rs` (`dispatch_pulse`: mask-conditional frame)
- Modify: `rust/runtime/src/engine/tests.rs` (fix A5's test to a real 0-based overlay curve)

- [ ] **Step 1: Add the per-stepper register.** In `StepperRef` (the per-motor struct in `stepping_state.rs` that holds `position_count`/`phase_offset_*`), add `pub overlay_step_frame: AtomicI32` (match the `position_count` field's type/atomic + its `new()`/`reset()` initialization to `0`). It is NOT reset between overlays — initialize to 0 once.

- [ ] **Step 2: Write the failing test.** Rewrite A5's `overlay_piece_does_not_advance_p_prev` (and/or add `overlay_uses_own_step_frame`) so the overlay piece is a real **0-based** buzz curve (coeffs starting at 0, e.g. `[0.0, x, x, 0.0]`-style net-zero), NOT hand-fitted to the axis frame. Assert:
  - after the overlay, `axis.last_step_count` is UNCHANGED (the overlay didn't touch the axis frame);
  - the targeted stepper's `position_count` moved by the overlay's steps;
  - `p_prev` unchanged (from A5).
  Run it; expect FAIL (current A4 code writes `axis.last_step_count` for the overlay, so the axis frame changes).

- [ ] **Step 3: Make `dispatch_pulse` mask-conditional on the frame.** In `dispatch_pulse`, replace the single `axis.last_step_count` use with a frame chosen by the mask. For `mask == 0`: read/write `axis.last_step_count` (unchanged). For `mask != 0`: derive the motor index (`stepper_sel - 1`), and read/write that stepper's `overlay_step_frame`; do NOT touch `axis.last_step_count`. Apply the same selection to the overflow-rollback path (roll back the overlay frame, not the axis frame, for masked pieces). Concretely the `prev_step_count`/`target_step_count`/write-back trio and the two rollback assignments (`axis.last_step_count = ...`) become conditional on the mask. Keep the math identical; only the backing register changes.

- [ ] **Step 4: Run tests + clippy + MCU build.** `cd rust && cargo nextest run -p runtime && cargo clippy -p runtime --all-targets -- -D warnings && ./scripts/ci.sh rust-mcu-h7`. Expected PASS (the corrected test now passes; full suite green).

- [ ] **Step 5: Commit.**
```bash
git add rust/runtime/src/stepping_state.rs rust/runtime/src/dispatch_stepper.rs rust/runtime/src/engine/tests.rs
git commit -m "feat(runtime): overlays use a free-running per-stepper step frame, not the axis frame"
```

Note for Phase C: the old per-axis `correction_last_step_count` is deleted (C1); this per-stepper `overlay_step_frame` replaces it. Do NOT delete `overlay_step_frame` in C1.

---

# Phase B — Corrections through the async pump

## Task B1: `PlannerMsg::AxisOverlay` + `submit_axis_overlay`

**Files:**
- Modify: `rust/motion-bridge/src/planner.rs:64-94` (`PlannerMsg`), add `submit_axis_overlay` method near `submit_move:196-214`.
- Reference: `rust/motion-bridge/src/pump.rs:213-236` (`EnqueueMsg`, `PumpMsg`), `rust/motion-bridge/src/enqueue.rs` (how moves become `EnqueueMsg`).

**Read first:** how `submit_move`/the planner thread builds `EnqueueMsg` and how `run_pump`'s `PumpMsg::Enqueue` arm (pump.rs:414-520) consumes `EnqueueMsg { key, pieces: Vec<(PieceEntry, f64)>, fresh_stream, lead_secs }`. The overlay path produces pre-built `PieceEntry`s (cubics, mask stamped) and must reach that same `PumpMsg::Enqueue`.

- [ ] **Step 1: Write the failing test**

In `planner.rs`'s test module (or a new one), test that `submit_axis_overlay` sends a `PumpMsg::Enqueue` whose `EnqueueMsg.key` matches `(mcu_id, axis_idx)` and whose pieces all carry the given `motor_mask`. Use a test that captures the pump channel (mirror how existing planner tests assert sends). Name it `submit_axis_overlay_enqueues_masked_pieces`.

- [ ] **Step 2: Run it (expect FAIL)**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(submit_axis_overlay_enqueues_masked_pieces)'`
Expected: FAIL.

- [ ] **Step 3: Implement**

Decide the simplest routing: an overlay is pre-shaped, so it can go straight to the pump's `Enqueue` (it does not need the planner thread's trajectory shaping). Add a `Planner::submit_axis_overlay`:

```rust
pub fn submit_axis_overlay(
    &self,
    mcu_id: u32,
    axis_idx: u8,
    pieces: Vec<(crate::piece_ring_reexport::PieceEntry, f64)>, // (entry, host_secs)
    lead_secs: f64,
) -> Result<(), PlannerError> {
    self.pump_tx
        .send(crate::pump::PumpMsg::Enqueue(crate::pump::EnqueueMsg {
            key: crate::pump::AxisKey { mcu_id, axis: axis_idx },
            pieces,
            fresh_stream: true,
            lead_secs,
        }))
        .map_err(|_| PlannerError::ChannelClosed)
}
```

If the `Planner` does not hold the `pump_tx` directly (it sends `PlannerMsg` and the planner thread forwards to the pump), instead add `PlannerMsg::AxisOverlay { mcu_id, axis_idx, pieces, lead_secs }`, and in the planner thread's match (where it handles `PlannerMsg::Move`) forward it as the `PumpMsg::Enqueue` above. Use whichever the existing wiring supports — read `run` loop of the planner thread to confirm. Report which path you took. `fresh_stream: true` because overlays are submitted to a drained axis (callers `wait_moves` first).

- [ ] **Step 4: Run tests + clippy**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(submit_axis_overlay)' && cargo clippy -p motion-bridge -- -D warnings`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/planner.rs
git commit -m "feat(bridge): planner submit_axis_overlay routes masked pieces to the pump"
```

---

## Task B2: Build masked `PieceEntry`s from correction segments

**Files:**
- Modify: `rust/motion-bridge/src/correction.rs` `to_piece_entries:149-174` (stamp the mask) — OR add a sibling that stamps it.

- [ ] **Step 1: Write the failing test**

In `correction/tests.rs`, test that the entry-builder stamps `motor_mask` on every produced `PieceEntry`:

```rust
#[test]
fn overlay_entries_carry_motor_mask() {
    let pieces = vec![ProfilePiece { coeffs: [0.0,1.0,2.0,3.0], duration: 0.4 }];
    let entries = to_overlay_piece_entries(&pieces, |s| (s*1000.0) as u64, 10.0, 0b0000_0100);
    assert!(entries.iter().all(|e| e.motor_mask == 0b0000_0100));
}
```

- [ ] **Step 2: Run it (expect FAIL)**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(overlay_entries_carry_motor_mask)'`
Expected: FAIL.

- [ ] **Step 3: Implement**

Add `to_overlay_piece_entries(pieces, project, start_host_secs, motor_mask)` mirroring `to_piece_entries` but setting `motor_mask` on each `PieceEntry` (and `_reserved: [0;3]`). Keep `to_piece_entries` until Phase C deletes the old streamer.

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(overlay_entries_carry_motor_mask)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/correction.rs rust/motion-bridge/src/correction/tests.rs
git commit -m "feat(bridge): build overlay PieceEntries with motor_mask stamped"
```

---

## Task B3: Rewrite `submit_correction_sequence`/`adjust_motor` to async overlay submit

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:2153-2198` (both methods), drop `start_print_time` arg; remove the call into `stream_correction_entries`.

**Read first:** how the bridge gets the host-time anchor + `host_time_to_mcu_clock` (the same `router` used by `stream_correction_entries`), and how `submit_axis_overlay`/the planner handle is reachable from `PyMotionBridge`.

- [ ] **Step 1: Write the failing test**

Bridge-level Rust test is hard (needs a live planner); instead assert the new signature compiles and returns a duration without performing inline wire I/O. Add a focused test in `bridge.rs` tests if a planner test-double exists; otherwise rely on B-phase Python tests (B5) + the Phase-B gate. Document the choice. (No fake wire calls — the point is there is no inline streaming.)

- [ ] **Step 2: Implement**

Rewrite both to: plan the cubic pieces (`plan_correction_sequence`/`plan_correction_profile`), compute `motor_mask = 1u8 << motor_idx` (validate `motor_idx < MAX_MOTORS_PER_AXIS`, else `PyRuntimeError`), anchor at `router.host_now_secs() + LEAD` and project via `host_time_to_mcu_clock` into `to_overlay_piece_entries`, then `planner.submit_axis_overlay(...)` and return `total_duration`. New signatures (no `start_print_time`):

```rust
fn submit_correction_sequence(&self, py, mcu_handle, axis_idx, motor_idx, segments, speed, accel) -> PyResult<f64>
fn adjust_motor(&self, py, mcu_handle, axis_idx, motor_idx, delta_mm, speed, accel) -> PyResult<f64>
```

Use `MAX_MOTORS_PER_AXIS` (define it as a `pub const` in `rust/runtime/src/stepping_state.rs` near where motor count lives, exported so the bridge and any mask code share it). Pick `LEAD` = the existing `anchor::DEFAULT_LEAD_SECS` (no new constant). Return `Ok(total_duration)`.

- [ ] **Step 3: Build + clippy**

Run: `cd rust && cargo build -p motion-bridge && cargo clippy -p motion-bridge -- -D warnings`
Expected: builds (any Python wrapper arity mismatch is fixed in B4).

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs rust/runtime/src/stepping_state.rs
git commit -m "feat(bridge): correction submit is async (masked overlay via pump), no inline streaming"
```

---

## Task B4: Update the Python wrapper signatures (drop `start_print_time`)

**Files:**
- Modify: `klippy/motion_bridge.py` `submit_correction_sequence`/`adjust_motor` (remove `start_print_time`).

- [ ] **Step 1: Edit**

`MotionBridgeWrapper.submit_correction_sequence(self, mcu_id, axis_idx, motor_idx, segments, speed, accel)` and `adjust_motor(self, mcu_id, axis_idx, motor_idx, delta_mm, speed, accel)` — forward the same args (no `start_print_time`, no `float(start_print_time)`).

- [ ] **Step 2: Verify import**

Run: `python -c "import klippy.motion_bridge"`
Expected: no error.

- [ ] **Step 3: Commit**

```bash
git add klippy/motion_bridge.py
git commit -m "feat(host): drop start_print_time from the correction wrapper"
```

---

## Task B5: Strip the timeline band-aids from `Motion`; callers drain via `wait_moves`

**Files:**
- Modify: `klippy/motion.py` `_stream_correction_on_timeline`/`submit_correction`/`submit_motor_adjust:241-268`
- Modify: `klippy/extras/motors_sync.py:380-394`, `klippy/extras/motor_adjust.py:38-48`, `klippy/extras/force_move.py:41-50`
- Test: `test/test_toolhead_shim.py`

- [ ] **Step 1: Rewrite the tests**

Replace the three correction tests in `test/test_toolhead_shim.py` so the `_RecordingBridge` records a plain async submit (no `wait_moves`/`dwell`/`start_print_time`): `submit_correction_sequence(mcu_id, axis_idx, motor_idx, segments, speed, accel)` records and returns a duration; `submit_correction(...)` returns that duration; assert NO `dwell`/`wait_moves` were called from inside `submit_correction` (the wrapper just forwards). Keep `test_get_last_move_time_uses_motion_lead`.

```python
def test_submit_correction_is_a_plain_async_forward():
    th = _make_correction_toolhead(0.6)
    dur = th.submit_correction(7, 1, 0, [0.3, -0.3], 80.0, 5000.0)
    call = th.bridge.last_call
    assert call["kind"] == "correction"
    assert (call["mcu_id"], call["axis_idx"], call["motor_idx"]) == (7, 1, 0)
    assert dur == pytest.approx(0.6)
    assert th.bridge.waits == 0 and th.bridge.dwells == []
```

- [ ] **Step 2: Run it (expect FAIL — current code waits/dwells)**

Run: `python -m pytest test/test_toolhead_shim.py::test_submit_correction_is_a_plain_async_forward -v`
Expected: FAIL.

- [ ] **Step 3: Reduce `Motion` to plain forwards**

Replace `_stream_correction_on_timeline` + `submit_correction` + `submit_motor_adjust` with:

```python
    def submit_correction(self, mcu_id, axis_idx, motor_idx, segments, speed, accel):
        return self.bridge.submit_correction_sequence(
            mcu_id, axis_idx, motor_idx, segments, speed, accel
        )

    def submit_motor_adjust(self, mcu_id, axis_idx, motor_idx, delta_mm, speed, accel):
        return self.bridge.adjust_motor(
            mcu_id, axis_idx, motor_idx, delta_mm, speed, accel
        )
```

- [ ] **Step 4: Callers wait via `wait_moves`**

In `motors_sync.py` `manual_move`, after `submit_correction(...)`, replace the `start/deadline/reactor.pause` busy-wait with `self.toolhead.wait_moves()`:

```python
    def manual_move(self, mcu_stepper, moves):
        segments = [m for m in moves if abs(m) >= 0.00001]
        if not segments:
            return
        name = mcu_stepper.get_name()
        mcu_id, axis_idx, motor_idx = self.toolhead.get_motor_binding(name)
        self.toolhead.submit_correction(
            mcu_id, axis_idx, motor_idx, segments,
            self.travel_speed, self.travel_accel)
        self.toolhead.wait_moves()
```

In `motor_adjust.py` `adjust`, replace the busy-wait after `submit_motor_adjust(...)` with `toolhead.wait_moves()`. In `force_move.py` `manual_move`, it just returns `toolhead.submit_correction(...)`; leave its callers (`angle.py`, `probe_eddy_current.py`) — they already `wait_moves`/discard. Remove `SETTLE_PAD`/`ADJUST_SETTLE_PAD` if now unused (grep first).

- [ ] **Step 5: Run tests + ruff**

Run: `python -m pytest test/test_toolhead_shim.py -v && ruff check klippy/motion.py klippy/extras/motors_sync.py klippy/extras/motor_adjust.py klippy/extras/force_move.py && ruff format --check <same files>`
Expected: PASS / clean (ruff-format if needed).

- [ ] **Step 6: Commit**

```bash
git add klippy/motion.py klippy/extras/motors_sync.py klippy/extras/motor_adjust.py klippy/extras/force_move.py test/test_toolhead_shim.py
git commit -m "feat(host): corrections submit async; callers drain via wait_moves"
```

---

# Phase C — Delete the bespoke correction path

## Task C1: Delete the MCU correction ring + dispatcher

**Files (per the seam map):**
- Delete file: `rust/runtime/src/dispatch_correction.rs`; remove `pub mod dispatch_correction;` (`lib.rs:30`).
- `rust/runtime/src/stepping_state.rs`: remove `CORRECTION_RING_DEPTH:14`, `correction_ring:79`, `correction_armed`/`correction_motor_idx`/`correction_last_step_count` fields (`:82,97-104`) and their `new()`/`reset()` lines.
- `rust/runtime/src/engine.rs`: remove `CORRECTION_RING_DEPTH` storage math (`:170,179`), `correction_ring` init (`:189-192`), `discard_pending` correction loops (`:240-247`), `write_correction_piece`/`commit_correction` (`:304-351`), the `tick_correction` call (`:416-430`), `correction_retired_counts()` (`:505-513`).
- `rust/runtime/src/log_codes.rs:202`: remove `EVENT_MOTION_CORRECTION_*`.
- Remove the now-unused `commit_position_count` (old, unmasked) if A4 left it dead.

- [ ] **Step 1: Delete + fix compile**

Make the deletions. Then `cd rust && cargo build -p runtime 2>&1 | tail -40` and resolve each reference until it builds. Re-run any storage-size asserts (`KALICO_RUNTIME_PIECE_RING_SIZE`/`STORAGE_SIZE` math that subtracted `CORRECTION_RING_DEPTH`).

- [ ] **Step 2: Tests + clippy + MCU build**

Run: `cd rust && cargo nextest run -p runtime && cargo clippy -p runtime -- -D warnings && ./scripts/ci.sh rust-mcu-h7`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add -A rust/runtime/
git commit -m "refactor(runtime): delete the separate correction ring + dispatcher"
```

---

## Task C2: Delete the correction wire message + host streamer + drain plumbing

**Files (per the seam map):**
- `rust/kalico-protocol/src/messages.rs`: remove `PushCorrectionPieces[Response]` enum entries (`:21-22`), decode arms (`:64-65`), structs+impls (`:263-337`); `schema_def.rs:110,124`.
- `rust/motion-bridge/src/bridge.rs`: remove `stream_correction_entries:4181-4289`, `correction_drain` field (`:619`), its heartbeat clone (`:2866`), and the `CORRECTION_RING_DEPTH` usages in ring-depth calc/tests (`:638,708-737`).
- Delete `rust/motion-bridge/src/correction.rs`'s now-unused `to_piece_entries`/`chunk_correction_messages` (keep `ProfilePiece`/`plan_correction_*`/`to_overlay_piece_entries` used by B). If the whole file isn't fully used, keep only the live parts.
- Heartbeat `correction_retired_counts`: `kalico-host-rt/.../events.rs:314-317`, `kalico_native.rs:284`, `runtime_events.rs:64`; `kalico-ethercat-rt/src/wire.rs:276`; `kalico-c-api/src/runtime_ffi.rs:1008,1246,1262`; regenerate `kalico-c-api/include/kalico_runtime.h` (cbindgen — do NOT hand-edit; run `cargo run -p kalico-c-api --bin gen-headers --no-default-features --features "host,header-runtime"`).
- C: `src/kalico_dispatch.c:212,545` (PushCorrectionPieces handler), `src/runtime_tick.c:270` (correction-retired call).
- Drop the now-unused `drain.rs` `reset_axis`/`room`/`wait_room` if only corrections used them (grep; the main ring uses `DrainSync` too — keep what `wait_moves`/pump use).

- [ ] **Step 1: Delete + fix compile across crates**

Make deletions crate by crate; build each (`cargo build -p kalico-protocol`, `-p motion-bridge`, `-p kalico-host-rt`, `-p kalico-c-api`). Regenerate the cbindgen header. For C, `make clean` then a sanity compile via `./scripts/ci.sh rust-mcu-h7` (and f446 if scripted).

- [ ] **Step 2: Workspace gate**

Run: `cd rust && cargo nextest run && cargo clippy --workspace -- -D warnings && cargo fmt --all --check`
Expected: PASS / clean.

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "refactor: delete correction wire message, host streamer, and retired-heartbeat plumbing"
```

---

## Task C3: Full gate

- [ ] **Step 1:** `cd rust && cargo nextest run` → all pass.
- [ ] **Step 2:** `./scripts/ci.sh quick` → `5 pass 0 fail`.
- [ ] **Step 3:** `./scripts/ci.sh py` (or local `python -m pytest test/`) → green.
- [ ] **Step 4:** `cd rust && cargo fmt --all --check` → clean.
- [ ] **Step 5:** Confirm the deletion: `grep -rn "CORRECTION_RING_DEPTH\|PushCorrectionPieces\|stream_correction_entries\|dispatch_correction\|correction_retired\|correction_ring" rust/ src/ klippy/ | grep -v "_reserved\|motor_mask"` → no output (or only the design/plan docs).
- [ ] **Step 6:** Net diff is deep red: `git diff --stat <phaseA-base>..HEAD` shows far more deletions than additions.

---

## Self-Review

**Spec coverage:** motor mask + MAX_MOTORS_PER_AXIS → A1/A2/B3. PieceEntry `_reserved` byte → A1. Evaluator step-sel/commit/p_prev keyed off mask → A3/A4/A5. Single-bit + fail-loud multi-bit → A2/A4. Direct-axis planner submit → B1. Async host submit → B3/B4/B5. Delete bespoke path → C1/C2. Position-of-record safety (p_prev untouched) → A5. ✓

**Placeholder scan:** Two tasks (A4 Step 4, B1 Step 3) instruct the implementer to read a cited function and choose the matching wiring — these are integration seams with one correct shape per the existing code; the exact transformation and representative code are given. Flagged explicitly rather than guessed. Test harness references (A3/A4/A5) say "match the file's existing test convention" because the runtime's engine-test scaffolding must be reused, not reinvented.

**Type consistency:** `motor_mask: u8` everywhere; `stepper_sel_from_mask(u8) -> Result<u8,()>`; `commit_position_count_masked(.., motor_mask, delta)`; `submit_axis_overlay`/`to_overlay_piece_entries(.., motor_mask)`; `MAX_MOTORS_PER_AXIS` single source in `stepping_state.rs`; host signatures drop `start_print_time` consistently in B3/B4/B5.
