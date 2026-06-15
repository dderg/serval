# Single-Motor Overlay Move (`manual_move`) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a relative single-motor "nudge" a normal piece on the main motion ring (carrying a per-piece `motor_mask`), planned closed-form (constant-velocity / trapezoid, no solver, no jerk), dispatched through the existing pipeline, off the axis position-of-record — and collapse motors_sync / z_tilt / quad_gantry_level / force_move onto that one primitive.

**Architecture:** A nudge enters as `PlannerMsg::Nudge` (sibling to `HomeDrip`), is profiled in closed form into cubic `ShapedSegment`s stamped with `motor_mask`, and flows through the unchanged `enqueue_segment` → pump → wire path. The MCU evaluates masked pieces by resetting that motor's overlay step-frame at piece arm and stepping only that motor, never advancing `p_prev`. Host API is the mainline-compatible `force_move.manual_move`.

**Tech Stack:** Rust (runtime = MCU engine; motion-bridge = host planner via PyO3); Python (klippy host shim + extras). Tests: `cargo nextest` (Rust), `pytest`/`./scripts/ci.sh py` (Python).

**Spec:** `docs/superpowers/specs/2026-06-15-single-motor-overlay-move-design.md`

---

## File Structure

**Runtime (MCU engine), `rust/runtime/src/`:**
- `engine.rs` — tick loop; add overlay-aware `p_sample_start` + frame reset-on-arm (Task A1).
- `dispatch_stepper.rs` — masked step output; reset frame on overlay arm (Task A1).
- `error.rs` — add `OVERLAY_UNSUPPORTED` code + `FaultCode` variant (Task A2).
- `fault_helpers.rs` — `raise_overlay_unsupported` (Task A2).

**Motion-bridge (host planner), `rust/motion-bridge/src/`:**
- `nudge.rs` — NEW: `plan_nudge_profile` closed-form planner (Task B2). Replaces `correction.rs`.
- `enqueue.rs` — thread `motor_mask` from `ShapedSegment` into `PieceEntry` (Task B1).
- `planner.rs` — `PlannerMsg::Nudge` + `NudgeParams` + `submit_nudge` + run-loop arm (Task B3).
- `bridge.rs` — `submit_nudge` pymethod; delete `pump_correction_overlay`/`submit_correction_sequence`/`adjust_motor` (Task B4).
- `correction.rs` (+ `correction/tests.rs`) — DELETED (Task B5).

**Trajectory, `rust/trajectory/src/lib.rs`:** add `motor_mask: u8` to `ShapedSegment` (Task B1).

**Python, `klippy/`:**
- `motion.py` — `submit_nudge` replaces `submit_correction`/`submit_motor_adjust` (Task C1).
- `extras/force_move.py` — `manual_move` → resolve + fail-loud-if-disabled + `submit_nudge` (Task C2).
- `extras/z_tilt.py`, `extras/z_tilt_ng.py` — `ZAdjustHelper` → `force_move.manual_move` (Task C3).
- `extras/motors_sync.py` — `StepperManualMove.manual_move` → loop `submit_nudge` (Task C4).
- `extras/motor_adjust.py` — DELETED (Task C5).
- `test/test_toolhead_shim.py` — update bridge stub + tests (Tasks C1, C2).

**Execution order:** A → B → C (Python depends on the bridge `submit_nudge`, which depends on runtime mask semantics). Within a phase, tasks are mostly independent except where noted.

---

## Phase A — Runtime (MCU evaluator)

### Task A1: Overlay frame reset-on-arm + overlay-aware `p_sample_start`

**Why:** The merged eval (`dispatch_stepper.rs:211-220`) interprets the overlay curve as an absolute position diffed against `overlay_step_frame`, which only works if the host tracks cumulative position. The host tracks nothing and sends a relative `0 → Δ` curve, so the MCU must (a) reset that motor's `overlay_step_frame` to `0` when an overlay piece arms (so `target = round(C(t_start)) = 0` at the seam — no jump, and each piece emits exactly `round(Δ)`), and (b) feed `dispatch_pulse` a `p_sample_start` that is the overlay curve's window-start value, not the axis `p_prev` (which the position-book gate at `engine.rs:323` freezes for overlay pieces, smearing step timing).

**Files:**
- Read first: `rust/runtime/src/motion_core.rs` (the `get_position_and_velocity` arming path + the `armed` field it mutates), `rust/runtime/src/engine.rs:303-365` (tick loop), `rust/runtime/src/dispatch_stepper.rs:156-322`.
- Modify: `rust/runtime/src/engine.rs`, `rust/runtime/src/dispatch_stepper.rs`.
- Test: `rust/runtime/src/dispatch_stepper_tests.rs` (create if absent; unit tests live in a separate file per CLAUDE.md).

- [ ] **Step 1: Read the arming path.** Read `motion_core.rs` `get_position_and_velocity` and how `axis.armed` transitions when a new piece becomes active. Identify the exact point where "a new piece just armed" is observable in `engine.rs::tick` (e.g. the `armed` segment-id/pointer changing, or `motion_core` returning an arm signal). You need a boolean `overlay_just_armed` available in the tick body for axis `i`.

- [ ] **Step 2: Write the failing test (reset-on-arm + exact `round(Δ)`).**

In `rust/runtime/src/dispatch_stepper_tests.rs`:

```rust
// Two consecutive overlay pieces on the same motor, each a relative 0 -> +Δ curve.
// Each must emit round(Δ) steps independently (frame reset at arm), with no
// negative jump at the second piece's first sample.
#[test]
fn overlay_piece_resets_frame_at_arm_and_emits_round_delta() {
    let mut h = OverlayHarness::new_single_motor(/*mstep_mm=*/0.01);
    // Piece 1: relative curve 0 -> 0.50 mm  => +50 microsteps.
    h.arm_overlay_piece(/*motor_idx=*/1, /*delta_mm=*/0.50);
    let s1 = h.run_piece_collect_signed_steps();
    assert_eq!(s1, 50);
    // Frame was left at round(0.50/0.01)=50; arming piece 2 must reset it to 0.
    h.arm_overlay_piece(1, 0.50);
    let s2 = h.run_piece_collect_signed_steps();
    assert_eq!(s2, 50, "second piece must reset frame and emit +50, not 0");
    // position_count is cumulative: +100; p_prev untouched.
    assert_eq!(h.position_count(1), 100);
    assert_eq!(h.p_prev(), 0.0);
}
```

`OverlayHarness` is a thin test fixture you write around `AxisConfig` + `dispatch_axis` (mirror existing `dispatch_stepper` test fixtures already in the runtime crate — grep `dispatch_axis` in `rust/runtime/src` tests for the closest existing harness and copy its setup).

- [ ] **Step 3: Run it — expect FAIL.**

Run: `cd rust && cargo nextest run -p runtime -E 'test(overlay_piece_resets_frame_at_arm_and_emits_round_delta)'`
Expected: FAIL (second piece emits a negative jump / wrong count under the current absolute interpretation).

- [ ] **Step 4: Implement reset-on-arm.** In `dispatch_stepper.rs::dispatch_pulse`, add an `overlay_just_armed: bool` parameter (threaded from `dispatch_axis`, threaded from `engine.rs::tick`). When `overlay_motor_idx.is_some() && overlay_just_armed`, store `0` into that motor's `overlay_step_frame` **before** `let prev_step_count = load_step_frame(axis);` (line 211), so `prev_step_count` reads `0`:

```rust
if overlay_just_armed {
    if let Some(idx) = overlay_motor_idx {
        if let Some(stepper) = axis.steppers.get(idx) {
            stepper.overlay_step_frame.store(0, Ordering::Release);
        }
    }
}
let prev_step_count = load_step_frame(axis);
```

- [ ] **Step 5: Implement overlay-aware `p_sample_start`.** In `engine.rs::tick`, replace the single `let p_sample_start = axis.p_prev;` (line 322) so that for an overlay active piece it is the overlay curve's window-start value (chained per tick, reset to the curve start on arm), not the frozen `p_prev`. Track a per-axis `overlay_last_p: f32` on `AxisConfig` (init `0.0`): on overlay arm set it to `0.0`; each overlay tick use it as `p_sample_start` then set `overlay_last_p = p_end`. For `mask == 0`, keep `p_sample_start = axis.p_prev`. Pass `overlay_just_armed` into `dispatch_axis`/`dispatch_pulse`.

- [ ] **Step 6: Run the test — expect PASS.**

Run: `cd rust && cargo nextest run -p runtime -E 'test(overlay_piece_resets_frame_at_arm_and_emits_round_delta)'`
Expected: PASS.

- [ ] **Step 7: Add a symmetric-buzz test (net zero).**

```rust
#[test]
fn symmetric_buzz_nets_position_count_to_zero() {
    let mut h = OverlayHarness::new_single_motor(0.01);
    h.arm_overlay_piece(1, 0.50);
    h.run_piece_collect_signed_steps();
    h.arm_overlay_piece(1, -0.50);
    h.run_piece_collect_signed_steps();
    assert_eq!(h.position_count(1), 0);
    assert_eq!(h.p_prev(), 0.0);
}
```

Run: `cd rust && cargo nextest run -p runtime -E 'test(symmetric_buzz_nets_position_count_to_zero)'`
Expected: PASS.

- [ ] **Step 8: Run the full runtime suite (no regressions on `mask == 0`).**

Run: `cd rust && cargo nextest run -p runtime`
Expected: all pass (the `mask == 0` path is unchanged).

- [ ] **Step 9: Commit.**

```bash
git add rust/runtime/src/engine.rs rust/runtime/src/dispatch_stepper.rs rust/runtime/src/dispatch_stepper_tests.rs
git commit -m "feat(runtime): reset overlay step-frame at piece arm; overlay-aware p_sample_start"
```

### Task A2: `OVERLAY_UNSUPPORTED` error code + raise helper (contract)

**Why:** The host stamps `motor_mask` on every piece uniformly; an MCU that cannot address an individual motor (EtherCAT/servo) must reject an overlay piece loudly at decode rather than drop it silently. Bare-metal honors it; this task defines the wire-stable code + the raise helper so the contract exists.

**Files:**
- Modify: `rust/runtime/src/error.rs` (constant + `FaultCode` variant), `rust/runtime/src/fault_helpers.rs` (raise helper).
- Test: `rust/runtime/src/error.rs` inline `#[cfg(test)]` is not allowed (separate file rule) — add to `rust/runtime/src/fault_helpers_tests.rs` (create if absent).

- [ ] **Step 1: Write the failing test.**

In `rust/runtime/src/fault_helpers_tests.rs`:

```rust
#[test]
fn overlay_unsupported_sets_fault_code_and_detail() {
    let shared = SharedState::new_for_test();
    raise_overlay_unsupported(&shared, /*axis_idx=*/2, /*mask=*/0b0000_0010);
    assert_eq!(shared.last_error.load(Ordering::Acquire), -314);
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!((detail >> 16) & 0xFF, 2);
    assert_eq!(detail & 0xFF, 0b0000_0010);
}
```

(Mirror the existing `raise_multi_motor_mask` test if one exists; copy its `SharedState` construction.)

- [ ] **Step 2: Run it — expect FAIL** (`raise_overlay_unsupported` undefined).

Run: `cd rust && cargo nextest run -p runtime -E 'test(overlay_unsupported_sets_fault_code_and_detail)'`
Expected: FAIL (cannot find function).

- [ ] **Step 3: Add the code + variant.** In `rust/runtime/src/error.rs`, after `KALICO_ERR_PHASE_MOTOR_UNMAPPED` (the last `-31x`), add:

```rust
pub const KALICO_ERR_OVERLAY_UNSUPPORTED: i32 = -314;
```

and in `enum FaultCode` after `PhaseMotorUnmapped = -313,`:

```rust
    OverlayUnsupported = -314,
```

- [ ] **Step 4: Add the raise helper.** In `rust/runtime/src/fault_helpers.rs`, mirroring `raise_multi_motor_mask`:

```rust
#[inline]
pub fn raise_overlay_unsupported(shared: &SharedState, axis_idx: usize, mask: u8) {
    let detail = ((axis_idx as u32 & 0xFF) << 16) | u32::from(mask);
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::OverlayUnsupported.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::OverlayUnsupported, detail);
}
```

- [ ] **Step 5: Run the test — expect PASS.**

Run: `cd rust && cargo nextest run -p runtime -E 'test(overlay_unsupported)'`
Expected: PASS.

- [ ] **Step 6: Commit.**

```bash
git add rust/runtime/src/error.rs rust/runtime/src/fault_helpers.rs rust/runtime/src/fault_helpers_tests.rs
git commit -m "feat(runtime): add OVERLAY_UNSUPPORTED (-314) fault code + raise helper"
```

---

## Phase B — Motion-bridge (host planner + dispatch)

### Task B1: `motor_mask` on `ShapedSegment`, threaded into `PieceEntry`

**Why:** Today `enqueue.rs` hardcodes `motor_mask: 0` when building every `PieceEntry` (line ~195). The nudge needs to carry a non-zero mask through the same dispatch path, so `ShapedSegment` must carry the mask and `enqueue_segment` must stamp it.

**Files:**
- Modify: `rust/trajectory/src/lib.rs:71-78` (`ShapedSegment`), all construction sites (`rust/trajectory/src/emit_shaped.rs:186`, `rust/trajectory/src/streaming/emit.rs:442`, test sites in `rust/trajectory` and `rust/motion-bridge`), `rust/motion-bridge/src/enqueue.rs`.
- Test: `rust/motion-bridge/src/enqueue/tests.rs`.

- [ ] **Step 1: Write the failing test.** In `rust/motion-bridge/src/enqueue/tests.rs`:

```rust
#[test]
fn enqueue_stamps_motor_mask_onto_every_piece() {
    let seg = test_shaped_segment_single_axis(/*axis=*/2, /*motor_mask=*/0b0000_0010);
    let cfgs = test_mcu_configs_one_axis(2);
    let msgs = enqueue_segment(&seg, &cfgs, 0.0, true, 0.0, 0.25, |_id, s| (s * 1e6) as u64, None);
    let all_pieces: Vec<_> = msgs.iter().flat_map(|m| m.pieces.iter()).collect();
    assert!(!all_pieces.is_empty());
    assert!(all_pieces.iter().all(|(p, _)| p.motor_mask == 0b0000_0010));
}
```

(Reuse or add `test_shaped_segment_single_axis` / `test_mcu_configs_one_axis` helpers near the existing enqueue tests; the existing tests already construct `ShapedSegment`s and `McuAxisConfig`s — copy those.)

- [ ] **Step 2: Run it — expect FAIL** (`ShapedSegment` has no `motor_mask`).

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(enqueue_stamps_motor_mask_onto_every_piece)'`
Expected: FAIL (compile error: no field `motor_mask`).

- [ ] **Step 3: Add the field.** In `rust/trajectory/src/lib.rs`:

```rust
#[derive(Debug, Clone)]
pub struct ShapedSegment {
    /// Index = axis registry index: 0..3 spatial, 3.. followers. Always ≥ 3.
    pub axes: Vec<nurbs::ScalarNurbs<f64>>,
    pub followers: Vec<geometry::segment::FollowerDemand>,
    pub t_start: f64,
    pub t_end: f64,
    /// 0 => normal full-axis move. Single bit i set => overlay on motor i.
    pub motor_mask: u8,
}
```

- [ ] **Step 4: Fix construction sites.** Add `motor_mask: 0` to every `ShapedSegment { .. }` literal: `rust/trajectory/src/emit_shaped.rs:186`, `rust/trajectory/src/streaming/emit.rs:442`, and any test constructors (`cargo build -p trajectory` will list them all). Normal moves are always `0`.

- [ ] **Step 5: Stamp the mask in `enqueue.rs`.** Thread `seg.motor_mask` into the inner `flatten_axis` helper and replace the hardcoded `motor_mask: 0` (around line 195) with the segment's mask:

```rust
PieceEntry {
    start_time,
    coeffs,
    duration: duration_f32,
    motor_mask: seg_motor_mask,   // was: 0
    _reserved: [0; 3],
},
```

where `seg_motor_mask` is `seg.motor_mask` passed down from `enqueue_segment`.

- [ ] **Step 6: Run the test + build — expect PASS.**

Run: `cd rust && cargo nextest run -p trajectory -p motion-bridge`
Expected: PASS (all existing tests still green; new test passes).

- [ ] **Step 7: Commit.**

```bash
git add rust/trajectory/src/lib.rs rust/trajectory/src/emit_shaped.rs rust/trajectory/src/streaming/emit.rs rust/motion-bridge/src/enqueue.rs rust/motion-bridge/src/enqueue/tests.rs
git commit -m "feat(motion-bridge): carry motor_mask on ShapedSegment, stamp it onto every PieceEntry"
```

### Task B2: Closed-form nudge planner `plan_nudge_profile`

**Why:** A nudge is a 1-D box (`accel == 0`, constant velocity) or trapezoid (`accel > 0`) move. No temporal solver. It produces a `ShapedSegment` (relative `0 → Δ` curve on the target axis, mask set) that flows through `enqueue_segment`.

**Files:**
- Read first: `rust/trajectory/src/emit_shaped.rs:150-190` (how a `ShapedSegment.axes` `ScalarNurbs` is built from control points) and the `nurbs` crate `ScalarNurbs` constructor for a piecewise cubic.
- Create: `rust/motion-bridge/src/nudge.rs`, `rust/motion-bridge/src/nudge/tests.rs`.
- Modify: `rust/motion-bridge/src/lib.rs` (or wherever modules are declared) to add `mod nudge;`.

- [ ] **Step 1: Write the failing tests.** In `rust/motion-bridge/src/nudge/tests.rs`:

```rust
use super::*;

#[test]
fn box_profile_when_accel_zero_is_constant_velocity() {
    // Δ=1.0 mm at 10 mm/s, accel=0 -> duration = 1.0/10 = 0.1 s, single phase.
    let segs = plan_nudge_profile(/*axis=*/2, /*delta_mm=*/1.0, /*speed=*/10.0, /*accel=*/0.0, /*mask=*/0b0000_0010).unwrap();
    let total: f64 = segs.iter().map(|s| s.t_end - s.t_start).sum();
    assert!((total - 0.1).abs() < 1e-9);
    assert!(segs.iter().all(|s| s.motor_mask == 0b0000_0010));
    assert!((axis_total_displacement(&segs, 2) - 1.0).abs() < 1e-6);
}

#[test]
fn trapezoid_profile_reaches_cruise_speed() {
    // Δ=10 mm, speed=100 mm/s, accel=1000 mm/s^2.
    let segs = plan_nudge_profile(2, 10.0, 100.0, 1000.0, 0b0000_0010).unwrap();
    assert!((axis_total_displacement(&segs, 2) - 10.0).abs() < 1e-6);
    let total: f64 = segs.iter().map(|s| s.t_end - s.t_start).sum();
    // accel_t = 100/1000 = 0.1; accel_d = 0.1*100 = ... full calc_move_time check:
    assert!(total > 0.0);
}

#[test]
fn short_move_degenerates_to_triangle_no_cruise() {
    // Δ too small to reach `speed`: still total displacement == Δ.
    let segs = plan_nudge_profile(2, 0.2, 100.0, 1000.0, 0b0000_0010).unwrap();
    assert!((axis_total_displacement(&segs, 2) - 0.2).abs() < 1e-6);
}
```

`axis_total_displacement(segs, axis)` is a test helper that sums each segment's target-axis curve net displacement (evaluate `ScalarNurbs` end − start).

- [ ] **Step 2: Run them — expect FAIL** (`plan_nudge_profile` undefined).

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(plan_nudge_profile) + test(box_profile) + test(trapezoid_profile) + test(short_move)'`
Expected: FAIL (module/function does not exist).

- [ ] **Step 3: Implement `calc_move_time` + `plan_nudge_profile`.** In `rust/motion-bridge/src/nudge.rs`, port mainline's `calc_move_time` (from `main` branch `klippy/extras/force_move.py:18-32`) and build the curve:

```rust
use trajectory::ShapedSegment;

/// Returns (accel_t, cruise_t, cruise_v). accel == 0 => single constant-velocity
/// phase (accel_t == 0, cruise_v == speed). Mirrors mainline calc_move_time.
fn calc_move_time(dist: f64, speed: f64, accel: f64) -> (f64, f64, f64) {
    let dist = dist.abs();
    if accel <= 0.0 || dist == 0.0 {
        let cruise_t = if speed > 0.0 { dist / speed } else { 0.0 };
        return (0.0, cruise_t, speed);
    }
    let max_cruise_v2 = dist * accel;
    let cruise_v = speed.min(max_cruise_v2.sqrt());
    let accel_t = cruise_v / accel;
    let accel_decel_d = accel_t * cruise_v;
    let cruise_t = (dist - accel_decel_d) / cruise_v;
    (accel_t, cruise_t.max(0.0), cruise_v)
}

/// Build the relative 0 -> Δ overlay move as cubic ShapedSegment(s) on `axis_idx`,
/// stamped with `motor_mask`. No temporal solver. accel == 0 => constant velocity.
pub fn plan_nudge_profile(
    axis_idx: u8,
    delta_mm: f64,
    speed: f64,
    accel: f64,
    motor_mask: u8,
) -> Result<Vec<ShapedSegment>, String> {
    if !delta_mm.is_finite() || !speed.is_finite() || speed <= 0.0 {
        return Err(format!("nudge: bad speed {speed} / delta {delta_mm}"));
    }
    let sign = if delta_mm < 0.0 { -1.0 } else { 1.0 };
    let (accel_t, cruise_t, cruise_v) = calc_move_time(delta_mm, speed, accel);
    // Phases (position is relative, starting at 0): accel (½·a·t², if accel_t>0),
    // cruise (linear at cruise_v), decel (mirror of accel). Build each as one
    // ShapedSegment whose target-axis ScalarNurbs is the exact cubic for that phase
    // (degree ≤ 2 polynomials are exact cubics), with all other axes constant and
    // followers empty. Chain t_start/t_end so phases are contiguous from 0.
    // ... construct per emit_shaped.rs pattern, applying `sign` to displacement ...
    todo!("build phase segments; see Step 1 reading")
}
```

Replace the `todo!` by building the phase `ShapedSegment`s. Use the `ScalarNurbs` construction pattern you read in Step 1 (emit_shaped.rs). The accel phase covers `[0, accel_t]` with position `½·a·t²·sign`; cruise `[accel_t, accel_t+cruise_t]` linear; decel `[.., +accel_t]` mirrored. For `accel_t == 0` emit a single cruise segment `[0, cruise_t]`. Every segment carries `motor_mask` and the target-axis curve; other axes hold the constant `0.0`.

- [ ] **Step 4: Declare the module + run the tests — expect PASS.**

Add `mod nudge;` (next to `mod correction;` in `lib.rs` for now — `correction` is deleted in B5).
Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(box_profile) + test(trapezoid_profile) + test(short_move)'`
Expected: PASS.

- [ ] **Step 5: Commit.**

```bash
git add rust/motion-bridge/src/nudge.rs rust/motion-bridge/src/nudge/tests.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(motion-bridge): closed-form plan_nudge_profile (box/trapezoid, no solver)"
```

### Task B3: `PlannerMsg::Nudge` + `submit_nudge` + run-loop arm

**Why:** A nudge must enter the planner thread (sibling to `HomeDrip`), profile via `plan_nudge_profile`, dispatch through the same closure, advance `last_move_time` (time book), and never touch `ShaperState`/`p_prev` (position book).

**Files:**
- Modify: `rust/motion-bridge/src/planner.rs` (enum variant + `NudgeParams` + `submit_nudge` method + run-loop arm).
- Test: `rust/motion-bridge/src/planner.rs` tests live in `rust/motion-bridge/src/planner/tests.rs` (per the `#[cfg(test)] mod tests;` at planner.rs:844-845 — point it at a sibling file if not already).

- [ ] **Step 1: Write the failing test.** In the planner test module:

```rust
#[test]
fn nudge_dispatches_masked_pieces_and_advances_last_move_time_only() {
    let dispatched = Arc::new(Mutex::new(Vec::<u8>::new())); // collect motor_masks
    let d2 = Arc::clone(&dispatched);
    let handle = PlannerHandle::spawn(test_planner_config(), Arc::new(move |seg| {
        d2.lock().unwrap().push(seg_motor_mask(seg));
        Ok(())
    }));
    let lmt0 = handle.last_move_time();
    let (tx, rx) = crossbeam_channel::bounded(1);
    handle.submit_nudge(NudgeParams {
        axis: 2, motor_mask: 0b0000_0010, delta_mm: 0.5, speed: 5.0, accel: 100.0, notify: tx,
    }).unwrap();
    rx.recv().unwrap().unwrap();
    assert!(handle.last_move_time() > lmt0, "nudge must advance the time book");
    assert!(dispatched.lock().unwrap().iter().all(|&m| m == 0b0000_0010));
}
```

(`seg_motor_mask(seg)` reads `seg.motor_mask`. Use the planner test config helpers already present in `planner/tests.rs`.)

- [ ] **Step 2: Run it — expect FAIL** (`NudgeParams`/`submit_nudge` undefined).

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(nudge_dispatches_masked_pieces_and_advances_last_move_time_only)'`
Expected: FAIL.

- [ ] **Step 3: Add `NudgeParams` + the enum variant.** In `planner.rs`:

```rust
pub struct NudgeParams {
    pub axis: u8,
    pub motor_mask: u8,
    pub delta_mm: f64,
    pub speed: f64,
    pub accel: f64,
    pub notify: crossbeam_channel::Sender<Result<(), String>>,
}
```

Add `Nudge(NudgeParams)` to `enum PlannerMsg` and a `"Nudge"` arm to the `tag` match in `run_loop` (around line 497-509).

- [ ] **Step 4: Add the `submit_nudge` method on `PlannerHandle`** (sibling to `home_drip`, planner.rs:303):

```rust
pub fn submit_nudge(&self, p: NudgeParams) -> Result<(), PlannerError> {
    self.sender
        .send(PlannerMsg::Nudge(p))
        .map_err(|_| PlannerError::ChannelClosed)
}
```

- [ ] **Step 5: Add the run-loop arm.** In `run_loop`'s `match msg`, add (mirroring the `HomeDrip` arm at planner.rs:762, but WITHOUT `state.reset` — a nudge must not disturb the main chain):

```rust
PlannerMsg::Nudge(p) => {
    let result = (|| -> Result<(), String> {
        let segs = crate::nudge::plan_nudge_profile(p.axis, p.delta_mm, p.speed, p.accel, p.motor_mask)?;
        let total_dur: f64 = segs.iter().map(|s| s.t_end - s.t_start).sum();
        for seg in &segs {
            dispatch(seg).map_err(|e| format!("nudge dispatch: {e}"))?;
        }
        advance_last_move_time(&last_move_time_bits, total_dur);
        Ok(())
    })();
    let _ = p.notify.send(result);
}
```

Note: the nudge's `ShapedSegment`s are dispatched directly (they are not appended to `ShaperState`), so `p_prev`/`t_appended` are untouched — the position book is left alone, only `last_move_time` (time book) advances.

- [ ] **Step 6: Run the test — expect PASS.**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(nudge_dispatches_masked_pieces_and_advances_last_move_time_only)'`
Expected: PASS.

- [ ] **Step 7: Commit.**

```bash
git add rust/motion-bridge/src/planner.rs rust/motion-bridge/src/planner/tests.rs
git commit -m "feat(motion-bridge): PlannerMsg::Nudge + submit_nudge (off position book, on time book)"
```

### Task B4: `submit_nudge` pymethod; delete the parallel-pump shortcut

**Why:** Expose one bridge primitive `submit_nudge` to Python (replacing `submit_correction_sequence` + `adjust_motor`), wired to `planner.submit_nudge`. Delete `pump_correction_overlay` (the hand-built `AxisKey` routing bug).

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (add `submit_nudge` pymethod ~near line 2134; delete `pump_correction_overlay` at 4138-4189, `submit_correction_sequence` at 2150-2164, `adjust_motor` at 2134-2148).
- Test: `rust/motion-bridge/src/bridge.rs` integration test module (or a `bridge/tests.rs`); if bridge methods are hard to unit-test in isolation, rely on the planner test (B3) + the Python test (C1) and assert here only the mask validation.

- [ ] **Step 1: Write the failing test (mask validation).** Add to the bridge test module:

```rust
#[test]
fn submit_nudge_rejects_multi_bit_mask() {
    // A multi-bit mask must be a loud error before anything is enqueued.
    let mask = 0b0000_0011u8;
    assert!(crate::piece_ring_mask_is_single_or_zero(mask) == false);
}
```

If no in-crate helper exists, assert via `runtime::piece_ring::stepper_sel_from_mask(0b11).is_err()`. (The pymethod itself wraps this; this test pins the validation rule the pymethod uses.)

- [ ] **Step 2: Run it — expect FAIL or compile error.**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(submit_nudge_rejects_multi_bit_mask)'`
Expected: FAIL.

- [ ] **Step 3: Add the `submit_nudge` pymethod.** In the `#[pymethods]` block (near `adjust_motor`, line 2134), add:

```rust
#[pyo3(signature = (mcu_id, axis_idx, motor_mask, delta_mm, speed, accel))]
fn submit_nudge(
    &self,
    _py: Python<'_>,
    mcu_id: u32,
    axis_idx: u8,
    motor_mask: u8,
    delta_mm: f64,
    speed: f64,
    accel: f64,
) -> PyResult<f64> {
    if runtime::piece_ring::stepper_sel_from_mask(motor_mask).is_err() {
        return Err(PyRuntimeError::new_err(format!(
            "submit_nudge: multi-bit motor_mask {motor_mask:#010b} not supported"
        )));
    }
    let (tx, rx) = crossbeam_channel::bounded(1);
    {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| PyRuntimeError::new_err("planner not initialized"))?;
        planner
            .submit_nudge(crate::planner::NudgeParams {
                axis: axis_idx, motor_mask, delta_mm, speed, accel, notify: tx,
            })
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
    }
    rx.recv()
        .map_err(|_| PyRuntimeError::new_err("nudge notify dropped"))?
        .map_err(PyRuntimeError::new_err)?;
    // Duration: closed-form total (cheap) so callers can dwell/track if needed.
    let (accel_t, cruise_t, _cruise_v) = crate::nudge::calc_move_time_pub(delta_mm, speed, accel);
    Ok(accel_t + cruise_t + accel_t)  // box: accel_t == 0 -> cruise_t
}
```

(Expose `calc_move_time` as `pub(crate) calc_move_time_pub` in `nudge.rs` so the timing formula lives in exactly one place. Do not duplicate the formula in `bridge.rs`.)

- [ ] **Step 4: Delete the shortcut.** Remove `pump_correction_overlay` (lines 4138-4189), `submit_correction_sequence` (2150-2164), and `adjust_motor` (2134-2148) from `bridge.rs`. `cargo build -p motion-bridge` will flag any remaining references.

- [ ] **Step 5: Run tests + clippy — expect PASS.**

Run: `cd rust && cargo nextest run -p motion-bridge && cargo clippy -p motion-bridge -- -D warnings`
Expected: PASS, no warnings, no references to the deleted methods.

- [ ] **Step 6: Commit.**

```bash
git add rust/motion-bridge/src/bridge.rs rust/motion-bridge/src/nudge.rs
git commit -m "feat(motion-bridge): submit_nudge pymethod; delete pump_correction_overlay shortcut"
```

### Task B5: Delete `correction.rs`

**Why:** `plan_correction_profile`/`plan_correction_sequence`/`to_overlay_piece_entries`/`ProfilePiece` (the second solver + direct PieceEntry builder) are fully replaced by `nudge.rs` + `enqueue_segment`.

**Files:**
- Delete: `rust/motion-bridge/src/correction.rs`, `rust/motion-bridge/src/correction/tests.rs`.
- Modify: `rust/motion-bridge/src/lib.rs` (remove `mod correction;`).

- [ ] **Step 1: Remove the module + files.**

```bash
git rm rust/motion-bridge/src/correction.rs rust/motion-bridge/src/correction/tests.rs
```

Remove `mod correction;` from `lib.rs`.

- [ ] **Step 2: Build — expect PASS** (nothing references `correction::*` after B4).

Run: `cd rust && cargo build -p motion-bridge`
Expected: PASS. If any reference remains, it is a leftover call site — replace it with `nudge`/`submit_nudge`.

- [ ] **Step 3: Full workspace test + clippy.**

Run: `cd rust && cargo nextest run && cargo clippy --workspace -- -D warnings && cargo fmt --all --check`
Expected: all green.

- [ ] **Step 4: Commit.**

```bash
git add -A
git commit -m "refactor(motion-bridge): delete correction.rs (replaced by nudge.rs + enqueue)"
```

---

## Phase C — Python (host shim + callers)

### Task C1: `Motion.submit_nudge` replaces `submit_correction`/`submit_motor_adjust`

**Files:**
- Modify: `klippy/motion.py:241-253`.
- Test: `klippy/test/test_toolhead_shim.py` (or wherever the repo keeps it — the file in this tree is `test/test_toolhead_shim.py`).

- [ ] **Step 1: Update the bridge stub + write the failing test.** In `test/test_toolhead_shim.py`, replace `_RecordingBridge.submit_correction_sequence`/`adjust_motor` with one `submit_nudge`:

```python
def submit_nudge(self, mcu_id, axis_idx, motor_mask, delta_mm, speed, accel):
    self.last_call = dict(kind="nudge", mcu_id=mcu_id, axis_idx=axis_idx,
                          motor_mask=motor_mask, delta_mm=delta_mm, speed=speed, accel=accel)
    return self._duration
```

And the test:

```python
def test_submit_nudge_builds_single_bit_mask_and_forwards():
    th = _make_correction_toolhead(0.6)
    dur = th.submit_nudge(7, 1, 2, 0.3, 80.0, 5000.0)  # motor_idx=2 -> mask 0b100
    call = th.bridge.last_call
    assert call["kind"] == "nudge"
    assert (call["mcu_id"], call["axis_idx"], call["motor_mask"]) == (7, 1, 0b100)
    assert call["delta_mm"] == pytest.approx(0.3)
    assert dur == pytest.approx(0.6)
    assert th.bridge.waits == 0 and th.bridge.dwells == []
```

- [ ] **Step 2: Run it — expect FAIL** (`Motion.submit_nudge` undefined).

Run: `python -m pytest test/test_toolhead_shim.py::test_submit_nudge_builds_single_bit_mask_and_forwards -v`
Expected: FAIL.

- [ ] **Step 3: Implement.** Replace `submit_correction` + `submit_motor_adjust` in `klippy/motion.py` with:

```python
    def submit_nudge(self, mcu_id, axis_idx, motor_idx, delta_mm, speed, accel):
        motor_mask = 1 << motor_idx
        return self.bridge.submit_nudge(
            mcu_id, axis_idx, motor_mask, delta_mm, speed, accel
        )
```

- [ ] **Step 4: Run the test — expect PASS.** Also remove/replace the old `test_submit_correction_is_a_plain_async_forward` / `test_submit_motor_adjust_is_a_plain_async_forward` (they reference deleted methods).

Run: `python -m pytest test/test_toolhead_shim.py -v`
Expected: PASS.

- [ ] **Step 5: Commit.**

```bash
git add klippy/motion.py test/test_toolhead_shim.py
git commit -m "feat(klippy): Motion.submit_nudge (single primitive, builds the motor mask)"
```

### Task C2: `force_move.manual_move` → resolve + fail-loud-if-disabled + `submit_nudge`

**Files:**
- Modify: `klippy/extras/force_move.py:41-49`.
- Test: `test/test_toolhead_shim.py` (add a focused test, or a new `test/test_force_move_manual_move.py`).

- [ ] **Step 1: Write the failing test.** In a new `test/test_force_move_manual_move.py`, build a minimal fake printer with a toolhead exposing `get_motor_binding`/`submit_nudge`, a `stepper_enable` whose `lookup_enable(name).is_motor_enabled()` returns `False`, and assert `manual_move` raises:

```python
def test_manual_move_raises_when_motor_disabled():
    fm, printer = make_force_move_with_disabled_motor("stepper_z1")
    with pytest.raises(Exception):
        fm.manual_move("stepper_z1", 0.5, 5.0, 100.0)

def test_manual_move_forwards_to_submit_nudge_when_enabled():
    fm, printer, toolhead = make_force_move_with_enabled_motor("stepper_z1")
    fm.manual_move("stepper_z1", 0.5, 5.0, 100.0)
    assert toolhead.last_nudge == dict(mcu_id=0, axis_idx=2, motor_idx=1, dist=0.5, speed=5.0, accel=100.0)
```

(Mirror existing klippy test fakes; `make_force_move_*` constructs the `ForceMove` with a stub printer.)

- [ ] **Step 2: Run — expect FAIL.**

Run: `python -m pytest test/test_force_move_manual_move.py -v`
Expected: FAIL.

- [ ] **Step 3: Implement.** Replace `force_move.py::manual_move`:

```python
    def manual_move(self, stepper, dist, speed, accel=0.0):
        toolhead = self.printer.lookup_object("toolhead")
        name = stepper if isinstance(stepper, str) else stepper.get_name()
        mcu_id, axis_idx, motor_idx = toolhead.get_motor_binding(name)
        if accel == 0.0:
            accel = toolhead.get_max_axis_accel(axis_idx)
        stepper_enable = self.printer.lookup_object("stepper_enable", None)
        if stepper_enable is not None:
            enable_line = stepper_enable.lookup_enable(name)
            if not enable_line.is_motor_enabled():
                raise self.printer.command_error(
                    "manual_move: motor '%s' is disabled; enable it first" % (name,)
                )
        return toolhead.submit_nudge(mcu_id, axis_idx, motor_idx, dist, speed, accel)
```

- [ ] **Step 4: Run — expect PASS.**

Run: `python -m pytest test/test_force_move_manual_move.py -v`
Expected: PASS.

- [ ] **Step 5: Commit.**

```bash
git add klippy/extras/force_move.py test/test_force_move_manual_move.py
git commit -m "feat(klippy): force_move.manual_move resolves binding, fails loud if disabled, calls submit_nudge"
```

### Task C3: Rewire `ZAdjustHelper` (z_tilt + z_tilt_ng) onto `force_move.manual_move`

**Why:** Both gantry-leveler helpers currently load `motor_adjust` and call `adjuster.adjust(...)`. Repoint them to `force_move.manual_move` so `motor_adjust` can be deleted.

**Files:**
- Modify: `klippy/extras/z_tilt.py:38-55` (`ZAdjustHelper.adjust_steppers`), `klippy/extras/z_tilt_ng.py:61-72` (its own `ZAdjustHelper.adjust_steppers`).
- Test: existing z_tilt tests if present (`rg -l z_tilt test/`); otherwise a focused fake-printer test asserting `adjust_steppers` calls `force_move.manual_move` per non-zero delta.

- [ ] **Step 1: Write/extend the failing test.** Assert that `adjust_steppers([a0, a1], speed)` calls `force_move.manual_move(name_i, delta_i, speed, accel)` once per stepper with `delta_i = a_i - min(adjustments)`, skipping deltas `< 1e-6`.

- [ ] **Step 2: Run — expect FAIL.**

- [ ] **Step 3: Implement.** In both `ZAdjustHelper.adjust_steppers`, replace:

```python
        adjuster = self.printer.load_object(self.config, "motor_adjust")
        ...
        for stepper, delta in zip(self.z_steppers, deltas):
            if delta < 1e-6:
                continue
            adjuster.adjust(stepper.get_name(), delta, speed, accel)
```

with:

```python
        force_move = self.printer.load_object(self.config, "force_move")
        toolhead = self.printer.lookup_object("toolhead")
        accel = toolhead.get_max_axis_accel(2)
        for stepper, delta in zip(self.z_steppers, deltas):
            if delta < 1e-6:
                continue
            force_move.manual_move(stepper, delta, speed, accel)
```

(Keep the existing `respond_info` summary and `min(adjustments)`/`deltas` computation. `force_move.manual_move` accepts a stepper object or name.)

- [ ] **Step 4: Run — expect PASS.**

Run: `python -m pytest test/ -k "z_tilt or gantry" -v` (and the new focused test)
Expected: PASS.

- [ ] **Step 5: Commit.**

```bash
git add klippy/extras/z_tilt.py klippy/extras/z_tilt_ng.py test/
git commit -m "refactor(klippy): z_tilt/z_tilt_ng ZAdjustHelper call force_move.manual_move (drop motor_adjust dep)"
```

### Task C4: motors_sync buzz → loop `submit_nudge`

**Why:** `StepperManualMove.manual_move(mcu_stepper, moves)` takes a list of relative segments and calls the deleted `submit_correction`. Loop the single-Δ primitive instead. motors_sync manages its own enable/disable via `steppers_enable`, so the motors are energized before the buzz.

**Files:**
- Modify: `klippy/extras/motors_sync.py:379-388` (`StepperManualMove.manual_move`).
- Test: if motors_sync has unit tests (`rg -l motors_sync test/`), extend; else a focused fake asserting one `submit_nudge` per non-trivial segment.

- [ ] **Step 1: Write/extend the failing test.** Assert `manual_move(mcu_stepper, [0.1, -0.1, 0.0])` calls `toolhead.submit_nudge` twice (the `0.0` filtered) with the resolved `(mcu_id, axis_idx, motor_idx)`, `travel_speed`, `travel_accel`, then `wait_moves()` once.

- [ ] **Step 2: Run — expect FAIL.**

- [ ] **Step 3: Implement.** Replace `StepperManualMove.manual_move`:

```python
    def manual_move(self, mcu_stepper, moves):
        segments = [m for m in moves if abs(m) >= 0.00001]
        if not segments:
            return
        name = mcu_stepper.get_name()
        mcu_id, axis_idx, motor_idx = self.toolhead.get_motor_binding(name)
        for dist in segments:
            self.toolhead.submit_nudge(
                mcu_id, axis_idx, motor_idx, dist,
                self.travel_speed, self.travel_accel)
        self.toolhead.wait_moves()
```

- [ ] **Step 4: Run — expect PASS.**

Run: `python -m pytest test/ -k motors_sync -v` (and the focused test)
Expected: PASS.

- [ ] **Step 5: Commit.**

```bash
git add klippy/extras/motors_sync.py test/
git commit -m "feat(klippy): motors_sync buzz loops submit_nudge per segment"
```

### Task C5: Delete `motor_adjust.py`

**Why:** The `MOTOR_ADJUST` command was a test-only shim; all real callers now use `force_move.manual_move`. Removing it also removes the `_ensure_motor_enabled` convenience (energizing is the caller's job).

**Files:**
- Delete: `klippy/extras/motor_adjust.py`.
- Verify no remaining references.

- [ ] **Step 1: Confirm no references remain.**

Run: `rg -n "motor_adjust|MOTOR_ADJUST" klippy/ test/`
Expected: only matches inside `motor_adjust.py` itself (z_tilt/z_tilt_ng were repointed in C3). If a config or test references it, update/remove it.

- [ ] **Step 2: Delete.**

```bash
git rm klippy/extras/motor_adjust.py
```

- [ ] **Step 3: Run the Python host suite.**

Run: `./scripts/ci.sh py`
Expected: PASS (no import of `motor_adjust`).

- [ ] **Step 4: Commit.**

```bash
git add -A
git commit -m "refactor(klippy): delete motor_adjust plugin (test-only; callers use manual_move)"
```

---

## Final verification (after all tasks)

- [ ] **Rust gate:** `cd rust && cargo nextest run && cargo test --doc && cargo clippy --workspace -- -D warnings && cargo fmt --all --check` — all green.
- [ ] **MCU build:** `./scripts/ci.sh rust-mcu-h7` — links.
- [ ] **Python gate:** `./scripts/ci.sh py` — green.
- [ ] **Quick gate (pre-PR):** `./scripts/ci.sh quick` — green.
- [ ] **Bench (manual, user-run):** flash both MCUs; `FORCE_MOVE` / a z_tilt adjust move the targeted motor repeatably; a motors_sync buzz packs back-to-back with `enable → buzz → disable` not overlapping audibly; no `seg0_deficit` / `-142` / `-309` in the structured logs.

## Spec coverage check

- Two-books model → A1 (frame/`p_prev` gate already at engine.rs:323) + B3 (advance `last_move_time`, not `ShaperState`).
- Mask = motor-select + interpretation → A1 + B1.
- Same tick loop, no ISR → A1 (no new timer; eval stays in `engine.tick`).
- Reset-per-piece anchoring → A1.
- `OVERLAY_UNSUPPORTED` contract → A2.
- Closed-form planner, no solver/jerk, limits unclamped → B2.
- `PlannerMsg::Nudge` sibling to HomeDrip → B3.
- `submit_nudge` primitive + delete shortcut → B4, B5.
- Host API `force_move.manual_move` (mainline-compatible) + fail-loud-if-disabled → C2.
- Levelers + motors_sync funnel through it; `motor_adjust` deleted → C3, C4, C5.
- `Motion` submit path collapses to one → C1.
