# Uniform Homed Position + Drive Limit Seeding (overshoot-corrected retract) — Design

**Date:** 2026-06-14
**Branch:** `position-reporting` (base: `sota-motion`)
**Status:** Approved design (settled in discussion), pre-implementation

## Problem

Two coupled issues surfaced bench-testing the EtherCAT servo (Neptune, A6-EC X axis):

1. **EtherCAT live position is wrong.** The endpoint frames position with a *transient* `CountMap` that is created per-move and reset to `None` at idle, so at rest it reports nothing (host shows 0) and while moving it reflects a per-move-relative value, not the true absolute position. Root cause: the drive's `6064h` is in the *drive's raw encoder frame*, never anchored to Klipper's homed coordinate frame, because we bypass the drive's homing.

2. **Homing seeds position at the wrong time and ignores overshoot.** In `klippy/extras/homing.py` `_home_axis`, `toolhead.set_position` fires at the trigger (line ~305) *before* the retract (lines ~322-325), and the retract uses a fixed `retract_dist` — the overshoot is computed for logging (line ~319) but never folded into the retract. So the final position carries the overshoot error.

## Settled design

Make position/limit seeding **uniform across motor types**, established **once after the final retract**, with the retract corrected for overshoot. For EtherCAT, the **drive owns its frame** (no host-side per-cycle anchoring math).

### 1. Overshoot-corrected retract (all motor types) — `klippy/extras/homing.py`
The retract move folds in the measured overshoot (`final_pos − trip_pos`, already computed at line ~319) so the toolhead ends at the intended post-retract position regardless of overshoot. Uniform; no motor-type special-casing.

### 2. Move the position(+limits) seed to *after* the final retract (all motor types)
Currently the seed is at the trigger, before retract. Move it to after the final retract. Rationale:
- For EtherCAT, the servo at trigger time is rammed into the end of travel (pushing); we must back off before declaring the frame. After the retract it's at a clean, known position.
- Future-proofs `min_homing_distance` (out of scope now): its retries *measure* the trigger position via the existing endstop-trigger reconstruction (frame-independent), so a single final seed after validation is correct — no second method needed.

### 3. Uniform seed primitive: "current position = Z, limits = [min, max]"
One host call after the final retract, carrying per-axis (position, min, max). The **bridge dispatches per motor type**, so the host homing code stays uniform (no EtherCAT-specific host logic):

- **Stepper:** the existing position seed (`runtime_seed_position`) — unchanged. Limits are a **no-op** on the MCU; the host continues to enforce `axis_min/max` as today.
- **EtherCAT:** the endpoint, at the clean post-retract position, performs CiA-402 **homing method 35** ("current position is home"): set `6098h=35`, `607Ch = Z × counts_per_mm`, pulse the homing control-word, wait for "home attained", return to CSP. The drive re-zeros `6064h` into our frame. Then write software limits `607D.01 = min × counts_per_mm`, `607D.02 = max × counts_per_mm`. The drive now owns the frame and enforces the limits.

### 4. EtherCAT reporting becomes drive-framed
With the drive zeroed (method 35), `QueryMotorState` reports `mm = 6064h / counts_per_mm` — a fixed scale, persistent (no idle reset), correct at rest and moving, surviving power cycles (absolute encoder). **Remove the transient `CountMap`-based reporting** (the bug). Velocity is unchanged (`606Ch` → `velocity_mm_s`).

## Out of scope
- `min_homing_distance` (design supports it; not implemented here).
- Hardware limit switches `P-OT`/`N-OT` (needs wiring; software `607D` only).
- Stepper-side MCU limit enforcement (host keeps enforcing).
- Drive sensorless/hard-stop homing (later; this keeps Klipper endstop homing).

## Verification
- Unit/CI: protocol codec, bridge per-type dispatch, endpoint method-35 + `607D` + reporting math.
- Bench (A6-EC, Neptune): home X; confirm live position reads **absolute at rest and moving**; confirm overshoot-corrected retract lands the toolhead at the intended position; confirm `607D` limits are enforced by the drive. (Requires a bench flash — user-triggered.)

## Notes
- This supersedes Plan 2's EtherCAT *reporting* mechanism (the transient `CountMap` path) but keeps Plan 2's `QueryMotorState`/`MotorStateResponse` wire + `606Ch` velocity.
- The host homing change is uniform and affects steppers too (seed moves after retract, retract corrects overshoot) — must be verified not to regress stepper homing.
