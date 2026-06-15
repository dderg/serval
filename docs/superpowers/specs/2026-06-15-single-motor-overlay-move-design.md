# Single-motor overlay moves (`manual_move`) — design

**Supersedes** `2026-06-15-unify-corrections-into-motion-pipeline-design.md` and
`2026-06-15-correction-stream-on-move-timeline-design.md`. Those framed the work
as "corrections" and assumed a solver-shaped, rest-to-rest profile. This design
generalizes to a first-class primitive — *move one (or a few) motors of an axis
by a relative amount, off the books* — and replaces the solver with a closed-form
trapezoid. The MCU-side mask machinery from the unify spec (already merged) is
kept; the host-side bespoke correction path and the second solver are deleted.

## Goal

One primitive: **issue a relative move to a single motor of an axis, through the
main motion pipeline, without affecting the axis position-of-record.** Buzz
(motors_sync), gantry-level adjust (z_tilt / quad_gantry_level / z_tilt_ng), and
single-stepper `FORCE_MOVE` all become thin callers of this one primitive. The
per-piece mask is a `u8` so it reserves room for a few motors per axis, but only
the single-motor path is honored now; multi-motor masks are a loud reject (YAGNI).

## The two books

"Off the books" splits into two independent ledgers:

- **Position book** — `axis.p_prev`, the axis position-of-record (the trajectory
  accumulator read by homing, `get_position`, `SET_KINEMATIC_POSITION`,
  `query_motor_state`). An overlay move **never** touches it. The per-motor
  offset lives implicitly in the divergence of that motor's `position_count`
  from the axis frame, which homing already ignores.
- **Time book** — `last_move_time` / the pump queue timeline. An overlay move
  **does** occupy a slot: the motor physically moves for some duration, so
  subsequent work, the enable/disable pins, and `wait_moves()` all schedule
  after it. This is why overlay moves "respect the pump queue, and the pump
  queue respects them" — and it is the real fix for the original
  enable-overlaps-buzz click (the buzz was previously off the time book, so the
  de-energize landed mid-shake).

An overlay move is, in one line, *a dwell that also moves one motor*: time
advances, position-of-record does not.

## The motor mask

Every `PieceEntry` carries `motor_mask: u8` (one bit per motor, `MAX_MOTORS_PER_AXIS = 8`).
The mask is both *which motors* and *how the MCU interprets the curve*:

- `mask == 0` → normal full-axis move: all motors step (`stepper_sel = ALL`), the
  curve is the **absolute** axis trajectory diffed against `last_step_count`,
  `position_count` advances for every motor, and `p_prev`/`v_prev` advance.
- `mask != 0` (single bit) → overlay: only that motor steps (`stepper_sel = idx+1`),
  the curve is interpreted **relative** and anchored on that motor's overlay
  accumulator, only that motor's `position_count` advances, and `p_prev` is **not**
  touched.
- `mask` with > 1 bit set → loud reject (`-143 MULTI_MOTOR_MASK`). Single-bit is
  the only live requirement; multi-bit is deferred (YAGNI).

The `PieceEntry` is unchanged in size — the mask is the byte already reserved in
the unify work (`piece_ring.rs`, byte 28). Wire framing and `to_le_bytes`/
`from_le_bytes` are unchanged.

## MCU evaluation — same tick loop, one branch, no ISR

There is **one** sample timer at `sample_rate_hz` driving **one** `engine.tick`
(`engine.rs:274`), a single `for i in 0..num_axes` loop. Overlay pieces ride the
**same per-axis ring**, time-ordered against normal pieces, evaluated against the
**same** `now`. There is no second ISR and there must not be one: a second
timebase is exactly the off-timeline bug this design eliminates, and two ISRs
would race on the same stepper's step queue / `position_count` / GPIO.

The mask is a small set of localized selections, already in the merged tree:

- **Position-book gate** (`engine.rs:323`): `if axis.ring.peek().motor_mask == 0
  { axis.p_prev = p_end; axis.v_prev = v_end; }`.
- **Step-frame select + step output** (`dispatch_stepper.rs`): `stepper_sel` from
  the mask; baseline is the motor's `overlay_step_frame` (mask ≠ 0) vs the axis
  `last_step_count` (mask 0); `commit_position_count_masked` scopes the count.

**Relative-curve anchoring (drift-free).** The host emits a relative curve
`C(t): 0 → Δ` and tracks nothing. The MCU owns the only running state — the
per-motor `overlay_step_frame` accumulator carrying the running offset **including
the sub-step fractional part**. On each overlay sample: `absolute = accum + C(t)`,
steps = `round(absolute) − round(prev_sample)`. Because the fractional part is
carried, back-to-back overlay moves and long asymmetric chains do not accumulate
rounding drift, and a host/MCU restart cannot desync (the host never held a copy).
A symmetric buzz (`+Δ, −Δ`) nets to zero regardless.

**Rejection contract.** A piece with `mask != 0` is only honorable by an MCU that
can address an individual motor. Any MCU that cannot (EtherCAT/servo) **rejects at
decode** with a new error code `OVERLAY_UNSUPPORTED` — fail-loud, no silent drop.
Bare-metal H7/F446 honor it; the EtherCAT side implements the rejection whenever
that firmware lands. This work defines the code + contract and the bare-metal
honoring path. The host never checks for servos — gating is the MCU's job, the
same pattern as `-310` ring-capture gating.

## Host — closed-form planner, no solver

A nudge is a 1-D boundary-value problem; the temporal solver (beta-loop, SLP jerk
relaxation, multi-axis temporal optimization) is overkill and is **not** invoked.
The CLAUDE.md throughput non-negotiable governs **print** trajectories on slicer
output; calibration/buzz motion has no print throughput at stake, so a closed-form
planner here violates nothing.

`plan_nudge_profile(delta_mm, speed, accel) -> Vec<cubic piece>`:

- `accel == 0` → **constant velocity** (infinite acceleration / box profile),
  matching mainline `manual_move`. One linear cubic piece.
- `accel > 0` → **trapezoid** via mainline's `calc_move_time` logic: `accel_t`,
  `cruise_t`, `accel_t`. Each phase is a low-order polynomial exact as a cubic
  Bézier (box = linear, ramp = parabola = cubic with zero cubic term). 1–3 pieces.

Caller `speed`/`accel` are **authoritative and may exceed the configured axis
limits** — there is no host-side clamp. The MCU `StepsPerSampleExceeded` fault is
the hard backstop for a physically impossible request (fail-loud at the hardware
boundary). Allowing `accel == 0` consciously drops any rest-to-rest invariant;
that is fine for the same reason.

The closed-form curve is wrapped as a `ShapedSegment` (target axis = the relative
`0 → Δ` curve, other axes constant) and pushed through the **unchanged**
`enqueue_segment` → pump → wire path (`bridge.rs:3084`). "Skip the solver" means
skip the *optimizer*, not the *plumbing*: routing, anchoring, the cubic
representation, `DrainSync`, and the time-book are all the existing move path.

### Planner entry point

`PlannerMsg::Nudge { axis, motor_mask, delta, speed, accel, notify }` — a sibling
arm to `HomeDrip` in the planner thread (`planner.rs` `run_loop`). It calls
`plan_nudge_profile`, stamps `motor_mask` on the emitted segments, anchors at the
current end-of-timeline, dispatches through the same closure, advances
`last_move_time` by the duration, and **does not** touch the main `ShaperState` /
`p_prev`. Because it is a planner message on the one channel, it serializes
in-order with `Move`s — it cannot race the main chain, and consecutive nudges pack
back-to-back (the time-book mechanism), which is what makes a buzz a tight
oscillation.

## Host API and callers

```
z_tilt / quad_gantry_level / z_tilt_ng
   → ZAdjustHelper.adjust_steppers (per-stepper Δ, loop)
        → force_move.manual_move(stepper, dist, speed, accel)
MOTOR_ADJUST gcode ........................... DELETED (test-only)
motors_sync buzz → enable once → loop force_move.manual_move(±Δ) → measure → disable
FORCE_MOVE gcode → force_move.manual_move
                              │
        submit_nudge(mcu_id, axis_idx, motor_mask, delta, speed, accel)   [bridge primitive]
                              │
        PlannerMsg::Nudge → plan_nudge_profile → enqueue_segment → pump → wire → MCU
```

- **`submit_nudge(mcu_id, axis_idx, motor_mask, delta_mm, speed, accel) -> f64`**
  (bridge): the single engine primitive, returns duration. Replaces both
  `submit_correction_sequence` and `adjust_motor`/`submit_motor_adjust`. Validates
  single-bit mask (multi-bit → loud error).
- **`force_move.manual_move(stepper, dist, speed, accel)`** (host): the one
  name-resolving helper. Mainline name, location, and first-four-args signature —
  mainline-derived macros/plugins calling it keep working unchanged. Resolves
  `name → (mcu_id, axis_idx, motor_idx)` via `get_motor_binding`, builds
  `mask = 1 << motor_idx`, **fails loud if the motor is disabled** (turns the old
  silent no-motion symptom into a clear error), and calls `submit_nudge`.
- **`Motion.submit_motor_adjust` / `submit_correction`** collapse to thin forwards
  to `submit_nudge` (or are removed in favor of it).
- **`motor_adjust.py` plugin is deleted** — the `MOTOR_ADJUST` command was a
  test-only shim, and `_ensure_motor_enabled` was a testing convenience, not a
  requirement. The gantry-levelers already funnel through `ZAdjustHelper`; that
  one helper now calls `force_move.manual_move` instead of `motor_adjust.adjust`.

## Mainline compatibility and divergences

- **Compatible:** `force_move.manual_move(stepper, dist, speed, accel)` — same
  name, location, signature. `accel == 0` → constant velocity, matching mainline.
- **Divergence (intentional):** the move is off the position book — it does not
  update kinematic position (mainline `FORCE_MOVE` "invalidates kinematics"; ours
  simply never touches `p_prev`). The profile is a clean trapezoid/box emitted as
  cubic Bézier pieces through the rewrite's pipeline, not the mainline trapq.
- **Out of scope:** `manual_stepper` (an absolute-position move on a
  self-positioned stepper — a different abstraction).

## What gets deleted

- `pump_correction_overlay`, the hand-built `AxisKey` + direct `pump_tx.send`
  (`bridge.rs`) — the routing bug.
- `to_overlay_piece_entries` (`correction.rs`) — direct `PieceEntry` building.
- `plan_correction_profile` / `plan_correction_sequence` and `ProfilePiece`
  (`correction.rs`) — the second solver.
- `motor_adjust.py` — the plugin, the `MOTOR_ADJUST` command, `_ensure_motor_enabled`.
- Any private correction lead/anchor constants — nudges anchor at end-of-timeline
  like any move.

## Testing

- **Runtime unit:** a single-bit-mask piece anchors on `overlay_step_frame`,
  advances only that motor's `position_count`, leaves `p_prev` untouched; relative
  back-to-back overlay pieces accumulate drift-free (fractional carry); a symmetric
  `+Δ, −Δ` pair nets to zero; `mask == 0` behavior unchanged; multi-bit → `-143`.
- **Planner unit:** `plan_nudge_profile` — `accel == 0` yields one constant-velocity
  piece; `accel > 0` yields a trapezoid whose total displacement = Δ and whose
  cruise = `speed`; short Δ degenerates to a triangle (no cruise); `speed`/`accel`
  above the configured axis limits are passed through unclamped.
- **Bridge unit:** `submit_nudge` sends a `PlannerMsg::Nudge`, returns the
  duration, advances `last_move_time` but not `p_prev`; multi-bit mask → loud error.
- **Python unit:** `force_move.manual_move` resolves the binding, builds the
  single-bit mask, raises on a disabled motor, and forwards to `submit_nudge`; no
  dwell/`wait_moves` timeline machinery.
- **Bench:** a buzz packs back-to-back; `enable → buzz → disable` no longer
  overlaps audibly; no `seg0_deficit` / `-142` / `-309`; `FORCE_MOVE` and a z_tilt
  adjustment move the targeted motor repeatably; the EtherCAT MCU (when present)
  rejects an overlay piece with `OVERLAY_UNSUPPORTED`.

## Non-goals

- Jerk-limiting / S-curve profiles (dropped — velocity + accel only).
- Multi-bit motor masks; additive superposition of an overlay onto a moving axis.
- Honoring overlay pieces on EtherCAT/servo MCUs (contract defined; rejection only).
- Reworking kinematics, the G5/G5.1 reduce stage, the pump, or `DrainSync`.
- `manual_stepper` compatibility.
