# `min_home_dist` Safety Rehome — Design

**Date:** 2026-06-15
**Branch:** `min-home-dist` (base: `sota-motion`)
**Status:** Approved design (settled in discussion), pre-implementation

## Problem

`min_home_dist` is a per-axis safety floor on homing travel: the toolhead should
have to move at least this far before the endstop legitimately fires. A trigger
that arrives sooner is suspect (electrical noise, a sensorless false stall, the
head parked on the switch). Today the parameter is parsed (`rail.py:67`, default
`homing_retract_dist`) and carried in `HomingInfo`, but **never consulted** —
`_home_axis` does a single approach + retract and trusts whatever trip arrives.

We want the mainline safety behavior: detect a too-short first trigger, back off
by `min_home_dist`, re-approach over a bounded distance, and — if the second
trigger is *also* too short — fail homing loudly instead of trusting a bogus
endstop.

## Reference: how mainline does it (verified)

`klippy/extras/homing.py` `home_rails` on `main`:

- First home. If `moved_less_than_dist(min_home_dist, axes)` → `needs_rehome`,
  and the retract distance is promoted to `min_home_dist`.
- Retract, then re-approach. The second-home move is set up to command
  `2 × retract_dist` of travel (start = `homepos − 2·axes_d·retract_r`), so the
  endstop — sitting `retract_dist` away after the back-off — triggers at
  `≈ retract_dist` with an equal overshoot allowance.
- `check_no_movement()` → `"Endstop still triggered after retract"` (both endstop
  types).
- **Early-trigger failure is sensorless-only** on `main`:
  `if use_sensorless_homing and needs_rehome and moved_less_than_dist(...)` →
  `"Early homing trigger on second home!"`.
- `moved_less_than_dist` uses a dead-band: short iff
  `abs(dist) < min_dist and (min_dist − abs(dist)) >= tolerance`, with
  `tolerance = danger_options.homing_elapsed_distance_tolerance` (default 0.5mm).
- Distance is the **halt** distance (`halt_pos − start_pos` through kinematics).

### Deliberate divergences from mainline

1. **Universal, not sensorless-gated.** We set `min_home_dist` per *axis* and
   sensorless per *motor*, so gating the safety check on `use_sensorless_homing`
   is awkward and inconsistent. The full back-off → re-approach → fail-if-still-
   short loop applies to **every** axis regardless of endstop type. Simpler and
   uniform. `use_sensorless_homing` and `second_homing_speed` stay unused (the
   latter is reserved for a future precision double-approach — a *different*
   mechanism, not in scope).
2. **Measure to the trigger, not the halt.** We compare against the
   reconstructed trigger position (`trip_pos`), which is the semantically correct
   "how far before the switch fired" and is stricter than mainline's halt-based
   distance (halt includes brake overshoot). We already have `trip_pos` cleanly
   from the bridge.

## Design

All logic stays host-side in `klippy/extras/homing.py`. The Rust bridge and MCU
are untouched: `trip_move` already returns `trip_pos` (kinematic position at the
trigger clock, reconstructed from motion history) and `final_pos` (at halt). We
compute distance and orchestrate back-off / re-approach with the existing
`toolhead.move` + `trip_move` primitives. (Pushing enforcement into the Rust
`home_axis` state machine was rejected — the host already drives the retract
moves; it would add Rust complexity and harder testing for no benefit.)

### 1. Config & semantics

- `min_home_dist` consulted for every axis. Default stays `homing_retract_dist`
  (check active by default — mainline parity). `min_home_dist = 0` opts out
  entirely: no rehome, no failure.
- The only practical edge of the active default is re-running `G28` while parked
  near the endstop (first approach travels ≈ `retract_dist` ≈ `min_home_dist`);
  the 0.5mm tolerance normally absorbs it, and at worst one harmless rehome
  occurs. Accepted.

### 2. Distance measurement & "early" predicate

- Capture `start_pos = toolhead.get_position()` immediately before each approach.
- After the approach: `traveled = abs(trip_pos[axis] − start_pos[axis])`.
- `tolerance = get_danger_options().homing_elapsed_distance_tolerance` (0.5mm
  default; already present at `danger_options.py:38`).
- `early = (min_home_dist > 0) and (traveled < min_home_dist)
           and (min_home_dist − traveled >= tolerance)`.

### 3. Rehome control flow (`_home_axis`)

One-shot: at most one back-off + one re-approach, then pass or fail.

```
start_pos = toolhead.get_position()
trip_pos, final_pos = approach(speed, max_travel)          # guarded trip
traveled = abs(trip_pos[axis] - start_pos[axis])
needs_rehome = early(traveled)
structured_log.event("homing", "needs_rehome",
    msg="homing: %s needs rehome: %s (traveled=%.4f min_home_dist=%.4f)" % ...,
    ...)                                                    # msg contains "needs rehome: True/False"

if needs_rehome:
    # Work in the REAL motion frame, not the configured-endstop frame: the
    # suspect trip is NOT trusted to be the true endstop, so we must not
    # declare trigger_height yet. Inform the toolhead of its actual halt
    # position (like mainline's haltpos), then back off relative to the trip.
    haltpos = list(toolhead.get_position()); haltpos[axis] = final_pos[axis]
    toolhead.set_position(haltpos, homing_axes=[axis])
    backoff = list(toolhead.get_position())
    backoff[axis] = trip_pos[axis] - direction * min_home_dist
    toolhead.move(backoff, hi.retract_speed); wait_moves()
    start_pos = toolhead.get_position()
    trip_pos, final_pos = approach(speed, 2 * min_home_dist)  # guarded; bounded, mainline parity
    traveled = abs(trip_pos[axis] - start_pos[axis])
    if early(traveled):                                     # resolves homing.py:425 TODO
        raise gcmd.error("%s early homing trigger: endstop tripped after only "
                         "%.2fmm on re-approach (min_home_dist %.2fmm) — false "
                         "trigger or stuck/miswired endstop")

# --- shared tail (single path for both rehome and no-rehome) ---
overshoot = final_pos[axis] - trip_pos[axis]
newpos[axis] = _homed_axis_position(provider, axis, trip_pos, final_pos, trigger_height)
toolhead.set_position(newpos, homing_axes=[axis])
structured_log.event("homing", "axis_homed", ...)          # unchanged
if hi.retract_dist:                                        # FINAL retract (overshoot-corrected)
    retractpos[axis] -= direction * hi.retract_dist + overshoot
    toolhead.move(retractpos, hi.retract_speed); wait_moves()
if servo_handle is not None:
    bridge.finalize_homed_axis(servo_handle, axis, toolhead.get_position()[axis])
_check_servo_drive_fault(gcmd, bridge, axis, servo_handle)
```

Notes:
- **Re-approach bound = `2 × min_home_dist`** (mainline parity), at the **same
  homing speed** as the first approach. Clearly bounded — never full-axis travel.
- **Both** approaches run through the existing servo-guarded trip wrapper
  (`_run_servo_guarded_trip`: drive-limits context + at-contact fault check), so
  EtherCAT axes get torque-reduced trips and following-error detection on the
  re-approach too. Factor the single-approach body into a helper invoked once or
  twice.
- **Real-frame back-off (critical).** The suspect first trip is NOT declared as
  `trigger_height`. We `set_position(final_pos)` (actual reconstructed halt, the
  motion frame the trip was measured in) and back off relative to `trip_pos`
  (`backoff[axis] = trip_pos[axis] - direction*min_home_dist`). `trigger_height`
  is declared only in the shared tail, after a *validated* trip. Declaring the
  configured endstop coordinate off a bogus trigger would make the back-off move
  the wrong distance.
- **Fail loudly** (per CLAUDE.md): a still-short re-approach raises
  `gcmd.error`; no silent recovery. This is the *universal* guard — a
  stuck/held-closed endstop insta-trips on the re-approach (`traveled ≈ 0`), so
  it falls under the same too-short failure. We deliberately do **not** add a
  type-specific live-pin / latch query (latch stays set from the first trip
  until re-arm; live-pin state is meaningless for sensorless/virtual endstops),
  which would not be uniform across endstop types.

### 4. EtherCAT seed ordering — unchanged, made explicit

The position/limit seed (`finalize_homed_axis`, CiA-402 method-35) stays **after
the final retract**, exactly as today (`homing.py:327`) and as settled in
`2026-06-14-homing-seed-position-limits-design.md`: at the trigger the servo is
rammed into the hard stop (deflected/following-error), so the drive frame must be
declared from the clean, backed-off position.

The rehome path adds exactly one requirement: the seed fires **once, after the
final retract that follows the *last* successful approach** — never after the
intermediate back-off retract. Because the seed lives in the shared tail after
the rehome branch, this is automatic; the spec makes it explicit and a test
guards it.

## Scope & assumptions

`min_home_dist` is a safety check for a suspect trigger that is **near the true
endstop** — head parked on/against the switch at `G28`, or an early sensorless
stall on acceleration. The re-approach is bounded to `2 × min_home_dist`, which
reaches the real endstop only if it lies within ~`min_home_dist` of the suspect
trigger. A genuine *mid-travel* false trigger far from the endstop is **out of
scope**: the re-approach will exhaust its bound and fail loudly with the existing
"no trigger within travel" error (or a position-limit error on the back-off).
This matches mainline's implicit assumption and is the intended fail-loud
behavior, not a regression.

## Testing

Semantic correctness lives in **deterministic host unit tests** (exact control of
per-approach trigger distances); the ELF sim test is an integration smoke. This
split is deliberate: the time-based GPIO sim cannot place the re-approach trigger
by distance precisely enough to test the 0.5mm tolerance boundary without races.

- **Decision logic — unit (`test/test_homing_min_dist.py`, new):** pure
  `_trigger_too_early(traveled, min_home_dist, tolerance)`: early / not-early /
  tolerance-boundary (`min_home_dist−traveled` just below vs. at the tolerance) /
  `min_home_dist = 0` disabled.
- **Rehome orchestration — unit (same file):** drive `_run_homing_attempts` with
  an injected `approach` callable and a fake toolhead: (a) not-early → no rehome,
  approach called once, returns first trip; (b) early-then-legit → rehome, returns
  second trip, back-off move recorded relative to `trip_pos`; (c) early-then-early
  → raises; (d) `min_home_dist = 0` → never rehomes.
- **Seed ordering — unit (same file):** `_commit_and_seed` with a fake
  toolhead/bridge: `finalize_homed_axis` is called once and its position argument
  equals the **post-final-retract** toolhead position (asserting the value pins
  the ordering — a pre-retract call would carry the homed, not retracted,
  coordinate); `servo_handle = None` → not called.
- **Integration smoke — ELF sim (`tools/sim_klippy/tests/test_homing_lag_repro.py`):**
  reworked to be **deterministic** (no GPIO-timing races). The sim control API
  exposes only GPIO/PWM, not live head position, and the happy-path success lands
  exactly on the 0.5mm tolerance boundary — unhittable reliably via forced-GPIO
  timing. So the sim covers *path-runs* + *fail-mode*, not boundary-success:
  - `test_homing_retract_timing` gets its **own** override with `min_home_dist=0`
    (it tests retract timing only; sharing `min_home_dist=15` would make its
    held-high pin rehome-then-fail).
  - `test_homing_retract_and_rehome` → repurposed: GPIO tripped early and **held
    high** across the back-off → first trip is short (`"needs rehome: True"`
    logged, the `kalico.event` logger routes to klippy.log) → re-approach
    insta-trips → `G28` fails with "early homing trigger ... on re-approach". This
    deterministically exercises the rehome path end-to-end on real firmware and
    guards the original lag/deadlock bug (must complete fast, not hang).
  - Happy-path **success** (early transient that clears, re-approach completes) is
    covered by the deterministic unit tests above and by user-triggered bench
    verification — not the CI sim.

## Out of scope

- Precision double-approach via `second_homing_speed` / `use_sensorless_homing`
  (a separate, unimplemented mechanism).
- Any Rust/MCU changes.
- Multi-retry rehome (we do one-shot, mainline parity).

## Verification

- `cargo nextest run` is unaffected (no Rust change); `./scripts/ci.sh py` for
  the host tests; `./scripts/ci.sh quick` before PR.
- Bench (Neptune A6-EC + a switch-endstop axis) is user-triggered: confirm a
  forced early trip rehomes and succeeds, confirm a persistently-early trip
  fails, confirm EtherCAT live position after a rehome is still correct
  (seed unchanged-in-timing).
