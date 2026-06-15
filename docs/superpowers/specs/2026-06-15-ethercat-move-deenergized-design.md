# EtherCAT deenergized movement — keep servos homed through M84

## Problem

After `M84` (motor-off), `_handle_motor_off` in `motion_kinematics._LinearKinematics`
calls `clear_homing_state((0, 1, 2))`, wiping the homed state for **every** axis, and the
servo's torque line drops via `BridgeTorqueLine.set_digital(..., 0)`.

For an EtherCAT servo this is both unnecessary and harmful:

- The drive has an absolute encoder. Once homed it retains its origin through a torque
  drop, so re-homing after a hand-push is pointless.
- Mainsail shows the axis as unhomed after `M84`, even though the drive still knows
  exactly where it is.
- On the next motion — including `G28` — the host plans a trajectory starting from its
  **stale** `commanded_pos`, so the first commanded target tells the drive to jump back
  to where it was before the hand-push. The drive yanks to that stale target at full
  force. This is acute in framed mode (post method-35): ec-rt commands absolute drive
  counts (`kalico-ethercat-rt.rs` ~line 704) with no soft-start `cmap` remap, so a stale
  host origin maps directly to a snap.

A pure stepper genuinely cannot retain a valid position after a hand-push (no absolute
feedback), so this behavior is **servo-backed axes only**; steppers keep clearing homing
on `M84` exactly as today.

## Goal

For servo-backed axes, `M84` deenergizes the drive but keeps the axis **homed** and marks
it **parked-dirty**. Before torque is restored on a parked-dirty servo (first move, homing
move, or explicit enable), the host blocking-reads the drive's actual position and
`set_position`s the host/MCU/drive origin to it. Issuing any G-code then "just works": the
trajectory starts from the true position, no yank, no re-home required.

## Mechanism (verified against current code)

- **`motion.set_position(newpos, homing_axes=())`** (`klippy/motion.py:225`) reseats the
  whole stack in one call: updates `commanded_pos`, calls `kin.set_position` →
  `bridge.set_position` (`rust/motion-bridge/src/bridge.rs:3433`) which flushes/drains,
  `kalico_stream_open`s at the new origin, and seeds the serial MCUs / EtherCAT drive
  frame. With `homing_axes=()` it does **not** touch `self.limits`, so the homed state
  survives. It fires `toolhead:set_position`, which `gcode_move` already handles by calling
  `reset_last_position()` — so the gcode coordinate layer resyncs automatically.
- **The blocking read exists and is torque-independent.**
  `bridge.query_motor_positions()` (`klippy/motion_bridge.py:473`) reads 6064h via the
  ec-rt `QueryMotorState` handler and returns assembled cartesian `{axis: (pos, vel)}`.
  It works while the drive is parked. For corexy the cartesian assembly already combines
  both lane motors, so the resync consumes cartesian values directly.
- **The snap is gated by torque state.** ec-rt only writes a target while
  `gate.state() == Enabled` (`kalico-ethercat-rt.rs` ~line 702); while parked it holds the
  last-written target PDO. Reseating the origin to actual *before* torque-on means the
  first streamed piece equals actual ⇒ no snap.
- **Torque-on chokepoints** are exactly two: (1) the move path via
  `_fire_active_callbacks` → `EnableTracking.motor_enable` → `set_torque(True)`; (2) explicit
  `SET_STEPPER_ENABLE ENABLE=1` / `motor_enable_group`. Homing moves run through
  `motion.move()` (`drip_move` → `move`), so a top-of-move resync covers the reported
  `G28`-yank case.

## Design

### 1. Dirty-state ownership — `motion_kinematics._LinearKinematics`

- Add `self._parked_dirty = [False, False, False]`.
- `_is_servo(axis)` ≡ `isinstance(self.rails[axis], servo_axis.ServoRail)`.
- `_handle_motor_off(print_time)`: per axis `i` —
  - if `_is_servo(i)` **and** axis `i` is currently homed (`limits[i][0] <= limits[i][1]`):
    set `_parked_dirty[i] = True` and **leave `limits[i]` intact**;
  - else: `clear_homing_state([i])` (current behavior).
- `set_position(newpos, homing_axes)`: clear `_parked_dirty[axis]` for every axis in
  `homing_axes` (a re-home or home-seed clears dirty).
- New accessors: `parked_dirty_axes()` → list of dirty axis indices;
  `clear_parked_dirty(axes)`.

### 2. Resync — `motion.Motion`

- `_resync_parked_servos()`:
  1. `dirty = self.kin.parked_dirty_axes()`; if empty, return immediately.
  2. `measured = self.bridge.query_motor_positions()`. If this raises, propagate the error
     (fail loud — never move with a stale origin).
  3. `newpos = list(self.commanded_pos)`; for each dirty axis `i`, replace
     `newpos[i]` with the measured cartesian value for `"xyz"[i]`.
  4. `self.set_position(newpos)` (empty `homing_axes` ⇒ homed preserved; fires
     `toolhead:set_position` ⇒ `gcode_move.reset_last_position()`).
  5. `self.kin.clear_parked_dirty(dirty)`.
- Call `_resync_parked_servos()` as the **first statement** of `move()` and `move_curve()`,
  before `Move(...)` is constructed from `commanded_pos`.

### 3. Explicit enable path — `stepper_enable.PrinterStepperEnable`

- Before re-energizing (the `enable=True` branch of `motor_debug_enable`, and in
  `motor_enable_group`), call `toolhead._resync_parked_servos()`. It is idempotent — a
  no-op when nothing is dirty — so a bare `SET_STEPPER_ENABLE ENABLE=1` / `M17` cannot snap.

### 4. Status

No change. `get_status().homed_axes` derives from `limits`, which is no longer cleared for
servo axes on `M84`, so Mainsail shows the axis homed through deenergize.

## Out of scope (YAGNI)

- Continuous polling of actual position while parked for live display. The user asked only
  for a read on the next motion; a poll loop would still need the chokepoint guard anyway.
- Any MCU/ec-rt firmware change. The fix is entirely host-side, reusing
  `query_motor_positions` and `set_position`.
- Stepper "keep homed through M84" — impossible without absolute feedback; explicitly
  unchanged.

## Test plan

Unit tests live in a separate file from the tested code.

- `_handle_motor_off`: servo axis homed → stays homed, becomes parked-dirty; servo axis
  unhomed → stays unhomed, not dirty; stepper axis → homing cleared, never dirty; mixed
  machine (stepper X/Y, servo Z) → `homed_axes` reflects per-axis outcome.
- `set_position` with `homing_axes` clears parked-dirty for those axes.
- `_resync_parked_servos`: no dirty axes → does not query; dirty Z → queries, calls
  `set_position` with measured Z and unchanged X/Y/E, clears dirty; query error →
  raises and does not call `set_position`.
- Move path: a `move()` issued while a servo axis is parked-dirty resyncs (origin updated
  to measured) before the `Move` deltas are computed.
- Explicit enable: `SET_STEPPER_ENABLE ENABLE=1` on a parked-dirty servo resyncs before
  torque-on; on a clean machine it is a no-op.
