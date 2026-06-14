# motors-sync Plugin Port to the Kalico Fork

Date: 2026-06-14
Status: design, pending review
Builds on: `2026-06-14-correction-stream-sequences-design.md` (the
`submit_correction_sequence` host primitive) and
`2026-06-12-per-motor-correction-moves-design.md` (the per-motor correction
mechanism and `get_motor_binding`).

## Why

motors-sync (a hard fork at `dderg/motors-sync`) synchronizes the two stepper
motors on a shared belt: it buzzes one motor to excite the belt, measures the
accelerometer response, and nudges one motor by single microsteps to remove a
detected desync. On our fork it does not even load — at class-definition time
`StepperManualMove` does `from . import force_move; calc_move_time =
force_move.calc_move_time`, and our fork deleted module-level
`force_move.calc_move_time`, so the import raises `AttributeError` before any
move runs. That is the live bench failure.

Underneath the import error is a deeper break: every single-motor move in the
plugin flows through `StepperManualMove.manual_move`, which open-codes the host
stepping path — `cartesian_stepper_alloc` → `set_stepper_kinematics` →
`set_trapq` → `trapq_append` → `generate_steps`. The motion rewrite deleted
that entire host-stepping path. The plugin must drive motors through the new
motion engine instead.

The new engine already exposes exactly what the plugin needs:
`submit_correction_sequence(mcu_id, axis_idx, motor_idx, segments, speed,
accel)` plays a gapless list of relative motor-space moves on one motor of an
axis without touching commanded XYZ — the buzz and the nudge are both just
`segments` lists.

## Non-goals / decisions

- **`force_move.manual_move` is one move per call.** Signature stays
  `manual_move(self, stepper, dist, speed, accel=0.0)` — scalar, matching
  upstream Klipper. No list overload, no type-sniffing. A buzz is a *list*, so
  it does not pretend to be a `manual_move`; it rides the list-native
  `submit_correction_sequence` directly.
- **No resurrection of the host-stepping path.** We do not rebuild
  `set_trapq`/`generate_steps`/host trapq as a bridge emulation to make the
  plugin run untouched. That path is single-motor, idle-axis only on the new
  engine; a faithful general emulation is large, leaky, and re-introduces what
  the rewrite removed. The plugin is rewritten onto the new primitive instead.
- **No new bridge primitive.** Both shapes (single nudge, buzz) use the
  already-built `submit_correction_sequence`. The seam and the plugin are
  pure host-Python consumers of it.
- **No planner for these moves.** Speed/accel stay caller-supplied (the
  plugin's existing `travel_speed`/`travel_accel`), trusted by the correction
  path as before.
- **Out of scope (flagged, not done):** un-gating the `FORCE_MOVE` and
  `STEPPER_BUZZ` *G-codes* (still `PHASE5_GATE`). They become trivial once
  `manual_move` works, but `FORCE_MOVE`'s "invalidates kinematics" semantics
  differ from a correction overlay and deserve their own pass. The sensor /
  accelerometer paths (`accel_chip`, beacon, `angle.py`, TMC phase query) are
  already compatible and unchanged.

## Design

Two coordinated pieces, in two repos.

### A. Fork seam — `klippy/extras/force_move.py`

Implement the stubbed `manual_move` on the bridge. It is the general
single-stepper seam: any plugin (and, later, the `FORCE_MOVE`/`STEPPER_BUZZ`
G-codes) that wants one motor nudged calls it and never touches the bridge or
the binding tuple.

```python
def manual_move(self, stepper, dist, speed, accel=0.0):
    toolhead = self.printer.lookup_object("toolhead")
    name = stepper.get_name() if hasattr(stepper, "get_name") else stepper
    mcu_id, axis_idx, motor_idx = toolhead.get_motor_binding(name)
    bridge = toolhead.get_bridge()
    return bridge.submit_correction_sequence(
        mcu_id, axis_idx, motor_idx, [dist], speed, accel
    )
```

`accel=0.0` (upstream's "unset" default) means "use a sensible accel"; the
implementation substitutes the axis machine-max accel via
`toolhead.get_max_axis_accel(axis_idx)` when `accel` is non-positive, so the
upstream-style call shape keeps working. The method returns the move duration
in seconds, matching `submit_correction_sequence` and the `MotorAdjust`
reference; the wait/settle is the caller's responsibility (see
`klippy/extras/motor_adjust.py` for the established reactor-wait pattern).

This seam only works for axis-bound motors (those with a `get_motor_binding`
entry). An unbound stepper raises the existing `config_error` from
`get_motor_binding` — a clear, loud failure, consistent with the rest of the
correction path.

### B. Plugin adapter — `motors_sync.py`

`StepperManualMove` collapses to a thin adapter over the bridge.

Deleted as dead weight (existed only to feed the old host-stepping path):

- `from . import force_move; calc_move_time = ...` (the import-time crash).
- `DummyPrinterMotionQueuing` (the whole "Klipper before motion_queuing"
  compat class) and the `motion_queuing` lookup.
- The `chelper` trapq machinery in `StepperManualMove.__init__`:
  `cartesian_stepper_alloc`, `allocate_trapq`, `lookup_trapq_append`.

`steppers_enable` stays as-is (it already uses
`stepper_enable.lookup_enable(name).motor_enable(print_time)`, which our fork
keeps).

`manual_move(self, mcu_stepper, moves)` becomes:

```python
def manual_move(self, mcu_stepper, moves):
    segments = [m for m in moves if abs(m) >= 0.00001]
    if not segments:
        return
    name = mcu_stepper.get_name()
    mcu_id, axis_idx, motor_idx = self.toolhead.get_motor_binding(name)
    bridge = self.toolhead.get_bridge()
    reactor = self.printer.get_reactor()
    start = reactor.monotonic()
    duration = bridge.submit_correction_sequence(
        mcu_id, axis_idx, motor_idx, segments,
        self.travel_speed, self.travel_accel)
    deadline = start + duration + SETTLE_PAD
    while reactor.monotonic() < deadline:
        reactor.pause(reactor.monotonic() + 0.01)
```

This one method already feeds every mover in the plugin: `step_move` passes
`[dist]`, the phase-restore nudges pass `[dist]`, and `buzz_move` passes the
full ~50-element fading-oscillation list. All flow through one
`submit_correction_sequence` call, so the buzz is gapless with no branching.

**Timeline note (load-bearing).** Corrections schedule on the host/wall-clock
timeline, not the print-time queue. The adapter therefore blocks on the
*reactor* (wall clock) until the move completes — not `toolhead.dwell` (print
time) — before returning, so the subsequent measurement is safe. The deadline
is anchored at `start` captured *before* the call, so any in-call refill
pacing (for buzzes longer than the correction ring) is absorbed and the total
wait equals the move duration, never double it. `SETTLE_PAD` mirrors
`MotorAdjust`'s `ADJUST_SETTLE_PAD` (0.05 s).

Motor enable/disable around moves still uses `steppers_enable` on print time;
the motor is held enabled across each correction (the plugin only toggles
enables at coarse sync boundaries), so the two timelines do not race.

### What the consumer sees

motors-sync drives motors through exactly one new call,
`submit_correction_sequence`, plus the unchanged enable/travel/sensor APIs. It
never sees a trapq, a stepper-kinematics handle, or `generate_steps`. Generic
plugins get the upstream-shaped `force_move.manual_move` seam for free.

## Validation / fail-loud

- `get_motor_binding` on an unknown/unbound stepper raises `config_error`
  (existing behavior) — loud, not silent.
- `submit_correction_sequence` already fails loud on a non-idle axis, an
  unbound motor, a stale start time, ring overflow, or host-refill-behind. The
  adapter adds no recovery; it surfaces those errors.
- An all-sub-epsilon `moves` list is a no-op return in the adapter (matches the
  old `abs(move) < 0.00001` skip), so a zero nudge does not reach the bridge,
  which would reject an empty sequence.

## Testing

- **Fork seam:** `klippy/extras/force_move.py` — exercised via the existing
  `MOTOR_ADJUST`/bridge sim path is not enough; add a host test (or kalico-sim
  scenario) that calls `force_move.manual_move(stepper, dist, speed)` on a
  bound motor of an idle multi-motor axis and asserts only the target stepper
  toggles, commanded position is unchanged, and the returned duration is
  positive. An unbound stepper raises `config_error`.
- **Plugin adapter:** the plugin needs a full klippy host to load, so coverage
  is integration-level. In kalico-sim, configure a 2-motor axis, run a single
  `step_move`-equivalent nudge and a `buzz_move`-equivalent sequence; assert
  the target stepper's correction stream fires (`motion.correction_start` /
  `motion.correction_drained`), no `motion.ring_full`, the partner motor stays
  put, and the reported axis position is unchanged.
- **Bench (the sign/units risk):** on the Trident, confirm the plugin loads,
  a buzz runs as one continuous shake on one belt motor with no inter-swing
  pause, the partner motor stays put, and — the real unknown — that the
  plugin's `move_d`-derived distances move the motor in the direction the
  plugin's measurement model expects. The sign convention between the plugin's
  microstep distances and what the correction primitive applies to a motor can
  only be confirmed on hardware; a sync that diverges instead of converging is
  the symptom of an inverted convention.

## Risks

- **Sign / units convention (primary).** Bench-settled, above. If inverted,
  the fix is a single sign flip at the adapter boundary, not a redesign.
- **Wall-clock vs print-time interleave.** Covered by the reactor-wait
  contract above; the bench buzz test confirms no gap and no over-wait.
